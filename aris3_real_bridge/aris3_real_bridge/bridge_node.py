"""aris3_real_bridge - 실물 E0509 를 ROS 인터페이스로 내놓는 노드 (2026-11-10).

GUI(aris3_gui PyQt, aris3_gui_cpp ARIS)는 로봇에 직접 붙지 않고 이 노드만 부른다.
로봇 쪽 코드는 새로 짜지 않고 그대로 쓴다:
  e0509_manual.robot_link.RobotLink   이동 전 안전 검사·도착 확인·정지·그리퍼 DRL·복구 (실물 검증됨)
  e0509_manual.bringup                dsr_bringup2 접속·해제
  aris3_real_bridge.real_pick         카메라 보정(PixelCalibration)·집기 순서(PickSequence)

  토픽      /aris3/real/state            RealRobotState 10 Hz
  서비스    /aris3/real/connect|disconnect|stop   std_srvs/Trigger   (stop 은 별도 콜백 그룹: 항상 응답)
            /aris3/real/set_armed|set_hand_guide  std_srvs/SetBool
            /aris3/real/calibration               RealCalibration
  액션      /aris3/real/motion           RealMotion     (취소 = 정지)
            /aris3/real/pick_place       RealPickPlace  (취소 = 정지)
  구독      /aris3/vision/detections_px  색 검출 픽셀 (집기 직전에 큐브 위치를 다시 읽음, 보정 ① 에 사용)

조작 허용(armed): GUI 의 「조작 허용」 과 별개로 이 노드도 막는다. armed 가 아니면 이동·그리퍼·집기 요청을
거부하고, arm_timeout_s 동안 명령이 없으면 스스로 잠근다. 정지는 armed 와 상관없이 언제나 동작한다.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import SetBool, Trigger

from aris3_interfaces.action import RealMotion, RealPickPlace
from aris3_interfaces.msg import DetectedObjectArray, RealRobotState
from aris3_interfaces.srv import RealCalibration
from aris3_real_bridge.real_pick import PickSequence, PixelCalibration

MAX_JOINT_VEL = 45.0          # deg/s - 속도 비율 100 % (RobotLink 도 같은 값으로 자른다)
MAX_CART_VEL = 100.0          # mm/s
GRIP_CLOSE_POS, GRIP_CURRENT_MA, GRIP_VEL = 740, 300, 1500
REFRESH_PX = 40.0             # 집기 직전: 받은 픽셀에서 이 거리 안의 검출로 큐브 위치를 고친다
PX_FRESH_S = 1.0              # 이보다 오래된 검출은 쓰지 않는다 (보정 ① 은 2 s)
STANDBY, MOVING = 1, 2
# launch 에서 z_min_mm:=77 처럼 정수를 줘도 받도록 (기본은 타입 고정이라 노드가 죽었다), 읽을 때 float()/int()
ANY_TYPE = ParameterDescriptor(dynamic_typing=True)


def nearest_px(objects, u, v, max_dist=None):
    """검출 목록에서 (u, v) 에 가장 가까운 것. max_dist 를 넘으면 None."""
    best = min(objects, key=lambda o: math.hypot(o.pose.pose.position.x - u, o.pose.pose.position.y - v),
               default=None)
    if best is None:
        return None
    if max_dist is not None and math.hypot(best.pose.pose.position.x - u, best.pose.pose.position.y - v) > max_dist:
        return None
    return best


def joint_vel(pct):
    return max(5.0, MAX_JOINT_VEL * float(pct) / 100.0)


def cart_vel(pct):
    return max(10.0, MAX_CART_VEL * float(pct) / 100.0)


def hold_code(holding, cls=RealRobotState):
    return cls.HOLD_UNKNOWN if holding is None else (cls.HOLD_YES if holding else cls.HOLD_NO)


class RealBridge(Node):
    def __init__(self, link, mgr, conn, state_ko=None):
        super().__init__('aris3_real_bridge')
        self.declare_parameter('arm_timeout_s', 120.0, ANY_TYPE)
        self.declare_parameter('state_rate_hz', 10.0, ANY_TYPE)
        self.link, self.mgr, self.conn = link, mgr, conn
        self.state_ko = state_ko or {}
        self.arm_timeout = float(self.get_parameter('arm_timeout_s').value)

        self.calib = PixelCalibration()
        self.picker = PickSequence(link, lambda s: self.get_logger().info(f'[집기] {s}'))
        self.calib_px = None                # 보정 ①: (u, v)
        try:
            self.calib.load()
            self.get_logger().info(f'카메라 보정 불러옴: {self.calib.summary()}')
        except (OSError, ValueError, KeyError):
            self.get_logger().info('카메라 보정 파일 없음 (보정 ① ② 로 점 4개 이상 기록)')

        self._lock = threading.Lock()
        self.armed = False
        self.last_cmd = 0.0
        self.active_goal = False            # 이동 액션 하나만 (motion, pick_place 공통)
        self.px_objects, self.px_stamp = [], 0.0

        cb = ReentrantCallbackGroup()
        cb_stop = MutuallyExclusiveCallbackGroup()   # 다른 콜백이 모두 바빠도 정지는 따로 처리
        p = '/aris3/real/'
        self.pub_state = self.create_publisher(RealRobotState, p + 'state', 10)
        self.create_subscription(DetectedObjectArray, '/aris3/vision/detections_px', self._on_px, 10,
                                 callback_group=cb)
        self.create_service(Trigger, p + 'stop', self._srv_stop, callback_group=cb_stop)
        self.create_service(Trigger, p + 'connect', self._srv_connect, callback_group=cb)
        self.create_service(Trigger, p + 'disconnect', self._srv_disconnect, callback_group=cb)
        self.create_service(SetBool, p + 'set_armed', self._srv_set_armed, callback_group=cb)
        self.create_service(SetBool, p + 'set_hand_guide', self._srv_set_hand_guide, callback_group=cb)
        self.create_service(RealCalibration, p + 'calibration', self._srv_calibration, callback_group=cb)
        self.motion_server = ActionServer(
            self, RealMotion, p + 'motion', self._exec_guard(self._exec_motion, RealMotion.Result), goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb, callback_group=cb)
        self.pick_server = ActionServer(
            self, RealPickPlace, p + 'pick_place', self._exec_guard(self._exec_pick, RealPickPlace.Result), goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb, callback_group=cb)
        rate = float(self.get_parameter('state_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick, callback_group=cb)
        self.get_logger().info('실물 브리지 시작 (조작 허용 꺼짐)')

    # ================================================================== 공통 검사
    def _touch(self):
        self.last_cmd = time.time()

    def why_blocked(self, need_ready=True):
        """이동 명령을 지금 받을 수 없는 이유, 없으면 None."""
        if not self.armed:
            return '「조작 허용」 이 꺼져 있음'
        if self.conn.busy():
            return self.mgr.describe()
        if self.picker.running:
            return '집기/놓기 순서가 진행 중'
        if need_ready:
            return self.link.why_not_ready()
        return None

    def _last_link_log(self, default):
        """RobotLink 가 방금 남긴 거부 이유 (시각 제거). RobotLink 는 거부할 때 로그만 남기고 False 를 돌려준다."""
        lines = list(self.link.log_lines)
        return lines[-1][9:] if lines else default

    def _on_px(self, msg):
        self.px_objects, self.px_stamp = list(msg.objects), time.time()

    def _fresh_px(self, max_age):
        return self.px_objects if time.time() - self.px_stamp <= max_age else []

    # ================================================================== 서비스
    def _srv_stop(self, _req, res):
        sent = self.link.cli_stop.service_is_ready()
        self.link.stop()                    # 연결이 없어도 부른다 (RobotLink 가 로그를 남김)
        self.get_logger().warn('정지 요청 받음' + ('' if sent else ' - 로봇 연결 없음, 보내지 못함'))
        res.success = bool(sent)
        res.message = '정지 명령 보냄' if sent else '정지 명령을 보낼 수 없음: 로봇에 연결되어 있지 않음'
        return res

    def _srv_connect(self, _req, res):
        if self.link.joints_fresh():
            res.success, res.message = True, '이미 접속됨'
        else:
            res.success = bool(self.conn.connect_async())
            res.message = '접속 시작 (서보가 켜지고 펜던트 제어권을 가져옴, 팔은 움직이지 않음)' if res.success \
                else '이미 접속/끊기 작업 중'
        return res

    def _srv_disconnect(self, _req, res):
        self.armed = False
        res.success = bool(self.conn.disconnect_async())
        res.message = '접속 끊기 시작 (서보 꺼짐, 브레이크가 팔을 잡음)' if res.success else '이미 접속/끊기 작업 중'
        return res

    def _srv_set_armed(self, req, res):
        with self._lock:
            self.armed = bool(req.data)
            self._touch()
        self.get_logger().info('조작 허용 켬' if self.armed else '조작 잠금')
        res.success, res.message = True, ('조작 허용' if self.armed else '조작 잠금')
        return res

    def _srv_set_hand_guide(self, req, res):
        on = bool(req.data)
        if on and self.picker.running:
            res.success, res.message = False, '집기/놓기 순서가 진행 중 - 끝난 뒤에 켜세요'
            return res
        self.armed = False                  # 손으로 움직이는 동안 GUI 명령은 잠근다 (예전 GUI 와 같음)
        res.success = bool(self.link.set_hand_guide(on))
        res.message = ('모드 전환 시작' if res.success else self._last_link_log('모드 전환 거부'))
        return res

    def _srv_calibration(self, req, res):
        C = RealCalibration.Request
        c = self.calib
        ok, msg = True, ''
        if req.command == C.REMEMBER:
            near = nearest_px(self._fresh_px(2.0), req.u, req.v)
            if near is None:
                ok, msg = False, '보정 ①: 지금 카메라에 큐브가 보이지 않음 (가려지지 않았는지 확인)'
            else:
                self.calib_px = (near.pose.pose.position.x, near.pose.pose.position.y)
                msg = (f'보정 ①: {near.id or near.color} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f}) 기억 '
                       '→ 조그로 그리퍼를 그 큐브를 집는 자세까지 내린 뒤 ②')
        elif req.command == C.RECORD:
            p = self.link.posx
            if self.calib_px is None:
                ok, msg = False, '보정 ②: 먼저 ① 로 큐브 픽셀을 기억하세요'
            elif p is None or not self.link.joints_fresh():
                ok, msg = False, '보정 ②: 로봇 TCP 좌표를 읽지 못함 (접속 확인)'
            else:
                c.add(self.calib_px[0], self.calib_px[1], p[0], p[1], p[2], label=req.label)
                msg = (f'보정 ②: 점 {len(c.points)} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f}) '
                       f'-> TCP ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  |  {c.summary()}')
                self.calib_px = None
        elif req.command == C.REMOVE_LAST:
            c.remove_last()
            msg = f'마지막 점 삭제  |  {c.summary()}'
        elif req.command == C.CLEAR:
            c.clear()
            self.calib_px = None
            msg = '보정 점 모두 삭제'
        elif req.command == C.SAVE:
            if not c.points:
                ok, msg = False, '보정 저장: 기록한 점이 없음'
            else:
                msg = f'보정 저장: {c.save()}'
        elif req.command == C.LOAD:
            try:
                c.load()
                msg = f'보정 불러옴: {c.summary()}'
            except (OSError, ValueError, KeyError) as e:
                ok, msg = False, f'보정 불러오기 실패: {e}'
        else:
            ok, msg = False, f'모르는 명령 {req.command}'
        if ok and c.ready() and req.command in (C.RECORD, C.REMOVE_LAST, C.LOAD):
            pts = '  '.join(f'{i + 1}:{e:.1f}' for i, e in enumerate(c.errors_mm))
            msg += f'  |  점별 오차 mm {pts}'
        if ok:      # rclpy: 한 호출 위치에서 수준을 바꿔 쓰면 ValueError -> 두 줄로 나눈다
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        res.success, res.message = ok, msg
        return res

    # ================================================================== 액션 공통
    def _goal_cb(self, _goal):
        with self._lock:
            if self.active_goal:
                self.get_logger().warn('거부: 이전 동작이 진행 중')
                return GoalResponse.REJECT
            self.active_goal = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.link.stop()
        return CancelResponse.ACCEPT

    def _wait(self, goal_handle, busy, feedback=None, settle=0.3):
        """busy() 가 settle 초 동안 계속 False 일 때까지 기다린다. 취소 요청이 오면 정지를 보낸다."""
        time.sleep(0.1)
        idle_since, stop_sent, last_fb = None, False, 0.0
        while True:
            if goal_handle.is_cancel_requested and not stop_sent:
                self.link.stop()
                stop_sent = True
            if busy():
                idle_since = None
            else:
                idle_since = idle_since or time.time()
                if time.time() - idle_since >= settle:
                    return
            if feedback and time.time() - last_fb > 0.2:
                feedback()
                last_fb = time.time()
            time.sleep(0.05)

    def _exec_guard(self, fn, result_cls):
        """실행 콜백이 어떻게 끝나든(예외 포함) 다음 목표를 받을 수 있게 active_goal 을 푼다."""
        def run(goal_handle):
            try:
                return fn(goal_handle)
            except Exception as e:      # noqa: BLE001 - 로봇 정지 후 실패로 끝낸다
                self.link.stop()
                self.get_logger().error(f'동작 중 예외 -> 정지: {e!r}')
                goal_handle.abort()
                res = result_cls()
                res.success, res.message = False, f'내부 오류로 정지: {e!r}'
                return res
            finally:
                with self._lock:
                    self.active_goal = False
        return run

    def _finish(self, goal_handle, result):
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = result.message or '취소됨 (정지)'
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ================================================================== 액션: 이동·그리퍼
    def _exec_motion(self, goal_handle):
        g = goal_handle.request
        G = RealMotion.Goal
        res = RealMotion.Result()
        names = {G.JOG_JOINT: f'J{g.index + 1} {g.delta:+.1f}°', G.JOG_TCP: f'{"XYZ"[min(g.index, 2)]} {g.delta:+.1f} mm',
                 G.HOME: '홈 이동', G.RECOVER_HOME: '복구 후 홈', G.GRIPPER_OPEN: '그리퍼 열기',
                 G.GRIPPER_CLOSE: '그리퍼 닫기'}
        what = names.get(g.kind)
        if what is None:
            res.success, res.message = False, f'모르는 동작 {g.kind}'
            return self._finish(goal_handle, res)
        # 복구는 로봇이 대기가 아닐 때 쓰는 것이라 why_not_ready 를 보지 않는다 (RobotLink 가 자체 검사)
        why = self.why_blocked(need_ready=g.kind != G.RECOVER_HOME)
        if why:
            res.success, res.message = False, f'{what} 거부: {why}'
            self.get_logger().warn(res.message)
            return self._finish(goal_handle, res)
        self._touch()
        L = self.link
        L.last_ok = False
        if g.kind == G.JOG_JOINT:
            started = L.jog_joint(int(g.index), float(g.delta), joint_vel(g.speed_percent))
        elif g.kind == G.JOG_TCP:
            started = L.jog_cart(int(g.index), float(g.delta), cart_vel(g.speed_percent))
        elif g.kind == G.HOME:
            started = L.move_home(joint_vel(g.speed_percent))
        elif g.kind == G.RECOVER_HOME:
            started = L.recover_and_home(joint_vel(g.speed_percent))
        else:
            close = g.kind == G.GRIPPER_CLOSE
            started = L.gripper_command('move', pos=GRIP_CLOSE_POS if close else 0, cur=GRIP_CURRENT_MA,
                                        vel=GRIP_VEL, label=what)
        if not started:
            res.success, res.message = False, self._last_link_log(f'{what} 거부')
            return self._finish(goal_handle, res)

        def fb():
            m = RealMotion.Feedback()
            m.message = self._last_link_log(what)
            goal_handle.publish_feedback(m)
        # 복구: recovering 이 끝난 뒤 홈 이동(busy)이 이어지므로 둘 다 볼 때까지 settle 을 길게
        self._wait(goal_handle, lambda: L.busy or L.recovering, fb,
                   settle=0.8 if g.kind == G.RECOVER_HOME else 0.3)
        self._touch()
        res.success = bool(L.last_ok) and not goal_handle.is_cancel_requested
        res.message = self._last_link_log(what)
        return self._finish(goal_handle, res)

    # ================================================================== 액션: 집기·놓기
    def _target_xy(self, name, u, v, notes):
        """집기 직전: 받은 픽셀 가까이의 지금 검출로 큐브 위치를 고친 뒤 로봇 (x, y)."""
        near = nearest_px(self._fresh_px(PX_FRESH_S), u, v, REFRESH_PX)
        if near is not None:
            u, v = near.pose.pose.position.x, near.pose.pose.position.y
        else:
            notes.append(f'{name or "큐브"} 가 지금 보이지 않음 - 받은 픽셀 사용')
        return self.calib.to_robot(u, v)

    def _exec_pick(self, goal_handle):
        g = goal_handle.request
        G = RealPickPlace.Goal
        res = RealPickPlace.Result()
        names = {G.HOVER: '큐브 위로', G.PICK: '집기', G.PLACE_AT: '고른 자리에 놓기',
                 G.PICK_AND_PLACE: '집어서 옮기기', G.PLACE_HERE: '여기 내려놓기'}
        what = names.get(g.kind)
        res.holding = hold_code(self.picker.holding, RealPickPlace.Result)
        if what is None:
            res.success, res.message = False, f'모르는 동작 {g.kind}'
            return self._finish(goal_handle, res)
        why = self.why_blocked() or (None if self.calib.ready() else '카메라 보정이 없음 (점 4개 이상 기록)')
        if why:
            res.success, res.message = False, f'{what} 거부: {why}'
            self.get_logger().warn(res.message)
            return self._finish(goal_handle, res)
        notes = []
        z = self.calib.grasp_z
        P = self.picker
        if g.kind in (G.HOVER, G.PICK, G.PICK_AND_PLACE):
            x, y = self._target_xy(g.target_name, g.target_u, g.target_v, notes)
        if g.kind in (G.PLACE_AT, G.PICK_AND_PLACE):
            px, py = self.calib.to_robot(g.place_u, g.place_v)
        for n in notes:
            self.get_logger().warn(f'{what}: {n}')
        self._touch()
        if g.kind == G.HOVER:
            started = P.hover(x, y, z)
        elif g.kind == G.PICK:
            started = P.pick(x, y, z)
        elif g.kind == G.PLACE_AT:
            started = P.place_at(px, py, z)
        elif g.kind == G.PICK_AND_PLACE:
            started = P.pick_and_place(x, y, z, px, py)
        else:
            started = P.place_here(z)
        if not started:
            res.success, res.message = False, f'{what} 거부 (이전 순서 진행 중이거나 현재 좌표 모름)'
            return self._finish(goal_handle, res)

        def fb():
            m = RealPickPlace.Feedback()
            m.step, m.step_index, m.step_count = P.step, P.step_index, P.step_count
            m.detail = self._last_link_log(P.step)
            goal_handle.publish_feedback(m)
        self._wait(goal_handle, lambda: P.running, fb, settle=0.0)
        self._touch()
        res.holding = hold_code(P.holding, RealPickPlace.Result)
        res.success = bool(P.last_ok) and not goal_handle.is_cancel_requested
        res.message = f'{what}: ' + ('완료' if res.success else f'「{P.step}」 에서 멈춤 - {self._last_link_log("")}')
        return self._finish(goal_handle, res)

    # ================================================================== 주기: 잠금 시간, 상태 발행
    def _tick(self):
        now = time.time()
        busy = self.link.busy or self.picker.running or self.active_goal
        if busy:
            self._touch()                   # 움직이는 동안에는 잠그지 않는다
        if self.armed and now - self.last_cmd > self.arm_timeout:
            self.armed = False
            self.get_logger().info(f'{self.arm_timeout:.0f}초 동안 명령이 없어 조작을 잠갔습니다')
        self.pub_state.publish(self.state_msg(now))

    def state_msg(self, now=None):
        now = now or time.time()
        L, P, c = self.link, self.picker, self.calib
        m = RealRobotState()
        m.stamp = self.get_clock().now().to_msg()
        m.connected = bool(L.joints_fresh())
        m.connecting = bool(self.conn.busy())
        m.connection_text = self.mgr.describe() or ('접속됨' if m.connected else '접속 안 됨')
        m.robot_state = int(L.robot_state or 0)
        name = L.state_name()
        m.robot_state_text = self.state_ko.get(name, name)
        m.robot_state_ok = L.robot_state in (STANDBY, MOVING)
        m.robot_mode = RealRobotState.ROBOT_MODE_UNKNOWN if L.robot_mode is None else int(L.robot_mode)
        m.mode_switching = bool(L.mode_switching)
        m.recovering = bool(L.recovering)
        m.can_recover = bool(L.cli_control.service_is_ready()) and not L.recovering
        m.busy = bool(L.busy)
        m.why_not_ready = (self.mgr.describe() if self.conn.busy() else L.why_not_ready()) or ''
        m.armed = bool(self.armed)
        m.armed_remaining_s = float(max(0.0, self.arm_timeout - (now - self.last_cmd))) if self.armed else 0.0
        if L.joints_deg is not None:
            m.joints_valid, m.joints_deg = True, [float(v) for v in L.joints_deg[:6]]
        if L.posx is not None:
            m.tcp_valid, m.tcp = True, [float(v) for v in L.posx[:6]]
        m.gripper_service_ready = bool(L.cli_drl_start.service_is_ready())
        m.gripper_running = bool(L.gripper_running)
        m.gripper_result = {None: RealRobotState.GRIPPER_RESULT_UNKNOWN, True: RealRobotState.GRIPPER_RESULT_OK,
                            False: RealRobotState.GRIPPER_RESULT_FAILED}[L.gripper_ok]
        if L.gripper:
            m.gripper_valid = True
            m.gripper_position = int(max(0, min(65535, L.gripper.get('position', 0))))
            m.gripper_torque = bool(L.gripper.get('torque'))
        m.sequence_running = bool(P.running)
        m.sequence_name, m.sequence_step = P.name, P.step
        m.holding = hold_code(P.holding)
        m.calib_points = len(c.points)
        m.calib_ready = bool(c.ready())
        if c.ready():
            m.calib_homography = [float(v) for v in c.H.reshape(-1)]
            m.calib_grasp_z_mm = float(c.grasp_z)
            e = [v for v in c.errors_mm if not math.isnan(v)]
            if e:
                m.calib_error_mean_mm, m.calib_error_max_mm = float(sum(e) / len(e)), float(max(e))
        m.calib_summary = c.summary()
        if self.calib_px is not None:
            m.calib_pending, (m.calib_pending_u, m.calib_pending_v) = True, map(float, self.calib_px)
        return m


def main(args=None):
    from e0509_manual.bringup import BringupManager, ConnectionController
    from e0509_manual.robot_link import STATE_KO, start_link

    rclpy.init(args=args)
    # 접속 설정은 이 노드의 파라미터로 받는다 (launch/real_bridge.launch.py)
    cfg = rclpy.create_node('aris3_real_bridge_cfg')
    for k, v in (('ns', 'e0509_real'), ('host', '110.120.1.66'), ('port', 12345), ('z_min_mm', 50.0)):
        cfg.declare_parameter(k, v, ANY_TYPE)
    ns, host = str(cfg.get_parameter('ns').value), str(cfg.get_parameter('host').value)
    port, z_min = int(cfg.get_parameter('port').value), float(cfg.get_parameter('z_min_mm').value)
    cfg.destroy_node()

    link, link_ex = start_link(ns, z_min)
    mgr = BringupManager(ns, host, port, rviz=False, log=link.log)
    conn = ConnectionController(link, mgr)
    node = RealBridge(link, mgr, conn, state_ko=STATE_KO)
    node.get_logger().info(f'로봇 {ns} @ {host}:{port}, 최저 높이 {z_min:.0f} mm')
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        if mgr.own_running():
            node.get_logger().info('이 노드가 실행한 로봇 접속을 끊습니다')
            mgr.stop()
        ex.shutdown()
        link_ex.shutdown()
        node.destroy_node()
        link.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
