"""ROS 2 side of the E0509 manual controller.

Talks to a running dsr_bringup2 (real or virtual) through the standard Doosan services.
All safety checks live here, not in the GUI, so they hold no matter who calls them:

  * motion only while the robot reports STANDBY and joint states are fresh
  * one motion at a time; the next command is refused until the robot is idle again
  * every target is ABSOLUTE and built from a freshly read pose (current + small delta)
  * step and speed are clamped; joint limits (with margin) and a minimum TCP height are enforced
  * stop is always allowed and never waits for anything

Motion commands are sent asynchronously (amovej / amovel) so that move_stop can be serviced
while the robot is moving.
"""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotError
from dsr_msgs2.srv import (DrlStart, DrlStop, GetCurrentPosx, GetDrlState, GetOutputRegisterInt,
                           GetRobotMode, GetRobotState, MoveJoint, MoveLine, MoveStop, SetOutputRegisterInt,
                           SetRobotControl, SetRobotMode)

from e0509_manual import gripper_drl

JOINTS = [f'joint_{i}' for i in range(1, 7)]

# E0509 joint limits from dsr_description2 (deg), minus a safety margin.
JOINT_LIMITS = [(-360, 360), (-360, 360), (-155, 155), (-360, 360), (-360, 360), (-360, 360)]
LIMIT_MARGIN = 5.0

MAX_JOINT_STEP = 10.0     # deg per click
MAX_JOINT_VEL = 45.0      # deg/s (2026-11-09 30 -> 45)
MAX_CART_STEP = 20.0      # mm per click
MAX_CART_VEL = 100.0      # mm/s (2026-11-09 50 -> 100)
# 목표 좌표 이동(move_tcp_to, 카메라로 집기용 2026-11-09): 베이스 중심에서 XY 거리 범위와 한 번 이동 거리 제한
REACH_MIN_MM = 150.0      # 베이스 바로 위(자기 몸)로 들어가지 않게
REACH_MAX_MM = 750.0      # E0509 도달 거리 약 900 mm 보다 여유 있게
MAX_MOVE_MM = 400.0       # 한 번에 이 거리보다 멀면 거부 (보정이 틀려 엉뚱한 곳으로 가는 것을 막음)
STALE_SEC = 0.5           # joint states older than this -> no motion
# 이동이 끝난 뒤 목표에 실제로 도착했는지 확인 (2026-11-09: 도달 불가 자세를 컨트롤러가 거부했는데
# 상태가 대기로 돌아와 「완료」 로 읽히고 다음 단계가 비스듬히 움직인 일이 있었다)
ARRIVE_TOL_MM = 5.0
ARRIVE_TOL_DEG = 1.0

STATE_NAMES = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
    5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
    9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY',
}
STATE_KO = {
    'STANDBY': '대기', 'MOVING': '이동 중', 'SAFE_OFF': '서보 꺼짐', 'SAFE_STOP': '안전 정지',
    'EMERGENCY_STOP': '비상 정지', 'TEACHING': '직접 교시', 'INITIALIZING': '초기화 중',
    'HOMMING': '원점 복귀 중', 'RECOVERY': '복구 모드', 'SAFE_STOP2': '안전 정지 2',
    'SAFE_OFF2': '서보 꺼짐 2', 'NOT_READY': '준비 안 됨', 'UNKNOWN': '알 수 없음 (접속 안 됨)',
}
STANDBY, MOVING = 1, 2
SAFE_OFF, SAFE_STOP, EMERGENCY_STOP, RECOVERY, SAFE_STOP2, SAFE_OFF2 = 3, 5, 6, 8, 9, 10
# set_robot_control 값 (dsr_common2 DRFC.h ROBOT_CONTROL)
CONTROL_RESET_SAFE_STOP = 2   # 안전 정지(SAFE_STOP) 해제
CONTROL_SERVO_ON = 3          # 서보 꺼짐(SAFE_OFF) -> 서보 켜기
HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # ARIS3 홈(관찰 자세, Task Manager 와 같음): 손끝이 아래를 봄
HOME_VEL = 30.0                                # deg/s
# 로봇 모드 (dsr_msgs2 SetRobotMode). 접속하면 dsr_hardware2 가 AUTONOMOUS 로 둔다.
#   MANUAL     : 팔 끝 콕핏의 직접 교시 버튼을 누른 채 손으로 팔을 끌 수 있다 (상태가 TEACHING 으로 바뀜).
#                API 이동·DRL(그리퍼) 은 쓰지 않는다 -> why_not_ready 가 막는다.
#   AUTONOMOUS : ROS/API 명령으로 움직이는 모드. 콕핏 직접 교시는 막힌다.
ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS = 0, 1
ROBOT_MODE_KO = {0: '수동 (손으로 움직이기)', 1: '자동', 2: '측정'}
STOP_TYPE_QUICK = 1       # 0 would also cut servo power (STO); 1 = quick stop, servo stays on


class RobotLink(Node):
    def __init__(self, ns='e0509_real', z_min_mm=50.0):
        super().__init__('e0509_manual_controller')
        self.ns = ns.strip('/')
        self.z_min_mm = float(z_min_mm)
        p = f'/{self.ns}'

        self._lock = threading.Lock()
        self.joints_deg = None          # latest [6] deg
        self.joints_stamp = 0.0         # wall time of last joint_states
        self.js_rate = 0.0
        self._js_count, self._js_t0 = 0, time.time()
        self.posx = None                # [x, y, z, rx, ry, rz] mm / deg, base frame
        self.robot_state = None         # int
        self.robot_mode = None          # int (ROBOT_MODE_*), None = 모름
        self.mode_switching = False     # 모드 바꾸는 중
        self.last_error = ''
        self.busy = False
        self._stop_requested = False
        self.last_ok = False            # 마지막 이동·그리퍼 명령이 정상으로 끝났는가 (wait_motion 이 읽음)
        self.log_lines = []             # last 200 lines
        self.log_seq = 0                # total lines ever logged

        self.create_subscription(JointState, f'{p}/joint_states', self._on_js, 20)
        self.create_subscription(RobotError, f'{p}/error', self._on_error, 20)
        self.cli_movej = self.create_client(MoveJoint, f'{p}/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, f'{p}/motion/move_line')
        self.cli_stop = self.create_client(MoveStop, f'{p}/motion/move_stop')
        self.cli_state = self.create_client(GetRobotState, f'{p}/system/get_robot_state')
        self.cli_get_mode = self.create_client(GetRobotMode, f'{p}/system/get_robot_mode')
        self.cli_set_mode = self.create_client(SetRobotMode, f'{p}/system/set_robot_mode')
        self.cli_control = self.create_client(SetRobotControl, f'{p}/system/set_robot_control')
        self.recovering = False
        self.cli_posx = self.create_client(GetCurrentPosx, f'{p}/aux_control/get_current_posx')
        self.cli_drl_start = self.create_client(DrlStart, f'{p}/drl/drl_start')
        self.cli_drl_stop = self.create_client(DrlStop, f'{p}/drl/drl_stop')
        self.cli_drl_state = self.create_client(GetDrlState, f'{p}/drl/get_drl_state')
        self.cli_greg = self.create_client(GetOutputRegisterInt, f'{p}/plc/get_output_register_int')
        self.cli_sreg = self.create_client(SetOutputRegisterInt, f'{p}/plc/set_output_register_int')

        self.gripper = None             # dict from gripper_drl.decode() of the last good reply
        self.gripper_stamp = 0.0        # wall time of self.gripper
        self.gripper_ok = None          # None unknown, True last command answered, False failed
        self.gripper_running = False    # a gripper DRL program is running right now
        self._grip_token = 0
        self.connect_seq = 0            # bumps on every successful connect from the GUI

        self._state_pending = 0.0      # wall time of the outstanding request, 0 = none
        self._posx_pending = 0.0
        self.create_timer(0.2, self._poll_state)
        self.create_timer(0.5, self._poll_posx)
        self._mode_pending = 0.0
        self.create_timer(1.0, self._poll_mode)

    # ------------------------------------------------------------------ incoming data
    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        if not all(j in d for j in JOINTS):
            return
        now = time.time()
        with self._lock:
            self.joints_deg = [math.degrees(d[j]) for j in JOINTS]
            self.joints_stamp = now
            self._js_count += 1
            if now - self._js_t0 >= 1.0:
                self.js_rate = self._js_count / (now - self._js_t0)
                self._js_count, self._js_t0 = 0, now

    def _on_error(self, m):
        txt = f'[{m.level}/{m.group}/{m.code}] {m.msg1} {m.msg2} {m.msg3}'.strip()
        if m.level == 1:                    # LOG_LEVEL_SYSINFO: information, not an error
            if m.code != 2007:              # 2007 = info on every DRL program run (gripper commands)
                self.log(f'로봇 알림(정보): {txt}')
            return
        with self._lock:
            self.last_error = txt
        self.log(f'로봇 {"경고" if m.level == 2 else "에러"}: {txt}')

    def _poll_state(self):
        if not self.cli_state.service_is_ready():
            self.robot_state = None             # never show a stale state after a disconnect
            return
        # a request sent just before a disconnect never completes: give up on it after 2 s
        if self._state_pending and time.time() - self._state_pending < 2.0:
            return
        self._state_pending = time.time()
        fut = self.cli_state.call_async(GetRobotState.Request())

        def done(f):
            self._state_pending = 0.0
            r = f.result()
            if r is not None and r.success:
                self.robot_state = int(r.robot_state)
        fut.add_done_callback(done)

    def _poll_mode(self):
        if not self.cli_get_mode.service_is_ready():
            self.robot_mode = None
            return
        if self.mode_switching or (self._mode_pending and time.time() - self._mode_pending < 2.0):
            return
        self._mode_pending = time.time()
        fut = self.cli_get_mode.call_async(GetRobotMode.Request())

        def done(f):
            self._mode_pending = 0.0
            r = f.result()
            if r is not None and r.success and not self.mode_switching:
                self.robot_mode = int(r.robot_mode)
        fut.add_done_callback(done)

    def set_hand_guide(self, on):
        """on=True: 수동 모드 (콕핏 버튼으로 손으로 움직이기), False: 자동 모드로 돌아감. 작업 스레드에서 실행."""
        label = '손으로 움직이기 켜기 (수동 모드)' if on else '손으로 움직이기 끄기 (자동 모드)'
        if not self.cli_set_mode.service_is_ready():
            self.log(f'{label} 거부: /{self.ns}/system/set_robot_mode 서비스 없음 (접속 필요)')
            return False
        reason = None
        with self._lock:                # log() 도 이 잠금을 쓰므로 안에서 log 하지 않는다
            if self.busy or self.mode_switching:
                reason = '이동·그리퍼 명령 또는 모드 전환이 진행 중'
            elif on and self.robot_state != STANDBY:
                reason = f'로봇 상태가 대기가 아님 ({STATE_KO.get(self.state_name(), self.state_name())})'
            else:
                self.mode_switching = True
        if reason:
            self.log(f'{label} 거부: {reason}')
            return False
        threading.Thread(target=self._run_set_mode, args=(label, on), daemon=True).start()
        return True

    def _run_set_mode(self, label, on):
        want = ROBOT_MODE_MANUAL if on else ROBOT_MODE_AUTONOMOUS
        try:
            req = SetRobotMode.Request()
            req.robot_mode = want
            r = self._call(self.cli_set_mode, req)
            if r is None or not r.success:
                self.log(f'{label}: 실패 (컨트롤러가 거부 - 펜던트가 제어권을 가졌거나 직접 교시 중일 수 있음)')
                return
            g = self._call(self.cli_get_mode, GetRobotMode.Request())      # 실제로 바뀌었는지 다시 읽는다
            if g is not None and g.success:
                self.robot_mode = int(g.robot_mode)
            ok = self.robot_mode == want
            self.log(f'{label}: ' + ('완료' if ok else f'요청은 성공했지만 지금 모드 {self.robot_mode}'))
            if ok and on:
                self.log('팔 끝 콕핏의 직접 교시 버튼을 누른 채 팔을 끌어 움직이세요. '
                         '이 모드에서는 GUI 이동·그리퍼 명령이 막힙니다.')
        finally:
            self.mode_switching = False

    # ------------------------------------------------------------------ 복구 · 홈
    def move_home(self, vel_deg_s=HOME_VEL):
        """홈 자세(HOME_DEG)로 관절 이동. jog 와 같은 검사(대기 상태·관절 신선도·한계)를 거친다."""
        vel = max(1.0, min(MAX_JOINT_VEL, float(vel_deg_s)))
        reason = self.why_not_ready()
        if reason:
            self.log(f'홈 이동 거부: {reason}')
            return False
        for i, t in enumerate(HOME_DEG):
            lo, hi = JOINT_LIMITS[i]
            if not lo + LIMIT_MARGIN <= t <= hi - LIMIT_MARGIN:
                self.log(f'홈 이동 거부: J{i + 1} 목표 {t:.0f}° 가 범위 밖')
                return False
        req = MoveJoint.Request()
        req.pos = [float(x) for x in HOME_DEG]
        req.vel, req.acc = vel, 2.0 * vel
        req.mode, req.sync_type = 0, 1
        return self._start_motion(f'홈 자세 {HOME_DEG} ({vel:.0f}°/s)', self.cli_movej, req,
                                  target_joints=list(HOME_DEG))

    def recover_and_home(self, vel_deg_s=HOME_VEL):
        """안전 정지·서보 꺼짐을 해제하고(필요하면 자동 모드로) 홈으로 간다. 작업 스레드에서 실행.

        비상정지·복구 모드는 사람이 로봇 쪽에서 풀어야 하므로 할 일만 알려 준다.
        """
        if not self.cli_control.service_is_ready():
            self.log('복구 거부: 로봇에 연결되어 있지 않음. 비상정지였다면 버튼 해제 -> 펜던트/DART 에서 리셋 '
                     '-> 「접속 끊기」 -> 10 초 뒤 「접속」')
            return False
        with self._lock:
            if self.recovering or self.busy or self.mode_switching:
                busy = True
            else:
                busy, self.recovering = False, True
        if busy:
            self.log('복구 거부: 다른 명령이 진행 중')
            return False
        threading.Thread(target=self._run_recover, args=(vel_deg_s,), daemon=True).start()
        return True

    def _wait_state(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.robot_state == want:
                return True
            time.sleep(0.2)
        return False

    def _run_recover(self, vel):
        try:
            st = self.robot_state
            name = STATE_KO.get(self.state_name(), self.state_name())
            self.log(f'복구 시작: 지금 상태 {name}')
            if st == EMERGENCY_STOP:
                self.log('복구 중단: 비상정지 상태 - 비상정지 버튼을 돌려 해제한 뒤 다시 누르세요 (소프트웨어로 풀 수 없음)')
                return
            if st == RECOVERY:
                self.log('복구 중단: 복구 모드 - 펜던트/DART 에서 한계를 벗어난 관절을 안쪽으로 옮긴 뒤 복구 모드를 끝내세요')
                return
            ctrl = {SAFE_STOP: CONTROL_RESET_SAFE_STOP, SAFE_STOP2: CONTROL_RESET_SAFE_STOP,
                    SAFE_OFF: CONTROL_SERVO_ON, SAFE_OFF2: CONTROL_SERVO_ON}.get(st)
            if ctrl is not None:
                req = SetRobotControl.Request()
                req.robot_control = ctrl
                r = self._call(self.cli_control, req)
                what = '안전 정지 해제' if ctrl == CONTROL_RESET_SAFE_STOP else '서보 켜기'
                if r is None or not r.success:
                    self.log(f'복구: {what} 실패 (컨트롤러가 거부 - 한계를 벗어난 관절이 있으면 펜던트의 복구 모드 필요)')
                    return
                self.log(f'복구: {what} 요청 성공 - 대기 상태를 기다림')
            elif st != STANDBY:
                self.log(f'복구 중단: 이 상태({name})는 자동으로 풀지 않음')
                return
            if not self._wait_state(STANDBY, 10.0):
                self.log(f'복구 실패: 10 초 안에 대기 상태가 되지 않음 (지금 {STATE_KO.get(self.state_name(), self.state_name())})')
                return
            if self.robot_mode == ROBOT_MODE_MANUAL:     # 손으로 움직이기 중이었으면 자동 모드로
                req = SetRobotMode.Request()
                req.robot_mode = ROBOT_MODE_AUTONOMOUS
                r = self._call(self.cli_set_mode, req)
                g = self._call(self.cli_get_mode, GetRobotMode.Request())
                if g is not None and g.success:
                    self.robot_mode = int(g.robot_mode)
                if r is None or not r.success or self.robot_mode != ROBOT_MODE_AUTONOMOUS:
                    self.log('복구 실패: 자동 모드로 바꾸지 못함')
                    return
                self.log('복구: 자동 모드로 바꿈')
            self.log('복구 완료 (대기 상태) -> 홈으로 이동')
            time.sleep(0.5)                                  # 관절 상태·좌표가 다시 들어오게
        finally:
            self.recovering = False
        if self.move_home(vel):
            if self.wait_motion():
                self.log('홈 도착')

    def _poll_posx(self):
        if not self.cli_posx.service_is_ready():
            return
        if self._posx_pending and time.time() - self._posx_pending < 2.0:
            return
        self._posx_pending = time.time()
        req = GetCurrentPosx.Request()
        req.ref = 0  # base
        fut = self.cli_posx.call_async(req)

        def done(f):
            self._posx_pending = 0.0
            r = f.result()
            if r is not None and r.success and r.task_pos_info:
                self.posx = list(r.task_pos_info[0].data[:6])
        fut.add_done_callback(done)

    # ------------------------------------------------------------------ helpers
    def log(self, msg):
        line = time.strftime('%H:%M:%S ') + msg
        with self._lock:
            self.log_lines.append(line)
            del self.log_lines[:-200]
            self.log_seq += 1
        self.get_logger().info(msg)

    def on_connected(self):
        with self._lock:
            self.last_error = ''
            self.connect_seq += 1
        self.gripper_ok = None

    def on_disconnected(self):
        with self._lock:
            self.robot_state = None
            self.robot_mode = None
            self.posx = None
            self.last_error = ''
        self.gripper_ok = None

    def state_name(self):
        if self.robot_state is None:
            return 'UNKNOWN'
        return STATE_NAMES.get(self.robot_state, str(self.robot_state))

    def joints_fresh(self):
        return self.joints_deg is not None and time.time() - self.joints_stamp < STALE_SEC

    def why_not_ready(self):
        """None if a new motion may be sent, else a short Korean reason."""
        if not self.cli_movej.service_is_ready():
            return f'/{self.ns} 로봇에 연결되어 있지 않음'
        if not self.joints_fresh():
            return '관절 각도가 들어오지 않음'
        if self.mode_switching:
            return '로봇 모드 바꾸는 중'
        if self.recovering:
            return '복구 중'
        if self.robot_mode == ROBOT_MODE_MANUAL:
            return '손으로 움직이기(수동 모드) 켜짐 - 체크를 끄면 자동 모드로 돌아감'
        if self.robot_state != STANDBY:
            return f'로봇 상태가 대기가 아님 ({STATE_KO.get(self.state_name(), self.state_name())})'
        if self.busy:
            return '이전 동작이 끝나지 않음'
        return None

    def _call(self, client, req, timeout=3.0):
        """Blocking call from a worker thread (the executor spins in another thread)."""
        ev = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return fut.result()

    def _fresh_posx(self):
        req = GetCurrentPosx.Request()
        req.ref = 0
        r = self._call(self.cli_posx, req)
        if r is None or not r.success or not r.task_pos_info:
            return None
        return list(r.task_pos_info[0].data[:6])

    # ------------------------------------------------------------------ motion
    def jog_joint(self, idx, delta_deg, vel_deg_s):
        """Move one joint by delta_deg (absolute target from the current pose)."""
        delta = max(-MAX_JOINT_STEP, min(MAX_JOINT_STEP, float(delta_deg)))
        vel = max(1.0, min(MAX_JOINT_VEL, float(vel_deg_s)))
        reason = self.why_not_ready()
        if reason:
            self.log(f'J{idx + 1} 이동 거부: {reason}')
            return False
        cur = list(self.joints_deg)
        target = list(cur)
        target[idx] += delta
        lo, hi = JOINT_LIMITS[idx]
        if not (lo + LIMIT_MARGIN <= target[idx] <= hi - LIMIT_MARGIN):
            self.log(f'J{idx + 1} 이동 거부: 목표 {target[idx]:.1f}°가 허용 범위 '
                     f'[{lo + LIMIT_MARGIN:.0f}, {hi - LIMIT_MARGIN:.0f}]° 밖')
            return False
        req = MoveJoint.Request()
        req.pos = [float(x) for x in target]
        req.vel, req.acc = vel, 2.0 * vel
        req.mode, req.sync_type = 0, 1      # absolute, async (so stop works mid-motion)
        return self._start_motion(f'J{idx + 1} {delta:+.1f}° ({vel:.0f}°/s)', self.cli_movej, req,
                                  target_joints=list(req.pos))

    def jog_cart(self, axis, delta_mm, vel_mm_s):
        """Move the TCP along base X/Y/Z (axis 0..2) by delta_mm, orientation unchanged."""
        delta = max(-MAX_CART_STEP, min(MAX_CART_STEP, float(delta_mm)))
        vel = max(1.0, min(MAX_CART_VEL, float(vel_mm_s)))
        reason = self.why_not_ready()
        name = 'XYZ'[axis]
        if reason:
            self.log(f'{name} 이동 거부: {reason}')
            return False
        cur = self._fresh_posx()
        if cur is None:
            self.log(f'{name} 이동 거부: 현재 좌표를 읽지 못함')
            return False
        target = list(cur)
        target[axis] += delta
        if target[2] < self.z_min_mm:
            self.log(f'{name} 이동 거부: 목표 높이 Z={target[2]:.1f} mm가 최저 높이 '
                     f'{self.z_min_mm:.0f} mm보다 낮음')
            return False
        req = MoveLine.Request()
        req.pos = [float(x) for x in target]
        req.vel = [vel, 30.0]
        req.acc = [2.0 * vel, 60.0]
        req.ref, req.mode, req.sync_type = 0, 0, 1   # base frame, absolute, async
        return self._start_motion(f'{name} {delta:+.1f} mm ({vel:.0f} mm/s)', self.cli_movel, req,
                                  target_xyz=tuple(req.pos[:3]))

    def move_tcp_to(self, x_mm, y_mm, z_mm, vel_mm_s, label=None):
        """TCP 를 베이스 좌표 (x, y, z) mm 로 직선 이동. 자세(회전)는 지금 그대로 유지한다.

        jog 와 같은 검사(대기 상태·관절 신선도·최저 높이)에 더해 도달 범위와 한 번 이동 거리를 제한한다.
        끝날 때까지 기다리려면 wait_motion() 을 부른다.
        """
        label = label or f'이동 ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm'
        vel = max(1.0, min(MAX_CART_VEL, float(vel_mm_s)))
        reason = self.why_not_ready()
        if reason:
            self.log(f'{label} 거부: {reason}')
            return False
        cur = self._fresh_posx()
        if cur is None:
            self.log(f'{label} 거부: 현재 좌표를 읽지 못함')
            return False
        if z_mm < self.z_min_mm:
            self.log(f'{label} 거부: 목표 높이 Z={z_mm:.1f} mm가 최저 높이 {self.z_min_mm:.0f} mm보다 낮음')
            return False
        reach = math.hypot(x_mm, y_mm)
        if not REACH_MIN_MM <= reach <= REACH_MAX_MM:
            self.log(f'{label} 거부: 베이스에서 {reach:.0f} mm (허용 {REACH_MIN_MM:.0f}~{REACH_MAX_MM:.0f} mm)')
            return False
        dist = math.dist(cur[:3], (x_mm, y_mm, z_mm))
        if dist > MAX_MOVE_MM:
            self.log(f'{label} 거부: 이동 거리 {dist:.0f} mm 가 한 번 이동 한계 {MAX_MOVE_MM:.0f} mm 보다 큼')
            return False
        req = MoveLine.Request()
        req.pos = [float(x_mm), float(y_mm), float(z_mm)] + [float(v) for v in cur[3:6]]
        req.vel = [vel, 30.0]
        req.acc = [2.0 * vel, 60.0]
        req.ref, req.mode, req.sync_type = 0, 0, 1   # base frame, absolute, async
        return self._start_motion(f'{label} ({vel:.0f} mm/s, {dist:.0f} mm)', self.cli_movel, req,
                                  target_xyz=(float(x_mm), float(y_mm), float(z_mm)))

    def wait_motion(self, timeout=90.0):
        """이동·그리퍼 명령이 끝날 때까지 기다린다 (작업 스레드에서만). 성공적으로 끝났으면 True."""
        t0 = time.time()
        time.sleep(0.1)
        while self.busy:
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return self.last_ok and not self._stop_requested

    def _start_motion(self, label, client, req, target_xyz=None, target_joints=None):
        with self._lock:
            if self.busy:
                return False
            self.busy = True
        threading.Thread(target=self._run_motion, args=(label, client, req, target_xyz, target_joints),
                         daemon=True).start()
        return True

    def _check_arrival(self, label, target_xyz, target_joints):
        """끝난 뒤 실제 위치가 목표와 맞는가. 맞지 않으면 실패로 기록하고 False."""
        if target_xyz is not None:
            p = self._fresh_posx()
            if p is None:
                self.log(f'{label}: 도착 확인 실패 (현재 좌표를 읽지 못함) -> 실패로 처리')
                return False
            d = math.dist(p[:3], target_xyz)
            if d > ARRIVE_TOL_MM:
                self.log(f'{label}: 목표에 도착하지 못함 (남은 거리 {d:.0f} mm - 도달 불가 자세였거나 컨트롤러가 '
                         f'이동을 거부) -> 실패로 처리')
                return False
        if target_joints is not None:
            j = self.joints_deg
            if j is None:
                self.log(f'{label}: 도착 확인 실패 (관절 각도 없음) -> 실패로 처리')
                return False
            err = max(abs(a - b) for a, b in zip(j, target_joints))
            if err > ARRIVE_TOL_DEG:
                self.log(f'{label}: 목표 관절 각도에 도착하지 못함 (최대 차이 {err:.1f}°) -> 실패로 처리')
                return False
        return True

    def _run_motion(self, label, client, req, target_xyz=None, target_joints=None):
        self.last_ok = False
        try:
            r = self._call(client, req, timeout=5.0)
            if r is None or not r.success:
                self.log(f'{label}: 컨트롤러가 명령을 거부함')
                return
            self._stop_requested = False
            self.log(f'{label}: 시작')
            # Wait until the robot has started (MOVING) or 0.6 s passed, then until it is idle again.
            t0 = time.time()
            while time.time() - t0 < 0.6 and self.robot_state != MOVING:
                time.sleep(0.05)
            t1 = time.time()
            while time.time() - t1 < 60.0:
                if self.robot_state != MOVING and self._joints_still(0.3):
                    break
                time.sleep(0.05)
            else:
                self.log(f'{label}: 60초 안에 끝나지 않음')
                return
            if self._stop_requested:
                self.log(f'{label}: 정지 명령으로 멈춤 (상태 {STATE_KO.get(self.state_name(), self.state_name())})')
                return
            if not self._check_arrival(label, target_xyz, target_joints):
                return
            self.last_ok = True
            self.log(f'{label}: 완료 (상태 {STATE_KO.get(self.state_name(), self.state_name())})')
        finally:
            self.busy = False

    def _joints_still(self, window):
        a = self.joints_deg
        time.sleep(window)
        b = self.joints_deg
        return a is not None and b is not None and max(abs(x - y) for x, y in zip(a, b)) < 0.01

    def stop(self):
        """Quick stop. Never blocks; always sent."""
        if not self.cli_stop.service_is_ready():
            self.log('정지 명령을 보낼 수 없음: 로봇에 연결되어 있지 않음')
            return
        req = MoveStop.Request()
        req.stop_mode = STOP_TYPE_QUICK
        self._stop_requested = True
        self.cli_stop.call_async(req)
        if self.gripper_running and self.cli_drl_stop.service_is_ready():
            q = DrlStop.Request()
            q.stop_mode = STOP_TYPE_QUICK
            self.cli_drl_stop.call_async(q)
            self.log('정지 명령 보냄 (그리퍼 프로그램도 중단 — 이미 보낸 손가락 목표는 그대로 진행될 수 있음)')
        else:
            self.log('정지 명령 보냄')

    # ------------------------------------------------------------------ gripper
    GRIP_LABELS = {'status': '그리퍼 상태 읽기', 'move': '그리퍼 이동', 'torque_off': '그리퍼 힘 풀기',
                   'prepare': '그리퍼 설정만 쓰기(토크 끈 채)'}

    def gripper_command(self, cmd, pos=0, cur=200, vel=600, label=None, retries=None):
        """Run one gripper DRL program in a worker thread. Shares the 'busy' lock with arm motion."""
        label = label or self.GRIP_LABELS.get(cmd, cmd)
        reason = self.why_not_ready()
        if reason is None and not self.cli_drl_start.service_is_ready():
            reason = 'DRL 서비스가 없음'
        if reason:
            self.log(f'{label} 거부: {reason}')
            return False
        with self._lock:
            if self.busy:
                return False
            self.busy = True
        if retries is None:
            retries = 1 if cmd == 'status' else 0      # only the read-only command is repeated automatically
        threading.Thread(target=self._run_gripper_retry, args=(label, cmd, pos, cur, vel, retries),
                         daemon=True).start()
        return True

    def _run_gripper_retry(self, label, cmd, pos, cur, vel, retries):
        self.last_ok = False
        self._stop_requested = False    # 이전 정지가 이번 명령의 결과로 잘못 읽히지 않게
        try:
            if cmd == 'move':
                retries = max(retries, 1)   # 재접속 직후 첫 명령은 토크가 안 켜져 안 움직이는 일이 있음 -> 한 번 더
            for attempt in range(retries + 1):
                self._grip_token = self._grip_token % 250 + 1
                code = gripper_drl.build(cmd, self._grip_token, pos=pos, cur=cur, vel=vel)
                ok = self._run_gripper(label, code, self._grip_token)
                if ok and cmd == 'move' and not (self.gripper or {}).get('torque'):
                    # 「완료」 로 끝났어도 토크가 꺼져 있으면 손가락은 움직이지 않았다 (2026-11-09 실측: 위치 2, 토크 꺼짐)
                    self.log(f'{label}: 토크가 켜지지 않아 손가락이 움직이지 않음 -> 실패로 처리')
                    self.gripper_ok = False
                    ok = False
                if ok:
                    self.last_ok = True
                    return
                if attempt == retries:
                    return
                self.log(f'{label}: 2초 뒤 한 번 더 시도')
                time.sleep(2.0)
                if self.robot_state != STANDBY:
                    return
        finally:
            self.busy = False

    def _greg(self, addr):
        q = GetOutputRegisterInt.Request()
        q.address, q.timeout_ms = addr, 500
        r = self._call(self.cli_greg, q, timeout=2.0)
        return gripper_drl.reg_byte(r.value) if (r is not None and r.success) else None

    def _drl_state(self):
        r = self._call(self.cli_drl_state, GetDrlState.Request(), timeout=2.0)
        return r.drl_state if (r is not None and r.success) else None

    def _run_gripper(self, label, code, token):
        t0 = time.time()
        try:
            self.gripper_running = True
            # clear status + token so stale results from an earlier run can never be taken as ours
            for a in (0, 22):
                q = SetOutputRegisterInt.Request()
                q.address, q.value = a, 0
                self._call(self.cli_sreg, q)
            q = DrlStart.Request()
            q.robot_system, q.code = 0, code
            r = self._call(self.cli_drl_start, q, timeout=5.0)
            if r is None or not r.success:
                self.log(f'{label}: 컨트롤러가 DRL 실행을 거부함 (펜던트에서 다른 프로그램이 돌고 있지 않은지 확인)')
                self.gripper_ok = False
                return False
            self.log(f'{label}: 시작')
            done = False
            while time.time() - t0 < gripper_drl.MOVE_TIMEOUT_S + 12.0:
                time.sleep(0.25)
                if self._greg(22) == token:
                    done = True
                    break
                if time.time() - t0 > 2.0 and self._drl_state() == 1 and self._greg(22) != token:
                    break                                   # program ended without writing our token
            if not done:
                self.log(f'{label}: 결과를 받지 못함 (DRL이 중간에 멈췄거나 시간 초과)')
                self.gripper_ok = False
                if self._drl_state() == 0:
                    q = DrlStop.Request()
                    q.stop_mode = STOP_TYPE_QUICK
                    self._call(self.cli_drl_stop, q)
                return False
            b = [self._greg(a) for a in range(24)]
            if any(v is None for v in b):
                self.log(f'{label}: 결과 레지스터를 읽지 못함')
                self.gripper_ok = False
                return False
            d = gripper_drl.decode(b)
            if d['status'] == 2:
                self.gripper, self.gripper_stamp, self.gripper_ok = d, time.time(), True
                extra = ' — 시간 안에 멈추지 않음(아직 움직이는 중일 수 있음)' if d['timed_out'] else ''
                err = f', 하드웨어 에러 {d["hw_error"]}' if d['hw_error'] else ''
                self.log(f'{label}: 완료 ({time.time() - t0:.1f}s) 위치 {d["position"]}, 전류 {d["current_mA"]} mA, '
                         f'토크 {"켜짐" if d["torque"] else "꺼짐"}{err}{extra}')
                return True
            elif d['status'] == 4:
                self.gripper_ok = False
                if d['stage'] == 8:
                    self.log(f'{label}: 거부 — 그리퍼 토크가 켜져 있음')
                else:
                    self.log(f'{label}: 그리퍼가 응답하지 않음 (단계: {d["stage_name"]}). '
                             '계속되면 「재접속」 버튼을 누르세요')
            else:
                self.gripper_ok = False
                self.log(f'{label}: DRL 실행 오류 (단계: {d["stage_name"]}, 상태 {d["status"]})')
            return False
        finally:
            # let the controller finish the program before anything else is sent
            t1 = time.time()
            while time.time() - t1 < 3.0 and self._drl_state() == 0:
                time.sleep(0.1)
            self.gripper_running = False


def start_link(ns, z_min_mm):
    """Create the node and spin it in a background thread. Returns (node, executor)."""
    if not rclpy.ok():
        rclpy.init()
    node = RobotLink(ns, z_min_mm)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return node, ex
