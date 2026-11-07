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
                           GetRobotState, MoveJoint, MoveLine, MoveStop, SetOutputRegisterInt)

from e0509_manual import gripper_drl

JOINTS = [f'joint_{i}' for i in range(1, 7)]

# E0509 joint limits from dsr_description2 (deg), minus a safety margin.
JOINT_LIMITS = [(-360, 360), (-360, 360), (-155, 155), (-360, 360), (-360, 360), (-360, 360)]
LIMIT_MARGIN = 5.0

MAX_JOINT_STEP = 10.0     # deg per click
MAX_JOINT_VEL = 30.0      # deg/s
MAX_CART_STEP = 20.0      # mm per click
MAX_CART_VEL = 50.0       # mm/s
STALE_SEC = 0.5           # joint states older than this -> no motion

STATE_NAMES = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
    5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
    9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY',
}
STATE_KO = {
    'STANDBY': '대기', 'MOVING': '이동 중', 'SAFE_OFF': '서보 꺼짐', 'SAFE_STOP': '안전 정지',
    'EMERGENCY_STOP': '비상 정지', 'TEACHING': '직접 교시', 'INITIALIZING': '초기화 중',
    'HOMMING': '원점 복귀 중', 'RECOVERY': '복구 모드', 'SAFE_STOP2': '안전 정지 2',
    'SAFE_OFF2': '서보 꺼짐 2', 'NOT_READY': '준비 안 됨',
}
STANDBY, MOVING = 1, 2
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
        self.last_error = ''
        self.busy = False
        self._stop_requested = False
        self.log_lines = []             # last 200 lines
        self.log_seq = 0                # total lines ever logged

        self.create_subscription(JointState, f'{p}/joint_states', self._on_js, 20)
        self.create_subscription(RobotError, f'{p}/error', self._on_error, 20)
        self.cli_movej = self.create_client(MoveJoint, f'{p}/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, f'{p}/motion/move_line')
        self.cli_stop = self.create_client(MoveStop, f'{p}/motion/move_stop')
        self.cli_state = self.create_client(GetRobotState, f'{p}/system/get_robot_state')
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

        self._state_pending = False
        self._posx_pending = False
        self.create_timer(0.2, self._poll_state)
        self.create_timer(0.5, self._poll_posx)

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
        if self._state_pending or not self.cli_state.service_is_ready():
            return
        self._state_pending = True
        fut = self.cli_state.call_async(GetRobotState.Request())

        def done(f):
            self._state_pending = False
            r = f.result()
            if r is not None and r.success:
                self.robot_state = int(r.robot_state)
        fut.add_done_callback(done)

    def _poll_posx(self):
        if self._posx_pending or not self.cli_posx.service_is_ready():
            return
        self._posx_pending = True
        req = GetCurrentPosx.Request()
        req.ref = 0  # base
        fut = self.cli_posx.call_async(req)

        def done(f):
            self._posx_pending = False
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
        return self._start_motion(f'J{idx + 1} {delta:+.1f}° ({vel:.0f}°/s)', self.cli_movej, req)

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
        return self._start_motion(f'{name} {delta:+.1f} mm ({vel:.0f} mm/s)', self.cli_movel, req)

    def _start_motion(self, label, client, req):
        with self._lock:
            if self.busy:
                return False
            self.busy = True
        threading.Thread(target=self._run_motion, args=(label, client, req), daemon=True).start()
        return True

    def _run_motion(self, label, client, req):
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
            how = '정지 명령으로 멈춤' if self._stop_requested else '완료'
            self.log(f'{label}: {how} (상태 {STATE_KO.get(self.state_name(), self.state_name())})')
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

    def gripper_command(self, cmd, pos=0, cur=200, vel=600, label=None):
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
        self._grip_token = self._grip_token % 250 + 1
        args = (label, gripper_drl.build(cmd, self._grip_token, pos=pos, cur=cur, vel=vel), self._grip_token)
        threading.Thread(target=self._run_gripper, args=args, daemon=True).start()
        return True

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
                return
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
                return
            b = [self._greg(a) for a in range(24)]
            if any(v is None for v in b):
                self.log(f'{label}: 결과 레지스터를 읽지 못함')
                self.gripper_ok = False
                return
            d = gripper_drl.decode(b)
            if d['status'] == 2:
                self.gripper, self.gripper_stamp, self.gripper_ok = d, time.time(), True
                extra = ' — 시간 안에 멈추지 않음(아직 움직이는 중일 수 있음)' if d['timed_out'] else ''
                err = f', 하드웨어 에러 {d["hw_error"]}' if d['hw_error'] else ''
                self.log(f'{label}: 완료 ({time.time() - t0:.1f}s) 위치 {d["position"]}, 전류 {d["current_mA"]} mA, '
                         f'토크 {"켜짐" if d["torque"] else "꺼짐"}{err}{extra}')
            elif d['status'] == 4:
                self.gripper_ok = False
                if d['stage'] == 8:
                    self.log(f'{label}: 거부 — 그리퍼 토크가 켜져 있음')
                else:
                    self.log(f'{label}: 그리퍼가 응답하지 않음 (단계: {d["stage_name"]}). '
                             '계속되면 로봇 접속(bringup)을 다시 시작하세요')
            else:
                self.gripper_ok = False
                self.log(f'{label}: DRL 실행 오류 (단계: {d["stage_name"]}, 상태 {d["status"]})')
        finally:
            # let the controller finish the program before anything else is sent
            t1 = time.time()
            while time.time() - t1 < 3.0 and self._drl_state() == 0:
                time.sleep(0.1)
            self.gripper_running = False
            self.busy = False


def start_link(ns, z_min_mm):
    """Create the node and spin it in a background thread. Returns (node, executor)."""
    if not rclpy.ok():
        rclpy.init()
    node = RobotLink(ns, z_min_mm)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return node, ex
