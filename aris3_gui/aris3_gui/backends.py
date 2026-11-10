"""모드별 로봇 연결: 시뮬레이션(Gazebo + MoveIt) / 실물(두산 E0509).

GUI 는 두 백엔드를 같은 방법으로 부른다:
  connect() / disconnect() / describe() / connected()
  joints_deg() / tcp_text()
  jog_joint(i, deg, vel) / jog_tcp(axis, mm, vel) / home() / gripper(open: bool) / stop()
  why_not_ready() -> None 이면 수동 조작 가능

작업 실행·카메라·인식·음성은 모드와 상관없이 ARIS3 공통 토픽(ros_io.GuiNode)으로 다룬다.
"""
import math
import os
import signal
import subprocess
import time

from aris3_gui.ros_io import ARM_JOINTS
from aris3_interfaces.srv import GripperCommand

SIM_LOG = '/tmp/aris3_gui_sim.log'
VOICE_LOG = '/tmp/aris3_gui_voice.log'
BRIDGE_LOG = '/tmp/aris3_gui_real_bridge.log'
GESTURE_LOG = '/tmp/aris3_gui_gesture.log'


class LaunchProc:
    """실물 모드 보조 노드(음성·손짓)를 ros2 launch 로 실행·종료한다 (시뮬레이션은 sim_full 이 음성을 띄움).

    이 노드들은 로봇을 움직이지 않는다 (인식 결과만 토픽으로 보냄).
    이미 다른 곳에서 같은 노드가 돌고 있으면 새로 띄우지 않는다 (명령이 두 번 나오지 않게).
    """

    def __init__(self, name, package, launch_file, pattern, log_path, log, args=()):
        """pattern: 이미 실행 중인지 pgrep -f 로 찾을 문자열, args: launch 인자 ['host:=...', ...]."""
        self.name, self.package, self.launch_file = name, package, launch_file
        self.pattern, self.log_path, self.log = pattern, log_path, log
        self.args = list(args)
        self.proc = None

    def external_running(self):
        """이 GUI 가 띄우지 않은 같은 노드가 이미 돌고 있는가."""
        return subprocess.run(['pgrep', '-f', self.pattern], capture_output=True).returncode == 0

    def start(self, note=''):
        """아직 없으면 ros2 launch 로 띄운다 (자식 프로세스 그룹을 따로 둬 GUI 를 꺼도 한 번에 정리)."""
        if self.proc is not None and self.proc.poll() is None:
            return
        if self.external_running():
            self.log('INFO', f'{self.name} 노드가 이미 실행 중 - 그것을 사용')
            return
        out = open(self.log_path, 'w')
        self.proc = subprocess.Popen(['ros2', 'launch', self.package, self.launch_file] + self.args,
                                     stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        self.log('INFO', f'{self.name} 노드 시작{note} (기록 {self.log_path})')

    def stop(self, timeout=5.0):
        """이 GUI 가 띄운 것만 끈다 (SIGINT -> 시간 안에 안 꺼지면 SIGKILL). 외부에서 띄운 노드는 그대로."""
        if self.proc is None or self.proc.poll() is not None:
            self.proc = None
            return
        pgid = os.getpgid(self.proc.pid)
        os.killpg(pgid, signal.SIGINT)
        end = time.time() + timeout
        while time.time() < end and self.proc.poll() is None:
            time.sleep(0.1)
        if self.proc.poll() is None:
            os.killpg(pgid, signal.SIGKILL)
        self.proc = None


class SimStack:
    """ros2 launch aris3_bringup aris3_sim_full.launch.py 를 실행·종료한다 (프로세스 그룹 단위)."""

    def __init__(self, grasp_assist=True, voice=True):
        self.grasp_assist = grasp_assist
        self.voice = voice
        self.proc = None

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if self.running():
            return
        log = open(SIM_LOG, 'w')
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'aris3_bringup', 'aris3_sim_full.launch.py',
             f'grasp_assist:={str(self.grasp_assist).lower()}', f'voice:={str(self.voice).lower()}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self, timeout=20.0):
        if not self.running():
            return
        pgid = os.getpgid(self.proc.pid)
        os.killpg(pgid, signal.SIGINT)        # launch 가 자식(Gazebo, MoveIt ...)을 차례로 끈다
        end = time.time() + timeout
        while time.time() < end and self.proc.poll() is None:
            time.sleep(0.2)
        if self.proc.poll() is None:
            os.killpg(pgid, signal.SIGKILL)
        self.proc = None


class SimBackend:
    mode = 'sim'
    title = 'Gazebo 시뮬레이션'

    def __init__(self, node, grasp_assist=True, voice=True):
        self.node = node
        self.stack = SimStack(grasp_assist, voice)

    # ---- 연결
    def connect(self):
        if self.connected():
            self.node.log('INFO', '시뮬레이션이 이미 실행 중 (다른 곳에서 실행한 것 포함)')
            return
        self.stack.start()
        self.node.log('INFO', f'시뮬레이션 시작: Gazebo -> MoveIt -> Task Manager (기록 {SIM_LOG})')

    def disconnect(self):
        if self.stack.running():
            self.node.log('INFO', '시뮬레이션 종료 중...')
            self.stack.stop()
            self.node.log('INFO', '시뮬레이션 종료')
        else:
            self.node.log('WARN', '이 GUI 가 실행한 시뮬레이션이 없음 (다른 곳에서 실행했다면 그쪽에서 끄세요)')

    def connected(self):
        return self.node.js_rate.fresh(1.0)

    def describe(self):
        s = self.node.status_flags()
        if self.stack.running() and not s['task_manager']:
            return '시작 중... (Gazebo -> MoveIt -> Task Manager)'
        if self.stack.running():
            return '이 GUI 가 실행한 시뮬레이션'
        return '외부에서 실행한 시뮬레이션' if s['joint_states'] else '실행 안 됨'

    # ---- 상태
    def joints_deg(self):
        j = self.node.snapshot()['joints']
        if not all(n in j for n in ARM_JOINTS):
            return None
        return [math.degrees(j[n]) for n in ARM_JOINTS]

    def tcp_text(self):
        pose = self.node.tcp_pose()
        if pose is None:
            return None
        _, tcp, rpy = pose
        return tcp, rpy

    def gripper_position(self):
        """0 = 열림 ~ 1 = 다 닫힘 (gripper_rh_r1 / 1.1 rad)."""
        j = self.node.snapshot()['joints']
        return None if 'gripper_rh_r1' not in j else max(0.0, min(1.0, j['gripper_rh_r1'] / 1.1))

    def gripper_status(self):
        """시스템 상태 패널용 (ok, 문구, 대기색). 시뮬레이션은 /aris3/gripper/command 서비스로 판단."""
        ok = self.node.status_flags()['gripper']
        return ok, ('준비' if ok else '서비스 없음'), False

    def why_not_ready(self):
        s = self.node.status_flags()
        if not s['joint_states']:
            return '관절 상태가 들어오지 않음 (Connect 로 시뮬레이션 시작)'
        if not s['moveit']:
            return 'MoveIt 준비 중'
        if self.node.task_running:
            return '작업 실행 중'
        if self.node.manual_busy:
            return '이전 수동 이동 진행 중'
        return None

    # ---- 명령
    def jog_joint(self, i, deg, vel_pct):
        self.node.jog_joint(i, deg, vel_pct / 100.0)

    def jog_tcp(self, axis, mm, vel_pct):
        self.node.jog_tcp(axis, mm, vel_pct / 100.0)

    def home(self, vel_pct=30):
        self.node.home(vel_pct / 100.0)

    def gripper(self, close):
        self.node.gripper(GripperCommand.Request.CLOSE if close else GripperCommand.Request.OPEN)

    def stop(self):
        self.node.stop_all()

    def hand_guide_state(self):
        return None                     # 시뮬레이션에는 없음

    def set_hand_guide(self, on):
        return False

    def set_armed(self, on):
        """시뮬레이션은 GUI 의 「조작 허용」 만 쓴다 (브리지 없음)."""
        return True

    def can_recover(self):
        """시뮬레이션에는 복구 후 홈이 없다."""
        return False

    def shutdown(self):
        self.stack.stop()


class RealBackend:
    """실물 E0509: aris3_real_bridge 노드를 통해 움직인다 (2026-11-10, 예전에는 GUI 안에서 RobotLink 를 직접 씀).

    GUI 를 켤 때 브리지가 없으면 함께 띄운다 (음성·손짓과 같은 방식). 로봇 안전 검사·접속은 브리지 안의
    e0509_manual RobotLink 가 그대로 하고, GUI 는 확인창·조작 허용만 맡는다.
    실물 Task Manager(두산 백엔드) 연동은 아직 검증되지 않았으므로 GUI 는 실물 모드에서 자동 작업 실행을 막는다.
    """
    mode = 'real'
    title = '실물 E0509'

    def __init__(self, node, ns='e0509_real', host='110.120.1.66', port=12345, z_min=50.0, rviz=False,
                 voice=True, gesture=True):
        """브리지 클라이언트를 만들고 브리지·음성·손짓 노드를 (없으면) 띄운다. rviz 는 예전 호환용 (쓰지 않음)."""
        from aris3_gui.real_client import RealBridgeClient
        from aris3_interfaces.action import RealMotion
        from aris3_interfaces.msg import RealRobotState
        self.node = node
        self.M, self.S = RealMotion.Goal, RealRobotState      # 메시지 상수 짧게
        self.bridge = RealBridgeClient(node)
        self.bridge_proc = LaunchProc('실물 브리지', 'aris3_real_bridge', 'real_bridge.launch.py',
                                      # 실제 노드 프로세스만: 「python3 .../lib/aris3_real_bridge/real_bridge ...」
                                      # (느슨한 패턴은 그 문자열이 들어간 다른 명령줄에도 걸려 브리지를 띄우지 않았다)
                                      r'^\S*python[0-9.]* \S*/lib/aris3_real_bridge/real_bridge( |$)', BRIDGE_LOG, node.log,
                                      args=[f'ns:={ns}', f'host:={host}', f'port:={port}', f'z_min_mm:={z_min}'])
        self.bridge_proc.start(f' - {host}, 최저 높이 {z_min:.0f} mm')
        # 로봇과 상관없이 GUI 를 켤 때 띄운다 (모델 로딩이 오래 걸림)
        self.voice = LaunchProc('음성', 'aris3_voice', 'voice.launch.py', 'aris3_voice/voice_node',
                                VOICE_LOG, node.log)
        self.gesture = LaunchProc('손짓(MediaPipe)', 'aris3_gesture', 'gesture.launch.py',
                                  'aris3_gesture/gesture_node', GESTURE_LOG, node.log)
        if voice:
            self.voice.start(' - Whisper 로딩 10~30 초')
        if gesture:
            self.gesture.start()

    def _st(self):
        """브리지의 마지막 상태 (1 초 안에 받은 것) 또는 None."""
        return self.bridge.state()

    # ---- 연결
    def connect(self):
        """로봇 접속 (서보 켜짐, 팔은 움직이지 않음). 진행은 상태 토픽으로."""
        self.bridge.trigger('connect', '로봇 접속')

    def disconnect(self):
        """로봇 접속 끊기 (서보 꺼짐)."""
        self.bridge.trigger('disconnect', '접속 끊기')

    def connected(self):
        """관절 상태가 들어오는가."""
        st = self._st()
        return bool(st and st.connected)

    def describe(self):
        """연결 설명 (시스템 상태 패널)."""
        st = self._st()
        if st is None:
            return '실물 브리지 없음 (aris3_real_bridge 시작 중이거나 꺼짐)'
        return st.connection_text

    # ---- 상태
    def joints_deg(self):
        """관절 6개 [deg] 또는 None."""
        st = self._st()
        return list(st.joints_deg) if (st and st.joints_valid) else None

    def tcp_text(self):
        """((x, y, z) [m], (rx, ry, rz) [rad]) 또는 None - 화면은 시뮬레이션과 같은 단위로 그린다."""
        st = self._st()
        if not (st and st.tcp_valid):
            return None
        p = list(st.tcp)
        return ((p[0] / 1000, p[1] / 1000, p[2] / 1000), tuple(math.radians(v) for v in p[3:6]))

    def gripper_position(self):
        """0 = 열림 ~ 1 = 다 닫힘 (그리퍼 위치 0~740)."""
        st = self._st()
        return st.gripper_position / 740.0 if (st and st.gripper_valid) else None

    def gripper_status(self):
        """시스템 상태 패널용 (ok, 문구, 대기색)."""
        st = self._st()
        if st is None or not st.gripper_service_ready:
            return False, 'DRL 서비스 없음 (접속 필요)', False
        if st.gripper_running:
            return True, '동작 중', False
        if st.gripper_result == self.S.GRIPPER_RESULT_FAILED:
            return False, '마지막 명령 실패', False
        if st.gripper_result == self.S.GRIPPER_RESULT_UNKNOWN:
            return False, '응답 확인 전 (열기/닫기로 확인)', True
        return True, '준비', False

    def why_not_ready(self):
        """지금 움직일 수 없는 이유 또는 None (조작 허용은 GUI 가 따로 본다)."""
        st = self._st()
        if st is None:
            return '실물 브리지 노드 없음'
        if self.bridge.motion_pending:
            return '이전 동작의 결과를 기다리는 중'
        return st.why_not_ready or None

    def robot_state_text(self):
        """두산 상태 한국어 ("대기", "안전 정지" ...)."""
        st = self._st()
        return st.robot_state_text if st else '모름 (브리지 없음)'

    def can_recover(self):
        """「복구 후 홈」 을 누를 수 있는가."""
        st = self._st()
        return bool(st and st.can_recover)

    # ---- 명령 (모두 브리지 액션: 결과는 로그로)
    def jog_joint(self, i, deg, vel_pct):
        """관절 i 를 deg 만큼 (속도 비율 %, 최대 45 deg/s)."""
        self.bridge.motion(self.M.JOG_JOINT, f'J{i + 1} {deg:+g}°', index=i, delta=deg, speed_percent=vel_pct)

    def jog_tcp(self, axis, mm, vel_pct):
        """TCP 를 베이스 X/Y/Z 로 mm 만큼 (최대 100 mm/s)."""
        self.bridge.motion(self.M.JOG_TCP, f'TCP {"XYZ"[axis]} {mm:+g} mm', index=axis, delta=mm,
                           speed_percent=vel_pct)

    def home(self, vel_pct=30):
        """홈 자세 [0, 0, 90, 0, 90, 0] 로."""
        self.bridge.motion(self.M.HOME, '홈 이동', speed_percent=vel_pct)

    def recover_and_home(self, vel_pct=30):
        """안전 정지 해제/서보 켜기 -> 자동 모드 -> 홈."""
        return self.bridge.motion(self.M.RECOVER_HOME, '복구 후 홈', speed_percent=vel_pct)

    def gripper(self, close):
        """그리퍼 닫기(잡기)/열기."""
        self.bridge.motion(self.M.GRIPPER_CLOSE if close else self.M.GRIPPER_OPEN,
                           '그리퍼 닫기' if close else '그리퍼 열기')

    def stop(self):
        """정지: 브리지(로봇·집기 순서) + Task Manager·MoveIt. 언제나 보낸다."""
        self.bridge.stop()
        self.node.stop_all()

    def hand_guide_state(self):
        """(수동 모드인가 True/False/None 모름, 바꾸는 중인가, 모드 이름)."""
        st = self._st()
        if st is None or st.robot_mode == self.S.ROBOT_MODE_UNKNOWN:
            return None, bool(st and st.mode_switching), '모름'
        manual = st.robot_mode == self.S.ROBOT_MODE_MANUAL
        return manual, st.mode_switching, '수동 (손으로 움직이기)' if manual else '자동'

    def set_hand_guide(self, on):
        """손으로 움직이기 켜기/끄기 (브리지가 조작 허용도 잠금)."""
        return self.bridge.set_hand_guide(on)

    def set_armed(self, on):
        """브리지 쪽 조작 허용을 GUI 체크와 맞춘다."""
        return self.bridge.set_armed(on)

    def shutdown(self):
        """GUI 종료: 이 GUI 가 띄운 노드만 끈다 (브리지를 끄면 브리지가 시작한 로봇 접속도 끊김 - 예전과 같음)."""
        self.voice.stop()
        self.gesture.stop()
        self.bridge_proc.stop(timeout=25.0)
