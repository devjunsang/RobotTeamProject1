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
GESTURE_LOG = '/tmp/aris3_gui_gesture.log'


class LaunchProc:
    """실물 모드 보조 노드(음성·손짓)를 ros2 launch 로 실행·종료한다 (시뮬레이션은 sim_full 이 음성을 띄움).

    이 노드들은 로봇을 움직이지 않는다 (인식 결과만 토픽으로 보냄).
    이미 다른 곳에서 같은 노드가 돌고 있으면 새로 띄우지 않는다 (명령이 두 번 나오지 않게).
    """

    def __init__(self, name, package, launch_file, pattern, log_path, log):
        self.name, self.package, self.launch_file = name, package, launch_file
        self.pattern, self.log_path, self.log = pattern, log_path, log
        self.proc = None

    def external_running(self):
        return subprocess.run(['pgrep', '-f', self.pattern], capture_output=True).returncode == 0

    def start(self, note=''):
        if self.proc is not None and self.proc.poll() is None:
            return
        if self.external_running():
            self.log('INFO', f'{self.name} 노드가 이미 실행 중 - 그것을 사용')
            return
        out = open(self.log_path, 'w')
        self.proc = subprocess.Popen(['ros2', 'launch', self.package, self.launch_file],
                                     stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        self.log('INFO', f'{self.name} 노드 시작{note} (기록 {self.log_path})')

    def stop(self, timeout=5.0):
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

    def shutdown(self):
        self.stack.stop()


class RealBackend:
    """실물 E0509: etc/e0509_manual 의 접속(BringupManager)·안전 검사(RobotLink)를 그대로 쓴다.

    실물 Task Manager(두산 백엔드) 연동은 아직 검증되지 않았으므로 GUI 는 실물 모드에서 작업 실행을 막는다.
    """
    mode = 'real'
    title = '실물 E0509'

    def __init__(self, node, ns='e0509_real', host='110.120.1.66', port=12345, z_min=50.0, rviz=False,
                 voice=True, gesture=True):
        from e0509_manual.bringup import BringupManager, ConnectionController   # 실물 모드에서만 필요
        from e0509_manual.robot_link import start_link
        self.node = node
        self.link, self.link_ex = start_link(ns, z_min)
        self.mgr = BringupManager(ns, host, port, rviz=rviz, log=self.link.log)
        self.conn = ConnectionController(self.link, self.mgr)
        self._log_seen = 0
        # 로봇과 상관없이 GUI 를 켤 때 띄운다 (모델 로딩이 오래 걸림)
        self.voice = LaunchProc('음성', 'aris3_voice', 'voice.launch.py', 'aris3_voice/voice_node',
                                VOICE_LOG, node.log)
        self.gesture = LaunchProc('손짓(MediaPipe)', 'aris3_gesture', 'gesture.launch.py',
                                  'aris3_gesture/gesture_node', GESTURE_LOG, node.log)
        if voice:
            self.voice.start(' - Whisper 로딩 10~30 초')
        if gesture:
            self.gesture.start()

    def connect(self):
        self.conn.connect_async()

    def disconnect(self):
        self.conn.disconnect_async()

    def connected(self):
        return self.link.joints_fresh()

    def describe(self):
        return self.mgr.describe() or ('접속됨' if self.connected() else '접속 안 됨')

    def joints_deg(self):
        return self.link.joints_deg

    def tcp_text(self):
        p = self.link.posx
        if p is None:
            return None
        return ((p[0] / 1000, p[1] / 1000, p[2] / 1000),
                tuple(math.radians(v) for v in p[3:6]))

    def gripper_position(self):
        d = self.link.gripper
        return None if not d else d['position'] / 740.0

    def gripper_status(self):
        """시스템 상태 패널용 (ok, 문구, 대기색). 실물은 RobotLink DRL 경로라 /aris3/gripper/command 를 쓰지 않는다."""
        if not self.link.cli_drl_start.service_is_ready():
            return False, 'DRL 서비스 없음 (접속 필요)', False
        if self.link.gripper_running:
            return True, '동작 중', False
        if self.link.gripper_ok is False:
            return False, '마지막 명령 실패', False
        if self.link.gripper_ok is None:
            return False, '응답 확인 전 (열기/닫기로 확인)', True
        return True, '준비', False

    def why_not_ready(self):
        if self.conn.busy():
            return self.mgr.describe()
        return self.link.why_not_ready()

    def jog_joint(self, i, deg, vel_pct):
        self.link.jog_joint(i, deg, max(5.0, 45.0 * vel_pct / 100.0))     # 실물 최대 45 deg/s

    def jog_tcp(self, axis, mm, vel_pct):
        self.link.jog_cart(axis, mm, max(10.0, 100.0 * vel_pct / 100.0))  # 실물 최대 100 mm/s

    def home(self, vel_pct=30):
        self.link.move_home(max(5.0, 45.0 * vel_pct / 100.0))      # 실물 최대 45 deg/s

    def recover_and_home(self, vel_pct=30):
        return self.link.recover_and_home(max(5.0, 45.0 * vel_pct / 100.0))

    def robot_state_text(self):
        from e0509_manual.robot_link import STATE_KO
        n = self.link.state_name()
        return STATE_KO.get(n, n)

    def gripper(self, close):
        self.link.gripper_command('move', pos=740 if close else 0, cur=300, vel=1500,
                                  label='그리퍼 닫기' if close else '그리퍼 열기')

    def stop(self):
        self.link.stop()
        self.node.stop_all()

    def hand_guide_state(self):
        """(수동 모드인가 True/False/None 모름, 바꾸는 중인가, 모드 이름)"""
        from e0509_manual.robot_link import ROBOT_MODE_KO, ROBOT_MODE_MANUAL
        m = self.link.robot_mode
        return (None if m is None else m == ROBOT_MODE_MANUAL, self.link.mode_switching,
                '모름' if m is None else ROBOT_MODE_KO.get(m, str(m)))

    def set_hand_guide(self, on):
        return self.link.set_hand_guide(on)

    def pump_logs(self):
        """RobotLink 기록을 GUI 로그로 옮긴다."""
        seq, lines = self.link.log_seq, list(self.link.log_lines)
        new = min(seq - self._log_seen, len(lines))
        for line in lines[-new:] if new > 0 else []:
            self.node.log('INFO', f'[실물] {line}')
        self._log_seen = seq

    def shutdown(self):
        self.voice.stop()
        self.gesture.stop()
        if self.mgr.own_running():
            self.mgr.stop()
        self.link_ex.shutdown()
        self.link.destroy_node()
