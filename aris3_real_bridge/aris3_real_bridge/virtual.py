"""가상 모드 (2026-11-10): 실물 로봇 없이 실물 경로를 시험하기 위한 두 가지 교체 부품.

  VirtualBringupManager : 「접속」 이 실물 dsr_bringup2(mode:=real) 대신 두산 가상 컨트롤러(mode:=virtual,
                          Docker DRCF 에뮬레이터)를 띄운다. 그 밖의 접속 절차(대기 상태 확인, 끊기)는 그대로다.
  ServiceGripper        : 그리퍼를 DRL(플랜지 RS-485, 에뮬레이터에는 없음) 대신 ARIS3 공통 그리퍼 서비스
                          /aris3/gripper/command 로 움직인다. 시뮬레이션에서는 aris3_sim_bridge gz_gripper_bridge 가 제공.

RobotLink(이동 전 검사·도착 확인·정지·복구)는 바꾸지 않는다: 가상 컨트롤러도 실물과 같은 두산 서비스를 내놓기 때문.
가상 컨트롤러가 재현하지 못하는 것: 충돌 감지·안전 정지(외력), 실제 그리퍼 DRL·토크, 관절 브레이크 -> 실물 재시험 필요.
"""
import subprocess
import threading
import time

from aris3_interfaces.srv import GripperCommand

GRIP_FULL = 740            # RobotLink 그리퍼 위치 단위 (0 열림 ~ 740 닫힘)


def joint_to_grip_position(rad, closed_rad=1.1):
    """Gazebo 손가락 관절 각도 [rad] -> RobotLink 그리퍼 위치 (0~740). 실물과 같은 「잡음/빈손」 판정에 쓴다."""
    return int(round(max(0.0, min(1.0, rad / closed_rad)) * GRIP_FULL))


def make_virtual_bringup(BringupManager):
    """e0509_manual 의 BringupManager 를 받아 가상 컨트롤러용 하위 클래스를 만든다.

    e0509_manual 은 (워크스페이스 사본이 실제로 실행되므로) 고치지 않고, 접속 명령만 바꾼다.
    """

    class VirtualBringupManager(BringupManager):
        """「접속」 = dsr_bringup2 mode:=virtual (가상 컨트롤러 Docker 를 함께 띄움). 주소는 127.0.0.1 고정."""

        def start(self):
            """가상 컨트롤러로 dsr_bringup2 를 띄운다. 실물용 네트워크·주소 검사는 하지 않는다 (같은 PC 안)."""
            with self._lock:
                if self.running():
                    self.log('접속 거부: 이미 접속되어 있음')
                    return False
                cmd = ['ros2', 'launch', 'dsr_bringup2', 'dsr_bringup2_rviz.launch.py',
                       'mode:=virtual', 'host:=127.0.0.1', f'port:={self.port}', 'model:=e0509',
                       'color:=white', f'name:={self.ns}', 'remap_tf:=true', 'gui:=false']
                self.crashed = False
                self._stopping = False
                logf = open(self.log_path, 'a')
                logf.write(f'\n===== {time.strftime("%F %T")} {" ".join(cmd)}\n')
                logf.flush()
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                             bufsize=1, start_new_session=True)
                threading.Thread(target=self._pump, args=(self.proc, logf), daemon=True).start()
                self.log(f'가상 컨트롤러 접속 시작 (Docker 에뮬레이터, 이름 /{self.ns}, 기록 {self.log_path})')
                return True

    return VirtualBringupManager


class ServiceGripper:
    """RobotLink.gripper_command 를 /aris3/gripper/command 호출로 바꿔 끼운다 (가상 모드 전용).

    RobotLink 와 같은 약속을 지킨다: 거부 이유 로그 + False, 실행 중에는 link.busy (이동과 같은 잠금),
    끝나면 link.gripper = {'position', 'torque'}, link.gripper_ok, link.last_ok. 그래서 PickSequence 의
    「잡았는지 확인」(위치 100~700 = 잡음, 740 = 빈손) 과 브리지 상태 토픽이 실물과 같은 방식으로 동작한다.
    """

    def __init__(self, link, call, read_joint):
        """link: RobotLink, call(req, timeout) -> 응답 또는 None, read_joint() -> 손가락 관절 [rad] 또는 None."""
        self.link, self.call, self.read_joint = link, call, read_joint

    def install(self):
        """link.gripper_command 를 이 객체의 command 로 바꾼다 (이 인스턴스만, RobotLink 클래스는 그대로)."""
        self.link.gripper_command = self.command
        self.link.log('그리퍼: 가상 모드 - /aris3/gripper/command 서비스로 움직임 (DRL 아님)')

    def command(self, cmd, pos=0, cur=200, vel=600, label=None, retries=None):
        """RobotLink.gripper_command 와 같은 모양. 'move' 만 지원 (pos > 0 = 닫기, 0 = 열기)."""
        L = self.link
        label = label or ('그리퍼 닫기' if pos else '그리퍼 열기')
        if cmd != 'move':
            L.log(f'{label} 거부: 가상 모드는 그리퍼 이동만 지원 ({cmd})')
            return False
        reason = L.why_not_ready()
        if reason:
            L.log(f'{label} 거부: {reason}')
            return False
        with L._lock:
            if L.busy:
                return False
            L.busy = True
        threading.Thread(target=self._run, args=(label, pos > 0, cur), daemon=True).start()
        return True

    def _run(self, label, close, cur):
        """작업 스레드: 서비스 호출 -> 손가락 위치 읽기 -> RobotLink 상태 갱신. 끝나면 busy 를 푼다."""
        L = self.link
        L.last_ok = False
        L._stop_requested = False
        L.gripper_running = True
        t0 = time.time()
        try:
            req = GripperCommand.Request(command=GripperCommand.Request.CLOSE if close else GripperCommand.Request.OPEN,
                                         force=float(max(0.0, min(1.0, cur / 1000.0))))
            r = self.call(req, 8.0)
            if r is None:
                L.gripper_ok = False
                L.log(f'{label}: 그리퍼 서비스 응답 없음 (aris3_sim_bridge gz_gripper_bridge 확인)')
                return
            if L._stop_requested:
                L.log(f'{label}: 정지 명령으로 멈춤')
                return
            rad = self.read_joint()
            if rad is not None:
                L.gripper = {'position': joint_to_grip_position(rad), 'torque': True, 'current_mA': cur,
                             'hw_error': 0, 'timed_out': False}
                L.gripper_stamp = time.time()
            L.gripper_ok = bool(r.success)
            L.last_ok = bool(r.success)
            pos = L.gripper['position'] if L.gripper else '?'
            L.log(f'{label}: {"완료" if r.success else "실패"} ({time.time() - t0:.1f}s) 위치 {pos} | {r.message}')
        finally:
            L.gripper_running = False
            L.busy = False
