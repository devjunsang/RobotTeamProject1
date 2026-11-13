"""가상 모드 부품 시험: ServiceGripper(그리퍼 서비스로 바꿔 끼우기), VirtualBringupManager(가상 컨트롤러 접속 명령)."""
import threading
import time
from types import SimpleNamespace

from aris3_interfaces.srv import GripperCommand
from aris3_real_bridge import virtual
from aris3_real_bridge.virtual import ServiceGripper, joint_to_grip_position, make_virtual_bringup


class FakeLink:
    """ServiceGripper 가 쓰는 RobotLink 부분."""

    def __init__(self, reason=None):
        """reason: why_not_ready() 가 돌려줄 거부 이유 (None = 움직일 수 있음)."""
        self._lock = threading.Lock()
        self.busy = self.gripper_running = self._stop_requested = False
        self.last_ok, self.gripper, self.gripper_ok = False, None, None
        self.reason = reason
        self.lines = []

    def why_not_ready(self):
        """거부 이유."""
        return self.reason

    def log(self, msg):
        """로그 기록."""
        self.lines.append(msg)


def wait_idle(link):
    """그리퍼 작업 스레드가 끝날 때까지 (최대 2 초)."""
    t0 = time.time()
    time.sleep(0.02)
    while link.busy and time.time() - t0 < 2:
        time.sleep(0.01)


def test_joint_to_grip_position_scale_and_clamp():
    """손가락 0 rad = 0 (열림), 1.1 rad = 740 (끝까지 닫힘), 범위 밖은 자른다."""
    assert joint_to_grip_position(0.0) == 0
    assert joint_to_grip_position(1.1) == 740
    assert joint_to_grip_position(0.55) == 370
    assert joint_to_grip_position(-0.1) == 0 and joint_to_grip_position(2.0) == 740


def test_close_on_object_reports_holding_position():
    """닫기 -> CLOSE 요청, 물체에 막혀 0.6 rad 에서 멈추면 위치 404 (= PickSequence 기준 「잡음」), 성공 기록."""
    link = FakeLink()
    sent = []

    def call(req, timeout):
        """가짜 서비스: 요청을 기록하고 성공 응답."""
        sent.append(req.command)
        return SimpleNamespace(success=True, holding=True, message='CLOSE: 막혀 멈춤')
    g = ServiceGripper(link, call, lambda: 0.6)
    g.install()
    assert link.gripper_command('move', pos=740, cur=300) is True
    wait_idle(link)
    assert sent == [GripperCommand.Request.CLOSE]
    assert link.gripper['position'] == 404 and link.gripper['torque'] is True
    assert link.last_ok and link.gripper_ok and not link.busy and not link.gripper_running


def test_refused_when_robot_not_ready_or_busy():
    """로봇이 준비되지 않았거나 다른 동작 중이면 거부하고 서비스를 부르지 않는다."""
    called = []
    link = FakeLink(reason='로봇에 연결되어 있지 않음')
    g = ServiceGripper(link, lambda r, t: called.append(r), lambda: 0.0)
    assert g.command('move', pos=0) is False
    assert any('거부' in m for m in link.lines)
    link.reason, link.busy = None, True
    assert g.command('move', pos=0) is False
    assert called == []


def test_no_response_marks_failure():
    """서비스 응답이 없으면 실패로 기록 (그리퍼 결과 FAILED, last_ok False)."""
    link = FakeLink()
    g = ServiceGripper(link, lambda r, t: None, lambda: 0.0)
    assert g.command('move', pos=0)
    wait_idle(link)
    assert link.gripper_ok is False and not link.last_ok and not link.busy


def test_virtual_bringup_launches_virtual_mode(monkeypatch):
    """가상 접속 명령: dsr_bringup2 mode:=virtual, host 127.0.0.1, 이름 = 브리지 ns, RViz 끔, TF 분리."""
    class Base:
        """BringupManager 의 start 가 쓰는 부분만 흉내."""

        def __init__(self, ns, host, port, rviz=False, log=print):
            """실물 BringupManager 와 같은 인자."""
            self.ns, self.host, self.port, self.log = ns, host, port, log
            self._lock = threading.Lock()
            self.proc, self.log_path = None, '/dev/null'

        def running(self):
            """실행 중인 접속 없음."""
            return False

        def _pump(self, proc, logf):
            """출력 읽기 (시험에서는 하지 않음)."""

    started = {}

    class FakePopen:
        """명령만 기록하는 가짜 프로세스."""

        def __init__(self, cmd, **kw):
            """cmd 를 기록."""
            started['cmd'] = cmd
            self.stdout = []

    monkeypatch.setattr(virtual.subprocess, 'Popen', FakePopen)
    mgr = make_virtual_bringup(Base)('e0509_real', '127.0.0.1', 12345, log=lambda m: None)
    assert mgr.start() is True
    cmd = started['cmd']
    assert 'mode:=virtual' in cmd and 'host:=127.0.0.1' in cmd and 'name:=e0509_real' in cmd
    assert 'gui:=false' in cmd and 'remap_tf:=true' in cmd and 'mode:=real' not in cmd
