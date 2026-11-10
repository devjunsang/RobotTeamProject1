"""PixelCalibration·PickSequence 단위 시험 (로봇 없이, 가짜 RobotLink)."""
import time

import pytest

from aris3_real_bridge.real_pick import APPROACH_MM, PickSequence, PixelCalibration, travel_waypoints


def affine(u, v):
    """시험용 정답 변환: 픽셀 -> 로봇 mm."""
    return 300.0 + 0.8 * u - 0.1 * v, -200.0 + 0.05 * u + 0.75 * v


def make_calib(n=6, z=(180, 182, 181, 183, 179, 182)):
    """정답 변환(affine)으로 만든 보정점 n 개짜리 PixelCalibration."""
    c = PixelCalibration()
    pts = [(100, 100), (500, 120), (480, 400), (120, 380), (300, 250), (200, 300)][:n]
    for (u, v), zz in zip(pts, z):
        x, y = affine(u, v)
        c.add(u, v, x, y, zz, label='cube')
    return c


def test_calibration_needs_four_points():
    """점이 3개뿐이면 보정을 쓸 수 없다 (변환 결과 None)."""
    c = make_calib(n=3)
    assert not c.ready()
    assert c.to_robot(10, 10) is None


def test_calibration_recovers_affine_map_and_median_height():
    """6 점으로 맞춘 변환이 정답과 같고, 집는 높이는 z 의 중앙값이다."""
    c = make_calib()
    assert c.ready()
    x, y = c.to_robot(333, 222)
    ex, ey = affine(333, 222)
    assert x == pytest.approx(ex, abs=1e-6) and y == pytest.approx(ey, abs=1e-6)
    assert c.grasp_z == pytest.approx(181.5)
    assert max(c.errors_mm) < 1e-6


def test_calibration_save_load_roundtrip(tmp_path):
    """저장한 보정을 다시 읽으면 같은 변환이 나온다."""
    c = make_calib()
    path = c.save(str(tmp_path / 'calib.json'))
    d = PixelCalibration()
    assert d.load(path)
    assert d.to_robot(250, 250) == pytest.approx(c.to_robot(250, 250))


def test_travel_waypoints_split_and_avoid_base():
    """베이스를 가로지르는 먼 이동은 나눠지고, 중간 점은 베이스에서 300 mm 밖으로 밀린다."""
    pts = travel_waypoints((400.0, 0.0), (-400.0, 10.0))
    assert pts[-1] == (-400.0, 10.0)
    for x, y in pts[:-1]:
        assert (x * x + y * y) ** 0.5 >= 299.0       # 베이스 가까이 지나지 않음


class FakeLink:
    """PickSequence 가 쓰는 RobotLink 부분만. 이동은 즉시 도착한다."""

    def __init__(self, grip_close_pos=500):
        """로봇 위치는 (400, 0, 300) mm 에서 시작. grip_close_pos: 닫았을 때 멈추는 위치 (500 = 잡음, 740 = 빈손)."""
        self.posx = [400.0, 0.0, 300.0, 180.0, 0.0, 90.0]
        self.gripper = None
        self.last_ok = False
        self.calls = []
        self.grip_close_pos = grip_close_pos

    def move_tcp_to(self, x, y, z, vel, label=None):
        """이동 기록 후 바로 도착한 것으로 한다."""
        self.calls.append(('move', round(x), round(y), round(z), label))
        self.posx[:3] = [x, y, z]
        self.last_ok = True
        return True

    def gripper_command(self, cmd, pos=0, cur=0, vel=0, label=None):
        """그리퍼 명령 기록. 닫기면 grip_close_pos 에서 멈춘 것으로, 토크는 켜짐."""
        self.calls.append(('grip', pos))
        self.gripper = {'position': self.grip_close_pos if pos else 0, 'torque': True}
        self.last_ok = True
        return True

    def wait_motion(self, timeout=90.0):
        """이동이 즉시 끝나므로 마지막 결과만 돌려준다."""
        return self.last_ok


def run(seq, fn, *args):
    """순서를 시작하고 끝날 때까지 (최대 5 초) 기다린다."""
    assert fn(*args)
    t0 = time.time()
    while seq.running and time.time() - t0 < 5:
        time.sleep(0.01)
    assert not seq.running


def test_pick_runs_steps_in_order_and_reports_holding():
    """집기: 이동·열기·하강·닫기·들기 순서, 하강·들기 높이, 물체에 막혀 멈추면 잡음으로 판단."""
    link = FakeLink(grip_close_pos=500)              # 물체에 막혀 500 에서 멈춤 = 잡음
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0)
    kinds = [c[0] for c in link.calls]
    assert kinds == ['move', 'grip', 'move', 'grip', 'move']
    assert link.calls[2][3] == 180                    # 집는 높이로 하강
    assert link.calls[-1][3] == 180 + APPROACH_MM     # 들어 올리기
    assert seq.holding is True and seq.last_ok
    assert seq.name == '집기' and seq.step_count == 6 and seq.step == '들어 올리기'


def test_pick_and_place_does_not_carry_when_empty():
    """집어서 옮기기: 빈손(끝까지 닫힘)이면 놓을 자리로 옮기지 않고 확인 단계에서 멈춘다."""
    link = FakeLink(grip_close_pos=740)              # 끝까지 닫힘 = 빈손
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick_and_place, 420.0, 30.0, 180.0, 300.0, -250.0)
    assert seq.holding is False and not seq.last_ok
    assert not any(c[0] == 'move' and c[1] == 300 and c[2] == -250 for c in link.calls)
    assert seq.step == '잡았는지 확인'


def test_second_start_is_refused_while_running():
    """이미 순서가 진행 중이면 새 순서를 거부한다."""
    link = FakeLink()
    seq = PickSequence(link, lambda s: None)
    seq.running = True
    assert not seq.hover(400.0, 0.0, 180.0)
