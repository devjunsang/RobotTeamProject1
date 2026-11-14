"""PixelCalibration·PickSequence 단위 시험 (로봇 없이, 가짜 RobotLink)."""
import time

import pytest

import math

from aris3_real_bridge.real_pick import (APPROACH_MM, PickSequence, PixelCalibration, judge_hold, plan_j6, tool_axes,
                                         tool_yaw_deg, travel_waypoints, wrap90)


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


def test_pick_and_stack_places_one_cube_higher():
    """쌓기: 집는 높이 180 mm 로 집고, 받침 큐브 위에서는 180 + 큐브 40 + 여유 5 mm 까지만 내려가 놓는다."""
    link = FakeLink(grip_close_pos=500)
    seq = PickSequence(link, log=lambda m: None)
    run(seq, seq.pick_and_stack, 420.0, 30.0, 180.0, 300.0, -150.0)
    assert ('move', 420, 30, 180, '집는 높이로 하강') in link.calls
    assert ('move', 300, -150, 225, '놓는 높이로 하강') in link.calls       # 180 + 40 + 여유 3 + 2
    assert seq.last_ok


def test_pick_top_of_stack_goes_one_level_higher():
    """2층 큐브 집기 (2026-11-13): 집는 높이 = 180 + 큐브 40, 접근·들기 높이도 같이 40 mm 올라간다."""
    link = FakeLink(grip_close_pos=500)
    seq = PickSequence(link, log=lambda m: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, seq.level_lift(2))
    assert ('move', 420, 30, 220, '집는 높이로 하강') in link.calls
    assert link.calls[-1][3] == 220 + APPROACH_MM
    assert seq.level_lift(0) == 0 and seq.level_lift(1) == 0 and seq.level_lift(3) == 80


def test_stack_on_two_level_base_places_higher_and_travels_higher():
    """2층 받침 위에 쌓기: 놓는 높이 = 180 + 2 x 40 + 여유, 이동 높이도 40 mm 더 높게 (들고 가는 큐브가 닿지 않게)."""
    link = FakeLink(grip_close_pos=500)
    seq = PickSequence(link, log=lambda m: None)
    run(seq, seq.pick_and_stack, 420.0, 30.0, 180.0, 300.0, -150.0, 2)
    assert ('move', 420, 30, 180, '집는 높이로 하강') in link.calls             # 집는 큐브는 1층
    assert ('move', 300, -150, 265, '놓는 높이로 하강') in link.calls          # 180 + 80 + 3 + 2
    assert ('move', 420, 30, 180 + APPROACH_MM + 40, '들어 올리기') in link.calls
    assert seq.last_ok


def test_second_start_is_refused_while_running():
    """이미 순서가 진행 중이면 새 순서를 거부한다."""
    link = FakeLink()
    seq = PickSequence(link, lambda s: None)
    seq.running = True
    assert not seq.hover(400.0, 0.0, 180.0)


class StopOnceLink(FakeLink):
    """stop_at 번째 하강(집는 높이) 이동에서 한 번 「정지」 된 것처럼 실패한다 (그 뒤로는 정상)."""

    def __init__(self, seq_holder, stop_label='집는 높이로 하강', **kw):
        """seq_holder: [PickSequence] - 정지 때 request_stop() 을 부르기 위해."""
        super().__init__(**kw)
        self.holder, self.stop_label, self.stopped = seq_holder, stop_label, False

    def move_tcp_to(self, x, y, z, vel, label=None):
        """처음 만나는 stop_label 이동에서 정지 버튼이 눌린 것처럼: request_stop + 실패."""
        if label == self.stop_label and not self.stopped:
            self.stopped = True
            self.holder[0].request_stop()
            self.calls.append(('stopped', label))
            self.last_ok = False
            return False
        return super().move_tcp_to(x, y, z, vel, label)


def test_resume_continues_from_stopped_step():
    """「멈춰」 로 하강 중 정지 -> 「계속해」 는 하강부터 이어서 (그리퍼 열기·접근을 다시 하지 않음)."""
    holder = [None]
    link = StopOnceLink(holder, grip_close_pos=500)
    seq = PickSequence(link, lambda s: None)
    holder[0] = seq
    run(seq, seq.pick, 420.0, 30.0, 180.0)
    assert not seq.last_ok and seq.resumable is not None
    n_before = len(link.calls)
    run(seq, seq.resume)
    after = [c[0] for c in link.calls[n_before:]]
    assert after == ['move', 'grip', 'move']           # 하강 -> 닫기 -> 들기 (열기·접근 없음)
    assert link.calls[n_before][3] == 180             # 첫 동작이 집는 높이로 하강
    assert seq.last_ok and seq.holding is True
    assert seq.name == '집기 (이어서)' and seq.resumable is None


def test_failure_is_not_resumable():
    """정지가 아니라 실패(빈손)로 끊긴 순서는 이을 수 없다."""
    link = FakeLink(grip_close_pos=740)
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick_and_place, 420.0, 30.0, 180.0, 300.0, -250.0)
    assert seq.resumable is None and not seq.resume()


def test_other_motion_forgets_resume():
    """정지 뒤 다른 동작(홈 등)을 하면 이을 순서를 버린다 - 바뀐 위치에서 「하강」 을 잇지 않게."""
    holder = [None]
    link = StopOnceLink(holder)
    seq = PickSequence(link, lambda s: None)
    holder[0] = seq
    run(seq, seq.pick, 420.0, 30.0, 180.0)
    assert seq.resumable is not None
    seq.forget_resume('홈 이동')
    assert not seq.resume()


# ---------------------------------------------------------------- 그리퍼 각도 맞추기 (2026-11-13)
HOME_POSX = [373.0, 0.0, 405.0, 130.0, -180.0, 130.0]     # 실물 홈 자세 TCP (2026-11-13 실측): 손끝이 아래를 봄


class TurnLink(FakeLink):
    """J6 조그도 되는 가짜 링크: 손끝이 아래를 볼 때 J6 를 d 돌리면 ZYZ 의 C 가 d 늘어난다 (툴 Z 축 회전)."""

    def __init__(self, grip_close_pos=500):
        """홈 자세(그리퍼 yaw 180°)에서 시작, J6 = 0."""
        super().__init__(grip_close_pos)
        self.posx = list(HOME_POSX)
        self.joints_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

    def jog_joint(self, idx, delta, vel):
        """관절 조그 기록 + J6 이면 TCP 자세도 같이 돈다. 한 번 10° 넘으면 RobotLink 처럼 잘린다고 보고 실패."""
        self.calls.append(('jog', idx, round(delta, 1)))
        if abs(delta) > 10.0 + 1e-6:
            return False
        self.joints_deg[idx] += delta
        if idx == 5:
            self.posx[5] += delta
        self.last_ok = True
        return True


def test_wrap90_folds_square_symmetry():
    """90° 마다 같은 모양: 어떤 각도든 [-45, 45) 로 접힌다."""
    assert wrap90(0.0) == pytest.approx(0.0)
    assert wrap90(100.0) == pytest.approx(10.0)
    assert wrap90(-50.0) == pytest.approx(40.0)
    assert wrap90(180.0) == pytest.approx(0.0)


def test_tool_yaw_at_home_points_down_along_minus_x():
    """실물 홈 자세 (A 130, B -180, C 130): 툴 Z 는 아래, 그리퍼 면 방향(툴 X)은 베이스 -X (180°)."""
    _, z = tool_axes(HOME_POSX)
    assert z[2] == pytest.approx(-1.0)
    assert wrap90(tool_yaw_deg(HOME_POSX)) == pytest.approx(0.0, abs=1e-6)
    assert abs(tool_yaw_deg(HOME_POSX)) == pytest.approx(180.0, abs=1e-6)


def test_plan_j6_turns_least_and_sign_follows_tool_down():
    """손끝이 아래(툴 Z·베이스 Z = -1) 면 yaw +30° 에 J6 -30°. 70° 어긋남은 반대로 20° 만 돈다 (90° 대칭)."""
    assert plan_j6(0.0, 30.0, -1.0, 0.0) == pytest.approx(-30.0)
    assert plan_j6(0.0, 70.0, -1.0, 0.0) == pytest.approx(20.0)
    assert plan_j6(10.0, 10.5, -1.0, 0.0) == pytest.approx(-0.5)


def test_plan_j6_unwinds_when_wrist_far_turned():
    """J6 가 이미 170° 감겨 있으면, 더 감는(-30 -> 200°) 대신 90° 다른 같은 모양 쪽(+60 -> 110°)을 고른다."""
    d = plan_j6(0.0, -30.0, -1.0, 170.0)
    assert abs(170.0 + d) <= 180.0
    assert wrap90(-d - (-30.0)) == pytest.approx(0.0, abs=1e-6)     # 결과 yaw 는 목표와 90° 대칭으로 같음


def test_calibration_angle_follows_affine_map():
    """영상 방향 -> 로봇 방향: 정답 변환(affine)의 그 방향 벡터 각도와 같다 (90° 대칭)."""
    c = make_calib()
    for ang in (0.0, 20.0, 45.0, 80.0):
        t = math.radians(ang)
        ex = 0.8 * math.cos(t) - 0.1 * math.sin(t)           # affine 의 u, v 계수
        ey = 0.05 * math.cos(t) + 0.75 * math.sin(t)
        assert c.angle_to_robot(300, 250, ang) == pytest.approx(wrap90(math.degrees(math.atan2(ey, ex))), abs=1e-6)
    assert PixelCalibration().angle_to_robot(1, 1, 0) is None


def test_pick_with_yaw_turns_wrist_before_opening():
    """집기 + 큐브 30°: 큐브 위 접근 높이에서 J6 -30° 를 10° 씩 3번 돌린 뒤 열고 내려간다. 그리퍼가 큐브 면에 맞음."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, 30.0)
    kinds = [c[0] for c in link.calls]
    assert kinds == ['move', 'jog', 'jog', 'jog', 'grip', 'move', 'grip', 'move']
    assert all(c == ('jog', 5, -10.0) for c in link.calls[1:4])
    assert wrap90(tool_yaw_deg(link.posx) - 30.0) == pytest.approx(0.0, abs=1e-6)
    assert seq.last_ok


def test_pick_and_place_aligns_with_arm_direction():
    """옮기기: 큐브 면에 맞춰 집고, 놓을 자리 (300, 300) 위에서 팔 방향 45° 와 평행·수직이 되게 다시 돌린 뒤 내려놓는다."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick_and_place, 420.0, 30.0, 180.0, 300.0, 300.0, 0.0, '옮기기', 0.0, 10.0, wrap90(45.0))
    labels = [c for c in link.calls]
    i_travel = next(i for i, c in enumerate(labels) if c[0] == 'move' and c[4] == '놓을 자리 위로')
    i_down = next(i for i, c in enumerate(labels) if c[0] == 'move' and c[4] == '놓는 높이로 하강')
    assert any(c[0] == 'jog' for c in labels[i_travel:i_down])          # 놓기 전 손목 회전
    assert wrap90(tool_yaw_deg(link.posx) - 45.0) == pytest.approx(0.0, abs=1e-6)
    assert seq.last_ok


def test_align_skipped_when_tool_not_down():
    """툴이 아래를 보지 않으면(예전 시험 자세 B = 0) 돌리지 않고 예전 순서 그대로 집는다."""
    link = TurnLink()
    link.posx = [400.0, 0.0, 300.0, 180.0, 0.0, 90.0]
    logs = []
    seq = PickSequence(link, logs.append)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, 30.0)
    assert not any(c[0] == 'jog' for c in link.calls)
    assert any('아래를 보지 않음' in m for m in logs) and seq.last_ok


def test_align_skipped_when_already_aligned():
    """1° 만 어긋나면(ALIGN_MIN_DEG 2° 미만) 손목을 돌리지 않는다."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, 1.0)
    assert not any(c[0] == 'jog' for c in link.calls) and seq.last_ok


# ---------------------------------------------------------------- 큐브 중심 맞추기 (2026-11-13)
def test_pick_centers_on_cube_before_descending():
    """중심 맞추기: 다시 잰 중심이 (+4, -3) mm 어긋나면 접근 높이에서 그 위로 옮긴 뒤, 고친 자리로 하강·들기."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, None, lambda: (424.0, 27.0))
    moves = [c for c in link.calls if c[0] == 'move']
    assert moves[0][1:4] == (420, 30, 180 + APPROACH_MM)                  # 처음 좌표 위
    assert moves[1] == ('move', 424, 27, 180 + APPROACH_MM, '큐브 중심 맞추기')
    assert ('move', 424, 27, 180, '집는 높이로 하강') in link.calls
    assert ('move', 424, 27, 180 + APPROACH_MM, '들어 올리기') in link.calls
    assert seq.last_ok


def test_center_order_after_wrist_turn():
    """순서: 접근 -> 각도 맞추기(J6) -> 중심 맞추기 -> 열기 -> 하강."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, 30.0, lambda: (425.0, 30.0))
    kinds = [c[0] if c[0] != 'move' else c[4] for c in link.calls]
    assert kinds[:6] == ['큐브 위 접근 높이로', 'jog', 'jog', 'jog', '큐브 중심 맞추기', 'grip']


def test_center_skipped_when_cube_not_seen():
    """다시 재지 못하면(그리퍼에 가려짐) 옮기지 않고 처음 좌표로 집는다."""
    link = TurnLink()
    logs = []
    seq = PickSequence(link, logs.append)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, None, lambda: '큐브가 잘 보이지 않음')
    assert ('move', 420, 30, 180, '집는 높이로 하강') in link.calls
    assert any('중심 맞추기 건너뜀' in m for m in logs) and seq.last_ok


def test_center_small_offset_does_not_move():
    """1 mm 어긋남(CENTER_MIN_MM 1.5 미만)은 옮기지 않는다."""
    link = TurnLink()
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick, 420.0, 30.0, 180.0, 0.0, None, lambda: (421.0, 30.0))
    assert not any(c[0] == 'move' and c[4] == '큐브 중심 맞추기' for c in link.calls)
    assert ('move', 420, 30, 180, '집는 높이로 하강') in link.calls


def test_center_stops_when_cube_moved_far():
    """30 mm 어긋남(CENTER_MAX_MM 25 초과) = 큐브가 움직임: 열지도 내려가지도 않고 멈춘다."""
    link = TurnLink()
    logs = []
    seq = PickSequence(link, logs.append)
    run(seq, seq.pick_and_place, 420.0, 30.0, 180.0, 300.0, 300.0, 0.0, '옮기기', 0.0, None, None,
        lambda: (450.0, 30.0))
    assert not any(c[0] == 'grip' for c in link.calls)
    assert not any(c[0] == 'move' and c[3] == 180 for c in link.calls)
    assert not seq.last_ok and seq.step == '큐브 중심 맞추기'
    assert any('움직였음' in m for m in logs)



# ---------------------------------------------------------------- 블록 자동 보정점 (2026-11-14)
class HomeLink(TurnLink):
    """TurnLink + 홈 이동 (기록만)."""

    def move_home(self, vel):
        """홈 이동 기록 후 바로 도착."""
        self.calls.append(('home',))
        self.last_ok = True
        return True


def test_block_calibration_point_centers_twice_then_records_and_measures():
    """블록 자동 보정점: 열기 -> 하강 -> 닫기·열기 -> 올려 손목 90° -> 하강 -> 닫기 -> 잡음 확인 -> 기록 -> 열기 -> 올리기
    -> 홈 -> 마커 재기. 기록은 두 번째로 닫았을 때(블록이 가운데)의 TCP."""
    link = HomeLink(grip_close_pos=500)
    seq = PickSequence(link, lambda s: None)
    got = {}
    run(seq, seq.block_calibration_point, 500.0, 40.0, 180.0, None,
        lambda: got.setdefault('tcp', list(link.posx[:3])) and True, lambda: got.setdefault('measured', True))
    kinds = [c[0] for c in link.calls]
    assert kinds == ['move', 'grip', 'move', 'grip', 'grip', 'move'] + ['jog'] * 9 + \
        ['move', 'grip', 'grip', 'move', 'home']
    assert sum(c[2] for c in link.calls if c[0] == 'jog') == pytest.approx(90.0)
    assert got['tcp'] == [500.0, 40.0, 180.0] and got['measured'] and seq.last_ok
    assert seq.holding is False                      # 블록을 놓고 끝났으므로 빈손 (잡음 표시가 남으면 다음 기록이 거부됨)


def test_block_calibration_point_stops_when_empty():
    """두 번째로 닫아도 빈손(끝까지 닫힘)이면 기록·홈·마커 재기 없이 멈춘다."""
    link = HomeLink(grip_close_pos=740)
    seq = PickSequence(link, lambda s: None)
    rec = []
    run(seq, seq.block_calibration_point, 500.0, 40.0, 180.0, None, lambda: rec.append(1) or True,
        lambda: rec.append(2) or True)
    assert rec == [] and not seq.last_ok and seq.step == '잡았는지 확인'
    assert ('home',) not in link.calls


# ---------------------------------------------------------------- 잡은 상태 판단 (2026-11-14)
def test_judge_hold_from_gripper_reply():
    """그리퍼 응답 -> 잡음/빈손/모름: 응답 없음 = 모름, 토크 꺼짐·열림·끝까지 닫힘 = 빈손, 중간에 멈춤 = 잡음."""
    assert judge_hold(None)[0] is None
    assert judge_hold({'position': 519, 'torque': False})[0] is False
    assert judge_hold({'position': 1, 'torque': True})[0] is False
    assert judge_hold({'position': 519, 'torque': True})[0] is True
    assert judge_hold({'position': 739, 'torque': True})[0] is False
    assert '잡음' in judge_hold({'position': 519, 'torque': True})[1]


def test_place_after_pick_ends_empty_handed():
    """집어서 옮기기: 잡았다가 놓으며 그리퍼를 열면 빈손으로 바뀐다 (잡음 표시가 남지 않음)."""
    link = FakeLink(grip_close_pos=500)
    seq = PickSequence(link, lambda s: None)
    run(seq, seq.pick_and_place, 420.0, 30.0, 180.0, 300.0, 300.0)
    assert seq.last_ok and seq.holding is False


def test_update_hold_reads_last_gripper_reply():
    """update_hold: 그리퍼의 마지막 응답으로 다시 판단하고 이유를 로그로 남긴다."""
    link = FakeLink()
    logs = []
    seq = PickSequence(link, logs.append)
    assert seq.update_hold() is None and '모름' in logs[-1]
    link.gripper = {'position': 519, 'torque': True}
    assert seq.update_hold() is True and seq.holding is True
