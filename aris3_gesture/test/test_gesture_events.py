"""GestureTracker 단위 시험: 예전 GUI _gesture_tick 과 같은 판단을 하는지."""
from aris3_gesture.gesture_events import CONFIRM_PICK, CONFIRM_PLACE, SELECT, GestureTracker


def feed(tr, frames):
    """(시각, 손짓, u, v) 목록을 차례로 넣고 모든 이벤트와 마지막 진행도를 돌려준다."""
    events, prog = [], 0.0
    for t, g, u, v in frames:
        prog, ev = tr.update(t, g is not None, g or '', u, v)
        events += ev
    return events, prog


def test_select_after_hold_once():
    """1 초 동안 같은 자리를 가리키면 시작한 곳에서 SELECT 한 번, 더 가리켜도 다시 내지 않는다."""
    tr = GestureTracker(1.0, 25)
    frames = [(i * 0.1, 'point', 100 + (i % 3), 200) for i in range(25)]   # 2.4 초, 손끝 2 px 흔들림
    events, prog = feed(tr, frames)
    assert events == [(SELECT, 100, 200)]
    assert prog == 0.0                                  # 고른 뒤 진행 링은 0


def test_progress_grows_then_resets_on_move():
    """멈춰 있는 동안 진행도가 커지고, 반경 밖으로 움직이면 0 부터 다시 센다."""
    tr = GestureTracker(1.0, 25)
    _, p1 = feed(tr, [(0.0, 'point', 100, 100), (0.5, 'point', 105, 100)])
    assert 0.45 < p1 < 0.55
    _, p2 = feed(tr, [(0.6, 'point', 200, 100)])        # 95 px 이동
    assert p2 == 0.0


def test_fist_after_point_confirms_pick_once():
    """가리킴 -> 주먹 으로 바뀐 순간 CONFIRM_PICK 한 번 (주먹을 계속 쥐어도 한 번)."""
    tr = GestureTracker(1.0, 25)
    events, _ = feed(tr, [(0.0, 'point', 10, 10), (0.1, 'fist', 12, 12), (0.2, 'fist', 12, 12)])
    assert events == [(CONFIRM_PICK, 12, 12)]


def test_open_after_point_confirms_place():
    """가리킴 -> 펼침 은 CONFIRM_PLACE."""
    tr = GestureTracker(1.0, 25)
    events, _ = feed(tr, [(0.0, 'point', 10, 10), (0.1, 'open', 10, 10)])
    assert events == [(CONFIRM_PLACE, 10, 10)]


def test_fist_without_point_does_nothing():
    """처음부터 주먹이거나, 손이 사라졌다 나타나 주먹이면 확정하지 않는다."""
    tr = GestureTracker(1.0, 25)
    events, _ = feed(tr, [(0.0, 'fist', 0, 0), (0.1, 'point', 0, 0), (0.2, None, 0, 0), (0.3, 'fist', 0, 0)])
    assert events == []


def test_ambiguous_frame_keeps_last_gesture():
    """가리킴과 주먹 사이에 애매한 판단('')이 끼어도 전환으로 인정한다 (예전 GUI 와 같음)."""
    tr = GestureTracker(1.0, 25)
    events, _ = feed(tr, [(0.0, 'point', 5, 5), (0.1, '', 5, 5), (0.2, 'fist', 5, 5)])
    assert events == [(CONFIRM_PICK, 5, 5)]
