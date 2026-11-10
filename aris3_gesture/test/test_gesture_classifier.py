"""손짓 분류 단위 시험: 손 모양을 관절 좌표로 직접 만들어 넣는다 (MediaPipe 불필요)."""
import math

from aris3_gesture.gesture_classifier import GestureSmoother, classify, finger_states

# 손바닥이 +y 방향(영상 아래쪽)을 향해 손가락을 뻗는 손. 손목 (0, 0).
BASES = {'thumb': -30, 'index': -15, 'middle': 0, 'ring': 15, 'pinky': 30}   # 손가락 뿌리 x


def hand(extended, angle_deg=0.0, scale=1.0, offset=(320, 240)):
    """extended: 편 손가락 이름 집합 -> 21 점 (x, y). angle 만큼 돌리고 scale 배, offset 으로 옮긴다."""
    pts = [(0.0, 0.0)] * 21
    pts[0] = (0.0, 0.0)
    idx = {'thumb': (1, 2, 3, 4), 'index': (5, 6, 7, 8), 'middle': (9, 10, 11, 12),
           'ring': (13, 14, 15, 16), 'pinky': (17, 18, 19, 20)}
    for name, (mcp, pip, dip, tip) in idx.items():
        x = BASES[name]
        pts[mcp] = (x, 60.0)
        pts[pip] = (x, 90.0)
        if name in extended:
            pts[dip], pts[tip] = (x, 115.0), (x, 140.0)
        else:                                   # 접으면 끝이 손바닥 쪽으로 돌아온다
            pts[dip], pts[tip] = (x, 75.0), (x, 55.0)
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return [(offset[0] + scale * (x * c - y * s), offset[1] + scale * (x * s + y * c)) for x, y in pts]


ALL4 = {'index', 'middle', 'ring', 'pinky'}


def test_point():
    assert classify(hand({'index'})) == 'point'


def test_point_with_thumb_out_still_point():
    assert classify(hand({'index', 'thumb'})) == 'point'


def test_open():
    assert classify(hand(ALL4 | {'thumb'})) == 'open'


def test_fist():
    assert classify(hand(set())) == 'fist'


def test_v_sign_is_unknown():
    assert classify(hand({'index', 'middle'})) == ''


def test_rotation_and_scale_invariant():
    for ang in (0, 45, 90, 180, 270):
        for sc in (0.3, 1.0, 2.5):
            assert classify(hand({'index'}, ang, sc)) == 'point'
            assert classify(hand(ALL4, ang, sc)) == 'open'
            assert classify(hand(set(), ang, sc)) == 'fist'


def test_normalized_coordinates():
    pts = [(x / 640, y / 640) for x, y in hand({'index'})]
    assert classify(pts) == 'point'


def test_finger_states():
    s = finger_states(hand({'index', 'ring'}))
    assert s == {'index': True, 'middle': False, 'ring': True, 'pinky': False}


def test_missing_landmarks():
    assert classify(None) == ''
    assert classify([(0, 0)] * 5) == ''


def test_smoother_needs_majority():
    sm = GestureSmoother(n=7, min_votes=5)
    out = [sm.update(g) for g in ['point'] * 4]
    assert out[-1] == ''                        # 아직 4 표
    assert sm.update('point') == 'point'        # 5 표
    assert sm.update('fist') == 'point'         # 한 장 튀어도 유지
    assert sm.update('fist') == 'point'
    for _ in range(3):
        last = sm.update('fist')
    assert last == 'fist'


def test_smoother_unknown_never_wins():
    sm = GestureSmoother(n=5, min_votes=3)
    for _ in range(5):
        assert sm.update('') == ''
