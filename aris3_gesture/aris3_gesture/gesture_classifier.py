"""MediaPipe 손 관절 21점 -> 손짓 (point / fist / open).  2026-11-09

ROS·MediaPipe 에 의존하지 않는 순수 함수라서 단위 시험(test/test_gesture_classifier.py)에서 바로 쓴다.

관절 번호 (MediaPipe Hand Landmarker):
  0 손목 / 엄지 1~4 / 검지 5(MCP) 6(PIP) 7 8(끝) / 중지 9 10 11 12 / 약지 13 14 15 16 / 새끼 17 18 19 20

손가락이 펴졌는가 = 손끝이 손목에서 PIP 관절보다 충분히 멀다.
위에서 내려다보는 카메라라 손이 어느 방향을 향해도 같은 기준이 되도록 「위·아래」 대신 손목까지 거리를 쓴다.
엄지는 접어도 옆으로 벌어지는 일이 많아 판단에 쓰지 않는다.

  point : 검지만 폄 (중지·약지·새끼 접음)   -> 가리키기 (검지 끝이 포인터)
  open  : 네 손가락 모두 폄                  -> 펼친 손 = 「놓기」 확정
  fist  : 네 손가락 모두 접음                -> 주먹 = 「집기」 확정
  ''    : 그 밖 (V 자, 반쯤 접음 ...) -> 아무것도 하지 않음
"""
import math
from collections import Counter, deque

WRIST = 0
FINGERS = {             # 이름: (PIP, 끝)
    'index': (6, 8),
    'middle': (10, 12),
    'ring': (14, 16),
    'pinky': (18, 20),
}
EXTEND_RATIO = 1.15     # 끝-손목 거리 > PIP-손목 거리 x 이 값 -> 폄
FOLD_RATIO = 1.0        # 끝-손목 거리 < PIP-손목 거리 x 이 값 -> 접음 (사이는 모름)


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finger_states(pts):
    """pts: 21 개 (x, y) -> {손가락: True 폄 / False 접음 / None 애매}."""
    w = pts[WRIST]
    out = {}
    for name, (pip, tip) in FINGERS.items():
        d_pip = _dist(pts[pip], w)
        d_tip = _dist(pts[tip], w)
        if d_pip <= 1e-6:
            out[name] = None
        elif d_tip > d_pip * EXTEND_RATIO:
            out[name] = True
        elif d_tip < d_pip * FOLD_RATIO:
            out[name] = False
        else:
            out[name] = None
    return out


def classify(pts):
    """pts: 21 개 (x, y) (픽셀이든 0~1 정규화든 가로세로 비율만 같으면 됨) -> 'point' | 'open' | 'fist' | ''."""
    if pts is None or len(pts) < 21:
        return ''
    s = finger_states(pts)
    others = (s['middle'], s['ring'], s['pinky'])
    if s['index'] is True and all(v is False for v in others):
        return 'point'
    if s['index'] is True and all(v is True for v in others):
        return 'open'
    if s['index'] is False and all(v is False for v in others):
        return 'fist'
    return ''


class GestureSmoother:
    """최근 n 장의 다수결. 과반(min_votes)을 넘는 손짓만 인정해 한두 장의 잘못된 판단을 걸러 낸다."""

    def __init__(self, n=7, min_votes=5):
        self.n, self.min_votes = n, min_votes
        self.hist = deque(maxlen=n)

    def reset(self):
        self.hist.clear()

    def update(self, gesture):
        self.hist.append(gesture)
        best, votes = Counter(self.hist).most_common(1)[0]
        return best if best and votes >= self.min_votes else ''
