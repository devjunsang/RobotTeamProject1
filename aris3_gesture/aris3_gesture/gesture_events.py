"""손짓 -> 확정 이벤트 판단 (2026-11-10, aris3_gui main_window._gesture_tick 에서 옮김).

예전에는 GUI 가 100 ms 마다 이 판단을 했다. C++ GUI(ARIS)와 PyQt GUI 가 같은 판단을 쓰도록 노드로 옮긴다.
ROS·MediaPipe 에 의존하지 않는 순수 클래스라서 단위 시험(test/test_gesture_events.py)에서 바로 쓴다.

  SELECT        : 가리킨(point) 채 hold_sec 동안 hold_radius_px 안에 머묾 -> 머물기 시작한 곳을 한 번 고름
                  (같은 자리에서 계속 가리켜도 다시 내지 않는다. 손가락을 반경 밖으로 옮기면 새로 셈)
  CONFIRM_PICK  : 가리킴 -> 주먹 으로 바뀐 순간 한 번
  CONFIRM_PLACE : 가리킴 -> 펼침 으로 바뀐 순간 한 번
  손이 처음부터 주먹/펼침이면 아무것도 내지 않는다 (실수로 확정되지 않게).
"""
import math

SELECT, CONFIRM_PICK, CONFIRM_PLACE = 'select', 'confirm_pick', 'confirm_place'


class GestureTracker:
    """영상마다 update() 를 부르면 (hold_progress, 이벤트 목록) 을 돌려준다."""

    def __init__(self, hold_sec=1.0, hold_radius_px=25.0):
        """hold_sec: 선택까지 가리키고 멈출 시간 [s], hold_radius_px: 멈춤으로 볼 손끝 이동 반경 [px]."""
        self.hold_sec = float(hold_sec)
        self.hold_radius = float(hold_radius_px)
        self.reset()

    def reset(self):
        """손이 사라졌을 때: 멈춤 시계와 마지막 손짓을 지운다."""
        self.hold = None        # (u, v, 시작 시각) - 가리키고 멈추기 시작한 곳
        self.fired = False      # 이번 멈춤에서 이미 SELECT 를 냈음
        self.last = ''          # 마지막으로 인정된 손짓 (point -> fist/open 전환을 본다)

    def update(self, now, hand_present, gesture='', u=0.0, v=0.0):
        """영상 한 장의 판단을 넣는다.

        now: 시각 [s] (단조 증가), gesture: 다수결 손짓 ('point' | 'fist' | 'open' | ''), (u, v): 검지 끝 픽셀.
        돌려줌: (hold_progress 0~1, [(이벤트 종류, u, v), ...]).
        """
        if not hand_present:
            self.reset()
            return 0.0, []
        events = []
        progress = 0.0
        if gesture == 'point':
            # 반경 밖으로 움직였으면 그 자리에서 새로 센다
            if self.hold is None or math.hypot(u - self.hold[0], v - self.hold[1]) > self.hold_radius:
                self.hold, self.fired = (u, v, now), False
            prog = min(1.0, (now - self.hold[2]) / self.hold_sec) if self.hold_sec > 0 else 1.0
            progress = 0.0 if self.fired else prog          # 고른 뒤에는 진행 링을 지운다
            if prog >= 1.0 and not self.fired:
                self.fired = True
                events.append((SELECT, self.hold[0], self.hold[1]))
        else:
            self.hold = None
        # 확정: 가리키다가 주먹/펼침으로 바뀐 순간 한 번만
        if gesture in ('fist', 'open') and self.last == 'point':
            events.append((CONFIRM_PICK if gesture == 'fist' else CONFIRM_PLACE, u, v))
        if gesture:                                          # '' (애매) 는 마지막 손짓을 바꾸지 않는다
            self.last = gesture
        return progress, events
