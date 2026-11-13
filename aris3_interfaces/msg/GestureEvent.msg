# 손짓 확정 이벤트 (토픽: /aris3/gesture/event, aris3_gesture -> GUI)  2026-11-10
# HandGesture(매 영상)에서 「가리키고 멈추기」 와 「가리킴 -> 주먹/펼침 전환」 을 판단해 한 번씩 낸다.
# 예전에는 GUI 안에서 판단했다 (aris3_gui _gesture_tick). 두 GUI 가 같은 판단을 쓰도록 노드로 옮긴다.
# 로봇을 움직이는 명령이 아니다: GUI 가 확인창 + 조작 허용을 거쳐 실행한다 (확인창이 떠 있으면 GUI 가 무시).

uint8 SELECT=0                    # 가리킨 채 hold_sec 동안 hold_radius_px 안에 멈춤 -> (u, v) 를 고름 (큐브 또는 놓을 자리)
uint8 CONFIRM_PICK=1              # 가리킴 -> 주먹: 고른 큐브 집기
uint8 CONFIRM_PLACE=2             # 가리킴 -> 펼침: 고른 자리에 놓기

builtin_interfaces/Time stamp
uint8 type
float32 u                         # SELECT: 멈추기 시작한 곳의 손끝 픽셀. CONFIRM_*: 그 순간의 손끝 픽셀
float32 v
