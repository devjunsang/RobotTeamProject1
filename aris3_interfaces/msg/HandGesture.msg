# 손 하나의 손짓 인식 결과 (토픽: /aris3/gesture, aris3_gesture -> GUI)  2026-11-09
# 로봇을 움직이는 명령이 아니다. GUI 가 「가리키고 멈추기 -> 확정 손짓」 두 단계와 확인창을 거쳐 실행한다.

builtin_interfaces/Time stamp     # 인식한 영상의 시각
bool hand_present                 # 손이 보이는가 (아래 값은 true 일 때만 의미 있음)
string gesture                    # "point" | "fist" | "open" | "" (모름) - 최근 몇 장의 다수결
string raw_gesture                # 이 영상 한 장만 본 판단 (진단용)
float32 tip_u                     # 검지 끝 영상 픽셀 (u 오른쪽, v 아래) - 입력 영상 해상도 기준
float32 tip_v
uint32 image_width
uint32 image_height
float32 confidence                # MediaPipe 손 판별 점수 0~1
string handedness                 # "Left" | "Right" (MediaPipe 판단, 위에서 본 영상이라 뒤바뀔 수 있음)
# --- 2026-11-10 추가: 「가리키고 멈추기」 진행 (선택 판단을 노드로 옮기면서, GUI 가 포인터 링을 그리는 데 씀)
float32 hold_progress             # 0~1, 가리킨 채 멈춘 시간 / hold_sec. 1 에 닿으면 GestureEvent SELECT 를 한 번 냄
