# 비전이 찾은 물체 하나 (계약: /aris3/detected_objects 의 원소)
#
# pose.header.frame_id 는 반드시 채운다 (예: "world"). 좌표계가 없는 좌표는 쓸 수 없다.
# pose.header.stamp 는 영상을 찍은 시각이다 (처리한 시각이 아님).

string id                         # 물체를 구분할 수 있으면 이름 (예: "red_block"), 모르면 ""
string color                      # "red" | "green" | "blue" ...
string shape                      # "cube" | "circle" | "triangle" | "square" | "" (모름)
geometry_msgs/PoseStamped pose    # 물체 중심 위치 (m)
float32 confidence                # 0.0 ~ 1.0 (비전 모듈이 정하는 신뢰도)
float32 size_px                   # 영상에서 물체가 차지한 넓이 [pixel] (진단용)
float32 height_mm                 # 탁자에서 물체 윗면까지 높이 [mm] (깊이 카메라, 2026-11-13). 0 = 모름
uint8 level                       # 쌓인 층: 1 = 탁자 위, 2 = 큐브 하나 위 ... 0 = 모름 (깊이 없음·시뮬레이션)
# 집기용 픽셀 (2026-11-13, frame_id = overhead_camera_image_px 인 픽셀 검출만): 쌓인 큐브 윗면은 보정 높이(1층 윗면)보다
# 높아서 원근 때문에 영상에서 밀려 보인다. 깊이로 잰 탁자 평면을 따라 윗면 중심을 1층 높이로 내린 뒤 다시 영상에
# 투영한 픽셀 = 카메라 보정(1층 높이 기준)에 넣을 픽셀. 1층·층 모름이면 pose.position 과 같다. 0, 0 = 채우지 않음.
float32 grasp_u
float32 grasp_v
