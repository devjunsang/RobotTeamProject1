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
