# 백업: 탁자 배치 적용 전 (2026-11-08)

시뮬레이션에서 로봇을 바닥(z=0)에서 탁자 위(z=0.75 m)로 옮기기 전의 원본 파일.
폴더 구조는 저장소의 원래 경로와 같다. 되돌리려면 같은 경로로 덮어쓴다.

| 원본 파일 | 바뀐 내용 |
|---|---|
| e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro | `base_z` 인자 추가 (기본 0.0, world_fixed 높이) |
| aris3_bringup/worlds/aris3_world.sdf | `workcell_table` 탁자 추가 (윗면 0.75 m, 1.4 x 1.4 m) |
| aris3_bringup/launch/simulation.launch.py | `TABLE_TOP_Z`, `base_z:=0.75` 전달, 카메라 높이 2.0 -> 2.75 m |
| aris3_bringup/config/object_layout.csv | 물체 z 0.04 -> 탁자 위 (이후 큐브 3개 배치로 변경) |
| aris3_bringup/scripts/spawn_objects.py | 정육면체 4개(빨·파·노·초, 8 cm) -> 빨·파·노 정육면체 3개(5 cm). 원·사각·삼각 기둥 생성 지원, 마찰 1.0, yaw 적용 |
| aris3_bringup/scripts/object_marker_publisher.py | RViz 마커를 도형별로 (CYLINDER / CUBE / 삼각 기둥) |
| aris3_bringup/scripts/start_controllers.py | 팔 컨트롤러를 arm_controller(궤적)로 활성화, dsr_position_controller 는 inactive |

같은 날 추가된 파일(백업 대상 아님): aris3_bringup/config/gz_controllers.yaml, aris3_moveit_config/
