cmake_minimum_required(VERSION 3.8)
project(aris3_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


install(
  PROGRAMS scripts/start_controllers.py scripts/wait_for_action.py
  DESTINATION lib/${PROJECT_NAME}
)

# 실물 비슷한 시뮬레이션 준비 (aris3_real_sim.launch.py, 2026-11-10)
install(
  PROGRAMS scripts/real_sim_setup.py
  DESTINATION lib/${PROJECT_NAME}
)


# PHASE 3: Gazebo 객체 자동 생성 스크립트 설치
install(
  PROGRAMS scripts/spawn_objects.py
  DESTINATION lib/${PROJECT_NAME}
)


# PHASE 3 / STEP 03
# Gazebo 카메라 모델 및 커스텀 월드 설치
install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# PHASE 3 / STEP 04 - RViz2 객체 시각화 노드 설치
install(
  PROGRAMS scripts/object_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# ARIS3 STEP 04: RViz2 화면 설정 파일 설치
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
