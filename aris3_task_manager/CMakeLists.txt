cmake_minimum_required(VERSION 3.8)
project(aris3_task_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aris3_interfaces REQUIRED)
find_package(dsr_msgs2 REQUIRED)

# 1) ROS 에 의존하지 않는 순수 로직 → 단위 시험에서 바로 링크한다.
add_library(task_logic STATIC src/geometry.cpp src/target_estimator.cpp)
target_include_directories(task_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(task_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 2) 노드 실행 파일
add_executable(task_manager
  src/main.cpp src/task_manager_node.cpp src/robot_client.cpp src/gripper_client.cpp)
target_link_libraries(task_manager task_logic)
ament_target_dependencies(task_manager
  rclcpp rclcpp_action geometry_msgs tf2 tf2_ros tf2_geometry_msgs aris3_interfaces dsr_msgs2)

# 3) 명령줄 액션 클라이언트
add_executable(send_task src/send_task.cpp)
ament_target_dependencies(send_task rclcpp rclcpp_action aris3_interfaces)

install(TARGETS task_manager send_task DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry task_logic)
  ament_add_gtest(test_target_estimator test/test_target_estimator.cpp)
  target_link_libraries(test_target_estimator task_logic)
endif()

ament_package()
