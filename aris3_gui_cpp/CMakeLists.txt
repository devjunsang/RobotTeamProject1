cmake_minimum_required(VERSION 3.8)
project(aris3_gui_cpp)

# ARIS - ARIS3 Control Station C++ Qt5 버전.
# 구조 (aris3_gui PyQt 와 같은 생각):
#   gui_logic   : Qt·ROS 에 의존하지 않는 순수 로직 (수신 주기, 로그 거르기, 작업 단계 추적) -> gtest
#   aris        : 실행 파일. ROS 노드(GuiNode)는 별도 스레드에서 돌고, Qt 화면은 100 ms 마다 snapshot 을 읽는다.

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
find_package(aris3_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Qt 의 moc(시그널·슬롯)·rcc(리소스: 로고·테마) 를 자동으로
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 1) 순수 로직
add_library(gui_logic STATIC src/gui_logic.cpp)
target_include_directories(gui_logic PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(gui_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 2) ARIS 실행 파일
add_executable(aris
  src/main.cpp
  src/ros_io.cpp
  src/sim_backend.cpp
  src/launch_proc.cpp
  src/real_backend.cpp
  src/main_window_real.cpp
  include/aris3_gui_cpp/backend.hpp
  include/aris3_gui_cpp/real_backend.hpp
  src/theme.cpp
  src/widgets.cpp
  src/main_window.cpp
  include/aris3_gui_cpp/theme.hpp
  include/aris3_gui_cpp/main_window.hpp
  include/aris3_gui_cpp/widgets.hpp
  include/aris3_gui_cpp/sim_backend.hpp
  resources/aris.qrc)
target_include_directories(aris PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aris gui_logic Qt5::Core Qt5::Gui Qt5::Widgets)
ament_target_dependencies(aris rclcpp rclcpp_action aris3_interfaces sensor_msgs std_msgs rcl_interfaces
  rosgraph_msgs moveit_msgs geometry_msgs shape_msgs tf2 tf2_ros std_srvs)

install(TARGETS aris DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gui_logic test/test_gui_logic.cpp)
  target_link_libraries(test_gui_logic gui_logic)
endif()

ament_package()
