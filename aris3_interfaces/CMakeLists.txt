cmake_minimum_required(VERSION 3.8)
project(aris3_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# .msg/.srv/.action 파일에서 C++ 헤더와 Python 모듈을 자동 생성한다 (rosidl).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/TaskStatus.msg"
  "msg/VoiceCommand.msg"
  "msg/HandGesture.msg"
  "msg/GestureEvent.msg"
  "msg/RealRobotState.msg"
  "srv/GripperCommand.srv"
  "srv/RealCalibration.srv"
  "action/ExecuteTask.action"
  "action/RealMotion.action"
  "action/RealPickPlace.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
