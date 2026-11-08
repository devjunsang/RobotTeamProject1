"""ARIS3 시뮬레이션 전체를 순서대로 실행 (GUI 의 Connect 버튼이 쓰는 진입점).

  ros2 launch aris3_bringup aris3_sim_full.launch.py [grasp_assist:=true] [voice:=true]

순서 (앞 단계가 실제로 준비된 것을 확인한 뒤 다음 단계):
  1. simulation.launch.py           Gazebo + 로봇 + 물체 + 카메라 + 컨트롤러
  2. arm_controller 액션이 보이면   aris3_moveit_config/move_group.launch.py
  3. move_group(/move_action) 이 보이면  aris3_task_manager/task_manager_aris3_sim.launch.py
  (음성) voice:=true 이면 aris3_voice/voice.launch.py 를 처음부터 함께 띄운다 (Whisper 로딩이 오래 걸림)
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, name, args=None, condition=None):
    path = Path(get_package_share_directory(pkg)) / "launch" / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    assist = LaunchConfiguration("grasp_assist")
    wait_arm = Node(package="aris3_bringup", executable="wait_for_action.py", output="screen",
                    arguments=["/e0509_gripper/arm_controller/follow_joint_trajectory"])
    wait_move = Node(package="aris3_bringup", executable="wait_for_action.py", output="screen",
                     arguments=["/move_action"])

    def after(proc, label, actions):
        return RegisterEventHandler(OnProcessExit(
            target_action=proc,
            on_exit=lambda event, ctx: ([LogInfo(msg=f"[ARIS3] {label}")] + actions
                                        if event.returncode == 0 else
                                        [LogInfo(msg=f"[ARIS3] {label} 실패 - 다음 단계를 시작하지 않음")])))

    return LaunchDescription([
        DeclareLaunchArgument("grasp_assist", default_value="true",
                              description="물체 붙이기 파지 보조 (false = 순수 마찰 파지)"),
        DeclareLaunchArgument("voice", default_value="true", description="aris3_voice 음성 노드도 실행"),
        _include("aris3_bringup", "simulation.launch.py", {"grasp_assist": assist}),
        _include("aris3_voice", "voice.launch.py", condition=IfCondition(LaunchConfiguration("voice"))),
        wait_arm,
        after(wait_arm, "컨트롤러 준비 -> MoveIt 시작",
              [_include("aris3_moveit_config", "move_group.launch.py"), wait_move]),
        after(wait_move, "MoveIt 준비 -> Task Manager 시작",
              [_include("aris3_task_manager", "task_manager_aris3_sim.launch.py", {"grasp_assist": assist})]),
    ])
