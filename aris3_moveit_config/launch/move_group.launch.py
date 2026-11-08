"""ARIS3 MoveIt 2 move_group (Gazebo 시뮬레이션용).

먼저 시뮬레이션을 띄운 뒤 실행한다:
  터미널 1: ros2 launch aris3_bringup simulation.launch.py
  터미널 2: ros2 launch aris3_moveit_config move_group.launch.py [rviz:=true]

연결:
  관절 상태  /e0509_gripper/joint_states        (Gazebo joint_state_broadcaster)
  궤적 실행  /e0509_gripper/arm_controller/follow_joint_trajectory
  TF         /tf (simulation.launch.py 의 robot_state_publisher)
  작업 셀    workcell_scene.py 가 탁자를 충돌 물체로 등록

robot_description 은 Gazebo 와 같은 xacro·같은 base_z 로 만든다.
TABLE_TOP_Z 를 바꾸면 aris3_bringup/launch/simulation.launch.py 와 함께 바꿀 것.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

ROBOT_NS = "/e0509_gripper"
TABLE_TOP_Z = 0.75   # aris3_bringup/launch/simulation.launch.py 의 TABLE_TOP_Z 와 같아야 함


def _setup(context):
    xacro_file = (Path(get_package_share_directory("e0509_gripper_description"))
                  / "urdf/e0509_with_gripper.urdf.xacro")
    config = (
        MoveItConfigsBuilder("e0509_with_gripper", package_name="aris3_moveit_config")
        .robot_description(file_path=str(xacro_file),
                           mappings={"use_gazebo": "true", "base_z": str(TABLE_TOP_Z)})
        .robot_description_semantic(file_path="config/e0509_gripper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],
                            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}
    remaps = [
        ("joint_states", f"{ROBOT_NS}/joint_states"),
        ("arm_controller/follow_joint_trajectory",
         f"{ROBOT_NS}/arm_controller/follow_joint_trajectory"),
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config.to_dict(), sim_time,
                    {"publish_robot_description_semantic": True}],
        remappings=remaps,
    )
    scene = Node(
        package="aris3_moveit_config",
        executable="workcell_scene.py",
        output="screen",
        parameters=[sim_time, {"table_top_z": TABLE_TOP_Z}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="aris3_moveit_rviz",
        output="log",
        arguments=["-d", str(Path(get_package_share_directory("aris3_moveit_config"))
                             / "config/moveit.rviz")],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [move_group, scene, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="MoveIt MotionPlanning 패널이 있는 RViz2 도 띄움"),
        OpaqueFunction(function=_setup),
    ])
