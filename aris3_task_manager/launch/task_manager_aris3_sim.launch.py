"""aris3_bringup Gazebo 작업 셀 + MoveIt 2 위에서 ARIS3 Task Manager 실행.

  터미널 1: ros2 launch aris3_bringup simulation.launch.py [grasp_assist:=true]
  터미널 2: ros2 launch aris3_moveit_config move_group.launch.py
  터미널 3: ros2 launch aris3_task_manager task_manager_aris3_sim.launch.py [grasp_assist:=true]
  터미널 4: ros2 run aris3_task_manager send_task blue A

함께 띄우는 것 (시뮬레이션 전용, aris3_sim_bridge):
  gz_gripper_bridge  /aris3/gripper/command -> Gazebo RH-P12-RN-A 컨트롤러
                     grasp_assist:=true 이면 물체 붙이기 보조 (simulation.launch.py 에도 같은 값을 줄 것)
  gz_truth_detector  [임시] Gazebo 정답 위치 -> /aris3/detected_objects (aris3_vision 이 생기면 truth:=false)
"""
import csv
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

TABLE_TOP_Z = 0.75   # aris3_bringup/launch/simulation.launch.py 와 같아야 함


def _setup(context):
    share = Path(get_package_share_directory("aris3_task_manager"))
    xacro_file = (Path(get_package_share_directory("e0509_gripper_description"))
                  / "urdf/e0509_with_gripper.urdf.xacro")
    moveit = (
        MoveItConfigsBuilder("e0509_with_gripper", package_name="aris3_moveit_config")
        .robot_description(file_path=str(xacro_file),
                           mappings={"use_gazebo": "true", "base_z": str(TABLE_TOP_Z)})
        .robot_description_semantic(file_path="config/e0509_gripper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    assist = LaunchConfiguration("grasp_assist").perform(context) == "true"
    layout = Path(get_package_share_directory("aris3_bringup")) / "config/object_layout.csv"
    with layout.open(encoding="utf-8") as f:
        names = [row["name"] for row in csv.DictReader(f)]
    sim_time = {"use_sim_time": True}

    return [
        # Gazebo 물체 위치 (spawn_objects.py 의 PosePublisher) -> ROS (gz_truth_detector 입력)
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="aris3_object_pose_bridge",
             arguments=[f"/model/{n}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose" for n in names],
             condition=IfCondition(LaunchConfiguration("truth"))),
        Node(package="aris3_sim_bridge", executable="gz_truth_detector", output="screen",
             parameters=[sim_time, {"objects": names}],
             condition=IfCondition(LaunchConfiguration("truth"))),
        Node(package="aris3_sim_bridge", executable="gz_gripper_bridge", output="screen",
             parameters=[sim_time, {"assist_objects": names if assist else [""], "tcp_offset_m": 0.105}]),
        Node(package="aris3_task_manager", executable="task_manager", output="screen",
             parameters=[str(share / "config/task_manager_aris3_sim.yaml"),
                         moveit.robot_description, moveit.robot_description_semantic,
                         moveit.robot_description_kinematics, sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("grasp_assist", default_value="false",
                              description="물체 붙이기 보조 (simulation.launch.py 와 같은 값)"),
        DeclareLaunchArgument("truth", default_value="true",
                              description="[임시] Gazebo 정답 위치로 /aris3/detected_objects 발행"),
        OpaqueFunction(function=_setup),
    ])
