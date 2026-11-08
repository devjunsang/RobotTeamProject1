"""Gazebo 작업 셀(e0509_sim) 위에서 ARIS3 Task Manager 실행.

  터미널 1: ros2 launch e0509_sim e0509_workcell.launch.py        # Gazebo + DRCF 에뮬레이터 + 카메라 + 그리퍼
  터미널 2: ros2 launch aris3_task_manager task_manager_sim.launch.py
  터미널 3: ros2 run aris3_task_manager send_task red A

bridges:=false 로 하면 어댑터를 띄우지 않는다 (팀의 aris3_vision / aris3_control 이 계약을 제공할 때).

좌표계: e0509_sim 의 TF 는 world -> base_link 를 0 으로 내보내지만 Gazebo 에서 로봇은 테이블 위
z = 0.75 m 에 있다. 카메라·구역 좌표(Gazebo 바닥 기준)를 'workcell' 로 부르고
workcell -> world 정적 변환(z = robot_base_z)을 여기서 내보내 TF 트리를 완성한다:
    workcell --(0, 0, 0.75)--> world --(0)--> base_link --> link_1 ...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('aris3_task_manager'), 'config', 'task_manager.yaml')
    robot_ns = LaunchConfiguration('robot_ns')
    bridges = LaunchConfiguration('bridges')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='/dsr01', description='두산 서비스 네임스페이스'),
        DeclareLaunchArgument('bridges', default_value='true', description='e0509_sim 어댑터 실행'),
        DeclareLaunchArgument('robot_base_z', default_value='0.75', description='Gazebo 에서 로봇 베이스 높이 [m]'),
        DeclareLaunchArgument('workcell_tf', default_value='true', description='workcell -> world 정적 TF 발행'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='workcell_to_world',
             arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('robot_base_z'),
                        '--frame-id', 'workcell', '--child-frame-id', 'world'],
             condition=IfCondition(LaunchConfiguration('workcell_tf'))),
        Node(package='aris3_sim_bridge', executable='vision_bridge', output='screen',
             condition=IfCondition(bridges)),
        Node(package='aris3_sim_bridge', executable='gripper_bridge', output='screen',
             parameters=[{'robot_ns': robot_ns}], condition=IfCondition(bridges)),
        Node(package='aris3_task_manager', executable='task_manager', output='screen',
             parameters=[params, {'robot_ns': robot_ns}]),
    ])
