"""Real E0509 + RViz + manual controller in one command.

  ros2 launch e0509_manual real.launch.py                       # host 110.120.1.66
  ros2 launch e0509_manual real.launch.py host:=<controller IP> z_min:=50

Connecting turns the servo ON, takes control authority from the teach pendant and sets
AUTONOMOUS mode (dsr_hardware2 behaviour). Only run it with someone at the robot.
The namespace is fixed to 'e0509_real' so simulation scripts (/dsr01/...) can never reach it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'e0509_real'


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dsr_bringup2'), 'launch', 'dsr_bringup2_rviz.launch.py')),
        launch_arguments={
            'mode': 'real', 'host': LaunchConfiguration('host'), 'port': '12345',
            'model': 'e0509', 'color': 'white', 'name': NS,
            'gui': LaunchConfiguration('rviz'), 'remap_tf': 'true',
        }.items())
    gui = Node(package='e0509_manual', executable='manual_controller', output='screen',
               arguments=['--ns', NS, '--z-min', LaunchConfiguration('z_min')])
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='110.120.1.66', description='E0509 controller IP'),
        DeclareLaunchArgument('rviz', default_value='true', description='also open RViz'),
        DeclareLaunchArgument('z_min', default_value='50', description='lowest TCP height [mm]'),
        bringup, gui,
    ])
