"""Real E0509 manual controller that connects to the robot right away.

  ros2 launch e0509_manual real.launch.py                       # host 110.120.1.66, with RViz
  ros2 launch e0509_manual real.launch.py host:=<controller IP> z_min:=50 rviz:=false

The window itself starts dsr_bringup2 (so its 접속 끊기 / 재접속 buttons can stop and restart it).
Connecting turns the servo ON, takes control authority from the teach pendant and sets
AUTONOMOUS mode (dsr_hardware2 behaviour). Only run it with someone at the robot.
The namespace is fixed to 'e0509_real' so simulation scripts (/dsr01/...) can never reach it.
Closing the window (or Ctrl-C here) disconnects the robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'e0509_real'


def generate_launch_description():
    gui = Node(package='e0509_manual', executable='manual_controller', output='screen',
               sigterm_timeout='15',
               arguments=['--ns', NS, '--host', LaunchConfiguration('host'),
                          '--rviz', LaunchConfiguration('rviz'),
                          '--z-min', LaunchConfiguration('z_min'), '--connect'])
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='110.120.1.66', description='E0509 controller IP'),
        DeclareLaunchArgument('rviz', default_value='true', description='also open RViz'),
        DeclareLaunchArgument('z_min', default_value='50', description='lowest TCP height [mm]'),
        gui,
    ])
