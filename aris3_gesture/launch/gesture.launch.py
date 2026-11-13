"""ARIS3 손짓 노드 (MediaPipe).

  ros2 launch aris3_gesture gesture.launch.py [python:=~/venvs/aris3_gesture/bin/python]

MediaPipe 는 시스템 Python 이 아니라 가상환경에 설치되어 있으므로, 그 Python 으로 노드를 실행한다.
(ROS 의 PYTHONPATH 는 그대로 이어져서 rclpy·메시지는 시스템 것을 쓴다. 가상환경 numpy 는 1.x 로 맞출 것)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=os.path.expanduser('~/venvs/aris3_gesture/bin/python'),
                              description='mediapipe 가 설치된 Python'),
        DeclareLaunchArgument('image_topic', default_value='/aris3/overhead_camera/image/compressed'),
        DeclareLaunchArgument('max_hz', default_value='15.0', description='손 인식 최대 처리 빈도'),
        Node(package='aris3_gesture', executable='gesture_node', output='screen',
             prefix=LaunchConfiguration('python'),
             parameters=[{'image_topic': LaunchConfiguration('image_topic'),
                          'max_hz': LaunchConfiguration('max_hz')}]),
    ])
