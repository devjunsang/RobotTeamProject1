"""ARIS3 음성 명령 노드.

  ros2 launch aris3_voice voice.launch.py [model:=base] [python:=~/venvs/mark1_voice/bin/python]

faster-whisper 는 시스템 Python 이 아니라 가상환경에 설치되어 있으므로, 그 Python 으로 노드를 실행한다.
(ROS 의 PYTHONPATH 는 그대로 이어져서 rclpy·메시지는 시스템 것을 쓴다. 가상환경 numpy 는 1.x 로 맞출 것)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=os.path.expanduser('~/venvs/mark1_voice/bin/python'),
                              description='faster-whisper 가 설치된 Python'),
        DeclareLaunchArgument('model', default_value='base', description='Whisper 모델 크기'),
        DeclareLaunchArgument('audio_device', default_value='default', description='arecord -D 장치'),
        Node(package='aris3_voice', executable='voice_node', output='screen',
             prefix=LaunchConfiguration('python'),
             parameters=[{'model': LaunchConfiguration('model'),
                          'audio_device': LaunchConfiguration('audio_device'),
                          'known_colors': ['red', 'blue', 'yellow']}]),
    ])
