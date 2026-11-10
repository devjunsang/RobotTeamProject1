"""실물 E0509 브리지 실행.

  ros2 launch aris3_real_bridge real_bridge.launch.py
  ros2 launch aris3_real_bridge real_bridge.launch.py host:=110.120.1.66 z_min_mm:=50.0

로봇 접속(dsr_bringup2)은 이 노드가 /aris3/real/connect 를 받았을 때 시작하고, 노드를 끄면 함께 끊는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [('ns', 'e0509_real', 'dsr_bringup2 이름'), ('host', '110.120.1.66', 'E0509 컨트롤러 IP'),
            ('port', '12345', '컨트롤러 포트'), ('z_min_mm', '50.0', '손끝 최저 높이 [mm]'),
            ('arm_timeout_s', '120.0', '명령이 없으면 조작 허용을 잠그는 시간 [s]')]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args] + [
        Node(package='aris3_real_bridge', executable='real_bridge', output='screen',
             parameters=[{'ns': LaunchConfiguration('ns'), 'host': LaunchConfiguration('host'),
                          'port': LaunchConfiguration('port'), 'z_min_mm': LaunchConfiguration('z_min_mm'),
                          'arm_timeout_s': LaunchConfiguration('arm_timeout_s')}]),
    ])
