"""실물 E0509 브리지 실행.

  ros2 launch aris3_real_bridge real_bridge.launch.py
  ros2 launch aris3_real_bridge real_bridge.launch.py host:=110.120.1.66 z_min_mm:=50.0
  ros2 launch aris3_real_bridge real_bridge.launch.py mode:=virtual gripper:=service   # 실물 비슷한 시뮬레이션

로봇 접속(dsr_bringup2)은 이 노드가 /aris3/real/connect 를 받았을 때 시작하고, 노드를 끄면 함께 끊는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """launch 인자(접속 주소·최저 높이·잠금 시간)를 받아 브리지 노드 하나를 띄운다."""
    args = [('ns', 'e0509_real', 'dsr_bringup2 이름'), ('host', '110.120.1.66', 'E0509 컨트롤러 IP'),
            ('port', '12345', '컨트롤러 포트'), ('z_min_mm', '50.0', '손끝 최저 높이 [mm]'),
            ('arm_timeout_s', '120.0', '명령이 없으면 조작 허용을 잠그는 시간 [s]'),
            ('mode', 'real', 'real: 실물 컨트롤러 / virtual: 두산 가상 컨트롤러(시뮬레이션, host 는 127.0.0.1 로)'),
            ('gripper', 'drl', 'drl: 실물 그리퍼(DRL) / service: /aris3/gripper/command (가상 모드 전용)'),
            ('calib_path', '', '카메라 보정 파일 (비우면 실물 real_camera_calib.json / 가상 sim_camera_calib.json)'),
            ('cube_mm', '0.0', '쌓기에 쓰는 큐브 높이 [mm] (0 이면 실물 40 / 가상 50)'),
            ('align_yaw', 'true', '그리퍼 각도 맞추기 (집기: 큐브 면, 놓기: 팔 방향, 쌓기: 받침 큐브)'),
            ('center_grasp', 'true', '집기 직전 큐브 중심 맞추기 (접근 높이에서 다시 재서 그 위로)')]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args] + [
        Node(package='aris3_real_bridge', executable='real_bridge', output='screen',
             parameters=[{'ns': LaunchConfiguration('ns'), 'host': LaunchConfiguration('host'),
                          'port': LaunchConfiguration('port'), 'z_min_mm': LaunchConfiguration('z_min_mm'),
                          'arm_timeout_s': LaunchConfiguration('arm_timeout_s'),
                          'mode': LaunchConfiguration('mode'), 'gripper': LaunchConfiguration('gripper'),
                          'calib_path': LaunchConfiguration('calib_path'),
                          'cube_mm': LaunchConfiguration('cube_mm'),
                          'align_yaw': LaunchConfiguration('align_yaw'),
                          'center_grasp': LaunchConfiguration('center_grasp')}]),
    ])
