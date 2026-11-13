"""실물과 비슷한 시뮬레이션: 실물 경로(aris3_real_bridge -> RobotLink -> 두산 서비스)를 로봇 없이 시험 (2026-11-10).

  ros2 launch aris3_bringup aris3_real_sim.launch.py          # 1) 이 월드를 띄우고
  ros2 run aris3_gui control_station --mode real --sim        # 2) GUI 실물 모드(가상) -> Connect

실물 구성                                   이 시뮬레이션
  두산 컨트롤러 (dsr_bringup2 mode:=real)     두산 가상 컨트롤러 (Docker DRCF, mode:=virtual) - GUI 의 Connect 가 띄움
  E0509 팔                                    Gazebo ARIS3 로봇, 가상 컨트롤러 관절을 그대로 따라감 (dsr_joint_mirror)
  RH-P12-RN-A 그리퍼 (DRL)                    Gazebo RH-P12-RN-A (gz_gripper_bridge, /aris3/gripper/command)
  D455 위쪽 카메라 JPEG 1280x720 30 Hz        Gazebo 카메라 1280x720 30 Hz, 같은 토픽에 JPEG 만 (jpeg_republisher)
    (탁자 위 약 0.8 m, 영상 위 = 로봇 +X)        (탁자 위 0.85 m, (0.40, 0) 위, 영상 위 = 로봇 +X, 약 1.2 mm/px)
  색 검출 scripts/aris3_color_detector.py     같은 스크립트 (영상 픽셀 -> /aris3/vision/detections_px)
  작업대 위 큐브 (로봇 앞 x 0.47~0.75 m)       큐브 3개를 로봇 앞으로 옮김 (config/object_layout_real_like.csv)

실물과 다른 점 (금요일 실물 재시험에서 확인할 것):
  - 가상 컨트롤러는 충돌 감지·외력 안전 정지·관절 브레이크를 재현하지 않는다. 팔은 탁자·큐브를 뚫고 지날 수 있다
    (Gazebo 로봇은 위치를 따라가기만 함). 최저 높이 검사(z_min_mm)는 RobotLink 가 그대로 한다.
  - 그리퍼는 DRL 이 아니라 서비스. 파지는 「물체 붙이기 보조」 (Gazebo 마찰 파지가 불안정해서).
    보조가 쓰는 물체 위치는 Gazebo 정답이며 /aris3/sim/truth_objects 로만 보낸다 (GUI·브리지는 실물처럼 카메라만 봄).
  - 큐브는 5 cm (실물 4 cm). 카메라 렌즈 왜곡·조명 변화 없음.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TABLE_TOP_Z = 0.75                    # simulation.launch.py 와 같은 값
CAMERA_XYZ = (0.40, 0.0, TABLE_TOP_Z + 0.85)   # 실물 보정에서 잰 영상 중심 (약 400, 0) mm 위
CAMERA_RAW_TOPIC = '/aris3/sim/real_camera/image'   # Gazebo 원본 (jpeg_republisher 만 씀)
CAMERA_TOPIC = '/aris3/overhead_camera/image/compressed'   # 실물 카메라 노드와 같은 토픽
OBJECTS = ['red_cube', 'blue_cube', 'yellow_cube']
SIM_TIME = {'use_sim_time': True}


def camera_sdf():
    """기본 Top View 카메라 모델을 실물 D455 비슷하게 바꾼 SDF 를 /tmp 에 만들고 경로를 돌려준다."""
    src = Path(get_package_share_directory('aris3_bringup')) / 'models/overhead_camera.sdf'
    text = src.read_text()
    for old, new in (('<model name="aris3_overhead_camera">', '<model name="aris3_real_like_camera">'),
                     ('<topic>/aris3/overhead_camera/image</topic>', f'<topic>{CAMERA_RAW_TOPIC}</topic>'),
                     ('<width>640</width>', '<width>1280</width>'), ('<height>480</height>', '<height>720</height>'),
                     ('<horizontal_fov>1.4</horizontal_fov>', '<horizontal_fov>1.5</horizontal_fov>'),
                     ('<update_rate>15</update_rate>', '<update_rate>30</update_rate>')):
        if old not in text:
            raise RuntimeError(f'[ARIS3 real-sim] 카메라 모델에서 찾지 못함: {old} ({src})')
        text = text.replace(old, new)
    out = Path('/tmp/aris3_real_like_camera.sdf')
    out.write_text(text)
    return out


def detector_script(context):
    """색 검출 스크립트 경로: 인자 > 저장소의 scripts/ (이 launch 파일이 저장소로 연결돼 있을 때)."""
    arg = LaunchConfiguration('detector_script').perform(context)
    if arg:
        return Path(arg)
    return Path(os.path.realpath(__file__)).parents[2] / 'scripts/aris3_color_detector.py'


def setup(context):
    """실행 시점에 경로·SDF 를 만들고 실행할 것들을 돌려준다."""
    share = Path(get_package_share_directory('aris3_bringup'))
    emu_ns = LaunchConfiguration('emulator_ns').perform(context)
    det = detector_script(context)
    actions = [
        # 1) ARIS3 월드·로봇·큐브 (기본 카메라는 끔, 물체 붙이기 보조 켬)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/simulation.launch.py')),
                                 launch_arguments={'grasp_assist': 'true', 'camera': 'false',
                                                   'arm_controller': 'dsr_position_controller'}.items()),
        # 2) 팔 제어기 확인·재생·큐브 배치
        Node(package='aris3_bringup', executable='real_sim_setup.py', output='screen'),
        # 3) 두산 가상 컨트롤러 관절 -> Gazebo 팔
        Node(package='aris3_sim_bridge', executable='dsr_joint_mirror', output='screen',
             parameters=[{'emulator_ns': emu_ns, 'robot_ns': '/e0509_gripper'}]),
        # 4) 그리퍼: /aris3/gripper/command -> Gazebo RH-P12-RN-A (+ 물체 붙이기 보조, 정답 위치는 내부 토픽으로만)
        # 물체 정답 위치: Gazebo PosePublisher -> ROS (/model/<물체>/pose). MoveIt 시뮬레이션은 task_manager launch 가
        # 띄우는 브리지인데 이 launch 에는 빠져 있어 보조가 동작하지 않았다 (2026-11-10 시험에서 발견)
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='aris3_real_sim_pose_bridge', output='screen',
             arguments=[f'/model/{n}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' for n in OBJECTS]),
        Node(package='aris3_sim_bridge', executable='gz_truth_detector', output='screen',
             parameters=[SIM_TIME, {'objects': OBJECTS, 'output_topic': '/aris3/sim/truth_objects'}]),
        Node(package='aris3_sim_bridge', executable='gz_gripper_bridge', output='screen',
             parameters=[SIM_TIME, {'assist_objects': OBJECTS, 'tcp_offset_m': 0.105}],
             remappings=[('/aris3/detected_objects', '/aris3/sim/truth_objects')]),
        # 5) 실물 비슷한 카메라: Gazebo 가 뜬 뒤 생성, 원본 -> JPEG
        TimerAction(period=8.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'empty', '-name', 'aris3_real_like_camera', '-file', str(camera_sdf()),
                            '-x', str(CAMERA_XYZ[0]), '-y', str(CAMERA_XYZ[1]), '-z', str(CAMERA_XYZ[2])]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='aris3_real_like_camera_bridge',
                 arguments=[f'{CAMERA_RAW_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image'], output='screen'),
        ]),
        Node(package='aris3_sim_bridge', executable='jpeg_republisher', output='screen',
             parameters=[{'input': CAMERA_RAW_TOPIC, 'output': CAMERA_TOPIC, 'quality': 80, 'max_hz': 30.0}]),
    ]
    # 6) 색 검출: 실물에서 쓰는 스크립트 그대로
    if det.is_file():
        actions.append(ExecuteProcess(cmd=['python3', str(det), '--input', CAMERA_TOPIC], output='screen'))
    else:
        actions.append(LogInfo(msg=f'[ARIS3 real-sim] 색 검출 스크립트 없음: {det} - detector_script:=<경로> 로 지정'))
    return actions


def generate_launch_description():
    """launch 인자 선언 후 setup() 으로 구성."""
    return LaunchDescription([
        DeclareLaunchArgument('emulator_ns', default_value='e0509_real',
                              description='두산 가상 컨트롤러 이름 (aris3_real_bridge 의 ns 와 같게)'),
        DeclareLaunchArgument('detector_script', default_value='',
                              description='색 검출 스크립트 경로 (비우면 저장소 scripts/aris3_color_detector.py)'),
        OpaqueFunction(function=setup),
    ])
