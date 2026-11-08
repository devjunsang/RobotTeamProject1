# 표준 라이브러리: 파일 경로 결합과 Gazebo Launch 파일 경로 작성 등에 사용한다.
import csv
import os
# 외부 프로그램(xacro, check_urdf)을 실행하기 위한 표준 라이브러리다.
import subprocess
# 파일 및 디렉터리 경로를 OS에 맞게 다루는 객체를 제공한다.
from pathlib import Path

# ROS 2 패키지 이름으로 설치된 share 디렉터리를 찾는다.
from ament_index_python.packages import get_package_share_directory
# LaunchDescription은 여러 실행 액션을 묶는 Launch 최상위 구성 객체다.
from launch import LaunchDescription
# Launch에서 사용하는 각종 실행/이벤트/환경 액션을 가져온다.
from launch.actions import (
    DeclareLaunchArgument,     # launch 인자 (grasp_assist)
    IncludeLaunchDescription,  # 다른 Launch 파일을 현재 Launch에 포함한다.
    OpaqueFunction,             # 실행 시점에 Python 함수로 액션 목록을 생성한다.
    RegisterEventHandler,      # 프로세스 종료 등의 이벤트를 감시한다.
    SetEnvironmentVariable,    # 이후 시작할 프로세스에 환경변수를 전달한다.
    TimerAction,               # 지정한 시간이 지난 뒤 액션을 실행한다.
    LogInfo,                   # Launch 로그 메시지를 출력한다.
)
# 특정 프로세스 종료 시점을 감지하는 이벤트 핸들러다.
from launch.event_handlers import OnProcessExit
# 다른 Python Launch 파일을 로드하는 소스 객체다.
from launch.launch_description_sources import PythonLaunchDescriptionSource
# ROS 2 노드를 Launch 액션으로 실행하기 위한 클래스다.
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 로봇 관련 ROS 2 노드에 적용할 네임스페이스다.
NAMESPACE = "e0509_gripper"
# Gazebo 월드에서 로봇 엔티티에 붙일 이름이다.
ROBOT_NAME = "e0509_with_gripper"
# 작업 탁자 윗면 높이 [m]. 로봇 베이스가 이 높이에 놓인다 (e0509_sim 작업 셀과 같은 값).
# 바꾸면 worlds/aris3_world.sdf 의 탁자와 config/object_layout.csv 의 z 도 같이 바꿀 것.
TABLE_TOP_Z = 0.75
# 카메라는 탁자 윗면 위 2.0 m 에 둔다 (탁자 도입 전 바닥 위 2.0 m 와 같은 시야).
CAMERA_Z = TABLE_TOP_Z + 2.0


def setup_simulation(context, *args, **kwargs):
    # 현재 로그인한 사용자 홈 디렉터리를 Path 객체로 얻는다.
    home = Path.home()

    # 설치된 e0509_gripper_description 패키지의 Xacro 경로를 찾는다.
    xacro_file = (
        Path(get_package_share_directory("e0509_gripper_description"))
        / "urdf/e0509_with_gripper.urdf.xacro"
    )
    # Doosan의 Gazebo 컨트롤러 설정 경로를 찾는다.
    # 이 경로는 아래에서 URDF 안의 문자열을 aris3_bringup/config/gz_controllers.yaml 로
    # 바꿀 때 찾는 대상으로 쓴다.
    controller_yaml = (
        Path(get_package_share_directory("dsr_controller2"))
        / "config/dsr_controller2_gz.yaml"
    )
    # Xacro를 변환한 URDF의 출력 파일 경로다.
    urdf_file = Path("/tmp/aris3_e0509.urdf")

    # ROS 2 실행 전에 의존 파일이 실제로 있는지 확인한다.
    for file in [xacro_file, controller_yaml]:
        if not file.is_file():
            raise RuntimeError(f"[ARIS3] 필수 파일 없음: {file}")

    # xacro CLI를 호출해 Gazebo용 URDF를 생성한다.
    subprocess.run(
        # base_z: URDF 의 world -> base_link 높이. 로봇을 탁자 위에 올린다.
        ["xacro", str(xacro_file), "use_gazebo:=true", f"base_z:={TABLE_TOP_Z}",
         "-o", str(urdf_file)],
        check=True,
    )

    # URDF의 기본 XML/링크/관절 구조를 검사한다.
    subprocess.run(
        ["check_urdf", str(urdf_file)],
        check=True,
        stdout=subprocess.DEVNULL,
    )

    # robot_state_publisher의 robot_description 파라미터에 넣을 URDF 문자열이다.
    robot_description = urdf_file.read_text()

    # Gazebo ros2_control 설정을 ARIS3 파일로 교체한다 (팔 궤적 컨트롤러 + 100 Hz).
    # dsr_description2 매크로가 넣는 경로를 문자열로 바꾸므로 외부 패키지는 수정하지 않는다.
    aris3_controllers = (
        Path(get_package_share_directory("aris3_bringup"))
        / "config/gz_controllers.yaml"
    )
    if str(controller_yaml) not in robot_description:
        raise RuntimeError(
            "[ARIS3] URDF 에서 Gazebo 컨트롤러 설정 경로를 찾지 못함: "
            f"{controller_yaml} (dsr_description2 버전 확인)"
        )
    robot_description = robot_description.replace(
        str(controller_yaml), str(aris3_controllers)
    )
    # [파지 보조 옵션] grasp_assist:=true 이면 물체마다 DetachableJoint 를 로봇에 붙인다.
    # 그리퍼 어댑터(aris3_sim_bridge gz_gripper_bridge)가 손가락이 물체에 막혔을 때만
    # /aris3/sim/<물체>/attach 를 보내 손(link_6)에 고정하고, 열 때 detach 한다.
    # 기본(false)은 순수 마찰 파지. 두 결과는 따로 기록한다 (docs/ARIS3_개발방향_확정.md).
    grasp_assist = LaunchConfiguration("grasp_assist").perform(context) == "true"
    object_names = []
    if grasp_assist:
        layout = (Path(get_package_share_directory("aris3_bringup"))
                  / "config/object_layout.csv")
        with layout.open(encoding="utf-8") as f:
            object_names = [row["name"] for row in csv.DictReader(f)]
        plugins = "".join(
            '<gazebo><plugin filename="gz-sim-detachable-joint-system" '
            'name="gz::sim::systems::DetachableJoint">'
            f"<parent_link>link_6</parent_link><child_model>{n}</child_model>"
            "<child_link>body</child_link>"
            f"<attach_topic>/aris3/sim/{n}/attach</attach_topic>"
            f"<detach_topic>/aris3/sim/{n}/detach</detach_topic>"
            f"<output_topic>/aris3/sim/{n}/state</output_topic>"
            "</plugin></gazebo>"
            for n in object_names
        )
        robot_description = robot_description.replace("</robot>", plugins + "</robot>")

    urdf_file.write_text(robot_description)   # Gazebo 스폰도 같은 URDF 를 쓴다

    # ROS 2 TF 및 robot_description을 제공하는 노드다.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # PHASE 3 / STEP 03: 카메라 센서 시스템이 포함된 Gazebo 월드와 카메라 모델 경로.
    world_file = (
        Path(get_package_share_directory("aris3_bringup"))
        / "worlds/aris3_world.sdf"
    )
    camera_file = (
        Path(get_package_share_directory("aris3_bringup"))
        / "models/overhead_camera.sdf"
    )

    # Gazebo 서버 및 GUI 실행. -r 옵션이 없어 일시정지 상태에서 시작한다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-v 4 {world_file}",
        }.items(),
    )

    # Gazebo Transport의 /clock을 ROS 2의 /clock으로 연결한다.
    clock_bridge = Node(
        name="aris3_clock_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # 생성한 URDF 파일을 Gazebo의 empty 월드에 삽입한다.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", ROBOT_NAME,
            "-file", str(urdf_file),
            "-x", "0",
            "-y", "0",
            "-z", "0",
        ],
        output="screen",
    )

    # 하드웨어 등록 대기 및 3개 컨트롤러 순차 활성화는 별도 스크립트가 담당한다.
    start_controllers = Node(
        package="aris3_bringup",
        executable="start_controllers.py",
        output="screen",
    )

    # PHASE 3 / STEP 02: 색상 정육면체 4개를 CSV 배치대로 생성한다.
    spawn_objects = Node(
        package="aris3_bringup",
        executable="spawn_objects.py",
        arguments=["--spawn"],
        output="screen",
    )

    # PHASE 3 / STEP 03: 로봇 베이스 바로 위(탁자 윗면 위 2.0m)에 고정 카메라를 설치한다.
    spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", "aris3_overhead_camera",
            "-file", str(camera_file),
            "-x", "0",
            "-y", "0",
            "-z", str(CAMERA_Z),
        ],
        output="screen",
    )

    # Gazebo 카메라 영상을 ROS 2 Image 메시지로 전달한다.
    camera_bridge = Node(
        name="aris3_camera_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/aris3/overhead_camera/image"
            "@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    # 파지 보조용 attach/detach/state 토픽 브리지 (grasp_assist:=true 일 때만)
    assist_bridge_args = []
    for n in object_names:
        assist_bridge_args += [
            f"/aris3/sim/{n}/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            f"/aris3/sim/{n}/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            f"/aris3/sim/{n}/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ]
    assist_bridge = Node(
        name="aris3_grasp_assist_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=assist_bridge_args,
        output="screen",
    )

    # 로봇 스폰 프로세스 종료 이벤트를 받으면 이후 노드를 실행한다.

    # ========================================================
    # PHASE 3 / STEP 04 : 카메라 TF 자동 발행
    # ========================================================

    # world -> overhead_camera_frame
    # Gazebo 카메라의 설치 위치 (0, 0, CAMERA_Z). world = Gazebo 바닥 원점
    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aris3_camera_mount_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", str(CAMERA_Z),
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "overhead_camera_frame",
        ],
        output="screen",
    )
    # ========================================================
    # PHASE 3 / STEP 04 - RViz2 자동 실행
    #
    # Gazebo 통합 Launch 실행 시 RViz2를 자동 실행하고,
    # 저장된 RobotModel, TF, MarkerArray 설정을 불러온다.
    # ========================================================
    rviz_config = os.path.join(
        get_package_share_directory("aris3_bringup"),
        "rviz",
        "aris3.rviz",
    )

    aris3_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="aris3_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    # overhead_camera_frame -> overhead_camera_optical_frame
    # 카메라 영상의 광학 좌표축 방향 정의
    # SDF의 센서 회전과 ROS 광학축 규약을 반영한 값
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aris3_camera_optical_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0.70710678",
            "--qy", "-0.70710678",
            "--qz", "0",
            "--qw", "0",
            "--frame-id", "overhead_camera_frame",
            # 실제 Gazebo 카메라 Image 메시지의 frame_id와 일치시킨다.
            "--child-frame-id", "aris3_overhead_camera/camera_link/rgb_camera",
        ],
        output="screen",
    )

    # CSV의 객체 4개를 RViz2 MarkerArray로 발행
    object_markers = Node(
        package="aris3_bringup",
        executable="object_marker_publisher.py",
        name="aris3_object_markers",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    def after_spawn(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="[ARIS3] Robot spawn failed")]

        # 이 네 액션은 순서대로 완료되는 것이 아니라 실행이 시작된다.
        return [
            LogInfo(msg="[ARIS3] Robot spawned; checking hardware"),
            start_controllers,
            spawn_objects,
            spawn_camera,
            camera_bridge,
        ] + ([assist_bridge] if object_names else [])

    # 로봇 스폰 완료 여부를 감시한다.
    controller_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=after_spawn,
        )
    )

    # ================================================================
    # PHASE 3 / STEP 03: 객체 자동 스폰 프로세스 종료 코드 감시 [추가]
    # ================================================================
    # 목적:
    # - spawn_objects.py가 Launch 내부에서 끝까지 실행됐는지 확인한다.
    # - 종료 코드 0이면 정상 종료 로그, 0이 아니면 오류 로그를 남긴다.
    # - 주의: 종료 코드 0은 "스크립트 성공"이지 "Gazebo 모델 유지"의 증명은 아니다.
    def after_objects_spawn(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg="[ARIS3] Object spawning process completed"),
            ]

        return [
            LogInfo(
                msg=(
                    "[ARIS3] Object spawning failed: "
                    f"returncode={event.returncode}"
                )
            ),
        ]

    # spawn_objects 노드가 종료될 때 위 콜백을 실행하도록 등록한다.
    object_spawn_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_objects,
            on_exit=after_objects_spawn,
        )
    )

    # Launch 액션 목록. 핸들러는 spawn_robot 실행 이전에 등록한다.
    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=":".join([
                str(home / "ros2_ws/install/dsr_description2/share"),
                str(home / "ros2_ws/install/rh_p12_rn_a_description/share"),
            ]),
        ),
        robot_state_publisher,
        clock_bridge,
        gazebo,
        camera_mount_tf,
        camera_optical_tf,
        object_markers,
        controller_start_handler,
	aris3_rviz,  # RViz2 자동 실행
        object_spawn_handler,  # [추가] 객체 자동 스폰 정상/비정상 종료 감시
        TimerAction(period=2.0, actions=[spawn_robot]),
    ]


def generate_launch_description():
    # ros2 launch 진입점.
    return LaunchDescription([
        DeclareLaunchArgument(
            "grasp_assist", default_value="false",
            description="true: 물체 붙이기(DetachableJoint) 파지 보조, false: 순수 마찰 파지"),
        OpaqueFunction(function=setup_simulation),
    ])


# ================================================================
# 학습용 실행 흐름 설명
# ================================================================
# 1. ros2 launch로 이 파일을 실행한다.
# 2. OpaqueFunction이 setup_simulation()을 호출한다.
# 3. Xacro를 URDF로 변환하고 check_urdf로 구조를 검사한다.
# 4. robot_state_publisher, clock_bridge, Gazebo를 실행한다.
# 5. TimerAction으로 약 2초 후 Gazebo에 로봇 생성을 요청한다.
# 6. 로봇 생성 프로세스가 성공 종료하면 after_spawn()이 호출된다.
# 7. 컨트롤러 초기화, 4개 객체 스폰, 카메라 스폰, 영상 브리지를 시작한다.
# 8. 객체 스폰 프로세스 종료 시 after_objects_spawn()이 로그를 출력한다.
# 9. Gazebo에 실제 객체가 존재하는지는 gz model --list로 별도 확인한다.
#
# 작성 이유:
# - 로봇·센서·작업 객체를 하나의 Launch 명령으로 실행하기 위해서다.
# - 객체가 안 보일 때 스폰 프로세스 자체가 실패했는지 진단하기 위해서다.
#
# 주의:
# - 현재 로봇 스폰/센서/객체 실행 순서의 구조는 기존 코드와 같다.
# - 이 수정은 로깅 기능만 추가하며 객체 누락의 근본 원인을 자동 해결하지 않는다.
# - 실제 사용자 컴퓨터의 파일은 자동 변경되지 않으므로 파일을 복사해야 한다.
