"""RobotTeamProject1 통합 실행 Launch (ROS 2 Jazzy).

실물 / 시뮬레이션 선택:
  ros2 launch aris3_bringup robot.launch.py mode:=sim
  ros2 launch aris3_bringup robot.launch.py mode:=real host:=110.120.1.66 rviz:=false
  ros2 launch aris3_bringup robot.launch.py mode:=auto allow_auto_real:=true interface:=enp3s0

설계 원칙:
  - mode 기본값은 sim: 단순 실행만으로 실제 로봇의 서보를 켜지 않는다.
  - real 모드는 e0509_manual/real.launch.py를 그대로 포함한다.
    따라서 Doosan 실물 드라이버, e0509_real 네임스페이스, GUI, RViz 옵션, z_min 기능이 보존된다.
  - sim 모드는 기존 aris3_bringup/simulation.launch.py를 그대로 포함한다.
  - auto 모드는 사용자가 allow_auto_real:=true를 명시한 경우에만 동작한다.
    ICMP Ping으로 선택할 뿐, 연결 안전성이나 ROS 드라이버 상태를 보장하지는 않는다.
  - 동시에 두 모드를 실행하지 않는다. 다른 실물 Bringup도 별도로 실행하면 안 된다.
"""

import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 실제 로봇을 자동으로 기동하는 것을 방지하기 위해 기본값은 시뮬레이션이다.
DEFAULT_MODE = "sim"


def _is_true(value):
    """Launch 인자에 사용되는 참/거짓 문자열 해석."""
    return str(value).strip().lower() in {"true", "1", "yes", "on"}


def _robot_responds_to_ping(host, interface):
    """ICMP Ping 한 번으로 네트워크 도달 가능성만 조회한다.

    두산의 제어 포트(12345)를 TCP로 열어 보지 않는다. 기존 드라이버의
    제어 연결과 충돌할 수 있으므로 단순 네트워크 진단만 수행한다.
    """
    command = ["ping", "-n", "-c", "1", "-W", "1"]
    if interface:
        command += ["-I", interface]
    command.append(host)
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _include_file(launch_path, launch_args=None):
    """기존 하위 Launch 파일을 재작성하지 않고 포함한다."""
    launch_path = Path(launch_path)
    if not launch_path.is_file():
        raise RuntimeError(f"[RobotTeamProject1] Launch 파일이 없습니다: {launch_path}")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(launch_args or {}).items(),
    )


def _child_launch(package, filename, launch_args=None):
    """패키지에 설치된 Launch 파일을 포함한다."""
    launch_path = Path(get_package_share_directory(package)) / "launch" / filename
    return _include_file(launch_path, launch_args)


def _start_selected_mode(context, *args, **kwargs):
    """실행 시점에 인자를 평가해 단 하나의 모드만 기동한다."""
    requested_mode = LaunchConfiguration("mode").perform(context).strip().lower()
    host = LaunchConfiguration("host").perform(context).strip()
    interface = LaunchConfiguration("interface").perform(context).strip()
    rviz = LaunchConfiguration("rviz").perform(context).strip()
    z_min = LaunchConfiguration("z_min").perform(context).strip()
    allow_auto_real = LaunchConfiguration("allow_auto_real").perform(context).strip()

    if requested_mode not in {"sim", "real", "auto"}:
        raise RuntimeError(
            "[RobotTeamProject1] mode는 sim, real, auto 중 하나여야 합니다."
        )

    selected_mode = requested_mode
    if requested_mode == "auto":
        # 실제 로봇 연결 시 서보가 켜질 수 있으므로 자동 전환을 명시적으로 승인해야 한다.
        if not _is_true(allow_auto_real):
            raise RuntimeError(
                "[RobotTeamProject1] mode:=auto는 실물 서보가 켜질 수 있습니다. "
                "현장을 확인한 후 allow_auto_real:=true를 명시하세요. "
                "실물 연결을 원치 않으면 mode:=sim을 사용하세요."
            )
        selected_mode = "real" if _robot_responds_to_ping(host, interface) else "sim"

    if selected_mode == "sim":
        # 기존 Gazebo 월드, E0509/그리퍼 모델, 작업 물체, 카메라 구성을 그대로 사용한다.
        return [
            LogInfo(msg="[RobotTeamProject1] 시뮬레이션 모드: simulation.launch.py 실행"),
            # 통합 Launch와 같은 디렉터리의 시뮬레이션 파일을 우선 사용한다.
            # 새 팀 프로젝트에서 직접 경로로 실행할 때 기존 ARIS3의
            # simulation.launch.py를 잘못 선택하는 일을 줄이기 위함이다.
            _include_file(Path(__file__).resolve().parent / "simulation.launch.py"),
        ]

    # 실물 모드: E0509 실물 드라이버 + GUI + 선택적 RViz를 기존 구현 그대로 실행한다.
    # e0509_manual/real.launch.py에서 사용하는 namespace는 e0509_real로 유지된다.
    try:
        real = _child_launch(
            "e0509_manual",
            "real.launch.py",
            launch_args={"host": host, "rviz": rviz, "z_min": z_min},
        )
    except Exception as exc:
        raise RuntimeError(
            "[RobotTeamProject1] 실물 모드에 필요한 e0509_manual 패키지를 "
            "찾거나 로드하지 못했습니다. colcon으로 패키지를 빌드하고 "
            "workspace를 source하세요. 상세 오류: " + str(exc)
        ) from exc
    return [
        LogInfo(msg=(
            f"[RobotTeamProject1] 실물 모드: {host}, namespace=e0509_real. "
            "연결 과정에서 서보 활성화/제어 권한 변경 가능; 별도 Bringup 중복 실행 금지"
        )),
        real,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value=DEFAULT_MODE,
            description="실행 모드: sim(기본), real(실물), auto(네트워크 검사 후 선택)",
        ),
        DeclareLaunchArgument(
            "host", default_value="110.120.1.66",
            description="실제 E0509 컨트롤러의 IP 주소",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="실물 real.launch.py에서 RViz2 실행 여부 (true/false)",
        ),
        DeclareLaunchArgument(
            "z_min", default_value="50",
            description="실물 GUI에서 허용하는 최소 TCP 높이 [mm]",
        ),
        DeclareLaunchArgument(
            "interface", default_value="",
            description="auto Ping에 사용할 네트워크 인터페이스 (예: enp3s0; 빈 값은 시스템 라우팅)",
        ),
        DeclareLaunchArgument(
            "allow_auto_real", default_value="false",
            description="auto 모드가 실물 로봇을 기동해도 되는지 명시적으로 승인 (true)",
        ),
        OpaqueFunction(function=_start_selected_mode),
    ])


# ================================================================
# 학습용: 전체 Launch 실행 흐름
# ================================================================
# 1) 사용자: ros2 launch aris3_bringup robot.launch.py mode:=sim|real|auto
# 2) Launch 시스템이 mode/host/rviz/z_min/interface 인자를 평가한다.
# 3) mode:=sim이면 기존 simulation.launch.py만 포함한다.
#    Gazebo에 로봇, 작업 물체, 카메라를 만드는 기존 구현은 수정하지 않는다.
# 4) mode:=real이면 e0509_manual/real.launch.py만 포함한다.
#    이 파일이 Doosan dsr_bringup2, 실제 로봇 연결, 수동 제어 GUI,
#    RViz2 옵션과 e0509_real 네임스페이스를 모두 유지한다.
# 5) mode:=auto이면 allow_auto_real:=true로 현장 승인을 표시한 경우에만
#    ICMP Ping을 한 번 보낸다. 응답이 있으면 real, 없으면 sim으로 선택한다.
#    Ping 성공은 실제 ROS 서비스 연결이나 전원 적합성을 보장하지 않는다.
# 6) 실물 드라이버는 이 Launch에서 한 번만 시작한다. 다른 실물 Bringup과
#    동시에 실행하면 로봇 제어 연결/ROS 2 컨트롤러 충돌이 발생할 수 있다.
# 7) 이 Launcher 자체는 그리퍼에 토크/위치 명령을 보내지 않는다.
#    단, 실물 GUI가 자체적으로 상태 읽기를 시도할 수 있다.
