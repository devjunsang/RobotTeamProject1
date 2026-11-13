#!/usr/bin/env python3

import subprocess
import sys
import time
from pathlib import Path

CM = "/e0509_gripper/controller_manager"
WORLD = "/world/empty/control"

from ament_index_python.packages import get_package_share_directory

CONFIG = (
    Path(get_package_share_directory("aris3_bringup"))
    / "config/gripper_controller.yaml"
)


def run(cmd, timeout=30):
    print("[ARIS3] RUN:", " ".join(cmd), flush=True)
    return subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def require(cmd, timeout=30):
    result = run(cmd, timeout)
    if result.stdout:
        print(result.stdout, flush=True)
    if result.stderr:
        print(result.stderr, flush=True)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}")
    return result


def wait_hardware(timeout=90):
    print("[ARIS3] Waiting for both hardware systems...", flush=True)
    end = time.monotonic() + timeout

    while time.monotonic() < end:
        try:
            result = run([
                "ros2", "control", "list_hardware_components",
                "-c", CM,
            ], timeout=8)

            output = result.stdout

            if (
                result.returncode == 0
                and "name: e0509_gripper_gz" in output
                and "name: gripper_gz" in output
                and output.count("label=active") >= 2
            ):
                interfaces = run([
                    "ros2", "control", "list_hardware_interfaces",
                    "-c", CM,
                ], timeout=8)

                if (
                    interfaces.returncode == 0
                    and "joint_1/position [available]" in interfaces.stdout
                    and "gripper_rh_r1/position [available]" in interfaces.stdout
                ):
                    print("[PASS] Both hardware systems ready", flush=True)
                    return
        except subprocess.TimeoutExpired:
            pass

        time.sleep(1)

    raise RuntimeError("Hardware initialization timed out")


def load_inactive(name, config=None):
    cmd = [
        "ros2", "run", "controller_manager", "spawner",
        name,
        "-c", CM,
        "--controller-manager-timeout", "30",
        "--inactive",
    ]

    if config:
        cmd += ["--param-file", str(config)]

    require(cmd, timeout=45)


def step_gazebo():
    result = run([
        "gz", "service",
        "-s", WORLD,
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", "multi_step: 100",
    ], timeout=7)

    if result.returncode != 0 or "data: true" not in result.stdout:
        raise RuntimeError("Gazebo step request failed")


def activate(name):
    print(f"[ARIS3] Activating {name}", flush=True)

    proc = subprocess.Popen(
        [
            "ros2", "control", "set_controller_state",
            name, "active", "-c", CM,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        deadline = time.monotonic() + 20

        while proc.poll() is None and time.monotonic() < deadline:
            step_gazebo()
            time.sleep(0.1)

        if proc.poll() is None:
            proc.kill()
            proc.wait()
            raise RuntimeError(f"Activation timeout: {name}")

        output = proc.communicate()[0]
        print(output, flush=True)

        if proc.returncode != 0:
            raise RuntimeError(f"Activation failed: {name}")

        print(f"[PASS] {name} active", flush=True)

    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def arm_controller_name():
    """켤 팔 제어기: --arm <이름> (기본 arm_controller = MoveIt 용).

    실물 비슷한 시뮬레이션(aris3_real_sim.launch.py)은 dsr_position_controller 를 켠다
    (두산 가상 컨트롤러의 관절 각도를 그대로 따라감). 둘은 같은 관절을 쓰므로 하나만 켠다.
    """
    args = sys.argv[1:]
    if '--arm' in args and args.index('--arm') + 1 < len(args):
        name = args[args.index('--arm') + 1]
        if name in ('arm_controller', 'dsr_position_controller'):
            return name
        raise RuntimeError(f'모르는 팔 제어기: {name}')
    return 'arm_controller'


def main():
    arm = arm_controller_name()
    if not CONFIG.is_file():
        raise RuntimeError(f"Missing configuration: {CONFIG}")

    wait_hardware()

    load_inactive("joint_state_broadcaster")
    # 팔: MoveIt 2 실행용 궤적 컨트롤러를 켠다.
    # dsr_position_controller(관절 직접 시험용)는 같은 관절을 쓰므로 inactive 로만 올려 둔다.
    # (설정: aris3_bringup/config/gz_controllers.yaml)
    load_inactive("arm_controller")
    load_inactive("dsr_position_controller")
    load_inactive("gripper_position_controller", CONFIG)

    activate("joint_state_broadcaster")
    activate(arm)
    activate("gripper_position_controller")

    result = require([
        "ros2", "control", "list_controllers",
        "-c", CM,
    ])

    expected = (
        "joint_state_broadcaster",
        arm,
        "gripper_position_controller",
    )

    for name in expected:
        matches = [
            line for line in result.stdout.splitlines()
            if line.split()[:1] == [name]
        ]
        if not matches or matches[0].split()[-1] != "active":
            raise RuntimeError(f"Controller not active: {name}")

    print("[PASS] ARIS3 CONTROL INITIALIZATION COMPLETE", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[FAIL] ARIS3: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
