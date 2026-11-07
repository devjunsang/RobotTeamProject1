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


def main():
    if not CONFIG.is_file():
        raise RuntimeError(f"Missing configuration: {CONFIG}")

    wait_hardware()

    load_inactive("joint_state_broadcaster")
    load_inactive("dsr_position_controller")
    load_inactive("gripper_position_controller", CONFIG)

    activate("joint_state_broadcaster")
    activate("dsr_position_controller")
    activate("gripper_position_controller")

    result = require([
        "ros2", "control", "list_controllers",
        "-c", CM,
    ])

    expected = (
        "joint_state_broadcaster",
        "dsr_position_controller",
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
