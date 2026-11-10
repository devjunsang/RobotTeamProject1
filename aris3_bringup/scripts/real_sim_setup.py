#!/usr/bin/env python3
"""실물 비슷한 시뮬레이션 준비 (aris3_real_sim.launch.py 가 실행, 2026-11-10).

simulation.launch.py 가 arm_controller:=dsr_position_controller 로 로봇·물체를 띄운 뒤에:
  1. 팔 제어기 dsr_position_controller 가 켜질 때까지 기다린다
     (팔은 MoveIt 이 아니라 두산 가상 컨트롤러가 움직인다: dsr_joint_mirror 가 관절 각도를 넘김.
      처음에는 MoveIt 용 제어기를 켠 뒤 바꿨는데, start_controllers 의 마지막 확인과 겹쳐 거짓 FAIL 이 났다)
  2. Gazebo 를 재생 상태로 둔다 (simulation.launch.py 는 일시정지로 시작)
  3. 큐브를 실물 작업대처럼 로봇 앞쪽(config/object_layout_real_like.csv)으로 옮긴다
     (기존 spawn_objects.py·object_layout.csv 는 MoveIt 시뮬레이션이 쓰므로 건드리지 않음)
"""
import csv
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

CM = '/e0509_gripper/controller_manager'
WORLD = 'empty'
LAYOUT = Path(get_package_share_directory('aris3_bringup')) / 'config/object_layout_real_like.csv'


def run(cmd, timeout=15):
    """명령 실행, (종료 코드, 출력) 을 돌려준다. 시간 초과면 (-1, '')."""
    try:
        r = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return -1, ''


def controller_states():
    """{제어기 이름: 상태} (ros2 control list_controllers 출력)."""
    code, out = run(['ros2', 'control', 'list_controllers', '-c', CM], timeout=10)
    if code != 0:
        return {}
    states = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            states[parts[0]] = parts[-1]
    return states


def gz_world(req):
    """Gazebo 월드 제어 요청 (예: 'pause: false', 'multi_step: 100')."""
    return run(['gz', 'service', '-s', f'/world/{WORLD}/control', '--reqtype', 'gz.msgs.WorldControl',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req], timeout=6)


def wait_controller(name, timeout=180):
    """simulation.launch.py 의 start_controllers 가 제어기를 켤 때까지 기다린다."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if controller_states().get(name) == 'active':
            return True
        time.sleep(1.0)
    return False


def place_cubes():
    """큐브를 실물 비슷한 자리로 옮긴다. 큐브가 아직 생성 전이면 몇 번 다시 시도."""
    with LAYOUT.open(encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        req = (f'name: "{row["name"]}", position: {{x: {row["x"]}, y: {row["y"]}, z: {row["z"]}}}, '
               'orientation: {w: 1}')
        for _ in range(20):
            code, out = run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
                             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req], timeout=6)
            if code == 0 and 'data: true' in out:
                print(f'[ARIS3 real-sim] {row["name"]} -> ({row["x"]}, {row["y"]})', flush=True)
                break
            time.sleep(1.0)
        else:
            raise RuntimeError(f'큐브를 옮기지 못함: {row["name"]}')


def main():
    """준비 단계를 차례로 실행. 실패하면 이유를 출력하고 종료 코드 1."""
    print('[ARIS3 real-sim] 팔 제어기(dsr_position_controller) 준비를 기다림...', flush=True)
    if not wait_controller('dsr_position_controller'):
        raise RuntimeError('dsr_position_controller 가 켜지지 않음 (simulation.launch.py 기록 확인)')
    print('[ARIS3 real-sim] 팔 제어기: dsr_position_controller (두산 가상 컨트롤러가 움직임)', flush=True)
    gz_world('pause: false')
    place_cubes()
    print('[PASS] ARIS3 실물 비슷한 시뮬레이션 준비 완료 - GUI: control_station --mode real --sim', flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # noqa: BLE001 - launch 기록에 이유를 남기고 끝낸다
        print(f'[FAIL] ARIS3 real-sim: {exc}', file=sys.stderr, flush=True)
        sys.exit(1)
