#!/usr/bin/env python3
"""실물 비슷한 시뮬레이션 전용: 카메라 보정 자동 만들기 (2026-11-10).

실물에서는 「① 큐브 픽셀 기억 -> 조그로 집는 자세 -> ② 로봇 위치 기록」 을 사람이 4 점 이상 한다.
시뮬레이션에서는 큐브의 정답 위치를 알므로, 큐브 하나를 Gazebo 에서 여러 자리로 옮기며
  색 검출 픽셀 (u, v)  <->  로봇 베이스 좌표 (x, y) [mm], 집는 높이 z [mm]
를 짝지어 aris3_real_bridge 와 같은 형식으로 저장한다. 보정 결과(오차)는 실물과 같은 방법으로 계산된다.
사람이 하는 ①② 절차는 GUI 에서 그대로 연습할 수 있다 (이 스크립트는 빠른 시험용).

  python3 scripts/aris3_sim_auto_calib.py                    # ~/.config/aris3/sim_camera_calib.json
  ros2 service call /aris3/real/calibration aris3_interfaces/srv/RealCalibration "{command: 5}"   # 브리지에 불러오기

전제: ros2 launch aris3_bringup aris3_real_sim.launch.py 실행 중 (색 검출이 /aris3/vision/detections_px 발행).
      팔이 카메라 시야를 가리지 않는 자세(홈)여야 한다 - 가상 컨트롤러는 처음에 팔을 위로 세운 자세로 시작한다.
점의 로봇 좌표는 Gazebo 정답 위치(/aris3/sim/truth_objects)를 쓴다 - 옮긴 큐브가 구르거나 밀릴 수 있어서.
좌표: 시뮬레이션 로봇 베이스는 world (0, 0, 0.75) 에 회전 없이 놓여 있어 베이스 XY = world XY.
집는 높이 z = 큐브 중심 높이(탁자 위 25 mm) + 플랜지 -> 집는 점 105 mm (gz_gripper_bridge tcp_offset_m).
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from aris3_interfaces.msg import DetectedObjectArray

REAL_CALIB = os.path.expanduser('~/.config/aris3/real_camera_calib.json')
SIM_CALIB = os.path.expanduser('~/.config/aris3/sim_camera_calib.json')
TABLE_Z, CUBE_HALF, TCP_OFFSET = 0.75, 0.025, 0.105
GRASP_Z_MM = (CUBE_HALF + TCP_OFFSET) * 1000.0         # 130 mm (플랜지 높이, 베이스 기준)
# 큐브를 옮겨 볼 자리 (world = 베이스 XY [m]): 카메라 시야 안, 로봇 앞 작업 영역을 고르게.
# 시뮬레이션 탁자는 1.4 m x 1.4 m (x, y = -0.70 ~ 0.70) 라 가장자리(0.70)에 놓으면 큐브가 기울거나 떨어진다
# (2026-11-10 첫 보정 오차 평균 17~54 mm 의 원인) -> 안쪽 0.62 까지만. 홈 자세 팔이 가리는 x < 0.45 근처는 피함.
SPOTS = [(0.48, 0.28), (0.62, 0.28), (0.62, -0.28), (0.48, -0.28), (0.55, 0.0), (0.60, 0.14), (0.52, -0.14),
         (0.62, 0.0)]
# 끝나고 큐브를 되돌릴 자리 (aris3_bringup/config/object_layout_real_like.csv 와 같음)
HOME = {'red_cube': (0.55, 0.15), 'blue_cube': (0.62, -0.05), 'yellow_cube': (0.50, -0.20)}


def set_pose(name, x, y, z=TABLE_Z + CUBE_HALF):
    """Gazebo 큐브를 (x, y) 로 옮긴다. 성공하면 True."""
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
    r = subprocess.run(['gz', 'service', '-s', '/world/empty/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True, timeout=6)
    return 'data: true' in r.stdout


class Watcher(Node):
    """색 검출 결과를 받아 두는 노드."""

    def __init__(self):
        """/aris3/vision/detections_px 구독."""
        super().__init__('aris3_sim_auto_calib')
        self.objs, self.stamp = [], 0.0
        self.truth = {}                 # Gazebo 정답 위치 {이름: (x, y)} [m] - 옮긴 뒤 실제로 놓인 자리
        self.create_subscription(DetectedObjectArray, '/aris3/vision/detections_px', self._on, 10)
        self.create_subscription(DetectedObjectArray, '/aris3/sim/truth_objects', self._on_truth, 10)

    def _on_truth(self, m):
        """정답 위치 기억 (명령한 자리가 아니라 실제로 멈춘 자리를 쓰기 위해: 큐브가 밀리거나 구를 수 있음)."""
        self.truth.update({o.id: (o.pose.pose.position.x, o.pose.pose.position.y) for o in m.objects})

    def _on(self, m):
        """마지막 검출 목록과 받은 시각."""
        self.objs, self.stamp = list(m.objects), time.monotonic()

    def pixel_of(self, color, settle=1.0, timeout=5.0):
        """settle 초 기다린 뒤 그 색 검출이 정확히 하나면 (u, v), 아니면 None."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - self.stamp < 0.5:
                same = [o for o in self.objs if o.color == color]
                if len(same) == 1:
                    return same[0].pose.pose.position.x, same[0].pose.pose.position.y
        return None


def main():
    """큐브를 옮기며 점을 모으고, 맞춰 본 뒤 저장. 실물 보정 파일에는 절대 쓰지 않는다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--cube', default='red_cube', help='옮길 큐브 (검출 색은 이름 앞부분)')
    ap.add_argument('--out', default=SIM_CALIB)
    a = ap.parse_args()
    if os.path.abspath(os.path.expanduser(a.out)) == os.path.abspath(REAL_CALIB):
        sys.exit('실물 보정 파일에는 쓰지 않습니다 (시뮬레이션 전용 스크립트)')
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'aris3_real_bridge'))
    from aris3_real_bridge.real_pick import PixelCalibration

    rclpy.init()
    w = Watcher()
    color = a.cube.split('_')[0]
    others = [n for n in HOME if n != a.cube]
    for n in others:                                   # 다른 큐브는 시야 밖으로 (검출이 섞이지 않게)
        set_pose(n, -0.6, 0.6 if n == others[0] else -0.6)
    c = PixelCalibration()
    for x, y in SPOTS:
        if not set_pose(a.cube, x, y):
            print(f'옮기기 실패: {a.cube} -> ({x}, {y})')
            continue
        px = w.pixel_of(color)
        if px is None:
            print(f'({x:.2f}, {y:.2f}): {color} 검출 없음 (팔이 가리는지 확인) - 건너뜀')
            continue
        if a.cube not in w.truth:
            print('  ※ 정답 위치(/aris3/sim/truth_objects)가 없어 명령한 자리를 씀 - 큐브가 밀렸으면 오차가 커짐')
        tx, ty = w.truth.get(a.cube, (x, y))              # 실제로 놓인 자리 (정답)
        if math.hypot(tx - x, ty - y) > 0.01:
            print(f'  ({x:.2f}, {y:.2f}) 에 놓았지만 ({tx:.3f}, {ty:.3f}) 에 멈춤 - 실제 자리를 씀')
        c.add(px[0], px[1], tx * 1000.0, ty * 1000.0, GRASP_Z_MM, label=f'{a.cube}@{tx:.3f},{ty:.3f}')
        print(f'점 {len(c.points)}: 픽셀 ({px[0]:.1f}, {px[1]:.1f}) -> 로봇 ({tx * 1000:.0f}, {ty * 1000:.0f}) mm')
    for n, (x, y) in HOME.items():                     # 큐브를 원래 자리로
        set_pose(n, x, y)
    if not c.ready():
        sys.exit(f'점이 부족함 ({len(c.points)}개) - 색 검출·카메라 확인')
    print(c.summary())
    print('저장:', c.save(os.path.expanduser(a.out)))
    w.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
