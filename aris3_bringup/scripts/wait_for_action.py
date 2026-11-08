#!/usr/bin/env python3
"""주어진 ROS 2 액션 서버가 그래프에 나타날 때까지 기다렸다가 종료한다 (launch 순서 맞추기용).

  wait_for_action.py /e0509_gripper/arm_controller/follow_joint_trajectory [--timeout 180]
종료 코드 0 = 나타남, 1 = 시간 초과. launch 의 OnProcessExit 로 다음 단계를 시작한다.
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import get_action_names_and_types


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action')
    ap.add_argument('--timeout', type=float, default=180.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node('aris3_wait_for_action')
    end = time.monotonic() + args.timeout
    try:
        while time.monotonic() < end:
            if any(name == args.action for name, _ in get_action_names_and_types(node)):
                print(f'[ARIS3] 준비됨: {args.action}', flush=True)
                return 0
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f'[ARIS3] 시간 초과 ({args.timeout:.0f} s): {args.action}', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
