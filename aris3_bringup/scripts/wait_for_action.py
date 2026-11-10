#!/usr/bin/env python3
"""주어진 ROS 2 액션 서버가 그래프에 나타날 때까지 기다렸다가 종료한다 (launch 순서 맞추기용).

  wait_for_action.py /e0509_gripper/arm_controller/follow_joint_trajectory [--timeout 180]
  wait_for_action.py <액션> --active-controller arm_controller --controller-manager /e0509_gripper/controller_manager
종료 코드 0 = 나타남, 1 = 시간 초과. launch 의 OnProcessExit 로 다음 단계를 시작한다.

--active-controller (2026-11-10): 제어기의 액션 서버는 제어기가 켜지기(active) 전에도 그래프에 보인다.
그래서 액션만 기다리면 MoveIt·Task Manager 가 너무 일찍 떠서, 바로 보낸 작업이 첫 이동에서
CONTROL_FAILED 로 실패했다 (ARIS 자동 시험에서 발견). 이 옵션은 그 제어기가 active 가 될 때까지 더 기다린다.
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import get_action_names_and_types


def controller_active(node, cm, name, timeout=3.0):
    """controller_manager 의 list_controllers 로 name 제어기가 active 인가. 서비스가 없거나 응답이 없으면 False."""
    from controller_manager_msgs.srv import ListControllers
    cli = node.create_client(ListControllers, f'{cm}/list_controllers')
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        r = fut.result()
        return bool(r) and any(c.name == name and c.state == 'active' for c in r.controller)
    finally:
        node.destroy_client(cli)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--active-controller', default='', help='이 제어기가 active 가 될 때까지도 기다림')
    ap.add_argument('--controller-manager', default='/e0509_gripper/controller_manager')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node('aris3_wait_for_action')
    end = time.monotonic() + args.timeout
    try:
        while time.monotonic() < end:
            if any(name == args.action for name, _ in get_action_names_and_types(node)):
                if not args.active_controller or controller_active(node, args.controller_manager,
                                                                    args.active_controller):
                    print(f'[ARIS3] 준비됨: {args.action}' +
                          (f' ({args.active_controller} active)' if args.active_controller else ''), flush=True)
                    return 0
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f'[ARIS3] 시간 초과 ({args.timeout:.0f} s): {args.action}', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
