#!/usr/bin/env python3
"""ARIS3 MoveIt 2 연동 시험 (GATE 4 준비: 계획 + Gazebo 실행 + 도착 확인).

  ros2 run aris3_moveit_config moveit_smoke_test.py [--x 0.45 --y 0.0 --z 1.05]

순서:
  1. 이름 붙은 자세 'home' 으로 관절 목표 이동
  2. 지정한 world 좌표 위로 link_6 이동 (그리퍼가 탁자를 수직으로 내려다보는 자세)
  3. 다시 home
각 이동 뒤 TF(world -> link_6)로 실제 Gazebo 위치를 읽어 목표와의 거리를 출력한다.
하나라도 실패하면 종료 코드 1.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

HOME = [0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0]
JOINTS = [f"joint_{i}" for i in range(1, 7)]
DOWN = (1.0, 0.0, 0.0, 0.0)   # link_6 z축이 world -z 를 향함 (x축 기준 180도), (x, y, z, w)


class SmokeTest(Node):
    def __init__(self):
        super().__init__("aris3_moveit_smoke_test", parameter_overrides=[])
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _request(self, goal_constraints, label):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = "arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [goal_constraints]
        goal.planning_options.plan_only = False
        self.get_logger().info(f"[{label}] 계획+실행 요청")
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[{label}] move_group 이 목표를 받지 않음")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=60.0)
        if res.result() is None:
            self.get_logger().error(f"[{label}] 결과 시간 초과")
            return False
        code = res.result().result.error_code.val
        t = res.result().result.planning_time
        ok = code == MoveItErrorCodes.SUCCESS
        (self.get_logger().info if ok else self.get_logger().error)(
            f"[{label}] {'성공' if ok else '실패'} (error_code={code}, 계획 {t:.2f}s)")
        return ok

    def joint_goal(self, values, label):
        c = Constraints(name=label)
        c.joint_constraints = [JointConstraint(joint_name=j, position=v, tolerance_above=0.01,
                                               tolerance_below=0.01, weight=1.0)
                               for j, v in zip(JOINTS, values)]
        return self._request(c, label)

    def pose_goal(self, x, y, z, label):
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = DOWN
        pc = PositionConstraint(link_name="link_6", weight=1.0)
        pc.header.frame_id = "world"
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        pc.constraint_region.primitive_poses = [target.pose]
        oc = OrientationConstraint(link_name="link_6", orientation=target.pose.orientation,
                                   absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
                                   absolute_z_axis_tolerance=3.15,   # 손목 회전(yaw)은 자유
                                   weight=1.0)
        oc.header.frame_id = "world"
        c = Constraints(name=label, position_constraints=[pc], orientation_constraints=[oc])
        return self._request(c, label)

    def link6(self):
        # 실행 끝난 뒤 TF 가 갱신될 시간을 준다
        end = self.get_clock().now() + Duration(seconds=1.0)
        while self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self.tf.lookup_transform("world", "link_6", Time(), timeout=Duration(seconds=2.0))
        p = t.transform.translation
        return p.x, p.y, p.z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.45)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=1.05)
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = SmokeTest()
    if not n.client.wait_for_server(timeout_sec=10.0):
        n.get_logger().error("/move_action 없음 - move_group.launch.py 실행 확인")
        sys.exit(1)
    results = []
    results.append(("home", n.joint_goal(HOME, "home")))
    ok = n.pose_goal(args.x, args.y, args.z, "pose")
    if ok:
        got = n.link6()
        err = math.dist(got, (args.x, args.y, args.z)) * 1000
        n.get_logger().info(f"[pose] Gazebo link_6 = ({got[0]:.3f}, {got[1]:.3f}, {got[2]:.3f}), "
                            f"목표와 {err:.1f} mm")
        ok = err < 10.0
    results.append(("pose", ok))
    results.append(("home again", n.joint_goal(HOME, "home again")))
    for name, r in results:
        print(f"{'PASS' if r else 'FAIL'}  {name}")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(r for _, r in results) else 1)


if __name__ == "__main__":
    main()
