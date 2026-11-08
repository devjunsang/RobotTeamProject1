#!/usr/bin/env python3
"""ARIS3 작업 셀의 고정 장애물을 MoveIt planning scene 에 등록한다.

등록하는 것: 작업 탁자 상판 (aris3_bringup/worlds/aris3_world.sdf 의 workcell_table 과 같은 크기)
등록하지 않는 것: 집을 물체 9개 - 위치가 바뀌므로 Task Manager 가 비전 결과로 관리한다.

move_group 의 /apply_planning_scene 서비스가 준비될 때까지 기다렸다가 한 번 등록하고 종료한다.
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class WorkcellScene(Node):
    def __init__(self):
        super().__init__("aris3_workcell_scene")
        self.declare_parameter("table_top_z", 0.75)
        self.declare_parameter("table_size_xy", [1.4, 1.4])
        self.declare_parameter("table_thickness", 0.04)
        self.declare_parameter("frame_id", "world")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def table(self):
        top = float(self.get_parameter("table_top_z").value)
        sx, sy = [float(v) for v in self.get_parameter("table_size_xy").value]
        th = float(self.get_parameter("table_thickness").value)
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, th])
        pose = Pose()
        # 윗면을 실제보다 1 mm 낮춘다: 로봇 베이스가 윗면에 딱 붙어 있어 '항상 충돌'로 보이지 않게
        pose.position.z = top - th / 2.0 - 0.001
        pose.orientation.w = 1.0
        obj = CollisionObject()
        obj.header.frame_id = self.get_parameter("frame_id").value
        obj.id = "workcell_table"
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def run(self):
        self.get_logger().info("move_group 의 /apply_planning_scene 기다리는 중...")
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("  아직 없음 - move_group 실행 확인", throttle_duration_sec=10.0)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [self.table()]
        future = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        ok = future.done() and future.result() is not None and future.result().success
        if ok:
            self.get_logger().info("작업 탁자를 planning scene 에 등록함")
        else:
            self.get_logger().error("planning scene 등록 실패")
        return ok


def main():
    rclpy.init()
    node = WorkcellScene()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
