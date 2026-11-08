#!/usr/bin/env python3

# ============================================================
# ARIS3 - PHASE 3 / STEP 04
# 파일명: object_marker_publisher.py
#
# [개발 목적]
# CSV에 저장된 정육면체 4개의 위치와 색상을
# RViz2에서 확인할 수 있도록 MarkerArray로 발행한다.
#
# [실행 흐름]
# CSV 읽기 → Marker 생성 → ROS 2 토픽 발행 → RViz2 표시
#
# [주의사항]
# Marker는 실제 물리 객체가 아니라 시각화용 객체다.
# ============================================================

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray


# CSV에 저장된 색상 이름을 RGBA 값으로 변환한다.
COLORS = {
    "red": (1.0, 0.0, 0.0, 1.0),
    "blue": (0.0, 0.0, 1.0, 1.0),
    "yellow": (1.0, 1.0, 0.0, 1.0),
    "green": (0.0, 1.0, 0.0, 1.0),
}


class ObjectMarkerPublisher(Node):
    """CSV의 객체 배치를 RViz2 MarkerArray로 발행하는 ROS 2 노드."""

    def __init__(self):
        # ROS 2 노드 생성
        super().__init__("object_marker_publisher")

        # 설치된 aris3_bringup 패키지의 CSV 경로
        self.csv_file = (
            Path(get_package_share_directory("aris3_bringup"))
            / "config/object_layout.csv"
        )

        # MarkerArray를 발행할 Publisher 생성
        self.publisher = self.create_publisher(
            MarkerArray,
            "/aris3/object_markers",
            10,
        )

        # 1초마다 Marker를 발행하는 타이머
        self.timer = self.create_timer(
            1.0,
            self.publish_markers,
        )

        self.get_logger().info(
            f"Object Marker Publisher started: {self.csv_file}"
        )

    def publish_markers(self):
        # CSV의 각 행을 딕셔너리로 읽는다.
        with self.csv_file.open(encoding="utf-8") as file:
            objects = list(csv.DictReader(file))

        # 여러 개의 Marker를 담는 메시지
        marker_array = MarkerArray()

        for index, obj in enumerate(objects):
            marker = Marker()

            # RViz2에서 사용할 기준 좌표계
            marker.header.frame_id = "world"

            # 현재 ROS 2 시간
            marker.header.stamp = self.get_clock().now().to_msg()

            # Marker 그룹 이름
            marker.ns = "aris3_objects"

            # 각 객체의 고유 ID
            marker.id = index

            # 정육면체 표시
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # CSV에 저장된 월드 좌표
            marker.pose.position.x = float(obj["x"])
            marker.pose.position.y = float(obj["y"])
            marker.pose.position.z = float(obj["z"])

            # 회전하지 않은 기본 자세
            marker.pose.orientation.w = 1.0

            # 객체의 실제 크기(m)
            marker.scale.x = float(obj["width"])
            marker.scale.y = float(obj["width"])
            marker.scale.z = float(obj["height"])

            # 객체 색상
            rgba = COLORS[obj["color"]]
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            marker.color.a = rgba[3]

            # lifetime=0이면 삭제할 때까지 유지
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        # MarkerArray 토픽 발행
        self.publisher.publish(marker_array)


def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)

    # Marker Publisher 노드 생성
    node = ObjectMarkerPublisher()

    try:
        # 노드를 종료할 때까지 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# ============================================================
# [전체 실행 흐름]
#
# 1. ROS 2 노드 시작
# 2. object_layout.csv 파일 위치 확인
# 3. MarkerArray Publisher 생성
# 4. CSV에서 정육면체 4개의 좌표와 크기 읽기
# 5. 각 객체를 Marker.CUBE로 변환
# 6. world 좌표계 기준으로 MarkerArray 발행
# 7. RViz2에서 /aris3/object_markers 토픽 구독
#
# [작성 이유]
# Gazebo에 존재하는 정육면체의 설계 좌표를
# RViz2에서 로봇과 함께 시각적으로 검증하기 위함이다.
#
# [주의]
# CSV 기반 마커는 Gazebo의 실제 이동 상태를 추적하지
# 않으므로 물체가 움직이면 표시 위치와 차이가 날 수 있다.
# ============================================================
