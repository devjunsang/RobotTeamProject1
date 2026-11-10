"""Gazebo 카메라 원본 영상 -> JPEG 압축 영상 (2026-11-10, 실물 비슷한 시뮬레이션용).

실물 위쪽 카메라는 scripts/aris3_usb_camera_pub.py 가 JPEG(품질 80)만 /aris3/overhead_camera/image/compressed 로
보낸다 (원본은 보내지 않음). 시뮬레이션도 같은 토픽·같은 형식으로만 내보내서, 색 검출·손짓·GUI 가 실물과 같은
입력을 받게 한다.

  구독: input  (sensor_msgs/Image, 기본 /aris3/sim/real_camera/image - Gazebo 카메라, 다른 노드는 쓰지 않음)
  발행: output (sensor_msgs/CompressedImage, 기본 /aris3/overhead_camera/image/compressed)
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def to_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8) -> OpenCV BGR 배열. 모르는 형식이면 None."""
    img = np.frombuffer(bytes(msg.data), np.uint8)
    if msg.encoding not in ('rgb8', 'bgr8') or img.size != msg.height * msg.step:
        return None
    img = img.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img


class JpegRepublisher(Node):
    """원본 영상을 받아 JPEG 로 압축해 다시 보내는 노드 (처리 빈도 제한)."""

    def __init__(self):
        """파라미터: input, output, quality(80 = 실물 카메라 노드와 같음), max_hz."""
        super().__init__('aris3_jpeg_republisher')
        self.declare_parameter('input', '/aris3/sim/real_camera/image')
        self.declare_parameter('output', '/aris3/overhead_camera/image/compressed')
        self.declare_parameter('quality', 80)
        self.declare_parameter('max_hz', 30.0)
        self.quality = int(self.get_parameter('quality').value)
        self.period = 1.0 / max(1.0, float(self.get_parameter('max_hz').value))
        self.last = 0.0
        self.pub = self.create_publisher(CompressedImage, self.get_parameter('output').value, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('input').value, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        """원본 한 장 -> JPEG 한 장 (max_hz 보다 빨리 오면 건너뜀)."""
        now = time.monotonic()
        if now - self.last < self.period:
            return
        bgr = to_bgr(msg)
        if bgr is None:
            self.get_logger().warn(f'모르는 영상 형식: {msg.encoding}', throttle_duration_sec=10.0)
            return
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        self.last = now
        out = CompressedImage(format='jpeg', data=buf.tobytes())
        out.header = msg.header
        self.pub.publish(out)


def main():
    """노드 실행."""
    rclpy.init()
    node = JpegRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
