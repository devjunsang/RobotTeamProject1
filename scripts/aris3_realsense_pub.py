#!/usr/bin/env python3
"""RealSense D455 컬러 + 컬러에 맞춘 깊이 발행 (pyrealsense2, 2026-11-13).

aris3_usb_camera_pub.py (V4L2 컬러만) 대신 쓰는 카메라 노드. 컬러 토픽 이름·해상도·JPEG 형식이 같아서
기존 화면·색 검출·카메라 보정(픽셀 -> 로봇 좌표)을 그대로 쓴다. 여기에 깊이를 더해 큐브가 몇 층으로
쌓였는지 계산할 수 있게 한다 (scripts/aris3_color_detector.py 가 받음).

  발행  /aris3/overhead_camera/image/compressed   컬러 JPEG (1280x720, 30 Hz)
        /aris3/overhead_camera/depth/compressed   컬러 픽셀에 맞춘 깊이 [mm], 16 bit PNG, 가로·세로 1/2 (640x360, 10 Hz)
                                                  format = '16UC1; png; scale=2' (원래 픽셀 = 깊이 픽셀 x 2)
        /aris3/overhead_camera/camera_info        컬러 카메라 내부 파라미터 (fx, fy, cx, cy - 1280x720 기준, 깊이와 같이 10 Hz)
                                                  깊이 픽셀을 3D 점으로 바꿔 탁자 평면에서 큐브까지 수직 높이를 재는 데 씀

깊이를 반으로 줄여 보내는 이유: 1280x720 16 bit PNG 는 한 장 0.5 MB 를 넘을 수 있어 Fast DDS 가 UDP 조각으로
나누다 버린다 (aris3_usb_camera_pub.py 설명 참고). 큐브(약 35 픽셀)의 층 계산에는 1/2 로도 충분하다.

설치 (처음 한 번, rclpy 를 쓰려고 시스템 패키지를 같이 봄):
  python3 -m venv --system-site-packages --without-pip ~/venvs/aris3_camera
  python3 -m pip --python ~/venvs/aris3_camera/bin/python install pyrealsense2

실행 (aris3_usb_camera_pub.py 와 동시에 띄우지 않는다 - 같은 카메라를 연다):
  ~/venvs/aris3_camera/bin/python scripts/aris3_realsense_pub.py
"""
import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

DEPTH_SCALE_DOWN = 2            # 깊이 영상을 가로·세로 이만큼 줄여 보냄


class RealSensePub(Node):
    """카메라에서 컬러·깊이 장을 받아 컬러는 JPEG, 깊이는 컬러에 맞춘 뒤 줄여서 PNG 로 보낸다."""

    def __init__(self, a):
        super().__init__('aris3_realsense_pub')
        self.a = a
        self.jpeg = [int(cv2.IMWRITE_JPEG_QUALITY), a.jpeg_quality]
        self.pub_color = self.create_publisher(CompressedImage, a.topic + '/compressed', qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(CompressedImage, a.depth_topic + '/compressed', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, a.info_topic, 10)

        # 카메라 열기: 컬러 = 기존 V4L2 노드와 같은 1280x720 (보정 픽셀이 같아야 함), 깊이 = D455 권장 848x480
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, a.fps)
        prof = self.pipe.start(cfg)
        # 깊이 단위 [m/값] -> mm 로 바꿀 배율 (D455 기본 0.001 = 1 mm)
        self.depth_to_mm = prof.get_device().first_depth_sensor().get_depth_scale() * 1000.0
        # 컬러 내부 파라미터 (깊이를 컬러에 맞추므로 깊이 픽셀도 이 값으로 3D 로 바꾼다)
        k = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.info = CameraInfo()
        self.info.width, self.info.height = k.width, k.height
        self.info.k = [k.fx, 0.0, k.ppx, 0.0, k.fy, k.ppy, 0.0, 0.0, 1.0]
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [float(c) for c in k.coeffs[:5]]
        self.info.header.frame_id = a.frame_id
        self.get_logger().info(f'컬러 내부 파라미터 fx {k.fx:.1f} fy {k.fy:.1f} cx {k.ppx:.1f} cy {k.ppy:.1f}')
        # 깊이를 컬러 픽셀 격자에 맞춤 (같은 (u, v) 에서 컬러와 깊이를 같이 읽으려고)
        self.align = rs.align(rs.stream.color)
        # 깊이 잡음 줄이기: 공간(이웃 픽셀) + 시간(앞 장들) 평균. 구멍 메우기는 큐브 가장자리에 가짜 값을 만들어 쓰지 않음
        self.filters = [rs.spatial_filter(), rs.temporal_filter()]
        self.last_depth_t = 0.0
        self.count, self.kb_c, self.kb_d, self.n_d, self.t0 = 0, 0.0, 0.0, 0, time.monotonic()
        self.get_logger().info(f'RealSense {a.width}x{a.height} {a.fps} Hz -> {a.topic}/compressed, '
                               f'깊이(컬러 맞춤, 1/{DEPTH_SCALE_DOWN}, {a.depth_rate:.0f} Hz) -> '
                               f'{a.depth_topic}/compressed')

    def spin_capture(self):
        """장마다: 컬러 JPEG 발행, depth_rate 간격으로 깊이를 걸러·맞춰·줄여 PNG 발행."""
        while rclpy.ok():
            try:
                frames = self.pipe.wait_for_frames(2000)
            except RuntimeError:
                self.get_logger().warn('영상 받기 실패 - 카메라 연결 확인', throttle_duration_sec=2.0)
                continue
            stamp = self.get_clock().now().to_msg()
            now = time.monotonic()
            send_depth = now - self.last_depth_t >= 1.0 / self.a.depth_rate
            # 컬러 발행
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())
            ok, buf = cv2.imencode('.jpg', img, self.jpeg)
            if ok:
                self._publish(self.pub_color, stamp, 'jpeg', buf, self.a.frame_id)
                self.kb_c += len(buf) / 1024
            if send_depth:
                self.last_depth_t = now
                # 깊이를 컬러 격자로 맞춘 뒤 잡음 거르기 (값 0 = 깊이 없음은 그대로 둠 - 검출 쪽에서 뺀다)
                aligned = self.align.process(frames).get_depth_frame()
                for f in self.filters:
                    aligned = f.process(aligned)
                if aligned:
                    d = np.asanyarray(aligned.get_data()).astype(np.float32) * self.depth_to_mm
                    d = cv2.resize(np.clip(d, 0, 65535).astype(np.uint16), None, fx=1 / DEPTH_SCALE_DOWN,
                                   fy=1 / DEPTH_SCALE_DOWN, interpolation=cv2.INTER_NEAREST)  # 경계 값이 섞이지 않게
                    ok, buf = cv2.imencode('.png', d)
                    if ok:
                        self._publish(self.pub_depth, stamp, f'16UC1; png; scale={DEPTH_SCALE_DOWN}', buf,
                                      self.a.frame_id)
                        self.kb_d += len(buf) / 1024
                        self.info.header.stamp = stamp
                        self.pub_info.publish(self.info)
                        self.n_d += 1
            self.count += 1
            if now - self.t0 > 10.0:
                dt = now - self.t0
                self.get_logger().info(f'컬러 {self.count / dt:.1f} Hz (평균 {self.kb_c / max(self.count, 1):.0f} KB), '
                                       f'깊이 {self.n_d / dt:.1f} Hz (평균 {self.kb_d / max(self.n_d, 1):.0f} KB)')
                self.count, self.kb_c, self.kb_d, self.n_d, self.t0 = 0, 0.0, 0.0, 0, now

    def _publish(self, pub, stamp, fmt, buf, frame_id):
        """인코딩한 영상 버퍼를 CompressedImage 로 발행."""
        m = CompressedImage()
        m.header.stamp, m.header.frame_id = stamp, frame_id
        m.format = fmt
        m.data = buf.tobytes()
        pub.publish(m)

    def close(self):
        """카메라 닫기 (다른 노드가 다시 열 수 있게)."""
        self.pipe.stop()


def main():
    """인자 읽기 -> 노드 만들기 -> Ctrl+C 까지 영상 발행."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--depth-rate', type=float, default=10.0, help='깊이 발행 속도 [Hz]')
    ap.add_argument('--topic', default='/aris3/overhead_camera/image', help='컬러 토픽 (JPEG 은 <토픽>/compressed)')
    ap.add_argument('--depth-topic', default='/aris3/overhead_camera/depth')
    ap.add_argument('--info-topic', default='/aris3/overhead_camera/camera_info')
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--frame-id', default='overhead_camera_color_optical_frame')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RealSensePub(a)
    try:
        node.spin_capture()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
