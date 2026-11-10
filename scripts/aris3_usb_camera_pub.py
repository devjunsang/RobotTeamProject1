#!/usr/bin/env python3
"""USB(UVC) 카메라 영상 -> /aris3/overhead_camera/image/compressed (JPEG, 임시, 2026-11-09).

RealSense ROS 드라이버(ros-jazzy-realsense2-camera)를 설치하기 전까지, D455 의 컬러 영상을
V4L2 로 직접 읽어 aris3_gui 카메라 화면에 띄우기 위한 보조 노드. 깊이·카메라 내부 파라미터는 없다.
화면 클릭 -> 탁자 좌표 변환은 카메라 TF(보정)가 있어야 동작하므로 이 노드만으로는 영상 표시까지만 된다.

  python3 scripts/aris3_usb_camera_pub.py                      # D455 컬러 노드 자동 선택
  python3 scripts/aris3_usb_camera_pub.py --device 0           # 노트북 웹캠
  python3 scripts/aris3_usb_camera_pub.py --rotate 90          # 카메라가 돌아가 있을 때 (90/180/270, 시계 방향)
  python3 scripts/aris3_usb_camera_pub.py --width 848 --height 480   # 해상도 낮추기

원본(raw) 영상은 기본으로 보내지 않는다 (--raw 로 켬). 1280x720 bgr8 은 한 장 2.7 MB 인데,
Fast DDS 는 약 0.5 MB 가 넘는 메시지를 공유 메모리 대신 UDP 조각으로 보내고 sensor_data(best effort)
에서는 조각 하나만 잃어도 장 전체를 버린다. 2026-11-09 측정: 320x240(225 KB) 28 Hz, 640x480 이상 0 Hz.
JPEG(품질 80)는 720p 한 장 약 100~200 KB 라 30 Hz 로 전달된다.

D455 는 /dev/video* 를 여러 개 만든다 (깊이·적외선·컬러). 적외선 노드는 YUYV 요청에도 UYVY 로 열려서,
YUYV 로 열리는 RealSense 노드를 컬러로 고른다 (2026-11-09 이 PC: /dev/video6).
"""
import argparse
import glob
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

ROTATE = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def fourcc_str(cap):
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    return ''.join(chr((v >> 8 * k) & 0xFF) for k in range(4))


def open_camera(index, width, height, fps):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def find_realsense_color(width, height, fps):
    for path in sorted(glob.glob('/sys/class/video4linux/video*'), key=lambda p: int(p.rsplit('video', 1)[1])):
        try:
            name = open(os.path.join(path, 'name')).read()
        except OSError:
            continue
        if 'RealSense' not in name:
            continue
        index = int(path.rsplit('video', 1)[1])
        cap = open_camera(index, width, height, fps)
        if cap is None:
            continue
        ok, frame = cap.read()
        if ok and frame is not None and fourcc_str(cap) == 'YUYV':
            return index, cap
        cap.release()
    return None, None


class CameraPub(Node):
    def __init__(self, cap, topic, frame_id, rotate, raw, jpeg_quality):
        super().__init__('aris3_usb_camera_pub')
        self.cap, self.frame_id, self.rotate = cap, frame_id, rotate
        self.jpeg = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        self.pub_jpeg = self.create_publisher(CompressedImage, topic + '/compressed', qos_profile_sensor_data)
        self.pub_raw = self.create_publisher(Image, topic, qos_profile_sensor_data) if raw else None
        self.kb = 0.0
        self.count, self.t0 = 0, time.monotonic()

    def spin_capture(self):
        while rclpy.ok():
            ok, frame = self.cap.read()            # 카메라 속도에 맞춰 기다린다
            if not ok or frame is None:
                self.get_logger().warn('영상 읽기 실패 - 카메라 연결 확인', throttle_duration_sec=2.0)
                time.sleep(0.2)
                continue
            if self.rotate in ROTATE:
                frame = cv2.rotate(frame, ROTATE[self.rotate])
            stamp = self.get_clock().now().to_msg()
            ok, buf = cv2.imencode('.jpg', frame, self.jpeg)
            if ok:
                c = CompressedImage()
                c.header.stamp, c.header.frame_id = stamp, self.frame_id
                c.format = 'jpeg'
                c.data = buf.tobytes()
                self.pub_jpeg.publish(c)
                self.kb += len(c.data) / 1024
            if self.pub_raw is not None:
                m = Image()
                m.header.stamp, m.header.frame_id = stamp, self.frame_id
                m.height, m.width = frame.shape[:2]
                m.encoding, m.is_bigendian, m.step = 'bgr8', 0, frame.shape[1] * 3
                m.data = frame.tobytes()
                self.pub_raw.publish(m)
            self.count += 1
            if time.monotonic() - self.t0 > 10.0:
                dt = time.monotonic() - self.t0
                self.get_logger().info(f'{frame.shape[1]}x{frame.shape[0]} {self.count / dt:.1f} Hz, '
                                       f'JPEG 평균 {self.kb / max(self.count, 1):.0f} KB')
                self.count, self.t0, self.kb = 0, time.monotonic(), 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', type=int, default=-1, help='/dev/videoN 의 N (기본: RealSense 컬러 자동 선택)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270])
    ap.add_argument('--topic', default='/aris3/overhead_camera/image', help='원본 토픽 (JPEG 은 <토픽>/compressed)')
    ap.add_argument('--raw', action='store_true', help='원본 bgr8 도 보냄 (720p 는 같은 PC 에서도 거의 전달 안 됨)')
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--frame-id', default='overhead_camera_color_optical_frame')
    a, ros_args = ap.parse_known_args()

    if a.device >= 0:
        index, cap = a.device, open_camera(a.device, a.width, a.height, a.fps)
    else:
        index, cap = find_realsense_color(a.width, a.height, a.fps)
    if cap is None:
        raise SystemExit('카메라를 열지 못함 (--device 로 직접 지정, v4l2-ctl --list-devices 로 확인)')

    rclpy.init(args=ros_args)
    node = CameraPub(cap, a.topic, a.frame_id, a.rotate, a.raw, a.jpeg_quality)
    node.get_logger().info(f'/dev/video{index} ({fourcc_str(cap)}) -> {a.topic}/compressed'
                           f'{" + 원본" if a.raw else ""}, 회전 {a.rotate}°')
    try:
        node.spin_capture()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
