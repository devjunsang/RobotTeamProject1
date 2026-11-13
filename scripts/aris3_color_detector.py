#!/usr/bin/env python3
"""실물 카메라 색 정육면체 검출 - 영상 픽셀 위치만 (시제품, 2026-11-09).

구독: /aris3/overhead_camera/image/compressed  (scripts/aris3_usb_camera_pub.py)
발행: /aris3/vision/detections_px  (aris3_interfaces/DetectedObjectArray)
      header.frame_id = 'overhead_camera_image_px'  ->  pose.position.x, y = 영상 픽셀 (u, v), z = 0
      ※ world 좌표가 아니다. 카메라 보정(카메라 -> world TF)이 없어서 /aris3/detected_objects 에는 보내지 않는다.
        Task Manager 는 이 토픽을 받지 않는다 (GUI 표시용). 보정 후 world 좌표 발행을 이 노드에 더한다.

방법: HSV 색 범위 -> 열기/닫기(잡음 제거) -> 외곽선 -> 넓이·가로세로비·채움 비율로 정육면체만 남김.
색 범위는 2026-11-09 실험실 흰 판 위 영상에서 잰 값 (노랑 큐브 H=16 은 실제로는 주황에 가까움, 빨강 H=175,
파랑 H=107), 피부를 빼려고 빨강·노랑은 채도 하한을 높였다. 조명이 바뀌면 --debug 로 마스크를 보고 다시 맞춘다.
나중에 aris3_vision (C++ OpenCV) 으로 옮길 기준 구현.

  python3 scripts/aris3_color_detector.py
  python3 scripts/aris3_color_detector.py --min-area 300 --max-area 8000 --debug   # 판단 근거를 로그로
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from aris3_interfaces.msg import DetectedObject, DetectedObjectArray

# OpenCV HSV: H 0~180, S·V 0~255. 빨강은 H 가 0 근처와 180 근처로 나뉘어 범위가 두 개.
COLOR_RANGES = {
    # 빨강·노랑 최소 채도(S)를 높게 둔 이유: 사람 피부가 H 3~20, S 100~130 이라 손이 들어오면 빨강으로
    # 잡혔다 (2026-11-09, 4 % 의 장). 큐브는 빨강 S 155 이상, 노랑 S 약 160.
    'red': [((0, 145, 60), (6, 255, 255)), ((165, 145, 60), (180, 255, 255))],
    'yellow': [((10, 140, 120), (35, 255, 255))],
    'green': [((40, 80, 50), (85, 255, 255))],
    'blue': [((95, 100, 50), (130, 255, 255))],
}
PIXEL_FRAME = 'overhead_camera_image_px'


def detect(bgr, min_area, max_area, max_aspect=1.6, min_fill=0.65):
    """영상 -> [(color, u, v, area, aspect, fill, confidence)], 버린 후보 목록(진단용)."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    found, rejected = [], []
    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)     # 작은 점 지우기
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)    # 반사로 생긴 구멍 메우기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area * 0.5:
                continue
            (u, v), (w, h), _ = cv2.minAreaRect(c)
            aspect = max(w, h) / max(min(w, h), 1.0)
            fill = area / max(w * h, 1.0)
            cand = (color, u, v, area, aspect, fill)
            if not (min_area <= area <= max_area and aspect <= max_aspect and fill >= min_fill):
                rejected.append(cand)
                continue
            # 신뢰도: 정사각형에 가깝고(aspect 1) 꽉 찰수록(fill 1) 높다
            conf = max(0.0, min(1.0, 0.5 * (1.0 - (aspect - 1.0) / (max_aspect - 1.0)) +
                                0.5 * (fill - min_fill) / (1.0 - min_fill)))
            found.append(cand + (conf,))
    return found, rejected


class ColorDetector(Node):
    def __init__(self, a):
        super().__init__('aris3_color_detector')
        self.a = a
        self.pub = self.create_publisher(DetectedObjectArray, a.output, 10)
        self.create_subscription(CompressedImage, a.input, self._on_image, qos_profile_sensor_data)
        self.last_t = 0.0
        self.n, self.t0, self.ms = 0, time.monotonic(), 0.0
        self.get_logger().info(f'{a.input} -> {a.output} (픽셀 좌표, {PIXEL_FRAME}), 최대 {a.rate:.0f} Hz')

    def _on_image(self, m):
        now = time.monotonic()
        if now - self.last_t < 0.9 / self.a.rate:       # 0.9: 30 Hz 영상에서 15 Hz 가 10 Hz 로 떨어지지 않게
            return
        self.last_t = now
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        t = time.perf_counter()
        found, rejected = detect(img, self.a.min_area, self.a.max_area)
        self.ms += (time.perf_counter() - t) * 1000

        out = DetectedObjectArray()
        out.header.stamp = m.header.stamp            # 영상 찍은 시각 (계약)
        out.header.frame_id = PIXEL_FRAME
        per_color = {}
        for f in found:
            per_color[f[0]] = per_color.get(f[0], 0) + 1
        for color, u, v, area, aspect, fill, conf in sorted(found, key=lambda f: -f[6]):
            o = DetectedObject()
            # 그 색이 하나뿐일 때만 이름을 붙인다 (시뮬레이션과 같은 이름: red_cube ...)
            o.id = f'{color}_cube' if per_color[color] == 1 else ''
            o.color, o.shape = color, 'cube'
            o.pose.header = out.header
            o.pose.pose.position.x, o.pose.pose.position.y = float(u), float(v)
            o.pose.pose.orientation.w = 1.0
            o.confidence = float(conf)
            o.size_px = float(area)
            out.objects.append(o)
        self.pub.publish(out)

        if self.a.debug:
            for c, u, v, area, aspect, fill in rejected:
                self.get_logger().info(f'버림 {c} ({u:.0f},{v:.0f}) 넓이 {area:.0f} 비 {aspect:.2f} 채움 {fill:.2f}',
                                       throttle_duration_sec=1.0)
        self.n += 1
        if now - self.t0 > 10.0:
            names = ', '.join(f'{o.color}({o.pose.pose.position.x:.0f},{o.pose.pose.position.y:.0f})'
                              for o in out.objects) or '없음'
            self.get_logger().info(f'{self.n / (now - self.t0):.1f} Hz, 처리 {self.ms / max(self.n, 1):.1f} ms, '
                                   f'검출 {len(out.objects)}개: {names}')
            self.n, self.t0, self.ms = 0, now, 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', default='/aris3/overhead_camera/image/compressed')
    ap.add_argument('--output', default='/aris3/vision/detections_px')
    ap.add_argument('--rate', type=float, default=15.0, help='최대 처리 속도 [Hz]')
    ap.add_argument('--min-area', type=float, default=300.0, help='[pixel] 2026-11-09 720p 에서 큐브 약 1000~1400')
    ap.add_argument('--max-area', type=float, default=8000.0)
    ap.add_argument('--debug', action='store_true', help='버린 후보와 이유를 로그로')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = ColorDetector(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
