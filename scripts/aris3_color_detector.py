#!/usr/bin/env python3
"""실물 카메라 색 정육면체 검출 - 영상 픽셀 위치만 (시제품, 2026-11-09).

구독: /aris3/overhead_camera/image/compressed  (scripts/aris3_usb_camera_pub.py)
발행: /aris3/vision/detections_px  (aris3_interfaces/DetectedObjectArray)
      header.frame_id = 'overhead_camera_image_px'  ->  pose.position.x, y = 영상 픽셀 (u, v), z = 0
      ※ world 좌표가 아니다. 카메라 보정(카메라 -> world TF)이 없어서 /aris3/detected_objects 에는 보내지 않는다.
        Task Manager 는 이 토픽을 받지 않는다 (GUI 표시용). 보정 후 world 좌표 발행을 이 노드에 더한다.

집기용 픽셀 (2026-11-13): 2층 이상이면 윗면 중심을 1층 높이로 내려 다시 투영한 픽셀을 grasp_u, grasp_v 에 담는다
(카메라 보정이 1층 윗면 높이 기준이라 쌓인 큐브는 원근 때문에 밀려 보임 - cube_top_measure 설명). 화면 표시는 pose.position.

층 (2026-11-13): 깊이(/aris3/overhead_camera/depth/compressed)와 내부 파라미터(camera_info, 둘 다
scripts/aris3_realsense_pub.py)가 오면, 큐브 둘레 탁자를 3D 평면으로 맞추고 윗면까지 수직 높이를 잰다
-> 높이 / 큐브 한 변(40 mm) = 층. 위에서는 맨 위 큐브 색만 보이므로 아래 큐브 색은 모른다 (몇 층인지만).
카메라가 탁자에 비스듬해서 같은 픽셀의 깊이 차이로는 높을수록 크게 재진다 - cube_height_mm 설명. 깊이가 없으면 level = 0 (모름).
--show 로 OpenCV 창에 「red_cube_1 · 2층」 처럼 보여 준다.

회전 (2026-11-13): 큐브 윗면 외곽선의 최소 넓이 사각형(minAreaRect) 각도 = 영상에서 큐브 면의 방향.
정사각형이라 0 ~ 90° 만 뜻이 있다 (90° 돌려도 같은 모양). pose.orientation 에 영상 픽셀 좌표계(u 오른쪽, v 아래)
z 축 회전 쿼터니언으로 담는다 (yaw = 각도). 브리지가 카메라 보정으로 로봇 베이스 각도로 바꿔 그리퍼를 맞춘다.
--show 창에는 이 각도로 돌린 상자를 그린다.

이름: 그 색이 하나면 red_cube, 같은 색이 여럿이면 red_cube_1, red_cube_2 ... (같은 큐브는 장이 바뀌어도 같은 번호, 2026-11-13)

방법: HSV 색 범위 -> 열기/닫기(잡음 제거) -> 외곽선 -> 넓이·가로세로비·채움 비율로 정육면체만 남김.
색 범위는 2026-11-09 실험실 흰 판 위 영상에서 잰 값 (노랑 큐브 H=16 은 실제로는 주황에 가까움, 빨강 H=175,
파랑 H=107), 피부를 빼려고 빨강·노랑은 채도 하한을 높였다. 조명이 바뀌면 --debug 로 마스크를 보고 다시 맞춘다.
나중에 aris3_vision (C++ OpenCV) 으로 옮길 기준 구현.

  python3 scripts/aris3_color_detector.py
  python3 scripts/aris3_color_detector.py --show                      # OpenCV 창에 이름·층 표시
  python3 scripts/aris3_color_detector.py --min-area 300 --max-area 8000 --debug   # 판단 근거를 로그로
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

try:                                    # 창에 한글을 쓰려고 (OpenCV 기본 글꼴은 한글이 없음)
    from PIL import Image as PilImage, ImageDraw, ImageFont
except ImportError:                     # 없으면 영어 「2F」 로 표시
    PilImage = None

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
KO_FONT = '/usr/share/fonts/truetype/nanum/NanumSquareB.ttf'
DEPTH_MAX_AGE_S = 0.5           # 이보다 오래된 깊이 장은 쓰지 않는다 (큐브를 옮겼을 수 있음)
BOX_BGR = {'red': (60, 60, 255), 'yellow': (0, 220, 255), 'green': (80, 220, 80), 'blue': (255, 140, 40)}


def cube_height_mm(depth, scale, u, v, size_px, K):
    """깊이 영상에서 픽셀 (u, v) 큐브 윗면의 탁자 평면 위 수직 높이 [mm]. 계산할 수 없으면 None.

    depth: 컬러에 맞춘 깊이 [mm] (가로·세로 1/scale), 0 = 깊이 없음. K = (fx, fy, cx, cy) 컬러(원래 크기) 내부 파라미터.
    1) 큐브 둘레 고리(2 r ~ 5 r, r = 큐브 반쪽 폭)의 깊이 픽셀을 3D 점으로 바꿔 탁자 평면을 RANSAC 으로 맞춤
       (타공판 구멍으로 보이는 바닥, 옆 큐브, 비스듬히 보이는 큐브 옆면은 평면에서 먼 점이라 빠진다)
    2) 윗면 = 큐브 중심 원(0.8 r) 안에서 카메라에 가까운 쪽 깊이 (옆면이 섞여도 윗면을 고르게 25 % 지점)
    3) 높이 = 윗면 3D 점에서 탁자 평면까지 수직 거리.
    같은 픽셀의 깊이 차이로 재면 카메라가 비스듬해서 높이 쌓인 큐브일수록 크게 나왔다 (2026-11-13 실측: 2층 88 mm).
    """
    r = _plane_and_top(depth, scale, u, v, size_px, K)
    if r is None:
        return None
    n, d0, p_top = r
    return float(abs(n @ p_top + d0))


def _plane_and_top(depth, scale, u, v, size_px, K):
    """cube_height_mm 의 1) 2) 단계: (탁자 평면 법선 n, d, 윗면 중심 3D 점) [mm, 카메라 좌표] 또는 None."""
    if K is None:
        return None
    fx, fy, cx, cy = K
    du, dv = u / scale, v / scale
    r = max(2.0, (max(size_px, 1.0) ** 0.5) / 2 / scale)
    h, w = depth.shape[:2]
    x0, x1 = max(0, int(du - 5 * r) - 1), min(w, int(du + 5 * r) + 2)
    y0, y1 = max(0, int(dv - 5 * r) - 1), min(h, int(dv + 5 * r) + 2)
    if x0 >= x1 or y0 >= y1:
        return None
    win = depth[y0:y1, x0:x1].astype(np.float64)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xx - du, yy - dv)
    top = win[(dist <= 0.8 * r) & (win > 0)]
    sel = (dist >= 2.0 * r) & (dist <= 5.0 * r) & (win > 0)
    if top.size < 4 or sel.sum() < 30:          # 깊이 구멍이 많으면 모름
        return None
    # 깊이 픽셀 -> 3D 점 (원래 크기 픽셀 좌표 = 깊이 픽셀 x scale)
    z = win[sel]
    pts = np.c_[(xx[sel] * scale - cx) / fx * z, (yy[sel] * scale - cy) / fy * z, z]
    plane = fit_plane_ransac(pts)
    if plane is None:
        return None
    n, d0 = plane
    zt = float(np.percentile(top, 25))
    p_top = np.array([(u - cx) / fx * zt, (v - cy) / fy * zt, zt])
    return n, d0, p_top


def cube_top_measure(depth, scale, u, v, size_px, K, layer_mm):
    """깊이로 (높이 [mm], 집기용 픽셀 (gu, gv)) 계산. 높이를 못 재면 (None, (u, v)).

    집기용 픽셀 (2026-11-13): 카메라 보정은 1층 큐브 윗면 높이에서 픽셀 -> 로봇 (x, y) 를 맞춘 것이라, 2층 이상
    윗면은 원근 때문에 영상에서 카메라 바로 아래 점의 바깥쪽으로 밀려 보인다 (카메라 약 0.8 m, 바로 아래에서 300 mm
    떨어진 2층 큐브면 약 16 mm). 윗면 중심 3D 점을 탁자 평면 법선을 따라 (높이 - layer_mm) 만큼 내려 1층 윗면
    높이로 옮긴 뒤 다시 영상에 투영한 픽셀을 돌려준다 -> 보정에 넣으면 그 큐브의 (x, y). 카메라가 비스듬해도 맞다.
    2층 미만(높이 < 1.5 층)이면 고치지 않는다 (1층은 보정 높이 그대로, 잡음만 더해짐).
    """
    r = _plane_and_top(depth, scale, u, v, size_px, K)
    if r is None:
        return None, (u, v)
    n, d0, p_top = r
    sd = float(n @ p_top + d0)               # 평면까지 부호 있는 거리 (법선 방향은 SVD 가 정해서 부호 모름)
    h = abs(sd)
    if h < 1.5 * layer_mm:
        return h, (u, v)
    fx, fy, cx, cy = K
    p1 = p_top - n * (sd - math.copysign(layer_mm, sd))   # 법선을 따라 1층 윗면 높이까지 내림
    if p1[2] <= 1.0:
        return h, (u, v)
    return h, (float(fx * p1[0] / p1[2] + cx), float(fy * p1[1] / p1[2] + cy))


def fit_plane_ransac(pts, tol_mm=4.0, iters=80, min_inliers=20, seed=0):
    """3D 점들에 평면 n·p + d = 0 (|n| = 1) 을 RANSAC 으로 맞춘 뒤 안쪽 점으로 다듬음. 못 맞추면 None.

    seed 를 고정해 같은 깊이 장이면 같은 답 (층이 장마다 흔들리지 않게).
    """
    if len(pts) < min_inliers:
        return None
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-6:                         # 세 점이 한 줄
            continue
        n = n / norm
        inl = np.abs(pts @ n - n @ a) <= tol_mm
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < min_inliers:
        return None
    # 안쪽 점으로 최소제곱 평면 (가장 작은 특이값 방향 = 법선)
    q = pts[best]
    c = q.mean(axis=0)
    # full_matrices=False: 기본값(True)은 점 N 개에 N x N 행렬까지 만들어 큐브 하나에 약 170 ms 가 걸렸다
    # (2026-11-13: 4개면 한 장 0.7 초 -> 깊이가 밀려 다음 장이 「층 모름」 = 화면 글자 깜빡임). 법선은 같다.
    n = np.linalg.svd(q - c, full_matrices=False)[2][-1]
    return n, float(-n @ c)


def level_from_height(height_mm, layer_mm):
    """높이 [mm] -> 층 (1, 2, 3 ...). 높이를 모르면 0. 큐브가 납작하게 재져도 1층으로 본다."""
    if height_mm is None:
        return 0
    return max(1, int(round(height_mm / layer_mm)))


def draw_overlay(img, items):
    """OpenCV 창용 그림: 큐브마다 돌린 상자 + 「이름 · N층 (높이) 각도」.

    items = [(color, u, v, size_px, 이름, 층, 높이, 각도[deg])].
    """
    out = img.copy()
    labels = []
    for color, u, v, size, name, level, h, ang in items:
        half = int((max(size, 1.0) ** 0.5) / 2) + 4
        c = BOX_BGR.get(color, (255, 255, 255))
        # 큐브 면 방향으로 돌린 상자 (그리퍼를 맞출 각도를 눈으로 확인)
        box = cv2.boxPoints(((float(u), float(v)), (2.0 * half, 2.0 * half), float(ang)))
        cv2.polylines(out, [box.astype(np.int32)], True, c, 2)
        floor = (f'{level}층' if PilImage else f'{level}F') if level else ('층?' if PilImage else '?F')
        text = (f'{name} · {floor}' + (f' ({h:.0f} mm)' if h is not None else '')
                + f' {ang:.0f}' + ('°' if PilImage else 'deg'))
        labels.append((int(u) - half, int(v) - half - 24, text, c))
    if PilImage is None:
        for x, y, text, c in labels:
            cv2.putText(out, text, (x, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        return out
    # 한글: PIL 로 글자를 그린 뒤 다시 OpenCV 영상으로
    pil = PilImage.fromarray(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    try:
        font = ImageFont.truetype(KO_FONT, 20)
    except OSError:
        font = ImageFont.load_default()
    for x, y, text, c in labels:
        x0, y0, x1, y1 = draw.textbbox((x, y), text, font=font)
        draw.rectangle((x0 - 3, y0 - 2, x1 + 3, y1 + 2), fill=(0, 0, 0))     # 글자 뒤 검은 띠 (밝은 판 위에서도 보이게)
        draw.text((x, y), text, font=font, fill=(c[2], c[1], c[0]))
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


class CubeNumberer:
    """같은 색 큐브에 번호(1, 2, ...)를 붙이고, 장이 바뀌어도 같은 큐브는 같은 번호를 유지한다 (2026-11-13).

    앞 장의 같은 색 큐브 중 가까운(max_dist 픽셀 안) 것의 번호를 이어받는다. 손에 잠깐 가려도 hold_s 초 동안은
    번호를 기억해 두어, 다시 보이면 같은 번호가 된다. 새 큐브는 비어 있는 가장 작은 번호를 왼쪽(u 작은)부터 받는다.
    """

    def __init__(self, max_dist=40.0, hold_s=2.0):
        self.max_dist = max_dist
        self.hold_s = hold_s
        self.tracks = {}            # 색 -> [{'n': 번호, 'u': 픽셀, 'v': 픽셀, 't': 마지막으로 본 시각}]

    def assign(self, color, points, now):
        """이번 장의 그 색 큐브 픽셀 [(u, v)] -> 같은 순서의 번호 목록."""
        # 오래 안 보인 번호는 버린다 (그 큐브를 치웠다고 봄)
        tracks = [t for t in self.tracks.get(color, []) if now - t['t'] <= self.hold_s]
        # 가까운 짝부터 이어 붙이기 (검출·기억 쌍을 거리순으로 보고 둘 다 아직 안 쓴 것만)
        pairs = sorted((((u - t['u']) ** 2 + (v - t['v']) ** 2) ** 0.5, i, j)
                       for i, (u, v) in enumerate(points) for j, t in enumerate(tracks))
        nums = [None] * len(points)
        used_t = set()
        for d, i, j in pairs:
            if d > self.max_dist:
                break
            if nums[i] is None and j not in used_t:
                nums[i] = tracks[j]['n']
                used_t.add(j)
        # 짝이 없는 새 큐브: 왼쪽부터 비어 있는 가장 작은 번호
        taken = {t['n'] for t in tracks}
        for i in sorted((i for i in range(len(points)) if nums[i] is None), key=lambda i: points[i]):
            n = 1
            while n in taken:
                n += 1
            nums[i] = n
            taken.add(n)
        # 기억 갱신: 이번에 본 큐브 + 아직 기억 시간이 남은 안 보인 큐브
        seen = [{'n': n, 'u': u, 'v': v, 't': now} for n, (u, v) in zip(nums, points)]
        self.tracks[color] = seen + [t for j, t in enumerate(tracks) if j not in used_t]
        return nums


def cube_id(color, number, count):
    """검출 이름: 그 색이 하나뿐이면 'red_cube' (시뮬레이션과 같은 이름), 여럿이면 'red_cube_1', 'red_cube_2' ..."""
    return f'{color}_cube' if count == 1 else f'{color}_cube_{number}'


def detect(bgr, min_area, max_area, max_aspect=1.6, min_fill=0.65):
    """영상 -> [(color, u, v, area, aspect, fill, confidence, angle_deg)], 버린 후보 목록(진단용).

    angle_deg: 큐브 면 방향 [0, 90) 도 - 영상 u 축에서 v 축 쪽(화면에서 시계 방향)으로 잰 각.
    """
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
            (u, v), (w, h), ang = cv2.minAreaRect(c)
            aspect = max(w, h) / max(min(w, h), 1.0)
            fill = area / max(w * h, 1.0)
            cand = (color, u, v, area, aspect, fill)
            if not (min_area <= area <= max_area and aspect <= max_aspect and fill >= min_fill):
                rejected.append(cand)
                continue
            # 신뢰도: 정사각형에 가깝고(aspect 1) 꽉 찰수록(fill 1) 높다
            conf = max(0.0, min(1.0, 0.5 * (1.0 - (aspect - 1.0) / (max_aspect - 1.0)) +
                                0.5 * (fill - min_fill) / (1.0 - min_fill)))
            found.append(cand + (conf, float(ang) % 90.0))   # 정사각형: 90° 마다 같은 모양
    return found, rejected


class ColorDetector(Node):
    def __init__(self, a):
        super().__init__('aris3_color_detector')
        self.a = a
        self.pub = self.create_publisher(DetectedObjectArray, a.output, 10)
        self.create_subscription(CompressedImage, a.input, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, a.depth, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.camera_info, self._on_info, 10)
        self.K = None                           # 컬러 내부 파라미터 (fx, fy, cx, cy) - 없으면 층을 모름
        self.depth, self.depth_scale, self.depth_t = None, 1, 0.0    # 마지막 깊이 장 [mm], 축소 배율, 받은 시각
        self.last_t = 0.0
        self.numberer = CubeNumberer()      # 같은 색 큐브 번호 (장이 바뀌어도 유지)
        self.n, self.t0, self.ms = 0, time.monotonic(), 0.0
        self.get_logger().info(f'{a.input} -> {a.output} (픽셀 좌표, {PIXEL_FRAME}), 최대 {a.rate:.0f} Hz')

    def _on_info(self, m):
        """카메라 내부 파라미터 수신 (깊이 픽셀 -> 3D 점에 씀)."""
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        """깊이 장 수신: 16 bit PNG 풀기 + format 의 scale=N (원래 픽셀 = 깊이 픽셀 x N) 읽기."""
        d = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_UNCHANGED)
        if d is None or d.dtype != np.uint16:
            return
        scale = 1
        for part in m.format.split(';'):
            if part.strip().startswith('scale='):
                scale = int(part.strip()[6:])
        self.depth, self.depth_scale, self.depth_t = d, scale, time.monotonic()

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
        # 색마다 번호 붙이기 (같은 색이 여럿이면 red_cube_1, red_cube_2 ...; 하나면 red_cube)
        depth = self.depth if now - self.depth_t <= DEPTH_MAX_AGE_S else None   # 오래된 깊이는 안 씀
        names = {}
        for color in {f[0] for f in found}:
            same = [f for f in found if f[0] == color]
            nums = self.numberer.assign(color, [(f[1], f[2]) for f in same], now)
            for f, n in zip(same, nums):
                names[id(f)] = cube_id(color, n, len(same))
        overlay = []
        for f in sorted(found, key=lambda f: -f[6]):
            color, u, v, area, aspect, fill, conf, ang = f
            o = DetectedObject()
            o.id = names[id(f)]
            # 층: 깊이가 있으면 탁자 위 높이 -> 층, 없으면 0 (모름)
            h, (gu, gv) = (cube_top_measure(depth, self.depth_scale, u, v, area, self.K, self.a.layer_mm)
                           if depth is not None else (None, (u, v)))
            o.height_mm = float(h) if h is not None else 0.0
            o.grasp_u, o.grasp_v = float(gu), float(gv)     # 집기용 픽셀 (쌓인 큐브 원근 보정, 1층이면 위치 그대로)
            o.level = level_from_height(h, self.a.layer_mm)
            overlay.append((color, u, v, area, o.id, o.level, h, ang))
            o.color, o.shape = color, 'cube'
            o.pose.header = out.header
            o.pose.pose.position.x, o.pose.pose.position.y = float(u), float(v)
            # 회전: 영상 픽셀 좌표계 z 축 회전 (yaw = 큐브 면 방향 [0, 90)°) 쿼터니언
            o.pose.pose.orientation.z = math.sin(math.radians(ang) / 2)
            o.pose.pose.orientation.w = math.cos(math.radians(ang) / 2)
            o.confidence = float(conf)
            o.size_px = float(area)
            out.objects.append(o)
        self.pub.publish(out)
        if self.a.show:                         # OpenCV 창: 이름·층
            cv2.imshow('ARIS3 cubes', draw_overlay(img, overlay))
            cv2.waitKey(1)

        if self.a.debug:
            for c, u, v, area, aspect, fill in rejected:
                self.get_logger().info(f'버림 {c} ({u:.0f},{v:.0f}) 넓이 {area:.0f} 비 {aspect:.2f} 채움 {fill:.2f}',
                                       throttle_duration_sec=1.0)
        self.n += 1
        if now - self.t0 > 10.0:
            names = ', '.join(f'{o.id}({o.pose.pose.position.x:.0f},{o.pose.pose.position.y:.0f})'
                              + (f' {o.level}층 {o.height_mm:.0f}mm' if o.level else '')
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
    ap.add_argument('--depth', default='/aris3/overhead_camera/depth/compressed', help='컬러에 맞춘 깊이 (층 계산)')
    ap.add_argument('--camera-info', default='/aris3/overhead_camera/camera_info', help='컬러 내부 파라미터')
    ap.add_argument('--layer-mm', type=float, default=40.0, help='한 층 높이 [mm] = 실물 큐브 한 변')
    ap.add_argument('--show', action='store_true', help='OpenCV 창에 이름·층 표시')
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
