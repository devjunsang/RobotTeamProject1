"""aris3_color_detector 의 깊이 계산 시험 (합성 깊이 영상, 카메라·로봇 없이, 2026-11-13).

비스듬히 내려다보는 카메라로 탁자(z = 0)와 큐브 기둥(한 변 40 mm, 높이 40 / 80 mm)을 광선 추적해 깊이 영상을 만들고
  - 높이·층이 맞는지 (1층 40, 2층 80 mm)
  - 2층 큐브의 집기용 픽셀이 「같은 자리 1층 큐브 윗면 중심」 의 픽셀과 같은지 (원근 보정)
  - 회전한 큐브 윤곽에서 각도를 재는지
를 본다.

  python3 -m pytest -q scripts/test_aris3_color_detector.py
"""
import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aris3_color_detector as D  # noqa: E402

FX = FY = 646.0
CX, CY = 644.0, 368.0
K = (FX, FY, CX, CY)
SCALE = 2                       # 깊이는 가로·세로 1/2 (aris3_realsense_pub.py 와 같음)
W, H = 1280, 720
CUBE = 40.0


def camera():
    """카메라 위치 C (탁자 위 800 mm, 옆으로 250 mm) 와 회전 R (열 = 카메라 x, y, z 축, 세계 기준). 25° 정도 비스듬함."""
    C = np.array([0.0, -250.0, 800.0])
    z = np.array([0.0, 120.0, 0.0]) - C          # 탁자 위 한 점을 바라봄
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return C, np.c_[x, y, z]


def project(p):
    """세계 점 [mm] -> 영상 픽셀 (u, v)."""
    C, R = camera()
    pc = R.T @ (np.asarray(p, float) - C)
    return FX * pc[0] / pc[2] + CX, FY * pc[1] / pc[2] + CY


def render_depth(boxes):
    """탁자 + 상자들 [(x0, x1, y0, y1, 높이)] 의 깊이 영상 [mm] (1/2 크기, uint16) - 광선과 가장 가까운 교점의 Z."""
    C, R = camera()
    vv, uu = np.mgrid[0:H // SCALE, 0:W // SCALE]
    rc = np.stack([(uu * SCALE - CX) / FX, (vv * SCALE - CY) / FY, np.ones_like(uu, float)], -1)   # 카메라 광선 (Z = 1)
    d = rc @ R.T                                                                          # 세계 방향
    t_best = np.where(d[..., 2] < 0, -C[2] / np.minimum(d[..., 2], -1e-9), np.inf)       # 탁자 z = 0
    for x0, x1, y0, y1, h in boxes:                                                      # 상자: slab 방법
        lo = np.array([x0, y0, 0.0])
        hi = np.array([x1, y1, h])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo - C) / d
            t2 = (hi - C) / d
        tmin = np.nanmax(np.minimum(t1, t2), -1)
        tmax = np.nanmin(np.maximum(t1, t2), -1)
        hit = (tmax >= tmin) & (tmin > 0)
        t_best = np.where(hit & (tmin < t_best), tmin, t_best)
    return np.where(np.isfinite(t_best), t_best, 0).astype(np.uint16)   # 광선 Z 성분이 1 이라 t = 깊이 Z


def cube_box(x, y, h):
    """(x, y) 중심, 높이 h 인 큐브 기둥 (축 정렬)."""
    return (x - CUBE / 2, x + CUBE / 2, y - CUBE / 2, y + CUBE / 2, h)


def top_area_px(x, y, h):
    """윗면 정사각형의 영상 넓이 [px] (검출 넓이 대신)."""
    corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
    pts = np.array([project((x + sx * CUBE / 2, y + sy * CUBE / 2, h)) for sx, sy in corners], np.float32)
    return float(cv2.contourArea(pts))


@pytest.mark.parametrize('x, y', [(0.0, 120.0), (250.0, 300.0), (-200.0, 0.0)])
def test_one_level_height_and_no_correction(x, y):
    """1층 큐브: 높이 약 40 mm, 집기용 픽셀은 화면 위치 그대로.

    윗면 깊이를 「카메라에 가까운 쪽 25 %」 로 고르므로 비스듬히 보이는 먼 큐브는 몇 mm 높게 재진다 (설계상, 층 계산엔
    영향 없음 - 40 mm 단위 반올림). 그래서 허용 5 mm.
    """
    depth = render_depth([cube_box(x, y, CUBE)])
    u, v = project((x, y, CUBE))
    h, (gu, gv) = D.cube_top_measure(depth, SCALE, u, v, top_area_px(x, y, CUBE), K, CUBE)
    assert h == pytest.approx(CUBE, abs=5.0)
    assert (gu, gv) == (u, v)
    assert D.level_from_height(h, CUBE) == 1


@pytest.mark.parametrize('x, y', [(0.0, 120.0), (250.0, 300.0), (-200.0, 0.0)])
def test_stacked_cube_grasp_pixel_matches_one_level_top(x, y):
    """2층 큐브: 높이 약 80 mm, 집기용 픽셀 = 같은 자리 1층 윗면 중심 픽셀 (1.5 px 안, 약 1.5 mm). 고치기 전엔 크게 다름."""
    depth = render_depth([cube_box(x, y, 2 * CUBE)])
    u, v = project((x, y, 2 * CUBE))
    h, (gu, gv) = D.cube_top_measure(depth, SCALE, u, v, top_area_px(x, y, 2 * CUBE), K, CUBE)
    eu, ev = project((x, y, CUBE))
    assert h == pytest.approx(2 * CUBE, abs=3.0)
    assert D.level_from_height(h, CUBE) == 2
    assert math.hypot(gu - eu, gv - ev) < 1.5
    assert math.hypot(u - eu, v - ev) > 5.0           # 고치지 않으면 이만큼 밀려 있었음


def test_three_levels_correction():
    """3층 (120 mm): 집기용 픽셀은 1층 윗면 높이 (40 mm) 로 내려 투영한 자리."""
    x, y = 200.0, 250.0
    depth = render_depth([cube_box(x, y, 3 * CUBE)])
    u, v = project((x, y, 3 * CUBE))
    h, (gu, gv) = D.cube_top_measure(depth, SCALE, u, v, top_area_px(x, y, 3 * CUBE), K, CUBE)
    eu, ev = project((x, y, CUBE))
    assert D.level_from_height(h, CUBE) == 3
    assert math.hypot(gu - eu, gv - ev) < 2.0


def test_neighbor_cube_does_not_break_plane():
    """옆에 1층 큐브가 붙어 있어도(고리 안에 들어옴) 탁자 평면은 RANSAC 이 맞춰 2층 높이가 맞다."""
    depth = render_depth([cube_box(0.0, 120.0, 2 * CUBE), cube_box(70.0, 120.0, CUBE)])
    u, v = project((0.0, 120.0, 2 * CUBE))
    h, _ = D.cube_top_measure(depth, SCALE, u, v, top_area_px(0.0, 120.0, 2 * CUBE), K, CUBE)
    assert h == pytest.approx(2 * CUBE, abs=3.0)


def test_unknown_without_intrinsics_or_depth():
    """내부 파라미터가 없거나 깊이가 비면 높이 모름(None), 집기용 픽셀은 화면 위치."""
    depth = render_depth([cube_box(0.0, 120.0, CUBE)])
    assert D.cube_top_measure(depth, SCALE, 640.0, 360.0, 1200.0, None, CUBE) == (None, (640.0, 360.0))
    empty = np.zeros_like(depth)
    assert D.cube_top_measure(empty, SCALE, 640.0, 360.0, 1200.0, K, CUBE) == (None, (640.0, 360.0))


@pytest.mark.parametrize('ang', [0.0, 17.0, 33.0, 62.0])
def test_detect_reports_cube_angle(ang):
    """돌아간 빨간 정사각형 (한 변 40 px) 을 그리면 그 각도를 90° 대칭으로 잰다 (3° 안)."""
    img = np.full((480, 640, 3), 235, np.uint8)
    box = cv2.boxPoints(((320.0, 240.0), (40.0, 40.0), ang)).astype(np.int32)
    cv2.fillPoly(img, [box], (30, 30, 210))
    found, _ = D.detect(img, 300, 8000)
    assert len(found) == 1
    got = found[0][7]
    assert abs((got - ang + 45.0) % 90.0 - 45.0) < 3.0
