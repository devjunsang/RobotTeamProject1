"""탁자 블록 마커 보정 도구 시험 (2026-11-14, 카메라·로봇 없이 합성 장면).

카메라가 탁자(860 mm 아래)를 내려다보고, 탁자 위 블록(높이 40 또는 60 mm) 윗면에 한 변 28 mm 마커가 있는
영상·깊이를 그려서
  - 마커 찾기·대각선 중심 (원근으로 기울여도), 여럿이면 가까운 것 / 큰 것
  - 깊이로 마커 3D 점, 둘레 탁자 평면, 블록 높이, 큐브 높이(40 mm) 픽셀로 옮기기
  - 강체 일관성 검사 (잘못 기록한 점 찾기)
가 정답과 맞는지 본다. scene() 은 test_bridge_node 도 쓴다.
"""
import math

import cv2
import numpy as np
import pytest

from aris3_real_bridge import marker_calib as MC

K = (646.0, 646.0, 640.0, 360.0)
TABLE_Z = 860.0                     # 카메라에서 탁자까지 [mm] (곧게 내려다봄)
MARKER_MM = 28.0                    # 마커 검은 테두리 한 변
BLOCK_MM = 40.0                     # 블록 한 변 (윗면)


def proj(X, Y, Z):
    """카메라 좌표 [mm] -> 픽셀."""
    fx, fy, cx, cy = K
    return fx * X / Z + cx, fy * Y / Z + cy


def scene(X=150.0, Y=80.0, height=40.0, yaw_deg=20.0, w=1280, h=720, scale=2):
    """블록 중심이 카메라 좌표 (X, Y), 윗면 높이 height, z 축으로 yaw 만큼 돈 장면 -> (흑백 영상, 깊이 [mm] 1/scale).

    깊이는 탁자 TABLE_Z, 블록 윗면 TABLE_Z - height (옆면은 무시 - 둘레 평면은 RANSAC 이 맞춤).
    """
    Zt = TABLE_Z - height
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))

    def corners(half, Z):
        return np.float32([proj(X + c * dx - s * dy, Y + s * dx + c * dy, Z)
                           for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half))])
    img = np.full((h, w), 230, np.uint8)
    cv2.fillConvexPoly(img, np.round(corners(BLOCK_MM / 2, Zt)).astype(np.int32), 250)    # 블록 윗면 (흰 여백)
    mk = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(MC.ARUCO_DICT), 0, 120)
    src = np.float32([[0, 0], [120, 0], [120, 120], [0, 120]])
    M = cv2.getPerspectiveTransform(src, corners(MARKER_MM / 2, Zt))
    warped = cv2.warpPerspective(mk, M, (w, h), flags=cv2.INTER_AREA, borderValue=255)
    mask = cv2.warpPerspective(np.full((120, 120), 255, np.uint8), M, (w, h)) > 0
    img[mask] = warped[mask]
    depth = np.full((h // scale, w // scale), TABLE_Z, np.float64)
    top = np.zeros((h // scale, w // scale), np.uint8)
    cv2.fillConvexPoly(top, np.round(corners(BLOCK_MM / 2, Zt) / scale).astype(np.int32), 1)
    depth[top > 0] = Zt
    return img, np.round(depth).astype(np.uint16)


@pytest.mark.parametrize('X, Y', [(150.0, 80.0), (-200.0, -120.0), (0.0, 0.0)])
def test_find_marker_center_is_projected_square_center(X, Y):
    """돌아간 블록의 마커 중심 = 블록 윗면 중심의 투영 (0.5 px 안)."""
    img, _ = scene(X, Y)
    ms = MC.find_markers(img, 0)
    assert len(ms) == 1
    u, v = proj(X, Y, TABLE_Z - 40.0)
    assert math.hypot(ms[0][0][0] - u, ms[0][0][1] - v) < 0.5
    assert ms[0][2] == pytest.approx(646 * MARKER_MM / (TABLE_Z - 40), rel=0.1)    # 한 변 px 는 참고값
    assert MC.find_markers(img, 5) == []


def test_pick_marker_nearest_or_largest():
    """여러 마커: 고른 픽셀이 있으면 가장 가까운 것, 없으면 가장 큰 것."""
    ms = [((100.0, 100.0), None, 20.0), ((500.0, 300.0), None, 30.0)]
    assert MC.pick_marker(ms, (120, 90))[0] == (100.0, 100.0)
    assert MC.pick_marker(ms)[0] == (500.0, 300.0)
    assert MC.pick_marker([]) is None


@pytest.mark.parametrize('height', [40.0, 60.0, 25.0])
def test_block_height_and_cube_height_pixel(height):
    """깊이로 블록 높이를 재고, 큐브 높이(40 mm) 윗면 중심 픽셀로 옮긴다 (1 px 안). 40 mm 블록은 거의 그대로."""
    X, Y = 220.0, 140.0
    img, depth = scene(X, Y, height)
    (c, corners, side), = MC.find_markers(img, 0)
    p = MC.marker_cam_point(depth, 2, corners, c, K)
    assert p[2] == pytest.approx(TABLE_Z - height, abs=1.0)
    plane = MC.table_around(depth, 2, c, side, K)
    hb, (gu, gv) = MC.to_cube_height(p, plane, 40.0, K)
    assert hb == pytest.approx(height, abs=1.5)
    eu, ev = proj(X, Y, TABLE_Z - 40.0)
    assert math.hypot(gu - eu, gv - ev) < 1.0
    if height != 40.0:
        assert math.hypot(c[0] - eu, c[1] - ev) > 2.0              # 고치지 않으면 밀려 있음


def test_no_depth_or_plane():
    """깊이가 비면 3D 점·평면을 못 잼 (None)."""
    img, depth = scene()
    (c, corners, side), = MC.find_markers(img, 0)
    empty = np.zeros_like(depth)
    assert MC.marker_cam_point(empty, 2, corners, c, K) is None
    assert MC.table_around(empty, 2, c, side, K) is None
    assert MC.to_cube_height(np.array([1.0, 2.0, 800.0]), None, 40.0, K) is None


def test_rigid_check_flags_bad_point():
    """카메라 3D 점 ↔ TCP: 맞게 잰 점들은 0 에 가깝고, 하나를 15 mm 틀리게 기록하면 그 점의 잔차가 가장 크다."""
    rng = np.random.default_rng(3)
    R = cv2.Rodrigues(np.array([0.1, -3.0, 0.2]))[0]
    t = np.array([600.0, 20.0, 900.0])
    cam = rng.uniform([-300, -200, 800], [300, 200, 840], (7, 3))
    tcp = cam @ R.T + t
    assert max(MC.rigid_check(cam, tcp)) < 1e-6
    tcp[4, 0] += 15.0
    r = MC.rigid_check(cam, tcp)
    assert int(np.argmax(r)) == 4 and r[4] > 8.0
    assert MC.rigid_check(cam[:3], tcp[:3]) is None


@pytest.mark.parametrize('yaw', [0.0, 20.0, 44.0, 70.0, 89.0])
def test_marker_angle(yaw):
    """곧게 내려다본 카메라에서 블록을 yaw 만큼 돌리면 마커 각도도 yaw (90° 대칭, 한 변 21 px 마커에서 1.5° 안)."""
    img, _ = scene(yaw_deg=yaw)
    (c, corners, side), = MC.find_markers(img, 0)
    a = MC.marker_angle(corners)
    assert abs((a - yaw + 45.0) % 90.0 - 45.0) < 1.5


def test_mean_angle90_wraps():
    """0° 와 89° 의 평균은 44.5° 가 아니라 89.5° (= -0.5°)."""
    assert MC.mean_angle90([0.0, 89.0]) == pytest.approx(89.5, abs=1e-6)
    assert MC.mean_angle90([10.0, 12.0]) == pytest.approx(11.0, abs=1e-6)


def test_diagonal_center_square():
    """정사각형 대각선 교점 = 중심."""
    assert MC.diagonal_center(np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)) == pytest.approx((5, 5))
