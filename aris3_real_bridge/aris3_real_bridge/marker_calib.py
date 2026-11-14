"""탁자 블록 마커 보정 도구 (2026-11-14): 블록 윗면에 붙인 ArUco 마커로 보정 ① 픽셀을 정확히 잰다.

보정 순서는 예전과 같다 (「손으로 가르쳐 주기」, real_pick.PixelCalibration):
  ① 마커 붙인 블록을 탁자 위 한 자리에 두고 「① 기억」 -> 마커 중심 픽셀 (대각선 교점, 0.1 px 단위)
     + D455 깊이로 마커 3D 점(카메라 좌표)과 블록 높이를 잰다.
  ② 조그·손으로 움직이기로 그리퍼를 그 블록을 집는 자세까지 내린 뒤 「② 기록」 -> 그때 TCP (x, y, z).
  자리를 바꿔 6~8 번. 색 큐브 중심(조명·그림자에 흔들림)보다 픽셀이 훨씬 정확하다.

보정 기준은 「1층 큐브 윗면 중심 픽셀 -> 그 큐브를 집는 TCP」 이다. 블록 높이가 큐브(40 mm)와 다르면
  - 픽셀: 마커 중심 3D 점을 둘레 탁자 평면의 법선을 따라 큐브 높이로 옮긴 뒤 다시 영상에 투영 (to_cube_height)
  - 집는 높이: 기록한 TCP z 에서 (블록 높이 - 큐브 높이) 를 뺀다 (윗면에서 같은 깊이로 집는다고 봄)
로 바꿔 기록한다 (bridge_node RECORD).

마커: ArUco DICT_4X4_50 (출력물은 ID 0). 로봇이 없는 계산은 모두 여기 있고 단위 시험한다 (test/test_marker_calib.py).
"""
import math

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50


def find_markers(gray, marker_id=0, dictionary=ARUCO_DICT):
    """흑백 영상의 marker_id 마커들 -> [(중심 (u, v), 모서리 4x2, 한 변 px)]. 중심 = 대각선 교점."""
    aruco = cv2.aruco
    d = aruco.getPredefinedDictionary(dictionary)
    if hasattr(aruco, 'ArucoDetector'):                       # OpenCV 4.7+
        corners, ids, _ = aruco.ArucoDetector(d, aruco.DetectorParameters()).detectMarkers(gray)
    else:                                                     # OpenCV 4.6 (Ubuntu 24.04)
        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = aruco.detectMarkers(gray, d, parameters=params)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        if int(i) != int(marker_id):
            continue
        p = c.reshape(4, 2).astype(np.float64)
        side = float(np.mean([np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]))
        out.append((diagonal_center(p), p, side))
    return out


def pick_marker(markers, near=None):
    """여러 마커 중 하나: near (u, v) 를 주면 가장 가까운 것, 아니면 가장 큰 것. 없으면 None."""
    if not markers:
        return None
    if near is not None:
        return min(markers, key=lambda m: math.hypot(m[0][0] - near[0], m[0][1] - near[1]))
    return max(markers, key=lambda m: m[2])


def diagonal_center(p):
    """사각형 모서리 p[0..3] 의 두 대각선 (0-2, 1-3) 교점 (u, v) - 원근에서도 정사각형 중심의 정확한 투영."""
    a, c, b, d = p[0], p[2], p[1], p[3]
    m = np.array([[c[0] - a[0], -(d[0] - b[0])], [c[1] - a[1], -(d[1] - b[1])]])
    s = np.linalg.solve(m, b - a)[0]
    q = a + s * (c - a)
    return float(q[0]), float(q[1])


def marker_angle(corners):
    """마커 면 방향 [0, 90) deg (영상): 네 변의 각도를 90° 대칭으로 접어 평균 (작은 마커는 한 변만 보면 ±2~4° 흔들림)."""
    p = np.asarray(corners, float)
    n = len(p)
    return mean_angle90([math.degrees(math.atan2(*(p[(k + 1) % n] - p[k])[::-1])) for k in range(n)])


def mean_angle90(angles):
    """90° 대칭 각도들의 평균 [0, 90) (0°·89° 처럼 경계에 걸쳐도 맞게 - 4 배 각의 원 평균)."""
    a = np.radians(np.asarray(angles, float) * 4.0)
    return math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())) / 4.0 % 90.0


def decode_depth(msg):
    """깊이 CompressedImage (16 bit PNG, format 의 scale=N) -> (깊이 [mm] uint16, scale). 못 풀면 None."""
    d = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
    if d is None or d.dtype != np.uint16:
        return None
    scale = 1
    for part in msg.format.split(';'):
        if part.strip().startswith('scale='):
            scale = int(part.strip()[6:])
    return d, scale


def marker_cam_point(depth, scale, corners, center, K, shrink=0.5):
    """마커 중심의 카메라 좌표 3D 점 [mm]. 깊이 = 마커 안쪽(shrink 배로 줄인 사각형) 깊이의 중앙값. 못 재면 None.

    depth: 컬러에 맞춘 깊이 [mm] (가로·세로 1/scale), K = (fx, fy, cx, cy) 원래 크기 컬러 내부 파라미터.
    """
    fx, fy, cx, cy = K
    c = np.asarray(center, np.float64)
    quad = (c + shrink * (np.asarray(corners, np.float64) - c)) / scale
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, np.round(quad).astype(np.int32), 1)
    z = depth[(mask > 0) & (depth > 0)]
    if z.size < 4:
        return None
    zc = float(np.median(z))
    return np.array([(c[0] - cx) / fx * zc, (c[1] - cy) / fy * zc, zc])


def fit_plane(pts, tol_mm=4.0, iters=120, seed=0):
    """점들에 평면 n·X + d = 0 (|n| = 1) 을 RANSAC + 최소제곱으로. 못 맞추면 None. 돌려줌 (n, d)."""
    if len(pts) < 30:
        return None
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs(pts @ n - n @ a) <= tol_mm
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 30:
        return None
    q = pts[best]
    c = q.mean(0)
    n = np.linalg.svd(q - c, full_matrices=False)[2][-1]
    return n, float(-n @ c)


def table_around(depth, scale, center, side_px, K, inner=1.5, outer=4.0):
    """마커 둘레 고리(마커 중심에서 모서리까지 거리의 inner ~ outer 배)의 깊이로 맞춘 탁자 평면 (n, d) [카메라 좌표].

    블록 옆면·옆 물체·판 구멍은 평면에서 먼 점이라 RANSAC 이 뺀다. 못 맞추면 None.
    """
    fx, fy, cx, cy = K
    r = max(2.0, side_px / 2 / scale) * math.sqrt(2)            # 마커 모서리까지 (블록이 돌아가 있어도)
    du, dv = center[0] / scale, center[1] / scale
    h, w = depth.shape[:2]
    x0, x1 = max(0, int(du - outer * r)), min(w, int(du + outer * r) + 1)
    y0, y1 = max(0, int(dv - outer * r)), min(h, int(dv + outer * r) + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    win = depth[y0:y1, x0:x1].astype(np.float64)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xx - du, yy - dv)
    sel = (dist >= inner * r) & (dist <= outer * r) & (win > 0)
    z = win[sel]
    pts = np.c_[(xx[sel] * scale - cx) / fx * z, (yy[sel] * scale - cy) / fy * z, z]
    return fit_plane(pts)


def to_cube_height(p_top, plane, cube_mm, K):
    """마커 윗면 3D 점을 탁자 평면 법선을 따라 큐브 높이(cube_mm)로 옮겨 다시 투영.

    돌려줌 (블록 높이 [mm], (u, v) 큐브 높이 픽셀). 평면이 없으면 None.
    """
    if plane is None:
        return None
    n, d = plane
    sd = float(n @ p_top + d)                    # 부호 있는 거리 (법선 부호는 SVD 가 정함)
    p1 = p_top - n * (sd - math.copysign(cube_mm, sd))
    fx, fy, cx, cy = K
    if p1[2] <= 1.0:
        return None
    return abs(sd), (float(fx * p1[0] / p1[2] + cx), float(fy * p1[1] / p1[2] + cy))


def kabsch(P, Q):
    """Q ≈ R P + t 인 회전 R·이동 t (점 대응, 최소제곱). P, Q: N x 3."""
    P = np.asarray(P, np.float64)
    Q = np.asarray(Q, np.float64)
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, qc - R @ pc


def rigid_check(cam_pts, tcp_pts):
    """보정점의 카메라 3D 점 ↔ TCP 를 강체 변환 하나로 맞춘 잔차 [mm] (점마다). 4 점 미만이면 None.

    카메라·로봇이 모두 맞게 쟀다면 모든 점이 한 변환으로 몇 mm 안에 맞는다 -> 잘못 기록한 점을 찾는 데 쓴다.
    """
    if len(cam_pts) < 4:
        return None
    R, t = kabsch(cam_pts, tcp_pts)
    return np.linalg.norm(np.asarray(cam_pts) @ R.T + t - np.asarray(tcp_pts), axis=1)
