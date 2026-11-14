"""실물 E0509: 카메라 픽셀 -> 로봇 좌표 보정과 집기 순서 (2026-11-09, 시제품).

2026-11-10: aris3_gui/real_pick.py 에서 aris3_real_bridge 로 옮김 (두 GUI 가 브리지 노드를 통해 같이 쓴다).

보정 (「손으로 가르쳐 주기」):
  큐브를 판 위 여러 곳에 놓고, 매번
    ① 카메라에 큐브가 보일 때 그 픽셀 (u, v) 를 기억하고
    ② 조그로 그리퍼를 그 큐브를 실제로 집는 자세까지 내린 뒤 로봇 TCP (x, y, z) 를 기록한다.
  판은 평평하므로 픽셀 -> (x, y) 는 호모그래피(3x3) 하나로 표현된다 (4 점 이상).
  집는 높이 z 는 기록한 z 의 중앙값을 그대로 쓴다 -> 공구 길이·카메라 높이를 몰라도 된다.
  큐브 윗면을 보고 잰 픽셀이므로 같은 크기(4 cm) 큐브에만 맞다.

집기 순서 (PickSequence, 작업 스레드):
  위로(접근 높이) -> (집는 각도 맞추기) -> 그리퍼 열기 -> 집는 높이로 하강 -> 닫기 -> 접근 높이로 들기.
  각도 맞추기 (2026-11-13): 접근 높이에서 J6 만 돌려 그리퍼 면을 큐브 면에 맞추고, 놓을 때는 놓을 자리 위에서
  큐브 면이 팔 방향(베이스 -> 놓을 자리)과 평행·수직이 되게(쌓기는 받침 큐브와 같은 각도로) 다시 돌린다.
  큐브·그리퍼 모두 90° 대칭이라 최대 45° 만 돈다. 그리퍼 축이 J6 축과 같다고 본다 (돌려도 손끝 XY 그대로).
  중심 맞추기 (2026-11-13): 각도를 맞춘 뒤 카메라로 큐브 중심을 다시 재서 (여러 장 중앙값) 그 바로 위로 옮긴 다음
  내려간다. 못 재면(그리퍼에 가려짐) 처음 좌표로, CENTER_MAX_MM 넘게 어긋나면(큐브가 움직임) 멈춘다.
  놓기(place_at): 접근 높이로 -> 놓을 자리 위 -> 하강 -> 열기 -> 위로. pick_and_place 는 둘을 이어 붙이고
  닫은 뒤 잡지 못했으면(빈손·모름) 옮기지 않는다.
  모든 이동은 RobotLink.move_tcp_to (안전 검사 포함) 를 거치고, 단계마다 끝났는지 확인한다.
  정지(link.stop) 를 누르면 진행 중인 단계가 실패로 끝나고 이후 단계는 보내지 않는다.
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

from aris3_real_bridge.marker_calib import rigid_check

CALIB_PATH = os.path.expanduser('~/.config/aris3/real_camera_calib.json')

APPROACH_MM = 100.0         # 큐브 위 접근 높이 (집는 높이 + 이 값)
TRAVEL_VEL = 80.0           # mm/s  접근 높이에서 수평 이동 (2026-11-09 40 -> 80)
DESCEND_VEL = 40.0          # mm/s  하강·들기 (2026-11-09 20 -> 40)
GRIP_CLOSE_POS = 740        # 그리퍼 닫기 목표 (0 열림 ~ 740 닫힘)
GRIP_CURRENT_MA = 300       # 닫는 힘
GRIP_HOLD_MAX = 700         # 닫은 뒤 위치가 이보다 작으면 물체에 막힌 것 = 잡음
GRIP_HOLD_MIN = 100         # 이보다 작으면 닫히지 않은 것 (토크 꺼짐 등) = 잡음 아님 (2026-11-09 실측 위치 2)
# 먼 수평 이동은 나눠서 간다: RobotLink 는 한 번에 400 mm 넘는 이동을 거부한다 (보정 오류 방지 안전장치).
TRAVEL_SEG_MM = 250.0       # 한 구간 최대 수평 거리 (높이 변화가 있어도 400 mm 안에 들게)
TRAVEL_R_SAFE_MM = 300.0    # 경유점이 베이스에서 이보다 가까우면 바깥으로 밀어 몸통 근처를 지나지 않게
# 쌓기 (2026-11-13): 받침 큐브 위에 놓을 때 놓는 높이를 큐브 한 개 높이만큼 올린다.
# 집은 큐브의 아랫면이 받침 큐브 윗면에 닿는 높이 = 집는 높이 + 큐브 높이. 큐브 크기는 실물과 시뮬레이션이 다르다:
#   실물 작업 큐브 4 cm, 시뮬레이션(aris3_bringup object_layout_real_like.csv) 5 cm
# 처음에 4 cm 하나로 두었더니 시뮬레이션에서 1 cm 더 눌러 받침 큐브를 옆으로 밀어냈다 (2026-11-13 실측).
STACK_MM = 40.0             # 실물 큐브 높이 [mm] (기본값). 브리지 파라미터 cube_mm 로 바꿈 (가상 모드 50)
STACK_SIM_MM = 50.0         # 시뮬레이션 큐브 높이 [mm]
MAX_LEVEL = 4               # 깊이로 잰 층이 이보다 높으면 집거나 쌓지 않는다 (잘못 잰 값으로 높이 올라가지 않게, 2026-11-13)
STACK_CLEAR_MM = 3.0        # 쌓을 때 더 띄우는 여유 [mm] (보정·파지 오차로 받침을 누르지 않게, _place_steps 의 2 mm 에 더함)
# 그리퍼 각도 맞추기 (2026-11-13): 접근 높이에서 J6(손목)만 돌려 그리퍼 면을 큐브 면(집기)·팔 방향 또는 받침 큐브(놓기)에 맞춘다.
# 큐브·그리퍼 모두 90° 대칭이라 목표는 90° 마다 같다 -> 최대 45° 만 돌린다. 이동은 RobotLink.jog_joint (안전 검사·한 번 10°).
ALIGN_MIN_DEG = 2.0         # 이보다 작게 어긋나면 돌리지 않는다 (검출 각도 흔들림 정도)
ALIGN_VEL = 30.0            # deg/s  손목 회전 속도
ALIGN_STEP_DEG = 10.0       # 한 번에 돌리는 최대 각 (RobotLink MAX_JOINT_STEP 과 같음)
ALIGN_TOOL_DOWN = 0.95      # 툴 Z 축과 베이스 -Z 의 내적이 이보다 작으면(아래를 보지 않음) 맞추지 않는다
ALIGN_J6_SOFT = 180.0       # 돌린 뒤 J6 가 이보다 크면 반대 방향(90° 다른 같은 모양) 을 고른다 (손목이 계속 감기지 않게)

# 큐브 중심 맞추기 (2026-11-13): 접근 높이에서 멈춘 뒤 카메라로 큐브를 다시 여러 장 재서(중앙값) 그리퍼를 큐브 중심
# 바로 위로 옮긴 다음 내려간다 (처음 고른 뒤 큐브가 밀렸거나, 한 장 검출이 흔들린 만큼을 고침).
CENTER_MIN_MM = 1.5         # 이보다 작게 어긋나면 옮기지 않는다
CENTER_MAX_MM = 25.0        # 이보다 크게 어긋나면 큐브가 움직인 것 -> 집지 않고 멈춘다 (다시 골라야 함)
CENTER_VEL = 20.0           # mm/s  중심 맞추기 수평 이동


def wrap90(a):
    """각도 [deg] 를 90° 대칭으로 접어 [-45, 45) 로 (정사각형 큐브·그리퍼는 90° 마다 같은 모양)."""
    return (a + 45.0) % 90.0 - 45.0


def tool_axes(posx):
    """Doosan TCP 자세 posx = [x, y, z, A, B, C] (ZYZ 오일러, deg) -> (툴 X 축, 툴 Z 축) 베이스 기준 단위 벡터."""
    a, b, c = (math.radians(float(v)) for v in posx[3:6])
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    # R = Rz(A) Ry(B) Rz(C) 의 첫째·셋째 열
    x_axis = (ca * cb * cc - sa * sc, sa * cb * cc + ca * sc, -sb * cc)
    z_axis = (ca * sb, sa * sb, cb)
    return x_axis, z_axis


def tool_yaw_deg(posx):
    """툴 X 축을 베이스 XY 평면에 내린 방향 [deg] (그리퍼 면 방향). 툴이 아래를 볼 때만 뜻이 있다."""
    x_axis, _ = tool_axes(posx)
    return math.degrees(math.atan2(x_axis[1], x_axis[0]))


def radial_yaw_deg(x, y):
    """베이스에서 (x, y) 로 뻗는 팔 방향 [deg]. 놓을 때 큐브 면을 이 방향과 평행·수직으로 맞춘다."""
    return math.degrees(math.atan2(y, x))


def plan_j6(yaw_now, yaw_target, tool_z_dot, j6_now):
    """그리퍼 면을 yaw_target (90° 대칭) 에 맞추는 J6 회전 [deg].

    tool_z_dot: 툴 Z 축 · 베이스 Z 축 (아래를 보면 약 -1). J6 를 d 만큼 돌리면 그리퍼 yaw 는 tool_z_dot x d 만큼 돈다.
    가장 적게 도는 쪽(최대 45°)을 고르되, 그러면 J6 가 ALIGN_J6_SOFT 를 넘는 경우에는 90° 다른 쪽 중
    J6 가 0 에 더 가까운 것을 고른다.
    """
    d0 = wrap90(yaw_target - yaw_now)
    options = sorted((d0 + k * 90.0 for k in (-1, 0, 1)), key=abs)[:2]   # 가장 작은 회전과 그다음
    best = options[0]
    if abs(j6_now + best / tool_z_dot) > ALIGN_J6_SOFT:
        best = min(options, key=lambda d: abs(j6_now + d / tool_z_dot))
    return best / tool_z_dot


def travel_waypoints(a, b, seg=TRAVEL_SEG_MM, r_safe=TRAVEL_R_SAFE_MM):
    """수평 이동 a(x, y) -> b(x, y) 를 seg 이하 구간으로 나눈 점 목록 (마지막 = b).

    중간 점이 베이스(원점)에서 r_safe 보다 가까우면 같은 방향으로 r_safe 까지 밀어낸다.
    """
    ax, ay = a
    bx, by = b
    d = math.hypot(bx - ax, by - ay)
    n = max(1, math.ceil(d / seg))
    pts = []
    for i in range(1, n + 1):
        t = i / n
        qx, qy = ax + t * (bx - ax), ay + t * (by - ay)
        r = math.hypot(qx, qy)
        if i < n and r < r_safe:
            if r < 1.0:                      # 베이스 바로 위: 이동 방향에 수직으로 민다
                ux, uy = -(by - ay) / max(d, 1e-9), (bx - ax) / max(d, 1e-9)
            else:
                ux, uy = qx / r, qy / r
            qx, qy = ux * r_safe, uy * r_safe
        pts.append((qx, qy))
    pts[-1] = (bx, by)
    return pts


def judge_hold(d):
    """그리퍼의 마지막 응답 d (position, torque) 로 물체를 잡고 있는지 판단. 돌려줌 (True 잡음 / False 빈손 / None 모름, 이유).

    닫는 중 물체에 막히면 위치가 GRIP_HOLD_MIN ~ GRIP_HOLD_MAX 사이에서 멈춘다. 끝까지 닫히면(MAX 이상) 빈손,
    MIN 보다 작으면 열려 있는 것(빈손), 토크가 꺼져 있으면 손가락이 힘을 주지 않으므로 빈손이다.
    """
    if not d:
        return None, '그리퍼 위치를 읽지 못함'
    pos = d.get('position', 0)
    if not d.get('torque'):
        return False, f'그리퍼 위치 {pos}, 토크 꺼짐 -> 닫히지 않음 (빈손으로 판단)'
    if pos < GRIP_HOLD_MIN:
        return False, f'그리퍼 위치 {pos} -> 닫히지 않음 (빈손으로 판단)'
    if pos < GRIP_HOLD_MAX:
        return True, f'그리퍼 위치 {pos} -> 물체에 막혀 멈춤 (잡음)'
    return False, f'그리퍼 위치 {pos} -> 끝까지 닫힘 (빈손)'


class PixelCalibration:
    """영상 픽셀 (u, v) -> 로봇 베이스 (x, y) mm, 집는 높이 z mm."""

    def __init__(self):
        """빈 보정: 점 없음, 변환 행렬 없음 (ready() 가 False)."""
        self.points = []        # dict(u, v, x, y, z, label)
        self.H = None
        self.grasp_z = None
        self.errors_mm = []     # 점마다 오차 (5 점 이상이면 그 점을 빼고 맞춘 뒤 잰 값)
        # 3D 일관성 (2026-11-14, 탁자 블록 마커): 마커 3D 점(카메라)이 있는 점끼리 강체 변환 하나로 맞춘 잔차 [mm].
        # 4 점 미만이면 빈 목록. 한 점만 크면 그 점을 잘못 기록한 것 (블록을 잡는 자세가 어긋남 등)
        self.rigid_mm = []

    # ---- 점 관리
    def add(self, u, v, x, y, z, label='', cam=None):
        """보정점 하나 (픽셀 -> TCP). cam: 탁자 블록 마커의 카메라 좌표 3D 점 [mm] (있으면 3D 일관성 검사에 씀)."""
        pt = dict(u=float(u), v=float(v), x=float(x), y=float(y), z=float(z), label=label)
        if cam is not None:
            pt['cam'] = [float(c) for c in cam]
        self.points.append(pt)
        self.fit()

    def remove_last(self):
        """마지막으로 기록한 점을 지우고 다시 맞춘다 (잘못 찍은 점 되돌리기)."""
        if self.points:
            self.points.pop()
        self.fit()

    def clear(self):
        """모든 점을 지운다."""
        self.points = []
        self.fit()

    # ---- 계산
    @staticmethod
    def _homography(pts):
        """점들의 픽셀 (u, v) -> 로봇 (x, y) 를 가장 잘 맞추는 3x3 호모그래피 (최소제곱)."""
        src = np.array([[p['u'], p['v']] for p in pts], np.float64)
        dst = np.array([[p['x'], p['y']] for p in pts], np.float64)
        H, _ = cv2.findHomography(src, dst, 0)      # 0 = 모든 점 최소제곱 (RANSAC 아님)
        return H

    @staticmethod
    def _apply(H, u, v):
        """호모그래피 H 로 픽셀 (u, v) 를 로봇 (x, y) mm 로 바꾼다 (동차 좌표를 w 로 나눔)."""
        q = H @ np.array([u, v, 1.0])
        return q[0] / q[2], q[1] / q[2]

    def fit(self):
        """점이 4개 이상이면 행렬·집는 높이·점별 오차를 다시 계산한다. 맞추면 True.

        점별 오차: 5 점 이상이면 그 점을 빼고 맞춘 행렬로 그 점을 예측해 잰다 (보정이 실제로 맞는지 검증).
        """
        self.H, self.grasp_z, self.errors_mm = None, None, []
        withcam = [p for p in self.points if p.get('cam')]
        r = rigid_check([p['cam'] for p in withcam], [[p['x'], p['y'], p['z']] for p in withcam])
        self.rigid_mm = [] if r is None else [float(v) for v in r]
        if len(self.points) < 4:
            return False
        H = self._homography(self.points)
        if H is None:
            return False
        self.H = H
        self.grasp_z = float(np.median([p['z'] for p in self.points]))
        for i, p in enumerate(self.points):
            if len(self.points) >= 5:               # 그 점을 빼고 맞춘 변환으로 그 점을 예측 (검증)
                Hi = self._homography(self.points[:i] + self.points[i + 1:])
            else:
                Hi = H
            if Hi is None:
                self.errors_mm.append(float('nan'))
                continue
            x, y = self._apply(Hi, p['u'], p['v'])
            self.errors_mm.append(math.hypot(x - p['x'], y - p['y']))
        return True

    def ready(self):
        """변환을 쓸 수 있는가 (점 4개 이상으로 행렬을 맞춤)."""
        return self.H is not None

    def to_robot(self, u, v):
        """픽셀 -> (x, y) mm 또는 None."""
        if self.H is None:
            return None
        return self._apply(self.H, u, v)

    def angle_to_robot(self, u, v, angle_px_deg, step_px=20.0):
        """영상에서 (u, v) 큐브 면 방향 angle_px_deg -> 로봇 베이스 기준 방향 [deg] (90° 대칭, [-45, 45)). 보정 없으면 None.

        영상의 방향은 원근·회전·뒤집힘 때문에 그대로 쓸 수 없어서, 그 방향으로 step_px 떨어진 점을 함께 로봇 좌표로
        바꿔 두 점을 잇는 방향을 잰다 (호모그래피를 그 점에서 미분한 것과 같다).
        """
        if self.H is None:
            return None
        t = math.radians(angle_px_deg)
        x0, y0 = self._apply(self.H, u, v)
        x1, y1 = self._apply(self.H, u + step_px * math.cos(t), v + step_px * math.sin(t))
        return wrap90(math.degrees(math.atan2(y1 - y0, x1 - x0)))

    def summary(self):
        """화면·로그용 한 줄 요약: 점 개수, 집는 높이, 오차 평균/최대."""
        n = len(self.points)
        if not self.ready():
            return f'보정점 {n}개 (4개 이상 필요)'
        e = [v for v in self.errors_mm if not math.isnan(v)]
        how = '한 점씩 빼고 검증' if n >= 5 else '맞춘 점 자체 오차 - 5개 이상이면 검증 가능'
        out = (f'보정점 {n}개 · 집는 높이 Z {self.grasp_z:.1f} mm · 오차 평균 {np.mean(e):.1f} / '
               f'최대 {np.max(e):.1f} mm ({how})')
        if self.rigid_mm:
            out += f' · 마커 3D 일관성 최대 {max(self.rigid_mm):.1f} mm ({len(self.rigid_mm)}점)'
        return out

    # ---- 파일
    def save(self, path=CALIB_PATH):
        """점들을 JSON 으로 저장하고 경로를 돌려준다."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = {'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'points': self.points,
                'note': '픽셀(u,v) -> 로봇 베이스 TCP (x,y,z) mm, cam = 블록 마커 카메라 3D 점. aris3_real_bridge real_pick.py'}
        with open(path, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
        return path

    def load(self, path=CALIB_PATH):
        """저장한 보정 파일을 읽어 다시 맞춘다. 파일이 없으면 OSError."""
        with open(path) as f:
            data = json.load(f)
        self.points = data['points']
        return self.fit()


class PickSequence:
    """접근 -> 열기 -> 하강 -> 닫기 -> 들기 를 작업 스레드에서 한 단계씩 실행한다.

    link: e0509_manual.robot_link.RobotLink (move_tcp_to, wait_motion, gripper_command, gripper)
    log: 함수(str)
    """

    def __init__(self, link, log):
        """link: RobotLink (또는 시험용 가짜), log: 문자열을 받는 로그 함수."""
        self.link, self.log = link, log
        self.stack_mm = STACK_MM  # 쌓기 높이 = 큐브 한 개 [mm] (브리지가 모드에 맞게 바꿈)
        self.running = False
        self.holding = None     # 마지막 닫기 결과: True 잡음 / False 빈손 / None 모름
        # 진행 상황 (aris3_real_bridge 가 상태 토픽·액션 feedback 으로 내보냄, 2026-11-10)
        self.name = ''          # 지금(또는 마지막) 순서 이름
        self.step = ''          # 지금 단계 이름
        self.step_index = 0     # 0 부터
        self.step_count = 0
        self.last_ok = False    # 마지막 순서가 모든 단계를 끝까지 마쳤는가
        # 「계속해」 (2026-11-11): 정지로 끊긴 순서의 남은 단계. 실패(빈손·거부)로 끊긴 것은 잇지 않는다
        self.stop_requested = False
        self.resumable = None   # (이름, 끊긴 단계부터의 [(이름, 함수)]) 또는 None

    def request_stop(self):
        """정지 요청이 왔다고 표시 (브리지의 정지 서비스·액션 취소가 부른다). 끊긴 순서를 이을 수 있게 된다."""
        self.stop_requested = True

    def forget_resume(self, why=''):
        """이을 순서를 버린다 - 정지 뒤 다른 동작(홈·조그·손 안내·접속 끊기)으로 팔 위치가 바뀌었을 때."""
        if self.resumable:
            self.log(f'「{self.resumable[0]}」 이어서 하기 취소' + (f': {why}' if why else ''))
        self.resumable = None

    def resume(self):
        """정지로 끊긴 순서를 멈춘 단계부터 다시 실행한다. 이을 것이 없거나 실행 중이면 False."""
        if self.running:
            self.log('이어서 하기 거부: 순서가 진행 중')
            return False
        if not self.resumable:
            self.log('이어서 하기 거부: 정지로 멈춘 순서가 없음')
            return False
        name, steps = self.resumable
        return self.start(f'{name} (이어서)', steps)

    def start(self, name, steps):
        """단계 목록 steps [(이름, 함수)] 를 작업 스레드에서 실행한다. 이미 실행 중이면 거부(False)."""
        if self.running:
            self.log(f'{name} 거부: 이전 순서가 진행 중')
            return False
        self.running = True
        self.stop_requested = False
        self.resumable = None                   # 새 순서를 시작하면 예전에 끊긴 순서는 버린다
        self.name, self.step, self.step_index, self.step_count, self.last_ok = name, '', 0, len(steps), False
        threading.Thread(target=self._run, args=(name, steps), daemon=True).start()
        return True

    def _run(self, name, steps):
        """작업 스레드 본체: 단계를 하나씩 실행하고 끝날 때까지 기다린다. 하나라도 실패·정지하면 남은 단계는 보내지 않는다."""
        try:
            self.log(f'{name}: 시작 ({len(steps)}단계)')
            for i, (label, fn) in enumerate(steps, 1):
                self.step, self.step_index = label, i - 1
                ok = bool(fn())
                if ok and not self.link.wait_motion():
                    ok = None                    # 보냈지만 끝나지 않음
                if ok is not True:
                    what = '실패 또는 정지' if ok is False else '끝나지 않음 또는 정지'
                    self.log(f'{name}: {i}/{len(steps)} 「{label}」 {what} -> 이후 단계 취소')
                    self._keep_for_resume(name, steps[i - 1:], label)
                    return
            self.log(f'{name}: 완료')
            self.last_ok = True
        finally:
            self.running = False

    def _keep_for_resume(self, name, rest, label):
        """정지로 끊겼으면 끊긴 단계부터 남겨 둔다 (그 단계는 처음부터 다시 - 이동은 목표 좌표, 그리퍼는 같은 명령)."""
        if self.stop_requested:
            base = name[:-len(' (이어서)')] if name.endswith(' (이어서)') else name
            self.resumable = (base, rest)
            self.log(f'{base}: 「{label}」 부터 이어서 할 수 있음 (「계속해」, 남은 {len(rest)}단계)')

    # ---- 단계 만들기
    def _move(self, x, y, z, vel, label):
        return (label, lambda: self.link.move_tcp_to(x, y, z, vel, label=label))

    def _travel(self, x, y, z, vel, label):
        """접근 높이에서 (x, y) 로 수평 이동. 실행할 때의 현재 위치에서 멀면 경유점을 거친다."""
        def run():
            """실행할 때 현재 위치를 읽어 경유점을 정하고 차례로 이동 (먼 이동을 나눠 몸통 근처를 피함)."""
            p = self.link.posx
            if p is None:
                self.log(f'{label} 거부: 현재 좌표 모름')
                return False
            pts = travel_waypoints((p[0], p[1]), (x, y))
            for i, (wx, wy) in enumerate(pts[:-1], 1):
                if not self.link.move_tcp_to(wx, wy, z, vel, label=f'{label} - 경유 {i}/{len(pts) - 1}'):
                    return False
                if not self.link.wait_motion():
                    return False
            return self.link.move_tcp_to(x, y, z, vel, label=label)
        return (label, run)

    def _grip(self, close):
        """그리퍼 열기/닫기 단계 (닫기는 GRIP_CURRENT_MA 전류로 - 큐브를 잡을 만큼만)."""
        label = '그리퍼 닫기' if close else '그리퍼 열기'

        def run():
            """명령을 보낸다. 열기를 받아들였으면 빈손으로 표시 (예전에는 잡음 표시가 남아 다음 순서를 막았다, 2026-11-14)."""
            ok = self.link.gripper_command('move', pos=GRIP_CLOSE_POS if close else 0, cur=GRIP_CURRENT_MA, vel=1500,
                                           label=label)
            if ok and not close:
                self.holding = False
            return ok
        return (label, run)

    def _align(self, yaw_target, label):
        """그리퍼 각도 맞추기 단계: 지금 자리(접근 높이)에서 J6 만 돌려 그리퍼 면을 yaw_target [deg, 90° 대칭] 에 맞춘다.

        실행할 때의 TCP 자세·J6 로 회전량을 계산한다 (「계속해」로 이어도 맞게). 자세를 모르거나 툴이 아래를
        보지 않으면 돌리지 않고 넘어간다 (예전처럼 지금 각도로 집기·놓기). 한 번에 ALIGN_STEP_DEG 이하로 나눠 돌린다.
        """
        def run():
            """회전량 계산 -> 작으면 건너뜀 -> jog_joint(J6) 를 나눠 보냄 (마지막 것은 _run 이 끝날 때까지 기다림)."""
            p, j = self.link.posx, getattr(self.link, 'joints_deg', None)
            if p is None or j is None or len(j) < 6:
                self.log(f'{label} 건너뜀: 지금 자세를 모름')
                self.link.last_ok = True
                return True
            _, z_axis = tool_axes(p)
            if -z_axis[2] < ALIGN_TOOL_DOWN:
                self.log(f'{label} 건너뜀: 그리퍼가 아래를 보지 않음 (툴 Z·아래 {-z_axis[2]:.2f})')
                self.link.last_ok = True
                return True
            yaw = tool_yaw_deg(p)
            d_j6 = plan_j6(yaw, yaw_target, z_axis[2], float(j[5]))
            if abs(d_j6) < ALIGN_MIN_DEG:
                self.log(f'{label}: 이미 맞음 (그리퍼 {yaw:.0f}°, 목표 {yaw_target:.0f}° ±90°)')
                self.link.last_ok = True
                return True
            n = max(1, math.ceil(abs(d_j6) / ALIGN_STEP_DEG - 1e-9))
            self.log(f'{label}: 그리퍼 {yaw:.0f}° -> 목표 {yaw_target:.0f}° (±90°) = J6 {d_j6:+.1f}° ({n}번에 나눠)')
            for k in range(n):
                if not self.link.jog_joint(5, d_j6 / n, ALIGN_VEL):
                    return False
                if k < n - 1 and not self.link.wait_motion():
                    return False
            return True
        return (label, run)

    def _center(self, tgt, z, locate, label='큐브 중심 맞추기'):
        """큐브 중심 맞추기 단계: locate() 로 큐브 중심 로봇 (x, y) 를 다시 재서 tgt 를 고치고 그 위로 수평 이동.

        tgt: {'x', 'y'} - 이후 하강·들기 단계가 실행할 때 읽는다. z: 지금 높이(접근 높이).
        locate(): (x, y) 또는 못 잰 이유(str). 못 재면(그리퍼에 가려짐 등) 처음 좌표로 그대로 집는다.
        CENTER_MAX_MM 넘게 어긋나면 큐브가 움직인 것으로 보고 멈춘다 (엉뚱한 곳을 집지 않게).
        """
        def run():
            """다시 잰 중심과 지금 목표의 차이에 따라 건너뜀 / 이동 / 멈춤."""
            r = locate()
            if isinstance(r, str) or r is None:
                self.log(f'{label} 건너뜀: {r or "큐브 위치를 다시 재지 못함"} - 처음 좌표로 집음')
                self.link.last_ok = True
                return True
            nx, ny = float(r[0]), float(r[1])
            d = math.hypot(nx - tgt['x'], ny - tgt['y'])
            if d < CENTER_MIN_MM:
                self.log(f'{label}: 이미 중심 위 (차이 {d:.1f} mm)')
                self.link.last_ok = True
                return True
            if d > CENTER_MAX_MM:
                self.log(f'{label} 실패: 큐브가 처음 자리에서 {d:.0f} mm 떨어져 있음 (움직였음) - 다시 골라 주세요')
                return False
            self.log(f'{label}: ({tgt["x"]:.1f}, {tgt["y"]:.1f}) -> ({nx:.1f}, {ny:.1f}) mm, {d:.1f} mm 옮김')
            tgt['x'], tgt['y'] = nx, ny
            return self.link.move_tcp_to(nx, ny, z, CENTER_VEL, label=label)
        return (label, run)

    def _j6_turn_90(self, label):
        """J6 를 90° 돌린다 (지금 J6 가 0 보다 크면 -90°, 아니면 +90° - 손목이 한쪽으로 감기지 않게). 10° 씩 나눠."""
        def run():
            """실행할 때 J6 를 읽어 방향을 정하고 jog_joint 를 나눠 보낸다."""
            j = getattr(self.link, 'joints_deg', None)
            d = -90.0 if (j is not None and len(j) >= 6 and float(j[5]) > 0) else 90.0
            n = math.ceil(abs(d) / ALIGN_STEP_DEG - 1e-9)
            for k in range(n):
                if not self.link.jog_joint(5, d / n, ALIGN_VEL):
                    return False
                if k < n - 1 and not self.link.wait_motion():
                    return False
            return True
        return (label, run)

    def block_calibration_point(self, x, y, z_grasp, yaw, record, measure, home_vel=30.0):
        """탁자 블록 자동 보정점 하나 (2026-11-14): 블록을 잡아 그리퍼 가운데로 맞춘 뒤 TCP 를 기록하고, 홈으로 비켜
        마커를 다시 잰다. 사람이 그리퍼를 맞추지 않아도 된다.

        (x, y): 지금 보정으로 계산한 블록 위치 (오차 몇 mm - 손가락이 약 106 mm 벌어져 충분), yaw: 블록 면 방향.
        닫으면 블록이 손가락 사이 가운데로 밀리고(한 축), 손목을 90° 돌려 다시 닫으면 다른 축도 가운데 -> 그때 TCP (x, y)
        = 블록 중심 (그리퍼 중심이 J6 축 위라고 봄). record(): 그때 TCP 를 저장, measure(): 홈에서 마커를 재서 점 추가.
        빈손이면(블록을 못 잡음) 기록하지 않고 멈춘다.
        """
        def wrap(fn):
            """움직이지 않는 단계: fn 결과를 돌려주고 wait_motion 이 바로 통과하게 한다."""
            def run():
                ok = bool(fn())
                self.link.last_ok = True
                return ok
            return run
        top = z_grasp + APPROACH_MM
        steps = [self._travel(x, y, top, TRAVEL_VEL, '블록 위 접근 높이로')]
        if yaw is not None:
            steps.append(self._align(yaw, '블록 각도 맞추기'))
        steps += [
            self._grip(False),
            self._move(x, y, z_grasp, DESCEND_VEL, '집는 높이로 하강'),
            self._grip(True),                                  # 한 축 가운데로
            self._grip(False),
            self._move(x, y, top, DESCEND_VEL, '손목 돌리기 전 올리기'),
            self._j6_turn_90('손목 90° 돌리기'),
            self._move(x, y, z_grasp, DESCEND_VEL, '집는 높이로 다시 하강'),
            self._grip(True),                                  # 다른 축도 가운데로
            self._check_hold(require=True),
            ('블록 중심 기록', wrap(record)),
            self._grip(False),
            self._move(x, y, top, DESCEND_VEL, '올리기'),
            ('홈으로 (카메라에서 비키기)', lambda: self.link.move_home(home_vel)),
            ('마커 다시 재기', wrap(measure)),
        ]
        return self.start('블록 자동 보정점', steps)

    def _move_tgt(self, tgt, z, vel, label):
        """tgt {'x', 'y'} 를 실행할 때 읽어 이동 (중심 맞추기가 고친 좌표로 하강·들기)."""
        return (label, lambda: self.link.move_tcp_to(tgt['x'], tgt['y'], z, vel, label=label))

    def update_hold(self):
        """그리퍼의 마지막 응답으로 잡음/빈손/모름을 다시 판단해 holding 에 저장하고 로그를 남긴다. 판단 결과를 돌려준다."""
        self.holding, why = judge_hold(self.link.gripper)
        self.log(why if self.holding is not None else f'잡았는지 모름: {why}')
        return self.holding

    def _check_hold(self, require=False):
        """require=True 이면 빈손이거나 모를 때 이후 단계(옮겨 놓기)를 취소한다."""
        def check():
            """닫은 뒤 그리퍼 위치·토크로 잡았는지 판단해 holding 에 저장 (require 면 빈손일 때 실패)."""
            self.update_hold()
            self.link.last_ok = True        # wait_motion 이 바로 통과하게 (움직임 없는 단계)
            return self.holding is True if require else True
        return ('잡았는지 확인', check)

    def level_lift(self, level):
        """층 -> 집는 높이에서 올릴 양 [mm]. 1층(또는 모름 0) = 0, 2층 = 큐브 하나 ..."""
        return max(0, int(level) - 1) * self.stack_mm

    def _place_steps(self, x, y, z_grasp, lift=0.0, top=None, yaw=None):
        """지금 자리에서 위로 -> (x, y) 위 접근 높이로 -> (각도 맞추기) -> 놓는 높이로 하강 -> 열기 -> 위로.

        lift: 놓는 높이를 이만큼 올린다 [mm] (쌓기 = 받침 층 x 큐브 높이 + 여유).
        top: 이동 높이 (기본 = 집는 높이 + 접근 높이 + 받침이 2층 이상이면 그만큼 더 - 들고 가는 큐브가 받침에 닿지 않게).
        yaw: 놓는 큐브 면 방향 [deg, 90° 대칭] (팔 방향 또는 받침 큐브 각도, 2026-11-13). None = 돌리지 않음.
        """
        if top is None:
            top = z_grasp + APPROACH_MM + max(0.0, lift - self.stack_mm - STACK_CLEAR_MM)

        def rise():                         # 지금 XY 에서 접근 높이까지 올라간 뒤 수평 이동
            """이미 접근 높이 이상이면 건너뛰고, 아니면 지금 XY 에서 접근 높이까지 수직으로 올린다."""
            p = self.link.posx
            if p is None:
                self.log('놓기 거부: 현재 좌표 모름')
                return False
            if p[2] >= top - 1.0:
                self.link.last_ok = True
                return True
            return self.link.move_tcp_to(p[0], p[1], top, DESCEND_VEL, label='접근 높이로 올리기')
        return [
            ('접근 높이로 올리기', rise),
            self._travel(x, y, top, TRAVEL_VEL, '놓을 자리 위로'),
        ] + ([self._align(yaw, '놓는 각도 맞추기')] if yaw is not None else []) + [
            self._move(x, y, z_grasp + lift + 2.0, DESCEND_VEL, '놓는 높이로 하강'),
            self._grip(False),
            self._move(x, y, top, DESCEND_VEL, '위로 빠지기'),
        ]

    def _pick_approach(self, tgt, top, yaw, locate):
        """큐브 위 접근: 이동 -> (집는 각도 맞추기) -> (큐브 중심 맞추기). yaw / locate 가 None 이면 그 단계 없음."""
        return ([self._travel(tgt['x'], tgt['y'], top, TRAVEL_VEL, '큐브 위 접근 높이로')]
                + ([self._align(yaw, '집는 각도 맞추기')] if yaw is not None else [])
                + ([self._center(tgt, top, locate)] if locate is not None else []))

    def hover(self, x, y, z_grasp, pick_lift=0.0, yaw=None, locate=None):
        """고른 큐브 위 접근 높이로만 간다 (집기 전에 위치·각도가 맞는지 눈으로 확인용).

        pick_lift: 쌓인 큐브면 층 높이. yaw: 큐브 면 방향 [deg] - 주면 그리퍼도 그 각도로 돌려 둔다.
        locate: 큐브 중심을 다시 재는 함수 - 주면 큐브 중심 바로 위로 맞춰 둔다.
        """
        tgt = {'x': x, 'y': y}
        return self.start('큐브 위로', self._pick_approach(tgt, z_grasp + pick_lift + APPROACH_MM, yaw, locate))

    def pick(self, x, y, z_grasp, pick_lift=0.0, yaw=None, locate=None):
        """큐브 집기: 접근 높이로 이동 -> (각도 맞추기) -> (중심 맞추기) -> 열기 -> 하강 -> 닫기 -> 잡았는지 확인 -> 들기.

        pick_lift: 쌓인 큐브의 맨 위를 집을 때 집는 높이를 올릴 양 [mm] (level_lift, 2026-11-13).
        yaw: 큐브 면 방향 [deg, 로봇 베이스, 90° 대칭] (2026-11-13). None = 돌리지 않음.
        locate: 접근 높이에서 큐브 중심 (x, y) 를 다시 재는 함수 (2026-11-13). None = 처음 좌표 그대로.
        """
        z_pick = z_grasp + pick_lift
        top = z_pick + APPROACH_MM
        tgt = {'x': x, 'y': y}
        return self.start('집기', self._pick_approach(tgt, top, yaw, locate) + [
            self._grip(False),
            self._move_tgt(tgt, z_pick, DESCEND_VEL, '집는 높이로 하강'),
            self._grip(True),
            self._check_hold(),
            self._move_tgt(tgt, top, DESCEND_VEL, '들어 올리기'),
        ])

    def place_at(self, x, y, z_grasp, yaw=None):
        """들고 있는 물체를 (x, y) 에 놓는다 (카메라 보정으로 바꾼 놓을 자리). yaw: 놓는 각도 (None = 돌리지 않음)."""
        return self.start('그 자리에 놓기', self._place_steps(x, y, z_grasp, yaw=yaw))

    def pick_and_place(self, x, y, z_grasp, px, py, lift=0.0, name='집어서 옮기기', pick_lift=0.0,
                       yaw=None, place_yaw=None, locate=None):
        """(x, y) 의 큐브를 집어 (px, py) 에 놓는다. 잡지 못했으면 옮기지 않는다.

        lift: 놓는 높이를 올릴 양 [mm] (pick_and_stack 이 받침 높이를 준다). name: 순서 이름 (로그·화면).
        pick_lift: 쌓인 큐브의 맨 위를 집을 때 집는 높이를 올릴 양 [mm].
        yaw / place_yaw: 집는 큐브 면 방향 / 놓는 방향 [deg, 90° 대칭] (2026-11-13). None = 돌리지 않음.
        locate: 접근 높이에서 큐브 중심을 다시 재는 함수 (pick 과 같음).
        이동 높이는 집는 쪽·놓는 쪽 중 높은 쪽에 맞춘다 (들고 가는 큐브가 쌓인 큐브에 닿지 않게).
        """
        z_pick = z_grasp + pick_lift
        top = z_grasp + APPROACH_MM + max(pick_lift, lift - self.stack_mm - STACK_CLEAR_MM, 0.0)
        tgt = {'x': x, 'y': y}
        return self.start(name, self._pick_approach(tgt, top, yaw, locate) + [
            self._grip(False),
            self._move_tgt(tgt, z_pick, DESCEND_VEL, '집는 높이로 하강'),
            self._grip(True),
            self._check_hold(require=True),
            self._move_tgt(tgt, top, DESCEND_VEL, '들어 올리기'),
        ] + self._place_steps(px, py, z_grasp, lift, top, yaw=place_yaw)[1:])

    def pick_and_stack(self, x, y, z_grasp, bx, by, base_level=1, pick_lift=0.0, yaw=None, base_yaw=None,
                       locate=None):
        """(x, y) 의 큐브를 집어 (bx, by) 에 있는 받침 위에 올린다.

        base_level: 받침이 몇 층인지 (맨 위 큐브의 층, 깊이로 잼). 놓는 높이 = 집는 높이 + 받침 층 x 큐브 높이 + 여유.
        yaw: 집는 큐브 면 방향. base_yaw: 받침 큐브 면 방향 - 올리는 큐브를 받침과 같은 각도로 놓는다 (2026-11-13).
        """
        lift = max(1, int(base_level)) * self.stack_mm + STACK_CLEAR_MM
        return self.pick_and_place(x, y, z_grasp, bx, by, lift=lift, name='쌓기', pick_lift=pick_lift,
                                   yaw=yaw, place_yaw=base_yaw, locate=locate)

    def place_on(self, bx, by, z_grasp, base_level=1, base_yaw=None):
        """지금 들고 있는 큐브를 (bx, by) 에 있는 받침 위에 올린다 (2026-11-14, 「빨간 큐브 위로 올려놔」).

        집지 않고 놓기만 한다 (쌓기의 뒤쪽 절반). 놓는 높이 = 집는 높이 + 받침 층 x 큐브 높이 + 여유,
        base_yaw: 받침 큐브 면 방향 - 들고 있는 큐브를 받침과 같은 각도로 놓는다 (None = 돌리지 않음).
        """
        lift = max(1, int(base_level)) * self.stack_mm + STACK_CLEAR_MM
        return self.start('올려놓기', self._place_steps(bx, by, z_grasp, lift, yaw=base_yaw))

    def place_here(self, z_grasp):
        """지금 XY 에서 집는 높이(+2 mm) 로 내려 놓고 다시 올라온다."""
        p = self.link.posx
        if p is None:
            self.log('내려놓기 거부: 현재 좌표 모름')
            return False
        x, y = p[0], p[1]
        top = max(p[2], z_grasp + APPROACH_MM)
        return self.start('내려놓기', [
            self._move(x, y, z_grasp + 2.0, DESCEND_VEL, '놓는 높이로 하강'),
            self._grip(False),
            self._move(x, y, top, DESCEND_VEL, '위로 빠지기'),
        ])
