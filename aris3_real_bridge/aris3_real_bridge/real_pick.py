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
  위로(접근 높이) -> 그리퍼 열기 -> 집는 높이로 하강 -> 닫기 -> 접근 높이로 들기.
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


class PixelCalibration:
    """영상 픽셀 (u, v) -> 로봇 베이스 (x, y) mm, 집는 높이 z mm."""

    def __init__(self):
        """빈 보정: 점 없음, 변환 행렬 없음 (ready() 가 False)."""
        self.points = []        # dict(u, v, x, y, z, label)
        self.H = None
        self.grasp_z = None
        self.errors_mm = []     # 점마다 오차 (5 점 이상이면 그 점을 빼고 맞춘 뒤 잰 값)

    # ---- 점 관리
    def add(self, u, v, x, y, z, label=''):
        self.points.append(dict(u=float(u), v=float(v), x=float(x), y=float(y), z=float(z), label=label))
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

    def summary(self):
        """화면·로그용 한 줄 요약: 점 개수, 집는 높이, 오차 평균/최대."""
        n = len(self.points)
        if not self.ready():
            return f'보정점 {n}개 (4개 이상 필요)'
        e = [v for v in self.errors_mm if not math.isnan(v)]
        how = '한 점씩 빼고 검증' if n >= 5 else '맞춘 점 자체 오차 - 5개 이상이면 검증 가능'
        return (f'보정점 {n}개 · 집는 높이 Z {self.grasp_z:.1f} mm · 오차 평균 {np.mean(e):.1f} / '
                f'최대 {np.max(e):.1f} mm ({how})')

    # ---- 파일
    def save(self, path=CALIB_PATH):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump({'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'points': self.points,
                       'note': '픽셀(u,v) -> 로봇 베이스 TCP (x,y,z) mm. aris3_gui real_pick.py'}, f,
                      ensure_ascii=False, indent=1)
        return path

    def load(self, path=CALIB_PATH):
        """저장한 보정 파일을 읽어 다시 맞춘다. 파일이 없으면 OSError."""
        with open(path) as f:
            self.points = json.load(f)['points']
        return self.fit()


class PickSequence:
    """접근 -> 열기 -> 하강 -> 닫기 -> 들기 를 작업 스레드에서 한 단계씩 실행한다.

    link: e0509_manual.robot_link.RobotLink (move_tcp_to, wait_motion, gripper_command, gripper)
    log: 함수(str)
    """

    def __init__(self, link, log):
        """link: RobotLink (또는 시험용 가짜), log: 문자열을 받는 로그 함수."""
        self.link, self.log = link, log
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
        return (label, lambda: self.link.gripper_command('move', pos=GRIP_CLOSE_POS if close else 0,
                                                         cur=GRIP_CURRENT_MA, vel=1500, label=label))

    def _check_hold(self, require=False):
        """require=True 이면 빈손이거나 모를 때 이후 단계(옮겨 놓기)를 취소한다."""
        def check():
            """닫은 뒤 그리퍼 위치·토크로 잡았는지 판단해 holding 에 저장 (require 면 빈손일 때 실패)."""
            d = self.link.gripper
            if not d:
                self.holding = None
                self.log('잡았는지 모름: 그리퍼 위치를 읽지 못함')
            elif not d.get('torque'):
                self.holding = False
                self.log(f'그리퍼 위치 {d["position"]}, 토크 꺼짐 -> 닫히지 않음 (빈손으로 판단)')
            elif d['position'] < GRIP_HOLD_MIN:
                self.holding = False
                self.log(f'그리퍼 위치 {d["position"]} -> 닫히지 않음 (빈손으로 판단)')
            else:
                self.holding = d['position'] < GRIP_HOLD_MAX
                self.log(f'그리퍼 위치 {d["position"]} -> ' +
                         ('물체에 막혀 멈춤 (잡음)' if self.holding else '끝까지 닫힘 (빈손)'))
            self.link.last_ok = True        # wait_motion 이 바로 통과하게 (움직임 없는 단계)
            return self.holding is True if require else True
        return ('잡았는지 확인', check)

    def _place_steps(self, x, y, z_grasp):
        """지금 자리에서 위로 -> (x, y) 위 접근 높이로 -> 놓는 높이로 하강 -> 열기 -> 위로."""
        top = z_grasp + APPROACH_MM

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
            self._move(x, y, z_grasp + 2.0, DESCEND_VEL, '놓는 높이로 하강'),
            self._grip(False),
            self._move(x, y, top, DESCEND_VEL, '위로 빠지기'),
        ]

    def hover(self, x, y, z_grasp):
        """고른 큐브 위 접근 높이로만 간다 (집기 전에 위치가 맞는지 눈으로 확인용)."""
        return self.start('큐브 위로', [self._travel(x, y, z_grasp + APPROACH_MM, TRAVEL_VEL, '큐브 위 접근 높이로')])

    def pick(self, x, y, z_grasp):
        """큐브 집기: 접근 높이로 이동 -> 열기 -> 하강 -> 닫기 -> 잡았는지 확인 -> 들기."""
        top = z_grasp + APPROACH_MM
        return self.start('집기', [
            self._travel(x, y, top, TRAVEL_VEL, '큐브 위 접근 높이로'),
            self._grip(False),
            self._move(x, y, z_grasp, DESCEND_VEL, '집는 높이로 하강'),
            self._grip(True),
            self._check_hold(),
            self._move(x, y, top, DESCEND_VEL, '들어 올리기'),
        ])

    def place_at(self, x, y, z_grasp):
        """들고 있는 물체를 (x, y) 에 놓는다 (카메라 보정으로 바꾼 놓을 자리)."""
        return self.start('그 자리에 놓기', self._place_steps(x, y, z_grasp))

    def pick_and_place(self, x, y, z_grasp, px, py):
        """(x, y) 의 큐브를 집어 (px, py) 에 놓는다. 잡지 못했으면 옮기지 않는다."""
        top = z_grasp + APPROACH_MM
        return self.start('집어서 옮기기', [
            self._travel(x, y, top, TRAVEL_VEL, '큐브 위 접근 높이로'),
            self._grip(False),
            self._move(x, y, z_grasp, DESCEND_VEL, '집는 높이로 하강'),
            self._grip(True),
            self._check_hold(require=True),
            self._move(x, y, top, DESCEND_VEL, '들어 올리기'),
        ] + self._place_steps(px, py, z_grasp)[1:])

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
