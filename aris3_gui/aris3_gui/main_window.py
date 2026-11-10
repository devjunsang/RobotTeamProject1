"""ARIS3 Control Station - 통합 관제 화면 (마스터플랜 07장 목업 배치).

  ┌ 상단: Connect · Disconnect · Home · Start Task · Stop · 정지(소프트웨어) ───────────────┐
  │ 시스템 상태   │ 카메라 Top View (검출 표시)        │ 작업 공간 2D (RViz 내장은 도전)   │
  │ 로봇 상태     │                                     │ 작업 큐 / 단계 진행               │
  │ 그리퍼 상태   ├─────────────────────────────────────┼──────────────┬────────────────────┤
  │ 카메라 정보   │ 로그                                │ 음성 명령    │ 수동 조작          │
  └───────────────┴─────────────────────────────────────┴──────────────┴────────────────────┘

안전 규칙 (e0509_manual/gui.py 와 같음):
  - 수동 조작·그리퍼 버튼은 '조작 허용' 을 체크해야 동작하고, 2분 동안 명령이 없으면 다시 잠긴다.
  - 정지는 언제나 동작한다 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
  - 작업 실행 중에는 수동 조작을 받지 않는다.
  - 음성 명령은 화면에 해석 결과를 보여 주고, '자동 실행' 을 켠 경우가 아니면 사용자가 '실행' 을 눌러야 한다.
"""
import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
                             QHeaderView, QLabel, QLineEdit, QListWidget, QMessageBox,
                             QPlainTextEdit, QProgressBar, QPushButton, QShortcut, QSlider,
                             QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from aris3_gui.real_pick import PickSequence, PixelCalibration
from aris3_gui.ros_io import LOG_NODES, STAGES
from aris3_gui.widgets import COLOR_KO, CameraView, WorkspaceView

ARM_TIMEOUT_SEC = 120
HOME_DEG_REAL = [0, 0, 90, 0, 90, 0]   # 표시용 (실제 값은 e0509_manual.robot_link.HOME_DEG)
# 손짓 (실물, MediaPipe): 가리키고 이 시간 동안 이 반경 안에 멈추면 선택
GESTURE_HOLD_SEC = 1.0
GESTURE_HOLD_RADIUS_PX = 25
CUBE_SELECT_PX = 40          # 클릭: 이 거리 안의 검출 큐브를 고름
CUBE_SELECT_GESTURE_PX = 60  # 손짓: 손끝이 큐브를 가리지 않게 조금 옆을 가리켜도 고름
GESTURE_KO = {'point': '☝ 가리킴', 'fist': '✊ 주먹', 'open': '🖐 펼침'}
STAGE_KO = {'VALIDATE': '요청·장비 확인', 'OBSERVE': '관찰 자세', 'PERCEIVE': '물체 인식',
            'PLAN': '경로 계획', 'PICK': '집기', 'PLACE': '놓기', 'VERIFY': '결과 확인'}
ZONES = ['A', 'B', 'C']
TASK_COLORS = ['red', 'blue', 'yellow']
OK_COLOR, BAD_COLOR, WAIT_COLOR = '#2e7d32', '#c62828', '#ef6c00'
# 'ARIS3 로그만' 에서 숨길, 매 계획마다 나오는 무해한 MoveIt 경고
NOISE = ('planning volume was not specified', "did not fill out the 'planner_id'",
         'No 3D sensor plugin', 'Resolution not specified for Octomap')


def nofocus(w):
    w.setFocusPolicy(Qt.NoFocus)     # Space 가 버튼을 '누르지' 않게 (Space = 정지)
    return w


def dot(ok, text, warn=False):
    color = OK_COLOR if ok else (WAIT_COLOR if warn else BAD_COLOR)
    return f'<span style="color:{color}">●</span> {text}'


class ControlStation(QWidget):
    def __init__(self, node, backend):
        super().__init__()
        self.node = node
        self.backend = backend
        self.queue = []                 # 남은 작업 [dict(color, zone, dry, target_id, place_xy)]
        self.sel_id = ''                # 카메라 화면 클릭(이후 손짓)으로 고른 물체
        self.sel_place = None           # 클릭으로 고른 놓을 자리 (world x, y)
        self.queue_running = False
        self.current_task = None
        self.last_cmd_time = time.time()
        self.log_shown = 0
        self.last_voice_stamp = None
        self._dialog_open = False       # 확인창이 떠 있는 동안 손짓·음성이 새 확인창을 띄우지 않게
        self.place_px = None            # 실물: 고른 놓을 자리 (영상 픽셀) - 클릭 또는 손짓
        self.g_hold = None              # 손짓: (u, v, 시작 시각) 가리키고 멈춘 곳
        self.g_fired = False            # 이번에 멈춘 곳에서 이미 선택했음
        self.g_last = ''                # 마지막으로 인정된 손짓 (point -> fist/open 전환을 본다)
        self.node.on_task_done = self._task_done_from_ros
        self._pending_next = False

        self.setWindowTitle(f'ARIS3 Control Station — {backend.title}')
        self.setFont(QFont('NanumSquare', 10))
        self.setStyleSheet(
            'QGroupBox { font-weight: bold; border: 1px solid #cfd8dc; border-radius: 6px; margin-top: 10px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }'
            'QPushButton { padding: 4px 10px; }')

        root = QVBoxLayout(self)
        root.addLayout(self._header())
        body = QHBoxLayout()
        body.addLayout(self._left_column(), 0)
        right = QVBoxLayout()
        top = QHBoxLayout()
        top.addWidget(self._camera_panel(), 3)
        # 카메라 크게 보기(더블클릭)에서 숨길 수 있게 옆·아래 패널을 위젯으로 묶는다
        self.side_box = QWidget()
        side = QVBoxLayout(self.side_box)
        side.setContentsMargins(0, 0, 0, 0)
        side.addWidget(self._workspace_panel(), 2)
        side.addWidget(self._queue_panel(), 3)
        top.addWidget(self.side_box, 2)
        right.addLayout(top, 3)
        self.bottom_box = QWidget()
        bottom = QHBoxLayout(self.bottom_box)
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.addWidget(self._log_panel(), 3)
        voice = self._voice_panel()
        bottom.addWidget(voice, 2)
        if backend.mode == 'real':                  # 실물: 음성 + 카메라 보정·집기 패널
            # 실물 음성은 「실물 집기」 순서로 이어지고 매번 확인창을 띄운다 -> 구역·자동 실행은 쓰지 않음
            self.cmb_voice_zone.setVisible(False)
            self.chk_voice_auto.setVisible(False)
            self.chk_voice_auto.setEnabled(False)
            bottom.addWidget(self._real_pick_panel(), 3)
        bottom.addWidget(self._jog_panel(), 2)
        right.addWidget(self.bottom_box, 2)
        body.addLayout(right, 1)
        root.addLayout(body, 1)

        for key in (Qt.Key_Space, Qt.Key_Escape):
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(self._stop_key if key == Qt.Key_Space else self._stop)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        # 카메라는 따로 빠르게 다시 그린다 (상태 패널 갱신 100 ms 에 묶이면 최대 10 Hz)
        self.cam_timer = QTimer(self)
        self.cam_timer.timeout.connect(self._refresh_camera)
        self.cam_timer.start(33)

    # ================================================================== 상단
    def _header(self):
        row = QHBoxLayout()
        title = QLabel('<b style="font-size:18pt">E0509 Control Station</b><br>'
                       '<span style="color:#607d8b">Doosan E0509 + RH-P12-RN-A &nbsp;|&nbsp; '
                       f'ROS 2 Jazzy + Gazebo + MoveIt 2 &nbsp;|&nbsp; 모드: {self.backend.title}</span>')
        row.addWidget(title, 1)

        def btn(text, slot, style=''):
            b = nofocus(QPushButton(text))
            b.setMinimumHeight(40)
            b.setMinimumWidth(110)
            b.clicked.connect(slot)
            if style:
                b.setStyleSheet(style)
            row.addWidget(b)
            return b
        self.btn_connect = btn('🔗 Connect', self._connect)
        self.btn_disconnect = btn('⛓ Disconnect', self._disconnect)
        self.btn_home = btn('⌂ Home', self._home)
        self.btn_start = btn('▶ Start Task', self._start_queue)
        self.btn_stop = btn('■ Stop', self._stop)
        self.btn_estop = btn('⚠ 정지 (Space / Esc)', self._stop,
                             'QPushButton { background:#d32f2f; color:white; font-weight:bold; border-radius:6px; }'
                             'QPushButton:pressed { background:#9a0007; }')
        self.btn_estop.setToolTip('소프트웨어 정지: 작업 취소 + 이동 중단. 인증된 비상정지 장치를 대신하지 않습니다.')
        return row

    # ================================================================== 왼쪽
    def _left_column(self):
        col = QVBoxLayout()
        sys_box = QGroupBox('시스템 상태 (System Status)')
        g = QGridLayout(sys_box)
        self.sys_labels = {}
        for i, (key, name) in enumerate([('conn', '연결'), ('gazebo', '로봇/시뮬레이터'), ('camera', '카메라'),
                                         ('detect', '물체 인식'), ('gripper', '그리퍼'), ('moveit', 'MoveIt 2'),
                                         ('tm', 'Task Manager'), ('voice', '음성 인식'),
                                         ('gesture', '손짓 인식')]):
            g.addWidget(QLabel(name), i, 0)
            lbl = QLabel('—')
            lbl.setTextFormat(Qt.RichText)
            g.addWidget(lbl, i, 1)
            self.sys_labels[key] = lbl
        col.addWidget(sys_box)

        rob = QGroupBox('로봇 상태 (Robot State)')
        g = QGridLayout(rob)
        self.lbl_mode = QLabel(f'<b>{"MoveIt 2 (시뮬레이션)" if self.backend.mode == "sim" else "두산 (실물)"}</b>')
        self.lbl_task_state = QLabel('—')
        g.addWidget(QLabel('모드'), 0, 0)
        g.addWidget(self.lbl_mode, 0, 1, 1, 3)
        g.addWidget(QLabel('작업 상태'), 1, 0)
        g.addWidget(self.lbl_task_state, 1, 1, 1, 3)
        mono = QFont('DejaVu Sans Mono', 9)
        self.lbl_joints = QLabel('—')
        self.lbl_joints.setFont(mono)
        self.lbl_tcp = QLabel('—')
        self.lbl_tcp.setFont(mono)
        g.addWidget(QLabel('관절 [deg]'), 2, 0, 1, 4)
        g.addWidget(self.lbl_joints, 3, 0, 1, 4)
        g.addWidget(QLabel('손끝 TCP [m, deg] (world)'), 4, 0, 1, 4)
        g.addWidget(self.lbl_tcp, 5, 0, 1, 4)
        col.addWidget(rob)

        grip = QGroupBox('그리퍼 상태 (Gripper State)')
        g = QGridLayout(grip)
        self.lbl_grip = QLabel('—')
        self.bar_grip = QProgressBar()
        self.bar_grip.setRange(0, 100)
        self.bar_grip.setFormat('닫힘 %p%')
        self.btn_gopen = nofocus(QPushButton('열기'))
        self.btn_gclose = nofocus(QPushButton('닫기 (잡기)'))
        self.btn_gopen.clicked.connect(lambda: self._gripper(False))
        self.btn_gclose.clicked.connect(lambda: self._gripper(True))
        g.addWidget(self.lbl_grip, 0, 0, 1, 2)
        g.addWidget(self.bar_grip, 1, 0, 1, 2)
        g.addWidget(self.btn_gopen, 2, 0)
        g.addWidget(self.btn_gclose, 2, 1)
        col.addWidget(grip)

        cal = QGroupBox('카메라 정보 (Camera)')
        g = QGridLayout(cal)
        self.lbl_cal = QLabel('—')
        self.lbl_cal.setWordWrap(True)
        g.addWidget(self.lbl_cal, 0, 0)
        col.addWidget(cal)
        col.addStretch(1)
        w = QWidget()
        w.setLayout(col)
        w.setFixedWidth(330)
        outer = QVBoxLayout()
        outer.addWidget(w)
        return outer

    # ================================================================== 가운데
    def _camera_panel(self):
        box = QGroupBox('카메라 뷰 (Camera View - Top View)  ·  더블클릭: 크게 보기')
        v = QVBoxLayout(box)
        top = QHBoxLayout()
        self.chk_detect = nofocus(QCheckBox('검출 표시 (Detection)'))
        self.chk_detect.setChecked(True)
        top.addStretch(1)
        top.addWidget(self.chk_detect)
        v.addLayout(top)
        self.camera = CameraView()
        self.camera.on_click = self._on_camera_click
        self.camera.on_double_click = self._toggle_camera_large
        v.addWidget(self.camera, 1)
        self.lbl_cam_info = QLabel('—')
        self.lbl_cam_info.setStyleSheet('color:#607d8b')
        v.addWidget(self.lbl_cam_info)
        return box

    def _real_pick_panel(self):
        """실물: 카메라 픽셀 -> 로봇 좌표 보정(손으로 가르쳐 주기)과 큐브 집기. real_pick.py 참고."""
        self.calib = PixelCalibration()
        self.picker = PickSequence(self.backend.link, lambda s: self.node.log('INFO', f'[집기] {s}'))
        self.pick_px = None                         # 고른 큐브의 영상 픽셀 (u, v)
        self.pick_name = ''
        self.calib_px = None                        # 보정 ① 에서 기억한 픽셀
        try:
            self.calib.load()
            self.node.log('INFO', f'카메라 보정 불러옴: {self.calib.summary()}')
        except (OSError, ValueError, KeyError):
            pass
        box = QGroupBox('실물 집기 · 카메라 보정 (Real Pick)')
        v = QVBoxLayout(box)
        self.lbl_pick = QLabel('카메라 화면에서 큐브를 클릭하세요')
        self.lbl_pick.setWordWrap(True)
        v.addWidget(self.lbl_pick)
        g = QGridLayout()
        b1 = nofocus(QPushButton('① 큐브 픽셀 기억'))
        b1.setToolTip('큐브가 카메라에 가려지지 않았을 때 누르세요 (고른 큐브의 지금 위치를 기억)')
        b1.clicked.connect(self._calib_remember)
        b2 = nofocus(QPushButton('② 로봇 위치 기록'))
        b2.setToolTip('조그로 그리퍼를 그 큐브를 집는 자세(손가락 사이에 큐브)까지 내린 뒤 누르세요')
        b2.clicked.connect(self._calib_record)
        b3 = nofocus(QPushButton('마지막 점 삭제'))
        b3.clicked.connect(lambda: (self.calib.remove_last(), self._calib_changed()))
        b4 = nofocus(QPushButton('보정 저장'))
        b4.clicked.connect(self._calib_save)
        g.addWidget(b1, 0, 0)
        g.addWidget(b2, 0, 1)
        g.addWidget(b3, 1, 0)
        g.addWidget(b4, 1, 1)
        v.addLayout(g)
        self.lbl_calib = QLabel(self.calib.summary())
        self.lbl_calib.setWordWrap(True)
        self.lbl_calib.setStyleSheet('color:#455a64')
        v.addWidget(self.lbl_calib)
        row = QHBoxLayout()
        for text, slot, tip in [('큐브 위로', self._pick_hover, '고른 큐브 위 접근 높이(+100 mm)로 가서 멈춤'),
                                ('집기', self._pick_run, '접근 -> 열기 -> 하강 -> 닫기 -> 들기'),
                                ('고른 자리에 놓기', self._place_selected,
                                 '카메라 화면에서 고른 빈 자리(청록 원) 위로 가서 내려놓고 위로'),
                                ('여기 내려놓기', self._pick_place, '지금 위치 아래로 내려놓고 위로')]:
            b = nofocus(QPushButton(text))
            b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
        v.addLayout(row)
        grow = QHBoxLayout()
        self.chk_gesture = nofocus(QCheckBox('손짓 사용 (MediaPipe)'))
        self.chk_gesture.setChecked(True)
        self.chk_gesture.setToolTip('☝ 가리키고 1초 멈춤 = 큐브/빈 자리 선택 · ☝→✊ 주먹 = 집기 · ☝→🖐 펼침 = 고른 자리에 놓기'
                                    '\n(실행 전 항상 확인창)')
        self.lbl_gesture = QLabel('손짓: —')
        grow.addWidget(self.chk_gesture)
        grow.addWidget(self.lbl_gesture, 1)
        v.addLayout(grow)
        note = QLabel('☝ 1초 멈춤 = 선택 · ☝→✊ = 집기 · ☝→🖐 = 고른 자리에 놓기 · 실행 전 확인창\n'
                      '움직이는 버튼은 「조작 허용」 체크 필요 · 정지: Space')
        note.setStyleSheet('color:#90a4ae')
        v.addWidget(note)
        v.addStretch(1)
        self.camera.on_pixel_click = self._on_pixel_click
        return box

    def _on_pixel_click(self, u, v):
        self._select_pixel(u, v, CUBE_SELECT_PX, '클릭')

    def _select_pixel(self, u, v, radius, how):
        """실물(보정 전/후 공통): 그 픽셀 가까이 색 검출 큐브가 있으면 그 큐브를, 없으면 그 자리를 놓을 곳으로 고른다."""
        px = [o for o in self.node.px_objects] if self.node.px_rate.fresh(2.0) else []
        near = min(px, key=lambda o: math.hypot(o.pose.pose.position.x - u, o.pose.pose.position.y - v),
                   default=None)
        if near is not None and math.hypot(near.pose.pose.position.x - u, near.pose.pose.position.y - v) <= radius:
            self.pick_px = (near.pose.pose.position.x, near.pose.pose.position.y)
            self.pick_name = near.id or near.color
            self.sel_id = near.id
            self.node.log('INFO', f'{how}: 큐브 {self.pick_name} 고름 (픽셀 {self.pick_px[0]:.0f}, {self.pick_px[1]:.0f})')
        else:
            self.place_px = (float(u), float(v))
            xy = self.calib.to_robot(u, v)
            self.node.log('INFO', f'{how}: 놓을 자리 고름 (픽셀 {u:.0f}, {v:.0f}' +
                          (')' if xy is None else f' -> 로봇 X {xy[0]:.0f}, Y {xy[1]:.0f} mm)'))
        self._update_pick_label()

    def _update_pick_label(self):
        lines = []
        if self.pick_px is None:
            lines.append('고른 큐브: — (카메라 화면에서 큐브를 클릭하거나 손으로 가리키세요)')
        else:
            u, v = self.pick_px
            xy = self.calib.to_robot(u, v)
            where = '로봇 좌표: 보정 전' if xy is None else f'로봇 X {xy[0]:.1f}  Y {xy[1]:.1f} mm'
            lines.append(f'고른 큐브: {self.pick_name}  픽셀 ({u:.0f}, {v:.0f})  →  {where}')
        if self.place_px is not None:
            u, v = self.place_px
            xy = self.calib.to_robot(u, v)
            where = '로봇 좌표: 보정 전' if xy is None else f'로봇 X {xy[0]:.1f}  Y {xy[1]:.1f} mm'
            lines.append(f'놓을 자리: 픽셀 ({u:.0f}, {v:.0f})  →  {where}')
        self.lbl_pick.setText('\n'.join(lines))

    def _place_target(self, what):
        """고른 놓을 자리의 로봇 (x, y) 또는 None."""
        if not self.calib.ready():
            self.node.log('WARN', f'{what}: 카메라 보정이 없음 (점 4개 이상 기록)')
            return None
        if self.place_px is None:
            self.node.log('WARN', f'{what}: 놓을 자리를 먼저 고르세요 (빈 곳 클릭 또는 손으로 가리키고 1초)')
            return None
        return self.calib.to_robot(*self.place_px)

    def _place_selected(self, *_, heard=''):
        if not self._arm_ok('고른 자리에 놓기'):
            return
        if self.picker.running:
            self.node.log('WARN', '고른 자리에 놓기: 이전 순서가 진행 중')
            return
        xy = self._place_target('고른 자리에 놓기')
        if xy is None:
            return
        said = f'{heard}\n\n' if heard else ''
        hold = {True: '', False: '\n※ 마지막 집기에서 빈손으로 판단됨', None: ''}[self.picker.holding]
        if self._ask('고른 자리에 놓기', f'{said}로봇 X {xy[0]:.0f}, Y {xy[1]:.0f} mm 위로 가서 내려놓습니다.{hold}'
                                       '\n\n로봇 주변(손 포함)이 비었습니까?'):
            self.picker.place_at(xy[0], xy[1], self.calib.grasp_z)
            self.place_px = None
            self._update_pick_label()

    # ---- 손짓 (MediaPipe, /aris3/gesture)
    def _gesture_tick(self, g):
        """100 ms 마다: 포인터 표시, 가리키고 멈춤 -> 선택, ☝ 에서 ✊/🖐 로 바뀌는 순간 -> 확정."""
        cam = self.camera
        if not self.chk_gesture.isChecked() or g is None or not g.hand_present:
            cam.pointer, cam.pointer_progress, cam.pointer_text = None, 0.0, ''
            self.g_hold, self.g_last = None, ''
            self.lbl_gesture.setText('손짓: 손 없음' if self.chk_gesture.isChecked() else '손짓: 꺼짐')
            return
        u, v, now, gest = g.tip_u, g.tip_v, time.time(), g.gesture
        cam.pointer = (u, v)
        cam.pointer_text = GESTURE_KO.get(gest, '…')
        self.lbl_gesture.setText(f'손짓: {GESTURE_KO.get(gest, "모름")}  (손끝 {u:.0f}, {v:.0f})')
        if gest == 'point':
            if self.g_hold is None or math.hypot(u - self.g_hold[0], v - self.g_hold[1]) > GESTURE_HOLD_RADIUS_PX:
                self.g_hold, self.g_fired = (u, v, now), False
            prog = min(1.0, (now - self.g_hold[2]) / GESTURE_HOLD_SEC)
            cam.pointer_progress = 0.0 if self.g_fired else prog
            if prog >= 1.0 and not self.g_fired and not self._dialog_open:
                self.g_fired = True
                self._select_pixel(self.g_hold[0], self.g_hold[1], CUBE_SELECT_GESTURE_PX, '손짓')
        else:
            self.g_hold, cam.pointer_progress = None, 0.0
        # 확정: 가리키다가(☝) 주먹/펼침으로 바뀐 순간 한 번만. 손이 처음부터 주먹이면 아무것도 안 함
        if gest in ('fist', 'open') and self.g_last == 'point' and not self._dialog_open and not self.picker.running:
            QTimer.singleShot(0, lambda gg=gest: self._gesture_confirm(gg))
        if gest:
            self.g_last = gest

    def _gesture_confirm(self, gest):
        if gest == 'fist':
            if self.pick_px is None:
                self.node.log('WARN', '손짓 ✊: 집을 큐브를 먼저 가리키세요 (☝ 1초)')
                return
            self._pick_run(heard='손짓: ☝ → ✊ 주먹 (집기)')
        else:
            self._place_selected(heard='손짓: ☝ → 🖐 펼침 (고른 자리에 놓기)')

    def _calib_remember(self):
        if self.pick_px is None:
            self.node.log('WARN', '보정 ①: 먼저 카메라 화면에서 큐브를 클릭하세요')
            return
        # 클릭한 뒤 큐브가 움직였을 수 있으니 같은 이름(없으면 가장 가까운) 검출의 지금 위치로 갱신
        px = list(self.node.px_objects) if self.node.px_rate.fresh(2.0) else []
        near = min(px, key=lambda o: math.hypot(o.pose.pose.position.x - self.pick_px[0],
                                                o.pose.pose.position.y - self.pick_px[1]), default=None)
        if near is None:
            self.node.log('WARN', '보정 ①: 지금 카메라에 큐브가 보이지 않음 (가려지지 않았는지 확인)')
            return
        self.calib_px = (near.pose.pose.position.x, near.pose.pose.position.y)
        self.camera.marker_px = self.calib_px
        self.node.log('INFO', f'보정 ①: {near.id or near.color} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f}) '
                      '기억 → 조그로 그리퍼를 그 큐브를 집는 자세까지 내린 뒤 ②')

    def _calib_record(self):
        if self.calib_px is None:
            self.node.log('WARN', '보정 ②: 먼저 ① 로 큐브 픽셀을 기억하세요')
            return
        p = self.backend.link.posx
        if p is None or not self.backend.connected():
            self.node.log('WARN', '보정 ②: 로봇 TCP 좌표를 읽지 못함 (접속 확인)')
            return
        self.calib.add(self.calib_px[0], self.calib_px[1], p[0], p[1], p[2], label=self.pick_name)
        self.node.log('INFO', f'보정 ②: 점 {len(self.calib.points)} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f})'
                      f' -> TCP ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm')
        self.calib_px = None
        self.camera.marker_px = None
        self._calib_changed()

    def _calib_changed(self):
        self.lbl_calib.setText(self.calib.summary())
        if self.calib.ready():
            pts = '  '.join(f'{i + 1}:{e:.1f}' for i, e in enumerate(self.calib.errors_mm))
            self.node.log('INFO', f'보정: {self.calib.summary()}  점별 오차 mm {pts}')
        self._update_pick_label()

    def _calib_save(self):
        if not self.calib.points:
            self.node.log('WARN', '보정 저장: 기록한 점이 없음')
            return
        self.node.log('INFO', f'보정 저장: {self.calib.save()}')

    def _pick_target(self, what):
        """고른 큐브의 지금 로봇 좌표 (x, y, z_grasp) 또는 None. 움직이기 직전에 픽셀을 다시 읽는다."""
        if not self.calib.ready():
            self.node.log('WARN', f'{what}: 카메라 보정이 없음 (점 4개 이상 기록)')
            return None
        if self.pick_px is None:
            self.node.log('WARN', f'{what}: 카메라 화면에서 큐브를 클릭하세요')
            return None
        if self.picker.running:
            self.node.log('WARN', f'{what}: 이전 순서가 진행 중')
            return None
        px = list(self.node.px_objects) if self.node.px_rate.fresh(1.0) else []
        near = min(px, key=lambda o: math.hypot(o.pose.pose.position.x - self.pick_px[0],
                                                o.pose.pose.position.y - self.pick_px[1]), default=None)
        if near is not None and math.hypot(near.pose.pose.position.x - self.pick_px[0],
                                           near.pose.pose.position.y - self.pick_px[1]) < 40:
            self.pick_px = (near.pose.pose.position.x, near.pose.pose.position.y)
        else:
            self.node.log('WARN', f'{what}: 고른 큐브가 지금 보이지 않음 - 마지막 위치 사용')
        x, y = self.calib.to_robot(*self.pick_px)
        self._update_pick_label()
        return x, y, self.calib.grasp_z

    def _pick_hover(self):
        if self._arm_ok('큐브 위로'):
            t = self._pick_target('큐브 위로')
            if t:
                self.picker.hover(*t)

    def _pick_run(self, *_, heard=''):
        if self._arm_ok('집기'):
            t = self._pick_target('집기')
            said = f'{heard}\n\n' if heard else ''
            if t and self._ask('집기', f'{said}{self.pick_name} 을(를) 집습니다.\n로봇 좌표 X {t[0]:.0f}, Y {t[1]:.0f} mm, '
                               f'집는 높이 Z {t[2]:.0f} mm\n\n먼저 「큐브 위로」 로 위치를 확인했습니까?'):
                self.picker.pick(*t)

    def _pick_place(self, *_, heard=''):
        if not self._arm_ok('내려놓기'):
            return
        if not self.calib.ready():
            self.node.log('WARN', '내려놓기: 카메라 보정이 없음 (집는 높이를 모름)')
            return
        if heard and not self._ask('내려놓기', f'{heard}\n\n지금 위치 바로 아래로 내려놓고 위로 빠집니다. '
                                              '실행할까요?'):
            return
        self.picker.place_here(self.calib.grasp_z)

    def _real_voice(self, v):
        """실물: 음성 명령 -> 「실물 집기」 패널의 집기/내려놓기 (확인창·조작 허용·안전 검사는 버튼과 같음)."""
        if self.picker.running or self._dialog_open:
            self.node.log('WARN', f'음성 「{v.text}」 무시: 이전 순서 진행 중이거나 확인창이 떠 있음')
            return
        heard = f'음성: 「{v.text}」'
        if v.action == 'home':                      # 「집으로 돌아가」
            self._home_voice(heard)
            return
        if v.refers_selected_place and not v.color and self.picker.holding is True:
            self._place_selected(heard=heard)          # 들고 있을 때 「저기 놔줘」 = 고른 자리에 놓기
            return
        if v.action == 'place':
            self._pick_place(heard=heard)
            return
        if v.color:                                 # 「빨간 큐브 집어줘」: 그 색 검출이 하나일 때만
            px = list(self.node.px_objects) if self.node.px_rate.fresh(1.0) else []
            same = [o for o in px if o.color == v.color]
            name = COLOR_KO.get(v.color, v.color)
            if not same:
                self.node.log('WARN', f'음성: 카메라에 {name} 큐브가 보이지 않음')
                return
            if len(same) > 1:
                self.node.log('WARN', f'음성: {name} 큐브가 {len(same)}개 - 화면에서 클릭한 뒤 「이거 집어줘」')
                return
            o = same[0]
            self.pick_px = (o.pose.pose.position.x, o.pose.pose.position.y)
            self.pick_name = o.id or o.color
            self.sel_id = o.id
            self._update_pick_label()
        elif self.pick_px is None:                  # 「이거 집어줘」
            self.node.log('WARN', '「이거」 가 무엇인지 모름 - 먼저 카메라 화면에서 큐브를 클릭하거나 가리키세요')
            return
        if v.refers_selected_place:                 # 「이거 저기로 옮겨줘」: 집어서 고른 자리에 놓기
            self._pick_and_place(heard=heard)
        else:
            self._pick_run(heard=heard)

    def _pick_and_place(self, heard=''):
        if not self._arm_ok('집어서 옮기기'):
            return
        xy = self._place_target('집어서 옮기기')
        if xy is None:
            return
        t = self._pick_target('집어서 옮기기')
        if t and self._ask('집어서 옮기기', f'{heard}\n\n{self.pick_name} 을(를) 로봇 X {t[0]:.0f}, Y {t[1]:.0f} mm 에서 '
                                       f'집어\nX {xy[0]:.0f}, Y {xy[1]:.0f} mm 에 놓습니다. (잡지 못하면 옮기지 않음)'
                                       '\n\n로봇 주변(손 포함)이 비었습니까?'):
            self.picker.pick_and_place(t[0], t[1], t[2], xy[0], xy[1])
            self.place_px = None
            self._update_pick_label()

    def _log_panel(self):
        box = QGroupBox('로그 (Logs / ROS Console)')
        v = QVBoxLayout(box)
        top = QHBoxLayout()
        self.chk_log_aris3 = nofocus(QCheckBox('ARIS3 로그만'))
        self.chk_log_aris3.setChecked(True)
        clear = nofocus(QPushButton('지우기'))
        top.addStretch(1)
        top.addWidget(self.chk_log_aris3)
        top.addWidget(clear)
        v.addLayout(top)
        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFocusPolicy(Qt.NoFocus)
        self.txt_log.setMaximumBlockCount(500)
        self.txt_log.setFont(QFont('DejaVu Sans Mono', 9))
        clear.clicked.connect(self.txt_log.clear)
        v.addWidget(self.txt_log)
        return box

    # ================================================================== 오른쪽 위
    def _workspace_panel(self):
        box = QGroupBox('작업 공간 (위에서 본 2D · RViz 3D 내장은 도전 과제)')
        v = QVBoxLayout(box)
        self.workspace = WorkspaceView()
        v.addWidget(self.workspace)
        return box

    def _queue_panel(self):
        box = QGroupBox('작업 큐 (Task Queue)')
        v = QVBoxLayout(box)
        add = QHBoxLayout()
        self.cmb_color = nofocus(QComboBox())
        for c in TASK_COLORS:
            self.cmb_color.addItem(f'{COLOR_KO[c]} 큐브', c)
        self.cmb_zone = nofocus(QComboBox())
        for z in ZONES:
            self.cmb_zone.addItem(f'구역 {z}', z)
        self.chk_dry = nofocus(QCheckBox('계획만 (dry run)'))
        b_add = nofocus(QPushButton('+ 추가'))
        b_del = nofocus(QPushButton('삭제'))
        b_add.clicked.connect(self._queue_add)
        b_del.clicked.connect(self._queue_del)
        for w in (self.cmb_color, QLabel('→'), self.cmb_zone, self.chk_dry, b_add, b_del):
            add.addWidget(w)
        v.addLayout(add)
        # ---- 화면에서 고르기: 카메라 화면 클릭 (손짓 포인터도 같은 경로를 쓴다)
        pick = QHBoxLayout()
        self.lbl_sel = QLabel('화면 선택: 카메라에서 큐브 클릭(이거) · 빈 곳 클릭(저기)')
        self.lbl_sel.setStyleSheet('color:#00838f')
        self.btn_sel_run = nofocus(QPushButton('▶ 선택한 물체를 그 자리로'))
        self.btn_sel_run.clicked.connect(self._run_selection)
        b_clear = nofocus(QPushButton('선택 해제'))
        b_clear.clicked.connect(self._clear_selection)
        pick.addWidget(self.lbl_sel, 1)
        pick.addWidget(self.btn_sel_run)
        pick.addWidget(b_clear)
        v.addLayout(pick)
        self.lst_queue = QListWidget()
        self.lst_queue.setMaximumHeight(48)
        v.addWidget(self.lst_queue)
        self.tbl_stage = QTableWidget(len(STAGES), 3)
        self.tbl_stage.setHorizontalHeaderLabels(['단계', '상태', '시간'])
        self.tbl_stage.verticalHeader().setVisible(False)
        self.tbl_stage.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tbl_stage.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tbl_stage.setFocusPolicy(Qt.NoFocus)
        self.tbl_stage.verticalHeader().setDefaultSectionSize(21)
        self.tbl_stage.setMinimumHeight(21 * len(STAGES) + 26)
        for i, s in enumerate(STAGES):
            self.tbl_stage.setItem(i, 0, QTableWidgetItem(f'{s} ({STAGE_KO[s]})'))
            self.tbl_stage.setItem(i, 1, QTableWidgetItem('대기'))
            self.tbl_stage.setItem(i, 2, QTableWidgetItem(''))
        v.addWidget(self.tbl_stage, 1)
        self.bar_task = QProgressBar()
        self.bar_task.setRange(0, 100)
        self.lbl_task_msg = QLabel('')
        self.lbl_task_msg.setWordWrap(True)
        v.addWidget(self.bar_task)
        v.addWidget(self.lbl_task_msg)
        return box

    # ================================================================== 오른쪽 아래
    def _voice_panel(self):
        box = QGroupBox('음성 명령 (Voice Command)')
        v = QVBoxLayout(box)
        self.lbl_voice_state = QLabel('—')
        v.addWidget(self.lbl_voice_state)
        self.btn_talk = nofocus(QPushButton('🎤 누르고 말하기'))
        self.btn_talk.setMinimumHeight(44)
        self.btn_talk.pressed.connect(lambda: self._talk(True))
        self.btn_talk.released.connect(lambda: self._talk(False))
        v.addWidget(self.btn_talk)
        row = QHBoxLayout()
        self.txt_cmd = QLineEdit()
        self.txt_cmd.setPlaceholderText('또는 문장 입력 (예: 이거 저기로 옮겨줘 / 노란 큐브 집어줘)')
        self.txt_cmd.returnPressed.connect(self._send_text)
        b = nofocus(QPushButton('해석'))
        b.clicked.connect(self._send_text)
        row.addWidget(self.txt_cmd, 1)
        row.addWidget(b)
        v.addLayout(row)
        self.lbl_heard = QLabel('인식: —')
        self.lbl_heard.setWordWrap(True)
        self.lbl_heard.setStyleSheet('background:#e3f2fd; padding:6px; border-radius:4px;')
        self.lbl_parsed = QLabel('해석: —')
        self.lbl_parsed.setWordWrap(True)
        v.addWidget(self.lbl_heard)
        v.addWidget(self.lbl_parsed)
        row = QHBoxLayout()
        self.cmb_voice_zone = nofocus(QComboBox())
        for z in ZONES:
            self.cmb_voice_zone.addItem(f'놓을 곳: 구역 {z}', z)
        self.chk_voice_auto = nofocus(QCheckBox('자동 실행'))
        self.btn_voice_run = nofocus(QPushButton('실행'))
        self.btn_voice_run.clicked.connect(self._voice_run)
        row.addWidget(self.cmb_voice_zone)
        row.addWidget(self.chk_voice_auto)
        row.addWidget(self.btn_voice_run)
        v.addLayout(row)
        v.addStretch(1)
        return box

    def _jog_panel(self):
        box = QGroupBox('수동 조작 (Manual Jog)')
        v = QVBoxLayout(box)
        self.chk_arm = nofocus(QCheckBox('조작 허용 (주변이 비었을 때만)'))
        self.chk_arm.toggled.connect(lambda on: (setattr(self, 'last_cmd_time', time.time()),
                                                 self.node.log('INFO', '조작 허용 켬' if on else '조작 잠금')))
        arm_row = QHBoxLayout()
        arm_row.addWidget(self.chk_arm)
        # 실물: 수동 모드 = 팔 끝 콕핏의 직접 교시 버튼을 누른 채 손으로 팔을 끌기 (보정 점 찍기를 빠르게)
        self.chk_hand = nofocus(QCheckBox('손으로 움직이기 (수동 모드)'))
        self.chk_hand.setToolTip('켜면 로봇을 수동 모드로 바꿉니다. 팔 끝 콕핏의 직접 교시 버튼을 누른 채 팔을 끌어 '
                                 '움직일 수 있습니다.\n켜져 있는 동안 GUI 의 이동·그리퍼·집기 명령은 막힙니다. '
                                 '끄면 자동 모드로 돌아갑니다.')
        self.chk_hand.clicked.connect(self._toggle_hand_guide)
        self.chk_hand.setVisible(self.backend.mode == 'real')
        arm_row.addWidget(self.chk_hand)
        arm_row.addStretch(1)
        v.addLayout(arm_row)
        self.jog_buttons = []
        tabs = QTabWidget()
        tabs.setFocusPolicy(Qt.NoFocus)
        # TCP
        tcp = QWidget()
        g = QGridLayout(tcp)
        for (r, c, text, axis, sign) in [(0, 1, '+X', 0, 1), (2, 1, '−X', 0, -1), (1, 0, '+Y', 1, 1),
                                         (1, 2, '−Y', 1, -1), (0, 3, '+Z', 2, 1), (2, 3, '−Z', 2, -1)]:
            b = nofocus(QPushButton(text))
            b.setMinimumHeight(34)
            b.clicked.connect(lambda _=False, a=axis, s=sign: self._jog_tcp(a, s))
            g.addWidget(b, r, c)
            self.jog_buttons.append(b)
        bh = nofocus(QPushButton('⌂'))
        bh.clicked.connect(self._home)
        g.addWidget(bh, 1, 1)
        self.jog_buttons.append(bh)
        tabs.addTab(tcp, 'TCP 이동')
        # 관절
        jt = QWidget()
        g = QGridLayout(jt)
        for i in range(6):
            g.addWidget(QLabel(f'J{i + 1}'), i // 2, (i % 2) * 3)
            for k, sign in enumerate((-1, 1)):
                b = nofocus(QPushButton('−' if sign < 0 else '+'))
                b.setFixedWidth(36)
                b.clicked.connect(lambda _=False, i=i, s=sign: self._jog_joint(i, s))
                g.addWidget(b, i // 2, (i % 2) * 3 + 1 + k)
                self.jog_buttons.append(b)
        tabs.addTab(jt, 'Joint 이동')
        v.addWidget(tabs)
        row = QHBoxLayout()
        self.cmb_step = nofocus(QComboBox())
        for s in (1, 5, 10, 20):
            self.cmb_step.addItem(f'{s} mm / °', s)
        self.cmb_step.setCurrentIndex(2)
        self.sld_speed = QSlider(Qt.Horizontal)
        self.sld_speed.setRange(5, 100)
        self.sld_speed.setValue(30)
        self.sld_speed.setFocusPolicy(Qt.NoFocus)
        self.lbl_speed = QLabel('30 %')
        self.sld_speed.valueChanged.connect(lambda v_: self.lbl_speed.setText(f'{v_} %'))
        row.addWidget(QLabel('스텝'))
        row.addWidget(self.cmb_step)
        row.addWidget(QLabel('속도'))
        row.addWidget(self.sld_speed, 1)
        row.addWidget(self.lbl_speed)
        v.addLayout(row)
        if self.backend.mode == 'real':
            rrow = QHBoxLayout()
            self.btn_recover = nofocus(QPushButton('⟲ 복구 후 홈'))
            self.btn_recover.setMinimumHeight(34)
            self.btn_recover.setToolTip('한계 동작·충돌 감지로 멈췄을 때: 안전 정지 해제(또는 서보 켜기) -> 자동 모드 -> '
                                        f'홈 자세 {HOME_DEG_REAL}° 로 이동.\n'
                                        '비상정지·복구 모드는 로봇 쪽에서 직접 풀어야 합니다.')
            self.btn_recover.clicked.connect(self._recover_home)
            self.lbl_robot_state = QLabel('로봇 상태: —')
            rrow.addWidget(self.btn_recover)
            rrow.addWidget(self.lbl_robot_state, 1)
            v.addLayout(rrow)
        self.lbl_jog_reason = QLabel('')
        self.lbl_jog_reason.setStyleSheet(f'color:{WAIT_COLOR}')
        self.lbl_jog_reason.setWordWrap(True)
        v.addWidget(self.lbl_jog_reason)
        return box

    # ================================================================== 동작
    def _ask(self, title, text):
        self._dialog_open = True
        try:
            return QMessageBox.question(self, title, text, QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.No) == QMessageBox.Yes
        finally:
            self._dialog_open = False

    def _connect(self):
        if self.backend.mode == 'real':
            if not self._ask('로봇 접속', '실물 로봇에 접속합니다. 서보가 켜지고 펜던트의 제어 권한을 가져옵니다.\n'
                             '팔은 움직이지 않습니다.\n\n로봇 옆에 사람이 있고 비상정지를 누를 수 있습니까?'):
                return
        self.backend.connect()

    def _disconnect(self):
        msg = ('시뮬레이션(Gazebo·MoveIt·Task Manager)을 종료합니다.' if self.backend.mode == 'sim' else
               '접속을 끊으면 로봇 서보가 꺼집니다(브레이크가 팔을 잡습니다).')
        if self._ask('Disconnect', msg + '\n\n계속할까요?'):
            self.chk_arm.setChecked(False)
            self.queue.clear()
            self.queue_running = False
            self.backend.disconnect()

    def _home(self):
        if not self._arm_ok('홈 이동'):
            return
        self.backend.home(self.sld_speed.value())

    def _home_voice(self, heard):
        """실물: 음성 「집으로 돌아가」 -> 확인창 -> 홈 자세 (⌂ Home 버튼과 같은 이동·검사)."""
        if not self._arm_ok('홈 이동'):
            return
        hold = '\n※ 그리퍼가 큐브를 들고 있으면 함께 움직입니다.' if self.picker.holding is True else ''
        if self._ask('홈으로 돌아가기', f'{heard}\n\n홈 자세 {HOME_DEG_REAL}° 로 관절 이동합니다 '
                                     f'(속도 슬라이더 {self.sld_speed.value()} %, 최대 45°/s).{hold}\n\n'
                                     '팔이 크게 움직일 수 있습니다. 주변(손 포함)이 비었습니까?'):
            self.backend.home(self.sld_speed.value())

    def _stop_key(self):
        """Space: 어디서나 정지. 단, 문장 입력창에 글을 치는 중이면 띄어쓰기로 넣는다 (Esc·정지 버튼은 항상 정지)."""
        w = QApplication.focusWidget()
        if isinstance(w, QLineEdit) and not self.node.task_running and not (
                getattr(self, 'picker', None) is not None and self.picker.running):
            w.insert(' ')
            return
        self._stop()

    def _recover_home(self):
        if not self.chk_arm.isChecked():
            self.node.log('WARN', '복구 후 홈: 「조작 허용」 을 먼저 체크하세요')
            return
        if getattr(self, 'picker', None) is not None and self.picker.running:
            self.node.log('WARN', '복구 후 홈: 집기/놓기 순서가 진행 중')
            return
        if not self._ask('복구 후 홈', f'지금 로봇 상태: {self.backend.robot_state_text()}\n\n'
                         '1. 안전 정지 해제 또는 서보 켜기\n2. 수동 모드였으면 자동 모드로\n'
                         f'3. 홈 자세 {HOME_DEG_REAL}° 로 관절 이동 (속도 슬라이더 비율, 최대 45°/s)\n\n'
                         '팔이 크게 움직일 수 있습니다. 주변이 비었고 비상정지를 누를 수 있습니까?\n'
                         '(그리퍼가 물체를 들고 있으면 함께 움직입니다)'):
            return
        self.last_cmd_time = time.time()
        self.backend.recover_and_home(self.sld_speed.value())

    def _stop(self):
        self.queue.clear()
        self.queue_running = False
        self.backend.stop()

    def _arm_ok(self, what):
        if not self.chk_arm.isChecked():
            self.node.log('WARN', f'{what}: 「조작 허용」 을 먼저 체크하세요')
            return False
        why = self.backend.why_not_ready()
        if why:
            self.node.log('WARN', f'{what}: {why}')
            return False
        self.last_cmd_time = time.time()
        return True

    def _toggle_hand_guide(self, on):
        manual, switching, _name = self.backend.hand_guide_state()
        self.chk_hand.setChecked(bool(manual))          # 실제 모드는 주기 갱신이 다시 맞춘다
        if switching:
            return
        if on:
            if getattr(self, 'picker', None) is not None and self.picker.running:
                self.node.log('WARN', '손으로 움직이기: 집기/놓기 순서가 진행 중 - 끝난 뒤에 켜세요')
                return
            if not self._ask('손으로 움직이기 (수동 모드)',
                             '로봇을 수동 모드로 바꿉니다.\n\n'
                             '· 팔 끝 콕핏의 직접 교시 버튼을 누르고 있는 동안만 팔이 손에 따라 움직입니다.\n'
                             '· 버튼을 놓으면 그 자리에 멈춥니다.\n'
                             '· 켜져 있는 동안 GUI 의 이동·그리퍼·집기 명령은 막힙니다.\n'
                             '· 그리퍼가 물체를 들고 있으면 먼저 내려놓으세요.\n\n계속할까요?'):
                return
        self.chk_arm.setChecked(False)
        self.backend.set_hand_guide(on)

    def _jog_tcp(self, axis, sign):
        if self._arm_ok('TCP 이동'):
            self.backend.jog_tcp(axis, sign * self.cmb_step.currentData(), self.sld_speed.value())

    def _jog_joint(self, i, sign):
        if self._arm_ok('관절 이동'):
            self.backend.jog_joint(i, sign * self.cmb_step.currentData(), self.sld_speed.value())

    def _gripper(self, close):
        if self._arm_ok('그리퍼'):
            self.backend.gripper(close)

    # ---- 작업 큐
    def _queue_add(self, color=None, zone=None, dry=None, target_id='', place_xy=None):
        self.queue.append({
            'color': '' if target_id else (color or self.cmb_color.currentData()),
            'zone': '' if place_xy is not None else (zone or self.cmb_zone.currentData()),
            'dry': self.chk_dry.isChecked() if dry is None else dry,
            'target_id': target_id, 'place_xy': place_xy})
        self._render_queue()

    def _queue_del(self):
        row = self.lst_queue.currentRow()
        if 0 <= row < len(self.queue):
            del self.queue[row]
            self._render_queue()

    # ---- 화면에서 고르기 (카메라 클릭; 손짓 포인터도 이 함수들을 쓴다)
    def _on_camera_click(self, x, y):
        """클릭한 탁자 위 점 (손짓 포인터의 대용): 
        1) 큐브 위(4 cm 안) -> 그 큐브를 고른다 (음성 「이거」)
        2) 탁자의 빈 곳     -> 놓을 자리로 정한다 (음성 「저기」). 순서는 상관없다."""
        objs = [o for o in self.node.snapshot()['objects'] if o.id]
        near = min(objs, key=lambda o: math.hypot(o.pose.pose.position.x - x, o.pose.pose.position.y - y),
                   default=None)
        if near is not None and math.hypot(near.pose.pose.position.x - x, near.pose.pose.position.y - y) < 0.04:
            self.sel_id = near.id
            self.node.log('INFO', f'선택(이거): 물체 {near.id}' +
                          ('' if self.sel_place else '  → 놓을 빈 자리를 클릭하세요'))
        else:
            self.sel_place = (x, y)
            self.node.log('INFO', f'선택(저기): 놓을 자리 ({x:.3f}, {y:.3f})' +
                          ('  → 「▶ 선택한 물체를 그 자리로」 또는 「이거 저기로 옮겨줘」' if self.sel_id
                           else '  → 옮길 큐브를 클릭하거나 「파란 큐브 저기로」 라고 말하세요'))
        self._update_selection_label()

    def _update_selection_label(self):
        place = '—' if self.sel_place is None else f'({self.sel_place[0]:.3f}, {self.sel_place[1]:.3f})'
        self.lbl_sel.setText(f'화면 선택: 물체 {self.sel_id or "—"}  →  놓을 자리 {place}')

    def _clear_selection(self):
        self.sel_id, self.sel_place = '', None
        self._update_selection_label()

    def _run_selection(self):
        if not self.sel_id or self.sel_place is None:
            return
        if self.backend.mode == 'real':
            self.node.log('WARN', '실물 모드에서는 화면 선택 작업을 실행하지 않음 (검증 전)')
            return
        self.queue.insert(0, {'color': '', 'zone': '', 'dry': self.chk_dry.isChecked(),
                              'target_id': self.sel_id, 'place_xy': self.sel_place})
        self._render_queue()
        self.queue_running = True
        self._next_task()
        self._clear_selection()

    def _render_queue(self):
        self.lst_queue.clear()
        for q in self.queue:
            what = q['target_id'] or f'{COLOR_KO.get(q["color"], q["color"])} 큐브'
            where = (f'({q["place_xy"][0]:.2f}, {q["place_xy"][1]:.2f})' if q['place_xy'] is not None
                     else f'구역 {q["zone"]}')
            self.lst_queue.addItem(f'{what} → {where}' + (' (계획만)' if q['dry'] else ''))

    def _start_queue(self):
        if self.backend.mode == 'real':
            QMessageBox.information(self, '작업 실행', '실물 로봇의 자동 작업(Task Manager 두산 백엔드)은 아직 '
                                    '검증되지 않아 이 화면에서 막아 두었습니다.\n수동 조작을 사용하세요.')
            return
        if not self.queue:
            self._queue_add()
        self.queue_running = True
        self._next_task()

    def _next_task(self):
        if not self.queue_running or self.node.task_running:
            return
        if not self.queue:
            self.queue_running = False
            self.node.log('INFO', '작업 큐 완료')
            return
        q = self.queue.pop(0)
        self._render_queue()
        self.current_task = q
        if not self.node.send_task(q['color'], q['zone'], q['dry'], target_id=q['target_id'],
                                   place_xy=q['place_xy']):
            self.queue_running = False

    def _task_done_from_ros(self, ok):
        # ROS 스레드에서 불린다 -> Qt 타이머(_refresh)가 다음 작업을 보낸다
        if not ok:
            self.queue_running = False
            self.queue.clear()
        self._pending_next = True

    # ---- 음성
    def _talk(self, on):
        self.node.voice_listen(on)

    def _send_text(self):
        t = self.txt_cmd.text().strip()
        if t:
            self.node.voice_text(t)
            self.txt_cmd.clear()

    def _voice_run(self, auto=False):
        v = self.node.snapshot()['voice']
        if v is None or not v.valid:
            self.node.log('WARN', '실행할 수 있는 음성 명령이 없음')
            return
        if v.action == 'place' and self.backend.mode != 'real':
            self.node.log('WARN', '「내려놔」(집은 자리에 놓기) 는 Task Manager 의 pick/place 분리 후 지원 예정 - '
                          '지금은 집기 명령이 놓기까지 한 번에 수행합니다')
            return
        if self.backend.mode == 'real':
            self._real_voice(v)
            return
        if v.action == 'home':                      # 시뮬레이션: ⌂ Home 버튼과 같음
            self.node.log('INFO', f'음성 명령 {"자동 " if auto else ""}실행: 홈으로 돌아가기')
            self._home()
            return
        # 무엇을: 「이거」 = 화면에서 고른 큐브, 아니면 말한 색
        target_id = ''
        if v.refers_selected_object and not v.color:
            if not self.sel_id:
                self.node.log('WARN', '「이거」 가 무엇인지 모름 - 먼저 카메라 화면에서 큐브를 클릭하세요')
                return
            target_id = self.sel_id
        # 어디에: 「저기」 = 화면에서 고른 자리, 아니면 구역 선택 상자
        place_xy, zone = None, self.cmb_voice_zone.currentData()
        if v.refers_selected_place:
            if self.sel_place is None:
                self.node.log('WARN', '「저기」 가 어디인지 모름 - 먼저 카메라 화면에서 탁자의 빈 곳을 클릭하세요')
                return
            place_xy, zone = self.sel_place, ''
        what = target_id or v.color
        where = f'({place_xy[0]:.3f}, {place_xy[1]:.3f})' if place_xy else f'구역 {zone}'
        self.node.log('INFO', f'음성 명령 {"자동 " if auto else ""}실행: {what} -> {where}')
        self.queue.insert(0, {'color': '' if target_id else v.color, 'zone': zone, 'dry': False,
                              'target_id': target_id, 'place_xy': place_xy})
        self._clear_selection()
        self._render_queue()
        self.queue_running = True
        self._next_task()

    def _toggle_camera_large(self):
        large = self.side_box.isVisible()
        self.side_box.setVisible(not large)
        self.bottom_box.setVisible(not large)

    def _refresh_camera(self):
        n = self.node
        with n.lock:
            image, seq, objects, px = n.image, n.image_seq, list(n.objects), list(n.px_objects)
        if not n.px_rate.fresh(2.0):
            px = []                                 # 검출 노드가 꺼졌으면 오래된 상자를 지운다
        if image is None and not objects:
            return
        target = self.current_task['color'] if (n.task_running and self.current_task) else ''
        self.camera.show_detections = self.chk_detect.isChecked()
        self.camera.selected_id = self.sel_id
        self.camera.place_xy = self.sel_place
        self.camera.place_px = self.place_px if self.backend.mode == 'real' else None
        self.camera.set_frame(image, objects, n.camera_transform(), target, seq, px)

    # ================================================================== 주기 갱신 (100 ms)
    def _refresh(self):
        n = self.node
        snap = n.snapshot()
        s = n.status_flags()
        b = self.backend
        if hasattr(b, 'pump_logs'):
            b.pump_logs()
        if self._pending_next:
            self._pending_next = False
            self._next_task()

        # ---- 버튼
        self.btn_connect.setEnabled(not b.connected())
        self.btn_start.setEnabled(b.mode == 'sim' and s['task_manager'] and not n.task_running)
        if self.chk_arm.isChecked() and time.time() - self.last_cmd_time > ARM_TIMEOUT_SEC:
            self.chk_arm.setChecked(False)
            n.log('INFO', f'{ARM_TIMEOUT_SEC}초 동안 명령이 없어 조작을 잠갔습니다')
        why = b.why_not_ready()
        if not self.chk_arm.isChecked():
            why = why or '「조작 허용」 이 꺼져 있음'
        self.lbl_jog_reason.setText('' if why is None else f'이동 불가: {why}')
        for w in self.jog_buttons + [self.btn_gopen, self.btn_gclose]:
            w.setEnabled(why is None)
        self.btn_home.setEnabled(why is None)
        if b.mode == 'real':
            st = b.robot_state_text()
            bad = st not in ('대기', '이동 중')
            self.lbl_robot_state.setText(f'로봇 상태: {st}')
            self.lbl_robot_state.setStyleSheet(f'color:{BAD_COLOR}' if bad else '')
            self.btn_recover.setEnabled(b.link.cli_control.service_is_ready() and not b.link.recovering)
            manual, switching, name = b.hand_guide_state()
            self.chk_hand.blockSignals(True)
            self.chk_hand.setChecked(bool(manual))
            self.chk_hand.blockSignals(False)
            self.chk_hand.setEnabled(manual is not None and not switching)
            self.chk_hand.setText('손으로 움직이기 (수동 모드)' + (' - 바꾸는 중…' if switching else
                                  '' if manual is None else f' · 지금: {name}'))

        # ---- 시스템 상태
        L = self.sys_labels
        L['conn'].setText(dot(b.connected(), b.describe(), warn=True))
        L['gazebo'].setText(dot(s['joint_states'], f'관절 상태 {s["js_hz"]:.0f} Hz' if s['joint_states']
                                else '관절 상태 없음'))
        L['camera'].setText(dot(s['camera_hz'] > 1, f'{s["camera_hz"]:.1f} Hz' if s['camera_hz'] else '영상 없음'))
        L['detect'].setText(dot(s['detections'], f'{len(snap["objects"])}개' if s['detections'] else '결과 없음'))
        L['gripper'].setText(dot(*b.gripper_status()))
        L['moveit'].setText(dot(s['moveit'], '준비' if s['moveit'] else '없음'))
        L['tm'].setText(dot(s['task_manager'], '준비' if s['task_manager'] else '없음'))
        vs = snap['voice_state']
        L['voice'].setText(dot(s['voice'] and vs in ('ready', 'recording', 'recognizing'),
                               {'ready': '준비', 'recording': '녹음 중', 'recognizing': '인식 중',
                                'loading': '모델 로딩 중'}.get(vs, vs or '없음'), warn=vs == 'loading'))

        gs = snap['gesture_state']
        L['gesture'].setText(dot(s['gesture'] and gs == 'ready', {'ready': '준비', 'loading': 'MediaPipe 로딩 중'}.get(
            gs, gs or '없음'), warn=gs == 'loading'))
        if b.mode == 'real':
            self._gesture_tick(snap['gesture'])

        # ---- 로봇 상태
        ts = snap['task_status']
        self.lbl_task_state.setText('—' if ts is None else
                                    f'{["대기", "작업 중", "오류"][ts.state] if ts.state < 3 else ts.state} '
                                    f'{ts.stage} {ts.task_id}')
        j = b.joints_deg()
        self.lbl_joints.setText('—' if j is None else
                                '  '.join(f'J{i + 1} {v:7.1f}' + ('\n' if i == 2 else '') for i, v in enumerate(j)))
        tcp = b.tcp_text()
        if tcp is None:
            self.lbl_tcp.setText('—')
        else:
            (x, y, z), (r, p, yw) = tcp
            self.lbl_tcp.setText(f'X {x:6.3f}  Y {y:6.3f}  Z {z:6.3f}\n'
                                 f'R {math.degrees(r):6.1f}  P {math.degrees(p):6.1f}  Y {math.degrees(yw):6.1f}')
        gp = b.gripper_position()
        if gp is None:
            self.lbl_grip.setText('—')
        else:
            self.bar_grip.setValue(int(gp * 100))
            state = '열림' if gp < 0.05 else ('닫힘 (빈손)' if gp > 0.9 else '닫힘 (물체를 잡고 있음)')
            self.lbl_grip.setText(f'{state}' + (f'  ·  {n.gripper_msg}' if n.gripper_msg else ''))

        # ---- 카메라
        cam_tf = n.camera_transform()
        target = self.current_task['color'] if (n.task_running and self.current_task) else ''
        img = snap['image']
        self.lbl_cam_info.setText('—' if img is None else
                                  f'RGB ({img[0]} × {img[1]})  |  {s["camera_hz"]:.1f} Hz  |  ' +
                                  (f'색 검출(픽셀) {len(n.px_objects)}개' if s['px_detections'] else
                                   f'검출 {len(snap["objects"])}개') +
                                  ('  |  ※ 시뮬레이션 정답 위치 (aris3_vision 전 임시)' if b.mode == 'sim' else ''))
        self.lbl_cal.setText(('카메라 TF: ' + ('있음' if cam_tf else '없음')) +
                             '\n시뮬레이션: 설치 위치 고정값 (탁자 위 2.0 m, 화각 80°) - 보정 없음'
                             if b.mode == 'sim' else '실물 카메라 보정: 아직 없음 (D455 연동 예정)')
        pose = n.tcp_pose()
        self.workspace.set_state(snap['objects'], None if pose is None else pose[1], target, self.sel_place)
        self.btn_sel_run.setEnabled(bool(self.sel_id) and self.sel_place is not None and
                                    b.mode == 'sim' and s['task_manager'] and not n.task_running)

        # ---- 작업 단계
        icons = {'': ('대기', '#90a4ae'), 'run': ('▶ 실행 중', '#1565c0'), 'done': ('✓ 완료', OK_COLOR),
                 'fail': ('✗ 실패', BAD_COLOR)}
        for i, st in enumerate(STAGES):
            text, color = icons[snap['stage_state'][st]]
            item = self.tbl_stage.item(i, 1)
            item.setText(text)
            item.setForeground(QColor(color))
            t = snap['stage_time'][st]
            self.tbl_stage.item(i, 2).setText(f'{t:.1f} s' if t else '')
        fb = snap['feedback']
        if n.task_running and fb is not None:
            self.bar_task.setValue(int(fb.progress * 100))
            self.lbl_task_msg.setText(f'{fb.stage}: {fb.detail}')
            self.lbl_task_msg.setStyleSheet('')
        elif snap['last_result'] is not None:
            ok, msg = snap['last_result']
            self.bar_task.setValue(100 if ok else self.bar_task.value())
            self.lbl_task_msg.setText(('✓ ' if ok else '✗ ') + msg)
            self.lbl_task_msg.setStyleSheet(f'color:{OK_COLOR if ok else BAD_COLOR}')

        # ---- 음성
        self.lbl_voice_state.setText({'ready': '🟢 준비 — 버튼을 누르는 동안 말하세요',
                                      'recording': '🔴 녹음 중...', 'recognizing': '⏳ 인식 중...',
                                      'loading': '⏳ Whisper 모델 로딩 중'}.get(vs, f'⚪ 음성 노드 없음 {vs}'))
        self.btn_talk.setEnabled(vs in ('ready', 'recording'))
        v = snap['voice']
        if v is not None and v.stamp != self.last_voice_stamp:
            self.last_voice_stamp = v.stamp
            self.lbl_heard.setText(f'인식: 「{v.text}」' + (f'  ({v.stt_time_s:.1f} s)' if v.stt_time_s else ''))
            what = {'pick': '집어서 옮기기', 'place': '내려놓기', 'home': '홈으로 돌아가기'}.get(v.action, '?')
            chosen = (self.pick_name if self.pick_px is not None else '') if b.mode == 'real' else self.sel_id
            obj = (f'이거 = {chosen or "(화면에서 큐브를 클릭하세요)"}' if v.refers_selected_object and not v.color
                   else f'{COLOR_KO.get(v.color, v.color or "?")} {v.shape or "물체"}')
            if v.refers_selected_place:
                obj += (f' → 저기 = ({self.sel_place[0]:.2f}, {self.sel_place[1]:.2f})' if self.sel_place
                        else ' → 저기 = (빈 곳을 클릭하세요)')
            self.lbl_parsed.setText(f'해석: {obj} · {what}  —  ' +
                                    (f'<span style="color:{OK_COLOR}">✓ {v.message}</span>' if v.valid else
                                     f'<span style="color:{BAD_COLOR}">✗ {v.message}</span>'))
            # 화면에서 고른 큐브·자리를 가리키는 명령(「저기로 옮겨줘」)은 클릭으로 이미 확인했으므로 바로 실행.
            # 색만 말한 명령은 「실행」 또는 「자동 실행」 이 있어야 한다.
            uses_selection = v.refers_selected_place and (bool(self.sel_id) or bool(v.color))
            if v.valid and b.mode == 'real':
                # 실물: 인식되면 바로 확인창을 띄운다 (예 를 눌러야 움직임). 주기 갱신 밖에서 연다
                QTimer.singleShot(0, lambda: self._voice_run(auto=True))
            elif v.valid and (self.chk_voice_auto.isChecked() or uses_selection):
                self._voice_run(auto=True)
        busy = self.picker.running if b.mode == 'real' else n.task_running
        self.btn_voice_run.setEnabled(v is not None and v.valid and not busy)

        # ---- 로그
        new = [e for e in snap['logs'] if e[0] > self.log_shown]
        if new:
            self.log_shown = new[-1][0]
            only = self.chk_log_aris3.isChecked()
            for _seq, level, name, text, stamp in new:
                if only and not (name == 'gui' or name.startswith(LOG_NODES) or level in ('WARN', 'ERROR')):
                    continue
                if only and any(s in text for s in NOISE):
                    continue
                color = {'WARN': '#ef6c00', 'ERROR': '#c62828', 'FATAL': '#c62828'}.get(level, '#1565c0')
                self.txt_log.appendHtml(f'<span style="color:{color}">[{level}]</span> '
                                        f'<span style="color:#78909c">[{stamp}] [{name}]</span> {text}')

    def closeEvent(self, ev):
        self.timer.stop()
        if self.backend.mode == 'sim' and self.backend.stack.running():
            if self._ask('종료', '이 화면이 실행한 시뮬레이션도 함께 끌까요?'):
                self.backend.shutdown()
        elif self.backend.mode == 'real':
            self.backend.shutdown()
        super().closeEvent(ev)

