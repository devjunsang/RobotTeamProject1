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
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
                             QHeaderView, QLabel, QLineEdit, QListWidget, QMessageBox,
                             QPlainTextEdit, QProgressBar, QPushButton, QShortcut, QSlider,
                             QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from aris3_gui.ros_io import LOG_NODES, STAGES
from aris3_gui.widgets import COLOR_KO, CameraView, WorkspaceView

ARM_TIMEOUT_SEC = 120
STAGE_KO = {'VALIDATE': '요청·장비 확인', 'OBSERVE': '관찰 자세', 'PERCEIVE': '물체 인식',
            'PLAN': '경로 계획', 'PICK': '집기', 'PLACE': '놓기', 'VERIFY': '결과 확인'}
ZONES = ['A', 'B', 'C']
TASK_COLORS = ['red', 'blue', 'yellow']
OK_COLOR, BAD_COLOR, WAIT_COLOR = '#2e7d32', '#c62828', '#ef6c00'


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
        self.queue = []                 # 남은 작업 [(color, zone, dry_run)]
        self.queue_running = False
        self.current_task = None
        self.last_cmd_time = time.time()
        self.log_shown = 0
        self.last_voice_stamp = None
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
        side = QVBoxLayout()
        side.addWidget(self._workspace_panel(), 2)
        side.addWidget(self._queue_panel(), 3)
        top.addLayout(side, 2)
        right.addLayout(top, 3)
        bottom = QHBoxLayout()
        bottom.addWidget(self._log_panel(), 3)
        bottom.addWidget(self._voice_panel(), 2)
        bottom.addWidget(self._jog_panel(), 2)
        right.addLayout(bottom, 2)
        body.addLayout(right, 1)
        root.addLayout(body, 1)

        for key in (Qt.Key_Space, Qt.Key_Escape):
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(self._stop)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

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
                                         ('tm', 'Task Manager'), ('voice', '음성 인식')]):
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
        box = QGroupBox('카메라 뷰 (Camera View - Top View)')
        v = QVBoxLayout(box)
        top = QHBoxLayout()
        self.chk_detect = nofocus(QCheckBox('검출 표시 (Detection)'))
        self.chk_detect.setChecked(True)
        top.addStretch(1)
        top.addWidget(self.chk_detect)
        v.addLayout(top)
        self.camera = CameraView()
        v.addWidget(self.camera, 1)
        self.lbl_cam_info = QLabel('—')
        self.lbl_cam_info.setStyleSheet('color:#607d8b')
        v.addWidget(self.lbl_cam_info)
        return box

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
        self.txt_cmd.setPlaceholderText('또는 문장 입력 (예: 노란 큐브 집어줘)')
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
        v.addWidget(self.chk_arm)
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
        self.lbl_jog_reason = QLabel('')
        self.lbl_jog_reason.setStyleSheet(f'color:{WAIT_COLOR}')
        self.lbl_jog_reason.setWordWrap(True)
        v.addWidget(self.lbl_jog_reason)
        return box

    # ================================================================== 동작
    def _ask(self, title, text):
        return QMessageBox.question(self, title, text, QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No) == QMessageBox.Yes

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
    def _queue_add(self, color=None, zone=None, dry=None):
        item = (color or self.cmb_color.currentData(), zone or self.cmb_zone.currentData(),
                self.chk_dry.isChecked() if dry is None else dry)
        self.queue.append(item)
        self._render_queue()

    def _queue_del(self):
        row = self.lst_queue.currentRow()
        if 0 <= row < len(self.queue):
            del self.queue[row]
            self._render_queue()

    def _render_queue(self):
        self.lst_queue.clear()
        for c, z, dry in self.queue:
            self.lst_queue.addItem(f'{COLOR_KO.get(c, c)} 큐브 → 구역 {z}' + (' (계획만)' if dry else ''))

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
        color, zone, dry = self.queue.pop(0)
        self._render_queue()
        self.current_task = (color, zone)
        if not self.node.send_task(color, zone, dry):
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
        if v.action == 'place':
            self.node.log('WARN', '「내려놔」(집은 자리에 놓기) 는 Task Manager 의 pick/place 분리 후 지원 예정 - '
                          '지금은 집기 명령이 놓기까지 한 번에 수행합니다')
            return
        if self.backend.mode == 'real':
            self.node.log('WARN', '실물 모드에서는 음성으로 작업을 실행하지 않음 (검증 전)')
            return
        zone = self.cmb_voice_zone.currentData()
        self.node.log('INFO', f'음성 명령 {"자동 " if auto else ""}실행: {v.color} -> {zone}')
        self.queue.insert(0, (v.color, zone, False))
        self._render_queue()
        self.queue_running = True
        self._next_task()

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

        # ---- 시스템 상태
        L = self.sys_labels
        L['conn'].setText(dot(b.connected(), b.describe(), warn=True))
        L['gazebo'].setText(dot(s['joint_states'], f'관절 상태 {s["js_hz"]:.0f} Hz' if s['joint_states']
                                else '관절 상태 없음'))
        L['camera'].setText(dot(s['camera_hz'] > 1, f'{s["camera_hz"]:.1f} Hz' if s['camera_hz'] else '영상 없음'))
        L['detect'].setText(dot(s['detections'], f'{len(snap["objects"])}개' if s['detections'] else '결과 없음'))
        L['gripper'].setText(dot(s['gripper'], '준비' if s['gripper'] else '서비스 없음'))
        L['moveit'].setText(dot(s['moveit'], '준비' if s['moveit'] else '없음'))
        L['tm'].setText(dot(s['task_manager'], '준비' if s['task_manager'] else '없음'))
        vs = snap['voice_state']
        L['voice'].setText(dot(s['voice'] and vs in ('ready', 'recording', 'recognizing'),
                               {'ready': '준비', 'recording': '녹음 중', 'recognizing': '인식 중',
                                'loading': '모델 로딩 중'}.get(vs, vs or '없음'), warn=vs == 'loading'))

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
        target = self.current_task[0] if (n.task_running and self.current_task) else ''
        self.camera.show_detections = self.chk_detect.isChecked()
        self.camera.set_frame(snap['image'], snap['objects'], cam_tf, target)
        img = snap['image']
        self.lbl_cam_info.setText('—' if img is None else
                                  f'RGB ({img[0]} × {img[1]})  |  {s["camera_hz"]:.1f} Hz  |  '
                                  f'검출 {len(snap["objects"])}개' +
                                  ('  |  ※ 시뮬레이션 정답 위치 (aris3_vision 전 임시)' if b.mode == 'sim' else ''))
        self.lbl_cal.setText(('카메라 TF: ' + ('있음' if cam_tf else '없음')) +
                             '\n시뮬레이션: 설치 위치 고정값 (탁자 위 2.0 m, 화각 80°) - 보정 없음'
                             if b.mode == 'sim' else '실물 카메라 보정: 아직 없음 (D455 연동 예정)')
        pose = n.tcp_pose()
        self.workspace.set_state(snap['objects'], None if pose is None else pose[1], target)

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
            what = {'pick': '집어서 옮기기', 'place': '내려놓기'}.get(v.action, '?')
            obj = f'{COLOR_KO.get(v.color, v.color or "?")} {v.shape or "물체"}'
            self.lbl_parsed.setText(f'해석: {obj} · {what}  —  ' +
                                    (f'<span style="color:{OK_COLOR}">✓ {v.message}</span>' if v.valid else
                                     f'<span style="color:{BAD_COLOR}">✗ {v.message}</span>'))
            if v.valid and self.chk_voice_auto.isChecked():
                self._voice_run(auto=True)
        self.btn_voice_run.setEnabled(v is not None and v.valid and not n.task_running)

        # ---- 로그
        new = [e for e in snap['logs'] if e[0] > self.log_shown]
        if new:
            self.log_shown = new[-1][0]
            only = self.chk_log_aris3.isChecked()
            for _seq, level, name, text, stamp in new:
                if only and not (name == 'gui' or name.startswith(LOG_NODES) or level in ('WARN', 'ERROR')):
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

