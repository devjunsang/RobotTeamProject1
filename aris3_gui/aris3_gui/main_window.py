"""ARIS3 Control Station - 통합 관제 화면 (마스터플랜 07장 목업 배치).

  ┌ 상단: Connect · Disconnect · Home · Start Task · Stop · 정지(소프트웨어) ───────────────┐
  │ 시스템 상태   │ 카메라 Top View (검출 표시)        │ 작업 공간 2D (RViz 내장은 도전)   │
  │ 로봇 상태     │                                     │ 작업 큐 / 단계 진행               │
  │ 그리퍼 상태   ├─────────────────────────────────────┼──────────────┬────────────────────┤
  │ 카메라 정보   │ 로그                                │ 음성 명령    │ 수동 조작          │
  └───────────────┴─────────────────────────────────────┴──────────────┴────────────────────┘
  (실물 모드는 음성 명령과 수동 조작 사이에 「실물 집기 · 카메라 보정」 패널이 들어간다)

이 파일은 화면 배치, 안전 공통 동작(확인창·조작 허용·정지), 주기 갱신 순서만 맡는다.
패널별 화면 구성·버튼 동작·갱신은 aris3_gui/panels/ 의 파일에 있다 (panels/__init__.py 참고).

안전 규칙 (e0509_manual/gui.py 와 같음):
  - 수동 조작·그리퍼 버튼은 '조작 허용' 을 체크해야 동작하고, 2분 동안 명령이 없으면 다시 잠긴다.
  - 정지는 언제나 동작한다 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
  - 작업 실행 중에는 수동 조작을 받지 않는다.
  - 음성 명령은 화면에 해석 결과를 보여 주고, '자동 실행' 을 켠 경우가 아니면 사용자가 '실행' 을 눌러야 한다.
"""
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QIcon, QKeySequence
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLineEdit, QMessageBox, QShortcut, QVBoxLayout, QWidget

from aris3_gui.panels.camera import CameraMixin
from aris3_gui.panels.header import LOGO_SVG, HeaderMixin
from aris3_gui.panels.jog import JogMixin
from aris3_gui.panels.log import LogMixin
from aris3_gui.panels.real_pick_panel import RealPickMixin
from aris3_gui.panels.status import StatusMixin
from aris3_gui.panels.task_queue import TaskQueueMixin
from aris3_gui.panels.voice import VoiceMixin


class ControlStation(HeaderMixin, StatusMixin, CameraMixin, TaskQueueMixin, VoiceMixin, RealPickMixin,
                     JogMixin, LogMixin, QWidget):
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

        self.setWindowTitle(f'ARIS — ARIS3 Control Station — {backend.title}')
        if LOGO_SVG.is_file():
            self.setWindowIcon(QIcon(str(LOGO_SVG)))      # 작업 표시줄·창 아이콘도 ARIS 엠블럼
        self.setFont(QFont('NanumSquare', 10))
        self.setStyleSheet(
            'QGroupBox { font-weight: bold; border: 1px solid #cfd8dc; border-radius: 6px; margin-top: 10px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }'
            'QPushButton { padding: 4px 10px; }')

        root = QVBoxLayout(self)
        root.addWidget(self._header())
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

    # ================================================================== 안전 공통 동작
    def _ask(self, title, text):
        self._dialog_open = True
        try:
            return QMessageBox.question(self, title, text, QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.No) == QMessageBox.Yes
        finally:
            self._dialog_open = False

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

    def _stop_key(self):
        """Space: 어디서나 정지. 단, 문장 입력창에 글을 치는 중이면 띄어쓰기로 넣는다 (Esc·정지 버튼은 항상 정지)."""
        w = QApplication.focusWidget()
        if isinstance(w, QLineEdit) and not self.node.task_running and not (
                getattr(self, 'picker', None) is not None and self.picker.running):
            w.insert(' ')
            return
        self._stop()

    def _stop(self):
        self.queue.clear()
        self.queue_running = False
        self.backend.stop()

    # ================================================================== 주기 갱신 (100 ms)
    def _refresh(self):
        n = self.node
        snap = n.snapshot()
        s = n.status_flags()
        b = self.backend
        if self._pending_next:
            self._pending_next = False
            self._next_task()
        self._refresh_header(s)
        self._refresh_jog()
        self._refresh_system(snap, s)
        if b.mode == 'real':
            self._gesture_tick(snap['gesture'])
            self._refresh_real_pick()
        self._refresh_robot(snap)
        self._refresh_views(snap, s)
        self._refresh_camera_info(n.camera_transform())
        self._refresh_queue(snap, s)
        self._refresh_voice(snap)
        self._refresh_log(snap)

    def closeEvent(self, ev):
        self.timer.stop()
        if self.backend.mode == 'sim' and self.backend.stack.running():
            if self._ask('종료', '이 화면이 실행한 시뮬레이션도 함께 끌까요?'):
                self.backend.shutdown()
        elif self.backend.mode == 'real':
            self.backend.shutdown()
        super().closeEvent(ev)
