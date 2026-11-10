"""상단 버튼 줄: Connect · Disconnect · Home · Start Task · Stop · 정지(소프트웨어)."""
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton

from aris3_gui.panels.common import nofocus


class HeaderMixin:
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

    def _refresh_header(self, s):
        b = self.backend
        self.btn_connect.setEnabled(not b.connected())
        self.btn_start.setEnabled(b.mode == 'sim' and s['task_manager'] and not self.node.task_running)
