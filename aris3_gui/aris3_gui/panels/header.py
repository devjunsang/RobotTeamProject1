"""상단 버튼 줄: Connect · Disconnect · Home · Start Task · Stop · 정지(소프트웨어), 가운데 ARIS 로고."""
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget

from aris3_gui.panels.common import nofocus

LOGO_SVG = Path(__file__).resolve().parent.parent / 'assets' / 'aris_mark.svg'   # ARIS 엠블럼 (2026-11-10)


class LogoBadge(QWidget):
    """ARIS 엠블럼 + 글자. 상단 줄 가운데에 겹쳐 놓는다 (버튼 배치와 상관없이 창 가운데에 오도록)."""

    def __init__(self, parent):
        """엠블럼(SVG) 52 px 와 「ARIS」 금색 글자. 파일이 없으면 글자만."""
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        if LOGO_SVG.is_file():
            mark = QSvgWidget(str(LOGO_SVG))
            mark.setFixedSize(52, 52)
            lay.addWidget(mark)
        word = QLabel('<span style="font-size:26pt; font-weight:800; letter-spacing:6px; color:#b8860b">ARIS</span>')
        word.setToolTip('ARIS - ARIS3 Control Station')
        lay.addWidget(word)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)   # 로고 뒤 버튼·글자를 가리지 않게 클릭은 통과
        self.adjustSize()


class HeaderBar(QWidget):
    """상단 줄: 왼쪽 제목, 오른쪽 버튼, 가운데(겹쳐서) 로고. 로고가 버튼과 겹치면 왼쪽으로 비켜 놓는다."""

    def __init__(self):
        """빈 줄을 만들고, _header 가 제목·버튼을 layout 에 넣은 뒤 로고를 붙인다."""
        super().__init__()
        self.row = QHBoxLayout(self)
        self.row.setContentsMargins(0, 0, 0, 0)
        self.logo = None
        self.first_button = None
        self.title = None

    def set_logo(self, logo, first_button, title=None):
        """로고와 겹침 확인용 위젯(가장 왼쪽 버튼, 왼쪽 제목)을 정한다."""
        self.logo, self.first_button, self.title = logo, first_button, title
        logo.raise_()
        self._place()

    def resizeEvent(self, ev):
        """창 크기가 바뀌면 로고를 다시 가운데로."""
        super().resizeEvent(ev)
        self._place()

    def _place(self):
        """로고를 줄 가운데에 두되, 버튼·제목과 겹치지 않게 (버튼보다 12 px 왼쪽, 제목 글자보다 12 px 오른쪽)."""
        if self.logo is None:
            return
        w, h = self.logo.sizeHint().width(), self.logo.sizeHint().height()
        x = (self.width() - w) // 2
        if self.first_button is not None and self.first_button.isVisible():
            x = min(x, self.first_button.geometry().left() - w - 12)
        if self.title is not None:
            x = max(x, self.title.geometry().left() + self.title.sizeHint().width() + 12)
        self.logo.setGeometry(max(0, x), (self.height() - h) // 2, w, h)


class HeaderMixin:
    """상단 줄 mixin: 제목, 가운데 ARIS 로고, Connect·Disconnect·Home·Start·Stop·정지 버튼."""
    def _header(self):
        """상단 줄 위젯을 만든다 (제목 · 가운데 로고 · 버튼)."""
        bar = HeaderBar()
        row = bar.row
        title = QLabel('<b style="font-size:18pt">E0509 Control Station</b><br>'
                       '<span style="color:#607d8b">Doosan E0509 + RH-P12-RN-A &nbsp;|&nbsp; '
                       f'ROS 2 Jazzy + Gazebo + MoveIt 2 &nbsp;|&nbsp; 모드: {self.backend.title}</span>')
        row.addWidget(title, 1)

        def btn(text, slot, style=''):
            """상단 버튼 하나를 만들어 줄에 넣는다 (Space 가 누르지 않게 포커스 없음)."""
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
        bar.set_logo(LogoBadge(bar), self.btn_connect, title)
        return bar

    def _connect(self):
        """Connect: 실물(가상 포함)이면 확인창 뒤 접속, 시뮬레이션이면 Gazebo·MoveIt·Task Manager 시작."""
        if self.backend.mode == 'real':
            if not self._ask('로봇 접속', '실물 로봇에 접속합니다. 서보가 켜지고 펜던트의 제어 권한을 가져옵니다.\n'
                             '팔은 움직이지 않습니다.\n\n로봇 옆에 사람이 있고 비상정지를 누를 수 있습니까?'):
                return
        self.backend.connect()

    def _disconnect(self):
        """Disconnect: 확인창 -> 조작 잠금·작업 큐 비우기 -> 접속 끊기(실물: 서보 꺼짐)."""
        msg = ('시뮬레이션(Gazebo·MoveIt·Task Manager)을 종료합니다.' if self.backend.mode == 'sim' else
               '접속을 끊으면 로봇 서보가 꺼집니다(브레이크가 팔을 잡습니다).')
        if self._ask('Disconnect', msg + '\n\n계속할까요?'):
            self.chk_arm.setChecked(False)
            self.queue.clear()
            self.queue_running = False
            self.backend.disconnect()

    def _home(self):
        """⌂ Home: 조작 허용 검사 후 홈 자세로 (속도 슬라이더 비율)."""
        if not self._arm_ok('홈 이동'):
            return
        self.backend.home(self.sld_speed.value())

    def _refresh_header(self, s):
        """주기 갱신: 접속돼 있으면 Connect 를 끄고, Task Manager 가 있을 때만 Start Task 를 켠다 (시뮬레이션)."""
        b = self.backend
        self.btn_connect.setEnabled(not b.connected())
        self.btn_start.setEnabled(b.mode == 'sim' and s['task_manager'] and not self.node.task_running)
