"""로그 (GUI·ROS 노드 로그, 'ARIS3 로그만' 거르기)."""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QCheckBox, QGroupBox, QHBoxLayout, QPlainTextEdit, QPushButton, QVBoxLayout

from aris3_gui.panels.common import nofocus
from aris3_gui.ros_io import LOG_NODES

# 'ARIS3 로그만' 에서 숨길, 매 계획마다 나오는 무해한 MoveIt 경고
NOISE = ('planning volume was not specified', "did not fill out the 'planner_id'",
         'No 3D sensor plugin', 'Resolution not specified for Octomap')


class LogMixin:
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

    def _refresh_log(self, snap):
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
