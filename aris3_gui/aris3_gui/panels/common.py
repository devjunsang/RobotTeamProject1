"""패널들이 함께 쓰는 상수와 작은 도우미."""
from PyQt5.QtCore import Qt

ARM_TIMEOUT_SEC = 120
HOME_DEG_REAL = [0, 0, 90, 0, 90, 0]   # 표시용 (실제 값은 e0509_manual.robot_link.HOME_DEG)
ZONES = ['A', 'B', 'C']
OK_COLOR, BAD_COLOR, WAIT_COLOR = '#2e7d32', '#c62828', '#ef6c00'


def nofocus(w):
    w.setFocusPolicy(Qt.NoFocus)     # Space 가 버튼을 '누르지' 않게 (Space = 정지)
    return w


def dot(ok, text, warn=False):
    color = OK_COLOR if ok else (WAIT_COLOR if warn else BAD_COLOR)
    return f'<span style="color:{color}">●</span> {text}'
