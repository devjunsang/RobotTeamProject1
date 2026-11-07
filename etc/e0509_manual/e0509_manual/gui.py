"""E0509 manual controller window (PyQt5).

  ros2 run e0509_manual manual_controller                 # attach to /e0509_real (real robot bringup)
  ros2 run e0509_manual manual_controller --ns e0509_sim_test --z-min 0

Buttons move one step per click. Motion buttons work only after '조작 허용' is ticked, and the
lock re-engages after 2 minutes without a command. Space or Esc = stop, from anywhere in the window.
The gripper panel drives the RH-P12-RN(A) on the tool flange through DRL programs (see gripper_drl.py);
arm and gripper commands never run at the same time.
"""
import argparse
import os
import sys
import time

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QKeySequence
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QShortcut,
                             QSpinBox, QVBoxLayout, QWidget)

from e0509_manual import gripper_drl
from e0509_manual.robot_link import (JOINT_LIMITS, LIMIT_MARGIN, STATE_KO, STANDBY, start_link)

MONO = None  # set in ManualWindow (needs QApplication)
JOINT_ROLES = ['몸통', '어깨', '팔꿈치', '손목 1', '손목 2', '손목 3']
ARM_TIMEOUT_SEC = 120


def _nofocus(w):
    w.setFocusPolicy(Qt.NoFocus)     # Space must never "click" a motion button
    return w


class ManualWindow(QWidget):
    def __init__(self, link):
        super().__init__()
        self.link = link
        self.last_cmd_time = time.time()
        self._log_shown = 0
        self.motion_buttons = []
        self.grip_buttons = []          # need '조작 허용' (fingers move / release)
        self.grip_read_buttons = []     # read only, no '조작 허용' needed
        self._grip_auto_read = False
        self.setWindowTitle(f'E0509 수동 컨트롤러 — /{link.ns}')
        global MONO
        MONO = QFont('DejaVu Sans Mono', 11)
        MONO.setStyleHint(QFont.Monospace)
        self.setFont(QFont('NanumSquare', 10))

        root = QVBoxLayout(self)

        # ---- status bar
        top = QGridLayout()
        self.lbl_conn = QLabel()
        self.lbl_state = QLabel()
        self.lbl_state.setFont(QFont('NanumSquare', 12, QFont.Bold))
        self.lbl_reason = QLabel()
        self.lbl_error = QLabel()
        self.lbl_error.setStyleSheet('color: #b00020')
        top.addWidget(QLabel('연결'), 0, 0)
        top.addWidget(self.lbl_conn, 0, 1)
        top.addWidget(QLabel('로봇 상태'), 1, 0)
        top.addWidget(self.lbl_state, 1, 1)
        top.addWidget(self.lbl_reason, 2, 1)
        top.addWidget(self.lbl_error, 3, 1)
        top.setColumnStretch(1, 1)
        root.addLayout(top)

        self.chk_arm = _nofocus(QCheckBox('조작 허용  (로봇 주변이 비어 있고 비상정지를 누를 수 있을 때만 체크)'))
        self.chk_arm.toggled.connect(self._on_arm)
        root.addWidget(self.chk_arm)

        # ---- joint and cartesian panels
        panels = QHBoxLayout()
        panels.addWidget(self._joint_panel())
        panels.addWidget(self._cart_panel())
        root.addLayout(panels)
        root.addWidget(self._gripper_panel())

        # ---- stop
        self.btn_stop = _nofocus(QPushButton('■  정지  (Space / Esc)'))
        self.btn_stop.setMinimumHeight(64)
        self.btn_stop.setFont(QFont('NanumSquare', 16, QFont.Bold))
        self.btn_stop.setStyleSheet('QPushButton { background: #d32f2f; color: white; border-radius: 6px; }'
                                    'QPushButton:pressed { background: #9a0007; }')
        self.btn_stop.clicked.connect(self._stop)
        root.addWidget(self.btn_stop)
        for key in (Qt.Key_Space, Qt.Key_Escape):
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(self._stop)

        # ---- log
        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFocusPolicy(Qt.NoFocus)
        self.txt_log.setMaximumBlockCount(300)
        self.txt_log.setFixedHeight(120)
        root.addWidget(self.txt_log)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self._refresh()

    # ------------------------------------------------------------------ panels
    def _combo(self, values, unit, default):
        c = _nofocus(QComboBox())
        for v in values:
            c.addItem(f'{v:g} {unit}', v)
        c.setCurrentIndex(values.index(default))
        return c

    def _pm_buttons(self, on_minus, on_plus):
        bm = _nofocus(QPushButton('−'))
        bp = _nofocus(QPushButton('+'))
        for b in (bm, bp):
            b.setFixedWidth(48)
            b.setFont(QFont('NanumSquare', 13, QFont.Bold))
            self.motion_buttons.append(b)
        bm.clicked.connect(on_minus)
        bp.clicked.connect(on_plus)
        return bm, bp

    def _joint_panel(self):
        box = QGroupBox('관절 이동')
        g = QGridLayout(box)
        self.cmb_jstep = self._combo([1, 2, 5, 10], '°', 5)
        self.cmb_jvel = self._combo([5, 10, 20, 30], '°/s', 10)
        g.addWidget(QLabel('한 번에'), 0, 0)
        g.addWidget(self.cmb_jstep, 0, 1)
        g.addWidget(QLabel('속도'), 0, 2)
        g.addWidget(self.cmb_jvel, 0, 3)
        self.lbl_joint = []
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            name = QLabel(f'J{i + 1} {JOINT_ROLES[i]}')
            name.setToolTip(f'허용 범위 {lo + LIMIT_MARGIN:.0f} ~ {hi - LIMIT_MARGIN:.0f}°')
            val = QLabel('—')
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFont(MONO)
            val.setMinimumWidth(90)
            bm, bp = self._pm_buttons(lambda _=False, i=i: self._jog_joint(i, -1),
                                      lambda _=False, i=i: self._jog_joint(i, +1))
            g.addWidget(name, i + 1, 0)
            g.addWidget(val, i + 1, 1)
            g.addWidget(bm, i + 1, 2)
            g.addWidget(bp, i + 1, 3)
            self.lbl_joint.append(val)
        return box

    def _cart_panel(self):
        box = QGroupBox('좌표 이동  (로봇 받침 기준, 방향 유지)')
        g = QGridLayout(box)
        self.cmb_cstep = self._combo([1, 5, 10, 20], 'mm', 10)
        self.cmb_cvel = self._combo([10, 20, 50], 'mm/s', 20)
        g.addWidget(QLabel('한 번에'), 0, 0)
        g.addWidget(self.cmb_cstep, 0, 1)
        g.addWidget(QLabel('속도'), 0, 2)
        g.addWidget(self.cmb_cvel, 0, 3)
        self.lbl_pos = []
        names = ['X  앞뒤', 'Y  좌우', 'Z  위아래', 'RX', 'RY', 'RZ']
        for i, n in enumerate(names):
            val = QLabel('—')
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFont(MONO)
            val.setMinimumWidth(110)
            g.addWidget(QLabel(n), i + 1, 0)
            g.addWidget(val, i + 1, 1)
            if i < 3:
                bm, bp = self._pm_buttons(lambda _=False, a=i: self._jog_cart(a, -1),
                                          lambda _=False, a=i: self._jog_cart(a, +1))
                g.addWidget(bm, i + 1, 2)
                g.addWidget(bp, i + 1, 3)
            else:
                g.addWidget(QLabel('표시만'), i + 1, 2, 1, 2)
            self.lbl_pos.append(val)
        g.addWidget(QLabel(f'최저 높이 Z ≥ {self.link.z_min_mm:.0f} mm'), 7, 0, 1, 4)
        return box

    def _gripper_panel(self):
        box = QGroupBox('그리퍼  RH-P12-RN(A)  (손목 RS-485, Modbus 57600)  —  0 = 완전히 열림, 740 = 닫힘')
        g = QGridLayout(box)
        self.lbl_grip = QLabel('—  (상태를 아직 읽지 않음)')
        self.lbl_grip.setFont(MONO)
        self.lbl_grip_info = QLabel('')
        g.addWidget(self.lbl_grip, 0, 0, 1, 8)
        g.addWidget(self.lbl_grip_info, 1, 0, 1, 8)

        self.cmb_gcur = _nofocus(QComboBox())
        for name, ma in [('약하게 (150 mA)', 150), ('보통 (300 mA)', 300), ('세게 (500 mA)', 500)]:
            self.cmb_gcur.addItem(name, ma)
        self.cmb_gvel = _nofocus(QComboBox())
        for name, v in [('느리게', 600), ('보통', 1500), ('빠르게', 2970)]:
            self.cmb_gvel.addItem(name, v)
        self.spn_gpos = QSpinBox()
        self.spn_gpos.setRange(gripper_drl.POS_OPEN, gripper_drl.POS_MAX)
        self.spn_gpos.setSingleStep(20)
        self.spn_gpos.setValue(370)
        self.spn_gpos.setSuffix('  (0~740)')
        g.addWidget(QLabel('잡는 힘'), 2, 0)
        g.addWidget(self.cmb_gcur, 2, 1)
        g.addWidget(QLabel('속도'), 2, 2)
        g.addWidget(self.cmb_gvel, 2, 3)
        g.addWidget(QLabel('목표 위치'), 2, 4)
        g.addWidget(self.spn_gpos, 2, 5)

        def btn(text, slot, needs_arm=True):
            b = _nofocus(QPushButton(text))
            b.setMinimumHeight(36)
            b.clicked.connect(slot)
            (self.grip_buttons if needs_arm else self.grip_read_buttons).append(b)
            return b
        g.addWidget(btn('열기', lambda: self._grip_move(gripper_drl.POS_OPEN, '그리퍼 열기')), 3, 0)
        g.addWidget(btn('닫기 (잡기)', lambda: self._grip_move(gripper_drl.POS_CLOSE, '그리퍼 닫기')), 3, 1)
        g.addWidget(btn('목표 위치로', lambda: self._grip_move(self.spn_gpos.value(),
                                                          f'그리퍼 → {self.spn_gpos.value()}')), 3, 2)
        g.addWidget(btn('힘 풀기 (토크 끄기)', self._grip_release), 3, 3)
        g.addWidget(btn('상태 읽기', self._grip_status, needs_arm=False), 3, 5)
        return box

    # ------------------------------------------------------------------ actions
    def _grip_move(self, pos, label):
        if not self.chk_arm.isChecked():
            return
        self.last_cmd_time = time.time()
        self.link.gripper_command('move', pos=pos, cur=self.cmb_gcur.currentData(),
                                  vel=self.cmb_gvel.currentData(),
                                  label=f'{label} (힘 {self.cmb_gcur.currentData()} mA)')

    def _grip_release(self):
        if not self.chk_arm.isChecked():
            return
        self.last_cmd_time = time.time()
        self.link.gripper_command('torque_off')

    def _grip_status(self):
        self.link.gripper_command('status')

    def _on_arm(self, on):
        self.last_cmd_time = time.time()
        self.link.log('조작 허용 켬' if on else '조작 잠금')

    def _jog_joint(self, i, sign):
        if not self.chk_arm.isChecked():
            return
        self.last_cmd_time = time.time()
        self.link.jog_joint(i, sign * self.cmb_jstep.currentData(), self.cmb_jvel.currentData())

    def _jog_cart(self, axis, sign):
        if not self.chk_arm.isChecked():
            return
        self.last_cmd_time = time.time()
        self.link.jog_cart(axis, sign * self.cmb_cstep.currentData(), self.cmb_cvel.currentData())

    def _stop(self):
        self.link.stop()

    # ------------------------------------------------------------------ periodic refresh
    def _refresh(self):
        L = self.link
        connected = L.cli_movej.service_is_ready()
        if L.joints_fresh():
            self.lbl_conn.setText(f'/{L.ns}  ●  관절 각도 수신 중 ({L.js_rate:.0f} Hz)')
            self.lbl_conn.setStyleSheet('color: #2e7d32')
        else:
            self.lbl_conn.setText(f'/{L.ns}  ○  ' + ('관절 각도 안 들어옴' if connected else '연결 안 됨 (bringup 실행 필요)'))
            self.lbl_conn.setStyleSheet('color: #b00020')

        sn = L.state_name()
        self.lbl_state.setText(f'{STATE_KO.get(sn, sn)}  ({sn})')
        self.lbl_state.setStyleSheet('color: #2e7d32' if L.robot_state == STANDBY else 'color: #e65100')
        self.lbl_error.setText(L.last_error)

        if self.chk_arm.isChecked() and time.time() - self.last_cmd_time > ARM_TIMEOUT_SEC:
            self.chk_arm.setChecked(False)
            L.log(f'{ARM_TIMEOUT_SEC}초 동안 명령이 없어 조작을 잠갔습니다')

        reason = L.why_not_ready()
        if not self.chk_arm.isChecked():
            reason = reason or '조작 허용이 꺼져 있음'
        self.lbl_reason.setText('' if reason is None else f'이동 불가: {reason}')
        for b in self.motion_buttons + self.grip_buttons:
            b.setEnabled(reason is None)
        read_reason = L.why_not_ready()
        for b in self.grip_read_buttons:
            b.setEnabled(read_reason is None)
        if not self._grip_auto_read and read_reason is None:
            self._grip_auto_read = True          # read the gripper once as soon as the robot is ready
            L.gripper_command('status')
        self._refresh_gripper()

        if L.joints_deg is not None:
            for lbl, v in zip(self.lbl_joint, L.joints_deg):
                lbl.setText(f'{v:8.2f} °')
        if L.posx is not None:
            for i, (lbl, v) in enumerate(zip(self.lbl_pos, L.posx)):
                lbl.setText(f'{v:9.1f} ' + ('mm' if i < 3 else '°'))

        seq, lines = L.log_seq, list(L.log_lines)
        new = min(seq - self._log_shown, len(lines))
        if new > 0:
            for line in lines[-new:]:
                self.txt_log.appendPlainText(line)
        self._log_shown = seq

    def _refresh_gripper(self):
        L = self.link
        d = L.gripper
        if d:
            pct = 100.0 * d['position'] / gripper_drl.POS_CLOSE
            self.lbl_grip.setText(
                f'위치 {d["position"]:4d}  ({pct:5.1f}% 닫힘)   전류 {d["current_mA"]:4d} mA   '
                f'토크 {"켜짐" if d["torque"] else "꺼짐"}   {"움직이는 중" if d["moving"] else "정지"}   '
                f'{d["temperature_C"]}°C  {d["voltage_V"]:.1f} V' +
                (f'   하드웨어 에러 {d["hw_error"]}' if d['hw_error'] else ''))
        age = time.time() - L.gripper_stamp if L.gripper_stamp else None
        if L.gripper_running:
            info, color = '그리퍼 명령 실행 중… (끝나면 값이 갱신됩니다)', '#1565c0'
        elif L.gripper_ok is False:
            info, color = '마지막 그리퍼 명령 실패 — 아래 기록을 확인하세요', '#b00020'
        elif age is not None:
            info, color = f'{age:.0f}초 전에 읽은 값 (그리퍼 값은 자동으로 갱신되지 않습니다 — 「상태 읽기」)', '#555'
        else:
            info, color = '', '#555'
        self.lbl_grip_info.setText(info)
        self.lbl_grip_info.setStyleSheet(f'color: {color}')

    def closeEvent(self, ev):
        self.timer.stop()
        super().closeEvent(ev)


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    p = argparse.ArgumentParser()
    p.add_argument('--ns', default='e0509_real', help='robot namespace used by dsr_bringup2 (name:=...)')
    p.add_argument('--z-min', type=float, default=50.0, help='lowest TCP height allowed [mm, base frame]')
    a = p.parse_args(argv[1:])

    # Same display backend as RViz (X11 via Xwayland); create Qt before any ROS thread starts.
    if os.environ.get('DISPLAY') and 'QT_QPA_PLATFORM' not in os.environ:
        os.environ['QT_QPA_PLATFORM'] = 'xcb'
    app = QApplication(sys.argv[:1])
    link, ex = start_link(a.ns, a.z_min)
    w = ManualWindow(link)
    w.resize(980, 820)
    w.show()
    code = app.exec_()
    ex.shutdown()
    link.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
