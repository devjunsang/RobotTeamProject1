"""수동 조작: 조작 허용(2분 잠금), TCP·관절 조그, 손으로 움직이기(실물), 복구 후 홈(실물)."""
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
                             QSlider, QTabWidget, QVBoxLayout, QWidget)

from aris3_gui.panels.common import ARM_TIMEOUT_SEC, BAD_COLOR, HOME_DEG_REAL, WAIT_COLOR, nofocus


class JogMixin:
    """수동 조작 패널 mixin: 조작 허용(2분 잠금), TCP·관절 조그, 손으로 움직이기, 복구 후 홈."""
    def _jog_panel(self):
        """수동 조작 패널 화면 구성 (실물이면 손으로 움직이기·복구 후 홈 줄을 더한다)."""
        box = QGroupBox('수동 조작 (Manual Jog)')
        v = QVBoxLayout(box)
        self.chk_arm = nofocus(QCheckBox('조작 허용 (주변이 비었을 때만)'))
        self.chk_arm.toggled.connect(self._on_arm_toggled)
        self._arm_sent_at = 0.0         # 브리지에 조작 허용을 마지막으로 보낸 시각 (상태 맞추기 유예용)
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

    def _on_arm_toggled(self, on):
        """「조작 허용」 체크가 바뀜: 잠금 시계를 다시 세고, 실물이면 브리지 쪽 조작 허용도 같게 한다."""
        self.last_cmd_time = time.time()
        self.node.log('INFO', '조작 허용 켬' if on else '조작 잠금')
        self._arm_sent_at = time.monotonic()
        self.backend.set_armed(on)

    def _sync_arm_with_bridge(self):
        """실물: 브리지가 조작 허용을 스스로 잠갔으면(시간 초과·손으로 움직이기·접속 끊기) 화면 체크도 끈다.

        보낸 직후에는 상태 토픽이 아직 예전 값이라 1.5 초 동안은 맞추지 않는다 (켜자마자 꺼지지 않게).
        """
        st = self.backend.bridge.state()
        if st is None or time.monotonic() - self._arm_sent_at < 1.5:
            return
        if self.chk_arm.isChecked() and not st.armed:
            self.chk_arm.blockSignals(True)             # 브리지는 이미 잠겼으므로 다시 보내지 않는다
            self.chk_arm.setChecked(False)
            self.chk_arm.blockSignals(False)
            self.node.log('INFO', '실물 브리지가 조작 허용을 잠금 (시간 초과·손으로 움직이기·접속 끊기)')
        elif not self.chk_arm.isChecked() and st.armed:
            self._arm_sent_at = time.monotonic()        # 화면은 잠겼는데 브리지는 허용 -> 브리지도 잠근다
            self.backend.set_armed(False)

    def _jog_tcp(self, axis, sign):
        """TCP 를 축(axis 0 X / 1 Y / 2 Z) 방향으로 스텝만큼 (sign = +1 / -1). 조작 허용 검사 후 백엔드로."""
        if self._arm_ok('TCP 이동'):
            self.backend.jog_tcp(axis, sign * self.cmb_step.currentData(), self.sld_speed.value())

    def _jog_joint(self, i, sign):
        """관절 i 를 스텝만큼 (sign = +1 / -1). 조작 허용 검사 후 백엔드로."""
        if self._arm_ok('관절 이동'):
            self.backend.jog_joint(i, sign * self.cmb_step.currentData(), self.sld_speed.value())

    def _recover_home(self):
        """「복구 후 홈」: 조작 허용·집기 순서 확인 -> 확인창 -> 안전 정지 해제/서보 켜기 -> 자동 모드 -> 홈."""
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

    def _toggle_hand_guide(self, on):
        """「손으로 움직이기」 체크: 켤 때는 확인창. 실제 모드는 주기 갱신이 로봇 상태로 다시 맞춘다."""
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

    # ---- 주기 갱신: 조작 허용 시간 초과, 이동 가능 여부, 실물 로봇 상태
    def _refresh_jog(self):
        n, b = self.node, self.backend
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
            self.btn_recover.setEnabled(b.can_recover())
            self._sync_arm_with_bridge()
            manual, switching, name = b.hand_guide_state()
            self.chk_hand.blockSignals(True)
            self.chk_hand.setChecked(bool(manual))
            self.chk_hand.blockSignals(False)
            self.chk_hand.setEnabled(manual is not None and not switching)
            self.chk_hand.setText('손으로 움직이기 (수동 모드)' + (' - 바꾸는 중…' if switching else
                                  '' if manual is None else f' · 지금: {name}'))
