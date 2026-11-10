"""음성 명령: 누르고 말하기 / 문장 입력 -> aris3_voice 해석 -> 실행.

시뮬레이션은 작업 큐(Task Manager)로, 실물은 실물 집기 패널(_real_voice)로 이어진다.
"""
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout

from aris3_gui.panels.common import BAD_COLOR, OK_COLOR, ZONES, nofocus
from aris3_gui.widgets import COLOR_KO


class VoiceMixin:
    def _voice_panel(self):
        box = QGroupBox('음성 명령 (Voice Command)')
        v = QVBoxLayout(box)
        self.lbl_voice_state = QLabel('—')
        v.addWidget(self.lbl_voice_state)
        # 음성 명령 모드 ON/OFF (2026-11-10): 켜 두면 계속 듣고 「아리스 ...」 로 시작하는 말만 명령으로 받는다
        self.btn_talk = nofocus(QPushButton('🎙 음성 명령 모드: OFF'))
        self.btn_talk.setCheckable(True)
        self.btn_talk.setMinimumHeight(44)
        self.btn_talk.setToolTip('켜면 마이크가 계속 듣습니다. 「아리스 빨간 도형 잡아」 처럼 앞에 「아리스」 를 붙여 말하세요.\n'
                                 '「아리스」 로 시작하지 않는 말은 무시합니다.')
        self.btn_talk.clicked.connect(self._talk)
        v.addWidget(self.btn_talk)
        row = QHBoxLayout()
        self.txt_cmd = QLineEdit()
        self.txt_cmd.setPlaceholderText('또는 문장 입력 (예: 아리스 빨간 도형 잡아 / 이거 저기로 옮겨줘)')
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

    def _talk(self, on):
        """음성 명령 모드 버튼: 켜기/끄기를 음성 노드에 보낸다 (실제 상태는 주기 갱신이 노드 상태로 다시 맞춤)."""
        self.node.voice_active(on)
        self.node.log('INFO', '음성 명령 모드 켬 - 「아리스 ...」 로 말하세요' if on else '음성 명령 모드 끔')

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
        if v.action == 'stop':                      # 「아리스 멈춰」: ■ Stop 과 같음 (2026-11-11)
            self.node.log('WARN', f'음성 「{v.text}」 -> 정지')
            self._stop()
            return
        if v.action in ('resume', 'power_on', 'disconnect'):
            # 「계속해」「가동」「연결해제」 는 ARIS(aris3_gui_cpp) 에서 지원. 여기서는 집기로 잘못 읽히지 않게 막는다
            self.node.log('WARN', f'음성 「{v.text}」: 이 GUI 는 지원하지 않음 - ARIS(aris3_gui_cpp)를 쓰세요')
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

    # ---- 주기 갱신
    def _refresh_voice(self, snap):
        n, b = self.node, self.backend
        vs = snap['voice_state']
        self.lbl_voice_state.setText({'ready': '⚪ 준비 — 음성 명령 모드를 켜세요',
                                      'listening': '🟢 듣는 중 — 「아리스 ...」 로 말하세요',
                                      'recording': '🔴 녹음 중...', 'recognizing': '⏳ 인식 중...',
                                      'loading': '⏳ Whisper 모델 로딩 중'}.get(vs, f'⚪ 음성 노드 없음 {vs}'))
        # 버튼은 노드의 실제 상태를 따른다 (듣는 중·인식 중 = ON)
        on = vs in ('listening', 'recognizing')
        self.btn_talk.blockSignals(True)
        self.btn_talk.setChecked(on)
        self.btn_talk.blockSignals(False)
        self.btn_talk.setText(f'🎙 음성 명령 모드: {"ON — 듣는 중" if on else "OFF"}')
        self.btn_talk.setEnabled(vs in ('ready', 'listening', 'recognizing'))
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
