"""실물 E0509: 카메라 보정·집기 패널, 손짓(MediaPipe), 실물 음성 명령.

보정과 집기 순서 자체는 aris3_real_bridge/real_pick.py (PixelCalibration, PickSequence) 에 있다.
이 패널은 큐브·놓을 자리 선택(클릭·손짓), 확인창, 「조작 허용」 검사를 맡는다.
"""
import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

from aris3_gui.panels.common import HOME_DEG_REAL, nofocus
from aris3_real_bridge.real_pick import PickSequence, PixelCalibration
from aris3_gui.widgets import COLOR_KO

# 손짓 (실물, MediaPipe): 가리키고 이 시간 동안 이 반경 안에 멈추면 선택
GESTURE_HOLD_SEC = 1.0
GESTURE_HOLD_RADIUS_PX = 25
CUBE_SELECT_PX = 40          # 클릭: 이 거리 안의 검출 큐브를 고름
CUBE_SELECT_GESTURE_PX = 60  # 손짓: 손끝이 큐브를 가리지 않게 조금 옆을 가리켜도 고름
GESTURE_KO = {'point': '☝ 가리킴', 'fist': '✊ 주먹', 'open': '🖐 펼침'}


class RealPickMixin:
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

    # ---- 큐브·놓을 자리 고르기 (클릭·손짓)
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

    # ---- 카메라 보정 (① 픽셀 기억 -> ② 로봇 위치 기록)
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

    # ---- 집기 · 놓기
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

    # ---- 실물 음성 명령
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

    def _home_voice(self, heard):
        """실물: 음성 「집으로 돌아가」 -> 확인창 -> 홈 자세 (⌂ Home 버튼과 같은 이동·검사)."""
        if not self._arm_ok('홈 이동'):
            return
        hold = '\n※ 그리퍼가 큐브를 들고 있으면 함께 움직입니다.' if self.picker.holding is True else ''
        if self._ask('홈으로 돌아가기', f'{heard}\n\n홈 자세 {HOME_DEG_REAL}° 로 관절 이동합니다 '
                                     f'(속도 슬라이더 {self.sld_speed.value()} %, 최대 45°/s).{hold}\n\n'
                                     '팔이 크게 움직일 수 있습니다. 주변(손 포함)이 비었습니까?'):
            self.backend.home(self.sld_speed.value())
