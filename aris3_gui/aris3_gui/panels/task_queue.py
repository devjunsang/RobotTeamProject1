"""작업 큐 · 단계 진행, 카메라 화면 선택(이거/저기)으로 작업 만들기 (시뮬레이션 Task Manager)."""
import math

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QListWidget,
                             QMessageBox, QProgressBar, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout)

from aris3_gui.panels.common import BAD_COLOR, OK_COLOR, ZONES, nofocus
from aris3_gui.ros_io import STAGES
from aris3_gui.widgets import COLOR_KO

STAGE_KO = {'VALIDATE': '요청·장비 확인', 'OBSERVE': '관찰 자세', 'PERCEIVE': '물체 인식',
            'PLAN': '경로 계획', 'PICK': '집기', 'PLACE': '놓기', 'VERIFY': '결과 확인'}
TASK_COLORS = ['red', 'blue', 'yellow']


class TaskQueueMixin:
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
        # ---- 화면에서 고르기: 카메라 화면 클릭 (손짓 포인터도 같은 경로를 쓴다)
        pick = QHBoxLayout()
        self.lbl_sel = QLabel('화면 선택: 카메라에서 큐브 클릭(이거) · 빈 곳 클릭(저기)')
        self.lbl_sel.setStyleSheet('color:#00838f')
        self.btn_sel_run = nofocus(QPushButton('▶ 선택한 물체를 그 자리로'))
        self.btn_sel_run.clicked.connect(self._run_selection)
        b_clear = nofocus(QPushButton('선택 해제'))
        b_clear.clicked.connect(self._clear_selection)
        pick.addWidget(self.lbl_sel, 1)
        pick.addWidget(self.btn_sel_run)
        pick.addWidget(b_clear)
        v.addLayout(pick)
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

    # ---- 작업 큐
    def _queue_add(self, color=None, zone=None, dry=None, target_id='', place_xy=None):
        self.queue.append({
            'color': '' if target_id else (color or self.cmb_color.currentData()),
            'zone': '' if place_xy is not None else (zone or self.cmb_zone.currentData()),
            'dry': self.chk_dry.isChecked() if dry is None else dry,
            'target_id': target_id, 'place_xy': place_xy})
        self._render_queue()

    def _queue_del(self):
        row = self.lst_queue.currentRow()
        if 0 <= row < len(self.queue):
            del self.queue[row]
            self._render_queue()

    # ---- 화면에서 고르기 (카메라 클릭; 손짓 포인터도 이 함수들을 쓴다)
    def _on_camera_click(self, x, y):
        """클릭한 탁자 위 점 (손짓 포인터의 대용):
        1) 큐브 위(4 cm 안) -> 그 큐브를 고른다 (음성 「이거」)
        2) 탁자의 빈 곳     -> 놓을 자리로 정한다 (음성 「저기」). 순서는 상관없다."""
        objs = [o for o in self.node.snapshot()['objects'] if o.id]
        near = min(objs, key=lambda o: math.hypot(o.pose.pose.position.x - x, o.pose.pose.position.y - y),
                   default=None)
        if near is not None and math.hypot(near.pose.pose.position.x - x, near.pose.pose.position.y - y) < 0.04:
            self.sel_id = near.id
            self.node.log('INFO', f'선택(이거): 물체 {near.id}' +
                          ('' if self.sel_place else '  → 놓을 빈 자리를 클릭하세요'))
        else:
            self.sel_place = (x, y)
            self.node.log('INFO', f'선택(저기): 놓을 자리 ({x:.3f}, {y:.3f})' +
                          ('  → 「▶ 선택한 물체를 그 자리로」 또는 「이거 저기로 옮겨줘」' if self.sel_id
                           else '  → 옮길 큐브를 클릭하거나 「파란 큐브 저기로」 라고 말하세요'))
        self._update_selection_label()

    def _update_selection_label(self):
        place = '—' if self.sel_place is None else f'({self.sel_place[0]:.3f}, {self.sel_place[1]:.3f})'
        self.lbl_sel.setText(f'화면 선택: 물체 {self.sel_id or "—"}  →  놓을 자리 {place}')

    def _clear_selection(self):
        self.sel_id, self.sel_place = '', None
        self._update_selection_label()

    def _run_selection(self):
        if not self.sel_id or self.sel_place is None:
            return
        if self.backend.mode == 'real':
            self.node.log('WARN', '실물 모드에서는 화면 선택 작업을 실행하지 않음 (검증 전)')
            return
        self.queue.insert(0, {'color': '', 'zone': '', 'dry': self.chk_dry.isChecked(),
                              'target_id': self.sel_id, 'place_xy': self.sel_place})
        self._render_queue()
        self.queue_running = True
        self._next_task()
        self._clear_selection()

    def _render_queue(self):
        self.lst_queue.clear()
        for q in self.queue:
            what = q['target_id'] or f'{COLOR_KO.get(q["color"], q["color"])} 큐브'
            where = (f'({q["place_xy"][0]:.2f}, {q["place_xy"][1]:.2f})' if q['place_xy'] is not None
                     else f'구역 {q["zone"]}')
            self.lst_queue.addItem(f'{what} → {where}' + (' (계획만)' if q['dry'] else ''))

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
        q = self.queue.pop(0)
        self._render_queue()
        self.current_task = q
        if not self.node.send_task(q['color'], q['zone'], q['dry'], target_id=q['target_id'],
                                   place_xy=q['place_xy']):
            self.queue_running = False

    def _task_done_from_ros(self, ok):
        # ROS 스레드에서 불린다 -> Qt 타이머(_refresh)가 다음 작업을 보낸다
        if not ok:
            self.queue_running = False
            self.queue.clear()
        self._pending_next = True

    # ---- 주기 갱신
    def _refresh_queue(self, snap, s):
        n, b = self.node, self.backend
        self.btn_sel_run.setEnabled(bool(self.sel_id) and self.sel_place is not None and
                                    b.mode == 'sim' and s['task_manager'] and not n.task_running)
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
