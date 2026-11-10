"""왼쪽 열: 시스템 상태 · 로봇 상태 · 그리퍼 상태 · 카메라 정보."""
import math

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QProgressBar, QPushButton, QVBoxLayout, QWidget

from aris3_gui.panels.common import dot, nofocus


class StatusMixin:
    def _left_column(self):
        col = QVBoxLayout()
        sys_box = QGroupBox('시스템 상태 (System Status)')
        g = QGridLayout(sys_box)
        self.sys_labels = {}
        for i, (key, name) in enumerate([('conn', '연결'), ('gazebo', '로봇/시뮬레이터'), ('camera', '카메라'),
                                         ('detect', '물체 인식'), ('gripper', '그리퍼'), ('moveit', 'MoveIt 2'),
                                         ('tm', 'Task Manager'), ('voice', '음성 인식'),
                                         ('gesture', '손짓 인식')]):
            g.addWidget(QLabel(name), i, 0)
            lbl = QLabel('—')
            lbl.setTextFormat(Qt.RichText)
            g.addWidget(lbl, i, 1)
            self.sys_labels[key] = lbl
        col.addWidget(sys_box)

        rob = QGroupBox('로봇 상태 (Robot State)')
        g = QGridLayout(rob)
        # 모드 글자: 실물 경로를 가상 컨트롤러로 시험 중이면 「가상」 을 분명히 (실물로 착각하지 않게)
        mode_text = ('MoveIt 2 (시뮬레이션)' if self.backend.mode == 'sim' else
                     '두산 가상 컨트롤러 (실물 경로 시뮬레이션)' if '가상' in self.backend.title else '두산 (실물)')
        self.lbl_mode = QLabel(f'<b>{mode_text}</b>')
        self.lbl_task_state = QLabel('—')
        g.addWidget(QLabel('모드'), 0, 0)
        g.addWidget(self.lbl_mode, 0, 1, 1, 3)
        g.addWidget(QLabel('작업 상태'), 1, 0)
        g.addWidget(self.lbl_task_state, 1, 1, 1, 3)
        mono = QFont('DejaVu Sans Mono', 9)
        self.lbl_joints = QLabel('—')
        self.lbl_joints.setFont(mono)
        self.lbl_tcp = QLabel('—')
        self.lbl_tcp.setFont(mono)
        g.addWidget(QLabel('관절 [deg]'), 2, 0, 1, 4)
        g.addWidget(self.lbl_joints, 3, 0, 1, 4)
        g.addWidget(QLabel('손끝 TCP [m, deg] (world)'), 4, 0, 1, 4)
        g.addWidget(self.lbl_tcp, 5, 0, 1, 4)
        col.addWidget(rob)

        grip = QGroupBox('그리퍼 상태 (Gripper State)')
        g = QGridLayout(grip)
        self.lbl_grip = QLabel('—')
        self.bar_grip = QProgressBar()
        self.bar_grip.setRange(0, 100)
        self.bar_grip.setFormat('닫힘 %p%')
        self.btn_gopen = nofocus(QPushButton('열기'))
        self.btn_gclose = nofocus(QPushButton('닫기 (잡기)'))
        self.btn_gopen.clicked.connect(lambda: self._gripper(False))
        self.btn_gclose.clicked.connect(lambda: self._gripper(True))
        g.addWidget(self.lbl_grip, 0, 0, 1, 2)
        g.addWidget(self.bar_grip, 1, 0, 1, 2)
        g.addWidget(self.btn_gopen, 2, 0)
        g.addWidget(self.btn_gclose, 2, 1)
        col.addWidget(grip)

        cal = QGroupBox('카메라 정보 (Camera)')
        g = QGridLayout(cal)
        self.lbl_cal = QLabel('—')
        self.lbl_cal.setWordWrap(True)
        g.addWidget(self.lbl_cal, 0, 0)
        col.addWidget(cal)
        col.addStretch(1)
        w = QWidget()
        w.setLayout(col)
        w.setFixedWidth(330)
        outer = QVBoxLayout()
        outer.addWidget(w)
        return outer

    def _gripper(self, close):
        if self._arm_ok('그리퍼'):
            self.backend.gripper(close)

    # ---- 주기 갱신
    def _refresh_system(self, snap, s):
        b = self.backend
        L = self.sys_labels
        L['conn'].setText(dot(b.connected(), b.describe(), warn=True))
        L['gazebo'].setText(dot(s['joint_states'], f'관절 상태 {s["js_hz"]:.0f} Hz' if s['joint_states']
                                else '관절 상태 없음'))
        L['camera'].setText(dot(s['camera_hz'] > 1, f'{s["camera_hz"]:.1f} Hz' if s['camera_hz'] else '영상 없음'))
        L['detect'].setText(dot(s['detections'], f'{len(snap["objects"])}개' if s['detections'] else '결과 없음'))
        L['gripper'].setText(dot(*b.gripper_status()))
        L['moveit'].setText(dot(s['moveit'], '준비' if s['moveit'] else '없음'))
        L['tm'].setText(dot(s['task_manager'], '준비' if s['task_manager'] else '없음'))
        vs = snap['voice_state']
        L['voice'].setText(dot(s['voice'] and vs in ('ready', 'recording', 'recognizing'),
                               {'ready': '준비', 'recording': '녹음 중', 'recognizing': '인식 중',
                                'loading': '모델 로딩 중'}.get(vs, vs or '없음'), warn=vs == 'loading'))

        gs = snap['gesture_state']
        L['gesture'].setText(dot(s['gesture'] and gs == 'ready', {'ready': '준비', 'loading': 'MediaPipe 로딩 중'}.get(
            gs, gs or '없음'), warn=gs == 'loading'))

    def _refresh_robot(self, snap):
        b, n = self.backend, self.node
        ts = snap['task_status']
        self.lbl_task_state.setText('—' if ts is None else
                                    f'{["대기", "작업 중", "오류"][ts.state] if ts.state < 3 else ts.state} '
                                    f'{ts.stage} {ts.task_id}')
        j = b.joints_deg()
        self.lbl_joints.setText('—' if j is None else
                                '  '.join(f'J{i + 1} {v:7.1f}' + ('\n' if i == 2 else '') for i, v in enumerate(j)))
        tcp = b.tcp_text()
        if tcp is None:
            self.lbl_tcp.setText('—')
        else:
            (x, y, z), (r, p, yw) = tcp
            self.lbl_tcp.setText(f'X {x:6.3f}  Y {y:6.3f}  Z {z:6.3f}\n'
                                 f'R {math.degrees(r):6.1f}  P {math.degrees(p):6.1f}  Y {math.degrees(yw):6.1f}')
        gp = b.gripper_position()
        if gp is None:
            self.lbl_grip.setText('—')
        else:
            self.bar_grip.setValue(int(gp * 100))
            state = '열림' if gp < 0.05 else ('닫힘 (빈손)' if gp > 0.9 else '닫힘 (물체를 잡고 있음)')
            self.lbl_grip.setText(f'{state}' + (f'  ·  {n.gripper_msg}' if n.gripper_msg else ''))

    def _refresh_camera_info(self, cam_tf):
        self.lbl_cal.setText(('카메라 TF: ' + ('있음' if cam_tf else '없음')) +
                             '\n시뮬레이션: 설치 위치 고정값 (탁자 위 2.0 m, 화각 80°) - 보정 없음'
                             if self.backend.mode == 'sim' else
                             '실물 카메라: 픽셀 -> 로봇 좌표 보정 사용 (아래 「실물 집기」 패널, 브리지가 계산)')
