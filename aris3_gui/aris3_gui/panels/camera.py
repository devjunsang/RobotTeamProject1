"""카메라 Top View (검출 표시·클릭 선택·크게 보기)와 작업 공간 2D."""
from PyQt5.QtWidgets import QCheckBox, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout

from aris3_gui.panels.common import nofocus
from aris3_gui.widgets import CameraView, WorkspaceView


class CameraMixin:
    def _camera_panel(self):
        box = QGroupBox('카메라 뷰 (Camera View - Top View)  ·  더블클릭: 크게 보기')
        v = QVBoxLayout(box)
        top = QHBoxLayout()
        self.chk_detect = nofocus(QCheckBox('검출 표시 (Detection)'))
        self.chk_detect.setChecked(True)
        top.addStretch(1)
        top.addWidget(self.chk_detect)
        v.addLayout(top)
        self.camera = CameraView()
        self.camera.on_click = self._on_camera_click
        self.camera.on_double_click = self._toggle_camera_large
        v.addWidget(self.camera, 1)
        self.lbl_cam_info = QLabel('—')
        self.lbl_cam_info.setStyleSheet('color:#607d8b')
        v.addWidget(self.lbl_cam_info)
        return box

    def _workspace_panel(self):
        box = QGroupBox('작업 공간 (위에서 본 2D · RViz 3D 내장은 도전 과제)')
        v = QVBoxLayout(box)
        self.workspace = WorkspaceView()
        v.addWidget(self.workspace)
        return box

    def _toggle_camera_large(self):
        large = self.side_box.isVisible()
        self.side_box.setVisible(not large)
        self.bottom_box.setVisible(not large)

    def _refresh_camera(self):
        """카메라 다시 그리기 (33 ms 타이머, 상태 패널 갱신과 따로)."""
        n = self.node
        with n.lock:
            image, seq, objects, px = n.image, n.image_seq, list(n.objects), list(n.px_objects)
        if not n.px_rate.fresh(2.0):
            px = []                                 # 검출 노드가 꺼졌으면 오래된 상자를 지운다
        if image is None and not objects:
            return
        target = self.current_task['color'] if (n.task_running and self.current_task) else ''
        self.camera.show_detections = self.chk_detect.isChecked()
        self.camera.selected_id = self.sel_id
        self.camera.place_xy = self.sel_place
        self.camera.place_px = self.place_px if self.backend.mode == 'real' else None
        self.camera.set_frame(image, objects, n.camera_transform(), target, seq, px)

    def _refresh_views(self, snap, s):
        """100 ms 주기: 카메라 아래 정보 줄과 작업 공간 2D."""
        n, b = self.node, self.backend
        target = self.current_task['color'] if (n.task_running and self.current_task) else ''
        img = snap['image']
        self.lbl_cam_info.setText('—' if img is None else
                                  f'RGB ({img[0]} × {img[1]})  |  {s["camera_hz"]:.1f} Hz  |  ' +
                                  (f'색 검출(픽셀) {len(n.px_objects)}개' if s['px_detections'] else
                                   f'검출 {len(snap["objects"])}개') +
                                  ('  |  ※ 시뮬레이션 정답 위치 (aris3_vision 전 임시)' if b.mode == 'sim' else ''))
        pose = n.tcp_pose()
        self.workspace.set_state(snap['objects'], None if pose is None else pose[1], target, self.sel_place)
