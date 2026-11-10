"""그림을 그리는 위젯: 카메라 Top View(검출 표시) / 작업 공간 2D 지도."""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget

COLORS = {'red': QColor('#d32f2f'), 'blue': QColor('#1565c0'), 'yellow': QColor('#f9a825'),
          'green': QColor('#2e7d32')}
COLOR_KO = {'red': '빨강', 'blue': '파랑', 'yellow': '노랑', 'green': '초록'}


def quat_apply(q, v):
    """쿼터니언 q(x, y, z, w) 로 벡터 v 회전."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class CameraView(QWidget):
    """카메라 영상 + 검출 물체 표시.

    물체 위치(world)를 카메라 광학 좌표(TF)로 옮긴 뒤 핀홀 모델로 화면 좌표를 구한다.
    초점 거리는 수평 화각(hfov, 카메라 SDF 값)과 영상 폭으로 계산한다: fx = (w/2) / tan(hfov/2).
    """

    def __init__(self, hfov=1.4):
        super().__init__()
        self.hfov = hfov
        self.on_click = None        # 콜백(world_x, world_y): 클릭한 화면 점을 탁자 높이 평면에 투영한 좌표
        self.on_double_click = None  # 콜백(): 더블클릭 - 크게 보기 전환
        self.on_pixel_click = None  # 콜백(u, v): 카메라 TF 가 없을 때(실물, 보정 전) 클릭한 영상 픽셀
        self.marker_px = None       # 영상 픽셀 표시 (실물 보정에서 기억한 큐브 위치)
        self.image_seq = -1         # 마지막으로 변환한 영상 번호
        self.plane_z = 0.775        # 투영할 평면 높이 (탁자 0.75 + 큐브 반 높이)
        self.selected_id = ''       # 클릭/손짓으로 고른 물체
        self.place_xy = None        # 클릭/손짓으로 고른 놓을 자리 (world)
        self.pointer = None         # 손짓 포인터 (영상 픽셀 u, v) - MediaPipe 검지 끝
        self.pointer_progress = 0.0  # 가리키고 멈춘 시간 / 선택에 필요한 시간 (0~1, 고리로 표시)
        self.pointer_text = ''      # 포인터 옆 글 (손짓 이름)
        self.place_px = None        # 실물: 고른 놓을 자리 (영상 픽셀 u, v)
        self.qimage = None
        self.objects = []
        self.px_objects = []        # 픽셀 좌표 검출 (실물 카메라, 보정 전): pose.position.x, y = (u, v)
        self.cam_tf = None          # world -> 광학 좌표 TransformStamped
        self.show_detections = True
        self.highlight = ''         # 지금 작업 중인 색
        self.setMinimumSize(480, 360)
        # ---- 줌·이동 (2026-11-09). 휠: 커서 기준 확대/축소, 오른쪽 끌기: 이동, 가운데 클릭: 원래대로
        self.zoom = 1.0             # 1 = 화면에 영상 전체가 들어감
        self.center = None          # 화면 가운데에 오는 영상 픽셀 (u, v). None = 영상 가운데
        self._drag = None           # 오른쪽 끌기 시작점 (마우스 x, y, 시작 center)

    def set_frame(self, image, objects, cam_tf, highlight='', seq=None, px_objects=()):
        self.px_objects = list(px_objects)
        if image is not None and (seq is None or seq != self.image_seq):
            self.image_seq = -1 if seq is None else seq
            w, h, enc, data = image
            fmt = {'rgb8': QImage.Format_RGB888, 'bgr8': QImage.Format_BGR888}.get(enc)
            if fmt is not None:
                if self.qimage is None or (self.qimage.width(), self.qimage.height()) != (w, h):
                    self.reset_zoom()               # 해상도가 바뀌면 줌을 처음으로
                self.qimage = QImage(data, w, h, w * 3, fmt).copy()
        self.objects = objects
        self.cam_tf = cam_tf
        self.highlight = highlight
        self.update()

    def _view_rect(self):
        """영상 -> 화면 변환 (ox, oy, scale): 화면 x = ox + u * scale. 그리기·클릭이 모두 이것을 쓴다."""
        img = self.qimage
        scale = min(self.width() / img.width(), self.height() / img.height()) * self.zoom
        cu, cv = self._clamped_center(scale)
        return self.width() / 2 - cu * scale, self.height() / 2 - cv * scale, scale

    def _clamped_center(self, scale):
        """확대했을 때 영상 밖(빈 곳)이 보이지 않게 가운데 점을 제한한다."""
        iw, ih = self.qimage.width(), self.qimage.height()
        cu, cv = self.center if self.center is not None else (iw / 2, ih / 2)

        def clamp(c, size, view):
            half = view / (2 * scale)               # 화면 절반이 영상 픽셀로 몇 개인가
            return size / 2 if 2 * half >= size else min(max(c, half), size - half)
        return clamp(cu, iw, self.width()), clamp(cv, ih, self.height())

    def reset_zoom(self):
        self.zoom, self.center = 1.0, None
        self.update()

    def wheelEvent(self, ev):
        if self.qimage is None:
            return
        ox, oy, scale = self._view_rect()
        u, v = (ev.x() - ox) / scale, (ev.y() - oy) / scale          # 커서 아래 영상 픽셀
        step = ev.angleDelta().y() / 120                               # 휠 한 칸 = 120
        self.zoom = min(max(self.zoom * 1.25 ** step, 1.0), 8.0)
        new_scale = min(self.width() / self.qimage.width(), self.height() / self.qimage.height()) * self.zoom
        # 커서 아래 픽셀이 확대 뒤에도 커서 아래에 남도록 가운데 점을 옮긴다
        self.center = (u - (ev.x() - self.width() / 2) / new_scale,
                       v - (ev.y() - self.height() / 2) / new_scale)
        self.center = self._clamped_center(new_scale)
        self.update()

    def mouseMoveEvent(self, ev):
        if self._drag is None or self.qimage is None:
            return
        x0, y0, (cu, cv) = self._drag
        _, _, scale = self._view_rect()
        self.center = (cu - (ev.x() - x0) / scale, cv - (ev.y() - y0) / scale)   # 끈 만큼 반대로
        self.center = self._clamped_center(scale)
        self.update()

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.RightButton:
            self._drag = None

    def unproject(self, u, v):
        """영상 픽셀 (u, v) -> plane_z 평면 위 world (x, y) 또는 None (project 의 반대)."""
        if self.cam_tf is None or self.qimage is None:
            return None
        t = self.cam_tf.transform
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        qi = (-q[0], -q[1], -q[2], q[3])                  # world <- 카메라 회전
        w, h = self.qimage.width(), self.qimage.height()
        f = (w / 2) / math.tan(self.hfov / 2)
        d = quat_apply(qi, ((u - w / 2) / f, (v - h / 2) / f, 1.0))
        o = quat_apply(qi, (-t.translation.x, -t.translation.y, -t.translation.z))   # 카메라 중심 (world)
        if abs(d[2]) < 1e-6:
            return None
        s = (self.plane_z - o[2]) / d[2]
        if s <= 0:
            return None
        return o[0] + s * d[0], o[1] + s * d[1]

    def mouseDoubleClickEvent(self, ev):
        if ev.button() == Qt.LeftButton and self.on_double_click is not None:
            self.on_double_click()

    def mousePressEvent(self, ev):
        if self.qimage is None:
            return
        if ev.button() == Qt.MiddleButton:
            self.reset_zoom()
            return
        if ev.button() == Qt.RightButton:
            _, _, scale = self._view_rect()
            self._drag = (ev.x(), ev.y(), self._clamped_center(scale))
            return
        if self.on_click is None or ev.button() != Qt.LeftButton:
            return
        ox, oy, scale = self._view_rect()
        u, v = (ev.x() - ox) / scale, (ev.y() - oy) / scale
        if 0 <= u < self.qimage.width() and 0 <= v < self.qimage.height():
            if self.cam_tf is None and self.on_pixel_click is not None:
                self.on_pixel_click(u, v)
                return
            xy = self.unproject(u, v)
            if xy is not None:
                self.on_click(*xy)

    def project(self, p):
        """world 점 -> 영상 픽셀 (u, v, 깊이) 또는 None."""
        if self.cam_tf is None or self.qimage is None:
            return None
        t = self.cam_tf.transform
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        c = quat_apply(q, p)
        c = (c[0] + t.translation.x, c[1] + t.translation.y, c[2] + t.translation.z)
        if c[2] <= 0.01:
            return None
        w, h = self.qimage.width(), self.qimage.height()
        f = (w / 2) / math.tan(self.hfov / 2)
        return w / 2 + f * c[0] / c[2], h / 2 + f * c[1] / c[2], c[2], f

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#263238'))
        if self.qimage is None:
            p.setPen(QColor('#cfd8dc'))
            p.setFont(QFont('NanumSquare', 12))
            p.drawText(self.rect(), Qt.AlignCenter, '카메라 영상 없음\n(/aris3/overhead_camera/image)')
            return
        img = self.qimage
        ox, oy, scale = self._view_rect()
        p.drawImage(QRectF(ox, oy, img.width() * scale, img.height() * scale), img)
        if self.zoom > 1.0:
            p.setPen(QColor('white'))
            p.setFont(QFont('NanumSquare', 10, QFont.Bold))
            p.fillRect(QRectF(6, 6, 250, 20), QColor(0, 0, 0, 140))
            p.drawText(QRectF(12, 6, 244, 20), Qt.AlignVCenter,
                       f'×{self.zoom:.1f}  ·  오른쪽 끌기: 이동  ·  휠 클릭: 원래대로')
        if self.place_xy is not None:
            r = self.project((self.place_xy[0], self.place_xy[1], self.plane_z))
            if r is not None:
                draw_place_marker(p, ox + r[0] * scale, oy + r[1] * scale)
        if self.marker_px is not None:
            draw_place_marker(p, ox + self.marker_px[0] * scale, oy + self.marker_px[1] * scale, r=16,
                              color='#ffea00')
        if self.place_px is not None:
            draw_place_marker(p, ox + self.place_px[0] * scale, oy + self.place_px[1] * scale)
        if self.pointer is not None:
            px, py = ox + self.pointer[0] * scale, oy + self.pointer[1] * scale
            p.setPen(QPen(QColor('#ff4081'), 3))
            p.setBrush(QColor(255, 64, 129, 90))
            p.drawEllipse(QPointF(px, py), 10, 10)
            if self.pointer_progress > 0:           # 멈춘 시간만큼 고리가 찬다 -> 다 차면 선택
                p.setPen(QPen(QColor('#00e5ff'), 4))
                p.setBrush(Qt.NoBrush)
                p.drawArc(QRectF(px - 18, py - 18, 36, 36), 90 * 16, int(-360 * 16 * self.pointer_progress))
            if self.pointer_text:
                p.setPen(QColor('white'))
                p.setFont(QFont('NanumSquare', 10, QFont.Bold))
                p.fillRect(QRectF(px + 20, py - 10, 80, 20), QColor(0, 0, 0, 150))
                p.drawText(QRectF(px + 24, py - 10, 76, 20), Qt.AlignVCenter, self.pointer_text)
        if not self.show_detections:
            return
        p.setFont(QFont('NanumSquare', 10, QFont.Bold))
        for o in self.px_objects:                   # 영상 픽셀 위치를 그대로 그린다 (TF 필요 없음)
            u, v = o.pose.pose.position.x, o.pose.pose.position.y
            half = (math.sqrt(max(o.size_px, 1.0)) / 2 + 6) * scale
            self._draw_box(p, ox + u * scale, oy + v * scale, half, o, o.color == self.highlight)
        for o in self.objects:
            pos = o.pose.pose.position
            r = self.project((pos.x, pos.y, pos.z))
            if r is None:
                continue
            u, v, depth, f = r
            half = max(8.0, f * 0.035 / depth) * scale     # 5 cm 물체 + 여유
            self._draw_box(p, ox + u * scale, oy + v * scale, half, o, o.color == self.highlight)

    def _draw_box(self, p, x, y, half, o, hl):
        """검출 상자 + 이름표 (x, y = 화면 좌표의 물체 중심)."""
        col = COLORS.get(o.color, QColor('white'))
        p.setPen(QPen(QColor('white') if hl else col, 3 if hl else 2))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(x - half, y - half, 2 * half, 2 * half))
        label = f'{o.shape or "?"} ({o.color}) {o.confidence:.2f}'
        fm = p.fontMetrics()
        tw = fm.horizontalAdvance(label) + 8
        p.fillRect(QRectF(x - half, y - half - 18, tw, 17), col)
        p.setPen(QColor('white'))
        p.drawText(QRectF(x - half + 4, y - half - 18, tw, 17), Qt.AlignVCenter, label)
        if o.id and o.id == self.selected_id:   # 클릭/손짓으로 고른 물체
            p.setPen(QPen(QColor('#00e5ff'), 3, Qt.DashLine))
            p.drawRect(QRectF(x - half - 5, y - half - 5, 2 * half + 10, 2 * half + 10))
        if hl:   # 집는 점
            p.setPen(QPen(QColor('white'), 2))
            p.drawLine(QPointF(x - 7, y), QPointF(x + 7, y))
            p.drawLine(QPointF(x, y - 7), QPointF(x, y + 7))


def draw_place_marker(p, x, y, r=12, color='#00e5ff'):
    """놓을 자리 표시 (원 + 십자)."""
    p.setPen(QPen(QColor(color), 2))
    p.setBrush(Qt.NoBrush)
    p.drawEllipse(QPointF(x, y), r, r)
    p.drawLine(QPointF(x - r - 4, y), QPointF(x + r + 4, y))
    p.drawLine(QPointF(x, y - r - 4), QPointF(x, y + r + 4))


class WorkspaceView(QWidget):
    """작업 공간을 위에서 본 2D 지도 (world x 위쪽, y 왼쪽 - 카메라 영상과 같은 방향).

    탁자, 로봇 베이스, 놓기 구역(A/B/C), 인식된 물체, 현재 손끝(TCP) 위치를 그린다.
    RViz 3D 화면을 GUI 안에 넣는 것은 마스터플랜의 도전 과제로 남겨 둔다.
    """

    def __init__(self, zones=None, table=1.4):
        super().__init__()
        self.zones = zones or {'A': (0.30, -0.35), 'B': (0.0, -0.45), 'C': (-0.30, -0.35)}
        self.table = table
        self.objects = []
        self.tcp = None
        self.target = ''
        self.place_xy = None
        self.setMinimumSize(260, 220)

    def set_state(self, objects, tcp, target='', place_xy=None):
        self.objects, self.tcp, self.target, self.place_xy = objects, tcp, target, place_xy
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#eceff1'))
        span = self.table * 1.1
        s = min(self.width(), self.height()) / span
        cx, cy = self.width() / 2, self.height() / 2

        def pt(x, y):           # world (x, y) -> 화면: x 는 위쪽, y 는 왼쪽
            return QPointF(cx - y * s, cy - x * s)

        half = self.table / 2
        p.setPen(QPen(QColor('#8d6e63'), 1))
        p.setBrush(QColor('#d7ccc8'))
        p.drawRect(QRectF(pt(half, half), pt(-half, -half)))
        p.setFont(QFont('NanumSquare', 9))
        for name, (x, y) in self.zones.items():
            p.setPen(QPen(QColor('#78909c'), 1, Qt.DashLine))
            p.setBrush(QColor(255, 255, 255, 120))
            p.drawRect(QRectF(pt(x + 0.06, y + 0.06), pt(x - 0.06, y - 0.06)))
            p.setPen(QColor('#455a64'))
            p.drawText(QRectF(pt(x + 0.06, y + 0.06), pt(x - 0.06, y - 0.06)), Qt.AlignCenter, name)
        p.setPen(QPen(QColor('#546e7a'), 2))
        p.setBrush(QColor('#cfd8dc'))
        p.drawEllipse(pt(0, 0), 0.08 * s, 0.08 * s)
        p.drawText(QRectF(pt(-0.09, 0.2), pt(-0.2, -0.2)), Qt.AlignCenter, '로봇')
        for o in self.objects:
            c = o.pose.pose.position
            col = COLORS.get(o.color, QColor('gray'))
            p.setPen(QPen(QColor('white') if o.color == self.target else col.darker(130),
                          3 if o.color == self.target else 1))
            p.setBrush(col)
            p.drawRect(QRectF(pt(c.x + 0.025, c.y + 0.025), pt(c.x - 0.025, c.y - 0.025)))
        if self.place_xy is not None:
            q = pt(*self.place_xy)
            draw_place_marker(p, q.x(), q.y(), r=7)
        if self.tcp is not None:
            p.setPen(QPen(QColor('#6a1b9a'), 2))
            p.setBrush(Qt.NoBrush)
            q = pt(self.tcp[0], self.tcp[1])
            p.drawEllipse(q, 7, 7)
            p.drawLine(q + QPointF(-10, 0), q + QPointF(10, 0))
            p.drawLine(q + QPointF(0, -10), q + QPointF(0, 10))
        p.setPen(QColor('#455a64'))
        p.drawText(8, self.height() - 8, '↑ +X (로봇 정면)   ← +Y   ◎ 손끝(TCP)')
        # 축 방향 화살표
        p.setPen(QPen(QColor('#d32f2f'), 2))
        base = QPointF(24, 40)
        p.drawLine(base, base + QPointF(0, -22))
        p.setBrush(QBrush(QColor('#d32f2f')))
        p.drawPolygon(QPolygonF([base + QPointF(0, -26), base + QPointF(-4, -18), base + QPointF(4, -18)]))
