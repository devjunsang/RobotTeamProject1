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
        self.qimage = None
        self.objects = []
        self.cam_tf = None          # world -> 광학 좌표 TransformStamped
        self.show_detections = True
        self.highlight = ''         # 지금 작업 중인 색
        self.setMinimumSize(480, 360)

    def set_frame(self, image, objects, cam_tf, highlight=''):
        if image is not None:
            w, h, enc, data = image
            fmt = {'rgb8': QImage.Format_RGB888, 'bgr8': QImage.Format_BGR888}.get(enc)
            if fmt is not None:
                self.qimage = QImage(data, w, h, w * 3, fmt).copy()
        self.objects = objects
        self.cam_tf = cam_tf
        self.highlight = highlight
        self.update()

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
        scale = min(self.width() / img.width(), self.height() / img.height())
        dw, dh = img.width() * scale, img.height() * scale
        ox, oy = (self.width() - dw) / 2, (self.height() - dh) / 2
        p.drawImage(QRectF(ox, oy, dw, dh), img)
        if not self.show_detections:
            return
        p.setFont(QFont('NanumSquare', 10, QFont.Bold))
        for o in self.objects:
            pos = o.pose.pose.position
            r = self.project((pos.x, pos.y, pos.z))
            if r is None:
                continue
            u, v, depth, f = r
            half = max(8.0, f * 0.035 / depth) * scale     # 5 cm 물체 + 여유
            x, y = ox + u * scale, oy + v * scale
            col = COLORS.get(o.color, QColor('white'))
            hl = o.color == self.highlight
            p.setPen(QPen(QColor('white') if hl else col, 3 if hl else 2))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(x - half, y - half, 2 * half, 2 * half))
            label = f'{o.shape or "?"} ({o.color}) {o.confidence:.2f}'
            fm = p.fontMetrics()
            tw = fm.horizontalAdvance(label) + 8
            p.fillRect(QRectF(x - half, y - half - 18, tw, 17), col)
            p.setPen(QColor('white'))
            p.drawText(QRectF(x - half + 4, y - half - 18, tw, 17), Qt.AlignVCenter, label)
            if hl:   # 집는 점
                p.setPen(QPen(QColor('white'), 2))
                p.drawLine(QPointF(x - 7, y), QPointF(x + 7, y))
                p.drawLine(QPointF(x, y - 7), QPointF(x, y + 7))


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
        self.setMinimumSize(260, 220)

    def set_state(self, objects, tcp, target=''):
        self.objects, self.tcp, self.target = objects, tcp, target
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
