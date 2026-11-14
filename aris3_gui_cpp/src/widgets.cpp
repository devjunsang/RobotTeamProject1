// ARIS 공용 위젯 구현 (widgets.hpp 설명 참고).
#include "aris3_gui_cpp/widgets.hpp"

#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QFontMetricsF>
#include <QPainter>
#include <QPixmap>

#include <algorithm>
#include <cmath>

#include "aris3_gui_cpp/theme.hpp"

namespace aris
{

// ---------------------------------------------------------------- Panel
Panel::Panel(const QString & title, QWidget * parent)
: QFrame(parent)
{
  setObjectName("panel");
  auto * v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  auto * t = new QLabel(title);
  t->setObjectName("panelTitle");
  v->addWidget(t);
  auto * holder = new QFrame;
  holder->setObjectName("panelBody");
  body_ = new QVBoxLayout(holder);
  body_->setContentsMargins(10, 8, 10, 8);
  body_->setSpacing(6);
  v->addWidget(holder, 1);
  // HUD 빛: 판 둘레에 아크 리액터 청록 빛이 은은히 번지게 (Qt 스타일시트는 그림자·빛을 못 그림)
  auto * sh = new QGraphicsDropShadowEffect(this);
  sh->setBlurRadius(22);
  sh->setOffset(0, 0);
  sh->setColor(QColor(95, 244, 255, 90));
  setGraphicsEffect(sh);
}

// ---------------------------------------------------------------- CameraView
namespace
{
// 색 이름 -> 그리기 색
QColor objColor(const std::string & c)
{
  if (c == "red") {return QColor("#ff4d4f");}
  if (c == "blue") {return QColor("#4f8dff");}
  if (c == "yellow") {return QColor("#ffd23f");}
  if (c == "green") {return QColor("#3ddc84");}
  return theme::kCyan;
}

// 놓을 자리 표시 (시안 조준선)
void drawPlaceMarker(QPainter & p, const QPointF & c, double r, const QColor & col)
{
  p.setPen(QPen(col, 2));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(c, r, r);
  p.drawLine(c + QPointF(-r - 6, 0), c + QPointF(-r + 4, 0));
  p.drawLine(c + QPointF(r - 4, 0), c + QPointF(r + 6, 0));
  p.drawLine(c + QPointF(0, -r - 6), c + QPointF(0, -r + 4));
  p.drawLine(c + QPointF(0, r - 4), c + QPointF(0, r + 6));
}

// 검출 상자 + 이름표 (c = 화면 좌표의 물체 중심, half = 반 변)
void drawBox(QPainter & p, const QPointF & c, double half, const QString & label, const QColor & col,
  bool highlight, bool selected)
{
  p.setPen(QPen(highlight ? QColor("white") : col, highlight ? 3 : 2));
  p.setBrush(Qt::NoBrush);
  p.drawRect(QRectF(c.x() - half, c.y() - half, 2 * half, 2 * half));
  const QFontMetrics fm = p.fontMetrics();
  const QRectF tr(c.x() - half, c.y() - half - 18, fm.horizontalAdvance(label) + 8, 17);
  p.fillRect(tr, col);
  p.setPen(QColor("#0b0f14"));
  p.drawText(tr.adjusted(4, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, label);
  if (selected) {      // 클릭(이후 손짓)으로 고른 물체: 금색 점선
    p.setPen(QPen(theme::kGold, 3, Qt::DashLine));
    p.drawRect(QRectF(c.x() - half - 6, c.y() - half - 6, 2 * half + 12, 2 * half + 12));
  }
}
}  // namespace

CameraView::CameraView(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(320, 200);
  setCursor(Qt::CrossCursor);
  setToolTip(QStringLiteral("큐브 클릭 = 고르기(이거) · 빈 곳 클릭 = 놓을 자리(저기)"));
}

void CameraView::setFrame(const QImage & img, uint64_t seq, const Objects & world, const Objects & px,
  const std::optional<PinholeCamera> & cam)
{
  world_ = world;
  px_ = px;
  cam_ = cam;
  if (seq != seq_ && !img.isNull()) {
    img_ = img;
    seq_ = seq;
  }
  update();
}

void CameraView::setSelection(const std::string & id, const std::optional<std::array<double, 2>> & place,
  const std::string & highlight_color)
{
  selected_id_ = id;
  place_ = place;
  highlight_ = highlight_color;
}

void CameraView::setPixelMarks(const std::optional<std::array<double, 2>> & place_px,
  const std::optional<std::array<double, 2>> & calib_px)
{
  place_px_ = place_px;
  calib_px_ = calib_px;
}

// 닿는 범위 경계선의 진하기 (1 = 불투명): 선은 흐리게 (금색·빨강 모두 20 %), 글자는 조금 더 보이게
constexpr double kReachLineOpacity = 0.20;
constexpr double kReachLabelOpacity = 0.45;

void CameraView::setReachBoundary(const std::vector<std::vector<std::array<double, 2>>> & lines, const QString & label,
  const std::vector<std::vector<std::array<double, 2>>> & min_lines, const QString & min_label)
{
  reach_lines_ = lines;
  reach_label_ = label;
  reach_min_lines_ = min_lines;
  reach_min_label_ = min_label;
}

void CameraView::setPointer(const std::optional<std::array<double, 2>> & tip, double progress, const QString & text)
{
  pointer_ = tip;
  pointer_progress_ = progress;
  pointer_text_ = text;
}

QRectF CameraView::imageRect() const
{
  const QSize s = img_.size().scaled(size(), Qt::KeepAspectRatio);
  return QRectF((width() - s.width()) / 2.0, (height() - s.height()) / 2.0, s.width(), s.height());
}

void CameraView::mousePressEvent(QMouseEvent * ev)
{
  if (img_.isNull() || ev->button() != Qt::LeftButton) {return;}
  const QRectF r = imageRect();
  const double k = r.width() / img_.width();
  const double u = (ev->pos().x() - r.x()) / k, v = (ev->pos().y() - r.y()) / k;
  if (u < 0 || v < 0 || u >= img_.width() || v >= img_.height()) {return;}
  if (cam_) {
    PinholeCamera c = *cam_;
    c.width = img_.width();
    c.height = img_.height();
    if (const auto xy = c.unproject(u, v, plane_z); xy && onWorldClick) {onWorldClick((*xy)[0], (*xy)[1]);}
  } else if (onPixelClick) {
    onPixelClick(u, v);
  }
}

void CameraView::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor("#0b1015"));
  if (img_.isNull()) {
    p.setPen(theme::kMuted);
    p.drawText(rect(), Qt::AlignCenter, QStringLiteral("영상 없음 - Connect 로 시뮬레이션을 시작하세요"));
  } else {
    const QRectF r = imageRect();
    const double k = r.width() / img_.width();
    p.drawImage(r, img_);
    p.setFont(QFont(font().family(), 8));
    std::optional<PinholeCamera> cam = cam_;
    if (cam) {
      cam->width = img_.width();
      cam->height = img_.height();
    }
    if (show_det_) {
      // world 검출 (시뮬레이션): 카메라 TF 로 투영, 상자 크기 = 큐브 5 cm 를 깊이로 나눈 것
      if (cam) {
        for (const auto & o : world_) {
          const auto & c = o.pose.pose.position;
          const auto uvd = cam->project({c.x, c.y, c.z});
          if (!uvd) {continue;}
          const double half = std::max(6.0, cam->focal() * 0.025 / (*uvd)[2] * k);
          const QString label = QString::fromStdString(o.color + " " + (o.shape.empty() ? "?" : o.shape)) +
            QString(" %1").arg(o.confidence, 0, 'f', 2);
          drawBox(p, QPointF(r.x() + (*uvd)[0] * k, r.y() + (*uvd)[1] * k), half, label, objColor(o.color),
            !highlight_.empty() && o.color == highlight_, !o.id.empty() && o.id == selected_id_);
        }
      }
      // 픽셀 검출 (색 검출 스크립트): 중심과 넓이로 정사각 상자
      for (const auto & o : px_) {
        const double half = std::sqrt(std::max(1.0, static_cast<double>(o.size_px))) * k / 2 + 4;
        // 이름 + 층 (깊이로 계산, 0 = 모름이면 생략) + 큐브 각도 + 신뢰도. 예: 「red_cube_1 · 2층 · 42° 0.93」
        // 각도 (2026-11-13): 영상에서 큐브 면 방향 [0, 90)° - 브리지가 이 각도로 그리퍼(J6)를 맞춰 집는다
        const auto & q = o.pose.pose.orientation;
        const double ang = std::atan2(q.z, q.w) * 2.0 * 180.0 / M_PI;
        const QString label = QString::fromStdString(o.id.empty() ? o.color : o.id) +
          (o.level > 0 ? QStringLiteral(" · %1층").arg(o.level) : QString()) +
          QStringLiteral(" · %1°").arg(ang, 0, 'f', 0) +
          QString(" %1").arg(o.confidence, 0, 'f', 2);
        drawBox(p, QPointF(r.x() + o.pose.pose.position.x * k, r.y() + o.pose.pose.position.y * k), half, label,
          theme::kCyan, false, !o.id.empty() && o.id == selected_id_);
      }
    }
    // 닿는 범위 경계선 (2026-11-14): 금색 = 먼 쪽 한계(이 선 바깥은 닿지 않음), 빨강 = 가까운 쪽 한계(이 선 안쪽은
    // 로봇에 너무 가까워 관절 한계에 걸림). 둘 다 흐린 점선으로, 영상 안쪽만 그린다 (사용자 결정: 20 % 만 보이게)
    const auto drawReach = [&](const std::vector<std::vector<std::array<double, 2>>> & lines, const QString & label,
        const QColor & color, bool label_below) {
        if (lines.empty()) {return;}
        p.save();
        p.setClipRect(r);
        p.setBrush(Qt::NoBrush);
        std::optional<QPointF> top;                  // 글자를 붙일 자리: 영상 안에서 가장 위쪽 점
        for (const auto & line : lines) {
          QPolygonF poly;
          for (const auto & q : line) {
            const QPointF pt(r.x() + q[0] * k, r.y() + q[1] * k);
            poly << pt;
            if (r.contains(pt) && (!top || pt.y() < top->y())) {top = pt;}
          }
          p.setOpacity(kReachLineOpacity);
          p.setPen(QPen(color, 2, Qt::DashLine));
          p.drawPolyline(poly);
        }
        p.setOpacity(kReachLabelOpacity);            // 글자는 읽을 수 있을 만큼만
        if (top && !label.isEmpty()) {
          const QRectF box = QFontMetricsF(p.font()).boundingRect(label).adjusted(-4, -2, 4, 2);
          // 먼 쪽 선은 글자를 선 아래(안쪽)에, 가까운 쪽 선은 선 위(바깥쪽)에 둔다 - 쓸 수 있는 쪽에 보이게
          const double y = label_below ? top->y() + 6 : top->y() - box.height() - 6;
          const QPointF at(std::clamp(top->x() - box.width() / 2, r.left() + 4, r.right() - box.width() - 4),
            std::clamp(y, r.top() + 4, r.bottom() - box.height() - 4));
          p.fillRect(QRectF(at, box.size()), QColor(0, 0, 0, 170));
          p.setPen(color);
          p.drawText(QRectF(at, box.size()), Qt::AlignCenter, label);
        }
        p.restore();
      };
    drawReach(reach_lines_, reach_label_, theme::kGold, true);
    drawReach(reach_min_lines_, reach_min_label_, QColor("#ff3b3b"), false);
    // 실물: 놓을 자리 픽셀(시안), 보정 ① 기억 픽셀(노랑)
    if (place_px_) {drawPlaceMarker(p, QPointF(r.x() + (*place_px_)[0] * k, r.y() + (*place_px_)[1] * k), 12, theme::kCyan);}
    if (calib_px_) {drawPlaceMarker(p, QPointF(r.x() + (*calib_px_)[0] * k, r.y() + (*calib_px_)[1] * k), 16, QColor("#ffea00"));}
    // 손짓 포인터: 손끝 점 + 가리키고 멈춘 시간만큼 차오르는 링 + 손짓 글자
    if (pointer_) {
      const QPointF c(r.x() + (*pointer_)[0] * k, r.y() + (*pointer_)[1] * k);
      p.setPen(QPen(QColor(0, 0, 0, 120), 6));
      p.setBrush(Qt::NoBrush);
      p.drawEllipse(c, 22, 22);
      if (pointer_progress_ > 0) {
        p.setPen(QPen(theme::kCyan, 6, Qt::SolidLine, Qt::RoundCap));
        p.drawArc(QRectF(c.x() - 22, c.y() - 22, 44, 44), 90 * 16, -static_cast<int>(pointer_progress_ * 360 * 16));
      }
      p.setPen(Qt::NoPen);
      p.setBrush(theme::kCyan);
      p.drawEllipse(c, 5, 5);
      if (!pointer_text_.isEmpty()) {
        p.setPen(QColor("white"));
        p.drawText(c + QPointF(28, 5), pointer_text_);
      }
    }
    // 놓을 자리 (화면 선택 「저기」)
    if (place_ && cam) {
      if (const auto uvd = cam->project({(*place_)[0], (*place_)[1], plane_z})) {
        drawPlaceMarker(p, QPointF(r.x() + (*uvd)[0] * k, r.y() + (*uvd)[1] * k), 12, theme::kCyan);
      }
    }
  }
  // HUD 모서리 (빨강)
  p.setPen(QPen(theme::kRed.lighter(130), 2));
  const int m = 6, L = 22;
  const QRect b = rect().adjusted(m, m, -m, -m);
  p.drawLine(b.topLeft(), b.topLeft() + QPoint(L, 0));
  p.drawLine(b.topLeft(), b.topLeft() + QPoint(0, L));
  p.drawLine(b.topRight(), b.topRight() + QPoint(-L, 0));
  p.drawLine(b.topRight(), b.topRight() + QPoint(0, L));
  p.drawLine(b.bottomLeft(), b.bottomLeft() + QPoint(L, 0));
  p.drawLine(b.bottomLeft(), b.bottomLeft() + QPoint(0, -L));
  p.drawLine(b.bottomRight(), b.bottomRight() + QPoint(-L, 0));
  p.drawLine(b.bottomRight(), b.bottomRight() + QPoint(0, -L));
}

// ---------------------------------------------------------------- WorkspaceView
WorkspaceView::WorkspaceView(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(200, 150);
}

void WorkspaceView::setState(const std::vector<aris3_interfaces::msg::DetectedObject> & objs,
  const std::optional<std::array<double, 2>> & tcp, const std::string & highlight_color,
  const std::optional<std::array<double, 2>> & place)
{
  objs_ = objs;
  tcp_ = tcp;
  highlight_ = highlight_color;
  place_ = place;
  update();
}

void WorkspaceView::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const double span = table_ * 1.1;
  const double s = std::min(width(), height() - 18) / span;
  const QPointF ctr(width() / 2.0, (height() - 18) / 2.0);
  // world (x, y) -> 화면: x 는 위쪽, y 는 왼쪽 (카메라 영상과 같은 방향)
  auto pt = [&](double x, double y) {return QPointF(ctr.x() - y * s, ctr.y() - x * s);};
  const double h = table_ / 2;
  p.setPen(QPen(theme::kGold.darker(150), 1));
  p.setBrush(QColor("#2a1d16"));
  p.drawRect(QRectF(pt(h, h), pt(-h, -h)));
  p.setFont(QFont(font().family(), 8));
  for (const auto & [name, xy] : zones_) {          // 놓기 구역 A/B/C
    p.setPen(QPen(theme::kGold, 1, Qt::DashLine));
    p.setBrush(Qt::NoBrush);
    const QRectF zr(pt(xy[0] + 0.06, xy[1] + 0.06), pt(xy[0] - 0.06, xy[1] - 0.06));
    p.drawRect(zr);
    p.setPen(theme::kGoldText);
    p.drawText(zr, Qt::AlignCenter, QString::fromStdString(name));
  }
  p.setPen(QPen(theme::kMuted, 2));                  // 로봇 베이스
  p.setBrush(QColor("#3a2424"));
  p.drawEllipse(pt(0, 0), 0.08 * s, 0.08 * s);
  for (const auto & o : objs_) {                     // 물체 (5 cm)
    const auto & c = o.pose.pose.position;
    const bool hl = !highlight_.empty() && o.color == highlight_;
    p.setPen(QPen(hl ? QColor("white") : objColor(o.color).darker(130), hl ? 3 : 1));
    p.setBrush(objColor(o.color));
    p.drawRect(QRectF(pt(c.x + 0.025, c.y + 0.025), pt(c.x - 0.025, c.y - 0.025)));
  }
  if (place_) {drawPlaceMarker(p, pt((*place_)[0], (*place_)[1]), 7, theme::kCyan);}
  if (tcp_) {                                        // 손끝(TCP): 시안 조준
    const QPointF q = pt((*tcp_)[0], (*tcp_)[1]);
    p.setPen(QPen(theme::kCyan, 2));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(q, 7, 7);
    p.drawLine(q + QPointF(-10, 0), q + QPointF(10, 0));
    p.drawLine(q + QPointF(0, -10), q + QPointF(0, 10));
  }
  p.setPen(theme::kMuted);
  p.drawText(QRectF(4, height() - 18, width() - 8, 16), Qt::AlignLeft | Qt::AlignVCenter,
    QStringLiteral("↑ +X (로봇 정면)   ← +Y   ◎ 손끝(TCP)"));
}

// ---------------------------------------------------------------- LogoBadge
LogoBadge::LogoBadge(QWidget * parent)
: QWidget(parent)
{
  auto * h = new QHBoxLayout(this);
  h->setContentsMargins(0, 0, 0, 0);
  h->setSpacing(10);
  auto * mark = new QLabel;
  // 엠블럼은 aris3_gui/assets/aris_mark.svg 를 PNG 로 만든 것 (리소스에 포함). 고해상도에서 줄여 선명하게
  mark->setPixmap(QPixmap(":/aris/aris_mark_256.png").scaled(56, 56, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  h->addWidget(mark);
  auto * word = new QLabel("ARIS");
  word->setObjectName("wordmark");
  word->setToolTip(QStringLiteral("ARIS - ARIS3 Control Station (C++ Qt)"));
  h->addWidget(word);
  setAttribute(Qt::WA_TransparentForMouseEvents);   // 로고 뒤를 가리지 않게 클릭은 통과
}

}  // namespace aris
