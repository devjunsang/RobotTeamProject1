// ARIS 공용 위젯 구현 (widgets.hpp 설명 참고).
#include "aris3_gui_cpp/widgets.hpp"

#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QMouseEvent>
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
  // 입체감: 판이 바탕에서 떠 보이게 (Qt 스타일시트는 그림자를 못 그림)
  auto * sh = new QGraphicsDropShadowEffect(this);
  sh->setBlurRadius(18);
  sh->setOffset(0, 5);
  sh->setColor(QColor(0, 0, 0, 200));
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
        const QString label = QString::fromStdString(o.id.empty() ? o.color : o.id) +
          QString(" %1").arg(o.confidence, 0, 'f', 2);
        drawBox(p, QPointF(r.x() + o.pose.pose.position.x * k, r.y() + o.pose.pose.position.y * k), half, label,
          theme::kCyan, false, !o.id.empty() && o.id == selected_id_);
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
