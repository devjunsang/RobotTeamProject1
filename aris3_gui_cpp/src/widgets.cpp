// ARIS 공용 위젯 구현 (widgets.hpp 설명 참고).
#include "aris3_gui_cpp/widgets.hpp"

#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
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
CameraView::CameraView(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(320, 200);
}

void CameraView::setFrame(const QImage & img, uint64_t seq,
  const std::vector<aris3_interfaces::msg::DetectedObject> & px)
{
  px_ = px;
  if (seq != seq_ && !img.isNull()) {
    img_ = img;
    seq_ = seq;
  }
  update();
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
    // 비율을 지키며 가운데에 맞춘다
    const QSize s = img_.size().scaled(size(), Qt::KeepAspectRatio);
    const QRect r((width() - s.width()) / 2, (height() - s.height()) / 2, s.width(), s.height());
    p.drawImage(r, img_);
    if (show_det_) {
      const double k = static_cast<double>(s.width()) / img_.width();
      p.setFont(QFont(font().family(), 8));
      for (const auto & o : px_) {
        // 픽셀 검출: 중심과 넓이로 정사각 상자 (색 검출 스크립트는 중심·넓이만 준다)
        const double side = std::sqrt(std::max(1.0, static_cast<double>(o.size_px))) * k;
        const QPointF c(r.x() + o.pose.pose.position.x * k, r.y() + o.pose.pose.position.y * k);
        p.setPen(QPen(theme::kCyan, 2));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(c.x() - side / 2 - 4, c.y() - side / 2 - 4, side + 8, side + 8));
        const QString label = QString::fromStdString(o.id.empty() ? o.color : o.id) +
          QString(" %1").arg(o.confidence, 0, 'f', 2);
        const QRectF tr(c.x() - side / 2 - 4, c.y() - side / 2 - 22, 8 + label.size() * 6.5, 16);
        p.fillRect(tr, theme::kCyan);
        p.setPen(QColor("#0b0f14"));
        p.drawText(tr.adjusted(3, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, label);
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
