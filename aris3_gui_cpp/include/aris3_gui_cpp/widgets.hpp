// ARIS 공용 위젯.
//   Panel      : 붉은 제목줄 + 금속 판 + 그림자 (목업의 패널)
//   CameraView : 카메라 영상 + 픽셀 검출 상자 + HUD 모서리 (M2 에서 TF 투영·클릭 선택을 더함)
//   LogoBadge  : ARIS 엠블럼 + 글자 (상단 줄 가운데)
#pragma once

#include <QFrame>
#include <QImage>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include <vector>

#include "aris3_interfaces/msg/detected_object.hpp"

namespace aris
{

class Panel : public QFrame
{
  Q_OBJECT

public:
  // title: 제목줄 글자 (예: "SYSTEM STATUS  시스템 상태")
  explicit Panel(const QString & title, QWidget * parent = nullptr);
  QVBoxLayout * body() {return body_;}

private:
  QVBoxLayout * body_;
};

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget * parent = nullptr);
  // 새 영상과 픽셀 검출 (영상 좌표). 같은 seq 면 다시 그리지 않음
  void setFrame(const QImage & img, uint64_t seq, const std::vector<aris3_interfaces::msg::DetectedObject> & px);
  void setShowDetections(bool on) {show_det_ = on; update();}

protected:
  void paintEvent(QPaintEvent * ev) override;

private:
  QImage img_;
  uint64_t seq_{0};
  std::vector<aris3_interfaces::msg::DetectedObject> px_;
  bool show_det_{true};
};

class LogoBadge : public QWidget
{
  Q_OBJECT

public:
  explicit LogoBadge(QWidget * parent = nullptr);
};

}  // namespace aris
