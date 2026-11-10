// ARIS 공용 위젯.
//   Panel      : 붉은 제목줄 + 금속 판 + 그림자 (목업의 패널)
//   CameraView    : 카메라 영상 + 검출 상자 (world 검출은 카메라 TF 로 투영, 픽셀 검출은 그대로) + 고른 물체·놓을 자리
//                   + HUD 모서리. 클릭하면 탁자 위 world 점(카메라 TF 있을 때) 또는 영상 픽셀을 알려 준다.
//   WorkspaceView : 작업 공간을 위에서 본 2D 지도 (탁자·로봇·구역·물체·손끝·놓을 자리)
//   LogoBadge     : ARIS 엠블럼 + 글자 (상단 줄 가운데)
#pragma once

#include <QFrame>
#include <QImage>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "aris3_gui_cpp/gui_logic.hpp"

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
  using Objects = std::vector<aris3_interfaces::msg::DetectedObject>;
  // 새 영상, world 검출(시뮬레이션), 픽셀 검출(색 검출), 카메라(TF 없으면 nullopt). 같은 seq 면 영상은 그대로
  void setFrame(const QImage & img, uint64_t seq, const Objects & world, const Objects & px,
    const std::optional<PinholeCamera> & cam);
  void setShowDetections(bool on) {show_det_ = on; update();}
  // 고른 물체 id·놓을 자리(world x, y)·작업 중인 색(강조)
  void setSelection(const std::string & id, const std::optional<std::array<double, 2>> & place,
    const std::string & highlight_color);

  std::function<void(double, double)> onWorldClick;   // 탁자 위 world (x, y) [m] - 카메라 TF 있을 때
  std::function<void(double, double)> onPixelClick;   // 영상 픽셀 (u, v) - 카메라 TF 없을 때 (실물)
  double plane_z{0.775};                               // 클릭을 탁자 위 이 높이(큐브 중심)로 본다

protected:
  void paintEvent(QPaintEvent * ev) override;
  void mousePressEvent(QMouseEvent * ev) override;

private:
  QRectF imageRect() const;                            // 위젯 안에서 영상이 그려지는 자리

  QImage img_;
  uint64_t seq_{0};
  Objects world_, px_;
  std::optional<PinholeCamera> cam_;
  std::string selected_id_, highlight_;
  std::optional<std::array<double, 2>> place_;
  bool show_det_{true};
};

class WorkspaceView : public QWidget
{
  Q_OBJECT

public:
  explicit WorkspaceView(QWidget * parent = nullptr);
  // 물체(world), 손끝(없으면 nullopt), 강조 색, 놓을 자리
  void setState(const std::vector<aris3_interfaces::msg::DetectedObject> & objs,
    const std::optional<std::array<double, 2>> & tcp, const std::string & highlight_color,
    const std::optional<std::array<double, 2>> & place);

protected:
  void paintEvent(QPaintEvent * ev) override;

private:
  std::vector<aris3_interfaces::msg::DetectedObject> objs_;
  std::optional<std::array<double, 2>> tcp_, place_;
  std::string highlight_;
  std::map<std::string, std::array<double, 2>> zones_{{"A", {0.30, -0.35}}, {"B", {0.0, -0.45}}, {"C", {-0.30, -0.35}}};
  double table_{1.4};
};

class LogoBadge : public QWidget
{
  Q_OBJECT

public:
  explicit LogoBadge(QWidget * parent = nullptr);
};

}  // namespace aris
