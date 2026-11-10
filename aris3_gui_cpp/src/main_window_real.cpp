// ARIS 메인 창 - 실물 모드 부분 (M3). aris3_gui panels/real_pick_panel.py·jog.py 실물 부분과 같은 순서·확인창.
//   실물 집기·카메라 보정 패널, 클릭·손짓으로 큐브/놓을 자리 고르기, 손짓 포인터·이벤트(aris3_gesture),
//   집기·놓기·옮기기(브리지 RealPickPlace), 실물 음성(매번 확인창), 복구 후 홈, 손으로 움직이기, 조작 허용 맞추기.
// 로봇을 움직이는 것은 모두 aris3_real_bridge 가 하고(이동 전 검사·도착 확인), 여기서는 확인창·조작 허용만 맡는다.
#include <QGridLayout>
#include <QHBoxLayout>
#include <QTimer>

#include <cmath>

#include "aris3_gui_cpp/main_window.hpp"
#include "aris3_gui_cpp/theme.hpp"

namespace aris
{

namespace
{
using PP = aris3_interfaces::action::RealPickPlace::Goal;    // 집기 종류 (PICK, PLACE_AT ...)
using CAL = aris3_interfaces::srv::RealCalibration::Request;  // 보정 명령 (REMEMBER, RECORD ...)
const double kCubeSelectPx = 40;          // 클릭: 이 거리 안의 검출 큐브를 고름
const double kCubeSelectGesturePx = 60;   // 손짓: 손끝이 큐브를 가리지 않게 조금 옆을 가리켜도 고름

// 손짓 이름 -> 화면 글자
QString gestureKo(const std::string & g)
{
  if (g == "point") {return QStringLiteral("☝ 가리킴");}
  if (g == "fist") {return QStringLiteral("✊ 주먹");}
  if (g == "open") {return QStringLiteral("🖐 펼침");}
  return QStringLiteral("…");
}

// 픽셀 검출 중 (u, v) 에 가장 가까운 것과 그 거리
const aris3_interfaces::msg::DetectedObject * nearestPx(const std::vector<aris3_interfaces::msg::DetectedObject> & px,
  double u, double v, double & dist)
{
  const aris3_interfaces::msg::DetectedObject * best = nullptr;
  dist = 1e9;
  for (const auto & o : px) {
    const double d = std::hypot(o.pose.pose.position.x - u, o.pose.pose.position.y - v);
    if (d < dist) {dist = d; best = &o;}
  }
  return best;
}

// Space 가 버튼을 「누르지」 않게
QPushButton * nf(QPushButton * b)
{
  b->setFocusPolicy(Qt::NoFocus);
  return b;
}
}  // namespace

// ======================================================================== 화면
QWidget * MainWindow::buildRealPickPanel()
{
  auto * p = new Panel(QStringLiteral("REAL PICK  실물 집기 · 카메라 보정"));
  lbl_pick_ = new QLabel(QStringLiteral("카메라 화면에서 큐브를 클릭하거나 손으로 가리키세요"));
  lbl_pick_->setWordWrap(true);
  p->body()->addWidget(lbl_pick_);

  auto * g = new QGridLayout;
  auto * b1 = nf(new QPushButton(QStringLiteral("① 큐브 픽셀 기억")));
  b1->setToolTip(QStringLiteral("큐브가 카메라에 가려지지 않았을 때 누르세요 (고른 큐브의 지금 위치를 기억)"));
  connect(b1, &QPushButton::clicked, this, [this]() {
      if (!pick_px_) {
        node_->log("WARN", "보정 ①: 먼저 카메라 화면에서 큐브를 클릭하세요");
        return;
      }
      real_->calibration(CAL::REMEMBER, (*pick_px_)[0], (*pick_px_)[1]);
    });
  auto * b2 = nf(new QPushButton(QStringLiteral("② 로봇 위치 기록")));
  b2->setToolTip(QStringLiteral("조그로 그리퍼를 그 큐브를 집는 자세(손가락 사이에 큐브)까지 내린 뒤 누르세요"));
  connect(b2, &QPushButton::clicked, this, [this]() {real_->calibration(CAL::RECORD, 0, 0, pick_name_);});
  auto * b3 = nf(new QPushButton(QStringLiteral("마지막 점 삭제")));
  connect(b3, &QPushButton::clicked, this, [this]() {real_->calibration(CAL::REMOVE_LAST);});
  auto * b4 = nf(new QPushButton(QStringLiteral("보정 저장")));
  connect(b4, &QPushButton::clicked, this, [this]() {real_->calibration(CAL::SAVE);});
  g->addWidget(b1, 0, 0);
  g->addWidget(b2, 0, 1);
  g->addWidget(b3, 1, 0);
  g->addWidget(b4, 1, 1);
  p->body()->addLayout(g);
  lbl_calib_ = new QLabel;
  lbl_calib_->setWordWrap(true);
  lbl_calib_->setStyleSheet("color:#9aa7b4");
  p->body()->addWidget(lbl_calib_);

  auto * row = new QHBoxLayout;
  auto addBtn = [&](const QString & text, const QString & tip, std::function<void()> fn, const char * name = nullptr) {
      auto * b = nf(new QPushButton(text));
      b->setToolTip(tip);
      if (name) {b->setObjectName(name);}
      connect(b, &QPushButton::clicked, this, fn);
      row->addWidget(b);
    };
  addBtn(QStringLiteral("큐브 위로"), QStringLiteral("고른 큐브 위 접근 높이(+100 mm)로 가서 멈춤"), [this]() {pickHover();});
  addBtn(QStringLiteral("집기"), QStringLiteral("접근 -> 열기 -> 하강 -> 닫기 -> 들기"), [this]() {pickRun();}, "gold");
  addBtn(QStringLiteral("고른 자리에 놓기"), QStringLiteral("카메라 화면에서 고른 빈 자리(시안) 위로 가서 내려놓고 위로"),
    [this]() {placeSelected();});
  addBtn(QStringLiteral("여기 내려놓기"), QStringLiteral("지금 위치 아래로 내려놓고 위로"), [this]() {placeHere();});
  p->body()->addLayout(row);

  auto * grow = new QHBoxLayout;
  chk_gesture_ = new QCheckBox(QStringLiteral("손짓 사용 (MediaPipe)"));
  chk_gesture_->setFocusPolicy(Qt::NoFocus);
  chk_gesture_->setChecked(true);
  chk_gesture_->setToolTip(QStringLiteral("☝ 가리키고 1초 멈춤 = 큐브/빈 자리 선택 · ☝→✊ = 집기 · ☝→🖐 = 고른 자리에 놓기\n"
                                          "(실행 전 항상 확인창)"));
  lbl_gesture_ = new QLabel(QStringLiteral("손짓: —"));
  grow->addWidget(chk_gesture_);
  grow->addWidget(lbl_gesture_, 1);
  p->body()->addLayout(grow);
  auto * note = new QLabel(QStringLiteral("☝ 1초 = 선택 · ☝→✊ = 집기 · ☝→🖐 = 놓기 · 실행 전 확인창 · "
                                          "움직이는 버튼은 「조작 허용」 필요 · 정지: Space"));
  note->setWordWrap(true);
  note->setStyleSheet("color:#7d8a96; font-size:8pt");
  p->body()->addWidget(note);
  p->body()->addStretch(1);
  return p;
}

// ======================================================================== 고르기 (클릭·손짓)
void MainWindow::selectPixel(double u, double v, double radius, const std::string & how)
{
  // 그 픽셀 가까이 색 검출 큐브가 있으면 그 큐브를, 없으면 그 자리를 놓을 곳으로
  double d = 0;
  const auto snap = node_->snapshot();
  const auto * near = nearestPx(snap.px_objects, u, v, d);
  char buf[160];
  if (near && d <= radius) {
    pick_px_ = std::array<double, 2>{near->pose.pose.position.x, near->pose.pose.position.y};
    pick_name_ = near->id.empty() ? near->color : near->id;
    sel_id_ = near->id;
    std::snprintf(buf, sizeof(buf), "%s: 큐브 %s 고름 (픽셀 %.0f, %.0f)", how.c_str(), pick_name_.c_str(),
      (*pick_px_)[0], (*pick_px_)[1]);
  } else {
    place_px_ = std::array<double, 2>{u, v};
    const auto xy = real_->toRobot(u, v);
    if (xy) {
      std::snprintf(buf, sizeof(buf), "%s: 놓을 자리 고름 (픽셀 %.0f, %.0f -> 로봇 X %.0f, Y %.0f mm)", how.c_str(), u, v,
        (*xy)[0], (*xy)[1]);
    } else {
      std::snprintf(buf, sizeof(buf), "%s: 놓을 자리 고름 (픽셀 %.0f, %.0f)", how.c_str(), u, v);
    }
  }
  node_->log("INFO", buf);
  updatePickLabel();
}

void MainWindow::updatePickLabel()
{
  QStringList lines;
  auto where = [this](const std::array<double, 2> & px) {
      const auto xy = real_->toRobot(px[0], px[1]);
      return xy ? QStringLiteral("로봇 X %1  Y %2 mm").arg((*xy)[0], 0, 'f', 1).arg((*xy)[1], 0, 'f', 1) :
             QStringLiteral("로봇 좌표: 보정 전");
    };
  if (!pick_px_) {
    lines << QStringLiteral("고른 큐브: — (카메라 화면에서 큐브를 클릭하거나 손으로 가리키세요)");
  } else {
    lines << QStringLiteral("고른 큐브: %1  픽셀 (%2, %3)  →  %4").arg(QString::fromStdString(pick_name_))
      .arg((*pick_px_)[0], 0, 'f', 0).arg((*pick_px_)[1], 0, 'f', 0).arg(where(*pick_px_));
  }
  if (place_px_) {
    lines << QStringLiteral("놓을 자리: 픽셀 (%1, %2)  →  %3").arg((*place_px_)[0], 0, 'f', 0)
      .arg((*place_px_)[1], 0, 'f', 0).arg(where(*place_px_));
  }
  lbl_pick_->setText(lines.join("\n"));
}

// ======================================================================== 주기 갱신
void MainWindow::refreshReal(const Snapshot & s)
{
  // 보정 요약 (바뀌면 고른 큐브의 로봇 좌표도 다시)
  const QString text = real_->calibSummary();
  if (text != calib_text_) {
    calib_text_ = text;
    lbl_calib_->setText(text);
    updatePickLabel();
  }
  gestureTick(s);
  // 로봇 상태·복구 버튼·손으로 움직이기
  const QString st = real_->robotStateText();
  const bool bad = st != QStringLiteral("대기") && st != QStringLiteral("이동 중");
  lbl_robot_state_->setText(QStringLiteral("로봇 상태: ") + st);
  lbl_robot_state_->setStyleSheet(bad ? QStringLiteral("color:#ff4d4f") : QString());
  btn_recover_->setEnabled(real_->canRecover());
  bool switching = false;
  QString name;
  const auto manual = real_->handGuide(switching, name);
  chk_hand_->blockSignals(true);
  chk_hand_->setChecked(manual.value_or(false));
  chk_hand_->blockSignals(false);
  chk_hand_->setEnabled(manual.has_value() && !switching);
  chk_hand_->setText(QStringLiteral("손으로 움직이기 (수동 모드)") +
    (switching ? QStringLiteral(" - 바꾸는 중…") : manual ? QStringLiteral(" · 지금: ") + name : QString()));
  // 조작 허용을 브리지와 맞춘다: 브리지가 스스로 잠갔으면(시간 초과·손으로 움직이기·접속 끊기) 화면도 끈다.
  // 보낸 직후 1.5 초는 상태 토픽이 아직 예전 값이라 맞추지 않는다 (켜자마자 꺼지지 않게).
  const auto bs = real_->state();
  if (bs && std::chrono::steady_clock::now() - arm_sent_at_ > std::chrono::milliseconds(1500)) {
    if (chk_arm_->isChecked() && !bs->armed) {
      chk_arm_->blockSignals(true);
      chk_arm_->setChecked(false);
      chk_arm_->blockSignals(false);
      node_->log("INFO", "실물 브리지가 조작 허용을 잠금 (시간 초과·손으로 움직이기·접속 끊기)");
    } else if (!chk_arm_->isChecked() && bs->armed) {
      arm_sent_at_ = std::chrono::steady_clock::now();
      real_->setArmed(false);
    }
  }
}

// ======================================================================== 손짓 (aris3_gesture 이벤트)
void MainWindow::gestureTick(const Snapshot & s)
{
  // 포인터·진행 링을 그리고, 선택·확정 이벤트를 처리한다.
  // 확인창이 떠 있거나 「손짓 사용」 이 꺼져 있으면 이벤트는 버린다 (확인창 위에 새 확인창을 띄우지 않게).
  const auto events = node_->popGestureEvents();
  const auto & g = s.gesture;
  if (!chk_gesture_->isChecked() || !g || !g->hand_present) {
    camera_->setPointer(std::nullopt, 0, QString());
    lbl_gesture_->setText(chk_gesture_->isChecked() ? QStringLiteral("손짓: 손 없음") : QStringLiteral("손짓: 꺼짐"));
    return;
  }
  camera_->setPointer(std::array<double, 2>{g->tip_u, g->tip_v}, g->hold_progress, gestureKo(g->gesture));
  lbl_gesture_->setText(QStringLiteral("손짓: %1  (손끝 %2, %3)").arg(gestureKo(g->gesture))
    .arg(g->tip_u, 0, 'f', 0).arg(g->tip_v, 0, 'f', 0));
  for (const auto & ev : events) {
    if (dialog_open_) {continue;}
    if (ev.type == aris3_interfaces::msg::GestureEvent::SELECT) {
      selectPixel(ev.u, ev.v, kCubeSelectGesturePx, "손짓");
    } else if (!real_->pickRunning()) {
      const bool pick = ev.type == aris3_interfaces::msg::GestureEvent::CONFIRM_PICK;
      QTimer::singleShot(0, this, [this, pick]() {gestureConfirm(pick);});   // 확인창은 주기 갱신 밖에서
    }
  }
}

void MainWindow::gestureConfirm(bool pick)
{
  if (pick) {
    if (!pick_px_) {
      node_->log("WARN", "손짓 ✊: 집을 큐브를 먼저 가리키세요 (☝ 1초)");
      return;
    }
    pickRun(QStringLiteral("손짓: ☝ → ✊ 주먹 (집기)"));
  } else {
    placeSelected(QStringLiteral("손짓: ☝ → 🖐 펼침 (고른 자리에 놓기)"));
  }
}

// ======================================================================== 집기·놓기
std::optional<std::array<double, 3>> MainWindow::pickTarget(const std::string & what)
{
  // 고른 큐브의 지금 로봇 좌표 (x, y, 집는 높이). 움직이기 직전에 픽셀을 다시 읽는다 (확인창 표시용)
  if (!real_->calibReady()) {
    node_->log("WARN", what + ": 카메라 보정이 없음 (점 4개 이상 기록)");
    return std::nullopt;
  }
  if (!pick_px_) {
    node_->log("WARN", what + ": 카메라 화면에서 큐브를 클릭하세요");
    return std::nullopt;
  }
  if (real_->pickRunning()) {
    node_->log("WARN", what + ": 이전 순서가 진행 중");
    return std::nullopt;
  }
  double d = 0;
  const auto * near = nearestPx(node_->snapshot().px_objects, (*pick_px_)[0], (*pick_px_)[1], d);
  if (near && d < 40) {
    pick_px_ = std::array<double, 2>{near->pose.pose.position.x, near->pose.pose.position.y};
  } else {
    node_->log("WARN", what + ": 고른 큐브가 지금 보이지 않음 - 마지막 위치 사용");
  }
  const auto xy = real_->toRobot((*pick_px_)[0], (*pick_px_)[1]);
  updatePickLabel();
  if (!xy) {return std::nullopt;}
  return std::array<double, 3>{(*xy)[0], (*xy)[1], real_->graspZ().value_or(0)};
}

void MainWindow::pickHover()
{
  if (!armOk("큐브 위로")) {return;}
  if (pickTarget("큐브 위로")) {real_->pickPlace(PP::HOVER, "큐브 위로", pick_name_, pick_px_, std::nullopt);}
}

void MainWindow::pickRun(const QString & heard)
{
  if (!armOk("집기")) {return;}
  const auto t = pickTarget("집기");
  if (!t) {return;}
  const QString said = heard.isEmpty() ? QString() : heard + "\n\n";
  if (ask(QStringLiteral("집기"), said + QStringLiteral("%1 을(를) 집습니다.\n로봇 좌표 X %2, Y %3 mm, 집는 높이 Z %4 mm\n\n"
      "먼저 「큐브 위로」 로 위치를 확인했습니까?").arg(QString::fromStdString(pick_name_))
      .arg((*t)[0], 0, 'f', 0).arg((*t)[1], 0, 'f', 0).arg((*t)[2], 0, 'f', 0)))
  {
    real_->pickPlace(PP::PICK, "집기", pick_name_, pick_px_, std::nullopt);
  }
}

void MainWindow::placeSelected(const QString & heard)
{
  if (!armOk("고른 자리에 놓기")) {return;}
  if (real_->pickRunning()) {
    node_->log("WARN", "고른 자리에 놓기: 이전 순서가 진행 중");
    return;
  }
  if (!real_->calibReady()) {
    node_->log("WARN", "고른 자리에 놓기: 카메라 보정이 없음 (점 4개 이상 기록)");
    return;
  }
  if (!place_px_) {
    node_->log("WARN", "고른 자리에 놓기: 놓을 자리를 먼저 고르세요 (빈 곳 클릭 또는 손으로 가리키고 1초)");
    return;
  }
  const auto xy = real_->toRobot((*place_px_)[0], (*place_px_)[1]);
  const auto h = real_->holding();
  const QString said = heard.isEmpty() ? QString() : heard + "\n\n";
  if (xy && ask(QStringLiteral("고른 자리에 놓기"), said + QStringLiteral("로봇 X %1, Y %2 mm 위로 가서 내려놓습니다.%3\n\n"
      "로봇 주변(손 포함)이 비었습니까?").arg((*xy)[0], 0, 'f', 0).arg((*xy)[1], 0, 'f', 0)
      .arg(h && !*h ? QStringLiteral("\n※ 마지막 집기에서 빈손으로 판단됨") : QString())))
  {
    real_->pickPlace(PP::PLACE_AT, "고른 자리에 놓기", "", std::nullopt, place_px_);
    place_px_.reset();
    updatePickLabel();
  }
}

void MainWindow::placeHere(const QString & heard)
{
  if (!armOk("내려놓기")) {return;}
  if (!real_->calibReady()) {
    node_->log("WARN", "내려놓기: 카메라 보정이 없음 (집는 높이를 모름)");
    return;
  }
  if (!heard.isEmpty() && !ask(QStringLiteral("내려놓기"),
      heard + QStringLiteral("\n\n지금 위치 바로 아래로 내려놓고 위로 빠집니다. 실행할까요?")))
  {
    return;
  }
  real_->pickPlace(PP::PLACE_HERE, "여기 내려놓기", "", std::nullopt, std::nullopt);
}

void MainWindow::pickAndPlace(const QString & heard)
{
  if (!armOk("집어서 옮기기")) {return;}
  if (!place_px_) {
    node_->log("WARN", "집어서 옮기기: 놓을 자리를 먼저 고르세요 (빈 곳 클릭 또는 손으로 가리키고 1초)");
    return;
  }
  const auto t = pickTarget("집어서 옮기기");
  const auto xy = real_->toRobot((*place_px_)[0], (*place_px_)[1]);
  if (!t || !xy) {return;}
  if (ask(QStringLiteral("집어서 옮기기"), heard + QStringLiteral("\n\n%1 을(를) 로봇 X %2, Y %3 mm 에서 집어\n"
      "X %4, Y %5 mm 에 놓습니다. (잡지 못하면 옮기지 않음)\n\n로봇 주변(손 포함)이 비었습니까?")
      .arg(QString::fromStdString(pick_name_)).arg((*t)[0], 0, 'f', 0).arg((*t)[1], 0, 'f', 0)
      .arg((*xy)[0], 0, 'f', 0).arg((*xy)[1], 0, 'f', 0)))
  {
    real_->pickPlace(PP::PICK_AND_PLACE, "집어서 옮기기", pick_name_, pick_px_, place_px_);
    place_px_.reset();
    updatePickLabel();
  }
}

// ======================================================================== 실물 음성 (매번 확인창)
void MainWindow::realVoice(const aris3_interfaces::msg::VoiceCommand & v)
{
  if (real_->pickRunning() || dialog_open_) {
    node_->log("WARN", "음성 「" + v.text + "」 무시: 이전 순서 진행 중이거나 확인창이 떠 있음");
    return;
  }
  const QString heard = QStringLiteral("음성: 「%1」").arg(QString::fromStdString(v.text));
  if (v.action == "home") {                         // 「아리스 홈으로」
    homeVoice(heard);
    return;
  }
  if (v.refers_selected_place && v.color.empty() && real_->holding().value_or(false)) {
    placeSelected(heard);                            // 들고 있을 때 「저기 놔줘」 = 고른 자리에 놓기
    return;
  }
  if (v.action == "place") {
    placeHere(heard);
    return;
  }
  if (!v.color.empty()) {                            // 「빨간 큐브 집어줘」: 그 색 검출이 하나일 때만
    std::vector<aris3_interfaces::msg::DetectedObject> same;
    for (const auto & o : node_->snapshot().px_objects) {
      if (o.color == v.color) {same.push_back(o);}
    }
    if (same.empty()) {
      node_->log("WARN", "음성: 카메라에 " + v.color + " 큐브가 보이지 않음");
      return;
    }
    if (same.size() > 1) {
      node_->log("WARN", "음성: " + v.color + " 큐브가 " + std::to_string(same.size()) +
        "개 - 화면에서 클릭한 뒤 「아리스 이거 집어줘」");
      return;
    }
    pick_px_ = std::array<double, 2>{same[0].pose.pose.position.x, same[0].pose.pose.position.y};
    pick_name_ = same[0].id.empty() ? same[0].color : same[0].id;
    updatePickLabel();
  } else if (!pick_px_) {                            // 「이거 집어줘」
    node_->log("WARN", "「이거」 가 무엇인지 모름 - 먼저 카메라 화면에서 큐브를 클릭하거나 가리키세요");
    return;
  }
  if (v.refers_selected_place) {                     // 「이거 저기로 옮겨줘」: 집어서 고른 자리에 놓기
    pickAndPlace(heard);
  } else {
    pickRun(heard);
  }
}

void MainWindow::homeVoice(const QString & heard)
{
  if (!armOk("홈 이동")) {return;}
  const QString hold = real_->holding().value_or(false) ? QStringLiteral("\n※ 그리퍼가 큐브를 들고 있으면 함께 움직입니다.") :
    QString();
  if (ask(QStringLiteral("홈으로 돌아가기"), heard + QStringLiteral("\n\n홈 자세 [0, 0, 90, 0, 90, 0]° 로 관절 이동합니다 "
      "(속도 %1 %, 최대 45°/s).%2\n\n팔이 크게 움직일 수 있습니다. 주변(손 포함)이 비었습니까?")
      .arg(sld_speed_->value()).arg(hold)))
  {
    backend_->home(sld_speed_->value());
  }
}

// ======================================================================== 복구·손으로 움직이기
void MainWindow::recoverHome()
{
  if (!chk_arm_->isChecked()) {
    node_->log("WARN", "복구 후 홈: 「조작 허용」 을 먼저 체크하세요");
    return;
  }
  if (real_->pickRunning()) {
    node_->log("WARN", "복구 후 홈: 집기/놓기 순서가 진행 중");
    return;
  }
  if (!ask(QStringLiteral("복구 후 홈"), QStringLiteral("지금 로봇 상태: %1\n\n1. 안전 정지 해제 또는 서보 켜기\n"
      "2. 수동 모드였으면 자동 모드로\n3. 홈 자세 [0, 0, 90, 0, 90, 0]° 로 관절 이동 (속도 슬라이더 비율, 최대 45°/s)\n\n"
      "팔이 크게 움직일 수 있습니다. 주변이 비었고 비상정지를 누를 수 있습니까?\n(그리퍼가 물체를 들고 있으면 함께 움직입니다)")
      .arg(real_->robotStateText())))
  {
    return;
  }
  last_cmd_ = std::chrono::steady_clock::now();
  real_->recoverHome(sld_speed_->value());
}

void MainWindow::toggleHandGuide(bool on)
{
  bool switching = false;
  QString name;
  const auto manual = real_->handGuide(switching, name);
  chk_hand_->setChecked(manual.value_or(false));    // 실제 모드는 주기 갱신이 다시 맞춘다
  if (switching) {return;}
  if (on) {
    if (real_->pickRunning()) {
      node_->log("WARN", "손으로 움직이기: 집기/놓기 순서가 진행 중 - 끝난 뒤에 켜세요");
      return;
    }
    if (!ask(QStringLiteral("손으로 움직이기 (수동 모드)"), QStringLiteral("로봇을 수동 모드로 바꿉니다.\n\n"
        "· 팔 끝 콕핏의 직접 교시 버튼을 누르고 있는 동안만 팔이 손에 따라 움직입니다.\n"
        "· 버튼을 놓으면 그 자리에 멈춥니다.\n· 켜져 있는 동안 이동·그리퍼·집기 명령은 막힙니다.\n"
        "· 그리퍼가 물체를 들고 있으면 먼저 내려놓으세요.\n\n계속할까요?")))
    {
      return;
    }
  }
  chk_arm_->setChecked(false);
  real_->setHandGuide(on);
}

// ======================================================================== 시험용 (--selftest, 실물 모드)
void MainWindow::runSelftestReal(std::function<void(bool)> done)
{
  auto_yes_ = true;                                  // 확인창은 자동 「예」 (시험 로그에 남김)
  auto report = [this](const std::string & name, bool ok) {
      selftest_ok_ &= ok;
      std::printf("SELFTEST %s %s\n", ok ? "PASS" : "FAIL", name.c_str());
      std::fflush(stdout);
    };
  auto mark = [this]() {
      const auto logs = node_->snapshot().logs;
      selftest_log_mark_ = logs.empty() ? 0 : logs.back().seq;
      selftest_t0_ = std::chrono::steady_clock::now();
    };
  auto elapsed = [this]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - selftest_t0_).count();
    };
  // 「label: 」 로 시작하는 브리지 결과 로그로 판정: INFO(성공) 이면 1, WARN/ERROR 이면 -1, 아직 없으면 0.
  // 문장 속 낱말로 판정하지 않는다 - 실패 문장 안에도 「그리퍼 닫기: 완료」 처럼 「완료」 가 들어갈 수 있다 (2026-11-11 거짓 PASS)
  auto logHas = [this](const std::string & label) {
      const std::string head = label + ": ";
      for (const auto & l : node_->snapshot().logs) {
        if (l.seq <= selftest_log_mark_ || l.node != "gui" || l.text.rfind(head, 0) != 0) {continue;}
        return l.level == "INFO" ? 1 : -1;
      }
      return 0;
    };
  connect(&selftest_timer_, &QTimer::timeout, this, [=]() {
      switch (selftest_step_) {
        case 0:                                     // 브리지 상태가 오면 접속
          if (!real_->state()) {return;}
          mark();
          backend_->connectRobot();
          selftest_step_ = 1;
          return;
        case 1: {                                   // 대기(STANDBY) + 접속 절차 끝(이동 불가 이유 없음)까지
            // 브리지는 대기가 된 뒤에도 그리퍼 부팅을 2 초 기다리며 「접속하는 중」 - 그때 누른 홈은 거부된다
            const auto s = real_->state();
            const bool ready = s && s->connected && s->robot_state_ok && !s->connecting && s->why_not_ready.empty();
            if (!ready && elapsed() < 60) {return;}
            report("가상 컨트롤러 접속 (대기, 움직일 수 있음)", ready);
            chk_arm_->setChecked(true);
            mark();
            selftest_step_ = 2;
            return;
          }
        case 2: {                                   // 조작 허용이 브리지에 반영되면 홈
            const auto s = real_->state();
            if (!(s && s->armed) && elapsed() < 5) {return;}
            report("조작 허용 (브리지 armed)", s && s->armed);
            mark();
            onHome();
            selftest_step_ = 3;
            return;
          }
        case 3: {
            const int r = logHas("홈 이동");
            if (r == 0 && elapsed() < 60) {return;}
            report("홈 이동", r == 1);
            mark();
            real_->calibration(CAL::LOAD);
            selftest_step_ = 4;
            return;
          }
        case 4: {                                   // 보정 불러오기 -> 빨강 큐브 픽셀 클릭 + 빈 자리 클릭
            if (!real_->calibReady() && elapsed() < 10) {return;}
            report("보정 불러오기", real_->calibReady());
            // 지금 카메라에 보이는 큐브 하나 (빨강이 보이면 빨강). 팔 아래에 있는 큐브는 가려져 안 보일 수 있다
            const auto px = node_->snapshot().px_objects;
            const aris3_interfaces::msg::DetectedObject * pick = nullptr;
            for (const auto & o : px) {
              if (!pick || o.color == "red") {pick = &o;}
            }
            const bool found = pick != nullptr;
            if (pick) {selectPixel(pick->pose.pose.position.x, pick->pose.pose.position.y, 40, "시험 클릭");}
            // 빈 자리: 후보 중 모든 검출에서 80 px 넘게 떨어진 첫 자리 (앞 시험이 큐브를 옮겨 두었을 수 있음)
            std::array<double, 2> spot{560, 300};
            for (const auto & c : {std::array<double, 2>{560, 300}, std::array<double, 2>{700, 320},
                std::array<double, 2>{460, 320}, std::array<double, 2>{640, 190}})
            {
              bool clear = true;
              for (const auto & o : px) {
                clear &= std::hypot(o.pose.pose.position.x - c[0], o.pose.pose.position.y - c[1]) > 80;
              }
              if (clear) {spot = c; break;}
            }
            selectPixel(spot[0], spot[1], 40, "시험 클릭");
            report("클릭으로 큐브·놓을 자리 고름", found && pick_px_ && place_px_);
            mark();
            pickAndPlace(QStringLiteral("시험"));
            selftest_step_ = 5;
            return;
          }
        case 5: {
            const int r = logHas("집어서 옮기기");
            if (r == 0 && elapsed() < 120) {return;}
            report("집어서 옮기기 (가상 컨트롤러 + 시뮬레이션 그리퍼)", r == 1);
            // 이동 중 정지: 홈으로 가다가 정지
            mark();
            onHome();
            QTimer::singleShot(600, this, [this]() {onStop();});
            selftest_step_ = 6;
            return;
          }
        case 6: {
            const int r = logHas("홈 이동");
            if (r == 0 && elapsed() < 30) {return;}
            report("이동 중 정지 -> 홈 이동이 끝나지 않고 멈춤", r == -1);
            chk_arm_->setChecked(false);
            selftest_timer_.stop();
            auto_yes_ = false;
            done(selftest_ok_);
            return;
          }
      }
    });
  selftest_timer_.start(500);
}

}  // namespace aris
