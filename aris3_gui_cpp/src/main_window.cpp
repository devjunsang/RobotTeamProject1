// ARIS 메인 창 구현 (main_window.hpp 설명 참고).
#include "aris3_gui_cpp/main_window.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QShortcut>
#include <QTabWidget>
#include <QVBoxLayout>

#include <cmath>
#include <functional>

#include "aris3_gui_cpp/theme.hpp"

namespace aris
{

namespace
{
// 단계 이름 -> 한국어 설명 (aris3_gui STAGE_KO 와 같음)
const std::map<std::string, QString> kStageKo = {
  {"VALIDATE", "요청·장비 확인"}, {"OBSERVE", "관찰 자세"}, {"PERCEIVE", "물체 인식"}, {"PLAN", "경로 계획"},
  {"PICK", "집기"}, {"PLACE", "놓기"}, {"VERIFY", "결과 확인"}};
// 색 이름 -> 한국어
const std::map<std::string, QString> kColorKo = {{"red", "빨강"}, {"blue", "파랑"}, {"yellow", "노랑"}, {"green", "초록"}};

QString ko(const std::string & color)
{
  auto it = kColorKo.find(color);
  return it == kColorKo.end() ? QString::fromStdString(color) : it->second;
}

// Space 가 버튼을 「누르지」 않게 (Space = 정지)
template<typename W>
W * nofocus(W * w)
{
  w->setFocusPolicy(Qt::NoFocus);
  return w;
}
}  // namespace

MainWindow::MainWindow(std::shared_ptr<GuiNode> node, Backend * backend, QWidget * parent)
: QMainWindow(parent), node_(std::move(node)), backend_(backend)
{
  real_ = dynamic_cast<RealBackend *>(backend_);      // 실물 모드면 실물 전용 화면(집기·보정·손짓)을 더한다
  setWindowTitle(QStringLiteral("ARIS — ARIS3 Control Station (C++) — %1").arg(backend_->title()));
  setWindowIcon(QIcon(":/aris/aris_mark_256.png"));
  auto * central = new QWidget;
  auto * root = new QVBoxLayout(central);
  root->setContentsMargins(10, 10, 10, 10);
  root->setSpacing(10);
  root->addWidget(buildHeader());

  auto * body = new QHBoxLayout;
  body->setSpacing(10);
  body->addWidget(buildLeftColumn(), 0);
  auto * right = new QVBoxLayout;
  right->setSpacing(10);
  auto * top = new QHBoxLayout;
  top->setSpacing(10);
  top->addWidget(buildCameraPanel(), 3);
  auto * side = new QVBoxLayout;
  side->setSpacing(10);
  // 작업 공간 지도는 작게, 작업 큐·단계 표(7 줄)가 다 보이게
  side->addWidget(buildWorkspacePanel(), 2);
  side->addWidget(buildQueuePanel(), 5);
  top->addLayout(side, 2);
  right->addLayout(top, 3);
  auto * bottom = new QHBoxLayout;
  bottom->setSpacing(10);
  bottom->addWidget(buildLogPanel(), 3);
  bottom->addWidget(buildVoicePanel(), 2);
  if (real_) {
    // 실물 음성은 「실물 집기」 순서로 이어지고 매번 확인창 -> 구역·자동 실행은 쓰지 않음
    cmb_voice_zone_->setVisible(false);
    chk_voice_auto_->setVisible(false);
    bottom->addWidget(buildRealPickPanel(), 3);
    camera_->onPixelClick = [this](double u, double v) {selectPixel(u, v, 40, "클릭");};
  }
  bottom->addWidget(buildJogPanel(), 2);
  right->addLayout(bottom, 2);
  body->addLayout(right, 1);
  root->addLayout(body, 1);
  setCentralWidget(central);

  // 정지 단축키: 어디서나 (Space, Esc)
  for (auto key : {Qt::Key_Space, Qt::Key_Escape}) {
    auto * sc = new QShortcut(QKeySequence(key), this);
    sc->setContext(Qt::ApplicationShortcut);
    connect(sc, &QShortcut::activated, this, [this, key]() {
        // Space 는 문장 입력창에 글을 치는 중이면 띄어쓰기 (Esc·정지 버튼은 항상 정지)
        auto * w = qobject_cast<QLineEdit *>(QApplication::focusWidget());
        if (key == Qt::Key_Space && w && !node_->taskRunning()) {
          w->insert(" ");
          return;
        }
        onStop();
      });
  }

  connect(&timer_, &QTimer::timeout, this, &MainWindow::refresh);
  timer_.start(100);
  // 카메라는 따로 빠르게 (상태 갱신 100 ms 에 묶이면 최대 10 Hz)
  connect(&cam_timer_, &QTimer::timeout, this, &MainWindow::refreshCamera);
  cam_timer_.start(33);
}

// ======================================================================== 상단 줄
QWidget * MainWindow::buildHeader()
{
  auto * bar = new QFrame;
  bar->setObjectName("header");
  bar->setMinimumHeight(70);
  auto * row = new QHBoxLayout(bar);
  row->setContentsMargins(14, 6, 10, 6);
  auto * left = new QVBoxLayout;
  title_ = new QLabel(QStringLiteral("<b style='font-size:15pt'>E0509 Control Station</b>"));
  auto * sub = new QLabel(QStringLiteral("Doosan E0509 + RH-P12-RN-A  |  ROS 2 Jazzy + Gazebo + MoveIt 2"));
  sub->setObjectName("subtitle");
  left->addWidget(title_);
  left->addWidget(sub);
  row->addLayout(left);
  auto * chip = new QLabel(!backend_->isReal() ? QStringLiteral("SIMULATION") :
    backend_->isVirtual() ? QStringLiteral("REAL · 가상 컨트롤러") : QStringLiteral("REAL ROBOT"));
  chip->setObjectName("modeChip");
  row->addWidget(chip);
  row->addStretch(1);

  auto btn = [&](const QString & text, auto slot, const QString & name = QString()) {
      auto * b = nofocus(new QPushButton(text));
      b->setMinimumHeight(40);
      b->setMinimumWidth(100);
      if (!name.isEmpty()) {b->setObjectName(name);}
      connect(b, &QPushButton::clicked, this, slot);
      row->addWidget(b);
      return b;
    };
  btn_connect_ = btn(QStringLiteral("Connect"), &MainWindow::onConnect);
  btn(QStringLiteral("Disconnect"), &MainWindow::onDisconnect);
  btn_home_ = btn(QStringLiteral("⌂ Home"), &MainWindow::onHome);
  btn_start_ = btn(QStringLiteral("▶ Start Task"), &MainWindow::onStartQueue, "gold");
  btn(QStringLiteral("■ Stop"), &MainWindow::onStop);
  auto * estop = btn(QStringLiteral("⚠ STOP · Space / Esc"), &MainWindow::onStop, "estop");
  estop->setMinimumHeight(46);
  estop->setToolTip(QStringLiteral("소프트웨어 정지: 작업 취소 + 이동 중단. 인증된 비상정지 장치를 대신하지 않습니다."));

  // 로고: 줄 배치와 상관없이 창 가운데에 오도록 겹쳐 놓는다 (resizeEvent 에서 위치 갱신)
  header_ = bar;
  logo_ = new LogoBadge(bar);
  logo_->raise();
  return bar;
}

void MainWindow::placeLogo()
{
  if (!logo_ || !header_) {return;}
  const QSize s = logo_->sizeHint();
  int x = (header_->width() - s.width()) / 2;
  // 가장 왼쪽 버튼(Connect)과 왼쪽 제목 글자와 겹치지 않게
  if (btn_connect_) {x = std::min(x, btn_connect_->geometry().left() - s.width() - 12);}
  if (title_) {x = std::max(x, title_->geometry().left() + title_->sizeHint().width() + 120);}
  logo_->setGeometry(std::max(0, x), (header_->height() - s.height()) / 2, s.width(), s.height());
}

void MainWindow::resizeEvent(QResizeEvent * ev)
{
  QMainWindow::resizeEvent(ev);
  QTimer::singleShot(0, this, &MainWindow::placeLogo);     // 배치가 끝난 뒤 버튼 위치로 계산
}

// ======================================================================== 왼쪽 열
QWidget * MainWindow::buildLeftColumn()
{
  auto * w = new QWidget;
  w->setFixedWidth(310);
  auto * col = new QVBoxLayout(w);
  col->setContentsMargins(0, 0, 0, 0);
  col->setSpacing(10);

  auto * sys = new Panel(QStringLiteral("SYSTEM STATUS  시스템 상태"));
  auto * g = new QGridLayout;
  const std::vector<std::pair<std::string, QString>> items = {
    {"conn", "연결"}, {"robot", "로봇/시뮬레이터"}, {"camera", "카메라"}, {"detect", "물체 인식"},
    {"gripper", "그리퍼"}, {"moveit", "MoveIt 2"}, {"tm", "Task Manager"}, {"voice", "음성 인식"},
    {"gesture", "손짓 인식"}};
  int r = 0;
  for (const auto & [key, name] : items) {
    auto * n = new QLabel(name);
    n->setStyleSheet("color:#9aa7b4");
    g->addWidget(n, r, 0);
    auto * v = new QLabel(QStringLiteral("—"));
    v->setTextFormat(Qt::RichText);
    g->addWidget(v, r++, 1);
    sys_[key] = v;
  }
  sys->body()->addLayout(g);
  col->addWidget(sys);

  auto * rob = new Panel(QStringLiteral("ROBOT STATE  로봇 상태"));
  rob->body()->addWidget(new QLabel(!backend_->isReal() ? QStringLiteral("<b>MoveIt 2 (시뮬레이션)</b>") :
    backend_->isVirtual() ? QStringLiteral("<b>두산 가상 컨트롤러 (실물 경로 시뮬레이션)</b>") :
    QStringLiteral("<b>두산 (실물)</b>")));
  lbl_task_state_ = new QLabel(QStringLiteral("작업 상태: —"));
  rob->body()->addWidget(lbl_task_state_);
  lbl_joints_ = new QLabel(QStringLiteral("—"));
  lbl_joints_->setFont(QFont("DejaVu Sans Mono", 9));
  rob->body()->addWidget(lbl_joints_);
  rob->body()->addWidget(new QLabel(QStringLiteral("<span style='color:#9aa7b4'>손끝 TCP [m, deg] (world)</span>")));
  lbl_tcp_ = new QLabel(QStringLiteral("—"));
  lbl_tcp_->setFont(QFont("DejaVu Sans Mono", 9));
  rob->body()->addWidget(lbl_tcp_);
  col->addWidget(rob);

  auto * grip = new Panel(QStringLiteral("GRIPPER  그리퍼"));
  lbl_grip_ = new QLabel(QStringLiteral("—"));
  bar_grip_ = new QProgressBar;
  bar_grip_->setRange(0, 100);
  bar_grip_->setFormat(QStringLiteral("닫힘 %p%"));
  grip->body()->addWidget(lbl_grip_);
  grip->body()->addWidget(bar_grip_);
  auto * gr = new QHBoxLayout;
  btn_gopen_ = nofocus(new QPushButton(QStringLiteral("열기")));
  btn_gclose_ = nofocus(new QPushButton(QStringLiteral("닫기 (잡기)")));
  connect(btn_gopen_, &QPushButton::clicked, this, [this]() {if (armOk("그리퍼")) {backend_->gripper(false);}});
  connect(btn_gclose_, &QPushButton::clicked, this, [this]() {if (armOk("그리퍼")) {backend_->gripper(true);}});
  gr->addWidget(btn_gopen_);
  gr->addWidget(btn_gclose_);
  grip->body()->addLayout(gr);
  col->addWidget(grip);

  auto * cam = new Panel(QStringLiteral("CAMERA  카메라 정보"));
  auto * cl = new QLabel(!backend_->isReal() ? QStringLiteral("시뮬레이션: 탁자 위 2.0 m 고정 카메라 (보정 없음)") :
    QStringLiteral("실물 카메라: 픽셀 -> 로봇 좌표 보정 사용 (「실물 집기」 패널, 브리지가 계산)"));
  cl->setWordWrap(true);
  cl->setStyleSheet("color:#9aa7b4");
  cam->body()->addWidget(cl);
  col->addWidget(cam);
  col->addStretch(1);
  return w;
}

// ======================================================================== 카메라
QWidget * MainWindow::buildCameraPanel()
{
  auto * p = new Panel(QStringLiteral("TOP VIEW  카메라"));
  auto * top = new QHBoxLayout;
  top->addStretch(1);
  chk_detect_ = nofocus(new QCheckBox(QStringLiteral("검출 표시")));
  chk_detect_->setChecked(true);
  connect(chk_detect_, &QCheckBox::toggled, this, [this](bool on) {camera_->setShowDetections(on);});
  top->addWidget(chk_detect_);
  p->body()->addLayout(top);
  camera_ = new CameraView;
  camera_->onWorldClick = [this](double x, double y) {onCameraClick(x, y);};
  p->body()->addWidget(camera_, 1);
  lbl_cam_info_ = new QLabel(QStringLiteral("—"));
  lbl_cam_info_->setStyleSheet("color:#9aa7b4; font-family:'DejaVu Sans Mono'; font-size:9pt");
  p->body()->addWidget(lbl_cam_info_);
  return p;
}

// ======================================================================== 작업 큐
QWidget * MainWindow::buildQueuePanel()
{
  auto * p = new Panel(QStringLiteral("TASK QUEUE  작업 큐 · 단계"));
  auto * add = new QHBoxLayout;
  cmb_color_ = nofocus(new QComboBox);
  for (const std::string c : {"red", "blue", "yellow"}) {cmb_color_->addItem(ko(c) + " 큐브", QString::fromStdString(c));}
  cmb_zone_ = nofocus(new QComboBox);
  for (const char * z : {"A", "B", "C"}) {cmb_zone_->addItem(QStringLiteral("구역 %1").arg(z), z);}
  chk_dry_ = nofocus(new QCheckBox(QStringLiteral("계획만")));
  auto * b_add = nofocus(new QPushButton(QStringLiteral("+ 추가")));
  connect(b_add, &QPushButton::clicked, this, &MainWindow::onQueueAdd);
  auto * b_del = nofocus(new QPushButton(QStringLiteral("삭제")));
  connect(b_del, &QPushButton::clicked, this, [this]() {
      const int row = lst_queue_->currentRow();
      if (row >= 0 && row < static_cast<int>(queue_.size())) {
        queue_.erase(queue_.begin() + row);
        delete lst_queue_->takeItem(row);
      }
    });
  add->addWidget(cmb_color_);
  add->addWidget(new QLabel(QStringLiteral("→")));
  add->addWidget(cmb_zone_);
  add->addWidget(chk_dry_);
  add->addWidget(b_add);
  add->addWidget(b_del);
  p->body()->addLayout(add);
  // 화면에서 고르기: 카메라 클릭 (큐브 = 이거, 빈 곳 = 저기)
  auto * pick = new QHBoxLayout;
  lbl_sel_ = new QLabel(QStringLiteral("화면 선택: 카메라에서 큐브 클릭(이거) · 빈 곳 클릭(저기)"));
  lbl_sel_->setStyleSheet("color:#5ff4ff");
  lbl_sel_->setWordWrap(true);
  btn_sel_run_ = nofocus(new QPushButton(QStringLiteral("▶ 선택한 물체를 그 자리로")));
  btn_sel_run_->setObjectName("gold");
  connect(btn_sel_run_, &QPushButton::clicked, this, &MainWindow::runSelection);
  auto * b_clear = nofocus(new QPushButton(QStringLiteral("선택 해제")));
  connect(b_clear, &QPushButton::clicked, this, &MainWindow::clearSelection);
  pick->addWidget(lbl_sel_, 1);
  pick->addWidget(btn_sel_run_);
  pick->addWidget(b_clear);
  p->body()->addLayout(pick);
  lst_queue_ = new QListWidget;
  lst_queue_->setMaximumHeight(52);
  p->body()->addWidget(lst_queue_);

  const auto & names = stageNames();
  tbl_stage_ = new QTableWidget(static_cast<int>(names.size()), 3);
  tbl_stage_->setHorizontalHeaderLabels({QStringLiteral("단계"), QStringLiteral("상태"), QStringLiteral("시간")});
  tbl_stage_->verticalHeader()->setVisible(false);
  tbl_stage_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  tbl_stage_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tbl_stage_->setFocusPolicy(Qt::NoFocus);
  tbl_stage_->verticalHeader()->setDefaultSectionSize(22);
  tbl_stage_->setMinimumHeight(22 * static_cast<int>(names.size()) + 28);   // 단계 7 줄이 항상 다 보이게
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    tbl_stage_->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(names[i]) + "  " + kStageKo.at(names[i])));
    tbl_stage_->setItem(i, 1, new QTableWidgetItem(QStringLiteral("대기")));
    tbl_stage_->setItem(i, 2, new QTableWidgetItem(QString()));
  }
  p->body()->addWidget(tbl_stage_, 1);
  bar_task_ = new QProgressBar;
  bar_task_->setRange(0, 100);
  p->body()->addWidget(bar_task_);
  lbl_task_msg_ = new QLabel;
  lbl_task_msg_->setWordWrap(true);
  p->body()->addWidget(lbl_task_msg_);
  return p;
}

// ======================================================================== 로그
QWidget * MainWindow::buildLogPanel()
{
  auto * p = new Panel(QStringLiteral("LOGS  로그"));
  auto * top = new QHBoxLayout;
  top->addStretch(1);
  chk_log_aris_ = nofocus(new QCheckBox(QStringLiteral("ARIS 로그만")));
  chk_log_aris_->setChecked(true);
  auto * clear = nofocus(new QPushButton(QStringLiteral("지우기")));
  top->addWidget(chk_log_aris_);
  top->addWidget(clear);
  p->body()->addLayout(top);
  txt_log_ = new QPlainTextEdit;
  txt_log_->setReadOnly(true);
  txt_log_->setFocusPolicy(Qt::NoFocus);
  txt_log_->setMaximumBlockCount(500);
  txt_log_->setFont(QFont("DejaVu Sans Mono", 9));
  connect(clear, &QPushButton::clicked, txt_log_, &QPlainTextEdit::clear);
  p->body()->addWidget(txt_log_, 1);
  return p;
}

// ======================================================================== 음성
QWidget * MainWindow::buildVoicePanel()
{
  auto * p = new Panel(QStringLiteral("VOICE  음성 명령"));
  lbl_voice_state_ = new QLabel(QStringLiteral("—"));
  p->body()->addWidget(lbl_voice_state_);
  // 음성 명령 모드 ON/OFF (2026-11-10): 켜 두면 계속 듣고 「아리스 ...」 로 시작하는 말만 명령으로 받는다
  btn_talk_ = nofocus(new QPushButton(QStringLiteral("🎙  음성 명령 모드: OFF")));
  btn_talk_->setObjectName("voice");                 // 붉은 판 + 선명한 금색 글자 (theme.cpp QPushButton#voice)
  btn_talk_->setCheckable(true);
  btn_talk_->setMinimumHeight(42);
  btn_talk_->setToolTip(QStringLiteral("켜면 마이크가 계속 듣습니다. 「아리스 빨간 도형 잡아」 처럼 앞에 「아리스」 를 붙여 "
                                       "말하세요.\n「아리스」 로 시작하지 않는 말은 무시합니다."));
  connect(btn_talk_, &QPushButton::clicked, this, [this](bool on) {
      node_->voiceActive(on);
      node_->log("INFO", on ? "음성 명령 모드 켬 - 「아리스 ...」 로 말하세요" : "음성 명령 모드 끔");
    });
  p->body()->addWidget(btn_talk_);
  auto * row = new QHBoxLayout;
  txt_cmd_ = new QLineEdit;
  txt_cmd_->setPlaceholderText(QStringLiteral("또는 문장 입력 (예: 아리스 빨간 도형 잡아)"));
  auto * b = nofocus(new QPushButton(QStringLiteral("해석")));
  auto send = [this]() {
      const QString t = txt_cmd_->text().trimmed();
      if (!t.isEmpty()) {
        node_->voiceText(t.toStdString());
        txt_cmd_->clear();
      }
    };
  connect(txt_cmd_, &QLineEdit::returnPressed, this, send);
  connect(b, &QPushButton::clicked, this, send);
  row->addWidget(txt_cmd_, 1);
  row->addWidget(b);
  p->body()->addLayout(row);
  lbl_heard_ = new QLabel(QStringLiteral("인식: —"));
  lbl_heard_->setWordWrap(true);
  lbl_heard_->setStyleSheet("background:#0b0708; border-left:3px solid #5ff4ff; padding:6px");
  lbl_parsed_ = new QLabel(QStringLiteral("해석: —"));
  lbl_parsed_->setWordWrap(true);
  p->body()->addWidget(lbl_heard_);
  p->body()->addWidget(lbl_parsed_);
  auto * row2 = new QHBoxLayout;
  cmb_voice_zone_ = nofocus(new QComboBox);
  for (const char * z : {"A", "B", "C"}) {cmb_voice_zone_->addItem(QStringLiteral("놓을 곳: 구역 %1").arg(z), z);}
  chk_voice_auto_ = nofocus(new QCheckBox(QStringLiteral("자동 실행")));
  btn_voice_run_ = nofocus(new QPushButton(QStringLiteral("실행")));
  connect(btn_voice_run_, &QPushButton::clicked, this, [this]() {onVoiceRun(false);});
  row2->addWidget(cmb_voice_zone_);
  row2->addWidget(chk_voice_auto_);
  row2->addWidget(btn_voice_run_);
  p->body()->addLayout(row2);
  p->body()->addStretch(1);
  return p;
}

QWidget * MainWindow::buildWorkspacePanel()
{
  auto * p = new Panel(QStringLiteral("WORKSPACE  작업 공간"));
  workspace_ = new WorkspaceView;
  p->body()->addWidget(workspace_, 1);
  return p;
}

// ======================================================================== 수동 조작
QWidget * MainWindow::buildJogPanel()
{
  auto * p = new Panel(QStringLiteral("MANUAL JOG  수동 조작"));
  chk_arm_ = nofocus(new QCheckBox(QStringLiteral("조작 허용 (주변이 비었을 때만 · 2분 뒤 잠김)")));
  chk_arm_->setStyleSheet("color:#f5c542; font-weight:600");
  connect(chk_arm_, &QCheckBox::toggled, this, [this](bool on) {
      last_cmd_ = std::chrono::steady_clock::now();
      arm_sent_at_ = last_cmd_;
      node_->log("INFO", on ? "조작 허용 켬" : "조작 잠금");
      backend_->setArmed(on);                        // 실물: 브리지 쪽 조작 허용도 같게 (이중 장치)
    });
  p->body()->addWidget(chk_arm_);
  if (real_) {
    // 실물: 손으로 움직이기 (콕핏 직접 교시 버튼을 누른 채 팔을 끌기 - 보정 점 찍기를 빠르게)
    chk_hand_ = nofocus(new QCheckBox(QStringLiteral("손으로 움직이기 (수동 모드)")));
    chk_hand_->setToolTip(QStringLiteral("켜면 로봇을 수동 모드로 바꿉니다. 팔 끝 콕핏의 직접 교시 버튼을 누른 채 팔을 끌어 "
                                         "움직일 수 있습니다.\n켜져 있는 동안 이동·그리퍼·집기 명령은 막힙니다."));
    connect(chk_hand_, &QCheckBox::clicked, this, &MainWindow::toggleHandGuide);
    p->body()->addWidget(chk_hand_);
  }

  auto * tabs = new QTabWidget;
  tabs->setFocusPolicy(Qt::NoFocus);
  auto jogBtn = [this](const QString & text, std::function<void()> fn) {
      auto * b = nofocus(new QPushButton(text));
      b->setMinimumHeight(32);
      connect(b, &QPushButton::clicked, this, fn);
      jog_buttons_.push_back(b);
      return b;
    };
  // TCP: 위 = +X (로봇 정면), 왼쪽 = +Y (작업 공간 지도와 같은 방향), 오른쪽 열 = Z
  auto * tcp = new QWidget;
  auto * g = new QGridLayout(tcp);
  struct Tcp {int r, c; const char * text; int axis, sign;};
  for (const Tcp & t : {Tcp{0, 1, "+X", 0, 1}, Tcp{2, 1, "−X", 0, -1}, Tcp{1, 0, "+Y", 1, 1}, Tcp{1, 2, "−Y", 1, -1},
      Tcp{0, 3, "+Z", 2, 1}, Tcp{2, 3, "−Z", 2, -1}})
  {
    g->addWidget(jogBtn(QString::fromUtf8(t.text), [this, t]() {
        if (armOk("TCP 이동")) {
          backend_->jogTcp(t.axis, t.sign * cmb_step_->currentData().toDouble(), sld_speed_->value());
        }
      }), t.r, t.c);
  }
  g->addWidget(jogBtn(QStringLiteral("⌂"), [this]() {onHome();}), 1, 1);
  tabs->addTab(tcp, QStringLiteral("TCP 이동"));
  // 관절: J1~J6 마다 − / +
  auto * jt = new QWidget;
  auto * gj = new QGridLayout(jt);
  for (int i = 0; i < 6; ++i) {
    gj->addWidget(new QLabel(QStringLiteral("J%1").arg(i + 1)), i / 2, (i % 2) * 3);
    for (int k = 0; k < 2; ++k) {
      const int sign = k == 0 ? -1 : 1;
      auto * b = jogBtn(sign < 0 ? QStringLiteral("−") : QStringLiteral("+"), [this, i, sign]() {
          if (armOk("관절 이동")) {
            backend_->jogJoint(i, sign * cmb_step_->currentData().toDouble(), sld_speed_->value());
          }
        });
      b->setFixedWidth(40);
      gj->addWidget(b, i / 2, (i % 2) * 3 + 1 + k);
    }
  }
  tabs->addTab(jt, QStringLiteral("Joint 이동"));
  p->body()->addWidget(tabs);

  auto * row = new QHBoxLayout;
  cmb_step_ = nofocus(new QComboBox);
  for (int s : {1, 5, 10, 20}) {cmb_step_->addItem(QStringLiteral("%1 mm / °").arg(s), s);}
  cmb_step_->setCurrentIndex(2);
  sld_speed_ = new QSlider(Qt::Horizontal);
  sld_speed_->setRange(5, 100);
  sld_speed_->setValue(30);
  sld_speed_->setFocusPolicy(Qt::NoFocus);
  lbl_speed_ = new QLabel(QStringLiteral("30 %"));
  connect(sld_speed_, &QSlider::valueChanged, this, [this](int v) {lbl_speed_->setText(QStringLiteral("%1 %").arg(v));});
  row->addWidget(new QLabel(QStringLiteral("스텝")));
  row->addWidget(cmb_step_);
  row->addWidget(new QLabel(QStringLiteral("속도")));
  row->addWidget(sld_speed_, 1);
  row->addWidget(lbl_speed_);
  p->body()->addLayout(row);
  if (real_) {
    // 실물: 한계 동작·충돌 감지로 멈췄을 때 복구 후 홈, 지금 로봇 상태
    auto * rrow = new QHBoxLayout;
    btn_recover_ = nofocus(new QPushButton(QStringLiteral("⟲ 복구 후 홈")));
    btn_recover_->setToolTip(QStringLiteral("안전 정지 해제(또는 서보 켜기) -> 자동 모드 -> 홈 자세 [0, 0, 90, 0, 90, 0]°.\n"
                                            "비상정지·복구 모드는 로봇 쪽에서 직접 풀어야 합니다."));
    connect(btn_recover_, &QPushButton::clicked, this, &MainWindow::recoverHome);
    lbl_robot_state_ = new QLabel(QStringLiteral("로봇 상태: —"));
    rrow->addWidget(btn_recover_);
    rrow->addWidget(lbl_robot_state_, 1);
    p->body()->addLayout(rrow);
  }
  lbl_jog_reason_ = new QLabel;
  lbl_jog_reason_->setWordWrap(true);
  lbl_jog_reason_->setStyleSheet("color:#f2a33a");
  p->body()->addWidget(lbl_jog_reason_);
  p->body()->addStretch(1);
  return p;
}

// ======================================================================== 동작
bool MainWindow::ask(const QString & title, const QString & text)
{
  if (auto_yes_) {                                   // 시험용 (--selftest)
    node_->log("INFO", "[시험] 확인창 자동 「예」: " + title.toStdString());
    return true;
  }
  if (voice_direct_) {                               // 음성 명령은 묻지 않고 바로 실행 (조작 허용·정지는 그대로)
    node_->log("INFO", "음성 명령 바로 실행: " + title.toStdString());
    return true;
  }
  dialog_open_ = true;                               // 떠 있는 동안 손짓·음성이 새 확인창을 띄우지 않게
  const bool yes = QMessageBox::question(this, title, text, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) ==
    QMessageBox::Yes;
  dialog_open_ = false;
  return yes;
}

void MainWindow::onConnect()
{
  if (backend_->isReal() && !ask(QStringLiteral("로봇 접속"),
      QStringLiteral("로봇에 접속합니다. 서보가 켜지고 펜던트의 제어 권한을 가져옵니다.\n팔은 움직이지 않습니다.\n\n"
                     "로봇 옆에 사람이 있고 비상정지를 누를 수 있습니까?")))
  {
    return;
  }
  backend_->connectRobot();
}

void MainWindow::onDisconnect()
{
  const QString msg = backend_->isReal() ? QStringLiteral("접속을 끊으면 로봇 서보가 꺼집니다(브레이크가 팔을 잡습니다).") :
    QStringLiteral("시뮬레이션(Gazebo·MoveIt·Task Manager)을 종료합니다.");
  if (ask(QStringLiteral("Disconnect"), msg + QStringLiteral("\n\n계속할까요?"))) {
    chk_arm_->setChecked(false);
    queue_.clear();
    lst_queue_->clear();
    queue_running_ = false;
    backend_->disconnectRobot();
  }
}

void MainWindow::onHome()
{
  if (armOk("홈 이동")) {backend_->home(sld_speed_->value());}
}

QString MainWindow::whyNotReady()
{
  return backend_->whyNotReady();
}

bool MainWindow::armOk(const std::string & what)
{
  if (!chk_arm_->isChecked()) {
    node_->log("WARN", what + ": 「조작 허용」 을 먼저 체크하세요");
    return false;
  }
  const QString why = whyNotReady();
  if (!why.isEmpty()) {
    node_->log("WARN", what + ": " + why.toStdString());
    return false;
  }
  last_cmd_ = std::chrono::steady_clock::now();
  return true;
}

// ---- 화면 선택 (카메라 클릭)
void MainWindow::onCameraClick(double x, double y)
{
  // 가장 가까운 물체가 4 cm 안이면 그 물체(이거), 아니면 그 자리를 놓을 곳(저기)으로. 순서는 상관없다.
  const auto objs = node_->snapshot().objects;
  const aris3_interfaces::msg::DetectedObject * near = nullptr;
  double best = 1e9;
  for (const auto & o : objs) {
    if (o.id.empty()) {continue;}
    const double d = std::hypot(o.pose.pose.position.x - x, o.pose.pose.position.y - y);
    if (d < best) {best = d; near = &o;}
  }
  char buf[96];
  if (near && best < 0.04) {
    sel_id_ = near->id;
    node_->log("INFO", "선택(이거): 물체 " + sel_id_ + (sel_place_ ? "" : "  → 놓을 빈 자리를 클릭하세요"));
  } else {
    sel_place_ = std::array<double, 2>{x, y};
    std::snprintf(buf, sizeof(buf), "선택(저기): 놓을 자리 (%.3f, %.3f)", x, y);
    node_->log("INFO", std::string(buf) + (sel_id_.empty() ? "  → 옮길 큐브를 클릭하거나 「아리스 파란 큐브 저기로」" :
      "  → 「▶ 선택한 물체를 그 자리로」 또는 「아리스 이거 저기로 옮겨줘」"));
  }
  updateSelectionLabel();
}

void MainWindow::updateSelectionLabel()
{
  const QString place = sel_place_ ? QStringLiteral("(%1, %2)").arg((*sel_place_)[0], 0, 'f', 3).arg((*sel_place_)[1], 0, 'f', 3) :
    QStringLiteral("—");
  lbl_sel_->setText(QStringLiteral("화면 선택: 물체 %1  →  놓을 자리 %2")
    .arg(sel_id_.empty() ? QStringLiteral("—") : QString::fromStdString(sel_id_), place));
}

void MainWindow::clearSelection()
{
  sel_id_.clear();
  sel_place_.reset();
  updateSelectionLabel();
}

void MainWindow::runSelection()
{
  if (sel_id_.empty() || !sel_place_) {return;}
  QueuedTask t;
  t.target_id = sel_id_;
  t.place = sel_place_;
  t.dry = chk_dry_->isChecked();
  queue_.insert(queue_.begin(), t);
  addQueueItem(t, true);
  clearSelection();
  queue_running_ = true;
  nextTask();
}

void MainWindow::addQueueItem(const QueuedTask & t, bool front)
{
  const QString what = t.target_id.empty() ? ko(t.color) + " 큐브" : QString::fromStdString(t.target_id);
  const QString where = t.place ? QStringLiteral("(%1, %2)").arg((*t.place)[0], 0, 'f', 2).arg((*t.place)[1], 0, 'f', 2) :
    QStringLiteral("구역 ") + QString::fromStdString(t.zone);
  const QString text = what + " → " + where + (t.dry ? " (계획만)" : "");
  if (front) {lst_queue_->insertItem(0, text);} else {lst_queue_->addItem(text);}
}

void MainWindow::onStop()
{
  queue_.clear();
  lst_queue_->clear();
  queue_running_ = false;
  backend_->stop();          // 실물: 브리지 정지 + 작업 취소, 시뮬레이션: 작업 취소 + MoveIt 중단
}

void MainWindow::onQueueAdd()
{
  QueuedTask t;
  t.color = cmb_color_->currentData().toString().toStdString();
  t.zone = cmb_zone_->currentData().toString().toStdString();
  t.dry = chk_dry_->isChecked();
  queue_.push_back(t);
  addQueueItem(t, false);
}

void MainWindow::queueTask(const std::string & color, const std::string & zone, bool dry)
{
  QueuedTask t;
  t.color = color;
  t.zone = zone;
  t.dry = dry;
  queue_.push_back(t);
  addQueueItem(t, false);
}

void MainWindow::onStartQueue()
{
  if (backend_->isReal()) {
    QMessageBox::information(this, QStringLiteral("작업 실행"),
      QStringLiteral("실물 로봇의 자동 작업(Task Manager 두산 백엔드)은 아직 검증되지 않아 막아 두었습니다.\n"
                     "「실물 집기」 패널(클릭·손짓·음성)을 사용하세요."));
    return;
  }
  if (queue_.empty()) {onQueueAdd();}
  queue_running_ = true;
  nextTask();
}

void MainWindow::nextTask()
{
  if (!queue_running_ || node_->taskRunning()) {return;}
  if (queue_.empty()) {
    queue_running_ = false;
    node_->log("INFO", "작업 큐 완료");
    return;
  }
  const QueuedTask t = queue_.front();
  queue_.erase(queue_.begin());
  delete lst_queue_->takeItem(0);
  current_color_ = t.color;
  if (!node_->sendTask(t.target_id.empty() ? t.color : "", t.place ? "" : t.zone, t.dry, t.target_id,
    t.place.has_value(), t.place ? (*t.place)[0] : 0.0, t.place ? (*t.place)[1] : 0.0))
  {
    queue_running_ = false;
  }
}

void MainWindow::onVoiceRun(bool automatic)
{
  const auto v = node_->snapshot().voice;
  if (!v || !v->valid) {
    node_->log("WARN", "실행할 수 있는 음성 명령이 없음");
    return;
  }
  if (real_) {                                       // 실물: 「실물 집기」 순서로 (매번 확인창)
    realVoice(*v);
    return;
  }
  if (v->action == "stop") {                         // 보통은 받는 즉시 처리되지만, 「실행」 버튼으로 와도 정지
    onStop();
    return;
  }
  if (v->action == "resume" || v->action == "power_on" || v->action == "disconnect") {
    node_->log("WARN", "음성 「" + v->text + "」: 실물 모드(--mode real)에서만 지원 - 시뮬레이션 모드는 Connect 버튼을 쓰세요");
    node_->say("그 명령은 이 모드에서는 할 수 없어요.");
    return;
  }
  // 예전 시뮬레이션 모드(MoveIt)의 대답: 조작 허용이 꺼져 있으면 그 이유를, 아니면 「네, …」
  node_->say(!chk_arm_->isChecked() && v->action == "home" ? std::string("조작 허용을 먼저 켜 주세요.") :
    voiceReply(v->action, v->color, v->shape, v->refers_selected_place));
  if (v->action == "place") {
    node_->log("WARN", "「내려놔」(집은 자리에 놓기)는 Task Manager 의 pick/place 분리 후 지원 예정 - "
      "지금은 집기 명령이 놓기까지 한 번에 수행합니다");
    return;
  }
  if (v->action == "home") {                         // 「아리스 홈으로」: ⌂ Home 버튼과 같음
    node_->log("INFO", std::string("음성 명령 ") + (automatic ? "자동 " : "") + "실행: 홈으로 돌아가기");
    onHome();
    return;
  }
  QueuedTask t;
  // 무엇을: 「이거」 = 화면에서 고른 큐브, 아니면 말한 색
  if (v->refers_selected_object && v->color.empty()) {
    if (sel_id_.empty()) {
      node_->log("WARN", "「이거」 가 무엇인지 모름 - 먼저 카메라 화면에서 큐브를 클릭하세요");
      return;
    }
    t.target_id = sel_id_;
  } else {
    t.color = v->color;
  }
  // 어디에: 「저기」 = 화면에서 고른 자리, 아니면 구역 선택 상자
  if (v->refers_selected_place) {
    if (!sel_place_) {
      node_->log("WARN", "「저기」 가 어디인지 모름 - 먼저 카메라 화면에서 탁자의 빈 곳을 클릭하세요");
      return;
    }
    t.place = sel_place_;
  } else {
    t.zone = cmb_voice_zone_->currentData().toString().toStdString();
  }
  node_->log("INFO", std::string("음성 명령 ") + (automatic ? "자동 " : "") + "실행: " +
    (t.target_id.empty() ? t.color : t.target_id) + " -> " + (t.place ? "고른 자리" : "구역 " + t.zone));
  queue_.insert(queue_.begin(), t);
  addQueueItem(t, true);
  clearSelection();
  queue_running_ = true;
  nextTask();
}

// ======================================================================== 주기 갱신
void MainWindow::refreshCamera()
{
  const auto s = node_->snapshot();
  std::optional<PinholeCamera> cam;
  PinholeCamera c;
  if (node_->cameraTransform(c.t, c.q)) {cam = c;}    // 시뮬레이션 카메라 (실물 카메라는 TF 없음 -> 픽셀만)
  camera_->setSelection(real_ ? std::string() : sel_id_, sel_place_, s.task_running ? current_color_ : std::string());
  if (real_) {
    camera_->setPixelMarks(place_px_, real_->calibPendingPx());
  }
  camera_->setFrame(s.image, s.image_seq, s.objects, s.px_objects, cam);
}

void MainWindow::refresh()
{
  const Snapshot s = node_->snapshot();
  const StatusFlags f = node_->statusFlags();

  // ---- 작업이 끝났으면 큐의 다음 작업 (실패면 큐 중단)
  if (was_task_running_ && !s.task_running) {
    if (s.last_result && !s.last_result->first) {
      queue_running_ = false;
      queue_.clear();
      lst_queue_->clear();
    }
    nextTask();
  }
  was_task_running_ = s.task_running;

  // ---- 버튼·수동 조작 (조작 허용은 2분 동안 명령이 없으면 다시 잠근다)
  btn_connect_->setEnabled(!backend_->connected());
  btn_start_->setEnabled(!backend_->isReal() && f.task_manager && !s.task_running);
  if (chk_arm_->isChecked() &&
    std::chrono::steady_clock::now() - last_cmd_ > std::chrono::seconds(120))
  {
    chk_arm_->setChecked(false);
    node_->log("INFO", "120초 동안 명령이 없어 조작을 잠갔습니다");
  }
  QString why = whyNotReady();
  if (!chk_arm_->isChecked() && why.isEmpty()) {why = QStringLiteral("「조작 허용」 이 꺼져 있음");}
  lbl_jog_reason_->setText(why.isEmpty() ? QString() : QStringLiteral("이동 불가: ") + why);
  for (auto * b : jog_buttons_) {b->setEnabled(why.isEmpty());}
  btn_home_->setEnabled(why.isEmpty());
  const GripperStatus gs_ = backend_->gripperStatus();
  btn_gopen_->setEnabled(why.isEmpty() && (gs_.ok || gs_.warn));
  btn_gclose_->setEnabled(why.isEmpty() && (gs_.ok || gs_.warn));
  if (real_) {refreshReal(s);}
  btn_sel_run_->setEnabled(!sel_id_.empty() && sel_place_ && f.task_manager && !s.task_running);

  // ---- 시스템 상태
  sys_["conn"]->setText(theme::dot(backend_->connected(), backend_->describe(), true));
  sys_["robot"]->setText(theme::dot(f.joint_states, f.joint_states ?
    QStringLiteral("관절 상태 %1 Hz").arg(f.js_hz, 0, 'f', 0) : QStringLiteral("관절 상태 없음")));
  sys_["camera"]->setText(theme::dot(f.camera_hz > 1, f.camera_hz > 0 ?
    QStringLiteral("%1 Hz").arg(f.camera_hz, 0, 'f', 1) : QStringLiteral("영상 없음")));
  sys_["detect"]->setText(theme::dot(f.detections || f.px_detections, f.detections ?
    QStringLiteral("%1개").arg(s.objects.size()) : f.px_detections ? QStringLiteral("픽셀 %1개").arg(s.px_objects.size()) :
    QStringLiteral("결과 없음")));
  sys_["gripper"]->setText(theme::dot(gs_.ok, gs_.text, gs_.warn));
  sys_["moveit"]->setText(theme::dot(f.moveit, f.moveit ? QStringLiteral("준비") : QStringLiteral("없음")));
  sys_["tm"]->setText(theme::dot(f.task_manager, f.task_manager ? QStringLiteral("준비") : QStringLiteral("없음")));
  const QString vs = QString::fromStdString(s.voice_state);
  const bool voice_ok = f.voice && (vs == "ready" || vs == "listening" || vs == "recording" || vs == "recognizing");
  sys_["voice"]->setText(theme::dot(voice_ok, vs.isEmpty() ? QStringLiteral("없음") :
    vs == "listening" ? QStringLiteral("듣는 중 (아리스)") : vs, vs == "loading"));
  const QString gs = QString::fromStdString(s.gesture_state);
  sys_["gesture"]->setText(theme::dot(f.gesture && gs == "ready", gs.isEmpty() ? QStringLiteral("없음") : gs,
    gs == "loading"));

  // ---- 로봇 상태
  if (s.task_status) {
    static const char * names[] = {"대기", "작업 중", "오류"};
    lbl_task_state_->setText(QStringLiteral("작업 상태: %1 %2").arg(
        s.task_status->state < 3 ? names[s.task_status->state] : "?",
        QString::fromStdString(s.task_status->stage)));
  }
  const auto jd = backend_->jointsDeg();
  QString j;
  if (jd) {
    for (int i = 0; i < 6; ++i) {
      j += QStringLiteral("J%1 %2  ").arg(i + 1).arg((*jd)[i], 6, 'f', 1);
      if (i == 2) {j += "\n";}
    }
  }
  lbl_joints_->setText(jd ? j : QStringLiteral("—"));
  double tcp[3], rpy[3];
  const bool has_tcp = backend_->tcp(tcp, rpy);
  lbl_tcp_->setText(has_tcp ? QStringLiteral("X %1  Y %2  Z %3\nR %4  P %5  Y %6")
    .arg(tcp[0], 6, 'f', 3).arg(tcp[1], 6, 'f', 3).arg(tcp[2], 6, 'f', 3)
    .arg(rpy[0] * 180 / M_PI, 6, 'f', 1).arg(rpy[1] * 180 / M_PI, 6, 'f', 1).arg(rpy[2] * 180 / M_PI, 6, 'f', 1) :
    QStringLiteral("—"));
  // 작업 공간 지도: 시뮬레이션은 world 손끝, 실물은 베이스 좌표 손끝 (베이스 = 지도 원점)
  workspace_->setState(s.objects, has_tcp ? std::optional<std::array<double, 2>>(std::array<double, 2>{tcp[0], tcp[1]}) :
    std::nullopt, s.task_running ? current_color_ : std::string(), sel_place_);

  // ---- 그리퍼 (0 열림 ~ 1 닫힘)
  if (const auto gp = backend_->gripperPosition()) {
    bar_grip_->setValue(static_cast<int>(*gp * 100));
    lbl_grip_->setText((*gp < 0.05 ? QStringLiteral("열림") : *gp > 0.9 ? QStringLiteral("닫힘 (빈손)") :
      QStringLiteral("닫힘 (물체를 잡고 있음)")) +
      (s.gripper_msg.empty() ? QString() : "  ·  " + QString::fromStdString(s.gripper_msg)));
  }

  // ---- 카메라 정보
  lbl_cam_info_->setText(s.image.isNull() ? QStringLiteral("—") :
    QStringLiteral("RGB %1 × %2  |  %3 Hz  |  검출 %4개").arg(s.image.width()).arg(s.image.height())
    .arg(f.camera_hz, 0, 'f', 1).arg(s.objects.size() + s.px_objects.size()));

  // ---- 작업 단계
  const auto & names = stageNames();
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    const auto st = s.stages.state(names[i]);
    auto * item = tbl_stage_->item(i, 1);
    switch (st) {
      case StageState::RUN: item->setText(QStringLiteral("▶ 실행 중")); item->setForeground(theme::kGoldText); break;
      case StageState::DONE: item->setText(QStringLiteral("✓ 완료")); item->setForeground(theme::kCyan); break;
      case StageState::FAIL: item->setText(QStringLiteral("✗ 실패")); item->setForeground(theme::kError); break;
      default: item->setText(QStringLiteral("대기")); item->setForeground(theme::kMuted); break;
    }
    const double t = s.stages.seconds(names[i]);
    tbl_stage_->item(i, 2)->setText(t > 0 ? QStringLiteral("%1 s").arg(t, 0, 'f', 1) : QString());
  }
  if (s.task_running && s.feedback) {
    bar_task_->setValue(static_cast<int>(s.feedback->progress * 100));
    lbl_task_msg_->setText(QString::fromStdString(s.feedback->stage + ": " + s.feedback->detail));
    lbl_task_msg_->setStyleSheet(QString());
  } else if (s.last_result) {
    if (s.last_result->first) {bar_task_->setValue(100);}
    lbl_task_msg_->setText((s.last_result->first ? "✓ " : "✗ ") + QString::fromStdString(s.last_result->second));
    lbl_task_msg_->setStyleSheet(QStringLiteral("color:%1").arg(
        (s.last_result->first ? theme::kCyan : theme::kError).name()));
  }

  // ---- 음성
  static const std::map<QString, QString> vtext = {
    {"ready", "○ 준비 — 음성 명령 모드를 켜세요"}, {"listening", "● 듣는 중 — 「아리스 ...」 로 말하세요"},
    {"recording", "● 녹음 중..."}, {"recognizing", "⏳ 인식 중..."}, {"loading", "⏳ Whisper 모델 로딩 중"}};
  auto vt = vtext.find(vs);
  lbl_voice_state_->setText(vt == vtext.end() ? QStringLiteral("○ 음성 노드 없음 %1").arg(vs) : vt->second);
  // 버튼은 음성 노드의 실제 상태를 따른다 (듣는 중·인식 중 = ON)
  const bool listening = vs == "listening" || vs == "recognizing";
  btn_talk_->blockSignals(true);
  btn_talk_->setChecked(listening);
  btn_talk_->blockSignals(false);
  btn_talk_->setText(listening ? QStringLiteral("🎙  음성 명령 모드: ON — 듣는 중") :
    QStringLiteral("🎙  음성 명령 모드: OFF"));
  btn_talk_->setEnabled(vs == "ready" || listening);
  if (s.voice) {
    const int64_t stamp = static_cast<int64_t>(s.voice->stamp.sec) * 1000000000LL + s.voice->stamp.nanosec;
    if (stamp != last_voice_stamp_) {
      last_voice_stamp_ = stamp;
      lbl_heard_->setText(QStringLiteral("인식: 「%1」").arg(QString::fromStdString(s.voice->text)));
      lbl_parsed_->setText(QStringLiteral("해석: %1 %2 · %3 — %4").arg(ko(s.voice->color),
          QString::fromStdString(s.voice->shape.empty() ? "물체" : s.voice->shape),
          QString::fromStdString(s.voice->action.empty() ? "?" : s.voice->action),
          QStringLiteral("<span style='color:%1'>%2 %3</span>").arg(
            (s.voice->valid ? theme::kCyan : theme::kError).name(), s.voice->valid ? "✓" : "✗",
            QString::fromStdString(s.voice->message))));
      if (s.voice->valid && s.voice->action == "stop") {
        // 「아리스 멈춰」: 정지 버튼과 같다. 다른 검사 없이 바로 (인식까지 약 1 초 걸린다 - 비상정지 대신이 아님)
        node_->log("WARN", "음성 「" + s.voice->text + "」 -> 정지");
        onStop();
        node_->say(voiceReply("stop", "", "", false));
      } else if (s.voice->valid && real_) {
        // 실물: 인식되면 바로 확인창 (예 를 눌러야 움직임). 주기 갱신 밖에서 연다
        QTimer::singleShot(0, this, [this]() {onVoiceRun(true);});
      } else if (s.voice->valid && chk_voice_auto_->isChecked()) {
        onVoiceRun(true);
      }
      if (!s.voice->valid && !s.voice->text.empty()) {
        // 알아듣지 못한 말 (호출어 뒤 말이 명령이 아님 등). 녹음이 짧거나 인식 실패(글자 없음)면 말하지 않는다
        node_->say(voiceReply("", "", "", false));
      }
    }
  }
  btn_voice_run_->setEnabled(s.voice && s.voice->valid && !(real_ ? real_->pickRunning() : s.task_running));

  // ---- 로그
  const bool only = chk_log_aris_->isChecked();
  for (const auto & l : s.logs) {
    if (l.seq <= log_shown_) {continue;}
    log_shown_ = l.seq;
    if (only && !isArisLog(l.level, l.node, l.text)) {continue;}
    const QString color = l.level == "WARN" ? theme::kAmber.name() :
      (l.level == "ERROR" || l.level == "FATAL") ? theme::kError.name() : theme::kCyan.name();
    txt_log_->appendHtml(QStringLiteral("<span style='color:%1'>[%2]</span> <span style='color:#7d8a96'>[%3] [%4]</span> %5")
      .arg(color, QString::fromStdString(l.level), QString::fromStdString(l.time), QString::fromStdString(l.node),
      QString::fromStdString(l.text).toHtmlEscaped()));
  }
}

// ======================================================================== 시험용 (--selftest)
void MainWindow::runSelftest(std::function<void(bool)> done)
{
  // 1 초마다 다음 단계로: 각 이동은 로그에 「<이름>: 완료」 / 「작업 성공」 이 나오면 통과, 「실패」 면 실패.
  auto logHas = [this](const std::string & a, const std::string & b) -> int {
      for (const auto & l : node_->snapshot().logs) {
        if (l.seq <= selftest_log_mark_) {continue;}
        if (l.text.find(a) != std::string::npos && l.text.find(b) != std::string::npos) {return 1;}
        if (l.text.find(a) != std::string::npos && l.text.find("실패") != std::string::npos) {return -1;}
      }
      return 0;
    };
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
  // 이동 하나를 기다리는 단계: label 의 완료/실패가 로그에 나올 때까지 (최대 60 초)
  auto waitMove = [=](const std::string & label, const std::string & ok_word) {
      const int r = logHas(label, ok_word);
      if (r == 0 && elapsed() < 60) {return false;}
      report(label, r == 1);
      return true;
    };
  connect(&selftest_timer_, &QTimer::timeout, this, [=]() {
      switch (selftest_step_) {
        case 0:   // 준비: MoveIt·관절 상태·Task Manager
          if (!whyNotReady().isEmpty() && whyNotReady() != QStringLiteral("이전 수동 이동 진행 중")) {return;}
          if (!node_->statusFlags().task_manager) {return;}
          chk_arm_->setChecked(true);
          report("조작 허용", chk_arm_->isChecked());
          mark();
          onHome();
          selftest_step_ = 1;
          return;
        case 1:
          if (!waitMove("홈 자세", ": 완료")) {return;}
          mark();
          node_->jogTcp(2, 20, 0.3);                  // armOk 와 같은 검사는 위에서 통과 (조작 허용·준비)
          selftest_step_ = 2;
          return;
        case 2:
          if (!waitMove("TCP Z +20 mm", ": 완료")) {return;}
          mark();
          node_->jogJoint(0, 10, 0.3);
          selftest_step_ = 3;
          return;
        case 3:
          if (!waitMove("J1 +10°", ": 완료")) {return;}
          {
            // 화면 선택: 파랑 큐브 위치를 클릭한 것처럼, 그다음 빈 곳 (0.45, 0.25)
            const auto objs = node_->snapshot().objects;
            bool found = false;
            for (const auto & o : objs) {
              if (o.id == "blue_cube") {
                onCameraClick(o.pose.pose.position.x, o.pose.pose.position.y);
                found = true;
              }
            }
            // 빈 자리: 후보 중 모든 물체에서 8 cm 넘게 떨어진 첫 자리 (앞 시험이 큐브를 옮겨 두었을 수 있음)
            std::array<double, 2> spot{0.45, 0.25};
            for (const auto & c : {std::array<double, 2>{0.45, 0.25}, std::array<double, 2>{0.40, 0.15},
                std::array<double, 2>{0.35, -0.20}, std::array<double, 2>{-0.20, 0.40}})
            {
              bool clear = true;
              for (const auto & o : objs) {
                clear &= std::hypot(o.pose.pose.position.x - c[0], o.pose.pose.position.y - c[1]) > 0.08;
              }
              if (clear) {spot = c; break;}
            }
            onCameraClick(spot[0], spot[1]);
            char name[96];
            std::snprintf(name, sizeof(name), "화면 선택 (이거 = blue_cube, 저기 = (%.2f, %.2f))", spot[0], spot[1]);
            report(name, found && sel_id_ == "blue_cube" && sel_place_);
            mark();
            runSelection();
          }
          selftest_step_ = 4;
          return;
        case 4: {
            const int r = logHas("작업 ", "성공");
            if (r == 0 && elapsed() < 120) {return;}
            report("선택한 물체를 그 자리로 (작업)", r == 1);
            selftest_timer_.stop();
            done(selftest_ok_);
            return;
          }
      }
    });
  selftest_timer_.start(1000);
}

void MainWindow::closeEvent(QCloseEvent * ev)
{
  timer_.stop();
  cam_timer_.stop();
  if (backend_->isReal()) {
    backend_->shutdown();          // 이 화면이 띄운 브리지·음성·손짓만 끈다 (브리지가 시작한 로봇 접속도 끊김)
  } else if (backend_->ownsProcess() &&
    ask(QStringLiteral("종료"), QStringLiteral("이 화면이 실행한 시뮬레이션도 함께 끌까요?")))
  {
    backend_->shutdown();
  }
  QMainWindow::closeEvent(ev);
}

}  // namespace aris
