// ARIS 메인 창 구현 (main_window.hpp 설명 참고).
#include "aris3_gui_cpp/main_window.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QShortcut>
#include <QVBoxLayout>

#include <cmath>

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

MainWindow::MainWindow(std::shared_ptr<GuiNode> node, SimBackend * backend, QWidget * parent)
: QMainWindow(parent), node_(std::move(node)), backend_(backend)
{
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
  side->addWidget(buildPlaceholder(QStringLiteral("WORKSPACE  작업 공간"),
    QStringLiteral("위에서 본 2D 지도 - 다음 단계(M2)")), 1);
  side->addWidget(buildQueuePanel(), 4);
  top->addLayout(side, 2);
  right->addLayout(top, 3);
  auto * bottom = new QHBoxLayout;
  bottom->setSpacing(10);
  bottom->addWidget(buildLogPanel(), 3);
  bottom->addWidget(buildVoicePanel(), 2);
  bottom->addWidget(buildPlaceholder(QStringLiteral("MANUAL JOG  수동 조작"),
    QStringLiteral("조작 허용 · TCP·관절 조그 · 그리퍼 · 홈 - 다음 단계(M2)\n"
                   "그 전에는 aris3_gui(PyQt)의 수동 조작을 쓰세요.")), 2);
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
  auto * chip = new QLabel(QStringLiteral("SIMULATION"));
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
  rob->body()->addWidget(new QLabel(QStringLiteral("<b>MoveIt 2 (시뮬레이션)</b>")));
  lbl_task_state_ = new QLabel(QStringLiteral("작업 상태: —"));
  rob->body()->addWidget(lbl_task_state_);
  lbl_joints_ = new QLabel(QStringLiteral("—"));
  lbl_joints_->setFont(QFont("DejaVu Sans Mono", 9));
  rob->body()->addWidget(lbl_joints_);
  col->addWidget(rob);

  auto * grip = new Panel(QStringLiteral("GRIPPER  그리퍼"));
  lbl_grip_ = new QLabel(QStringLiteral("—"));
  bar_grip_ = new QProgressBar;
  bar_grip_->setRange(0, 100);
  bar_grip_->setFormat(QStringLiteral("닫힘 %p%"));
  grip->body()->addWidget(lbl_grip_);
  grip->body()->addWidget(bar_grip_);
  col->addWidget(grip);

  auto * cam = new Panel(QStringLiteral("CAMERA  카메라 정보"));
  auto * cl = new QLabel(QStringLiteral("시뮬레이션: 탁자 위 2.0 m 고정 카메라 (보정 없음)"));
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
  btn_talk_->setObjectName("gold");
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

QWidget * MainWindow::buildPlaceholder(const QString & title, const QString & text)
{
  auto * p = new Panel(title);
  auto * l = new QLabel(text);
  l->setWordWrap(true);
  l->setAlignment(Qt::AlignCenter);
  l->setStyleSheet("color:#9aa7b4");
  p->body()->addWidget(l, 1);
  return p;
}

// ======================================================================== 동작
bool MainWindow::ask(const QString & title, const QString & text)
{
  return QMessageBox::question(this, title, text, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) ==
         QMessageBox::Yes;
}

void MainWindow::onConnect()
{
  backend_->connectSim();
}

void MainWindow::onDisconnect()
{
  if (ask(QStringLiteral("Disconnect"),
      QStringLiteral("시뮬레이션(Gazebo·MoveIt·Task Manager)을 종료합니다.\n\n계속할까요?")))
  {
    queue_.clear();
    lst_queue_->clear();
    queue_running_ = false;
    backend_->disconnectSim();
  }
}

void MainWindow::onStop()
{
  queue_.clear();
  lst_queue_->clear();
  queue_running_ = false;
  node_->stopAll();
}

void MainWindow::onQueueAdd()
{
  QueuedTask t{cmb_color_->currentData().toString().toStdString(), cmb_zone_->currentData().toString().toStdString(),
    chk_dry_->isChecked()};
  queue_.push_back(t);
  lst_queue_->addItem(ko(t.color) + " 큐브 → 구역 " + QString::fromStdString(t.zone) + (t.dry ? " (계획만)" : ""));
}

void MainWindow::queueTask(const std::string & color, const std::string & zone, bool dry)
{
  queue_.push_back(QueuedTask{color, zone, dry});
  lst_queue_->addItem(ko(color) + " 큐브 → 구역 " + QString::fromStdString(zone) + (dry ? " (계획만)" : ""));
}

void MainWindow::onStartQueue()
{
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
  if (!node_->sendTask(t.color, t.zone, t.dry)) {queue_running_ = false;}
}

void MainWindow::onVoiceRun(bool automatic)
{
  const auto v = node_->snapshot().voice;
  if (!v || !v->valid) {
    node_->log("WARN", "실행할 수 있는 음성 명령이 없음");
    return;
  }
  if (v->action != "pick") {
    // 「내려놔」·「홈」·「이거/저기」 는 다음 단계 (aris3_gui 에는 있음)
    node_->log("WARN", "ARIS M1 은 「<색> 큐브 집어줘」 만 실행합니다 (그 밖의 명령은 aris3_gui 사용)");
    return;
  }
  if (v->refers_selected_object || v->refers_selected_place) {
    node_->log("WARN", "「이거/저기」(화면 선택)는 ARIS 다음 단계에서 지원");
    return;
  }
  const std::string zone = cmb_voice_zone_->currentData().toString().toStdString();
  node_->log("INFO", std::string("음성 명령 ") + (automatic ? "자동 " : "") + "실행: " + v->color + " -> 구역 " + zone);
  queue_.insert(queue_.begin(), QueuedTask{v->color, zone, false});
  lst_queue_->insertItem(0, ko(v->color) + " 큐브 → 구역 " + QString::fromStdString(zone));
  queue_running_ = true;
  nextTask();
}

// ======================================================================== 주기 갱신
void MainWindow::refreshCamera()
{
  const auto s = node_->snapshot();
  camera_->setFrame(s.image, s.image_seq, s.px_objects);
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

  // ---- 버튼
  btn_connect_->setEnabled(!backend_->connected());
  btn_start_->setEnabled(f.task_manager && !s.task_running);

  // ---- 시스템 상태
  sys_["conn"]->setText(theme::dot(backend_->connected(), backend_->describe(), true));
  sys_["robot"]->setText(theme::dot(f.joint_states, f.joint_states ?
    QStringLiteral("관절 상태 %1 Hz").arg(f.js_hz, 0, 'f', 0) : QStringLiteral("관절 상태 없음")));
  sys_["camera"]->setText(theme::dot(f.camera_hz > 1, f.camera_hz > 0 ?
    QStringLiteral("%1 Hz").arg(f.camera_hz, 0, 'f', 1) : QStringLiteral("영상 없음")));
  sys_["detect"]->setText(theme::dot(f.detections || f.px_detections, f.detections ?
    QStringLiteral("%1개").arg(s.objects.size()) : f.px_detections ? QStringLiteral("픽셀 %1개").arg(s.px_objects.size()) :
    QStringLiteral("결과 없음")));
  sys_["gripper"]->setText(theme::dot(f.gripper, f.gripper ? QStringLiteral("준비") : QStringLiteral("서비스 없음")));
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
  QString j;
  for (int i = 1; i <= 6; ++i) {
    auto it = s.joints.find("joint_" + std::to_string(i));
    j += QStringLiteral("J%1 %2  ").arg(i).arg(it == s.joints.end() ? 0.0 : it->second * 180.0 / M_PI, 6, 'f', 1);
    if (i == 3) {j += "\n";}
  }
  lbl_joints_->setText(s.joints.empty() ? QStringLiteral("—") : j);

  // ---- 그리퍼 (gripper_rh_r1 0 ~ 1.1 rad)
  auto g = s.joints.find("gripper_rh_r1");
  if (g != s.joints.end()) {
    const double pos = std::clamp(g->second / 1.1, 0.0, 1.0);
    bar_grip_->setValue(static_cast<int>(pos * 100));
    lbl_grip_->setText((pos < 0.05 ? QStringLiteral("열림") : pos > 0.9 ? QStringLiteral("닫힘 (빈손)") :
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
      if (s.voice->valid && chk_voice_auto_->isChecked()) {onVoiceRun(true);}
    }
  }
  btn_voice_run_->setEnabled(s.voice && s.voice->valid && !s.task_running);

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

void MainWindow::closeEvent(QCloseEvent * ev)
{
  timer_.stop();
  cam_timer_.stop();
  if (backend_->running() &&
    ask(QStringLiteral("종료"), QStringLiteral("이 화면이 실행한 시뮬레이션도 함께 끌까요?")))
  {
    backend_->disconnectSim();
  }
  QMainWindow::closeEvent(ev);
}

}  // namespace aris
