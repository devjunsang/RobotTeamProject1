// 실물 모드 백엔드 구현 (real_backend.hpp 설명 참고).
#include "aris3_gui_cpp/real_backend.hpp"

#include <cmath>

#include "aris3_gui_cpp/ros_io.hpp"

namespace aris
{

namespace
{
const std::string P = "/aris3/real/";
}  // namespace

RealBackend::RealBackend(std::shared_ptr<GuiNode> node, const std::string & ns, const std::string & host, int port,
  double z_min_mm, bool voice, bool gesture, bool virtual_mode)
: node_(std::move(node)), virtual_(virtual_mode),
  world_(QStringLiteral("시뮬레이션 월드(Gazebo)"), {"aris3_bringup", "aris3_real_sim.launch.py"},
    QStringLiteral("ros2 launch aris3_bringup aris3_real_sim.launch.py"), QStringLiteral("/tmp/aris_sim_world.log")),
  // 실제 노드 프로세스만 찾는 패턴 (느슨하면 그 문자열이 든 다른 명령줄에도 걸린다 - aris3_gui 에서 겪음)
  bridge_(QStringLiteral("실물 브리지"),
    QStringList{"aris3_real_bridge", "real_bridge.launch.py", QString::fromStdString("ns:=" + ns),
      QString::fromStdString("host:=" + (virtual_mode ? std::string("127.0.0.1") : host)),
      QString("port:=%1").arg(port), QString("z_min_mm:=%1").arg(z_min_mm)} +
    (virtual_mode ? QStringList{"mode:=virtual", "gripper:=service"} : QStringList{}),
    QStringLiteral("^\\S*python[0-9.]* \\S*/lib/aris3_real_bridge/real_bridge( |$)"),
    QStringLiteral("/tmp/aris_real_bridge.log")),
  voice_(QStringLiteral("음성"), {"aris3_voice", "voice.launch.py"}, QStringLiteral("aris3_voice/voice_node"),
    QStringLiteral("/tmp/aris_voice.log")),
  gesture_(QStringLiteral("손짓(MediaPipe)"), {"aris3_gesture", "gesture.launch.py"},
    QStringLiteral("aris3_gesture/gesture_node"), QStringLiteral("/tmp/aris_gesture.log"))
{
  sub_state_ = node_->create_subscription<State>(P + "state", 10, [this](State::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      state_ = *m;
      state_time_ = std::chrono::steady_clock::now();
    });
  for (const char * n : {"connect", "disconnect", "stop"}) {
    trig_[n] = node_->create_client<std_srvs::srv::Trigger>(P + n);
  }
  srv_armed_ = node_->create_client<std_srvs::srv::SetBool>(P + "set_armed");
  srv_hand_ = node_->create_client<std_srvs::srv::SetBool>(P + "set_hand_guide");
  srv_calib_ = node_->create_client<Calib>(P + "calibration");
  motion_client_ = rclcpp_action::create_client<Motion>(node_, P + "motion");
  pick_client_ = rclcpp_action::create_client<PickPlace>(node_, P + "pick_place");

  // 시뮬레이션이면 월드부터 (이미 떠 있으면 그것을 씀). Gazebo 통신은 루프백에 고정 (Wi-Fi 가 바뀌어도 끊기지 않게)
  if (virtual_) {
    if (!qEnvironmentVariableIsSet("GZ_IP")) {qputenv("GZ_IP", "127.0.0.1");}
    node_->log("INFO", (world_.start() + " - Gazebo 가 뜨는 데 20~30 초").toStdString());
  }
  // 보조 노드 띄우기 (이미 돌고 있으면 그것을 씀)
  for (LaunchProc * lp : {&bridge_}) {node_->log("INFO", lp->start().toStdString());}
  if (voice) {node_->log("INFO", (voice_.start() + " - Whisper 로딩 10~30 초").toStdString());}
  if (gesture) {node_->log("INFO", gesture_.start().toStdString());}
}

QString RealBackend::title() const
{
  return virtual_ ? QStringLiteral("가상 E0509 (실물 경로 시뮬레이션)") : QStringLiteral("실물 E0509");
}

std::optional<RealBackend::State> RealBackend::state()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!state_ || std::chrono::steady_clock::now() - state_time_ > std::chrono::seconds(1)) {return std::nullopt;}
  return state_;
}

// ---------------------------------------------------------------- 서비스·액션 공통
void RealBackend::callTrigger(const std::string & name, const std::string & what)
{
  auto c = trig_.at(name);
  if (!c->service_is_ready()) {
    node_->log("ERROR", what + ": 실물 브리지 서비스 없음 (aris3_real_bridge 실행 확인)");
    return;
  }
  c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, what](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      auto r = f.get();
      node_->log(r && r->success ? "INFO" : "WARN", what + ": " + (r ? r->message : std::string("응답 없음")));
    });
}

template<typename A>
void RealBackend::sendAction(typename rclcpp_action::Client<A>::SharedPtr client, const typename A::Goal & goal,
  const std::string & what, bool & pending)
{
  if (!client->action_server_is_ready()) {
    node_->log("ERROR", what + ": 실물 브리지 액션 없음 (aris3_real_bridge 실행 확인)");
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (pending) {
      node_->log("WARN", what + ": 이전 요청의 결과를 기다리는 중");
      return;
    }
    pending = true;                  // 결과가 올 때까지 「진행 중」 (두 번 보내지 않게)
  }
  typename rclcpp_action::Client<A>::SendGoalOptions opt;
  opt.goal_response_callback = [this, what, &pending](typename rclcpp_action::ClientGoalHandle<A>::SharedPtr h) {
      if (!h) {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          pending = false;
        }
        node_->log("WARN", what + ": 브리지가 거부 (이전 동작이 진행 중)");
      }
    };
  opt.result_callback = [this, what, &pending](const typename rclcpp_action::ClientGoalHandle<A>::WrappedResult & r) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        pending = false;
      }
      const std::string msg = r.result ? r.result->message : std::string();
      if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->success) {
        node_->log("INFO", what + ": " + (msg.empty() ? "완료" : msg));
      } else if (r.code == rclcpp_action::ResultCode::CANCELED) {
        node_->log("WARN", what + ": 취소됨 (정지)");
      } else {
        node_->log("WARN", what + ": " + (msg.empty() ? "실패" : msg));
        // 닿지 않는 위치 (2026-11-14): 브리지가 거리로 거부했거나 컨트롤러가 NOT REACHABLE 로 거부 -> 말로 알린다
        if (msg.find("닿지 않는 위치") != std::string::npos) {
          node_->say("거기는 팔이 닿지 않아요.");
        } else if (msg.find("너무 가까운 위치") != std::string::npos) {   // 베이스에 너무 가까움 (관절 한계)
          node_->say("거기는 로봇에 너무 가까워서 팔이 닿지 않아요.");
        }
      }
    };
  client->async_send_goal(goal, opt);
}

// ---------------------------------------------------------------- 연결
void RealBackend::connectRobot() {callTrigger("connect", "로봇 접속");}
bool RealBackend::bridgeReady() {return trig_.at("connect")->service_is_ready();}
void RealBackend::disconnectRobot() {callTrigger("disconnect", "접속 끊기");}

bool RealBackend::connected()
{
  const auto s = state();
  return s && s->connected;
}

QString RealBackend::describe()
{
  const auto s = state();
  return s ? QString::fromStdString(s->connection_text) : QStringLiteral("실물 브리지 없음 (시작 중이거나 꺼짐)");
}

bool RealBackend::ownsProcess()
{
  return bridge_.running();
}

void RealBackend::shutdown()
{
  // 이 화면이 띄운 노드만 끈다. 브리지를 끄면 브리지가 시작한 로봇 접속도 끊긴다 (서보 꺼짐 - 예전 GUI 종료와 같음)
  voice_.stop();
  gesture_.stop();
  bridge_.stop(25.0);
  world_.stop(20.0);                                  // 이 화면이 띄운 시뮬레이션 월드만 (따로 띄운 월드는 그대로)
}

// ---------------------------------------------------------------- 상태
std::optional<std::vector<double>> RealBackend::jointsDeg()
{
  const auto s = state();
  if (!s || !s->joints_valid) {return std::nullopt;}
  return std::vector<double>(s->joints_deg.begin(), s->joints_deg.end());
}

bool RealBackend::tcp(double xyz_m[3], double rpy_rad[3])
{
  // 브리지는 mm / deg (로봇 베이스) -> 화면은 시뮬레이션과 같은 m / rad 로 그린다
  const auto s = state();
  if (!s || !s->tcp_valid) {return false;}
  for (int i = 0; i < 3; ++i) {
    xyz_m[i] = s->tcp[i] / 1000.0;
    rpy_rad[i] = s->tcp[3 + i] * M_PI / 180.0;
  }
  return true;
}

std::optional<double> RealBackend::gripperPosition()
{
  const auto s = state();
  if (!s || !s->gripper_valid) {return std::nullopt;}
  return s->gripper_position / 740.0;
}

GripperStatus RealBackend::gripperStatus()
{
  const auto s = state();
  if (!s || !s->gripper_service_ready) {return {false, QStringLiteral("서비스 없음 (접속 필요)"), false};}
  if (s->gripper_running) {return {true, QStringLiteral("동작 중"), false};}
  if (s->gripper_result == State::GRIPPER_RESULT_FAILED) {return {false, QStringLiteral("마지막 명령 실패"), false};}
  if (s->gripper_result == State::GRIPPER_RESULT_UNKNOWN) {
    return {false, QStringLiteral("응답 확인 전 (열기/닫기로 확인)"), true};
  }
  return {true, QStringLiteral("준비"), false};
}

QString RealBackend::whyNotReady()
{
  const auto s = state();
  if (!s) {return QStringLiteral("실물 브리지 노드 없음");}
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (motion_pending_) {return QStringLiteral("이전 동작의 결과를 기다리는 중");}
  }
  return QString::fromStdString(s->why_not_ready);
}

QString RealBackend::robotStateText()
{
  const auto s = state();
  return s ? QString::fromStdString(s->robot_state_text) : QStringLiteral("모름 (브리지 없음)");
}

bool RealBackend::canRecover()
{
  const auto s = state();
  return s && s->can_recover;
}

std::optional<bool> RealBackend::handGuide(bool & switching, QString & name)
{
  const auto s = state();
  switching = s && s->mode_switching;
  if (!s || s->robot_mode == State::ROBOT_MODE_UNKNOWN) {
    name = QStringLiteral("모름");
    return std::nullopt;
  }
  const bool manual = s->robot_mode == State::ROBOT_MODE_MANUAL;
  name = manual ? QStringLiteral("수동 (손으로 움직이기)") : QStringLiteral("자동");
  return manual;
}

// ---------------------------------------------------------------- 명령
void RealBackend::home(int pct)
{
  Motion::Goal g;
  g.kind = Motion::Goal::HOME;
  g.speed_percent = pct;
  sendAction<Motion>(motion_client_, g, "홈 이동", motion_pending_);
}

// 「아리스 가동」: 모든 관절 0° 로 관절 이동 (브리지가 홈과 같은 검사를 한다)
void RealBackend::stretch(int pct)
{
  Motion::Goal g;
  g.kind = Motion::Goal::STRETCH;
  g.speed_percent = pct;
  sendAction<Motion>(motion_client_, g, "가동 자세", motion_pending_);
}

// 「아리스 계속해」: 브리지가 기억한 끊긴 순서를 이어서 (없으면 브리지가 거부 사유를 돌려준다)
void RealBackend::resumePick()
{
  PickPlace::Goal g;
  g.kind = PickPlace::Goal::RESUME;
  sendAction<PickPlace>(pick_client_, g, "이어서 하기", pick_pending_);
}

void RealBackend::recoverHome(int pct)
{
  Motion::Goal g;
  g.kind = Motion::Goal::RECOVER_HOME;
  g.speed_percent = pct;
  sendAction<Motion>(motion_client_, g, "복구 후 홈", motion_pending_);
}

void RealBackend::jogJoint(int index, double deg, int pct)
{
  Motion::Goal g;
  g.kind = Motion::Goal::JOG_JOINT;
  g.index = index;
  g.delta = deg;
  g.speed_percent = pct;
  char what[32];
  std::snprintf(what, sizeof(what), "J%d %+g°", index + 1, deg);
  sendAction<Motion>(motion_client_, g, what, motion_pending_);
}

void RealBackend::jogTcp(int axis, double mm, int pct)
{
  Motion::Goal g;
  g.kind = Motion::Goal::JOG_TCP;
  g.index = axis;
  g.delta = mm;
  g.speed_percent = pct;
  char what[32];
  std::snprintf(what, sizeof(what), "TCP %c %+g mm", "XYZ"[axis], mm);
  sendAction<Motion>(motion_client_, g, what, motion_pending_);
}

void RealBackend::gripper(bool close)
{
  Motion::Goal g;
  g.kind = close ? Motion::Goal::GRIPPER_CLOSE : Motion::Goal::GRIPPER_OPEN;
  sendAction<Motion>(motion_client_, g, close ? "그리퍼 닫기" : "그리퍼 열기", motion_pending_);
}

void RealBackend::stop()
{
  // 정지: 브리지(로봇·집기 순서) + Task Manager·MoveIt. 결과를 기다리지 않고 언제나 보낸다
  callTrigger("stop", "정지");
  node_->stopAll();
}

void RealBackend::setArmed(bool on)
{
  if (!srv_armed_->service_is_ready()) {return;}
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  srv_armed_->async_send_request(req);
}

void RealBackend::setHandGuide(bool on)
{
  if (!srv_hand_->service_is_ready()) {
    node_->log("ERROR", "손으로 움직이기: 실물 브리지 서비스 없음");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  srv_hand_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      auto r = f.get();
      node_->log(r && r->success ? "INFO" : "WARN", "손으로 움직이기: " + (r ? r->message : std::string("응답 없음")));
    });
}

void RealBackend::calibration(uint8_t command, double u, double v, const std::string & label)
{
  if (!srv_calib_->service_is_ready()) {
    node_->log("ERROR", "보정: 실물 브리지 서비스 없음");
    return;
  }
  auto req = std::make_shared<Calib::Request>();
  req->command = command;
  req->u = u;
  req->v = v;
  req->label = label;
  srv_calib_->async_send_request(req, [this](rclcpp::Client<Calib>::SharedFuture f) {
      auto r = f.get();
      node_->log(r && r->success ? "INFO" : "WARN", r ? r->message : std::string("보정: 응답 없음"));
    });
}

void RealBackend::pickPlace(uint8_t kind, const std::string & what, const std::string & name,
  const std::optional<Px> & target, const std::optional<Px> & place)
{
  PickPlace::Goal g;
  g.kind = kind;
  g.target_name = name;
  if (target) {
    g.target_u = (*target)[0];
    g.target_v = (*target)[1];
  }
  if (place) {
    g.place_u = (*place)[0];
    g.place_v = (*place)[1];
  }
  sendAction<PickPlace>(pick_client_, g, what, pick_pending_);
}

bool RealBackend::motionPending()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return motion_pending_;
}

bool RealBackend::pickRunning()
{
  const auto s = state();
  std::lock_guard<std::mutex> lk(mtx_);
  return pick_pending_ || (s && s->sequence_running);
}

std::optional<bool> RealBackend::holding()
{
  const auto s = state();
  if (!s || s->holding == State::HOLD_UNKNOWN) {return std::nullopt;}
  return s->holding == State::HOLD_YES;
}

// ---------------------------------------------------------------- 보정 보기
bool RealBackend::calibReady()
{
  const auto s = state();
  return s && s->calib_ready;
}

std::optional<RealBackend::Px> RealBackend::toRobot(double u, double v)
{
  // 브리지와 같은 3x3 호모그래피: [x y w] = H [u v 1]
  const auto s = state();
  if (!s || !s->calib_ready) {return std::nullopt;}
  const auto & h = s->calib_homography;
  const double x = h[0] * u + h[1] * v + h[2], y = h[3] * u + h[4] * v + h[5], w = h[6] * u + h[7] * v + h[8];
  if (std::abs(w) < 1e-12) {return std::nullopt;}
  return Px{x / w, y / w};
}

std::optional<std::array<double, 9>> RealBackend::calibMatrix()
{
  const auto s = state();
  if (!s || !s->calib_ready) {return std::nullopt;}
  std::array<double, 9> h{};
  for (std::size_t k = 0; k < 9; ++k) {h[k] = s->calib_homography[k];}
  return h;
}

std::optional<double> RealBackend::graspZ()
{
  const auto s = state();
  if (!s || !s->calib_ready) {return std::nullopt;}
  return s->calib_grasp_z_mm;
}

QString RealBackend::calibSummary()
{
  const auto s = state();
  return s ? QString::fromStdString(s->calib_summary) : QStringLiteral("보정 정보 없음 (실물 브리지 연결 전)");
}

std::optional<RealBackend::Px> RealBackend::calibPendingPx()
{
  const auto s = state();
  if (!s || !s->calib_pending) {return std::nullopt;}
  return Px{s->calib_pending_u, s->calib_pending_v};
}

}  // namespace aris
