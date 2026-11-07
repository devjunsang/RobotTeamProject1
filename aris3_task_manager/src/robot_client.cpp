#include "aris3_task_manager/robot_client.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "aris3_task_manager/service_call.hpp"

namespace aris3_task_manager
{

using namespace dsr_msgs2::srv;  // NOLINT

namespace
{
constexpr int8_t kRefBase = 0;       // DR_BASE: 베이스 좌표계
constexpr int8_t kModeAbsolute = 0;  // 절대 좌표 목표
constexpr int8_t kSyncAsync = 1;     // 명령만 넣고 바로 돌아옴
constexpr int32_t kStopQuick = 1;    // 감속 정지, 서보 유지 (0 은 서보 전원 차단)
constexpr int8_t kMotionIdle = 0;    // check_motion: 0 = 정지
}  // namespace

RobotClient::RobotClient(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr group, Options options)
: node_(node), opt_(std::move(options))
{
  const std::string p = opt_.ns;
  const auto qos = rclcpp::ServicesQoS();
  move_joint_ = node_->create_client<MoveJoint>(p + "/motion/move_joint", qos, group);
  move_line_ = node_->create_client<MoveLine>(p + "/motion/move_line", qos, group);
  move_stop_ = node_->create_client<MoveStop>(p + "/motion/move_stop", qos, group);
  check_motion_ = node_->create_client<CheckMotion>(p + "/motion/check_motion", qos, group);
  ikin_ = node_->create_client<Ikin>(p + "/motion/ikin", qos, group);
  fkin_ = node_->create_client<Fkin>(p + "/motion/fkin", qos, group);
  get_posx_ = node_->create_client<GetCurrentPosx>(p + "/aux_control/get_current_posx", qos, group);
  get_state_ = node_->create_client<GetRobotState>(p + "/system/get_robot_state", qos, group);
}

bool RobotClient::servicesReady(std::string * missing) const
{
  const std::vector<rclcpp::ClientBase *> all{move_joint_.get(), move_line_.get(), move_stop_.get(),
    check_motion_.get(), ikin_.get(), fkin_.get(), get_posx_.get(), get_state_.get()};
  for (auto * c : all) {
    if (!c->service_is_ready()) {
      if (missing) {*missing = c->get_service_name();}
      return false;
    }
  }
  return true;
}

int RobotClient::robotState()
{
  auto res = callService<GetRobotState>(get_state_, std::make_shared<GetRobotState::Request>(),
      opt_.service_timeout_s);
  return (res && res->success) ? res->robot_state : -1;
}

std::optional<std::array<double, 6>> RobotClient::currentPose()
{
  auto req = std::make_shared<GetCurrentPosx::Request>();
  req->ref = kRefBase;
  auto res = callService<GetCurrentPosx>(get_posx_, req, opt_.service_timeout_s);
  if (!res || !res->success || res->task_pos_info.empty() || res->task_pos_info[0].data.size() < 6) {
    return std::nullopt;
  }
  std::array<double, 6> out{};
  for (int i = 0; i < 6; ++i) {out[i] = res->task_pos_info[0].data[i];}
  return out;
}

bool RobotClient::stop()
{
  auto req = std::make_shared<MoveStop::Request>();
  req->stop_mode = kStopQuick;
  auto res = callService<MoveStop>(move_stop_, req, opt_.service_timeout_s);
  return res && res->success;
}

void RobotClient::waitForMotion(const std::string & label, const AbortCheck & abort)
{
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  const auto period = std::chrono::duration<double>(opt_.poll_period_s);
  bool seen_moving = false;
  while (true) {
    const ErrorCode why = abort ? abort() : ErrorCode::NONE;
    if (why != ErrorCode::NONE) {
      stop();
      throw TaskError(why, label + ": 이동 중 중단 (" + errorName(why) + ")");
    }
    const double elapsed = std::chrono::duration<double>(clock::now() - t0).count();
    if (elapsed > opt_.motion_timeout_s) {
      stop();
      throw TaskError(ErrorCode::TIMEOUT, label + ": 이동 시간 초과");
    }
    auto res = callService<CheckMotion>(check_motion_, std::make_shared<CheckMotion::Request>(),
        opt_.service_timeout_s);
    if (!res || !res->success) {
      stop();
      throw TaskError(ErrorCode::MOTION_FAILED, label + ": check_motion 응답 없음");
    }
    if (res->status != kMotionIdle) {
      seen_moving = true;
    } else if (seen_moving || elapsed > 0.5) {
      // 움직이다 멈췄거나, 0.5 초가 지나도 움직이기 시작하지 않음(이미 그 자리 / 무시된 명령)
      return;
    }
    std::this_thread::sleep_for(period);
  }
}

void RobotClient::moveJoints(const std::array<double, 6> & deg, const std::string & label, const AbortCheck & abort)
{
  auto req = std::make_shared<MoveJoint::Request>();
  for (int i = 0; i < 6; ++i) {req->pos[i] = deg[i];}
  req->vel = opt_.joint_vel_deg_s;
  req->acc = opt_.joint_acc_deg_s2;
  req->mode = kModeAbsolute;
  req->sync_type = kSyncAsync;
  auto res = callService<MoveJoint>(move_joint_, req, opt_.service_timeout_s);
  if (!res || !res->success) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": move_joint 거부됨");
  }
  waitForMotion(label, abort);
  // 도착 확인: 관절각 대신 플랜지 자세를 FK 로 비교하면 단순해지지만, 관절 목표는 관절로 확인하는 것이 정확하다.
  // 두산 get_current_posj 를 쓰지 않고, 목표 관절각의 FK 와 현재 플랜지 위치를 비교한다(서비스 수를 줄이기 위해).
  auto fk_req = std::make_shared<Fkin::Request>();
  for (int i = 0; i < 6; ++i) {fk_req->pos[i] = deg[i];}
  fk_req->ref = kRefBase;
  auto fk = callService<Fkin>(fkin_, fk_req, opt_.service_timeout_s);
  auto now = currentPose();
  if (!fk || !now) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": 도착 확인 실패 (fkin/현재 위치 응답 없음)");
  }
  const Vec3 d{now->at(0) - fk->conv_posx[0], now->at(1) - fk->conv_posx[1], now->at(2) - fk->conv_posx[2]};
  if (norm(d) > opt_.arrival_tolerance_mm) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s: 목표 자세에 도착하지 못함 (플랜지 위치 차이 %.1f mm)", label.c_str(), norm(d));
    throw TaskError(ErrorCode::MOTION_FAILED, buf);
  }
}

void RobotClient::moveLinear(const std::array<double, 6> & pose, const std::string & label, const AbortCheck & abort)
{
  auto req = std::make_shared<MoveLine::Request>();
  for (int i = 0; i < 6; ++i) {req->pos[i] = pose[i];}
  req->vel = {opt_.linear_vel_mm_s, 60.0};    // [직선 mm/s, 회전 deg/s]
  req->acc = {opt_.linear_acc_mm_s2, 120.0};
  req->ref = kRefBase;
  req->mode = kModeAbsolute;
  req->sync_type = kSyncAsync;
  auto res = callService<MoveLine>(move_line_, req, opt_.service_timeout_s);
  if (!res || !res->success) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": move_line 거부됨");
  }
  waitForMotion(label, abort);
  auto now = currentPose();
  if (!now) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": 도착 확인 실패 (현재 위치 응답 없음)");
  }
  const Vec3 d{now->at(0) - pose[0], now->at(1) - pose[1], now->at(2) - pose[2]};
  if (norm(d) > opt_.arrival_tolerance_mm) {
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%s: 목표에 도착하지 못함 (남은 거리 %.1f mm, 도달 불가 위치일 수 있음)",
      label.c_str(), norm(d));
    throw TaskError(ErrorCode::MOTION_FAILED, buf);
  }
}

double RobotClient::reachabilityErrorMm(const std::array<double, 6> & pose)
{
  auto ik_req = std::make_shared<Ikin::Request>();
  for (int i = 0; i < 6; ++i) {ik_req->pos[i] = pose[i];}
  ik_req->sol_space = 2;  // 팔꿈치 위 자세 계열 (홈 자세와 같은 계열)
  ik_req->ref = kRefBase;
  auto ik = callService<Ikin>(ikin_, ik_req, opt_.service_timeout_s);
  if (!ik || !ik->success) {return -1.0;}
  auto fk_req = std::make_shared<Fkin::Request>();
  fk_req->pos = ik->conv_posj;
  fk_req->ref = kRefBase;
  auto fk = callService<Fkin>(fkin_, fk_req, opt_.service_timeout_s);
  if (!fk || !fk->success) {return -1.0;}
  const Vec3 d{fk->conv_posx[0] - pose[0], fk->conv_posx[1] - pose[1], fk->conv_posx[2] - pose[2]};
  return norm(d);
}

}  // namespace aris3_task_manager
