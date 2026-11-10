#include "aris3_task_manager/task_manager_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <functional>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "aris3_task_manager/moveit_robot_client.hpp"

namespace aris3_task_manager
{

using aris3_interfaces::msg::DetectedObjectArray;
using aris3_interfaces::msg::TaskStatus;
using namespace std::chrono_literals;

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}
std::string upper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::toupper(c);});
  return s;
}
std::string fmt(const char * f, double a, double b, double c)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}
std::string xyz(const Vec3 & p) {return fmt("(%.3f, %.3f, %.3f)", p.x, p.y, p.z);}
geometry_msgs::msg::Point toPoint(const Vec3 & p)
{
  geometry_msgs::msg::Point q;
  q.x = p.x; q.y = p.y; q.z = p.z;
  return q;
}
constexpr int kStandby = 1;           // 두산 로봇 상태: 대기
constexpr size_t kMaxBufferedFrames = 50;
}  // namespace

// =====================================================================================
// 생성자: 파라미터 → 통신 객체 → 액션 서버 순서로 만든다.
// =====================================================================================
TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("aris3_task_manager", options)
{
  loadParameters();

  // 콜백 그룹: 서비스 응답은 여러 개가 동시에 처리돼도 되므로 Reentrant.
  // 카메라 구독은 한 번에 하나씩(MutuallyExclusive) 처리한다.
  client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RobotClient::Options ro;
  ro.ns = get_parameter("robot_ns").as_string();
  ro.linear_vel_mm_s = get_parameter("linear_vel_mm_s").as_double();
  ro.linear_acc_mm_s2 = 2.0 * ro.linear_vel_mm_s;
  ro.joint_vel_deg_s = get_parameter("joint_vel_deg_s").as_double();
  ro.joint_acc_deg_s2 = ro.joint_vel_deg_s;
  ro.motion_timeout_s = get_parameter("motion_timeout_s").as_double();
  ro.arrival_tolerance_mm = get_parameter("arrival_tolerance_mm").as_double();
  // 로봇 이동 백엔드: dsr = 두산 서비스 (e0509_sim / 실제 컨트롤러), moveit = MoveIt 2 (aris3 Gazebo)
  const std::string backend = get_parameter("robot_backend").as_string();
  if (backend == "moveit") {
    MoveItRobotClient::Options mo;
    mo.group = get_parameter("moveit_group").as_string();
    mo.tip_link = get_parameter("moveit_tip_link").as_string();
    mo.base_frame = base_frame_;
    mo.joint_states_topic = get_parameter("moveit_joint_states_topic").as_string();
    mo.planning_time_s = get_parameter("moveit_planning_time_s").as_double();
    mo.velocity_scaling = get_parameter("moveit_velocity_scaling").as_double();
    mo.acceleration_scaling = get_parameter("moveit_acceleration_scaling").as_double();
    mo.motion_timeout_s = ro.motion_timeout_s;
    mo.arrival_tolerance_mm = ro.arrival_tolerance_mm;
    robot_ = std::make_unique<MoveItRobotClient>(this, mo);
  } else if (backend == "dsr") {
    robot_ = std::make_unique<RobotClient>(this, client_group_, ro);
  } else {
    throw std::runtime_error("robot_backend 는 dsr 또는 moveit (현재: " + backend + ")");
  }
  gripper_ = std::make_unique<GripperClient>(this, client_group_,
      get_parameter("gripper_service").as_string(), get_parameter("gripper_timeout_s").as_double());

  // TF: 별도 스레드가 /tf, /tf_static 을 받아 buffer 에 쌓는다.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = input_group_;
  detections_sub_ = create_subscription<DetectedObjectArray>(
    get_parameter("detections_topic").as_string(), 10,
    std::bind(&TaskManagerNode::onDetections, this, std::placeholders::_1), sub_opts);

  status_pub_ = create_publisher<TaskStatus>("/aris3/task_status", 10);
  status_timer_ = create_wall_timer(1s, std::bind(&TaskManagerNode::onStatusTimer, this));

  // 중복 실행 방지: 같은 이름의 액션 서버가 둘이면 목표가 양쪽에 다 전달되어 두 프로세스가 같은 로봇을
  // 동시에 움직인다 (2026-11-07 시뮬레이션에서 실제로 겪음).
  const std::string action_name = get_parameter("action_name").as_string();
  if (actionServerAlive(action_name)) {
    throw std::runtime_error("같은 이름의 액션 서버(" + action_name + ")가 이미 실행 중 - 두 번 띄우지 마세요");
  }

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this, action_name,
    std::bind(&TaskManagerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskManagerNode::handleCancel, this, std::placeholders::_1),
    std::bind(&TaskManagerNode::handleAccepted, this, std::placeholders::_1));

  publishStatus(TaskStatus::STATE_IDLE, "", "", "작업 대기");
  RCLCPP_INFO(get_logger(), "준비됨: 액션 %s, 비전 %s, 그리퍼 %s, 로봇 %s (%s)",
    get_parameter("action_name").as_string().c_str(),
    get_parameter("detections_topic").as_string().c_str(), gripper_->serviceName().c_str(),
    backend == "moveit" ? "move_group" : ro.ns.c_str(), backend.c_str());
}

// 이미 살아 있는 액션 서버가 있는가?
// ROS 그래프에 이름이 보이는지만 보면 안 된다: 강제 종료된 프로세스의 이름이 잠시(수십 초) 남아 있어서
// 정상적인 재시작까지 막는다(실제로 겪음). 그래서 "존재하지 않는 목표의 결과"를 실제로 물어본다.
// 살아 있는 서버는 STATUS_UNKNOWN 으로 바로 답하고, 진행 중인 작업에는 아무 영향이 없다.
bool TaskManagerNode::actionServerAlive(const std::string & action_name)
{
  using GetResult = ExecuteTask::Impl::GetResultService;
  auto probe = std::make_shared<rclcpp::Node>("aris3_task_manager_probe");
  auto client = probe->create_client<GetResult>(action_name + "/_action/get_result");
  if (!client->wait_for_service(3s)) {   // 새 프로세스가 다른 노드를 발견하는 데 1~2 초 걸릴 수 있다
    RCLCPP_INFO(probe->get_logger(), "중복 확인: %s 없음", action_name.c_str());
    return false;   // 이름조차 없음
  }
  auto req = std::make_shared<GetResult::Request>();
  for (size_t i = 0; i < req->goal_id.uuid.size(); ++i) {req->goal_id.uuid[i] = static_cast<uint8_t>(0xA5 ^ i);}
  auto fut = client->async_send_request(req);
  const auto rc = rclcpp::spin_until_future_complete(probe, fut, 1500ms);
  RCLCPP_INFO(probe->get_logger(), "중복 확인: %s 이름 있음, 응답 %s", action_name.c_str(),
    rc == rclcpp::FutureReturnCode::SUCCESS ? "있음(살아 있는 서버)" : "없음(이전 프로세스의 흔적)");
  return rc == rclcpp::FutureReturnCode::SUCCESS;
}

TaskManagerNode::~TaskManagerNode()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

void TaskManagerNode::loadParameters()
{
  // declare_parameter: 이름, 기본값. 실제 값은 config/task_manager.yaml 이 덮어쓴다.
  declare_parameter("action_name", "/aris3/execute_task");
  declare_parameter("robot_ns", "/dsr01");
  declare_parameter("robot_backend", "dsr");          // dsr | moveit
  declare_parameter("moveit_group", "arm");
  declare_parameter("moveit_tip_link", "link_6");
  declare_parameter("moveit_joint_states_topic", "/e0509_gripper/joint_states");
  declare_parameter("moveit_planning_time_s", 5.0);
  declare_parameter("moveit_velocity_scaling", 0.3);
  declare_parameter("moveit_acceleration_scaling", 0.3);
  declare_parameter("detections_topic", "/aris3/detected_objects");
  declare_parameter("gripper_service", "/aris3/gripper/command");
  declare_parameter("gripper_timeout_s", 10.0);
  declare_parameter("world_frame", "workcell");   // 기본값은 config/task_manager.yaml 과 같게 유지
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("home_joints_deg", std::vector<double>{0.0, 0.0, 90.0, 0.0, 90.0, 0.0});
  declare_parameter("tool_orientation_zyz_deg", std::vector<double>{0.0, 180.0, 0.0});
  declare_parameter("tcp_offset_m", 0.08);
  declare_parameter("grasp_offset_m", 0.005);
  declare_parameter("approach_height_m", 0.10);
  declare_parameter("object_height_m", 0.04);
  declare_parameter("place_clearance_m", 0.002);
  declare_parameter("workspace_r_min_m", 0.20);
  declare_parameter("workspace_r_max_m", 0.75);
  declare_parameter("workspace_z_min_m", 0.005);
  declare_parameter("workspace_z_max_m", 0.60);
  declare_parameter("zone_names", std::vector<std::string>{"A", "B", "C"});
  declare_parameter("zone_xy_world", std::vector<double>{0.30, 0.35, 0.45, 0.35, 0.60, 0.35});
  declare_parameter("table_z_world", 0.75);
  declare_parameter("zone_tolerance_m", 0.03);
  declare_parameter("zone_clear_radius_m", 0.06);
  declare_parameter("known_colors", std::vector<std::string>{"red", "green", "blue"});
  declare_parameter("perception_frames", 5);
  declare_parameter("perception_timeout_s", 5.0);
  declare_parameter("max_spread_m", 0.005);
  declare_parameter("min_confidence", 0.5);
  declare_parameter("settle_time_s", 0.5);
  declare_parameter("linear_vel_mm_s", 100.0);
  declare_parameter("joint_vel_deg_s", 40.0);
  declare_parameter("motion_timeout_s", 30.0);
  declare_parameter("arrival_tolerance_mm", 3.0);
  declare_parameter("reach_tolerance_mm", 1.0);
  declare_parameter("task_timeout_s", 180.0);
  declare_parameter("grip_force", 0.5);
  declare_parameter("max_grasp_attempts", 2);
  declare_parameter("return_home_on_failure", true);

  world_frame_ = get_parameter("world_frame").as_string();
  base_frame_ = get_parameter("robot_base_frame").as_string();
  const auto home = get_parameter("home_joints_deg").as_double_array();
  const auto tool = get_parameter("tool_orientation_zyz_deg").as_double_array();
  if (home.size() != 6 || tool.size() != 3) {
    throw std::runtime_error("home_joints_deg 는 6개, tool_orientation_zyz_deg 는 3개여야 함");
  }
  std::copy(home.begin(), home.end(), home_deg_.begin());
  std::copy(tool.begin(), tool.end(), tool_zyz_deg_.begin());
  geom_.tcp_offset_m = get_parameter("tcp_offset_m").as_double();
  geom_.grasp_offset_m = get_parameter("grasp_offset_m").as_double();
  geom_.approach_height_m = get_parameter("approach_height_m").as_double();
  geom_.object_height_m = get_parameter("object_height_m").as_double();
  geom_.place_clearance_m = get_parameter("place_clearance_m").as_double();
  workspace_.r_min_m = get_parameter("workspace_r_min_m").as_double();
  workspace_.r_max_m = get_parameter("workspace_r_max_m").as_double();
  workspace_.z_min_m = get_parameter("workspace_z_min_m").as_double();
  workspace_.z_max_m = get_parameter("workspace_z_max_m").as_double();

  const auto names = get_parameter("zone_names").as_string_array();
  const auto xy = get_parameter("zone_xy_world").as_double_array();
  const double table_z = get_parameter("table_z_world").as_double();
  table_z_world_ = table_z;
  if (xy.size() != 2 * names.size()) {
    throw std::runtime_error("zone_xy_world 는 zone_names 개수의 2배 (x, y 쌍) 여야 함");
  }
  for (size_t i = 0; i < names.size(); ++i) {
    zones_world_[upper(names[i])] = Vec3{xy[2 * i], xy[2 * i + 1], table_z};
  }
  // 주의: get_parameter(...) 가 돌려주는 임시 객체의 배열을 바로 for 로 돌면, 반복 중에 임시 객체가
  // 이미 사라져 있다(dangling reference). 먼저 지역 변수로 복사한다.
  const auto colors = get_parameter("known_colors").as_string_array();
  for (const auto & c : colors) {
    known_colors_.push_back(lower(c));
  }
  zone_tolerance_m_ = get_parameter("zone_tolerance_m").as_double();
  zone_clear_radius_m_ = get_parameter("zone_clear_radius_m").as_double();
  perception_frames_ = static_cast<int>(get_parameter("perception_frames").as_int());
  perception_timeout_s_ = get_parameter("perception_timeout_s").as_double();
  max_spread_m_ = get_parameter("max_spread_m").as_double();
  min_confidence_ = get_parameter("min_confidence").as_double();
  settle_time_s_ = get_parameter("settle_time_s").as_double();
  task_timeout_s_ = get_parameter("task_timeout_s").as_double();
  reach_tolerance_mm_ = get_parameter("reach_tolerance_mm").as_double();
  grip_force_ = get_parameter("grip_force").as_double();
  max_grasp_attempts_ = static_cast<int>(get_parameter("max_grasp_attempts").as_int());
  return_home_on_failure_ = get_parameter("return_home_on_failure").as_bool();
}

// =====================================================================================
// 액션 서버 콜백 3종 — executor 스레드에서 불리므로 오래 걸리는 일을 하면 안 된다.
// =====================================================================================
namespace
{
// 로그용 한 줄 요약: "blue_cube -> (0.300, -0.200)" / "red -> A"
std::string describeGoal(const aris3_interfaces::action::ExecuteTask::Goal & g)
{
  std::string what = g.target_id.empty() ? g.target_color : g.target_id;
  if (g.target_id.empty() && !g.target_shape.empty()) {what += " " + g.target_shape;}
  char where[64];
  if (g.use_place_position) {
    std::snprintf(where, sizeof(where), "(%.3f, %.3f)", g.place_position.x, g.place_position.y);
  } else {
    std::snprintf(where, sizeof(where), "%s", g.destination.c_str());
  }
  return what + " -> " + where + (g.dry_run ? " (dry_run)" : "");
}
}  // namespace

rclcpp_action::GoalResponse TaskManagerNode::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
{
  // 한 번에 한 작업만. exchange(true) 는 "이전 값을 돌려주고 true 로 바꾸기"를 한 번에(원자적으로) 한다.
  if (busy_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "거절: 이미 작업 중 (%s 요청)", describeGoal(*goal).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "수락: %s", describeGoal(*goal).c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskManagerNode::handleCancel(std::shared_ptr<GoalHandle>)
{
  RCLCPP_WARN(get_logger(), "취소 요청 받음 - 다음 확인 시점(최대 0.1 s)에 정지");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskManagerNode::handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  if (worker_.joinable()) {
    worker_.join();  // 이전 작업 스레드는 이미 끝났다 (busy_ 가 false 였으므로)
  }
  worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
}

// =====================================================================================
// 작업 스레드: 상태 머신 본체
// =====================================================================================
void TaskManagerNode::execute(std::shared_ptr<GoalHandle> goal_handle)
{
  TaskContext ctx;
  ctx.goal_handle = goal_handle;
  ctx.goal = *goal_handle->get_goal();
  ctx.start = std::chrono::steady_clock::now();
  ctx.task_id = describeGoal(ctx.goal) + " #" +
    std::to_string(++task_counter_);

  ErrorCode code = ErrorCode::NONE;
  std::string message;
  try {
    stageValidate(ctx);
    stageObserve(ctx);
    stagePerceive(ctx);
    stagePlan(ctx);
    if (ctx.goal.dry_run) {
      message = "dry_run: 인식과 계획까지 성공 (로봇은 움직이지 않음). 물체 " + xyz(ctx.object_world);
    } else {
      // 집기는 실패하면 정해진 횟수만큼 "다시 보고, 다시 계획하고" 재시도한다.
      for (ctx.grasp_attempt = 1;; ++ctx.grasp_attempt) {
        try {
          stagePick(ctx);
          break;
        } catch (const TaskError & e) {
          if (e.code() != ErrorCode::GRASP_FAILED || ctx.grasp_attempt >= max_grasp_attempts_) {
            throw;
          }
          RCLCPP_WARN(get_logger(), "집기 실패 (%d/%d): %s -> 다시 관찰", ctx.grasp_attempt,
            max_grasp_attempts_, e.what());
          stageObserve(ctx);
          stagePerceive(ctx);
          stagePlan(ctx);
        }
      }
      stagePlace(ctx);
      stageVerify(ctx);
      message = "완료: " + ctx.dest_label + fmt(" 중심에서 %.1f mm (허용 %.0f mm)", ctx.result->place_error_mm,
          zone_tolerance_m_ * 1000, 0);
    }
  } catch (const TaskError & e) {
    code = e.code();
    message = e.what();
  } catch (const std::exception & e) {
    code = ErrorCode::INTERNAL;
    message = std::string("예상하지 못한 오류: ") + e.what();
  }

  auto & r = *ctx.result;
  const Stage failed_stage = ctx.stage;
  if (code != ErrorCode::NONE) {
    RCLCPP_ERROR(get_logger(), "[%s] 실패 %s: %s", stageName(failed_stage), errorName(code), message.c_str());
    recover(ctx, code);
    if (ctx.holding) {
      message += " (물체를 잡은 상태로 끝남)";
    }
  }
  r.success = (code == ErrorCode::NONE);
  r.error_code = static_cast<uint8_t>(code);
  r.message = message;
  r.failed_stage = r.success ? "" : stageName(failed_stage);
  r.duration_s = std::chrono::duration<float>(std::chrono::steady_clock::now() - ctx.start).count();

  try {
    if (code == ErrorCode::CANCELED && goal_handle->is_canceling()) {
      goal_handle->canceled(ctx.result);
    } else if (r.success) {
      goal_handle->succeed(ctx.result);
    } else {
      goal_handle->abort(ctx.result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "결과 전송 실패: %s", e.what());
  }
  RCLCPP_INFO(get_logger(), "작업 %s 종료: %s (%.1f s) %s", ctx.task_id.c_str(),
    r.success ? "성공" : errorName(code), r.duration_s, message.c_str());
  publishStatus(r.success ? TaskStatus::STATE_IDLE : TaskStatus::STATE_ERROR, ctx.task_id,
    r.success ? "DONE" : stageName(failed_stage), message);
  busy_ = false;  // 맨 마지막에 풀어야 다음 요청이 이 작업과 겹치지 않는다
}

// ---- 1. VALIDATE: 요청 내용과 장비 준비 상태를 움직이기 전에 모두 확인한다.
void TaskManagerNode::stageValidate(TaskContext & ctx)
{
  enterStage(ctx, Stage::VALIDATE, "요청과 장비 준비 상태 확인");
  const auto & g = ctx.goal;
  if (!g.task_type.empty() && lower(g.task_type) != "pick_place") {
    throw TaskError(ErrorCode::INVALID_GOAL, "지원하지 않는 작업 종류: " + g.task_type);
  }
  // 무엇을: 물체 이름(target_id, 손짓으로 고른 물체) 또는 색(+모양)
  const std::string color = lower(g.target_color);
  if (g.target_id.empty() &&
    std::find(known_colors_.begin(), known_colors_.end(), color) == known_colors_.end())
  {
    std::string list;
    for (const auto & c : known_colors_) {list += c + " ";}
    throw TaskError(ErrorCode::INVALID_GOAL, "모르는 색 '" + g.target_color + "' (가능: " + list + ")");
  }
  ctx.query = TargetQuery{color, lower(g.target_shape), min_confidence_, g.target_id};
  ctx.target_label = g.target_id.empty() ? color : g.target_id;

  // 어디에: 이름 붙은 구역 또는 임의 좌표(place_position, world_frame, 탁자 표면)
  if (g.use_place_position) {
    if (!std::isfinite(g.place_position.x) || !std::isfinite(g.place_position.y)) {
      throw TaskError(ErrorCode::INVALID_GOAL, "놓을 좌표가 숫자가 아님");
    }
    ctx.zone_world = Vec3{g.place_position.x, g.place_position.y, table_z_world_};
    ctx.dest_label = fmt("(%.3f, %.3f)", g.place_position.x, g.place_position.y, 0);
    // 작업 공간 밖 좌표는 PLAN 단계의 작업 공간·IK 검사가 움직이기 전에 거부한다.
  } else {
    auto zone = zones_world_.find(upper(g.destination));
    if (zone == zones_world_.end()) {
      std::string list;
      for (const auto & z : zones_world_) {list += z.first + " ";}
      throw TaskError(ErrorCode::INVALID_GOAL, "모르는 구역 '" + g.destination + "' (가능: " + list + ")");
    }
    ctx.zone_world = zone->second;
    ctx.dest_label = "구역 " + zone->first;
  }

  std::string missing;
  if (!robot_->servicesReady(&missing)) {
    throw TaskError(ErrorCode::NOT_READY, "로봇 서비스 없음: " + missing + " (시뮬레이션/로봇 접속 확인)");
  }
  const int state = robot_->robotState();
  if (state != kStandby) {
    throw TaskError(ErrorCode::NOT_READY, "로봇이 대기(STANDBY) 상태가 아님 (state=" + std::to_string(state) + ")");
  }
  if (!gripper_->ready()) {
    throw TaskError(ErrorCode::NOT_READY, "그리퍼 서비스 없음: " + gripper_->serviceName());
  }
  // 이전 작업이 취소되어 물체를 잡은 채 끝났을 수 있다. 그 상태로 새 작업을 시작하면 안 된다.
  const auto gs = gripper_->status();
  if (!gs.ok) {
    throw TaskError(ErrorCode::NOT_READY, "그리퍼 상태 확인 실패: " + gs.message);
  }
  if (gs.holding) {
    throw TaskError(ErrorCode::NOT_READY, "그리퍼가 이미 " + (gs.object_id.empty() ? std::string("물체") : gs.object_id) +
            " 를 잡고 있음 - 먼저 정리 (README의 '물체를 잡은 채 멈췄을 때')");
  }
  {
    std::lock_guard<std::mutex> lk(frames_mutex_);
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_time_).count();
    if (frame_seq_ == 0 || age > 2.0) {
      throw TaskError(ErrorCode::NOT_READY, "카메라 인식 결과가 들어오지 않음 (" +
              get_parameter("detections_topic").as_string() + ")");
    }
  }
  std::string tf_error;
  if (!refreshTransforms(&tf_error)) {
    throw TaskError(ErrorCode::NOT_READY, "좌표 변환 없음: " + tf_error);
  }
  report(ctx, "확인 완료: 로봇 대기, 그리퍼·카메라·TF 정상. 놓을 자리 " + ctx.dest_label + " " +
    xyz(ctx.zone_world), 1.0);
}

// ---- 2. OBSERVE: 카메라 시야를 가리지 않는 자세(홈)로 이동.
void TaskManagerNode::stageObserve(TaskContext & ctx)
{
  enterStage(ctx, Stage::OBSERVE, "관찰 자세(홈)로 이동");
  if (ctx.goal.dry_run) {
    report(ctx, "dry_run: 이동 생략, 현재 자세에서 관찰", 1.0);
    return;
  }
  robot_->moveJoints(home_deg_, "관찰 자세(홈)", abortCheck(ctx));
  sleepChecked(ctx, settle_time_s_);  // 팔이 멈춘 뒤 찍힌 영상만 쓰기 위해
  report(ctx, "관찰 자세 도착", 1.0);
}

// ---- 3. PERCEIVE: 여러 장의 카메라 결과로 목표 물체 위치를 정한다.
void TaskManagerNode::stagePerceive(TaskContext & ctx)
{
  enterStage(ctx, Stage::PERCEIVE, "카메라로 " + ctx.target_label + " 물체 찾기");
  const Estimate e = perceive(ctx, "집기 전");
  switch (e.status) {
    case EstimateStatus::OK: break;
    case EstimateStatus::NOT_FOUND: throw TaskError(ErrorCode::TARGET_NOT_FOUND, "목표를 못 찾음: " + e.reason);
    case EstimateStatus::UNSTABLE: throw TaskError(ErrorCode::TARGET_UNSTABLE, "목표 위치 불안정: " + e.reason);
    case EstimateStatus::AMBIGUOUS: throw TaskError(ErrorCode::TARGET_AMBIGUOUS, e.reason);
  }
  ctx.object_world = e.mean;
  ctx.object_id = e.id;
  ctx.result->picked_from = toPoint(e.mean);
  // 놓을 구역이 비어 있는가? 다른 물체가 있으면 그 위에 쌓거나 부딪힌다.
  const auto occupants = occupantsNear(ctx.last_objects, ctx.zone_world, zone_clear_radius_m_, e.mean);
  if (!occupants.empty()) {
    const auto & o = occupants.front();
    throw TaskError(ErrorCode::DESTINATION_OCCUPIED, ctx.dest_label + "에 이미 " +
            (o.id.empty() ? o.color : o.id) + " 가 있음 " + xyz(o.position) + " - 움직이지 않음");
  }
  report(ctx, "찾음: " + (e.id.empty() ? ctx.target_label : e.id) + " " + xyz(e.mean) +
    fmt(" (영상 %.0f장, 흔들림 %.1f mm)", e.frames_matched, e.spread_m * 1000, 0), 1.0);
}

Estimate TaskManagerNode::perceive(TaskContext & ctx, const std::string & purpose)
{
  TargetEstimator est(ctx.query);
  uint64_t last_seq;
  {
    std::lock_guard<std::mutex> lk(frames_mutex_);
    last_seq = frame_seq_;  // 지금 이후에 들어오는 영상만 쓴다 (이동 중에 찍힌 영상 제외)
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(perception_timeout_s_);
  while (std::chrono::steady_clock::now() < deadline) {
    const ErrorCode why = abortReason(ctx);
    if (why != ErrorCode::NONE) {
      throw TaskError(why, purpose + " 인식 중 중단");
    }
    {
      std::lock_guard<std::mutex> lk(frames_mutex_);
      for (const auto & f : frames_) {
        if (f.seq > last_seq) {
          est.addFrame(f.objects);
          last_seq = f.seq;
          ctx.last_objects = f.objects;
        }
      }
    }
    if (est.framesMatched() >= perception_frames_ || est.framesSeen() >= 3 * perception_frames_) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  return est.estimate(perception_frames_, max_spread_m_);
}

// ---- 4. PLAN: 경로점을 계산하고, 움직이기 전에 모두 갈 수 있는지 확인한다.
void TaskManagerNode::stagePlan(TaskContext & ctx)
{
  enterStage(ctx, Stage::PLAN, "경로점 계산과 도달 가능성 확인");
  const Vec3 object_base = worldToBase(ctx.object_world);
  const Vec3 zone_base = worldToBase(ctx.zone_world);
  ctx.plan = makePickPlacePlan(object_base, zone_base, geom_);
  const auto wps = ctx.plan.waypoints();
  for (size_t i = 0; i < wps.size(); ++i) {
    const auto & wp = wps[i];
    const std::string why = checkWorkspace(wp.tcp, workspace_);
    if (!why.empty()) {
      throw TaskError(ErrorCode::UNREACHABLE, wp.name + " " + xyz(wp.tcp) + ": " + why + " - 움직이지 않음");
    }
    const double err = robot_->reachabilityErrorMm(flangePoseMmDeg(wp.tcp, geom_, tool_zyz_deg_));
    if (err < 0.0) {
      throw TaskError(ErrorCode::NOT_READY, "역기구학(ikin/fkin) 서비스 응답 없음");
    }
    if (err > reach_tolerance_mm_) {
      throw TaskError(ErrorCode::UNREACHABLE, wp.name + " " + xyz(wp.tcp) +
              fmt(": 로봇이 닿지 않음 (IK->FK 오차 %.1f mm) - 움직이지 않음", err, 0, 0));
    }
    report(ctx, wp.name + " " + xyz(wp.tcp) + " (base) 확인", (i + 1.0) / wps.size());
  }
}

// ---- 5. PICK: 열기 → 물체 위 → 하강 → 닫기(잡았는지 확인) → 들어 올리기
void TaskManagerNode::stagePick(TaskContext & ctx)
{
  enterStage(ctx, Stage::PICK, "집기 (시도 " + std::to_string(ctx.grasp_attempt) + "/" +
    std::to_string(max_grasp_attempts_) + ")");
  auto g = gripper_->open();
  if (!g.ok) {
    throw TaskError(ErrorCode::MOTION_FAILED, "그리퍼 열기 실패: " + g.message);
  }
  moveTcp(ctx, ctx.plan.above_object, "물체 위로", true);
  moveTcp(ctx, ctx.plan.grasp, "집기 위치로 하강");
  report(ctx, "그리퍼 닫기", 0.6);
  g = gripper_->close(grip_force_);
  const bool wrong_object = !ctx.object_id.empty() && !g.object_id.empty() && g.object_id != ctx.object_id;
  if (!g.ok || !g.holding || wrong_object) {
    gripper_->open();
    moveTcp(ctx, ctx.plan.above_object, "빈손으로 위로");
    throw TaskError(ErrorCode::GRASP_FAILED, "잡지 못함: " + g.message +
            (wrong_object ? " (다른 물체 " + g.object_id + ")" : ""));
  }
  ctx.holding = true;
  report(ctx, "잡음: " + (g.object_id.empty() ? ctx.target_label : g.object_id), 0.7);
  moveTcp(ctx, ctx.plan.above_object, "들어 올리기");
}

// ---- 6. PLACE: 구역 위 → 하강 → 열기 → 위로 빠지기
void TaskManagerNode::stagePlace(TaskContext & ctx)
{
  enterStage(ctx, Stage::PLACE, ctx.dest_label + "에 놓기");
  moveTcp(ctx, ctx.plan.above_zone, "구역 위로", true);
  moveTcp(ctx, ctx.plan.place, "놓는 위치로 하강");
  auto g = gripper_->open();
  if (!g.ok) {
    throw TaskError(ErrorCode::MOTION_FAILED, "그리퍼 열기 실패: " + g.message);
  }
  ctx.holding = false;
  report(ctx, "놓음", 0.7);
  moveTcp(ctx, ctx.plan.above_zone, "위로 빠지기");
}

// ---- 7. VERIFY: 홈으로 돌아가 카메라로 물체가 구역 안에 있는지 다시 확인 (닫힌 고리)
void TaskManagerNode::stageVerify(TaskContext & ctx)
{
  enterStage(ctx, Stage::VERIFY, "관찰 자세로 돌아가 카메라로 결과 확인");
  robot_->moveJoints(home_deg_, "관찰 자세(홈)", abortCheck(ctx));
  sleepChecked(ctx, settle_time_s_);
  const Estimate e = perceive(ctx, "놓은 뒤");
  if (e.status != EstimateStatus::OK) {
    throw TaskError(ErrorCode::PLACE_VERIFY_FAILED, "놓은 뒤 물체를 확인하지 못함: " + e.reason);
  }
  const double err = normXY(e.mean - ctx.zone_world);
  ctx.result->placed_at = toPoint(e.mean);
  ctx.result->place_error_mm = static_cast<float>(err * 1000.0);
  if (err > zone_tolerance_m_) {
    throw TaskError(ErrorCode::PLACE_VERIFY_FAILED,
            ctx.dest_label + fmt(" 밖: 중심에서 %.1f mm (허용 %.0f mm)", err * 1000, zone_tolerance_m_ * 1000, 0));
  }
  report(ctx, ctx.dest_label + " 안 확인: 물체 " + xyz(e.mean) + fmt(", 중심에서 %.1f mm", err * 1000, 0, 0), 1.0);
}

// ---- 실패·취소 후 처리
// 취소: 정지만 한다. 사용자가 "멈춰" 라고 했는데 로봇이 더 움직이면 놀란다.
// 실패: 정지 → (낮은 곳에 있으면) 위로 피하기 → 홈. 물체를 잡고 있으면 놓지 않는다(떨어뜨리지 않기).
void TaskManagerNode::recover(TaskContext & ctx, ErrorCode cause)
{
  const Stage failed = ctx.stage;
  ctx.stage = Stage::RECOVER;
  report(ctx, std::string("정지 (원인 ") + errorName(cause) + ")");
  robot_->stop();
  if (cause == ErrorCode::CANCELED) {
    report(ctx, ctx.holding ? "취소: 정지만 함 (물체는 잡은 채)" : "취소: 정지만 함");
    return;
  }
  const bool arm_may_be_low = (failed == Stage::PICK || failed == Stage::PLACE);
  if (!return_home_on_failure_ || !arm_may_be_low || ctx.goal.dry_run) {
    return;
  }
  // 복구 이동에는 취소를 적용하지 않고 이동 시간 초과만 적용한다.
  const RobotClient::AbortCheck only_timeout = []() {return ErrorCode::NONE;};
  try {
    if (auto pose = robot_->currentPose()) {
      Vec3 tcp = tcpFromFlangeMm(*pose, geom_);
      const double safe_z = std::min(tcp.z + geom_.approach_height_m, workspace_.z_max_m);
      if (tcp.z < safe_z - 0.005) {
        tcp.z = safe_z;
        report(ctx, "복구: 위로 피하기");
        robot_->moveLinear(flangePoseMmDeg(tcp, geom_, tool_zyz_deg_), "복구: 위로", only_timeout);
      }
    }
    report(ctx, "복구: 홈으로");
    robot_->moveJoints(home_deg_, "복구: 홈", only_timeout);
    report(ctx, "복구 완료");
  } catch (const TaskError & e) {
    RCLCPP_ERROR(get_logger(), "복구 이동 실패: %s (사람이 확인해야 함)", e.what());
  }
}

// =====================================================================================
// 도우미
// =====================================================================================
void TaskManagerNode::moveTcp(TaskContext & ctx, const Vec3 & tcp_base, const std::string & label, bool free)
{
  report(ctx, label + " " + xyz(baseToWorld(tcp_base)) + " (" + world_frame_ + ")",
    std::min(0.95, ctx.stage_fraction + 0.15));
  const auto pose = flangePoseMmDeg(tcp_base, geom_, tool_zyz_deg_);
  if (free) {
    robot_->moveFree(pose, label, abortCheck(ctx));
  } else {
    robot_->moveLinear(pose, label, abortCheck(ctx));
  }
}

void TaskManagerNode::enterStage(TaskContext & ctx, Stage stage, const std::string & detail)
{
  ctx.stage = stage;
  ctx.stage_fraction = 0.0;
  publishStatus(TaskStatus::STATE_BUSY, ctx.task_id, stageName(stage), detail);
  report(ctx, detail, 0.0);
}

void TaskManagerNode::report(TaskContext & ctx, const std::string & detail, double stage_fraction)
{
  auto fb = std::make_shared<ExecuteTask::Feedback>();
  const int index = std::min(static_cast<int>(ctx.stage), kMainStageCount);
  fb->stage = stageName(ctx.stage);
  fb->stage_index = static_cast<uint8_t>(index);
  fb->stage_count = kMainStageCount;
  fb->detail = detail;
  if (stage_fraction >= 0.0) {  // 음수 = 진행률은 그대로 두고 설명만 보냄
    ctx.stage_fraction = std::max(ctx.stage_fraction, std::min(1.0, stage_fraction));
  }
  fb->progress = static_cast<float>(std::min(1.0, (index + ctx.stage_fraction) / kMainStageCount));
  RCLCPP_INFO(get_logger(), "[%s] %s", fb->stage.c_str(), detail.c_str());
  try {
    if (ctx.goal_handle->is_active()) {
      ctx.goal_handle->publish_feedback(fb);
    }
  } catch (const std::exception &) {
    // 결과를 이미 보낸 뒤라면 feedback 은 보낼 수 없다 — 무시
  }
}

ErrorCode TaskManagerNode::abortReason(const TaskContext & ctx) const
{
  if (!rclcpp::ok()) {return ErrorCode::CANCELED;}
  if (ctx.goal_handle->is_canceling()) {return ErrorCode::CANCELED;}
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - ctx.start).count();
  if (elapsed > task_timeout_s_) {return ErrorCode::TIMEOUT;}
  return ErrorCode::NONE;
}

RobotClient::AbortCheck TaskManagerNode::abortCheck(const TaskContext & ctx) const
{
  return [this, &ctx]() {return abortReason(ctx);};
}

void TaskManagerNode::sleepChecked(TaskContext & ctx, double seconds)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    const ErrorCode why = abortReason(ctx);
    if (why != ErrorCode::NONE) {
      throw TaskError(why, "대기 중 중단");
    }
    std::this_thread::sleep_for(50ms);
  }
}

// ---- TF2: world <-> base_link
bool TaskManagerNode::refreshTransforms(std::string * error)
{
  for (int i = 0; i < 20; ++i) {   // 최대 2 초 기다린다 (TF 는 별도 스레드가 채워 준다)
    try {
      T_base_world_ = tf_buffer_->lookupTransform(base_frame_, world_frame_, tf2::TimePointZero);
      T_world_base_ = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & e) {
      if (error) {*error = e.what();}
      std::this_thread::sleep_for(100ms);
    }
  }
  return false;
}

Vec3 TaskManagerNode::worldToBase(const Vec3 & p) const
{
  geometry_msgs::msg::PointStamped in, out;
  in.point = toPoint(p);
  tf2::doTransform(in, out, T_base_world_);
  return {out.point.x, out.point.y, out.point.z};
}

Vec3 TaskManagerNode::baseToWorld(const Vec3 & p) const
{
  geometry_msgs::msg::PointStamped in, out;
  in.point = toPoint(p);
  tf2::doTransform(in, out, T_world_base_);
  return {out.point.x, out.point.y, out.point.z};
}

// ---- 카메라 결과 받기 (구독 콜백: 짧게 처리하고 버퍼에 넣기만 한다)
void TaskManagerNode::onDetections(const DetectedObjectArray::SharedPtr msg)
{
  std::vector<Observation> objs;
  objs.reserve(msg->objects.size());
  for (const auto & o : msg->objects) {
    const std::string frame = o.pose.header.frame_id.empty() ? msg->header.frame_id : o.pose.header.frame_id;
    Vec3 p{o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z};
    if (frame != world_frame_) {
      try {
        geometry_msgs::msg::PointStamped in, out;
        in.header.frame_id = frame;
        in.point = o.pose.pose.position;
        out = tf_buffer_->transform(in, world_frame_);
        p = {out.point.x, out.point.y, out.point.z};
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "검출 좌표 변환 실패 (%s -> %s): %s",
          frame.c_str(), world_frame_.c_str(), e.what());
        continue;
      }
    }
    objs.push_back(Observation{o.id, o.color, o.shape, p, o.confidence});
  }
  std::lock_guard<std::mutex> lk(frames_mutex_);
  frames_.push_back(Frame{++frame_seq_, std::move(objs)});
  while (frames_.size() > kMaxBufferedFrames) {frames_.pop_front();}
  last_frame_time_ = std::chrono::steady_clock::now();
}

// ---- 상태 토픽
void TaskManagerNode::publishStatus(
  uint8_t state, const std::string & task_id, const std::string & stage, const std::string & message)
{
  std::lock_guard<std::mutex> lk(status_mutex_);
  last_status_.state = state;
  last_status_.task_id = task_id;
  last_status_.stage = stage;
  last_status_.message = message;
  last_status_.stamp = now();
  status_pub_->publish(last_status_);
}

void TaskManagerNode::onStatusTimer()
{
  std::lock_guard<std::mutex> lk(status_mutex_);
  last_status_.stamp = now();
  status_pub_->publish(last_status_);
}

}  // namespace aris3_task_manager
