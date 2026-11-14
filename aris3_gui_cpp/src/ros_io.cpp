// ARIS GuiNode 구현 (ros_io.hpp 설명 참고).
#include "aris3_gui_cpp/ros_io.hpp"

#include <QByteArray>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2/exceptions.h"

namespace aris
{

namespace
{
// 지금 시각 "HH:MM:SS" (로그 표시용)
std::string nowText()
{
  std::time_t t = std::time(nullptr);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&t));
  return buf;
}

// rcl_interfaces/Log 수준 숫자 -> 글자
std::string levelName(uint8_t l)
{
  switch (l) {
    case 10: return "DEBUG";
    case 20: return "INFO";
    case 30: return "WARN";
    case 40: return "ERROR";
    case 50: return "FATAL";
    default: return std::to_string(l);
  }
}
}  // namespace

GuiNode::GuiNode(const std::string & robot_ns, const std::string & camera_topic)
: rclcpp::Node("aris_gui")
{
  using std::placeholders::_1;
  namespace msg = aris3_interfaces::msg;
  cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions o;
  o.callback_group = cb_;
  const auto sensor = rclcpp::SensorDataQoS();

  // ---- 구독: 받은 값은 잠금 아래 보관만 한다 (그리기는 Qt 스레드)
  subs_.push_back(create_subscription<sensor_msgs::msg::JointState>(robot_ns + "/joint_states", 50,
    [this](sensor_msgs::msg::JointState::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {s_.joints[m->name[i]] = m->position[i];}
      js_rate_.tick();
    }, o));
  subs_.push_back(create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10,
    [this](rosgraph_msgs::msg::Clock::SharedPtr) {std::lock_guard<std::mutex> lk(mtx_); clock_rate_.tick();}, o));
  subs_.push_back(create_subscription<sensor_msgs::msg::Image>(camera_topic, sensor,
    std::bind(&GuiNode::onImage, this, _1), o));
  // JPEG 압축 영상 (image_transport 이름 규칙 '<토픽>/compressed') - 실물 카메라·실물 비슷한 시뮬레이션
  subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(camera_topic + "/compressed", sensor,
    std::bind(&GuiNode::onCompressed, this, _1), o));
  subs_.push_back(create_subscription<msg::DetectedObjectArray>("/aris3/detected_objects", 10,
    [this](msg::DetectedObjectArray::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_); s_.objects = m->objects; det_rate_.tick();
    }, o));
  // 픽셀 좌표 검출 (scripts/aris3_color_detector.py). world 좌표가 아니므로 화면 표시에만 쓴다.
  subs_.push_back(create_subscription<msg::DetectedObjectArray>("/aris3/vision/detections_px", 10,
    [this](msg::DetectedObjectArray::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_); s_.px_objects = m->objects; px_rate_.tick();
    }, o));
  subs_.push_back(create_subscription<msg::TaskStatus>("/aris3/task_status", 10,
    [this](msg::TaskStatus::SharedPtr m) {std::lock_guard<std::mutex> lk(mtx_); s_.task_status = *m;}, o));
  subs_.push_back(create_subscription<msg::VoiceCommand>("/aris3/voice_command", 10,
    [this](msg::VoiceCommand::SharedPtr m) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        s_.voice = *m;
      }
      addLog("INFO", "gui", "음성: 「" + m->text + "」 -> " + (m->action.empty() ? "?" : m->action) + " " +
        (m->color.empty() ? "?" : m->color) + (m->valid ? " (실행 가능)" : " - " + m->message));
    }, o));
  subs_.push_back(create_subscription<std_msgs::msg::String>("/aris3/voice/state", 10,
    [this](std_msgs::msg::String::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_); s_.voice_state = m->data; voice_rate_.tick();
    }, o));
  subs_.push_back(create_subscription<std_msgs::msg::String>("/aris3/gesture/state", 10,
    [this](std_msgs::msg::String::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_); s_.gesture_state = m->data; gesture_rate_.tick();
    }, o));
  // 손짓 (실물 위쪽 카메라, MediaPipe): 매 영상 손끝·손짓, 선택·확정 이벤트
  subs_.push_back(create_subscription<msg::HandGesture>("/aris3/gesture", 10,
    [this](msg::HandGesture::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      s_.gesture = *m;
      gesture_time_ = std::chrono::steady_clock::now();
    }, o));
  subs_.push_back(create_subscription<msg::GestureEvent>("/aris3/gesture/event", 10,
    [this](msg::GestureEvent::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      gesture_events_.push_back(*m);
      if (gesture_events_.size() > 20) {gesture_events_.erase(gesture_events_.begin());}
    }, o));
  subs_.push_back(create_subscription<rcl_interfaces::msg::Log>("/rosout", 100,
    [this](rcl_interfaces::msg::Log::SharedPtr m) {
      if (m->name == get_name()) {return;}       // 자기 로그는 이미 화면에 넣었음
      if (isIgnoredSpeechLog(m->name, m->msg)) {return;}   // 「아리스」 없이 들린 말은 화면에 넣지 않는다 (2026-11-14)
      addLog(levelName(m->level), m->name, m->msg);
    }, o));

  // ---- 명령 쪽
  task_client_ = rclcpp_action::create_client<ExecuteTask>(this, "/aris3/execute_task", cb_);
  move_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/move_action", cb_);
  gripper_client_ = create_client<aris3_interfaces::srv::GripperCommand>("/aris3/gripper/command",
    rclcpp::ServicesQoS(), cb_);
  pub_listen_ = create_publisher<std_msgs::msg::Bool>("/aris3/voice/listen", 10);
  pub_active_ = create_publisher<std_msgs::msg::Bool>("/aris3/voice/active", 10);   // 음성 명령 모드 ON/OFF
  pub_text_ = create_publisher<std_msgs::msg::String>("/aris3/voice/text", 10);
  pub_say_ = create_publisher<std_msgs::msg::String>("/aris3/voice/say", 10);     // ARIS 목소리 (음성 노드가 말함)
  pub_traj_event_ = create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
}

// ---------------------------------------------------------------- 영상
void GuiNode::onImage(const sensor_msgs::msg::Image::SharedPtr m)
{
  // 원본 rgb8 / bgr8 -> QImage (깊은 복사: 메시지 버퍼는 콜백이 끝나면 사라짐)
  QImage img;
  if (m->encoding == "rgb8") {
    img = QImage(m->data.data(), m->width, m->height, m->step, QImage::Format_RGB888).copy();
  } else if (m->encoding == "bgr8") {
    img = QImage(m->data.data(), m->width, m->height, m->step, QImage::Format_RGB888).rgbSwapped();
  } else {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  s_.image = img;
  s_.image_frame = m->header.frame_id;
  ++s_.image_seq;
  cam_rate_.tick();
}

void GuiNode::onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr m)
{
  // JPEG 풀기는 ROS 콜백 스레드에서 (Qt 화면 스레드를 막지 않게)
  QImage img;
  if (!img.loadFromData(m->data.data(), static_cast<int>(m->data.size()))) {return;}
  std::lock_guard<std::mutex> lk(mtx_);
  s_.image = img;
  s_.image_frame = m->header.frame_id;
  ++s_.image_seq;
  cam_rate_.tick();
}

// ---------------------------------------------------------------- 로그
void GuiNode::addLog(const std::string & level, const std::string & node, const std::string & text)
{
  std::lock_guard<std::mutex> lk(mtx_);
  s_.logs.push_back({++log_seq_, level, node, text, nowText()});
  if (s_.logs.size() > 400) {s_.logs.erase(s_.logs.begin(), s_.logs.begin() + (s_.logs.size() - 400));}
}

void GuiNode::log(const std::string & level, const std::string & text)
{
  addLog(level, "gui", text);
  // 화면 로그와 같은 내용을 ~/.ros/log 파일에도 (원인 찾기용)
  if (level == "WARN") {
    RCLCPP_WARN(get_logger(), "%s", text.c_str());
  } else if (level == "ERROR") {
    RCLCPP_ERROR(get_logger(), "%s", text.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }
}

// ---------------------------------------------------------------- 상태 읽기
Snapshot GuiNode::snapshot()
{
  std::lock_guard<std::mutex> lk(mtx_);
  Snapshot out = s_;
  // 손짓은 0.5 초 안에 받은 것만 (손 인식 노드가 멈추면 포인터를 지운다)
  if (std::chrono::steady_clock::now() - gesture_time_ > std::chrono::milliseconds(500)) {out.gesture.reset();}
  return out;
}

std::vector<aris3_interfaces::msg::GestureEvent> GuiNode::popGestureEvents()
{
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<aris3_interfaces::msg::GestureEvent> out;
  out.swap(gesture_events_);
  return out;
}

StatusFlags GuiNode::statusFlags()
{
  StatusFlags f;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    f.gazebo = clock_rate_.fresh(2.0);
    f.joint_states = js_rate_.fresh(1.0);
    f.js_hz = js_rate_.hz();
    f.camera_hz = cam_rate_.hz();
    f.px_detections = px_rate_.fresh(2.0);
    f.detections = det_rate_.fresh(2.0);
    f.voice = voice_rate_.fresh(3.0);
    f.gesture = gesture_rate_.fresh(3.0);
  }
  f.gripper = gripper_client_->service_is_ready();
  f.moveit = move_client_->action_server_is_ready();
  f.task_manager = task_client_->action_server_is_ready();
  return f;
}

bool GuiNode::taskRunning()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return s_.task_running;
}

// ---------------------------------------------------------------- 작업 (ExecuteTask)
bool GuiNode::sendTask(const std::string & color, const std::string & zone, bool dry_run,
  const std::string & target_id, bool has_place, double place_x, double place_y)
{
  if (taskRunning()) {
    log("WARN", "이미 작업 중 - 요청 무시");
    return false;
  }
  if (!task_client_->action_server_is_ready()) {
    log("ERROR", "Task Manager(/aris3/execute_task) 없음");
    return false;
  }
  ExecuteTask::Goal g;
  g.task_type = "pick_place";
  g.target_color = color;
  g.destination = zone;
  g.dry_run = dry_run;
  g.target_id = target_id;
  g.use_place_position = has_place;
  g.place_position.x = place_x;
  g.place_position.y = place_y;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    s_.task_running = true;
    s_.stages.reset();
    s_.feedback.reset();
    s_.last_result.reset();
  }
  log("INFO", "작업 요청: " + (target_id.empty() ? color : target_id) + " -> " +
    (has_place ? "(" + std::to_string(place_x).substr(0, 5) + ", " + std::to_string(place_y).substr(0, 5) + ")" :
    "구역 " + zone) + (dry_run ? " (dry_run)" : ""));

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions opt;
  opt.goal_response_callback = [this](GoalHandle::SharedPtr h) {
      if (!h) {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          s_.task_running = false;
          s_.last_result = std::make_pair(false, std::string("Task Manager 가 작업을 거절함 (이미 작업 중?)"));
        }
        log("ERROR", "작업 거절됨");
        return;
      }
      std::lock_guard<std::mutex> lk(mtx_);
      goal_ = h;
    };
  opt.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteTask::Feedback> fb) {
      std::lock_guard<std::mutex> lk(mtx_);
      s_.feedback = *fb;
      s_.stages.feedback(fb->stage);
    };
  opt.result_callback = std::bind(&GuiNode::onResult, this, std::placeholders::_1);
  task_client_->async_send_goal(g, opt);
  return true;
}

void GuiNode::onResult(const GoalHandle::WrappedResult & r)
{
  const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->success;
  const std::string text = r.result ? r.result->message : std::string("결과 없음");
  {
    std::lock_guard<std::mutex> lk(mtx_);
    s_.stages.finish(ok, r.result ? r.result->failed_stage : "");
    s_.task_running = false;
    s_.last_result = std::make_pair(ok, text);
    goal_.reset();
  }
  log(ok ? "INFO" : "ERROR", std::string("작업 ") + (ok ? "성공: " : "실패: ") + text);
}

void GuiNode::cancelTask()
{
  GoalHandle::SharedPtr h;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    h = goal_;
  }
  if (h) {
    task_client_->async_cancel_goal(h);
    log("WARN", "작업 취소 요청");
  }
}

void GuiNode::stopAll()
{
  cancelTask();
  std_msgs::msg::String m;
  m.data = "stop";
  pub_traj_event_->publish(m);
  log("WARN", "정지: 작업 취소 + MoveIt 실행 중단");
}

// ---------------------------------------------------------------- 그리퍼·음성
void GuiNode::gripper(uint8_t command)
{
  if (taskRunning()) {
    log("WARN", "작업 중에는 그리퍼를 직접 조작하지 않음");
    return;
  }
  if (!gripper_client_->service_is_ready()) {
    log("ERROR", "그리퍼 서비스(/aris3/gripper/command) 없음");
    return;
  }
  auto req = std::make_shared<aris3_interfaces::srv::GripperCommand::Request>();
  req->command = command;
  req->force = 0.5;
  gripper_client_->async_send_request(req,
    [this](rclcpp::Client<aris3_interfaces::srv::GripperCommand>::SharedFuture f) {
      auto r = f.get();
      {
        std::lock_guard<std::mutex> lk(mtx_);
        s_.gripper_msg = r ? r->message : "응답 없음";
      }
      log(r && r->success ? "INFO" : "ERROR", "그리퍼: " + (r ? r->message : std::string("응답 없음")));
    });
}

// ---------------------------------------------------------------- TF
bool GuiNode::cameraTransform(Vec3 & t, Quat & q)
{
  std::string frame;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    frame = s_.image_frame;
  }
  if (frame.empty()) {return false;}
  try {
    const auto tr = tf_->lookupTransform(frame, "world", tf2::TimePointZero);
    t = {tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z};
    q = {tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w};
    return true;
  } catch (const tf2::TransformException &) {
    return false;     // 실물 카메라처럼 카메라 TF 가 없을 수 있음 (그때는 픽셀 검출만 그림)
  }
}

bool GuiNode::tcpPose(Vec3 & tcp, Vec3 & rpy, double tcp_offset)
{
  try {
    const auto tr = tf_->lookupTransform("world", "link_6", tf2::TimePointZero);
    const auto & p = tr.transform.translation;
    const auto & q = tr.transform.rotation;
    // 공구 z 축 = 회전 행렬의 3번째 열 -> 집는 점은 link_6 에서 그 방향으로 tcp_offset
    const double zx = 2 * (q.x * q.z + q.w * q.y), zy = 2 * (q.y * q.z - q.w * q.x), zz = 1 - 2 * (q.x * q.x + q.y * q.y);
    tcp = {p.x + zx * tcp_offset, p.y + zy * tcp_offset, p.z + zz * tcp_offset};
    rpy = {std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
      std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)),
      std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// ---------------------------------------------------------------- 수동 조작 (MoveIt)
bool GuiNode::manualBusy()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return s_.manual_busy;
}

void GuiNode::moveGoal(const moveit_msgs::msg::Constraints & c, const std::string & label,
  const std::string & pipeline, const std::string & planner, double vel)
{
  // 안전: 작업 중에는 수동 조작을 받지 않고, 한 번에 하나만
  if (taskRunning()) {
    log("WARN", label + ": 작업 중에는 수동 조작 불가");
    return;
  }
  if (manualBusy()) {
    log("WARN", label + ": 이전 수동 이동이 아직 끝나지 않음");
    return;
  }
  if (!move_client_->action_server_is_ready()) {
    log("ERROR", "MoveIt(/move_action) 없음");
    return;
  }
  moveit_msgs::action::MoveGroup::Goal g;
  auto & r = g.request;
  r.group_name = "arm";
  r.pipeline_id = pipeline;
  r.planner_id = planner;
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 3.0;
  r.max_velocity_scaling_factor = vel;
  r.max_acceleration_scaling_factor = vel;
  r.goal_constraints = {c};
  g.planning_options.plan_only = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    s_.manual_busy = true;
  }
  log("INFO", "수동: " + label);
  using MG = moveit_msgs::action::MoveGroup;
  rclcpp_action::Client<MG>::SendGoalOptions opt;
  opt.goal_response_callback = [this, label](rclcpp_action::ClientGoalHandle<MG>::SharedPtr h) {
      if (!h) {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          s_.manual_busy = false;
        }
        log("ERROR", label + ": MoveIt 이 거절");
      }
    };
  opt.result_callback = [this, label](const rclcpp_action::ClientGoalHandle<MG>::WrappedResult & res) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        s_.manual_busy = false;
      }
      const int code = res.result ? res.result->error_code.val : 0;
      if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        log("INFO", label + ": 완료");
      } else {
        log("ERROR", label + ": 실패 (MoveIt error_code " + std::to_string(code) +
          " - 도달 불가·충돌·관절 한계일 수 있음)");
      }
    };
  move_client_->async_send_goal(g, opt);
}

moveit_msgs::msg::Constraints GuiNode::jointConstraints(const std::vector<double> & target_rad)
{
  moveit_msgs::msg::Constraints c;
  for (size_t i = 0; i < target_rad.size(); ++i) {
    moveit_msgs::msg::JointConstraint j;
    j.joint_name = "joint_" + std::to_string(i + 1);
    j.position = target_rad[i];
    j.tolerance_above = j.tolerance_below = 0.005;
    j.weight = 1.0;
    c.joint_constraints.push_back(j);
  }
  return c;
}

void GuiNode::home(double vel)
{
  const double d = M_PI / 180.0;
  moveGoal(jointConstraints({0.0, 0.0, 90 * d, 0.0, 90 * d, 0.0}), "홈 자세", "ompl", "", vel);
}

void GuiNode::jogJoint(int index, double step_deg, double vel)
{
  const double step = std::clamp(step_deg, -10.0, 10.0);       // 실물 한계(e0509_manual)와 같은 값
  std::vector<double> cur(6);
  bool ok = true;
  {
    // 잠금 안에서는 값만 읽는다 (log() 도 같은 잠금을 쓰므로 여기서 부르면 멈춤)
    std::lock_guard<std::mutex> lk(mtx_);
    ok = js_rate_.fresh();
    for (int i = 0; i < 6 && ok; ++i) {
      auto it = s_.joints.find("joint_" + std::to_string(i + 1));
      ok = it != s_.joints.end();
      if (ok) {cur[i] = it->second;}
    }
  }
  if (!ok) {
    log("ERROR", "관절 상태가 들어오지 않음");
    return;
  }
  cur[index] += step * M_PI / 180.0;
  char label[48];
  std::snprintf(label, sizeof(label), "J%d %+.0f°", index + 1, step);
  moveGoal(jointConstraints(cur), label, "ompl", "", vel);
}

void GuiNode::jogTcp(int axis, double step_mm, double vel)
{
  const double step = std::clamp(step_mm, -20.0, 20.0) / 1000.0;
  geometry_msgs::msg::TransformStamped tr;
  try {
    tr = tf_->lookupTransform("world", "link_6", tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    log("ERROR", std::string("현재 손끝 위치 모름 (TF): ") + e.what());
    return;
  }
  // link_6 를 world 기준 X/Y/Z 로 직선 이동, 방향 유지 (Pilz LIN)
  geometry_msgs::msg::Pose target;
  target.position.x = tr.transform.translation.x;
  target.position.y = tr.transform.translation.y;
  target.position.z = tr.transform.translation.z;
  (axis == 0 ? target.position.x : axis == 1 ? target.position.y : target.position.z) += step;
  target.orientation = tr.transform.rotation;
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = "world";
  pc.link_name = "link_6";
  pc.weight = 1.0;
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {0.001};
  pc.constraint_region.primitives = {sphere};
  pc.constraint_region.primitive_poses = {target};
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = "world";
  oc.link_name = "link_6";
  oc.orientation = target.orientation;
  oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01;
  oc.weight = 1.0;
  moveit_msgs::msg::Constraints c;
  c.position_constraints = {pc};
  c.orientation_constraints = {oc};
  char label[48];
  std::snprintf(label, sizeof(label), "TCP %c %+.0f mm", "XYZ"[axis], step * 1000);
  moveGoal(c, label, "pilz_industrial_motion_planner", "LIN", vel);
}

void GuiNode::voiceListen(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  pub_listen_->publish(m);
}

void GuiNode::voiceActive(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  pub_active_->publish(m);
}

void GuiNode::voiceText(const std::string & text)
{
  std_msgs::msg::String m;
  m.data = text;
  pub_text_->publish(m);
}

void GuiNode::say(const std::string & text)
{
  std_msgs::msg::String m;
  m.data = text;
  pub_say_->publish(m);
  log("INFO", "ARIS: 「" + text + "」");
}

}  // namespace aris

