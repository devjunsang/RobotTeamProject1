// ARIS GuiNode 구현 (ros_io.hpp 설명 참고).
#include "aris3_gui_cpp/ros_io.hpp"

#include <QByteArray>

#include <chrono>
#include <ctime>

#include "rclcpp/qos.hpp"

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
  subs_.push_back(create_subscription<rcl_interfaces::msg::Log>("/rosout", 100,
    [this](rcl_interfaces::msg::Log::SharedPtr m) {
      if (m->name == get_name()) {return;}       // 자기 로그는 이미 화면에 넣었음
      addLog(levelName(m->level), m->name, m->msg);
    }, o));

  // ---- 명령 쪽
  task_client_ = rclcpp_action::create_client<ExecuteTask>(this, "/aris3/execute_task", cb_);
  move_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/move_action", cb_);
  gripper_client_ = create_client<aris3_interfaces::srv::GripperCommand>("/aris3/gripper/command",
    rclcpp::ServicesQoS(), cb_);
  pub_listen_ = create_publisher<std_msgs::msg::Bool>("/aris3/voice/listen", 10);
  pub_text_ = create_publisher<std_msgs::msg::String>("/aris3/voice/text", 10);
  pub_traj_event_ = create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
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
  return s_;
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

void GuiNode::voiceListen(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  pub_listen_->publish(m);
}

void GuiNode::voiceText(const std::string & text)
{
  std_msgs::msg::String m;
  m.data = text;
  pub_text_->publish(m);
}

}  // namespace aris
