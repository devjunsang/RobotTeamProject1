// ARIS 의 ROS 2 쪽: GuiNode 하나가 별도 스레드의 executor 에서 돈다 (aris3_gui ros_io.py 와 같은 구조).
// 받은 값은 잠금(mutex) 아래 보관하고, Qt 타이머가 100 ms 마다 snapshot() 으로 복사해 화면을 그린다.
// 로봇을 움직이는 경로는 ARIS3 공통 인터페이스뿐이다: 작업 = /aris3/execute_task (Task Manager 가 검증·계획·실패 처리).
// 정지는 언제나 허용: 작업 취소 + MoveIt 실행 중단(/trajectory_execution_event "stop").
#pragma once

#include <QImage>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "aris3_gui_cpp/gui_logic.hpp"
#include "aris3_interfaces/action/execute_task.hpp"
#include "aris3_interfaces/msg/detected_object_array.hpp"
#include "aris3_interfaces/msg/task_status.hpp"
#include "aris3_interfaces/msg/voice_command.hpp"
#include "aris3_interfaces/srv/gripper_command.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace aris
{

// 로그 한 줄
struct LogLine
{
  uint64_t seq;
  std::string level, node, text, time;
};

// 화면이 한 번에 읽는 상태 복사본
struct Snapshot
{
  std::map<std::string, double> joints;                     // 관절 이름 -> rad
  QImage image;                                             // 마지막 카메라 영상 (원본 또는 JPEG 을 푼 것)
  uint64_t image_seq{0};                                    // 새 영상마다 +1
  std::vector<aris3_interfaces::msg::DetectedObject> objects;     // world 좌표 검출 (시뮬레이션 정답 등)
  std::vector<aris3_interfaces::msg::DetectedObject> px_objects;  // 영상 픽셀 검출 (색 검출)
  std::optional<aris3_interfaces::msg::TaskStatus> task_status;
  std::optional<aris3_interfaces::msg::VoiceCommand> voice;
  std::string voice_state, gesture_state;
  std::optional<aris3_interfaces::action::ExecuteTask::Feedback> feedback;
  std::optional<std::pair<bool, std::string>> last_result;  // (성공, 문장)
  StageTracker stages;
  bool task_running{false};
  bool manual_busy{false};                                  // 수동 이동(MoveIt) 진행 중
  std::string image_frame;                                  // 영상의 frame_id (카메라 TF 찾기용)
  std::vector<LogLine> logs;                                // 최근 400 줄
  std::string gripper_msg;
};

// 각 구성 요소가 살아 있는가 (시스템 상태 패널)
struct StatusFlags
{
  bool gazebo{false}, joint_states{false}, px_detections{false}, detections{false};
  bool gripper{false}, moveit{false}, task_manager{false}, voice{false}, gesture{false};
  double js_hz{0}, camera_hz{0};
};

class GuiNode : public rclcpp::Node
{
public:
  using ExecuteTask = aris3_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  // robot_ns: 시뮬레이션 로봇 이름 공간, camera_topic: 원본 영상 (압축은 + "/compressed")
  explicit GuiNode(const std::string & robot_ns = "/e0509_gripper",
    const std::string & camera_topic = "/aris3/overhead_camera/image");

  Snapshot snapshot();
  StatusFlags statusFlags();

  // GUI 로그 (화면 + ~/.ros/log). level: INFO / WARN / ERROR
  void log(const std::string & level, const std::string & text);

  // 작업 요청: target_id 가 있으면 색 대신 그 물체, has_place 면 구역 대신 (x, y) [m world]
  bool sendTask(const std::string & color, const std::string & zone, bool dry_run,
    const std::string & target_id = "", bool has_place = false, double place_x = 0, double place_y = 0);
  void cancelTask();
  // 정지: 작업 취소 + MoveIt 실행 중단. 언제든 허용 (소프트웨어 정지 - 인증된 비상정지가 아님)
  void stopAll();
  bool taskRunning();

  // 그리퍼 (시뮬레이션 /aris3/gripper/command): 0 열기, 1 닫기
  void gripper(uint8_t command);

  // ---- TF
  // world -> 카메라 광학 좌표 변환 (영상 frame_id 기준). 없으면 false
  bool cameraTransform(Vec3 & t, Quat & q);
  // 손끝(TCP) world 위치 [m] 와 RPY [rad]: link_6 + 공구 z 방향 tcp_offset. 없으면 false
  bool tcpPose(Vec3 & tcp, Vec3 & rpy, double tcp_offset = 0.105);

  // ---- 수동 조작 (MoveIt /move_action) - 한 번에 하나, 작업 중에는 거부 (aris3_gui ros_io 와 같은 제한)
  bool manualBusy();
  void home(double vel);                                    // 홈(관찰) 자세 [0, 0, 90, 0, 90, 0]°
  void jogJoint(int index, double step_deg, double vel);    // 관절 하나를 step 만큼 (최대 10°)
  void jogTcp(int axis, double step_mm, double vel);        // link_6 를 world X/Y/Z 로 직선 (최대 20 mm, Pilz LIN)

  // 음성: 음성 명령 모드 ON/OFF (켜져 있는 동안 음성 노드가 계속 듣고 「아리스 ...」 만 명령으로 냄) / 문장 입력
  void voiceListen(bool on);
  void voiceActive(bool on);
  void voiceText(const std::string & text);

private:
  void addLog(const std::string & level, const std::string & node, const std::string & text);
  void onImage(const sensor_msgs::msg::Image::SharedPtr m);
  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr m);
  void onResult(const GoalHandle::WrappedResult & r);
  void moveGoal(const moveit_msgs::msg::Constraints & c, const std::string & label,
    const std::string & pipeline, const std::string & planner, double vel);
  moveit_msgs::msg::Constraints jointConstraints(const std::vector<double> & target_rad);

  std::mutex mtx_;
  Snapshot s_;
  uint64_t log_seq_{0};
  RateMeter js_rate_, clock_rate_, cam_rate_, px_rate_, det_rate_, voice_rate_, gesture_rate_;
  GoalHandle::SharedPtr goal_;

  rclcpp::CallbackGroup::SharedPtr cb_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr task_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_client_;   // 준비 여부 확인용 (조그는 M2)
  rclcpp::Client<aris3_interfaces::srv::GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_listen_, pub_active_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_text_, pub_traj_event_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace aris
