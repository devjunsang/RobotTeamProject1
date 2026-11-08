// ARIS3 Task Manager: Pick & Place 작업 상태 머신 (ROS 2 액션 서버).
//
//   /aris3/execute_task (ExecuteTask 액션)
//        │ goal: target_color, target_shape, destination, dry_run
//        ▼
//   VALIDATE → OBSERVE → PERCEIVE → PLAN → PICK → PLACE → VERIFY → 완료
//        │ 어느 단계든 실패·취소·시간 초과 → RECOVER (정지 / 위로 피하기 / 홈) → 결과
//
// 입력  /aris3/detected_objects  (aris3_interfaces/DetectedObjectArray)  ← 비전
// 출력  /aris3/task_status       (aris3_interfaces/TaskStatus)           → GUI·모니터링
// 사용  /aris3/gripper/command   (aris3_interfaces/srv/GripperCommand)   → 그리퍼 어댑터
//       <robot_ns>/motion/*      (dsr_msgs2)                             → 두산 컨트롤러(에뮬레이터)  [robot_backend: dsr]
//       /move_action 등          (MoveIt 2 move_group)                   → ros2_control           [robot_backend: moveit]
//       TF world → base_link                                              → 좌표 변환
//
// 스레드 구조:
//   - 액션 콜백(handleGoal/handleCancel/handleAccepted)과 구독 콜백은 executor 스레드에서 빨리 끝난다.
//   - 실제 작업(execute)은 별도 std::thread 에서 돈다. 서비스 응답은 executor 의 다른 스레드가 받는다.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "aris3_interfaces/action/execute_task.hpp"
#include "aris3_interfaces/msg/detected_object_array.hpp"
#include "aris3_interfaces/msg/task_status.hpp"

#include "aris3_task_manager/geometry.hpp"
#include "aris3_task_manager/gripper_client.hpp"
#include "aris3_task_manager/robot_client.hpp"
#include "aris3_task_manager/target_estimator.hpp"
#include "aris3_task_manager/types.hpp"

namespace aris3_task_manager
{

class TaskManagerNode : public rclcpp::Node
{
public:
  using ExecuteTask = aris3_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskManagerNode() override;

private:
  // ---- 작업 하나의 진행 상황을 담는 구조체 (execute() 안에서만 사용)
  struct TaskContext
  {
    std::shared_ptr<GoalHandle> goal_handle;
    ExecuteTask::Goal goal;
    std::string task_id;
    std::chrono::steady_clock::time_point start;
    Stage stage{Stage::VALIDATE};
    double stage_fraction{0.0};   // 현재 단계 안에서의 진행 (0~1, 줄어들지 않음)
    TargetQuery query;
    Vec3 zone_world;              // 구역 중심의 테이블 표면 점 (world)
    Vec3 object_world;            // 카메라가 본 물체 중심 (world)
    std::string object_id;
    std::vector<Observation> last_objects;  // 마지막으로 받은 영상의 모든 물체 (구역 점유 확인용)
    PickPlacePlan plan;           // base frame
    bool holding{false};
    int grasp_attempt{0};
    std::shared_ptr<ExecuteTask::Result> result{std::make_shared<ExecuteTask::Result>()};
  };

  // ---- 액션 서버 콜백 (executor 스레드, 빨리 끝나야 함)
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);

  // ---- 작업 스레드
  void execute(std::shared_ptr<GoalHandle> goal_handle);
  void stageValidate(TaskContext & ctx);
  void stageObserve(TaskContext & ctx);
  void stagePerceive(TaskContext & ctx);
  void stagePlan(TaskContext & ctx);
  void stagePick(TaskContext & ctx);
  void stagePlace(TaskContext & ctx);
  void stageVerify(TaskContext & ctx);
  void recover(TaskContext & ctx, ErrorCode cause);

  Estimate perceive(TaskContext & ctx, const std::string & purpose);
  // free=true: 경로 모양 상관없이 (옮기는 구간), false: 공구 직선 (접근·하강·들어 올리기)
  void moveTcp(TaskContext & ctx, const Vec3 & tcp_base, const std::string & label, bool free = false);
  void enterStage(TaskContext & ctx, Stage stage, const std::string & detail);
  void report(TaskContext & ctx, const std::string & detail, double stage_fraction = -1.0);
  ErrorCode abortReason(const TaskContext & ctx) const;
  RobotClient::AbortCheck abortCheck(const TaskContext & ctx) const;
  void sleepChecked(TaskContext & ctx, double seconds);

  // ---- 좌표 변환 (TF2)
  Vec3 worldToBase(const Vec3 & p) const;
  Vec3 baseToWorld(const Vec3 & p) const;
  bool refreshTransforms(std::string * error);

  // ---- 입력/출력
  void onDetections(const aris3_interfaces::msg::DetectedObjectArray::SharedPtr msg);
  void publishStatus(uint8_t state, const std::string & task_id, const std::string & stage,
    const std::string & message);
  void onStatusTimer();

  static bool actionServerAlive(const std::string & action_name);

  // ---- 파라미터 (config/task_manager.yaml)
  void loadParameters();
  std::string world_frame_;
  std::string base_frame_;
  std::array<double, 6> home_deg_{};
  std::array<double, 3> tool_zyz_deg_{};
  GraspGeometry geom_;
  Workspace workspace_;
  std::map<std::string, Vec3> zones_world_;   // 구역 이름 → 테이블 표면 점 (world)
  std::vector<std::string> known_colors_;
  double zone_tolerance_m_{0.03};
  double zone_clear_radius_m_{0.06};
  int perception_frames_{5};
  double perception_timeout_s_{5.0};
  double max_spread_m_{0.005};
  double min_confidence_{0.5};
  double settle_time_s_{0.5};
  double task_timeout_s_{180.0};
  double reach_tolerance_mm_{1.0};
  double grip_force_{0.5};
  int max_grasp_attempts_{2};
  bool return_home_on_failure_{true};

  // ---- ROS 통신 객체
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Subscription<aris3_interfaces::msg::DetectedObjectArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<aris3_interfaces::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<RobotBackend> robot_;   // robot_backend: dsr(RobotClient) | moveit(MoveItRobotClient)
  std::unique_ptr<GripperClient> gripper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped T_base_world_;   // world 점을 base 로
  geometry_msgs::msg::TransformStamped T_world_base_;   // base 점을 world 로

  // ---- 카메라 결과 버퍼 (구독 콜백이 쓰고 작업 스레드가 읽음 → mutex)
  struct Frame
  {
    uint64_t seq;
    std::vector<Observation> objects;   // world frame
  };
  mutable std::mutex frames_mutex_;
  std::deque<Frame> frames_;
  uint64_t frame_seq_{0};
  std::chrono::steady_clock::time_point last_frame_time_{};

  // ---- 작업 상태
  std::atomic<bool> busy_{false};
  std::thread worker_;
  int task_counter_{0};
  std::mutex status_mutex_;
  aris3_interfaces::msg::TaskStatus last_status_;
};

}  // namespace aris3_task_manager
