// MoveIt 2 move_group 으로 로봇을 움직이는 백엔드 (robot_backend: moveit).
//
//   관절 이동 (moveJoints) : OMPL 로 충돌 없는 경로 계획 -> 실행
//   직선 이동 (moveLinear) : Pilz LIN 으로 공구 직선 경로 (접근·하강·들어 올리기)
//   자유 이동 (moveFree)   : OMPL 로 목표 자세까지 충돌 없는 경로 (물체 위로·구역 위로 옮기기)
//   도달 가능성            : 현재 로봇 모델로 IK 를 풀어 본다 (움직이기 전에 거부)
//   실행                   : move_group 이 FollowJointTrajectory 로 컨트롤러에 보낸다
//
// 구조: Task Manager 노드와 별도인 내부 노드(<이름>_moveit)를 만들어 자기 executor 스레드로 돌린다.
//   MoveGroupInterface 는 shared_ptr 노드가 필요하고, 자기 콜백(현재 상태 구독·액션 응답)이
//   Task Manager 의 작업 스레드와 독립적으로 처리되어야 하기 때문이다.
//   move_group 이 뜨기 전에 Task Manager 가 시작돼도 되도록, 연결은 별도 스레드에서 기다린다.
//
// robot_description / robot_description_semantic / 운동학 설정은 Task Manager 노드의 파라미터로
// 받아(launch 가 aris3_moveit_config 에서 넣어 줌) 내부 노드에 넘긴다.
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "aris3_task_manager/robot_client.hpp"

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}
}  // namespace moveit

namespace aris3_task_manager
{

class MoveItRobotClient : public RobotBackend
{
public:
  struct Options
  {
    std::string group{"arm"};
    std::string tip_link{"link_6"};
    std::string base_frame{"base_link"};
    std::string joint_states_topic{"/e0509_gripper/joint_states"};
    double planning_time_s{5.0};
    double velocity_scaling{0.3};
    double acceleration_scaling{0.3};
    double motion_timeout_s{30.0};
    double arrival_tolerance_mm{3.0};
    double joint_tolerance_deg{1.0};
  };

  // parent: Task Manager 노드 (robot_description 등 파라미터를 여기서 읽는다)
  MoveItRobotClient(rclcpp::Node * parent, Options options);
  ~MoveItRobotClient() override;

  bool servicesReady(std::string * missing = nullptr) const override;
  int robotState() override;
  std::optional<std::array<double, 6>> currentPose() override;
  void moveJoints(const std::array<double, 6> & deg, const std::string & label, const AbortCheck & abort) override;
  void moveLinear(const std::array<double, 6> & pose_mm_deg, const std::string & label,
    const AbortCheck & abort) override;
  void moveFree(const std::array<double, 6> & pose_mm_deg, const std::string & label,
    const AbortCheck & abort) override;
  bool stop() override;
  double reachabilityErrorMm(const std::array<double, 6> & pose_mm_deg) override;

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi() const;
  template<class Setup>
  void planAndExecute(const std::string & label, const AbortCheck & abort, Setup setup);
  void waitArrival(const std::array<double, 6> & pose_mm_deg, const std::string & label);

  Options opt_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::thread connect_thread_;
  mutable std::mutex mutex_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::string connect_error_{"move_group 연결 중"};
  std::atomic<bool> shutting_down_{false};
};

}  // namespace aris3_task_manager
