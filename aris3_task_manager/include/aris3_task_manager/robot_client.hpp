// 두산 로봇(실제 또는 DRCF 에뮬레이터)에 이동 명령을 보내는 어댑터.
//
// 사용 서비스 (<ns> = /dsr01):
//   <ns>/motion/move_joint, move_line   비동기(sync_type=1)로 보내고 끝날 때까지 직접 감시
//   <ns>/motion/check_motion            0 = 정지, 그 외 = 움직이는 중
//   <ns>/motion/move_stop               정지 (취소·시간 초과·실패 시)
//   <ns>/motion/ikin, fkin              도달 가능성 검사 (IK -> FK 왕복)
//   <ns>/aux_control/get_current_posx   도착 확인
//   <ns>/system/get_robot_state         준비 상태 (1 = STANDBY)
//
// 왜 동기(sync_type=0)가 아닌 비동기인가: 동기 호출은 이동이 끝날 때까지 돌아오지 않아서
// 그 사이 취소·시간 초과를 처리할 수 없다. 비동기로 보내고 0.1 초마다 상태를 보면서 기다린다.
//
// 왜 도착을 따로 확인하는가: 에뮬레이터에서 도달할 수 없는 위치로 비동기 move_line 을 보내도
// success=true 가 돌아오고 아무 일도 일어나지 않았다(2026-11-07 확인). "명령이 접수됨" 과
// "목표에 도착함" 은 다르다.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/srv/check_motion.hpp"
#include "dsr_msgs2/srv/fkin.hpp"
#include "dsr_msgs2/srv/get_current_posx.hpp"
#include "dsr_msgs2/srv/get_robot_state.hpp"
#include "dsr_msgs2/srv/ikin.hpp"
#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_line.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"

#include "aris3_task_manager/types.hpp"

namespace aris3_task_manager
{

// 로봇 이동 백엔드의 공통 인터페이스. Task Manager 상태 머신은 이것만 안다.
//   RobotClient       : 두산 서비스(DRCF 에뮬레이터·실제 컨트롤러)  robot_backend: dsr
//   MoveItRobotClient : MoveIt 2 move_group -> ros2_control           robot_backend: moveit
// 자세 단위: 플랜지(link_6) [x, y, z mm, ZYZ 오일러 deg], 로봇 베이스 기준.
class RobotBackend
{
public:
  // 기다리는 동안 계속 불러서 "중단해야 하는가" 를 묻는다. NONE 이면 계속, 아니면 그 이유로 중단.
  using AbortCheck = std::function<ErrorCode()>;

  virtual ~RobotBackend() = default;
  virtual bool servicesReady(std::string * missing = nullptr) const = 0;
  virtual int robotState() = 0;   // 1 = 대기(STANDBY, 명령 받을 수 있음), 실패 시 -1
  virtual std::optional<std::array<double, 6>> currentPose() = 0;
  // 실패하면 TaskError 를 던진다.
  virtual void moveJoints(const std::array<double, 6> & deg, const std::string & label, const AbortCheck & abort) = 0;
  virtual void moveLinear(const std::array<double, 6> & pose_mm_deg, const std::string & label,
    const AbortCheck & abort) = 0;
  // 경로 모양은 상관없이 목표 자세로 (물체를 옮기는 구간 등). 기본은 직선 이동과 같다.
  // MoveIt 백엔드는 충돌을 피하는 경로(OMPL)로 간다: 직선이 로봇 몸통을 지나는 경우가 있기 때문.
  virtual void moveFree(const std::array<double, 6> & pose_mm_deg, const std::string & label,
    const AbortCheck & abort) {moveLinear(pose_mm_deg, label, abort);}
  virtual bool stop() = 0;
  // 도달 가능성: 목표 자세의 IK 해 오차(mm). 도달 불가면 큰 값, 백엔드가 응답하지 않으면 음수.
  virtual double reachabilityErrorMm(const std::array<double, 6> & pose_mm_deg) = 0;
};

class RobotClient : public RobotBackend
{
public:
  struct Options
  {
    std::string ns{"/dsr01"};
    double linear_vel_mm_s{100.0};
    double linear_acc_mm_s2{200.0};
    double joint_vel_deg_s{40.0};
    double joint_acc_deg_s2{40.0};
    double motion_timeout_s{30.0};
    double arrival_tolerance_mm{3.0};
    double joint_tolerance_deg{0.5};
    double service_timeout_s{3.0};
    double poll_period_s{0.1};
    double reach_tolerance_mm{1.0};
  };

  RobotClient(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr group, Options options);

  bool servicesReady(std::string * missing = nullptr) const override;
  int robotState() override;  // 실패 시 -1
  std::optional<std::array<double, 6>> currentPose() override;  // 플랜지 [mm, deg], 베이스 기준

  // 실패하면 TaskError 를 던진다.
  void moveJoints(const std::array<double, 6> & deg, const std::string & label, const AbortCheck & abort) override;
  void moveLinear(const std::array<double, 6> & pose_mm_deg, const std::string & label,
    const AbortCheck & abort) override;
  bool stop() override;

  // IK 로 관절각을 구하고 FK 로 되돌려서 원래 위치와의 차이(mm)를 돌려준다. 서비스 실패 시 음수.
  // 두산 ikin 서비스는 도달할 수 없는 점에도 success=true 를 주므로 왕복 오차로 판단한다.
  double reachabilityErrorMm(const std::array<double, 6> & pose_mm_deg) override;

  const Options & options() const {return opt_;}

private:
  void waitForMotion(const std::string & label, const AbortCheck & abort);

  rclcpp::Node * node_;
  Options opt_;
  rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr move_joint_;
  rclcpp::Client<dsr_msgs2::srv::MoveLine>::SharedPtr move_line_;
  rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedPtr move_stop_;
  rclcpp::Client<dsr_msgs2::srv::CheckMotion>::SharedPtr check_motion_;
  rclcpp::Client<dsr_msgs2::srv::Ikin>::SharedPtr ikin_;
  rclcpp::Client<dsr_msgs2::srv::Fkin>::SharedPtr fkin_;
  rclcpp::Client<dsr_msgs2::srv::GetCurrentPosx>::SharedPtr get_posx_;
  rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedPtr get_state_;
};

}  // namespace aris3_task_manager
