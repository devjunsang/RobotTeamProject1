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

class RobotClient
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

  // 기다리는 동안 계속 불러서 "중단해야 하는가" 를 묻는다. NONE 이면 계속, 아니면 그 이유로 중단.
  using AbortCheck = std::function<ErrorCode()>;

  RobotClient(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr group, Options options);

  bool servicesReady(std::string * missing = nullptr) const;
  int robotState();  // 실패 시 -1
  std::optional<std::array<double, 6>> currentPose();  // 플랜지 [mm, deg], 베이스 기준

  // 실패하면 TaskError 를 던진다.
  void moveJoints(const std::array<double, 6> & deg, const std::string & label, const AbortCheck & abort);
  void moveLinear(const std::array<double, 6> & pose_mm_deg, const std::string & label, const AbortCheck & abort);
  bool stop();

  // IK 로 관절각을 구하고 FK 로 되돌려서 원래 위치와의 차이(mm)를 돌려준다. 서비스 실패 시 음수.
  // 두산 ikin 서비스는 도달할 수 없는 점에도 success=true 를 주므로 왕복 오차로 판단한다.
  double reachabilityErrorMm(const std::array<double, 6> & pose_mm_deg);

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
