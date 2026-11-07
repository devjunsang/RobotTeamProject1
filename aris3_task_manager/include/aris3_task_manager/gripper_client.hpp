// ARIS3 그리퍼 계약(/aris3/gripper/command)을 부르는 얇은 클라이언트.
// 실제 그리퍼가 시뮬레이션인지 진짜인지 모른다 — 그게 계약(인터페이스)의 목적이다.
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "aris3_interfaces/srv/gripper_command.hpp"

namespace aris3_task_manager
{

struct GripperResult
{
  bool ok{false};        // 서비스가 응답했고 success=true
  bool holding{false};
  std::string object_id;
  std::string message;
};

class GripperClient
{
public:
  GripperClient(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr group,
    const std::string & service, double timeout_s);
  bool ready() const {return client_->service_is_ready();}
  std::string serviceName() const {return client_->get_service_name();}
  GripperResult open();
  GripperResult close(double force = 0.5);
  GripperResult status();   // 움직이지 않고 잡고 있는지만 확인

private:
  GripperResult send(uint8_t command, double force);
  rclcpp::Client<aris3_interfaces::srv::GripperCommand>::SharedPtr client_;
  double timeout_s_;
};

}  // namespace aris3_task_manager
