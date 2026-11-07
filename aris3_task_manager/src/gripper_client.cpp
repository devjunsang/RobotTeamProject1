#include "aris3_task_manager/gripper_client.hpp"

#include "aris3_task_manager/service_call.hpp"

namespace aris3_task_manager
{

using aris3_interfaces::srv::GripperCommand;

GripperClient::GripperClient(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr group,
  const std::string & service, double timeout_s)
: timeout_s_(timeout_s)
{
  client_ = node->create_client<GripperCommand>(service, rclcpp::ServicesQoS(), group);
}

GripperResult GripperClient::open() {return send(GripperCommand::Request::OPEN, 0.0);}
GripperResult GripperClient::close(double force) {return send(GripperCommand::Request::CLOSE, force);}
GripperResult GripperClient::status() {return send(GripperCommand::Request::STATUS, 0.0);}

GripperResult GripperClient::send(uint8_t command, double force)
{
  auto req = std::make_shared<GripperCommand::Request>();
  req->command = command;
  req->force = static_cast<float>(force);
  auto res = callService<GripperCommand>(client_, req, timeout_s_);
  GripperResult out;
  if (!res) {
    out.message = std::string(client_->get_service_name()) + " 응답 없음";
    return out;
  }
  out.ok = res->success;
  out.holding = res->holding;
  out.object_id = res->object_id;
  out.message = res->message;
  return out;
}

}  // namespace aris3_task_manager
