// 작업 스레드에서 ROS 2 서비스를 "보내고 기다리는" 도우미.
//
// 중요한 전제: 이 함수는 executor 가 돌리는 콜백 안이 아니라 별도 작업 스레드(execute())에서 부른다.
// 응답은 MultiThreadedExecutor 의 다른 스레드가 받아서 future 를 채워 준다.
// 같은 스레드에서 기다리면 응답을 처리할 스레드가 없어 영원히 멈춘다(deadlock).
#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace aris3_task_manager
{

template<class SrvT>
typename SrvT::Response::SharedPtr callService(
  const typename rclcpp::Client<SrvT>::SharedPtr & client,
  const typename SrvT::Request::SharedPtr & request,
  double timeout_s)
{
  if (!client->service_is_ready()) {
    return nullptr;
  }
  auto pending = client->async_send_request(request);
  const auto timeout = std::chrono::duration<double>(timeout_s);
  if (pending.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(pending);  // 늦게 온 응답이 쌓이지 않게 정리
    return nullptr;
  }
  return pending.get();
}

}  // namespace aris3_task_manager
