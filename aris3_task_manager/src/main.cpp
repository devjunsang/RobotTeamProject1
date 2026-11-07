// Task Manager 실행 파일. MultiThreadedExecutor 가 필요한 이유는 task_manager_node.hpp 머리말 참고.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "aris3_task_manager/task_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<aris3_task_manager::TaskManagerNode> node;
  try {
    node = std::make_shared<aris3_task_manager::TaskManagerNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("aris3_task_manager"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
