// 명령줄에서 작업을 보내는 액션 클라이언트 (음성 모듈이 생기기 전까지 사람이 직접 요청하는 입구).
//
//   ros2 run aris3_task_manager send_task red A
//   ros2 run aris3_task_manager send_task blue C --dry-run
//   ros2 run aris3_task_manager send_task green B --shape cube
//   ros2 run aris3_task_manager send_task red A --action /aris3/test_execute_task   (시험용 서버)
//   ros2 run aris3_task_manager send_task --id blue_cube --at 0.30,-0.20      (물체 이름 + 좌표, 손짓용)
//
// 진행 상황(feedback)을 한 줄씩 출력하고, Ctrl-C 를 누르면 작업 "취소"를 요청한 뒤 결과를 기다린다.
// 종료 코드: 0 = 성공, 1 = 실패/취소, 2 = 서버 없음·거절.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "aris3_interfaces/action/execute_task.hpp"

using ExecuteTask = aris3_interfaces::action::ExecuteTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;
using namespace std::chrono_literals;

static std::atomic<bool> g_interrupted{false};

int main(int argc, char ** argv)
{
  // 기본 Ctrl-C 처리(=즉시 종료)를 끄고 직접 처리한다: 종료 전에 작업 취소를 보내야 하므로.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) {g_interrupted = true;});

  auto args = rclcpp::remove_ros_arguments(argc, argv);
  ExecuteTask::Goal goal;
  goal.task_type = "pick_place";
  std::vector<std::string> pos;
  std::string action_name = "/aris3/execute_task";
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--dry-run") {
      goal.dry_run = true;
    } else if (args[i] == "--action" && i + 1 < args.size()) {
      action_name = args[++i];
    } else if (args[i] == "--shape" && i + 1 < args.size()) {
      goal.target_shape = args[++i];
    } else if (args[i] == "--id" && i + 1 < args.size()) {
      goal.target_id = args[++i];
    } else if (args[i] == "--at" && i + 1 < args.size()) {
      double x = 0.0, y = 0.0;
      if (std::sscanf(args[++i].c_str(), "%lf,%lf", &x, &y) != 2) {
        std::fprintf(stderr, "--at 은 X,Y 형식 (예: --at 0.30,-0.20)\n");
        rclcpp::shutdown();
        return 2;
      }
      goal.use_place_position = true;
      goal.place_position.x = x;
      goal.place_position.y = y;
    } else {
      pos.push_back(args[i]);
    }
  }
  // 위치 인자: 색이 필요하면(--id 없음) 첫 번째, 구역이 필요하면(--at 없음) 그다음
  const size_t need = (goal.target_id.empty() ? 1 : 0) + (goal.use_place_position ? 0 : 1);
  if (pos.size() != need) {
    std::fprintf(stderr,
      "사용법: send_task <색> <구역: A|B|C> [--dry-run] [--shape cube] [--action 이름]\n"
      "        send_task --id <물체 이름> --at X,Y      (물체 이름으로 고르고 좌표에 놓기, world [m])\n"
      "        send_task <색> --at X,Y  |  send_task --id <물체 이름> <구역>\n");
    rclcpp::shutdown();
    return 2;
  }
  size_t k = 0;
  if (goal.target_id.empty()) {goal.target_color = pos[k++];}
  if (!goal.use_place_position) {goal.destination = pos[k++];}

  auto node = rclcpp::Node::make_shared("aris3_send_task");
  auto client = rclcpp_action::create_client<ExecuteTask>(node, action_name);
  if (!client->wait_for_action_server(5s)) {
    std::fprintf(stderr, "액션 서버 %s 가 없음 (task_manager 실행 확인)\n", action_name.c_str());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions opts;
  opts.feedback_callback = [](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteTask::Feedback> fb) {
      std::printf("[%3.0f%%] %-8s %s\n", fb->progress * 100.0, fb->stage.c_str(), fb->detail.c_str());
      std::fflush(stdout);
    };
  char where[64];
  if (goal.use_place_position) {
    std::snprintf(where, sizeof(where), "(%.3f, %.3f)", goal.place_position.x, goal.place_position.y);
  } else {
    std::snprintf(where, sizeof(where), "%s", goal.destination.c_str());
  }
  const std::string what = goal.target_id.empty() ?
    goal.target_color + (goal.target_shape.empty() ? "" : " " + goal.target_shape) : goal.target_id;
  std::printf("요청: %s -> %s%s\n", what.c_str(), where, goal.dry_run ? " (dry_run: 로봇은 움직이지 않음)" : "");
  auto goal_future = client->async_send_goal(goal, opts);
  if (rclcpp::spin_until_future_complete(node, goal_future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
    std::fprintf(stderr, "요청 전송 실패\n");
    rclcpp::shutdown();
    return 2;
  }
  auto handle = goal_future.get();
  if (!handle) {
    std::fprintf(stderr, "거절됨 (다른 작업이 실행 중일 수 있음)\n");
    rclcpp::shutdown();
    return 2;
  }

  auto result_future = client->async_get_result(handle);
  bool cancel_sent = false;
  while (rclcpp::ok()) {
    if (g_interrupted && !cancel_sent) {
      std::printf("\nCtrl-C: 작업 취소를 요청합니다...\n");
      client->async_cancel_goal(handle);
      cancel_sent = true;
    }
    if (rclcpp::spin_until_future_complete(node, result_future, 100ms) == rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }
  }
  const auto wrapped = result_future.get();
  const auto & r = *wrapped.result;
  const char * how = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? "성공" :
    wrapped.code == rclcpp_action::ResultCode::CANCELED ? "취소됨" : "실패";
  std::printf("\n결과: %s (error_code=%u%s%s)\n  %s\n", how, r.error_code,
    r.failed_stage.empty() ? "" : ", 단계 ", r.failed_stage.c_str(), r.message.c_str());
  std::printf("  집기 전 위치 (%.3f, %.3f, %.3f)  놓은 뒤 위치 (%.3f, %.3f, %.3f)  구역 오차 %.1f mm  소요 %.1f s\n",
    r.picked_from.x, r.picked_from.y, r.picked_from.z, r.placed_at.x, r.placed_at.y, r.placed_at.z,
    r.place_error_mm, r.duration_s);
  rclcpp::shutdown();
  return r.success ? 0 : 1;
}
