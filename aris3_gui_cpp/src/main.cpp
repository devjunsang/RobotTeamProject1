// ARIS 실행 파일.
//   ros2 run aris3_gui_cpp aris                      # 시뮬레이션 모드 (Connect = 시뮬레이션 시작)
//   ros2 run aris3_gui_cpp aris --no-voice           # 음성 노드 없이
//   ros2 run aris3_gui_cpp aris --no-assist          # 순수 마찰 파지
// 시험용: --screenshot <png> --quit-after <초>  (화면을 파일로 저장하고 종료)
//         --connect (시작하자마자 Connect), --task blue:A (Task Manager 가 준비되면 작업 큐에 넣고 실행, 여러 번 가능)
// 실물 모드(--mode real)는 다음 단계(M3) - 그 전에는 aris3_gui(PyQt) --mode real 을 쓴다.
#include <QApplication>
#include <QTimer>

#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "aris3_gui_cpp/main_window.hpp"
#include "aris3_gui_cpp/ros_io.hpp"
#include "aris3_gui_cpp/sim_backend.hpp"
#include "aris3_gui_cpp/theme.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
QApplication * g_app = nullptr;
// Ctrl-C / 종료 신호: Qt 이벤트 루프에서 창을 닫게 한다 (종료 확인·정리를 거치도록)
void onSignal(int)
{
  if (g_app) {QMetaObject::invokeMethod(g_app, "closeAllWindows", Qt::QueuedConnection);}
}
}  // namespace

int main(int argc, char ** argv)
{
  // ROS 인자(--ros-args ...)는 rclcpp 가, 나머지는 여기서 읽는다
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  bool voice = true, assist = true, connect_now = false;
  std::vector<std::pair<std::string, std::string>> tasks;   // (색, 구역)
  std::string screenshot;
  double quit_after = 0.0;
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--no-voice") {
      voice = false;
    } else if (args[i] == "--no-assist") {
      assist = false;
    } else if (args[i] == "--connect") {
      connect_now = true;
    } else if (args[i] == "--task" && i + 1 < args.size()) {
      const std::string t = args[++i];
      const auto c = t.find(':');
      tasks.emplace_back(t.substr(0, c), c == std::string::npos ? "A" : t.substr(c + 1));
    } else if (args[i] == "--screenshot" && i + 1 < args.size()) {
      screenshot = args[++i];
    } else if (args[i] == "--quit-after" && i + 1 < args.size()) {
      quit_after = std::stod(args[++i]);
    } else if (args[i] == "--mode" && i + 1 < args.size() && args[i + 1] == "real") {
      std::fprintf(stderr, "ARIS 실물 모드는 다음 단계(M3)입니다. 지금은 ros2 run aris3_gui control_station --mode real\n");
      return 2;
    }
  }

  QApplication app(argc, argv);
  g_app = &app;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  app.setStyleSheet(aris::theme::styleSheet());

  // ROS 노드는 별도 스레드에서 돈다 (Qt 화면은 snapshot 만 읽음)
  auto node = std::make_shared<aris::GuiNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  std::thread spin([&exec]() {exec.spin();});

  aris::SimBackend backend(node, assist, voice);
  aris::MainWindow w(node, &backend);
  w.resize(1680, 980);
  w.show();
  if (connect_now) {QTimer::singleShot(1000, &w, [&w]() {w.connectNow();});}
  // 작업: Task Manager 가 보일 때까지 1 초마다 확인한 뒤 한 번만 큐에 넣고 시작
  QTimer task_timer;
  if (!tasks.empty()) {
    QObject::connect(&task_timer, &QTimer::timeout, &w, [&]() {
        if (!node->statusFlags().task_manager) {return;}
        task_timer.stop();
        for (const auto & [color, zone] : tasks) {w.queueTask(color, zone);}
        w.startQueue();
      });
    task_timer.start(1000);
  }
  if (!screenshot.empty()) {
    QTimer::singleShot(static_cast<int>(std::max(1.0, quit_after - 0.5) * 1000), &w,
      [&w, screenshot]() {w.grab().save(QString::fromStdString(screenshot));});
  }
  if (quit_after > 0) {QTimer::singleShot(static_cast<int>(quit_after * 1000), &app, &QApplication::quit);}
  const int code = app.exec();

  exec.cancel();
  spin.join();
  rclcpp::shutdown();
  return code;
}
