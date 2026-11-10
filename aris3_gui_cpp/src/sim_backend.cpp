// 시뮬레이션 모드 백엔드 구현 (sim_backend.hpp 설명 참고).
#include "aris3_gui_cpp/sim_backend.hpp"

#include <QProcess>
#include <QProcessEnvironment>
#include <QThread>

#include <signal.h>
#include <sys/wait.h>

#include <chrono>
#include <thread>

#include "aris3_gui_cpp/ros_io.hpp"

namespace aris
{

namespace
{
const char * kSimLog = "/tmp/aris_sim.log";

// pid 프로세스가 아직 살아 있는가 (좀비 제외: 우리가 띄운 자식이면 거둬 들인다)
bool alive(qint64 pid)
{
  if (pid <= 0) {return false;}
  int status = 0;
  if (::waitpid(static_cast<pid_t>(pid), &status, WNOHANG) == static_cast<pid_t>(pid)) {return false;}
  return ::kill(static_cast<pid_t>(pid), 0) == 0;
}
}  // namespace

SimBackend::SimBackend(std::shared_ptr<GuiNode> node, bool grasp_assist, bool voice, QObject * parent)
: QObject(parent), node_(std::move(node)), grasp_assist_(grasp_assist), voice_(voice)
{
}

SimBackend::~SimBackend()
{
  // 종료 확인(묻기)은 창이 한다. 여기서는 남은 것이 있으면 정리만.
}

bool SimBackend::running() const
{
  return alive(pid_);
}

bool SimBackend::connected() const
{
  return node_->statusFlags().joint_states;
}

QString SimBackend::describe() const
{
  const auto f = node_->statusFlags();
  if (running() && !f.task_manager) {return QStringLiteral("시작 중... (Gazebo -> MoveIt -> Task Manager)");}
  if (running()) {return QStringLiteral("이 화면이 실행한 시뮬레이션");}
  return f.joint_states ? QStringLiteral("외부에서 실행한 시뮬레이션") : QStringLiteral("실행 안 됨");
}

void SimBackend::connectSim()
{
  if (connected()) {
    node_->log("INFO", "시뮬레이션이 이미 실행 중 (다른 곳에서 실행한 것 포함)");
    return;
  }
  if (running()) {return;}
  // setsid: 새 프로세스 그룹 -> 끌 때 kill(-pgid) 로 launch 와 자식들을 한 번에
  const QStringList args = {
    "ros2", "launch", "aris3_bringup", "aris3_sim_full.launch.py",
    QString("grasp_assist:=%1").arg(grasp_assist_ ? "true" : "false"),
    QString("voice:=%1").arg(voice_ ? "true" : "false")};
  QProcess p;
  // 이 화면의 Qt 표시 설정(예: 시험용 QT_QPA_PLATFORM=offscreen)을 시뮬레이션에 물려주지 않는다.
  // 물려주면 Gazebo 화면·RViz 가 화면 없이 떠서 RViz 가 죽고 Gazebo 진행 요청이 실패했다 (2026-11-10 시험에서 발견)
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.remove("QT_QPA_PLATFORM");
  p.setProcessEnvironment(env);
  p.setProgram("setsid");
  p.setArguments(args);
  p.setStandardOutputFile(kSimLog);
  p.setStandardErrorFile(kSimLog, QIODevice::Append);
  qint64 pid = 0;
  if (!p.startDetached(&pid)) {
    node_->log("ERROR", "시뮬레이션을 시작하지 못함 (ros2 명령·환경 확인)");
    return;
  }
  pid_ = pid;
  node_->log("INFO", std::string("시뮬레이션 시작: Gazebo -> MoveIt -> Task Manager (기록 ") + kSimLog + ")");
}

void SimBackend::disconnectSim()
{
  if (!running()) {
    node_->log("WARN", "이 화면이 실행한 시뮬레이션이 없음 (다른 곳에서 실행했다면 그쪽에서 끄세요)");
    return;
  }
  node_->log("INFO", "시뮬레이션 종료 중...");
  // launch 에 SIGINT -> 자식(Gazebo, MoveIt ...)을 차례로 끈다. 20 초 안에 안 꺼지면 SIGKILL
  ::kill(-static_cast<pid_t>(pid_), SIGINT);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (alive(pid_) && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  if (alive(pid_)) {::kill(-static_cast<pid_t>(pid_), SIGKILL);}
  pid_ = 0;
  node_->log("INFO", "시뮬레이션 종료");
}

}  // namespace aris
