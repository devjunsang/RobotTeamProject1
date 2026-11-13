// 시뮬레이션 모드 백엔드 구현 (sim_backend.hpp 설명 참고).
#include "aris3_gui_cpp/sim_backend.hpp"

#include <algorithm>
#include <cmath>

#include "aris3_gui_cpp/ros_io.hpp"

namespace aris
{

SimBackend::SimBackend(std::shared_ptr<GuiNode> node, bool grasp_assist, bool voice)
: node_(std::move(node)),
  sim_(QStringLiteral("시뮬레이션"),
    {"aris3_bringup", "aris3_sim_full.launch.py", QString("grasp_assist:=%1").arg(grasp_assist ? "true" : "false"),
      QString("voice:=%1").arg(voice ? "true" : "false")},
    QString(), QStringLiteral("/tmp/aris_sim.log"))
{
}

bool SimBackend::connected()
{
  return node_->statusFlags().joint_states;
}

bool SimBackend::ownsProcess()
{
  return sim_.running();
}

QString SimBackend::describe()
{
  const auto f = node_->statusFlags();
  if (sim_.running() && !f.task_manager) {return QStringLiteral("시작 중... (Gazebo -> MoveIt -> Task Manager)");}
  if (sim_.running()) {return QStringLiteral("이 화면이 실행한 시뮬레이션");}
  return f.joint_states ? QStringLiteral("외부에서 실행한 시뮬레이션") : QStringLiteral("실행 안 됨");
}

void SimBackend::connectRobot()
{
  if (connected()) {
    node_->log("INFO", "시뮬레이션이 이미 실행 중 (다른 곳에서 실행한 것 포함)");
    return;
  }
  const QString msg = sim_.start();
  if (!msg.isEmpty()) {node_->log("INFO", msg.toStdString() + " - Gazebo -> MoveIt -> Task Manager");}
}

void SimBackend::disconnectRobot()
{
  if (!sim_.running()) {
    node_->log("WARN", "이 화면이 실행한 시뮬레이션이 없음 (다른 곳에서 실행했다면 그쪽에서 끄세요)");
    return;
  }
  node_->log("INFO", "시뮬레이션 종료 중...");
  sim_.stop(20.0);          // launch 가 자식(Gazebo, MoveIt ...)을 차례로 끈다
  node_->log("INFO", "시뮬레이션 종료");
}

void SimBackend::shutdown()
{
  if (sim_.running()) {sim_.stop(20.0);}
}

std::optional<std::vector<double>> SimBackend::jointsDeg()
{
  const auto j = node_->snapshot().joints;
  std::vector<double> out;
  for (int i = 1; i <= 6; ++i) {
    auto it = j.find("joint_" + std::to_string(i));
    if (it == j.end()) {return std::nullopt;}
    out.push_back(it->second * 180.0 / M_PI);
  }
  return out;
}

bool SimBackend::tcp(double xyz_m[3], double rpy_rad[3])
{
  Vec3 t{}, r{};
  if (!node_->tcpPose(t, r)) {return false;}
  for (int i = 0; i < 3; ++i) {
    xyz_m[i] = t[i];
    rpy_rad[i] = r[i];
  }
  return true;
}

std::optional<double> SimBackend::gripperPosition()
{
  // 0 = 열림 ~ 1 = 다 닫힘 (gripper_rh_r1 / 1.1 rad)
  const auto j = node_->snapshot().joints;
  auto it = j.find("gripper_rh_r1");
  if (it == j.end()) {return std::nullopt;}
  return std::clamp(it->second / 1.1, 0.0, 1.0);
}

GripperStatus SimBackend::gripperStatus()
{
  const bool ok = node_->statusFlags().gripper;
  return {ok, ok ? QStringLiteral("준비") : QStringLiteral("서비스 없음"), false};
}

QString SimBackend::whyNotReady()
{
  const auto f = node_->statusFlags();
  if (!f.joint_states) {return QStringLiteral("관절 상태가 들어오지 않음 (Connect 로 시뮬레이션 시작)");}
  if (!f.moveit) {return QStringLiteral("MoveIt 준비 중");}
  if (node_->taskRunning()) {return QStringLiteral("작업 실행 중");}
  if (node_->manualBusy()) {return QStringLiteral("이전 수동 이동 진행 중");}
  return QString();
}

void SimBackend::home(int speed_pct) {node_->home(speed_pct / 100.0);}
void SimBackend::jogJoint(int index, double deg, int speed_pct) {node_->jogJoint(index, deg, speed_pct / 100.0);}
void SimBackend::jogTcp(int axis, double mm, int speed_pct) {node_->jogTcp(axis, mm, speed_pct / 100.0);}
void SimBackend::gripper(bool close) {node_->gripper(close ? 1 : 0);}
void SimBackend::stop() {node_->stopAll();}

}  // namespace aris
