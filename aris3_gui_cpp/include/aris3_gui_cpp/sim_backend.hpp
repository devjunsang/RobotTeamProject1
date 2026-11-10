// 시뮬레이션 모드: Connect = ros2 launch aris3_bringup aris3_sim_full.launch.py (Gazebo -> MoveIt -> Task Manager).
// 수동 조작은 GuiNode 의 MoveIt 경로, 상태는 시뮬레이션 관절 상태·TF 에서 읽는다 (aris3_gui SimBackend 와 같음).
#pragma once

#include <memory>

#include "aris3_gui_cpp/backend.hpp"

namespace aris
{

class GuiNode;

class SimBackend : public Backend
{
public:
  // node: ROS 쪽, grasp_assist: 물체 붙이기 보조, voice: 음성 노드도 함께 (aris3_sim_full 이 띄움)
  SimBackend(std::shared_ptr<GuiNode> node, bool grasp_assist, bool voice);

  QString title() const override {return QStringLiteral("Gazebo 시뮬레이션");}
  bool isReal() const override {return false;}

  void connectRobot() override;           // 시뮬레이션 시작 (이미 돌고 있으면 그것을 씀)
  void disconnectRobot() override;        // 이 화면이 띄운 시뮬레이션만 끈다
  bool connected() override;              // 관절 상태가 들어오는가 (어디서 띄웠든)
  QString describe() override;
  bool ownsProcess() override;
  void shutdown() override;

  std::optional<std::vector<double>> jointsDeg() override;
  bool tcp(double xyz_m[3], double rpy_rad[3]) override;
  std::optional<double> gripperPosition() override;
  GripperStatus gripperStatus() override;
  QString whyNotReady() override;

  void home(int speed_pct) override;
  void jogJoint(int index, double deg, int speed_pct) override;
  void jogTcp(int axis, double mm, int speed_pct) override;
  void gripper(bool close) override;
  void stop() override;

private:
  std::shared_ptr<GuiNode> node_;
  LaunchProc sim_;
};

}  // namespace aris
