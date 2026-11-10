// 시뮬레이션 모드: Connect = ros2 launch aris3_bringup aris3_sim_full.launch.py (Gazebo -> MoveIt -> Task Manager).
// aris3_gui backends.SimStack 과 같다: 프로세스 그룹으로 띄워 끌 때 자식(Gazebo, MoveIt ...)까지 한 번에 정리.
#pragma once

#include <QObject>
#include <QString>

#include <memory>

namespace aris
{

class GuiNode;

class SimBackend : public QObject
{
  Q_OBJECT

public:
  // node: 로그·상태 확인용, grasp_assist: 물체 붙이기 보조, voice: 음성 노드도 함께
  SimBackend(std::shared_ptr<GuiNode> node, bool grasp_assist, bool voice, QObject * parent = nullptr);
  ~SimBackend() override;

  QString title() const {return QStringLiteral("Gazebo 시뮬레이션");}
  void connectSim();        // 시뮬레이션 시작 (이미 돌고 있으면 그것을 씀)
  void disconnectSim();     // 이 화면이 띄운 시뮬레이션만 끈다
  bool running() const;     // 이 화면이 띄운 launch 가 살아 있는가
  bool connected() const;   // 관절 상태가 들어오는가 (어디서 띄웠든)
  QString describe() const; // 시스템 상태 패널용 설명

private:
  std::shared_ptr<GuiNode> node_;
  bool grasp_assist_, voice_;
  qint64 pid_{0};           // setsid 로 띄운 launch 의 pid (= 프로세스 그룹 id)
};

}  // namespace aris
