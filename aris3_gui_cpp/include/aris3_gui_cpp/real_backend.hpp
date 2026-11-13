// 실물 모드 (M3): aris3_real_bridge 의 /aris3/real/* 를 부른다 (aris3_gui real_client.py + RealBackend 와 같음).
// 로봇 안전 코드(이동 전 검사·도착 확인·정지·복구·그리퍼 확인)와 보정·집기 순서는 브리지 노드 안에 있고,
// ARIS 는 확인창·조작 허용만 맡는다. 브리지가 없으면 함께 띄우고(LaunchProc), 끌 때는 이 화면이 띄운 것만 끈다.
// virtual_mode: 실물 경로를 두산 가상 컨트롤러로 시험 (aris3_real_sim.launch.py 와 함께, 보정 파일도 따로).
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "aris3_gui_cpp/backend.hpp"
#include "aris3_interfaces/action/real_motion.hpp"
#include "aris3_interfaces/action/real_pick_place.hpp"
#include "aris3_interfaces/msg/real_robot_state.hpp"
#include "aris3_interfaces/srv/real_calibration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace aris
{

class GuiNode;

class RealBackend : public Backend
{
public:
  using State = aris3_interfaces::msg::RealRobotState;
  using Motion = aris3_interfaces::action::RealMotion;
  using PickPlace = aris3_interfaces::action::RealPickPlace;
  using Calib = aris3_interfaces::srv::RealCalibration;
  using Px = std::array<double, 2>;

  RealBackend(std::shared_ptr<GuiNode> node, const std::string & ns, const std::string & host, int port,
    double z_min_mm, bool voice, bool gesture, bool virtual_mode);

  QString title() const override;
  bool isReal() const override {return true;}
  bool isVirtual() const override {return virtual_;}

  void connectRobot() override;
  void disconnectRobot() override;
  bool connected() override;
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
  void setArmed(bool on) override;

  // ---- 실물 전용
  std::optional<State> state();                         // 1 초 안에 받은 브리지 상태, 없으면 브리지 없음
  QString robotStateText();                             // 「대기」 「안전 정지」 ...
  bool canRecover();
  void recoverHome(int speed_pct);                      // 안전 정지 해제/서보 켜기 -> 자동 모드 -> 홈
  void stretch(int speed_pct);                          // 가동 자세: 모든 관절 0° (팔을 위로 일자로 펴기)
  void resumePick();                                    // 정지로 끊긴 집기 순서를 멈춘 단계부터 이어서
  bool bridgeReady();                                   // 브리지의 접속 서비스가 보이는가 (창을 연 직후 자동 접속용)
  // 손으로 움직이기: (수동 모드인가 - 모르면 nullopt, 바꾸는 중인가, 모드 이름)
  std::optional<bool> handGuide(bool & switching, QString & name);
  void setHandGuide(bool on);
  void calibration(uint8_t command, double u = 0, double v = 0, const std::string & label = "");
  // 집기·놓기: 대상·놓을 자리는 영상 픽셀 (브리지가 움직이기 직전에 검출로 다시 고침)
  void pickPlace(uint8_t kind, const std::string & what, const std::string & name,
    const std::optional<Px> & target, const std::optional<Px> & place);
  bool motionPending();                                 // 단순 이동(조그·홈) 결과를 기다리는 중 (색 탐색용)
  bool pickRunning();                                   // 집기 순서 진행 중 (보내고 결과 기다리는 중 포함)
  std::optional<bool> holding();                        // 마지막 닫기: 잡음 / 빈손 / 모름(nullopt)
  // 보정 (상태 토픽의 행렬로 계산)
  bool calibReady();
  std::optional<Px> toRobot(double u, double v);        // 픽셀 -> 로봇 (x, y) [mm]
  std::optional<double> graspZ();
  QString calibSummary();
  std::optional<Px> calibPendingPx();                   // 보정 ① 에서 기억한 픽셀

private:
  void callTrigger(const std::string & name, const std::string & what);
  template<typename A>
  void sendAction(typename rclcpp_action::Client<A>::SharedPtr client, const typename A::Goal & goal,
    const std::string & what, bool & pending);

  std::shared_ptr<GuiNode> node_;
  bool virtual_;
  std::mutex mtx_;
  std::optional<State> state_;
  std::chrono::steady_clock::time_point state_time_;
  bool motion_pending_{false}, pick_pending_{false};

  rclcpp::Subscription<State>::SharedPtr sub_state_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trig_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr srv_armed_, srv_hand_;
  rclcpp::Client<Calib>::SharedPtr srv_calib_;
  rclcpp_action::Client<Motion>::SharedPtr motion_client_;
  rclcpp_action::Client<PickPlace>::SharedPtr pick_client_;

  // world_: 시뮬레이션(virtual)일 때만 띄우는 실물 비슷한 월드 (aris3_real_sim.launch.py - Gazebo·카메라·그리퍼)
  LaunchProc world_, bridge_, voice_, gesture_;
};

}  // namespace aris
