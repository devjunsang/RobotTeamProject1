// ARIS 모드별 로봇 연결 (aris3_gui backends.py 와 같은 생각).
// 화면(MainWindow)은 두 백엔드를 같은 방법으로 부른다:
//   SimBackend  : 시뮬레이션 - Connect = aris3_sim_full.launch.py, 수동 조작 = MoveIt, 작업 = Task Manager
//   RealBackend : 실물(또는 가상 컨트롤러) - aris3_real_bridge 의 /aris3/real/* (로봇 안전 코드는 브리지 안 RobotLink)
// LaunchProc   : ros2 launch 를 새 프로세스 그룹(setsid)으로 띄우고, 끌 때 그 그룹째 정리하는 도우미.
#pragma once

#include <QString>
#include <QStringList>

#include <optional>
#include <string>
#include <vector>

namespace aris
{

// 그리퍼 상태 한 줄 (시스템 상태 패널): 정상인가, 글자, 대기(앰버)인가
struct GripperStatus
{
  bool ok{false};
  QString text;
  bool warn{false};
};

class Backend
{
public:
  virtual ~Backend() = default;

  virtual QString title() const = 0;         // 창 제목·모드 표시
  virtual bool isReal() const = 0;           // 실물 모드(가상 컨트롤러 포함)인가
  virtual bool isVirtual() const {return false;}   // 실물 경로를 가상 컨트롤러로 시험 중인가

  // ---- 연결
  virtual void connectRobot() = 0;
  virtual void disconnectRobot() = 0;
  virtual bool connected() = 0;
  virtual QString describe() = 0;            // 시스템 상태 패널의 연결 설명
  virtual bool ownsProcess() = 0;            // 이 화면이 띄운 프로세스가 있는가 (닫을 때 물어봄)
  virtual void shutdown() = 0;               // 창을 닫을 때 정리

  // ---- 상태 (없으면 nullopt)
  virtual std::optional<std::vector<double>> jointsDeg() = 0;          // 관절 6개 [deg]
  virtual bool tcp(double xyz_m[3], double rpy_rad[3]) = 0;            // 손끝 [m] / [rad] (world 또는 베이스)
  virtual std::optional<double> gripperPosition() = 0;                 // 0 열림 ~ 1 닫힘
  virtual GripperStatus gripperStatus() = 0;
  virtual QString whyNotReady() = 0;         // 지금 움직일 수 없는 이유 (조작 허용 제외), 없으면 빈 문자열

  // ---- 명령 (속도는 0~100 %)
  virtual void home(int speed_pct) = 0;
  virtual void jogJoint(int index, double deg, int speed_pct) = 0;
  virtual void jogTcp(int axis, double mm, int speed_pct) = 0;
  virtual void gripper(bool close) = 0;
  virtual void stop() = 0;                   // 정지 - 언제나 보낸다
  virtual void setArmed(bool) {}             // 실물: 브리지 쪽 조작 허용도 같게
};

// ros2 launch 를 setsid 로 (새 프로세스 그룹) 띄우고 그 그룹째 끄는 도우미.
// pattern: 이미 다른 곳에서 같은 노드가 돌고 있는지 pgrep -f 로 찾을 정규식 (있으면 새로 띄우지 않음)
class LaunchProc
{
public:
  LaunchProc(QString name, QStringList launch_args, QString pattern, QString log_path)
  : name_(std::move(name)), args_(std::move(launch_args)), pattern_(std::move(pattern)), log_(std::move(log_path)) {}

  bool externalRunning() const;               // 이 화면이 띄우지 않은 같은 노드가 돌고 있는가
  bool running() const;                       // 이 화면이 띄운 것이 살아 있는가
  // 띄운다. 이미 돌고 있으면 띄우지 않고 그 사실을 문장으로 돌려준다 (로그용)
  QString start();
  void stop(double timeout_s = 5.0);          // 이 화면이 띄운 것만 끈다 (SIGINT -> 안 꺼지면 SIGKILL)
  const QString & name() const {return name_;}

private:
  QString name_;
  QStringList args_;
  QString pattern_, log_;
  qint64 pid_{0};
};

}  // namespace aris
