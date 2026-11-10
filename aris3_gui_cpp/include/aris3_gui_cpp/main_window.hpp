// ARIS 메인 창 (M3: 시뮬레이션·실물 모드). 배치는 목업과 aris3_gui 와 같다:
//   ┌ 상단: 제목 · [가운데 ARIS 로고] · Connect · Disconnect · Home · Start Task · Stop · 정지 ─────┐
//   │ 시스템 상태  │ 카메라 Top View (클릭: 이거/저기) │ 작업 공간 2D                 │
//   │ 로봇 상태    │                                  │ 작업 큐 / 화면 선택 / 단계   │
//   │ 그리퍼       ├──────────────────────────────────┼──────────────┬───────────────┤
//   │ 카메라 정보  │ 로그                             │ 음성 명령    │ (실물 집기) │ 수동 조작 │
// 실물 모드 (aris3_real_bridge): 실물 집기·카메라 보정 패널, 손짓 포인터·이벤트, 복구 후 홈, 손으로 움직이기,
//   음성은 매번 확인창 (aris3_gui 와 같음). 자동 작업(Task Manager 두산 백엔드)은 검증 전이라 막아 둔다.
// 안전 규칙 (aris3_gui 와 같음):
//   - 수동 조작·그리퍼·홈은 「조작 허용」 을 체크해야 동작하고, 2분 동안 명령이 없으면 다시 잠긴다.
//   - 정지는 언제나 동작 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
//   - 작업 실행 중에는 수동 조작을 받지 않는다.
//   - 음성 명령은 해석 결과를 보여 주고, 「자동 실행」 이 아니면 사용자가 「실행」 을 눌러야 한다.
#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QTableWidget>
#include <QTimer>

#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aris3_gui_cpp/backend.hpp"
#include "aris3_gui_cpp/real_backend.hpp"
#include "aris3_gui_cpp/ros_io.hpp"
#include "aris3_gui_cpp/widgets.hpp"

namespace aris
{

// 작업 큐 한 줄
struct QueuedTask
{
  std::string color, zone;
  bool dry{false};
  std::string target_id;                              // 화면에서 고른 물체 (있으면 색 대신)
  std::optional<std::array<double, 2>> place;         // 화면에서 고른 자리 (있으면 구역 대신)
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(std::shared_ptr<GuiNode> node, Backend * backend, QWidget * parent = nullptr);

  // 시험용 (명령줄 --connect / --task): 버튼을 누른 것과 같은 동작
  void connectNow() {onConnect();}
  void queueTask(const std::string & color, const std::string & zone, bool dry = false);
  void startQueue() {onStartQueue();}
  // 시험용 (--selftest): 조작 허용 -> 홈 -> TCP +Z -> J1 +10° -> 화면 선택(파랑 큐브 -> 빈 곳) 작업.
  // 단계마다 로그로 완료를 확인하고 결과를 표준 출력에 「SELFTEST PASS/FAIL ...」 로 남긴 뒤 done(성공 여부) 을 부른다.
  void runSelftest(std::function<void(bool)> done);
  // 시험용 (--selftest, 실물 모드): 접속 -> 조작 허용 -> 홈 -> 보정 불러오기 -> 빨강 큐브 픽셀 클릭 -> 빈 자리 클릭
  // -> 집어서 옮기기 (확인창은 자동 「예」) -> 이동 중 정지. 결과는 표준 출력.
  void runSelftestReal(std::function<void(bool)> done);

protected:
  void closeEvent(QCloseEvent * ev) override;
  void resizeEvent(QResizeEvent * ev) override;

private:
  // ---- 화면 구성
  QWidget * buildHeader();
  QWidget * buildLeftColumn();
  QWidget * buildCameraPanel();
  QWidget * buildQueuePanel();
  QWidget * buildLogPanel();
  QWidget * buildVoicePanel();
  QWidget * buildWorkspacePanel();
  QWidget * buildJogPanel();
  void placeLogo();                 // 상단 줄 가운데에 로고 (버튼·제목과 겹치지 않게)

  // ---- 동작
  bool ask(const QString & title, const QString & text);
  void onConnect();
  void onDisconnect();
  void onStop();
  void onQueueAdd();
  void onStartQueue();
  void nextTask();
  void onVoiceRun(bool automatic);
  void onHome();
  bool armOk(const std::string & what);   // 「조작 허용」 + 이동 가능 검사. 통과하면 잠금 시계를 다시 셈
  QString whyNotReady();                  // 지금 수동 조작을 못 하는 이유 (없으면 빈 문자열)
  // 화면 선택 (카메라 클릭): 큐브 위 = 그 물체(이거), 빈 곳 = 놓을 자리(저기)
  void onCameraClick(double x, double y);
  void runSelection();
  void clearSelection();
  void updateSelectionLabel();
  void addQueueItem(const QueuedTask & t, bool front);   // 큐 목록에 한 줄 (front = 맨 앞)

  // ---- 실물 모드 (main_window_real.cpp)
  QWidget * buildRealPickPanel();
  void selectPixel(double u, double v, double radius, const std::string & how);   // 큐브 또는 놓을 자리
  void updatePickLabel();
  void refreshReal(const Snapshot & s);                  // 100 ms: 보정·손짓·로봇 상태·조작 허용 맞추기
  void gestureTick(const Snapshot & s);
  void gestureConfirm(bool pick);
  std::optional<std::array<double, 3>> pickTarget(const std::string & what);   // 고른 큐브의 로봇 (x, y, z) [mm]
  void pickHover();
  void pickRun(const QString & heard = QString());
  void placeSelected(const QString & heard = QString());
  void placeHere(const QString & heard = QString());
  void pickAndPlace(const QString & heard = QString());
  void realVoice(const aris3_interfaces::msg::VoiceCommand & v);
  void homeVoice(const QString & heard);
  void powerOnTick();                                   // 「가동」 대기: 준비되면 가동 자세로
  void recoverHome();
  void toggleHandGuide(bool on);
  // ---- 주기 갱신
  void refresh();                   // 100 ms
  void refreshCamera();             // 33 ms

  std::shared_ptr<GuiNode> node_;
  Backend * backend_;
  std::vector<QueuedTask> queue_;
  bool queue_running_{false};
  bool was_task_running_{false};
  uint64_t log_shown_{0};
  int64_t last_voice_stamp_{-1};

  // 위젯
  QWidget * header_{nullptr};
  LogoBadge * logo_{nullptr};
  QLabel * title_{nullptr};
  QPushButton * btn_connect_{nullptr}, * btn_start_{nullptr};
  std::map<std::string, QLabel *> sys_;
  QLabel * lbl_task_state_{nullptr}, * lbl_joints_{nullptr}, * lbl_grip_{nullptr}, * lbl_cam_info_{nullptr};
  QProgressBar * bar_grip_{nullptr}, * bar_task_{nullptr};
  CameraView * camera_{nullptr};
  QCheckBox * chk_detect_{nullptr}, * chk_dry_{nullptr}, * chk_log_aris_{nullptr}, * chk_voice_auto_{nullptr};
  QComboBox * cmb_color_{nullptr}, * cmb_zone_{nullptr}, * cmb_voice_zone_{nullptr};
  QListWidget * lst_queue_{nullptr};
  QTableWidget * tbl_stage_{nullptr};
  QLabel * lbl_task_msg_{nullptr};
  QPlainTextEdit * txt_log_{nullptr};
  QLabel * lbl_voice_state_{nullptr}, * lbl_heard_{nullptr}, * lbl_parsed_{nullptr};
  QPushButton * btn_talk_{nullptr}, * btn_voice_run_{nullptr};
  QLineEdit * txt_cmd_{nullptr};
  QTimer timer_, cam_timer_;

  // ---- M2: 수동 조작·화면 선택·작업 공간
  QPushButton * btn_home_{nullptr}, * btn_gopen_{nullptr}, * btn_gclose_{nullptr}, * btn_sel_run_{nullptr};
  QCheckBox * chk_arm_{nullptr};
  QComboBox * cmb_step_{nullptr};
  QSlider * sld_speed_{nullptr};
  QLabel * lbl_speed_{nullptr}, * lbl_jog_reason_{nullptr}, * lbl_tcp_{nullptr}, * lbl_sel_{nullptr};
  std::vector<QPushButton *> jog_buttons_;
  WorkspaceView * workspace_{nullptr};
  std::chrono::steady_clock::time_point last_cmd_{std::chrono::steady_clock::now()};
  std::string sel_id_;                                 // 고른 물체 (이거)
  std::optional<std::array<double, 2>> sel_place_;     // 고른 놓을 자리 (저기, world [m])
  std::string current_color_;                          // 지금 작업 중인 색 (카메라·지도에서 강조)
  // 시험용 상태
  QTimer selftest_timer_;
  int selftest_step_{0};
  uint64_t selftest_log_mark_{0};
  bool selftest_ok_{true};
  std::chrono::steady_clock::time_point selftest_t0_;
  bool auto_yes_{false};                               // 시험용: 확인창을 띄우지 않고 「예」
  bool voice_direct_ = false;   // 음성 명령 처리 중: 확인창 없이 바로 실행 (2026-11-11 사용자 결정)
  // 「아리스 가동」: 접속이 끝나고(대기) 조작 허용이 켜지면 팔을 편다. 이 시각까지 기다린다 (없으면 대기 안 함)
  std::optional<std::chrono::steady_clock::time_point> power_on_until_;
  bool power_on_arm_warned_ = false;

  // ---- 실물 모드
  Backend * backend_base_{nullptr};
  RealBackend * real_{nullptr};                        // 실물 모드일 때만 (아니면 nullptr)
  bool dialog_open_{false};                            // 확인창이 떠 있는 동안 손짓·음성은 새 확인창을 띄우지 않음
  std::optional<std::array<double, 2>> pick_px_, place_px_;   // 고른 큐브·놓을 자리 (영상 픽셀)
  std::string pick_name_;
  QString calib_text_;
  std::chrono::steady_clock::time_point arm_sent_at_{};
  QLabel * lbl_pick_{nullptr}, * lbl_calib_{nullptr}, * lbl_gesture_{nullptr}, * lbl_robot_state_{nullptr};
  QCheckBox * chk_gesture_{nullptr}, * chk_hand_{nullptr};
  QPushButton * btn_recover_{nullptr};
};

}  // namespace aris
