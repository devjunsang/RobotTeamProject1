// ARIS 메인 창 (M1: 시뮬레이션 모드). 배치는 목업과 aris3_gui 와 같다:
//   ┌ 상단: 제목 · [가운데 ARIS 로고] · Connect · Disconnect · Start Task · Stop · 정지 ───────────┐
//   │ 시스템 상태  │ 카메라 Top View                 │ 작업 공간 2D (M2)            │
//   │ 로봇 상태    │                                  │ 작업 큐 / 단계 진행          │
//   │ 그리퍼       ├──────────────────────────────────┼──────────────┬───────────────┤
//   │ 카메라 정보  │ 로그                             │ 음성 명령    │ 수동 조작 (M2) │
// 안전 규칙 (aris3_gui 와 같음): 정지는 언제나 동작 (버튼, Space, Esc) - 화면의 정지는 인증된 비상정지 장치를
// 대신하지 않는다. 음성 명령은 해석 결과를 보여 주고, 「자동 실행」 이 아니면 사용자가 「실행」 을 눌러야 한다.
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
#include <QTableWidget>
#include <QTimer>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aris3_gui_cpp/ros_io.hpp"
#include "aris3_gui_cpp/sim_backend.hpp"
#include "aris3_gui_cpp/widgets.hpp"

namespace aris
{

// 작업 큐 한 줄
struct QueuedTask
{
  std::string color, zone;
  bool dry{false};
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(std::shared_ptr<GuiNode> node, SimBackend * backend, QWidget * parent = nullptr);

  // 시험용 (명령줄 --connect / --task): 버튼을 누른 것과 같은 동작
  void connectNow() {onConnect();}
  void queueTask(const std::string & color, const std::string & zone, bool dry = false);
  void startQueue() {onStartQueue();}

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
  QWidget * buildPlaceholder(const QString & title, const QString & text);
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

  // ---- 주기 갱신
  void refresh();                   // 100 ms
  void refreshCamera();             // 33 ms

  std::shared_ptr<GuiNode> node_;
  SimBackend * backend_;
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
};

}  // namespace aris
