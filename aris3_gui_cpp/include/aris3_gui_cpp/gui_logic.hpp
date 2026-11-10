// ARIS 순수 로직 (Qt·ROS 없음) - 단위 시험(test/test_gui_logic.cpp)에서 바로 쓴다.
//   RateMeter     : 최근 메시지 간격으로 수신 주기(Hz)와 「아직 살아 있는가」 를 잰다
//   isArisLog     : 「ARIS 로그만」 거르기 (aris3_gui ros_io.LOG_NODES·NOISE 와 같은 규칙)
//   StageTracker  : ExecuteTask feedback 의 단계 이름으로 단계별 상태(대기/실행/완료/실패)·걸린 시간을 계산
#pragma once

#include <chrono>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace aris
{

using Clock = std::chrono::steady_clock;

// 수신 주기 측정기: tick() 을 메시지마다 부른다.
class RateMeter
{
public:
  explicit RateMeter(size_t window = 20) : window_(window) {}
  // 메시지 하나 받음 (now 를 주면 그 시각, 시험용)
  void tick(Clock::time_point now = Clock::now());
  // 최근 window 개 간격으로 계산한 Hz. 2 초 넘게 안 오면 0
  double hz(Clock::time_point now = Clock::now()) const;
  // max_age 초 안에 받았는가
  bool fresh(double max_age = 1.0, Clock::time_point now = Clock::now()) const;

private:
  size_t window_;
  std::deque<Clock::time_point> t_;
};

// 「ARIS 로그만」 에 보일 줄인가: GUI 자신, ARIS3·Task Manager·실물 로봇(e0509_manual) 노드, 또는 WARN 이상.
// 매 계획마다 나오는 무해한 MoveIt 경고(NOISE)는 숨긴다.
bool isArisLog(const std::string & level, const std::string & node, const std::string & text);

// 작업 단계 (Task Manager 순서와 같음)
const std::vector<std::string> & stageNames();

enum class StageState { WAIT, RUN, DONE, FAIL };

// ExecuteTask 진행 추적: 새 단계가 시작되면 앞 단계는 끝난 것으로 본다.
class StageTracker
{
public:
  StageTracker() { reset(); }
  // 새 작업 시작: 모든 단계 대기
  void reset();
  // feedback 의 단계 이름 (모르는 이름은 무시)
  void feedback(const std::string & stage, Clock::time_point now = Clock::now());
  // 작업 끝: 실행 중이던 단계를 완료/실패로, failed_stage 가 있으면 그 단계를 실패로
  void finish(bool ok, const std::string & failed_stage, Clock::time_point now = Clock::now());
  StageState state(const std::string & stage) const;
  // 단계에 걸린 시간 [s] (끝난 단계만, 아니면 0)
  double seconds(const std::string & stage) const;

private:
  std::map<std::string, StageState> state_;
  std::map<std::string, double> sec_;
  Clock::time_point t0_;
};

}  // namespace aris
