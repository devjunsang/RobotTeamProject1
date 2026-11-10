// ARIS 순수 로직 구현 (gui_logic.hpp 설명 참고).
#include "aris3_gui_cpp/gui_logic.hpp"

#include <algorithm>

namespace aris
{

void RateMeter::tick(Clock::time_point now)
{
  t_.push_back(now);
  while (t_.size() > window_) {t_.pop_front();}
}

double RateMeter::hz(Clock::time_point now) const
{
  if (t_.size() < 2) {return 0.0;}
  if (std::chrono::duration<double>(now - t_.back()).count() > 2.0) {return 0.0;}   // 끊김
  const double span = std::chrono::duration<double>(t_.back() - t_.front()).count();
  return span > 0.0 ? static_cast<double>(t_.size() - 1) / span : 0.0;
}

bool RateMeter::fresh(double max_age, Clock::time_point now) const
{
  return !t_.empty() && std::chrono::duration<double>(now - t_.back()).count() < max_age;
}

bool isArisLog(const std::string & level, const std::string & node, const std::string & text)
{
  // 매 계획마다 나오는 무해한 MoveIt 경고
  static const std::vector<std::string> noise = {
    "planning volume was not specified", "did not fill out the 'planner_id'", "No 3D sensor plugin",
    "Resolution not specified for Octomap"};
  for (const auto & n : noise) {
    if (text.find(n) != std::string::npos) {return false;}
  }
  if (level == "WARN" || level == "ERROR" || level == "FATAL") {return true;}
  // INFO 까지 보여 줄 노드: GUI 자신, ARIS3 노드, Task Manager, 실물 브리지 안 RobotLink
  for (const char * prefix : {"aris", "task_manager", "e0509_manual"}) {
    if (node.rfind(prefix, 0) == 0) {return true;}
  }
  return node == "gui";
}

const std::vector<std::string> & stageNames()
{
  static const std::vector<std::string> names = {
    "VALIDATE", "OBSERVE", "PERCEIVE", "PLAN", "PICK", "PLACE", "VERIFY"};
  return names;
}

void StageTracker::reset()
{
  for (const auto & s : stageNames()) {
    state_[s] = StageState::WAIT;
    sec_[s] = 0.0;
  }
  t0_ = Clock::now();
}

void StageTracker::feedback(const std::string & stage, Clock::time_point now)
{
  auto it = state_.find(stage);
  if (it == state_.end() || it->second == StageState::RUN) {return;}
  // 새 단계 시작: 실행 중이던 단계는 끝난 것
  for (auto & [name, st] : state_) {
    if (st == StageState::RUN) {
      st = StageState::DONE;
      sec_[name] = std::chrono::duration<double>(now - t0_).count();
    }
  }
  it->second = StageState::RUN;
  t0_ = now;
}

void StageTracker::finish(bool ok, const std::string & failed_stage, Clock::time_point now)
{
  for (auto & [name, st] : state_) {
    if (st == StageState::RUN) {
      st = ok ? StageState::DONE : StageState::FAIL;
      sec_[name] = std::chrono::duration<double>(now - t0_).count();
    }
  }
  if (!ok && state_.count(failed_stage)) {state_[failed_stage] = StageState::FAIL;}
}

StageState StageTracker::state(const std::string & stage) const
{
  auto it = state_.find(stage);
  return it == state_.end() ? StageState::WAIT : it->second;
}

double StageTracker::seconds(const std::string & stage) const
{
  auto it = sec_.find(stage);
  return it == sec_.end() ? 0.0 : it->second;
}

}  // namespace aris
