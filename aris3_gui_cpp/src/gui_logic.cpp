// ARIS 순수 로직 구현 (gui_logic.hpp 설명 참고).
#include "aris3_gui_cpp/gui_logic.hpp"

#include <algorithm>
#include <cmath>

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

Vec3 quatApply(const Quat & q, const Vec3 & v)
{
  // v' = v + w t + q_vec x t,  t = 2 (q_vec x v)  (aris3_gui widgets.quat_apply 와 같음)
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double tx = 2 * (y * v[2] - z * v[1]);
  const double ty = 2 * (z * v[0] - x * v[2]);
  const double tz = 2 * (x * v[1] - y * v[0]);
  return {v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)};
}

double PinholeCamera::focal() const
{
  return (width / 2.0) / std::tan(hfov / 2.0);
}

std::optional<Vec3> PinholeCamera::project(const Vec3 & p) const
{
  Vec3 c = quatApply(q, p);
  c = {c[0] + t[0], c[1] + t[1], c[2] + t[2]};
  if (c[2] <= 0.01) {return std::nullopt;}           // 카메라 뒤 또는 너무 가까움
  const double f = focal();
  return Vec3{width / 2.0 + f * c[0] / c[2], height / 2.0 + f * c[1] / c[2], c[2]};
}

std::optional<std::array<double, 2>> PinholeCamera::unproject(double u, double v, double plane_z) const
{
  const Quat qi{-q[0], -q[1], -q[2], q[3]};          // world <- 카메라 회전
  const double f = focal();
  const Vec3 d = quatApply(qi, {(u - width / 2.0) / f, (v - height / 2.0) / f, 1.0});   // 광선 방향 (world)
  const Vec3 o = quatApply(qi, {-t[0], -t[1], -t[2]});                                // 카메라 중심 (world)
  if (std::abs(d[2]) < 1e-9) {return std::nullopt;}
  const double s = (plane_z - o[2]) / d[2];
  if (s <= 0) {return std::nullopt;}
  return std::array<double, 2>{o[0] + s * d[0], o[1] + s * d[1]};
}

// ======================================================================== ARIS 목소리 대답
std::string josa(const std::string & word, const std::string & with_final, const std::string & without)
{
  // UTF-8 마지막 글자(한글은 3바이트)를 코드 포인트로
  if (word.size() < 3) {return without;}
  const auto * b = reinterpret_cast<const unsigned char *>(word.data() + word.size() - 3);
  if ((b[0] & 0xF0) != 0xE0) {return without;}
  const unsigned cp = ((b[0] & 0x0Fu) << 12) | ((b[1] & 0x3Fu) << 6) | (b[2] & 0x3Fu);
  if (cp < 0xAC00 || cp > 0xD7A3) {return without;}
  return (cp - 0xAC00) % 28 != 0 ? with_final : without;    // 28 = 받침 종류 수 (0 = 받침 없음)
}

std::string colorKo(const std::string & color)
{
  static const std::map<std::string, std::string> k{
    {"red", "빨간색"}, {"blue", "파란색"}, {"green", "초록색"}, {"yellow", "노란색"}};
  const auto it = k.find(color);
  return it == k.end() ? std::string() : it->second;
}

std::string objectKo(const std::string & color, const std::string & shape)
{
  static const std::map<std::string, std::string> k{{"cube", "큐브"}, {"triangle", "삼각형"}, {"circle", "원"}};
  const auto it = k.find(shape);
  const std::string c = colorKo(color);
  if (c.empty() && it == k.end()) {return "고르신 물체";}
  const std::string s = it == k.end() ? std::string("큐브") : it->second;   // 작업 공간의 물체는 큐브
  return c.empty() ? s : c + " " + s;
}

std::string voiceReply(const std::string & action, const std::string & color, const std::string & shape,
  bool ref_place)
{
  const std::string obj = objectKo(color, shape);
  if (action == "pick") {
    return ref_place ? "네, " + obj + josa(obj, "을", "를") + " 집어서 고르신 자리로 옮깁니다." :
           "네, " + obj + josa(obj, "을", "를") + " 잡습니다.";
  }
  if (action == "place") {return ref_place ? "네, 고르신 자리에 내려놓겠습니다." : "네, 여기에 내려놓겠습니다.";}
  if (action == "home") {return "네, 홈 자세로 돌아갑니다.";}
  if (action == "stop") {return "네, 멈춥니다.";}
  if (action == "resume") {return "네, 이어서 하겠습니다.";}
  if (action == "power_on") {return "네, 가동합니다.";}
  if (action == "disconnect") {return "네, 연결을 해제합니다.";}
  return "잘 못 알아들었어요. 다시 말씀해 주세요.";
}

}  // namespace aris

