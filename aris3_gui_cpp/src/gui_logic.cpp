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

bool isIgnoredSpeechLog(const std::string & node, const std::string & text)
{
  if (node.rfind("aris3_voice", 0) != 0) {return false;}
  // 음성 노드(voice_node.py)가 명령으로 받지 않은 말에 붙이는 문구
  for (const char * mark : {"호출어(아리스) 없음, 무시", "ARIS 가 말하는 중에 들린 말이라 무시"}) {
    if (text.find(mark) != std::string::npos) {return true;}
  }
  return false;
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

std::string objectKo(const std::string & color, const std::string & shape, int number)
{
  static const std::map<std::string, std::string> k{{"cube", "큐브"}, {"triangle", "삼각형"}, {"circle", "원"}};
  const auto it = k.find(shape);
  const std::string c = colorKo(color);
  if (c.empty() && it == k.end()) {return "고르신 물체";}
  const std::string s = it == k.end() ? std::string("큐브") : it->second;   // 작업 공간의 물체는 큐브
  if (c.empty()) {return s;}
  // 큐브 번호 (2026-11-14): 같은 색이 여럿일 때 「1번 파란색 큐브」
  return (number > 0 ? std::to_string(number) + "번 " : std::string()) + c + " " + s;
}

int cubeNumberFromId(const std::string & id)
{
  // 맨 끝의 "_숫자" 만 번호로 본다 ("blue_cube_2" -> 2). 숫자가 없거나 "_" 가 앞에 없으면 0
  const auto us = id.rfind('_');
  if (us == std::string::npos || us + 1 >= id.size()) {return 0;}
  int n = 0;
  for (std::size_t i = us + 1; i < id.size(); ++i) {
    if (id[i] < '0' || id[i] > '9' || n > 99) {return 0;}
    n = n * 10 + (id[i] - '0');
  }
  return n;
}

int pickCubeIndex(const std::vector<std::string> & ids, int number)
{
  if (ids.empty()) {return -1;}
  if (number <= 0) {                                  // 번호를 말하지 않음: 하나뿐일 때만 정해진다
    return ids.size() == 1 ? 0 : -2;
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (cubeNumberFromId(ids[i]) == number) {return static_cast<int>(i);}
  }
  // 그 색이 하나뿐이면 이름에 번호가 없다 ("blue_cube") - 「1번 파란색 큐브」 는 그 큐브
  if (ids.size() == 1 && number == 1 && cubeNumberFromId(ids[0]) == 0) {return 0;}
  return -1;
}

std::vector<double> searchSweepDeg()
{
  return {20.0, 40.0, -20.0, -40.0, 0.0};
}

std::vector<std::string> missingColors(const std::vector<std::string> & want, const std::vector<std::string> & seen)
{
  std::vector<std::string> out;
  for (const auto & c : want) {
    if (c.empty() || std::find(out.begin(), out.end(), c) != out.end()) {continue;}
    if (std::find(seen.begin(), seen.end(), c) == seen.end()) {out.push_back(c);}
  }
  return out;
}

bool tooFar(double x_mm, double y_mm, double max_mm)
{
  return std::hypot(x_mm, y_mm) > max_mm;
}

std::vector<std::vector<std::array<double, 2>>> reachBoundaryPx(const std::array<double, 9> & H, double radius_mm,
  int n, double max_px)
{
  std::vector<std::vector<std::array<double, 2>>> out;
  // 3x3 역행렬 (여인수). 행렬식이 0 에 가까우면 옮길 수 없다
  const double a = H[0], b = H[1], c = H[2], d = H[3], e = H[4], f = H[5], g = H[6], h = H[7], i = H[8];
  const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (std::abs(det) < 1e-12 || n < 3 || radius_mm <= 0.0) {return out;}
  const std::array<double, 9> inv{
    (e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det,
    (f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det,
    (d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det};
  // 로봇 (x, y) mm -> 픽셀. w 의 부호가 작업 공간(로봇 앞쪽) 점과 다르면 영상 평면 뒤 = 보이지 않는 점
  const auto project = [&](double x, double y, double & u, double & v) {
      const double w = inv[6] * x + inv[7] * y + inv[8];
      u = (inv[0] * x + inv[1] * y + inv[2]) / w;
      v = (inv[3] * x + inv[4] * y + inv[5]) / w;
      return w;
    };
  double ru = 0.0, rv = 0.0;
  const double w_ref = project(radius_mm * 0.7, 0.0, ru, rv);
  std::vector<std::array<double, 2>> cur;
  bool all_ok = true;
  for (int k = 0; k < n; ++k) {
    const double th = 2.0 * M_PI * k / n;
    double u = 0.0, v = 0.0;
    const double w = project(radius_mm * std::cos(th), radius_mm * std::sin(th), u, v);
    const bool ok = w * w_ref > 0.0 && std::abs(w) > 1e-3 * std::abs(w_ref) && std::isfinite(u) && std::isfinite(v) &&
      std::abs(u) < max_px && std::abs(v) < max_px;
    if (ok) {
      cur.push_back({u, v});
    } else {
      all_ok = false;
      if (cur.size() > 1) {out.push_back(cur);}
      cur.clear();
    }
  }
  if (all_ok && !cur.empty()) {
    cur.push_back(cur.front());                        // 원 전체가 보임: 닫는다
    out.push_back(cur);
  } else if (cur.size() > 1) {
    // 끝 조각이 첫 조각과 이어져 있으면(0° 를 지나는 호) 하나로 붙인다
    if (!out.empty() && out.front().size() > 1) {
      double u0 = 0.0, v0 = 0.0;
      const double w0 = project(radius_mm, 0.0, u0, v0);
      if (w0 * w_ref > 0.0 && std::abs(out.front().front()[0] - u0) < 1e-6 && std::abs(out.front().front()[1] - v0) < 1e-6) {
        cur.insert(cur.end(), out.front().begin(), out.front().end());
        out.front() = cur;
        return out;
      }
    }
    out.push_back(cur);
  }
  return out;
}

std::string tooFarReply(const std::string & what)
{
  return what + josa(what, "은", "는") + " 너무 멀어서 팔이 닿지 않아요.";
}

std::string voiceReply(const std::string & action, const std::string & color, const std::string & shape,
  bool ref_place, const std::string & target_color, int number, int target_number)
{
  const std::string obj = objectKo(color, shape, number);
  if (action == "pick" && !target_color.empty()) {   // 쌓기: 받침 큐브 위에 올리기
    const std::string base = objectKo(target_color, "cube", target_number);
    return "네, " + obj + josa(obj, "을", "를") + " " + base + " 위에 올립니다.";
  }
  if (action == "pick") {
    return ref_place ? "네, " + obj + josa(obj, "을", "를") + " 집어서 고르신 자리로 옮깁니다." :
           "네, " + obj + josa(obj, "을", "를") + " 잡습니다.";
  }
  if (action == "place" && !target_color.empty()) {  // 올려놓기 (2026-11-14): 들고 있는 큐브를 받침 위에
    return "네, 들고 있는 큐브를 " + objectKo(target_color, "cube", target_number) + " 위에 올려놓겠습니다.";
  }
  if (action == "place") {return ref_place ? "네, 고르신 자리에 내려놓겠습니다." : "네, 여기에 내려놓겠습니다.";}
  if (action == "find") {return "네, " + obj + josa(obj, "을", "를") + " 찾아볼게요.";}   // 찾기 (2026-11-14)
  if (action == "scan") {return "네, 큐브를 찾아볼게요.";}   // 탐색 (2026-11-14): 세 가지 색이 보일 때까지
  if (action == "home") {return "네, 홈 자세로 돌아갑니다.";}
  if (action == "stop") {return "네, 멈춥니다.";}
  if (action == "resume") {return "네, 이어서 하겠습니다.";}
  if (action == "power_on") {return "네, 가동합니다.";}
  if (action == "disconnect") {return "네, 연결을 해제합니다.";}
  if (action == "gripper_open") {return "네, 그리퍼를 엽니다.";}
  return "잘 못 알아들었어요. 다시 말씀해 주세요.";
}

}  // namespace aris

