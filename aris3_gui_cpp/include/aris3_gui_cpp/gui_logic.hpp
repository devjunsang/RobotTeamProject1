// ARIS 순수 로직 (Qt·ROS 없음) - 단위 시험(test/test_gui_logic.cpp)에서 바로 쓴다.
//   RateMeter     : 최근 메시지 간격으로 수신 주기(Hz)와 「아직 살아 있는가」 를 잰다
//   isArisLog     : 「ARIS 로그만」 거르기 (aris3_gui ros_io.LOG_NODES·NOISE 와 같은 규칙)
//   StageTracker  : ExecuteTask feedback 의 단계 이름으로 단계별 상태(대기/실행/완료/실패)·걸린 시간을 계산
//   PinholeCamera : 카메라 TF + 화각으로 world 점 <-> 영상 픽셀 (aris3_gui widgets.CameraView.project/unproject 와 같음)
//   voiceReply 등 : ARIS 목소리로 말할 대답 문장 (「네, 노란색 큐브를 잡습니다.」)
#pragma once

#include <array>
#include <chrono>
#include <optional>
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
// 로그 창에 넣지 않을 음성 줄인가 (2026-11-14): 호출어 「아리스」 없이 들린 말(주변 대화·ARIS 자신의 대답)을
// 음성 노드가 「… 무시」 로 남긴 줄. 「아리스」 로 부른 명령만 화면에 보인다 (기록 파일 /tmp/aris_voice.log 에는 그대로)
bool isIgnoredSpeechLog(const std::string & node, const std::string & text);

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

// 3차원 벡터·쿼터니언 (x, y, z, w)
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

// 쿼터니언 q 로 벡터 v 회전
Vec3 quatApply(const Quat & q, const Vec3 & v);

// 바늘구멍 카메라: world -> 카메라 광학 좌표 변환(TF lookup_transform(camera_frame, "world"))과 수평 화각.
// 광학 좌표: z = 앞(광축), x = 영상 오른쪽, y = 영상 아래 (ROS 광학 프레임 규약)
struct PinholeCamera
{
  Vec3 t{0, 0, 0};        // 변환의 이동
  Quat q{0, 0, 0, 1};     // 변환의 회전
  double hfov{1.4};       // 수평 화각 [rad]
  int width{640}, height{480};

  double focal() const;   // 픽셀 단위 초점 거리
  // world 점 -> (u, v, 깊이). 카메라 뒤면 없음
  std::optional<Vec3> project(const Vec3 & p) const;
  // 영상 픽셀 -> world 평면 z = plane_z 위의 (x, y). 평면과 만나지 않으면 없음
  std::optional<std::array<double, 2>> unproject(double u, double v, double plane_z) const;
};

// ---- ARIS 목소리 대답 (2026-11-11). 문장에 「아리스」 를 넣지 않는다: 마이크가 이 소리를 다시 들어도
//      호출어가 없어 명령으로 받지 않게.
// 받침에 맞는 조사: 마지막 글자에 받침이 있으면 with_final(을·이), 없으면 without(를·가). 한글이 아니면 without
std::string josa(const std::string & word, const std::string & with_final, const std::string & without);
// 색 이름 (red -> 빨간색). 모르는 색은 빈 문자열
std::string colorKo(const std::string & color);
// 말할 물체 이름: 「노란색 큐브」 「파란색 삼각형」. 색·도형이 없으면 「고르신 물체」.
// number > 0 (큐브 번호, 2026-11-14) 이고 색이 있으면 「1번 파란색 큐브」
std::string objectKo(const std::string & color, const std::string & shape, int number = 0);
// 색 검출 이름의 큐브 번호: "blue_cube_2" -> 2, 번호가 없으면("blue_cube") 0
int cubeNumberFromId(const std::string & id);
// 같은 색 큐브들의 이름 ids 중 말한 번호 number 의 큐브 순번 (2026-11-14, 음성 「1번 파란색 큐브」).
//   number = 0 (번호를 말하지 않음): 하나뿐이면 0, 없으면 -1, 여럿이면 -2 (어느 것인지 모름)
//   number > 0: 그 번호의 큐브 순번. 그 색이 하나뿐이라 이름에 번호가 없을 때는 1번만 그 큐브로 본다. 없으면 -1
int pickCubeIndex(const std::vector<std::string> & ids, int number);
// 명령을 받아 실행할 때의 대답: 「네, 노란색 큐브를 잡습니다.」. 모르는 동작이면 다시 말해 달라는 문장
// 색 탐색 (2026-11-13): 말한 색이 로봇 팔에 가려 안 보일 때 J1 을 돌릴 목표 (시작 자세 기준 [deg]).
// 좌우로 점점 넓게 갔다가 마지막에 시작 자세(0)로 돌아온다.
std::vector<double> searchSweepDeg();
// want 중에서 seen(지금 검출된 색들)에 없는 색 (중복·빈 값 제외, 순서 유지)
std::vector<std::string> missingColors(const std::vector<std::string> & want, const std::vector<std::string> & seen);

// target_color: 쌓기의 받침 큐브 색 (2026-11-13) - 「네, 노란색 큐브를 파란색 큐브 위에 올립니다.」
// 닿는 거리 (2026-11-14): 로봇 베이스에서 (x, y) mm 까지가 max_mm 를 넘으면 팔이 닿지 않는다고 본다.
// 브리지 파라미터 reach_max_mm 와 같은 값 (실물에서 686 mm 는 갔고 716 mm 는 컨트롤러가 거부)
constexpr double kReachMaxMm = 700.0;
bool tooFar(double x_mm, double y_mm, double max_mm = kReachMaxMm);
// 가까운 쪽 한계 (2026-11-14): 베이스에서 min_mm 보다 가까우면 집는 높이로 내려가다 3번 관절 한계에 걸린다.
// 브리지 파라미터 reach_min_mm 와 같은 값 (실물에서 약 210 mm 는 관절 한계로 멈췄고 428 mm 는 됨)
constexpr double kReachMinMm = 300.0;
bool tooClose(double x_mm, double y_mm, double min_mm = kReachMinMm);
// 「파란색 큐브는 로봇에 너무 가까워서 집을 수 없어요.」
std::string tooCloseReply(const std::string & what);
// 닿는 범위 경계선 (2026-11-14): 로봇 베이스에서 radius_mm 인 원을 카메라 영상 픽셀로 옮긴 선들.
//   H: 카메라 보정 3x3 (픽셀 -> 로봇 mm, 행 우선 9개 - RealRobotState.calib_homography). 그 역으로 원 위 n 점을 옮긴다.
//   원이 영상 평면 뒤로 넘어가거나(원근) max_px 보다 멀리 떨어지는 점에서는 선을 끊는다 -> 이어진 조각들의 목록.
//   원 전체가 보이면 조각 하나이고 첫 점으로 닫혀 있다. 보정이 뒤집힐 수 없는 행렬이면 빈 목록.
std::vector<std::vector<std::array<double, 2>>> reachBoundaryPx(const std::array<double, 9> & H, double radius_mm,
  int n = 120, double max_px = 4000.0);
// 「노란색 큐브는 너무 멀어서 팔이 닿지 않아요.」 (what: 「노란색 큐브」 「놓을 자리」 ...)
std::string tooFarReply(const std::string & what);

// number, target_number: 큐브 번호 (2026-11-14, 0 = 말하지 않음) - 「네, 1번 파란색 큐브를 잡습니다.」
std::string voiceReply(const std::string & action, const std::string & color, const std::string & shape,
  bool ref_place, const std::string & target_color = "", int number = 0, int target_number = 0);

}  // namespace aris
