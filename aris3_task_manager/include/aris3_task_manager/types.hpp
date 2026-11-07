// 공통 타입: 3차원 벡터, 작업 단계, 오류 코드, 단계 실패 예외.
// ROS 에 의존하지 않는 순수 C++ 이라서 단위 시험(gtest)에서 그대로 쓸 수 있다.
#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace aris3_task_manager
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline Vec3 operator*(const Vec3 & a, double s) {return {a.x * s, a.y * s, a.z * s};}
inline double norm(const Vec3 & a) {return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);}
inline double normXY(const Vec3 & a) {return std::sqrt(a.x * a.x + a.y * a.y);}

// 작업 단계. 액션 feedback 의 stage 문자열과 1:1 로 대응한다.
enum class Stage { VALIDATE, OBSERVE, PERCEIVE, PLAN, PICK, PLACE, VERIFY, RECOVER, DONE };

inline const char * stageName(Stage s)
{
  switch (s) {
    case Stage::VALIDATE: return "VALIDATE";
    case Stage::OBSERVE: return "OBSERVE";
    case Stage::PERCEIVE: return "PERCEIVE";
    case Stage::PLAN: return "PLAN";
    case Stage::PICK: return "PICK";
    case Stage::PLACE: return "PLACE";
    case Stage::VERIFY: return "VERIFY";
    case Stage::RECOVER: return "RECOVER";
    case Stage::DONE: return "DONE";
  }
  return "?";
}

// 정상 흐름의 단계 수 (VALIDATE ~ VERIFY). feedback 의 stage_count.
constexpr int kMainStageCount = 7;

// ExecuteTask.action 의 ERROR_* 상수와 같은 값 (바꾸면 둘 다 바꿔야 한다).
enum class ErrorCode : uint8_t
{
  NONE = 0,
  INVALID_GOAL = 1,
  NOT_READY = 2,
  TARGET_NOT_FOUND = 3,
  TARGET_UNSTABLE = 4,
  TARGET_AMBIGUOUS = 5,
  UNREACHABLE = 6,
  MOTION_FAILED = 7,
  GRASP_FAILED = 8,
  PLACE_VERIFY_FAILED = 9,
  TIMEOUT = 10,
  CANCELED = 11,
  INTERNAL = 12,
  DESTINATION_OCCUPIED = 13,
};

inline const char * errorName(ErrorCode c)
{
  switch (c) {
    case ErrorCode::NONE: return "NONE";
    case ErrorCode::INVALID_GOAL: return "INVALID_GOAL";
    case ErrorCode::NOT_READY: return "NOT_READY";
    case ErrorCode::TARGET_NOT_FOUND: return "TARGET_NOT_FOUND";
    case ErrorCode::TARGET_UNSTABLE: return "TARGET_UNSTABLE";
    case ErrorCode::TARGET_AMBIGUOUS: return "TARGET_AMBIGUOUS";
    case ErrorCode::UNREACHABLE: return "UNREACHABLE";
    case ErrorCode::MOTION_FAILED: return "MOTION_FAILED";
    case ErrorCode::GRASP_FAILED: return "GRASP_FAILED";
    case ErrorCode::PLACE_VERIFY_FAILED: return "PLACE_VERIFY_FAILED";
    case ErrorCode::TIMEOUT: return "TIMEOUT";
    case ErrorCode::CANCELED: return "CANCELED";
    case ErrorCode::INTERNAL: return "INTERNAL";
    case ErrorCode::DESTINATION_OCCUPIED: return "DESTINATION_OCCUPIED";
  }
  return "?";
}

// 단계 안에서 "더 진행할 수 없음"을 알리는 예외.
// 각 단계 함수는 실패하면 이 예외를 던지고, execute() 한 곳에서 잡아 액션 Result 로 바꾼다.
// (단계마다 if (실패) return ... 을 반복하지 않아 흐름이 읽기 쉬워진다.)
class TaskError : public std::runtime_error
{
public:
  TaskError(ErrorCode code, const std::string & msg)
  : std::runtime_error(msg), code_(code) {}
  ErrorCode code() const {return code_;}

private:
  ErrorCode code_;
};

}  // namespace aris3_task_manager
