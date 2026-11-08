// 여러 장의 카메라 결과에서 목표 물체 하나의 위치를 추정한다 (순수 C++, 단위 시험 대상).
//
// 왜 한 장이 아니라 여러 장인가: 한 장의 값은 잡음·순간 오검출이 섞일 수 있다.
// N 장을 모아 평균 내고, 값이 얼마나 흔들렸는지(spread)를 보고 믿을지 결정한다.
// "인식이 불확실하면 동작하지 않는다" (팀 안전 원칙) 를 코드로 옮긴 부분이다.
#pragma once

#include <string>
#include <vector>

#include "aris3_task_manager/types.hpp"

namespace aris3_task_manager
{

struct Observation
{
  std::string id;      // 비전이 붙인 이름 (모르면 "")
  std::string color;
  std::string shape;
  Vec3 position;       // world frame [m]
  double confidence{1.0};
};

struct TargetQuery
{
  std::string color;          // 필수
  std::string shape;          // "" = 상관없음
  double min_confidence{0.5};
};

enum class EstimateStatus { OK, NOT_FOUND, UNSTABLE, AMBIGUOUS };

struct Estimate
{
  EstimateStatus status{EstimateStatus::NOT_FOUND};
  Vec3 mean;                // world [m]
  double spread_m{0.0};     // 평균에서 가장 먼 표본까지의 수평 거리
  int frames_seen{0};       // 받은 영상 수
  int frames_matched{0};    // 목표가 보인 영상 수
  std::string id;           // 비전이 붙인 이름
  std::string reason;       // 실패 이유 (사람이 읽는 설명)
};

class TargetEstimator
{
public:
  // same_object_radius_m: 한 영상 안에서 이 거리 안의 여러 검출은 같은 물체가 쪼개진 것으로 본다.
  explicit TargetEstimator(TargetQuery query, double same_object_radius_m = 0.02);

  void addFrame(const std::vector<Observation> & objects);
  Estimate estimate(int required_frames, double max_spread_m) const;

  int framesSeen() const {return seen_;}
  int framesMatched() const {return static_cast<int>(samples_.size());}
  bool matches(const Observation & o) const;

private:
  TargetQuery q_;
  double same_r_;
  std::vector<Vec3> samples_;
  int seen_{0};
  int ambiguous_frames_{0};
  int max_candidates_{0};
  std::string id_;
};

// 구역 중심 radius 안에 있는 "목표가 아닌" 물체들 (놓을 자리가 비었는지 확인용).
// target_pos 에서 same_object_radius 안의 검출은 목표 자신으로 보고 뺀다.
std::vector<Observation> occupantsNear(
  const std::vector<Observation> & objects, const Vec3 & zone_center, double radius,
  const Vec3 & target_pos, double same_object_radius = 0.02);

}  // namespace aris3_task_manager
