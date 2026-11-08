#include "aris3_task_manager/target_estimator.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace aris3_task_manager
{

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}
}  // namespace

TargetEstimator::TargetEstimator(TargetQuery query, double same_object_radius_m)
: q_(std::move(query)), same_r_(same_object_radius_m)
{
  q_.color = lower(q_.color);
  q_.shape = lower(q_.shape);
}

bool TargetEstimator::matches(const Observation & o) const
{
  if (lower(o.color) != q_.color) {return false;}
  if (!q_.shape.empty() && lower(o.shape) != q_.shape) {return false;}
  return o.confidence >= q_.min_confidence;
}

void TargetEstimator::addFrame(const std::vector<Observation> & objects)
{
  ++seen_;
  std::vector<const Observation *> hits;
  for (const auto & o : objects) {
    if (matches(o)) {hits.push_back(&o);}
  }
  if (hits.empty()) {return;}

  Vec3 mean;
  for (auto * h : hits) {mean = mean + h->position;}
  mean = mean * (1.0 / static_cast<double>(hits.size()));
  for (auto * h : hits) {
    if (normXY(h->position - mean) > same_r_) {   // 서로 떨어진 후보가 둘 이상 -> 어느 것인지 모름
      ++ambiguous_frames_;
      max_candidates_ = std::max(max_candidates_, static_cast<int>(hits.size()));
      return;
    }
  }
  samples_.push_back(mean);
  if (id_.empty()) {id_ = hits.front()->id;}
}

Estimate TargetEstimator::estimate(int required_frames, double max_spread_m) const
{
  Estimate e;
  e.frames_seen = seen_;
  e.frames_matched = framesMatched();
  e.id = id_;
  char buf[200];
  if (ambiguous_frames_ > 0) {
    e.status = EstimateStatus::AMBIGUOUS;
    std::snprintf(buf, sizeof(buf), "조건에 맞는 물체가 서로 떨어져 %d개 보임 (영상 %d장) - 어느 것인지 정할 수 없음",
      max_candidates_, ambiguous_frames_);
    e.reason = buf;
    return e;
  }
  if (e.frames_matched < required_frames) {
    e.status = EstimateStatus::NOT_FOUND;
    std::snprintf(buf, sizeof(buf), "영상 %d장 중 %d장에서만 보임 (필요 %d장)",
      seen_, e.frames_matched, required_frames);
    e.reason = buf;
    return e;
  }
  for (const auto & s : samples_) {e.mean = e.mean + s;}
  e.mean = e.mean * (1.0 / static_cast<double>(samples_.size()));
  for (const auto & s : samples_) {e.spread_m = std::max(e.spread_m, normXY(s - e.mean));}
  if (e.spread_m > max_spread_m) {
    e.status = EstimateStatus::UNSTABLE;
    std::snprintf(buf, sizeof(buf), "위치가 %.1f mm 흔들림 (허용 %.1f mm)", e.spread_m * 1000, max_spread_m * 1000);
    e.reason = buf;
    return e;
  }
  e.status = EstimateStatus::OK;
  return e;
}

std::vector<Observation> occupantsNear(
  const std::vector<Observation> & objects, const Vec3 & zone_center, double radius,
  const Vec3 & target_pos, double same_object_radius)
{
  std::vector<Observation> out;
  for (const auto & o : objects) {
    if (normXY(o.position - target_pos) <= same_object_radius) {continue;}   // 목표 자신
    if (normXY(o.position - zone_center) <= radius) {out.push_back(o);}
  }
  return out;
}

}  // namespace aris3_task_manager
