// target_estimator.cpp 단위 시험: "언제 믿고 언제 거부하는가"를 확인한다.
#include <gtest/gtest.h>

#include "aris3_task_manager/target_estimator.hpp"

using namespace aris3_task_manager;  // NOLINT

namespace
{
Observation obs(const std::string & color, double x, double y, double conf = 0.9, const std::string & shape = "cube")
{
  return Observation{color + "_block", color, shape, Vec3{x, y, 0.77}, conf};
}
}  // namespace

TEST(TargetEstimator, AveragesStableDetections)
{
  TargetEstimator est({"red", "", 0.5, ""});
  for (int i = 0; i < 5; ++i) {
    est.addFrame({obs("red", 0.450 + 0.001 * (i % 2), -0.150), obs("blue", 0.45, 0.15)});
  }
  const auto e = est.estimate(5, 0.005);
  ASSERT_EQ(e.status, EstimateStatus::OK);
  EXPECT_NEAR(e.mean.x, 0.4504, 1e-4);
  EXPECT_EQ(e.id, "red_block");
  EXPECT_LT(e.spread_m, 0.005);
}

TEST(TargetEstimator, NotFoundWhenTooFewFrames)
{
  TargetEstimator est({"green", "", 0.5, ""});
  est.addFrame({obs("green", 0.45, 0.0)});
  est.addFrame({obs("red", 0.45, -0.15)});
  const auto e = est.estimate(5, 0.005);
  EXPECT_EQ(e.status, EstimateStatus::NOT_FOUND);
  EXPECT_EQ(e.frames_seen, 2);
  EXPECT_EQ(e.frames_matched, 1);
}

TEST(TargetEstimator, UnstableWhenPositionJumps)
{
  TargetEstimator est({"red", "", 0.5, ""});
  for (int i = 0; i < 5; ++i) {est.addFrame({obs("red", 0.45 + (i == 4 ? 0.03 : 0.0), -0.15)});}
  EXPECT_EQ(est.estimate(5, 0.005).status, EstimateStatus::UNSTABLE);
}

TEST(TargetEstimator, AmbiguousWhenTwoSeparateMatches)
{
  TargetEstimator est({"red", "", 0.5, ""});
  est.addFrame({obs("red", 0.45, -0.15), obs("red", 0.30, 0.10)});
  EXPECT_EQ(est.estimate(1, 0.005).status, EstimateStatus::AMBIGUOUS);
}

TEST(TargetEstimator, SplitBlobOfOneObjectIsMerged)
{
  TargetEstimator est({"red", "", 0.5, ""});
  est.addFrame({obs("red", 0.450, -0.150), obs("red", 0.455, -0.150)});  // 5 mm 떨어진 두 조각
  const auto e = est.estimate(1, 0.005);
  ASSERT_EQ(e.status, EstimateStatus::OK);
  EXPECT_NEAR(e.mean.x, 0.4525, 1e-9);
}

TEST(TargetEstimator, FiltersByShapeAndConfidence)
{
  TargetEstimator est({"red", "triangle", 0.5, ""});
  est.addFrame({obs("red", 0.45, -0.15, 0.9, "cube")});
  EXPECT_EQ(est.framesMatched(), 0);
  TargetEstimator low({"red", "", 0.5, ""});
  low.addFrame({obs("red", 0.45, -0.15, 0.2)});
  EXPECT_EQ(low.framesMatched(), 0);
  TargetEstimator any({"RED", "", 0.5, ""});   // 대소문자 무시
  any.addFrame({obs("red", 0.45, -0.15)});
  EXPECT_EQ(any.framesMatched(), 1);
}

TEST(OccupantsNear, FindsOtherObjectInZoneButNotTheTarget)
{
  const Vec3 zone{0.60, 0.35, 0.75};
  const Vec3 target{0.45, 0.0, 0.77};
  const std::vector<Observation> objs{obs("green", 0.45, 0.0), obs("blue", 0.61, 0.34), obs("red", 0.30, 0.35)};
  const auto occ = occupantsNear(objs, zone, 0.06, target);
  ASSERT_EQ(occ.size(), 1u);
  EXPECT_EQ(occ[0].color, "blue");
  // 목표가 이미 그 구역에 있으면 목표 자신은 점유로 치지 않는다
  EXPECT_TRUE(occupantsNear({obs("green", 0.60, 0.35)}, zone, 0.06, Vec3{0.60, 0.35, 0.77}).empty());
}

TEST(TargetEstimator, SelectsByIdIgnoringColor)
{
  // 손짓으로 고른 물체: 같은 색이 둘 있어도 이름(id)으로 하나만 고른다 -> 모호하지 않음
  auto named = [](const std::string & id, const std::string & color, double x, double y) {
      Observation o = obs(color, x, y);
      o.id = id;
      return o;
    };
  TargetQuery q;
  q.id = "red_cube_2";
  TargetEstimator est(q);
  for (int i = 0; i < 5; ++i) {
    est.addFrame({named("red_cube_1", "red", 0.0, 0.45), named("red_cube_2", "red", 0.45, 0.0)});
  }
  const Estimate e = est.estimate(5, 0.005);
  ASSERT_EQ(e.status, EstimateStatus::OK);
  EXPECT_NEAR(e.mean.x, 0.45, 1e-9);
  EXPECT_EQ(e.id, "red_cube_2");
  TargetEstimator missing(TargetQuery{"", "", 0.5, "blue_cube"});
  missing.addFrame({named("red_cube_1", "red", 0.0, 0.45)});
  EXPECT_EQ(missing.framesMatched(), 0);
}
