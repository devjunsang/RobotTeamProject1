// geometry.cpp 단위 시험: ROS 없이 계산만 확인한다.
#include <gtest/gtest.h>

#include "aris3_task_manager/geometry.hpp"

using namespace aris3_task_manager;  // NOLINT

TEST(Geometry, PlanHeightsFollowGraspGeometry)
{
  GraspGeometry g;  // tcp 0.08, grasp_offset 0.005, approach 0.10, object 0.04, clearance 0.002
  const Vec3 object{0.45, -0.15, 0.02};      // 물체 중심 (테이블 표면 z=0 기준 4 cm 큐브)
  const Vec3 zone{0.30, 0.35, 0.0};          // 구역 중심의 테이블 표면
  const auto p = makePickPlacePlan(object, zone, g);
  EXPECT_NEAR(p.grasp.z, 0.025, 1e-9);                 // 중심 + 5 mm
  EXPECT_NEAR(p.above_object.z, 0.125, 1e-9);          // + 100 mm
  EXPECT_NEAR(p.place.z, 0.0 + 0.02 + 0.002 + 0.005, 1e-9);
  EXPECT_NEAR(p.above_zone.z, p.place.z + 0.10, 1e-9);
  EXPECT_DOUBLE_EQ(p.grasp.x, 0.45);
  EXPECT_DOUBLE_EQ(p.place.y, 0.35);
  EXPECT_EQ(p.waypoints().size(), 4u);
}

TEST(Geometry, FlangePoseAddsTcpOffsetAndConvertsToMm)
{
  GraspGeometry g;
  const auto f = flangePoseMmDeg({0.45, -0.15, 0.025}, g, {0.0, 180.0, 0.0});
  EXPECT_NEAR(f[0], 450.0, 1e-9);
  EXPECT_NEAR(f[1], -150.0, 1e-9);
  EXPECT_NEAR(f[2], 105.0, 1e-9);   // (0.025 + 0.08) m
  EXPECT_DOUBLE_EQ(f[4], 180.0);
  const Vec3 back = tcpFromFlangeMm(f, g);
  EXPECT_NEAR(back.z, 0.025, 1e-9);
}

TEST(Geometry, WorkspaceRejectsTooFarTooCloseAndBelowTable)
{
  Workspace ws;  // r 0.20~0.75, z 0.005~0.60
  EXPECT_EQ(checkWorkspace({0.45, 0.0, 0.10}, ws), "");
  EXPECT_NE(checkWorkspace({1.20, 0.0, 0.10}, ws), "");   // 너무 멂
  EXPECT_NE(checkWorkspace({0.05, 0.0, 0.10}, ws), "");   // 베이스에 너무 가까움
  EXPECT_NE(checkWorkspace({0.45, 0.0, -0.30}, ws), "");  // 테이블 아래 (IK 는 풀리지만 위험)
}
