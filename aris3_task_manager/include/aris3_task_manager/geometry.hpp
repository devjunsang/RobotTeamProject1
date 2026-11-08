// Pick & Place 경로점 계산과 작업 공간 검사 (순수 함수, 단위 시험 대상).
//
// 좌표계: 모두 "로봇 베이스(base_link)" 기준, 단위 m.
// 가정:   그리퍼는 항상 테이블을 수직으로 내려다본다(툴 z축 = 아래).
//         그래서 플랜지 위치 = TCP 위치 + (0, 0, tcp_offset).
#pragma once

#include <array>
#include <string>
#include <vector>

#include "aris3_task_manager/types.hpp"

namespace aris3_task_manager
{

struct GraspGeometry
{
  double tcp_offset_m{0.08};       // 플랜지 -> 집는 점(TCP) 거리 (툴 축 방향)
  double grasp_offset_m{0.005};    // 집을 때 TCP 를 물체 중심보다 이만큼 위에 둔다 (손끝이 테이블에 닿지 않게)
  double approach_height_m{0.10};  // 집기/놓기 전·후에 위에서 멈추는 높이
  double object_height_m{0.04};    // 물체 높이 (정육면체 한 변)
  double place_clearance_m{0.002}; // 놓을 때 물체 바닥과 테이블 사이 여유
};

// 로봇이 안전하게 갈 수 있는 TCP 범위 (베이스 기준)
struct Workspace
{
  double r_min_m{0.20};   // 베이스 축에서의 수평 거리 최소 (너무 가까우면 팔이 접힘)
  double r_max_m{0.75};   // 수평 거리 최대 (E0509 도달 거리 900 mm 에 여유)
  double z_min_m{0.005};  // TCP 최저 높이 (테이블 = 0)
  double z_max_m{0.60};
};

struct Waypoint
{
  std::string name;
  Vec3 tcp;  // base frame [m]
};

struct PickPlacePlan
{
  Vec3 above_object;  // 물체 위 (접근·들어 올리기 높이)
  Vec3 grasp;         // 집는 위치
  Vec3 above_zone;    // 구역 위
  Vec3 place;         // 놓는 위치
  std::vector<Waypoint> waypoints() const;  // 위 4개를 이름과 함께 (검사·출력용)
};

// object_base: 물체 중심(카메라가 준 값을 베이스 좌표로 바꾼 것)
// zone_surface_base: 목표 구역 중심의 "테이블 표면" 점
PickPlacePlan makePickPlacePlan(
  const Vec3 & object_base, const Vec3 & zone_surface_base, const GraspGeometry & g);

// 작업 공간 안이면 "", 밖이면 이유를 돌려준다.
std::string checkWorkspace(const Vec3 & tcp_base, const Workspace & ws);

// TCP 위치(베이스, m) -> 두산 move_line 이 받는 플랜지 자세 [x, y, z (mm), A, B, C (deg, ZYZ)]
std::array<double, 6> flangePoseMmDeg(
  const Vec3 & tcp_base, const GraspGeometry & g, const std::array<double, 3> & tool_zyz_deg);

// 플랜지 자세(mm) -> TCP 위치(베이스, m). flangePoseMmDeg 의 역.
Vec3 tcpFromFlangeMm(const std::array<double, 6> & flange_mm_deg, const GraspGeometry & g);

}  // namespace aris3_task_manager
