#include "aris3_task_manager/geometry.hpp"

#include <cstdio>

namespace aris3_task_manager
{

std::vector<Waypoint> PickPlacePlan::waypoints() const
{
  return {{"above_object", above_object}, {"grasp", grasp}, {"above_zone", above_zone}, {"place", place}};
}

PickPlacePlan makePickPlacePlan(
  const Vec3 & object_base, const Vec3 & zone_surface_base, const GraspGeometry & g)
{
  const Vec3 up{0.0, 0.0, g.approach_height_m};
  PickPlacePlan p;
  // 집기: 물체 중심보다 grasp_offset 만큼 위
  p.grasp = object_base + Vec3{0.0, 0.0, g.grasp_offset_m};
  p.above_object = p.grasp + up;
  // 놓기: 물체 중심이 "테이블 + 물체 높이/2 + 여유" 에 오도록, 집을 때와 같은 grasp_offset 유지
  p.place = zone_surface_base +
    Vec3{0.0, 0.0, g.object_height_m / 2.0 + g.place_clearance_m + g.grasp_offset_m};
  p.above_zone = p.place + up;
  return p;
}

std::string checkWorkspace(const Vec3 & tcp, const Workspace & ws)
{
  char buf[160];
  const double r = normXY(tcp);
  if (r < ws.r_min_m || r > ws.r_max_m) {
    std::snprintf(buf, sizeof(buf), "수평 거리 %.0f mm 가 허용 범위 [%.0f, %.0f] mm 밖",
      r * 1000, ws.r_min_m * 1000, ws.r_max_m * 1000);
    return buf;
  }
  if (tcp.z < ws.z_min_m || tcp.z > ws.z_max_m) {
    std::snprintf(buf, sizeof(buf), "높이 %.0f mm 가 허용 범위 [%.0f, %.0f] mm 밖",
      tcp.z * 1000, ws.z_min_m * 1000, ws.z_max_m * 1000);
    return buf;
  }
  return "";
}

std::array<double, 6> flangePoseMmDeg(
  const Vec3 & tcp, const GraspGeometry & g, const std::array<double, 3> & zyz)
{
  return {tcp.x * 1000.0, tcp.y * 1000.0, (tcp.z + g.tcp_offset_m) * 1000.0, zyz[0], zyz[1], zyz[2]};
}

Vec3 tcpFromFlangeMm(const std::array<double, 6> & f, const GraspGeometry & g)
{
  return {f[0] / 1000.0, f[1] / 1000.0, f[2] / 1000.0 - g.tcp_offset_m};
}

}  // namespace aris3_task_manager
