#include "aris3_task_manager/moveit_robot_client.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <vector>

#include <Eigen/Geometry>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace aris3_task_manager
{

using moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr double kDeg = M_PI / 180.0;
// 궤적 실행이 "끝남" 으로 돌아와도 컨트롤러 목표 허용 오차(관절당 0.02 rad) 안에서 아직 수렴 중일 수 있다.
// 도착 판정 전에 이만큼 기다리며 다시 잰다 (2026-11-08: 물체를 든 뒤 5.2 mm 남은 채 판정되어 실패).
constexpr double kSettleTimeS = 1.5;
constexpr double kSettlePollS = 0.1;

// 두산 ZYZ 오일러 [deg] -> 회전 (R = Rz(a) * Ry(b) * Rz(c))
Eigen::Matrix3d zyzToMatrix(double a, double b, double c)
{
  return (Eigen::AngleAxisd(a * kDeg, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(b * kDeg, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(c * kDeg, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

// 회전 -> ZYZ [deg]. b = 0 또는 180 근처(특이점)이면 c = 0 으로 둔다.
std::array<double, 3> matrixToZyz(const Eigen::Matrix3d & r)
{
  const double sb = std::hypot(r(2, 0), r(2, 1));
  const double b = std::atan2(sb, r(2, 2));
  double a, c;
  if (sb > 1e-6) {
    a = std::atan2(r(1, 2), r(0, 2));
    c = std::atan2(r(2, 1), -r(2, 0));
  } else {
    c = 0.0;
    a = (r(2, 2) > 0) ? std::atan2(r(1, 0), r(0, 0)) : std::atan2(-r(1, 0), -r(0, 0));
  }
  return {a / kDeg, b / kDeg, c / kDeg};
}

Eigen::Isometry3d poseFromMmDeg(const std::array<double, 6> & p)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = zyzToMatrix(p[3], p[4], p[5]);
  t.translation() = Eigen::Vector3d(p[0], p[1], p[2]) / 1000.0;
  return t;
}

std::string errName(const moveit::core::MoveItErrorCode & code)
{
  return moveit::core::errorCodeToString(code) + " (" + std::to_string(code.val) + ")";
}
}  // namespace

MoveItRobotClient::MoveItRobotClient(rclcpp::Node * parent, Options options)
: opt_(std::move(options))
{
  // MoveIt 이 읽을 파라미터를 Task Manager 노드에서 받아 내부 노드로 넘긴다.
  const std::string kin = "robot_description_kinematics." + opt_.group + ".";
  auto str = [parent](const std::string & name) {
      return parent->has_parameter(name) ? parent->get_parameter(name).as_string() :
             parent->declare_parameter<std::string>(name, "");
    };
  auto dbl = [parent](const std::string & name, double def) {
      return parent->has_parameter(name) ? parent->get_parameter(name).as_double() :
             parent->declare_parameter<double>(name, def);
    };
  const std::string urdf = str("robot_description");
  const std::string srdf = str("robot_description_semantic");
  std::string solver = str(kin + "kinematics_solver");
  if (solver.empty()) {solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";}

  rclcpp::NodeOptions no;
  no.use_global_arguments(false);   // Task Manager 의 이름 바꾸기(remap)가 내부 노드에 번지지 않게
  no.arguments({"--ros-args", "-r", "joint_states:=" + opt_.joint_states_topic});
  no.parameter_overrides({
      {"robot_description", urdf},
      {"robot_description_semantic", srdf},
      {kin + "kinematics_solver", solver},
      {kin + "kinematics_solver_search_resolution", dbl(kin + "kinematics_solver_search_resolution", 0.005)},
      {kin + "kinematics_solver_timeout", dbl(kin + "kinematics_solver_timeout", 0.05)},
      {"use_sim_time", parent->get_parameter("use_sim_time").as_bool()},
    });
  no.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>(std::string(parent->get_name()) + "_moveit", no);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  if (urdf.empty() || srdf.empty()) {
    connect_error_ = "robot_description / robot_description_semantic 파라미터가 없음 "
      "(task_manager_aris3_sim.launch.py 로 실행했는가?)";
    return;
  }
  // move_group 액션 서버가 뜰 때까지 기다리는 동안 Task Manager 는 정상적으로 시작한다.
  connect_thread_ = std::thread([this]() {
        while (!shutting_down_ && rclcpp::ok()) {
          try {
            auto m = std::make_shared<MoveGroupInterface>(
              node_, opt_.group, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(5.0));
            m->setPlanningTime(opt_.planning_time_s);
            m->setMaxVelocityScalingFactor(opt_.velocity_scaling);
            m->setMaxAccelerationScalingFactor(opt_.acceleration_scaling);
            m->setPoseReferenceFrame(opt_.base_frame);
            m->setEndEffectorLink(opt_.tip_link);
            std::lock_guard<std::mutex> lk(mutex_);
            mgi_ = m;
            connect_error_.clear();
            RCLCPP_INFO(node_->get_logger(), "move_group 연결됨 (그룹 %s, 기준 %s, 끝 %s)",
            opt_.group.c_str(), opt_.base_frame.c_str(), opt_.tip_link.c_str());
            return;
          } catch (const std::exception & e) {
            {
              std::lock_guard<std::mutex> lk(mutex_);
              connect_error_ = std::string("move_group 연결 대기: ") + e.what();
            }
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "%s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
        }
      });
}

MoveItRobotClient::~MoveItRobotClient()
{
  shutting_down_ = true;
  if (connect_thread_.joinable()) {connect_thread_.join();}
  {
    std::lock_guard<std::mutex> lk(mutex_);
    mgi_.reset();
  }
  executor_->cancel();
  if (spin_thread_.joinable()) {spin_thread_.join();}
}

std::shared_ptr<MoveGroupInterface> MoveItRobotClient::mgi() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return mgi_;
}

bool MoveItRobotClient::servicesReady(std::string * missing) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (mgi_) {return true;}
  if (missing) {*missing = connect_error_;}
  return false;
}

int MoveItRobotClient::robotState()
{
  auto m = mgi();
  if (!m) {return -1;}
  // 관절 상태가 들어오고 있어야 명령을 받을 수 있는 상태(1)로 본다.
  return m->getCurrentState(2.0) ? 1 : -1;
}

std::optional<std::array<double, 6>> MoveItRobotClient::currentPose()
{
  auto m = mgi();
  if (!m) {return std::nullopt;}
  auto st = m->getCurrentState(2.0);
  if (!st) {return std::nullopt;}
  const Eigen::Isometry3d rel = st->getGlobalLinkTransform(opt_.base_frame).inverse() *
    st->getGlobalLinkTransform(opt_.tip_link);
  const auto zyz = matrixToZyz(rel.linear());
  const Eigen::Vector3d p = rel.translation() * 1000.0;
  return std::array<double, 6>{p.x(), p.y(), p.z(), zyz[0], zyz[1], zyz[2]};
}

bool MoveItRobotClient::stop()
{
  auto m = mgi();
  if (!m) {return false;}
  m->stop();
  return true;
}

// 계획 -> (중단·시간 초과를 감시하면서) 실행. 실패하면 TaskError.
template<class Setup>
void MoveItRobotClient::planAndExecute(const std::string & label, const AbortCheck & abort, Setup setup)
{
  auto m = mgi();
  if (!m) {
    throw TaskError(ErrorCode::NOT_READY, label + ": MoveIt 연결 안 됨");
  }
  m->clearPoseTargets();
  m->setStartStateToCurrentState();
  setup(*m);
  MoveGroupInterface::Plan plan;
  const auto planned = m->plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": 경로 계획 실패 " + errName(planned));
  }
  if (abort) {
    const ErrorCode why = abort();
    if (why != ErrorCode::NONE) {
      throw TaskError(why, label + ": 실행 전 중단 (" + errorName(why) + ")");
    }
  }
  auto running = std::async(std::launch::async, [m, &plan]() {return m->execute(plan);});
  const auto t0 = std::chrono::steady_clock::now();
  while (running.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    const ErrorCode why = abort ? abort() : ErrorCode::NONE;
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (why != ErrorCode::NONE || elapsed > opt_.motion_timeout_s) {
      m->stop();
      running.wait();
      if (why != ErrorCode::NONE) {
        throw TaskError(why, label + ": 이동 중 중단 (" + errorName(why) + ")");
      }
      throw TaskError(ErrorCode::TIMEOUT, label + ": 이동 시간 초과");
    }
  }
  const auto executed = running.get();
  if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
    throw TaskError(ErrorCode::MOTION_FAILED, label + ": 실행 실패 " + errName(executed));
  }
}

void MoveItRobotClient::moveJoints(const std::array<double, 6> & deg, const std::string & label,
  const AbortCheck & abort)
{
  std::vector<double> target(6);
  for (int i = 0; i < 6; ++i) {target[i] = deg[i] * kDeg;}
  planAndExecute(label, abort, [&](MoveGroupInterface & m) {
      m.setPlanningPipelineId("ompl");
      m.setPlannerId("");
      m.setJointValueTarget(target);
    });
  // 도착 확인: 실제 관절각과 목표의 차이 (수렴할 시간을 준다)
  auto m = mgi();
  size_t worst_joint = 0;
  double worst = 0.0;
  for (int tries = 0; ; ++tries) {
    auto st = m->getCurrentState(2.0);
    if (!st) {
      throw TaskError(ErrorCode::MOTION_FAILED, label + ": 도착 확인 실패 (관절 상태 없음)");
    }
    std::vector<double> now;
    st->copyJointGroupPositions(opt_.group, now);
    worst = 0.0;
    for (size_t i = 0; i < 6 && i < now.size(); ++i) {
      const double diff = std::abs(now[i] - target[i]) / kDeg;
      if (diff > worst) {worst = diff; worst_joint = i;}
    }
    if (worst <= opt_.joint_tolerance_deg) {return;}
    if (tries * kSettlePollS >= kSettleTimeS) {break;}
    std::this_thread::sleep_for(std::chrono::duration<double>(kSettlePollS));
  }
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s: joint_%zu 목표에 도착하지 못함 (차이 %.2f deg)", label.c_str(),
    worst_joint + 1, worst);
  throw TaskError(ErrorCode::MOTION_FAILED, buf);
}

void MoveItRobotClient::moveLinear(const std::array<double, 6> & pose_mm_deg, const std::string & label,
  const AbortCheck & abort)
{
  const geometry_msgs::msg::Pose target = tf2::toMsg(poseFromMmDeg(pose_mm_deg));
  planAndExecute(label, abort, [&](MoveGroupInterface & m) {
      // Pilz LIN: 공구 끝이 직선으로 움직인다 (물체에 수직으로 접근·하강)
      m.setPlanningPipelineId("pilz_industrial_motion_planner");
      m.setPlannerId("LIN");
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = opt_.base_frame;
      ps.pose = target;
      m.setPoseTarget(ps, opt_.tip_link);
    });
  waitArrival(pose_mm_deg, label);
}

void MoveItRobotClient::moveFree(const std::array<double, 6> & pose_mm_deg, const std::string & label,
  const AbortCheck & abort)
{
  const geometry_msgs::msg::Pose target = tf2::toMsg(poseFromMmDeg(pose_mm_deg));
  planAndExecute(label, abort, [&](MoveGroupInterface & m) {
      m.setPlanningPipelineId("ompl");
      m.setPlannerId("");
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = opt_.base_frame;
      ps.pose = target;
      m.setPoseTarget(ps, opt_.tip_link);
    });
  waitArrival(pose_mm_deg, label);
}

// 도착 확인: 실제 플랜지 위치와 목표의 거리 (수렴할 시간을 준다)
void MoveItRobotClient::waitArrival(const std::array<double, 6> & pose_mm_deg, const std::string & label)
{
  double d = 0.0;
  for (int tries = 0; ; ++tries) {
    const auto now = currentPose();
    if (!now) {
      throw TaskError(ErrorCode::MOTION_FAILED, label + ": 도착 확인 실패 (현재 위치 모름)");
    }
    d = std::sqrt(std::pow(now->at(0) - pose_mm_deg[0], 2) +
        std::pow(now->at(1) - pose_mm_deg[1], 2) + std::pow(now->at(2) - pose_mm_deg[2], 2));
    if (d <= opt_.arrival_tolerance_mm || tries * kSettlePollS >= kSettleTimeS) {break;}
    std::this_thread::sleep_for(std::chrono::duration<double>(kSettlePollS));
  }
  if (d > opt_.arrival_tolerance_mm) {
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%s: 목표에 도착하지 못함 (남은 거리 %.1f mm)", label.c_str(), d);
    throw TaskError(ErrorCode::MOTION_FAILED, buf);
  }
}

double MoveItRobotClient::reachabilityErrorMm(const std::array<double, 6> & pose_mm_deg)
{
  auto m = mgi();
  if (!m) {return -1.0;}
  auto st = m->getCurrentState(2.0);
  if (!st) {return -1.0;}
  const auto * jmg = st->getJointModelGroup(opt_.group);
  const Eigen::Isometry3d target_world =
    st->getGlobalLinkTransform(opt_.base_frame) * poseFromMmDeg(pose_mm_deg);
  moveit::core::RobotState probe(*st);
  if (!probe.setFromIK(jmg, target_world, opt_.tip_link, 0.1)) {
    return 1e6;   // IK 해 없음 = 도달 불가
  }
  probe.update();
  return (probe.getGlobalLinkTransform(opt_.tip_link).translation() - target_world.translation()).norm() * 1000.0;
}

}  // namespace aris3_task_manager
