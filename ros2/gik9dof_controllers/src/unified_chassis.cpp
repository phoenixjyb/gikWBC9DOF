#include "gik9dof_controllers/unified_chassis.hpp"

#include <algorithm>
#include <cmath>

namespace
{

double wrap_to_pi(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  angle = std::fmod(angle + M_PI, kTwoPi);
  if (angle < 0.0)
  {
    angle += kTwoPi;
  }
  return angle - M_PI;
}

}  // namespace

namespace gik9dof_ros
{

UnifiedChassisController::UnifiedChassisController()
: UnifiedChassisController(UnifiedChassisParams{})
{
}

UnifiedChassisController::UnifiedChassisController(const UnifiedChassisParams & params)
{
  set_params(params);
  reset();
}

void UnifiedChassisController::set_params(const UnifiedChassisParams & params)
{
  params_ = params;
}

void UnifiedChassisController::reset()
{
  has_prev_ = false;
  prev_x_ = 0.0;
  prev_y_ = 0.0;
  prev_theta_ = 0.0;
}

geometry_msgs::msg::Twist UnifiedChassisController::compute_stage_a() const
{
  geometry_msgs::msg::Twist cmd;
  return cmd;
}

geometry_msgs::msg::Twist UnifiedChassisController::compute_stage_b(double v, double w) const
{
  const double vx = std::clamp(v, -params_.vx_max, params_.vx_max);
  const double wz = clamp_yaw(vx, w);
  return clamp_command(vx, wz);
}

geometry_msgs::msg::Twist UnifiedChassisController::compute_stage_c(
  double ref_x, double ref_y, double ref_theta, double sample_time,
  const std::array<double, 3> & est_pose)
{
  geometry_msgs::msg::Twist cmd;
  if (!has_prev_ || sample_time <= 0.0)
  {
    has_prev_ = true;
    prev_x_ = ref_x;
    prev_y_ = ref_y;
    prev_theta_ = ref_theta;
    return cmd;
  }

  const double dx = ref_x - prev_x_;
  const double dy = ref_y - prev_y_;
  const double vx_world = dx / sample_time;
  const double vy_world = dy / sample_time;
  const double yaw_delta = wrap_to_pi(ref_theta - prev_theta_);
  const double w_ref = yaw_delta / sample_time;

  const double yaw_est = est_pose[2];
  const double cos_yaw = std::cos(yaw_est);
  const double sin_yaw = std::sin(yaw_est);
  const double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
  const double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

  double heading_desired = 0.0;
  if (std::abs(vx_world) > 1e-9 || std::abs(vy_world) > 1e-9)
  {
    heading_desired = std::atan2(vy_world, vx_world);
  }
  const double heading_error = wrap_to_pi(heading_desired - yaw_est);

  double vx_cmd = std::copysign(std::min(std::abs(vx_body), params_.vx_max), vx_body);
  vx_cmd *= std::cos(heading_error);
  double wz_cmd = params_.yaw_kp * heading_error + params_.yaw_kff * w_ref;
  wz_cmd = clamp_yaw(vx_cmd, wz_cmd);

  prev_x_ = ref_x;
  prev_y_ = ref_y;
  prev_theta_ = ref_theta;

  return clamp_command(vx_cmd, wz_cmd);
}

double UnifiedChassisController::clamp_yaw(double vx, double wz) const
{
  const double Wwheel = (params_.track <= 0.0)
    ? 0.0
    : (2.0 * std::max(0.0, params_.wheel_speed_max - std::abs(vx)) / params_.track);
  const double cap = std::min(params_.wz_max, Wwheel);
  return std::clamp(wz, -cap, cap);
}

geometry_msgs::msg::Twist UnifiedChassisController::clamp_command(double vx, double wz) const
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(vx, -params_.vx_max, params_.vx_max);
  cmd.angular.z = std::clamp(wz, -params_.wz_max, params_.wz_max);
  return cmd;
}

}  // namespace gik9dof_ros
