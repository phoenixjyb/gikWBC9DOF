#pragma once

#include <array>

#include "geometry_msgs/msg/twist.hpp"

namespace gik9dof_ros
{

struct UnifiedChassisParams
{
  double track {0.329};
  double wheel_speed_max {1.5};
  double vx_max {0.8};
  double wz_max {2.5};
  double yaw_kp {2.0};
  double yaw_kff {0.2};
};

class UnifiedChassisController
{
public:
  UnifiedChassisController();
  explicit UnifiedChassisController(const UnifiedChassisParams & params);

  void set_params(const UnifiedChassisParams & params);
  void reset();

  geometry_msgs::msg::Twist compute_stage_a() const;
  geometry_msgs::msg::Twist compute_stage_b(double v, double w) const;
  geometry_msgs::msg::Twist compute_stage_c(double ref_x,
                                            double ref_y,
                                            double ref_theta,
                                            double sample_time,
                                            const std::array<double, 3> & est_pose);

private:
  double clamp_yaw(double vx, double wz) const;
  geometry_msgs::msg::Twist clamp_command(double vx, double wz) const;

  UnifiedChassisParams params_ {};
  bool has_prev_ {false};
  double prev_x_ {0.0};
  double prev_y_ {0.0};
  double prev_theta_ {0.0};
};

}  // namespace gik9dof_ros
