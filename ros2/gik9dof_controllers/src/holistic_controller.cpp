#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "gik_solver_wrapper.hpp"

namespace
{

double wrapToPi(double angle)
{
  constexpr double kTwoPi = 6.28318530717958647693;
  angle = std::fmod(angle, kTwoPi);
  if (angle > 3.14159265358979323846)
  {
    angle -= kTwoPi;
  }
  else if (angle < -3.14159265358979323846)
  {
    angle += kTwoPi;
  }
  return angle;
}

double unwrapAngle(double previous, double current)
{
  return previous + wrapToPi(current - previous);
}

struct PoseSample
{
  rclcpp::Time stamp;
  double x {0.0};
  double theta {0.0};
};

template<typename Accessor>
double backwardDerivative(const std::deque<PoseSample> & history, Accessor accessor)
{
  const std::size_t n = history.size();
  if (n < 2)
  {
    return 0.0;
  }

  const double h = (history[0].stamp - history[1].stamp).seconds();
  if (h <= 0.0)
  {
    return 0.0;
  }

  const double a0 = accessor(history[0]);
  const double a1 = accessor(history[1]);
  if (n >= 5)
  {
    const double a2 = accessor(history[2]);
    const double a3 = accessor(history[3]);
    const double a4 = accessor(history[4]);
    return (25.0 * a0 - 48.0 * a1 + 36.0 * a2 - 16.0 * a3 + 3.0 * a4) / (12.0 * h);
  }
  if (n == 4)
  {
    const double a2 = accessor(history[2]);
    const double a3 = accessor(history[3]);
    return (55.0 * a0 - 59.0 * a1 + 37.0 * a2 - 9.0 * a3) / (24.0 * h);
  }
  if (n == 3)
  {
    const double a2 = accessor(history[2]);
    return (3.0 * a0 - 4.0 * a1 + a2) / (2.0 * h);
  }
  return (a0 - a1) / h;
}

}  // namespace

namespace gik9dof_ros
{

class HolisticController : public rclcpp::Node
{
public:
  HolisticController()
  : Node("holistic_controller")
  {
    this->declare_parameter<double>("control_rate", 100.0);
    this->declare_parameter<double>("distance_lower_bound", 0.2);
    this->declare_parameter<double>("distance_weight", 0.5);
    this->declare_parameter<std::string>("joint_command_topic", "/control/joint_commands");
    this->declare_parameter<std::string>("base_command_topic", "/control/base_commands");
    this->declare_parameter<bool>("log_solver_timing", false);
    this->declare_parameter<double>("base_command_rate", 50.0);
    this->declare_parameter<double>("max_base_speed", 1.5);
    this->declare_parameter<double>("max_base_yaw", 3.5);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot_state/joint_states", 10,
      std::bind(&HolisticController::onJointState, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot_state/odometry", 10,
      std::bind(&HolisticController::onOdometry, this, std::placeholders::_1));

    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      "/target/end_effector_trajectory", 10,
      std::bind(&HolisticController::onTrajectory, this, std::placeholders::_1));

    obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/obstacles/floor_discs", 10,
      std::bind(&HolisticController::onObstacles, this, std::placeholders::_1));

    joint_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("joint_command_topic").as_string(), 10);
    base_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("base_command_topic").as_string(), 10);

    double control_rate_hz = this->get_parameter("control_rate").as_double();
    if (control_rate_hz <= 0.0)
    {
      control_rate_hz = 10.0;
    }
    const auto solver_period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = this->create_wall_timer(solver_period, std::bind(&HolisticController::controlStep, this));

    double base_command_rate = this->get_parameter("base_command_rate").as_double();
    if (base_command_rate <= 0.0)
    {
      base_command_rate = 50.0;
    }
    max_base_speed_ = std::max(0.0, this->get_parameter("max_base_speed").as_double());
    max_base_yaw_ = std::max(0.0, this->get_parameter("max_base_yaw").as_double());
    const auto base_period = std::chrono::duration<double>(1.0 / base_command_rate);
    base_command_timer_ = this->create_wall_timer(
      base_period, std::bind(&HolisticController::publishBaseVelocity, this));
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < joint_positions_.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "JointState has insufficient length");
      return;
    }
    for (size_t i = 0; i < joint_positions_.size(); ++i)
    {
      joint_positions_[i] = msg->position[i];
    }
    have_joint_state_ = true;
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
    have_odometry_ = true;
  }

  void onTrajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
  {
    queued_poses_.clear();
    for (const auto & point : msg->points)
    {
      if (point.transforms.empty())
      {
        continue;
      }
      const auto & tf = point.transforms.front();
      std::array<double, 16> pose = quaternionToMatrix(tf.rotation);
      pose[3] = tf.translation.x;
      pose[7] = tf.translation.y;
      pose[11] = tf.translation.z;
      queued_poses_.push_back(pose);
    }
    have_target_ = !queued_poses_.empty();
    next_pose_index_ = 0;
    resetBaseVelocityEstimate();
    RCLCPP_INFO(get_logger(), "Received trajectory with %zu waypoints", queued_poses_.size());
  }

  std::array<double, 16> quaternionToMatrix(const geometry_msgs::msg::Quaternion & q_msg) const
  {
    const double w = q_msg.w;
    const double x = q_msg.x;
    const double y = q_msg.y;
    const double z = q_msg.z;
    std::array<double, 16> m = gik9dof_ros::identityPose();
    m[0] = 1 - 2 * (y * y + z * z);
    m[1] = 2 * (x * y - z * w);
    m[2] = 2 * (x * z + y * w);
    m[4] = 2 * (x * y + z * w);
    m[5] = 1 - 2 * (x * x + z * z);
    m[6] = 2 * (y * z - x * w);
    m[8] = 2 * (x * z - y * w);
    m[9] = 2 * (y * z + x * w);
    m[10] = 1 - 2 * (x * x + y * y);
    return m;
  }

  void onObstacles(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    (void)msg;
  }

  void controlStep()
  {
    if (!have_joint_state_ || !have_target_)
    {
      return;
    }

    SolverOptions options;
    options.distance_lower_bound = this->get_parameter("distance_lower_bound").as_double();
    options.distance_weight = this->get_parameter("distance_weight").as_double();

    const bool log_timing = this->get_parameter("log_solver_timing").as_bool();
    const auto & target_pose = queued_poses_[std::min(next_pose_index_, queued_poses_.size() - 1)];
    const auto start = std::chrono::steady_clock::now();
    auto result = solveHolisticStep(joint_positions_, target_pose, options);
    if (log_timing)
    {
      const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "solveHolisticStep latency: %.3f ms (%.0f us)",
        static_cast<double>(elapsed_us) / 1000.0, static_cast<double>(elapsed_us));
    }

    publishJointCommand(result.joint_positions);
    recordBasePose(result.joint_positions);

    ++next_pose_index_;
    if (next_pose_index_ >= queued_poses_.size())
    {
      have_target_ = false;
      resetBaseVelocityEstimate();
    }
  }

  void publishJointCommand(const std::array<double, 9> & joints)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {
      "joint_x", "joint_y", "joint_theta",
      "left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
      "left_arm_joint4", "left_arm_joint5", "left_arm_joint6"
    };
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(joints.begin(), joints.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(0.01);
    msg.points.push_back(pt);
    joint_command_pub_->publish(msg);
  }

  void publishBaseVelocity()
  {
    geometry_msgs::msg::Twist cmd;
    if (have_target_ && have_velocity_estimate_)
    {
      cmd = last_base_cmd_;
    }
    base_command_pub_->publish(cmd);
  }

  void recordBasePose(const std::array<double, 9> & joints)
  {
    PoseSample sample;
    sample.stamp = this->now();
    sample.x = joints[0];
    sample.theta = joints[2];
    if (!base_pose_history_.empty())
    {
      sample.theta = unwrapAngle(base_pose_history_.front().theta, sample.theta);
    }

    base_pose_history_.push_front(sample);
    if (base_pose_history_.size() > 5)
    {
      base_pose_history_.pop_back();
    }

    if (base_pose_history_.size() >= 2)
    {
      const double vx = backwardDerivative(base_pose_history_, [](const PoseSample & s) {
        return s.x;
      });
      const double wz = backwardDerivative(base_pose_history_, [](const PoseSample & s) {
        return s.theta;
      });

      geometry_msgs::msg::Twist estimate;
      estimate.linear.x = std::clamp(vx, -max_base_speed_, max_base_speed_);
      estimate.angular.z = std::clamp(wz, -max_base_yaw_, max_base_yaw_);
      last_base_cmd_ = estimate;
      have_velocity_estimate_ = true;
    }
  }

  void resetBaseVelocityEstimate()
  {
    base_pose_history_.clear();
    have_velocity_estimate_ = false;
    last_base_cmd_ = geometry_msgs::msg::Twist();
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr base_command_timer_;

  std::array<double, 9> joint_positions_{};
  bool have_joint_state_ {false};
  bool have_odometry_ {false};
  bool have_target_ {false};
  size_t next_pose_index_ {0};
  std::vector<std::array<double, 16>> queued_poses_;

  std::deque<PoseSample> base_pose_history_;
  geometry_msgs::msg::Twist last_base_cmd_;
  bool have_velocity_estimate_ {false};
  double max_base_speed_ {1.5};
  double max_base_yaw_ {3.5};
};

}  // namespace gik9dof_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gik9dof_ros::HolisticController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
