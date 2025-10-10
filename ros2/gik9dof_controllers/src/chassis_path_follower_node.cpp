// chassis_path_follower_node.cpp
// ROS2 wrapper node for MATLAB Coder generated chassis path follower
// Supports 3 controller modes: 0 (differentiation), 1 (heading), 2 (pure pursuit)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include "ChassisPathFollower.h"
#include "chassisPathFollowerCodegen_types.h"

namespace gik9dof {

class ChassisPathFollowerNode : public rclcpp::Node
{
public:
  ChassisPathFollowerNode()
  : Node("chassis_path_follower"),
    controller_initialized_(false),
    path_received_(false),
    pose_received_(false)
  {
    // Declare parameters
    declare_parameters();
    
    // Initialize controller with default parameters
    initialize_controller();
    
    // Create publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::Bool>(
      "path_following_active", 10);
    
    // Create subscribers
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", 10,
      std::bind(&ChassisPathFollowerNode::path_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ChassisPathFollowerNode::odom_callback, this, std::placeholders::_1));
    
    // Control timer (100 Hz)
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ChassisPathFollowerNode::control_loop, this));
    
    RCLCPP_INFO(get_logger(), "Chassis Path Follower Node initialized");
    RCLCPP_INFO(get_logger(), "Controller Mode: %d (0=diff, 1=heading, 2=pure pursuit)",
                params_.ControllerMode);
  }

private:
  void declare_parameters()
  {
    // Controller mode
    declare_parameter("controller_mode", 2);  // Default: pure pursuit
    
    // Velocity limits
    declare_parameter("vx_max", 1.0);
    declare_parameter("vx_min", -0.5);
    declare_parameter("wz_max", 1.0);
    
    // Acceleration limits
    declare_parameter("accel_max", 0.5);
    declare_parameter("decel_max", 0.8);
    declare_parameter("jerk_limit", 2.0);
    
    // Lookahead parameters (Mode 1 & 2)
    declare_parameter("lookahead_base", 0.3);
    declare_parameter("lookahead_gain", 0.5);
    declare_parameter("lookahead_min", 0.2);
    declare_parameter("lookahead_max", 2.0);
    
    // Pure pursuit parameters (Mode 2)
    declare_parameter("kappa_threshold", 0.5);
    declare_parameter("vx_reduction", 0.3);
    
    // Heading controller gains (Mode 1)
    declare_parameter("heading_kp", 2.0);
    declare_parameter("feedforward_gain", 0.8);
    
    // Chassis parameters
    declare_parameter("track_width", 0.5);
    declare_parameter("wheel_radius", 0.1);
    declare_parameter("wheel_speed_max", 2.0);
    
    // Goal tolerance
    declare_parameter("goal_tolerance", 0.1);
    
    // Reverse control
    declare_parameter("reverse_enabled", false);
  }
  
  void initialize_controller()
  {
    // Get parameters from ROS2 parameter server
    params_.ControllerMode = get_parameter("controller_mode").as_int();
    
    params_.VxMax = get_parameter("vx_max").as_double();
    params_.VxMin = get_parameter("vx_min").as_double();
    params_.WzMax = get_parameter("wz_max").as_double();
    
    params_.AccelMax = get_parameter("accel_max").as_double();
    params_.DecelMax = get_parameter("decel_max").as_double();
    params_.JerkLimit = get_parameter("jerk_limit").as_double();
    
    params_.LookaheadBase = get_parameter("lookahead_base").as_double();
    params_.LookaheadGain = get_parameter("lookahead_gain").as_double();
    params_.LookaheadMin = get_parameter("lookahead_min").as_double();
    params_.LookaheadMax = get_parameter("lookahead_max").as_double();
    
    params_.KappaThreshold = get_parameter("kappa_threshold").as_double();
    params_.VxReduction = get_parameter("vx_reduction").as_double();
    
    params_.HeadingKp = get_parameter("heading_kp").as_double();
    params_.FeedforwardGain = get_parameter("feedforward_gain").as_double();
    
    params_.Chassis.TrackWidth = get_parameter("track_width").as_double();
    params_.Chassis.WheelRadius = get_parameter("wheel_radius").as_double();
    params_.Chassis.WheelSpeedMax = get_parameter("wheel_speed_max").as_double();
    
    params_.GoalTolerance = get_parameter("goal_tolerance").as_double();
    params_.ReverseEnabled = get_parameter("reverse_enabled").as_bool();
    
    // Initialize PathInfo arrays (will be populated from path message)
    params_.PathInfo_X_size[0] = 0;
    params_.PathInfo_X_size[1] = 1;
    params_.PathInfo_Y_size[0] = 0;
    params_.PathInfo_Y_size[1] = 1;
    params_.PathInfo_Theta_size[0] = 0;
    params_.PathInfo_Theta_size[1] = 1;
    params_.PathInfo_Curvature_size[0] = 0;
    params_.PathInfo_Curvature_size[1] = 1;
    params_.PathInfo_DistanceRemaining_size[0] = 0;
    params_.PathInfo_DistanceRemaining_size[1] = 1;
    
    // Initialize state
    state_.LastVelocity = 0.0;
    state_.LastAcceleration = 0.0;
    state_.PreviousPose[0] = 0.0;
    state_.PreviousPose[1] = 0.0;
    state_.PreviousPose[2] = 0.0;
    state_.PathIndex = 0;
    state_.DistanceTraveled = 0.0;
    state_.TimeElapsed = 0.0;
    state_.IsActive = false;
    state_.GoalReached = false;
    
    controller_initialized_ = true;
    
    RCLCPP_INFO(get_logger(), "Controller initialized with mode %d", params_.ControllerMode);
  }
  
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path");
      return;
    }
    
    // Limit to 500 points (max in codegen)
    size_t num_points = std::min(msg->poses.size(), size_t(500));
    
    // Extract path data
    for (size_t i = 0; i < num_points; ++i) {
      params_.PathInfo_X_data[i] = msg->poses[i].pose.position.x;
      params_.PathInfo_Y_data[i] = msg->poses[i].pose.position.y;
      
      // Extract yaw from quaternion
      auto& q = msg->poses[i].pose.orientation;
      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      params_.PathInfo_Theta_data[i] = std::atan2(siny_cosp, cosy_cosp);
      
      // Curvature and distance remaining (placeholder - should be computed)
      params_.PathInfo_Curvature_data[i] = 0.0;
      params_.PathInfo_DistanceRemaining_data[i] = 0.0;
    }
    
    // Update array sizes
    params_.PathInfo_X_size[0] = num_points;
    params_.PathInfo_Y_size[0] = num_points;
    params_.PathInfo_Theta_size[0] = num_points;
    params_.PathInfo_Curvature_size[0] = num_points;
    params_.PathInfo_DistanceRemaining_size[0] = num_points;
    
    // Compute distance remaining (backwards from goal)
    if (num_points > 0) {
      params_.PathInfo_DistanceRemaining_data[num_points - 1] = 0.0;
      for (int i = num_points - 2; i >= 0; --i) {
        double dx = params_.PathInfo_X_data[i + 1] - params_.PathInfo_X_data[i];
        double dy = params_.PathInfo_Y_data[i + 1] - params_.PathInfo_Y_data[i];
        double seg_dist = std::sqrt(dx * dx + dy * dy);
        params_.PathInfo_DistanceRemaining_data[i] = 
          params_.PathInfo_DistanceRemaining_data[i + 1] + seg_dist;
      }
    }
    
    // Compute curvature (3-point method)
    if (num_points >= 3) {
      for (size_t i = 1; i < num_points - 1; ++i) {
        double dx1 = params_.PathInfo_X_data[i] - params_.PathInfo_X_data[i - 1];
        double dy1 = params_.PathInfo_Y_data[i] - params_.PathInfo_Y_data[i - 1];
        double dx2 = params_.PathInfo_X_data[i + 1] - params_.PathInfo_X_data[i];
        double dy2 = params_.PathInfo_Y_data[i + 1] - params_.PathInfo_Y_data[i];
        
        double dtheta = std::atan2(dy2, dx2) - std::atan2(dy1, dx1);
        // Wrap to [-pi, pi]
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
        
        double ds = std::sqrt(dx2 * dx2 + dy2 * dy2);
        params_.PathInfo_Curvature_data[i] = (ds > 1e-6) ? (dtheta / ds) : 0.0;
      }
      // Endpoints use neighbor values
      params_.PathInfo_Curvature_data[0] = params_.PathInfo_Curvature_data[1];
      params_.PathInfo_Curvature_data[num_points - 1] = 
        params_.PathInfo_Curvature_data[num_points - 2];
    }
    
    path_received_ = true;
    state_.IsActive = true;
    state_.GoalReached = false;
    
    RCLCPP_INFO(get_logger(), "Received path with %zu points", num_points);
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_[0] = msg->pose.pose.position.x;
    current_pose_[1] = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    auto& q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_pose_[2] = std::atan2(siny_cosp, cosy_cosp);
    
    pose_received_ = true;
  }
  
  void control_loop()
  {
    if (!controller_initialized_ || !path_received_ || !pose_received_) {
      return;
    }
    
    if (!state_.IsActive || state_.GoalReached) {
      // Publish zero velocity when inactive
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      return;
    }
    
    // Call MATLAB Coder generated function
    double dt = 0.01;  // 100 Hz control rate
    double vx_cmd, wz_cmd;
    struct0_T status;
    
    gik9dof::ChassisPathFollower controller;
    controller.chassisPathFollowerCodegen(
      current_pose_, dt, &state_, &params_,
      &vx_cmd, &wz_cmd, &state_, &status);
    
    // Publish velocity command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx_cmd;
    cmd.angular.z = wz_cmd;
    cmd_vel_pub_->publish(cmd);
    
    // Publish status
    std_msgs::msg::Bool active_msg;
    active_msg.data = !status.isFinished;
    status_pub_->publish(active_msg);
    
    // Update goal reached flag
    if (status.isFinished) {
      state_.GoalReached = true;
      state_.IsActive = false;
      RCLCPP_INFO(get_logger(), "Goal reached!");
    }
  }
  
  // Member variables
  ChassisPathParams params_;
  ChassisPathState state_;
  double current_pose_[3];
  
  bool controller_initialized_;
  bool path_received_;
  bool pose_received_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace gik9dof

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gik9dof::ChassisPathFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
