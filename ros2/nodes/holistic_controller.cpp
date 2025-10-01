#include <chrono>
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

    double rate_hz = this->get_parameter("control_rate").as_double();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = this->create_wall_timer(period, std::bind(&HolisticController::controlStep, this));
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
        continue;
      const auto & tf = point.transforms.front();
      std::array<double, 16> pose = quaternionToMatrix(tf.rotation);
      pose[3] = tf.translation.x;
      pose[7] = tf.translation.y;
      pose[11] = tf.translation.z;
      queued_poses_.push_back(pose);
    }
    have_target_ = !queued_poses_.empty();
    next_pose_index_ = 0;
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
      return;

    SolverOptions options;
    options.distance_lower_bound = this->get_parameter("distance_lower_bound").as_double();
    options.distance_weight = this->get_parameter("distance_weight").as_double();

    const auto & target_pose = queued_poses_[std::min(next_pose_index_, queued_poses_.size() - 1)];
    auto result = solveHolisticStep(joint_positions_, target_pose, options);

    publishJointCommand(result.joint_positions);
    publishBaseCommand();

    ++next_pose_index_;
    if (next_pose_index_ >= queued_poses_.size())
      have_target_ = false;
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

  void publishBaseCommand()
  {
    geometry_msgs::msg::Twist cmd;
    base_command_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 9> joint_positions_{};
  bool have_joint_state_ {false};
  bool have_odometry_ {false};
  bool have_target_ {false};
  size_t next_pose_index_ {0};
  std::vector<std::array<double, 16>> queued_poses_;
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
