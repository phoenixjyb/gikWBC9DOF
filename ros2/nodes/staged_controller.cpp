#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "gik_solver_wrapper.hpp"

namespace gik9dof_ros
{

class StagedController : public rclcpp::Node
{
public:
  StagedController()
  : Node("staged_controller")
  {
    this->declare_parameter<double>("control_rate", 50.0);
    this->declare_parameter<int>("stage_a_samples", 40);
    this->declare_parameter<int>("stage_b_samples", 40);
    this->declare_parameter<double>("distance_lower_bound", 0.2);
    this->declare_parameter<double>("distance_weight", 0.5);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot_state/joint_states", 10,
      std::bind(&StagedController::onJointState, this, std::placeholders::_1));

    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      "/target/end_effector_trajectory", 10,
      std::bind(&StagedController::onTrajectory, this, std::placeholders::_1));

    joint_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/control/joint_commands", 10);
    base_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/control/base_commands", 10);

    double rate_hz = this->get_parameter("control_rate").as_double();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&StagedController::controlStep, this));
  }

private:
  enum class Stage { Idle, A, B, C };

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < joint_positions_.size())
      return;
    std::copy_n(msg->position.begin(), joint_positions_.size(), joint_positions_.begin());
    have_joint_state_ = true;
  }

  void onTrajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
  {
    stages_[0].clear();
    stages_[1].clear();
    stages_[2].clear();

    if (msg->points.empty())
    {
      current_stage_ = Stage::Idle;
      return;
    }

    const int stage_a_samples = this->get_parameter("stage_a_samples").as_int();
    const int stage_b_samples = this->get_parameter("stage_b_samples").as_int();

    auto appendPoses = [&](std::deque<std::array<double,16>> & container, int count) {
      for (int k = 0; k < count && next_transform_index_ < static_cast<int>(msg->points.size()); ++k)
      {
        const auto & transforms = msg->points[next_transform_index_].transforms;
        if (transforms.empty())
        {
          ++next_transform_index_;
          continue;
        }
        container.push_back(quaternionToMatrix(transforms.front()));
        ++next_transform_index_;
      }
    };

    next_transform_index_ = 0;
    appendPoses(stages_[0], stage_a_samples);
    appendPoses(stages_[1], stage_b_samples);
    appendPoses(stages_[2], static_cast<int>(msg->points.size()) - next_transform_index_);

    current_stage_ = Stage::A;
    RCLCPP_INFO(get_logger(), "Staged controller queued: A=%zu, B=%zu, C=%zu", stages_[0].size(), stages_[1].size(), stages_[2].size());
  }

  void controlStep()
  {
    if (!have_joint_state_ || current_stage_ == Stage::Idle)
      return;

    SolverOptions options;
    options.distance_lower_bound = this->get_parameter("distance_lower_bound").as_double();
    options.distance_weight = this->get_parameter("distance_weight").as_double();

    bool arm_active = false;
    bool base_active = false;
    std::deque<std::array<double,16>> * queue = nullptr;

    switch (current_stage_)
    {
      case Stage::A:
        queue = &stages_[0];
        arm_active = true;
        base_active = false;
        break;
      case Stage::B:
        queue = &stages_[1];
        arm_active = false;
        base_active = true;
        break;
      case Stage::C:
        queue = &stages_[2];
        arm_active = true;
        base_active = true;
        break;
      default:
        return;
    }

    if (!queue || queue->empty())
    {
      advanceStage();
      return;
    }

    const auto target_pose = queue->front();
    auto result = solveStagedStep(joint_positions_, target_pose, options, arm_active, base_active);
    queue->pop_front();

    publishJointCommand(result.joint_positions);
    publishBaseCommand(base_active);

    if (queue->empty())
      advanceStage();
  }

  void advanceStage()
  {
    switch (current_stage_)
    {
      case Stage::A:
        current_stage_ = stages_[1].empty() ? Stage::C : Stage::B;
        break;
      case Stage::B:
        current_stage_ = Stage::C;
        break;
      case Stage::C:
      default:
        current_stage_ = Stage::Idle;
        break;
    }
  }

  std::array<double, 16> quaternionToMatrix(const geometry_msgs::msg::Transform & tf)
  {
    const auto & q_msg = tf.rotation;
    std::array<double, 16> m = gik9dof_ros::identityPose();
    const double w = q_msg.w;
    const double x = q_msg.x;
    const double y = q_msg.y;
    const double z = q_msg.z;
    m[0] = 1 - 2 * (y*y + z*z);
    m[1] = 2 * (x*y - z*w);
    m[2] = 2 * (x*z + y*w);
    m[4] = 2 * (x*y + z*w);
    m[5] = 1 - 2 * (x*x + z*z);
    m[6] = 2 * (y*z - x*w);
    m[8] = 2 * (x*z - y*w);
    m[9] = 2 * (y*z + x*w);
    m[10] = 1 - 2 * (x*x + y*y);
    m[3] = tf.translation.x;
    m[7] = tf.translation.y;
    m[11] = tf.translation.z;
    return m;
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
    pt.time_from_start = rclcpp::Duration::from_seconds(0.02);
    msg.points.push_back(pt);
    joint_command_pub_->publish(msg);
  }

  void publishBaseCommand(bool base_active)
  {
    geometry_msgs::msg::Twist cmd;
    if (!base_active)
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }
    base_command_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 9> joint_positions_{};
  bool have_joint_state_ {false};
  Stage current_stage_ {Stage::Idle};
  std::array<std::deque<std::array<double,16>>, 3> stages_;
  int next_transform_index_ {0};
};

}  // namespace gik9dof_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gik9dof_ros::StagedController>());
  rclcpp::shutdown();
  return 0;
}
