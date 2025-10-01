#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gik9dof_ros
{

class TrajectoryManager : public rclcpp::Node
{
public:
  TrajectoryManager()
  : Node("trajectory_manager")
  {
    this->declare_parameter<std::vector<double>>("waypoints", {1.65, 0.08, 0.86, 0.5, -0.5, 0.5, -0.5});
    this->declare_parameter<std::string>("trajectory_topic", "/target/end_effector_trajectory");
    this->declare_parameter<double>("publish_rate", 1.0);

    publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      this->get_parameter("trajectory_topic").as_string(), 10);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "publish_trajectory", std::bind(&TrajectoryManager::handleTrigger, this,
      std::placeholders::_1, std::placeholders::_2));

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      std::bind(&TrajectoryManager::publishTrajectory, this));
  }

private:
  void publishTrajectory()
  {
    auto msg = buildTrajectory();
    if (msg.points.empty())
      return;
    msg.header.stamp = this->now();
    publisher_->publish(msg);
  }

  void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    publishTrajectory();
    resp->success = true;
    resp->message = "Trajectory published";
  }

  trajectory_msgs::msg::MultiDOFJointTrajectory buildTrajectory()
  {
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    const auto waypoints = this->get_parameter("waypoints").as_double_array();
    const size_t stride = 7;
    if (waypoints.size() < stride)
      return msg;

    for (size_t idx = 0; idx + stride <= waypoints.size(); idx += stride)
    {
      geometry_msgs::msg::Transform transform;
      transform.translation.x = waypoints[idx + 0];
      transform.translation.y = waypoints[idx + 1];
      transform.translation.z = waypoints[idx + 2];
      transform.rotation.x = waypoints[idx + 3];
      transform.rotation.y = waypoints[idx + 4];
      transform.rotation.z = waypoints[idx + 5];
      transform.rotation.w = waypoints[idx + 6];

      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      point.transforms.push_back(transform);
      point.time_from_start = rclcpp::Duration::from_seconds((idx / stride) * 1.0);
      msg.points.push_back(point);
    }
    return msg;
  }

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gik9dof_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gik9dof_ros::TrajectoryManager>());
  rclcpp::shutdown();
  return 0;
}
