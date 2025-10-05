#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace gik9dof_ros
{

struct Disc
{
  double x;
  double y;
  double radius;
  double safety_margin;
};

class ObstacleProvider : public rclcpp::Node
{
public:
  ObstacleProvider()
  : Node("obstacle_provider")
  {
    this->declare_parameter<std::vector<double>>("discs", {-2.0, -2.0, 0.4, 0.05});
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate", 1.0);

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/obstacles/floor_discs", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      std::bind(&ObstacleProvider::publishMarkers, this));

    discs_ = parseDiscs();
  }

private:
  std::vector<Disc> parseDiscs()
  {
    std::vector<Disc> discs;
    const auto flat = this->get_parameter("discs").as_double_array();
    const size_t stride = 4;
    for (size_t idx = 0; idx + stride <= flat.size(); idx += stride)
    {
      discs.push_back({flat[idx], flat[idx+1], flat[idx+2], flat[idx+3]});
    }
    return discs;
  }

  void publishMarkers()
  {
    auto msg = buildMarkerArray();
    publisher_->publish(msg);
  }

  visualization_msgs::msg::MarkerArray buildMarkerArray()
  {
    visualization_msgs::msg::MarkerArray array;
    const auto frame = this->get_parameter("frame_id").as_string();
    const auto stamp = this->now();
    int id = 0;
    for (const auto & disc : discs_)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = stamp;
      marker.ns = "floor_disc";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = disc.x;
      marker.pose.position.y = disc.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      const double diameter = 2.0 * (disc.radius + disc.safety_margin);
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = 0.02;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.4f;
      array.markers.push_back(marker);
    }
    return array;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Disc> discs_;
};

}  // namespace gik9dof_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gik9dof_ros::ObstacleProvider>());
  rclcpp::shutdown();
  return 0;
}
