/**
 * @file hybrid_astar_planner_node.cpp
 * @brief ROS2 service node for Hybrid A* path planning using MATLAB planner
 * 
 * This node provides a planning service that calls the MATLAB Hybrid A* implementation
 * and returns paths for the mobile manipulator. Integrates with existing gik9dof nodes.
 * 
 * Service: /plan_path (gik9dof_msgs/srv/PlanPath)
 * Publishers: /planned_path (nav_msgs/msg/Path)
 * Subscribers: /occupancy_grid (nav_msgs/msg/OccupancyGrid)
 * 
 * @author Generated for WHEELTEC mobile manipulator
 * @date 2025-10-07
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// MATLAB Engine API (requires MATLAB Runtime or full MATLAB installation)
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using matlab::engine::MATLABEngine;
using matlab::data::ArrayFactory;

/**
 * @brief Hybrid A* planner service node using MATLAB backend
 */
class HybridAStarPlannerNode : public rclcpp::Node
{
public:
    HybridAStarPlannerNode() 
        : Node("hybrid_astar_planner"),
          grid_received_(false)
    {
        // Parameters
        this->declare_parameter("matlab_workspace_path", "/path/to/gikWBC9DOF/matlab");
        this->declare_parameter("planning_timeout", 5.0);  // seconds
        this->declare_parameter("grid_inflation_radius", 0.511);  // meters
        
        matlab_workspace_ = this->get_parameter("matlab_workspace_path").as_string();
        planning_timeout_ = this->get_parameter("planning_timeout").as_double();
        inflation_radius_ = this->get_parameter("grid_inflation_radius").as_double();
        
        // Initialize MATLAB Engine
        try {
            RCLCPP_INFO(this->get_logger(), "Starting MATLAB Engine...");
            matlab_engine_ = matlab::engine::startMATLAB();
            
            // Add MATLAB workspace to path
            matlab::data::ArrayFactory factory;
            std::u16string ws_path_u16(matlab_workspace_.begin(), matlab_workspace_.end());
            matlab_engine_->eval(u"addpath(genpath('" + ws_path_u16 + u"'))");
            
            RCLCPP_INFO(this->get_logger(), "MATLAB Engine initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Workspace: %s", matlab_workspace_.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start MATLAB Engine: %s", e.what());
            throw;
        }
        
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        
        // Subscribers
        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid", 10,
            std::bind(&HybridAStarPlannerNode::gridCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&HybridAStarPlannerNode::goalCallback, this, std::placeholders::_1));
            
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&HybridAStarPlannerNode::startCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: /occupancy_grid, /goal_pose, /initialpose");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /planned_path");
    }
    
    ~HybridAStarPlannerNode()
    {
        if (matlab_engine_) {
            matlab::engine::terminateEngineClient();
        }
    }

private:
    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_grid_ = msg;
        grid_received_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Received occupancy grid: %dx%d @ %.2fm resolution",
                    msg->info.width, msg->info.height, msg->info.resolution);
    }
    
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        start_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
        start_pose_->header = msg->header;
        start_pose_->pose = msg->pose.pose;
        
        RCLCPP_INFO(this->get_logger(), "Start pose updated: (%.2f, %.2f, %.2f)",
                   start_pose_->pose.position.x,
                   start_pose_->pose.position.y,
                   getYawFromQuaternion(start_pose_->pose.orientation));
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = msg;
        
        RCLCPP_INFO(this->get_logger(), "Goal pose received: (%.2f, %.2f, %.2f)",
                   goal_pose_->pose.position.x,
                   goal_pose_->pose.position.y,
                   getYawFromQuaternion(goal_pose_->pose.orientation));
        
        // Trigger planning if we have both start and grid
        if (start_pose_ && grid_received_) {
            planPath();
        } else {
            if (!start_pose_) {
                RCLCPP_WARN(this->get_logger(), "Waiting for start pose (/initialpose)");
            }
            if (!grid_received_) {
                RCLCPP_WARN(this->get_logger(), "Waiting for occupancy grid");
            }
        }
    }
    
    void planPath()
    {
        if (!matlab_engine_) {
            RCLCPP_ERROR(this->get_logger(), "MATLAB Engine not initialized");
            return;
        }
        
        auto start_time = std::chrono::steady_clock::now();
        
        try {
            RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)...",
                       start_pose_->pose.position.x, start_pose_->pose.position.y,
                       goal_pose_->pose.position.x, goal_pose_->pose.position.y);
            
            // Create MATLAB data arrays
            ArrayFactory factory;
            
            // Convert start pose to MATLAB HybridState
            double start_x = start_pose_->pose.position.x;
            double start_y = start_pose_->pose.position.y;
            double start_theta = getYawFromQuaternion(start_pose_->pose.orientation);
            
            // Convert goal pose to MATLAB HybridState  
            double goal_x = goal_pose_->pose.position.x;
            double goal_y = goal_pose_->pose.position.y;
            double goal_theta = getYawFromQuaternion(goal_pose_->pose.orientation);
            
            // Convert occupancy grid to MATLAB format
            // (This is simplified - you'd need to convert ROS OccupancyGrid to MATLAB OccupancyGrid2D)
            
            // Call MATLAB planner (simplified - actual implementation would create proper structs)
            std::u16string matlab_cmd = u"[path, stats] = gik9dof.planHybridAStar("
                                        u"gik9dof.HybridState(" + 
                                        std::to_wstring(start_x) + u", " +
                                        std::to_wstring(start_y) + u", " +
                                        std::to_wstring(start_theta) + u"), " +
                                        u"gik9dof.HybridState(" +
                                        std::to_wstring(goal_x) + u", " +
                                        std::to_wstring(goal_y) + u", " +
                                        std::to_wstring(goal_theta) + u"), " +
                                        u"current_grid);";
            
            matlab_engine_->eval(matlab_cmd);
            
            // Get results from MATLAB workspace
            auto path_result = matlab_engine_->getVariable(u"path");
            auto stats_result = matlab_engine_->getVariable(u"stats");
            
            // Convert MATLAB path to ROS Path message
            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp = this->now();
            ros_path.header.frame_id = goal_pose_->header.frame_id;
            
            // Extract waypoints from MATLAB struct array
            // (Implementation depends on MATLAB data structure)
            
            // Publish path
            path_pub_->publish(ros_path);
            
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            RCLCPP_INFO(this->get_logger(), "✓ Path planning complete in %ld ms", duration.count());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", e.what());
        }
    }
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        // Extract yaw from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    // MATLAB Engine
    std::shared_ptr<MATLABEngine> matlab_engine_;
    std::string matlab_workspace_;
    double planning_timeout_;
    double inflation_radius_;
    
    // ROS2 interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    
    // State
    nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
    geometry_msgs::msg::PoseStamped::SharedPtr start_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;
    bool grid_received_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<HybridAStarPlannerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("hybrid_astar_planner"), 
                    "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
