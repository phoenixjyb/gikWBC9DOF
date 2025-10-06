/**
 * @file gik9dof_solver_node.cpp
 * @brief ROS2 node wrapping MATLAB Coder generated IK solver
 * 
 * This node subscribes to end-effector trajectory commands and publishes
 * joint-space trajectories for the 9-DOF mobile manipulator.
 * 
 * Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "gik9dof_msgs/msg/end_effector_trajectory.hpp"
#include "gik9dof_msgs/msg/solver_diagnostics.hpp"

#include <memory>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// MATLAB Coder generated headers (to be included after code generation)
// #include "gik9dof/GIKSolver.h"

using namespace std::chrono_literals;

class GIK9DOFSolverNode : public rclcpp::Node
{
public:
    GIK9DOFSolverNode() : Node("gik9dof_solver_node")
    {
        // Declare parameters
        this->declare_parameter("control_rate", 10.0);  // Hz
        this->declare_parameter("max_solve_time", 0.05);  // 50ms
        this->declare_parameter("distance_lower_bound", 0.1);  // meters
        this->declare_parameter("distance_weight", 1.0);
        this->declare_parameter("publish_diagnostics", true);
        
        // Get parameters
        control_rate_ = this->get_parameter("control_rate").as_double();
        max_solve_time_ = this->get_parameter("max_solve_time").as_double();
        distance_lower_ = this->get_parameter("distance_lower_bound").as_double();
        distance_weight_ = this->get_parameter("distance_weight").as_double();
        publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();
        
        // Initialize state
        current_config_.resize(9, 0.0);
        target_config_.resize(9, 0.0);
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_arm_left", 10,
            std::bind(&GIK9DOFSolverNode::jointStateCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_wheel", 10,
            std::bind(&GIK9DOFSolverNode::odomCallback, this, std::placeholders::_1));
        
        trajectory_sub_ = this->create_subscription<gik9dof_msgs::msg::EndEffectorTrajectory>(
            "/gik9dof/target_trajectory", 10,
            std::bind(&GIK9DOFSolverNode::trajectoryCallback, this, std::placeholders::_1));
        
        // Publishers
        joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motion_target/target_joint_state_arm_left", 10);
        
        if (publish_diagnostics_) {
            diagnostics_pub_ = this->create_publisher<gik9dof_msgs::msg::SolverDiagnostics>(
                "/gik9dof/solver_diagnostics", 10);
        }
        
        // Control timer
        auto timer_period = std::chrono::duration<double>(1.0 / control_rate_);
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&GIK9DOFSolverNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "GIK9DOF Solver Node initialized");
        RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_);
        RCLCPP_INFO(this->get_logger(), "Max solve time: %.0f ms", max_solve_time_ * 1000.0);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update arm joint states (6 DOF)
        // Assumes message contains joints in order: left_arm_joint1-6
        if (msg->position.size() >= 6) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < 6; ++i) {
                current_config_[3 + i] = msg->position[i];  // Arm starts at index 3
            }
            arm_state_received_ = true;
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update base state (3 DOF: x, y, theta)
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_config_[0] = msg->pose.pose.position.x;
        current_config_[1] = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        current_config_[2] = std::atan2(2.0 * (qw * qz + qx * qy), 
                                        1.0 - 2.0 * (qy * qy + qz * qz));
        
        base_state_received_ = true;
    }
    
    void trajectoryCallback(const gik9dof_msgs::msg::EndEffectorTrajectory::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_ = msg;
        trajectory_sequence_ = msg->sequence_id;
        
        RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu waypoints, seq=%u",
                    msg->waypoints.size(), msg->sequence_id);
    }
    
    void controlLoop()
    {
        // Check if we have valid state
        if (!arm_state_received_ || !base_state_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for robot state (arm: %d, base: %d)",
                                 arm_state_received_, base_state_received_);
            return;
        }
        
        // Get current trajectory target
        geometry_msgs::msg::Pose target_pose;
        bool has_target = false;
        
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (current_trajectory_ && !current_trajectory_->waypoints.empty()) {
                // For now, use first waypoint (TODO: interpolate based on time)
                target_pose = current_trajectory_->waypoints[0].pose;
                has_target = true;
            }
        }
        
        if (!has_target) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No trajectory target available");
            return;
        }
        
        // Solve IK
        auto solve_start = this->now();
        bool solve_success = solveIK(target_pose);
        auto solve_end = this->now();
        double solve_time_ms = (solve_end - solve_start).seconds() * 1000.0;
        
        if (solve_success) {
            publishJointCommand();
            
            if (publish_diagnostics_) {
                publishDiagnostics(solve_time_ms, target_pose);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "IK solve failed (%.2f ms)", solve_time_ms);
        }
    }
    
    bool solveIK(const geometry_msgs::msg::Pose& target_pose)
    {
        // Convert ROS Pose to 4x4 homogeneous transform
        Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
        
        // Position
        target_transform(0, 3) = target_pose.position.x;
        target_transform(1, 3) = target_pose.position.y;
        target_transform(2, 3) = target_pose.position.z;
        
        // Orientation (quaternion to rotation matrix)
        Eigen::Quaterniond q(target_pose.orientation.w,
                             target_pose.orientation.x,
                             target_pose.orientation.y,
                             target_pose.orientation.z);
        target_transform.block<3, 3>(0, 0) = q.toRotationMatrix();
        
        // TODO: Call MATLAB Coder generated function
        // Example (actual signature depends on code generation):
        // double q_current[9];
        // double q_next[9];
        // double target_matrix[16];
        // 
        // // Copy current config to C array
        // std::copy(current_config_.begin(), current_config_.end(), q_current);
        // 
        // // Flatten target transform (column-major for MATLAB)
        // for (int i = 0; i < 16; ++i) {
        //     target_matrix[i] = target_transform.data()[i];
        // }
        // 
        // // Call generated solver
        // gik9dof::GIKSolver solver;
        // solver.solveGIKStepWrapper(q_current, target_matrix, 
        //                           distance_lower_, distance_weight_, q_next);
        // 
        // // Copy result
        // std::copy(q_next, q_next + 9, target_config_.begin());
        
        // PLACEHOLDER: For now, just copy current config (replace when code is generated)
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_config_ = current_config_;
        
        return true;  // Replace with actual solver status
    }
    
    void publishJointCommand()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base";
        
        // Joint names (arm only - base is controlled separately)
        msg.name = {"left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
                    "left_arm_joint4", "left_arm_joint5", "left_arm_joint6"};
        
        // Joint positions (arm DOFs only)
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.position.assign(target_config_.begin() + 3, target_config_.end());
        
        joint_cmd_pub_->publish(msg);
    }
    
    void publishDiagnostics(double solve_time_ms, const geometry_msgs::msg::Pose& target_pose)
    {
        auto msg = gik9dof_msgs::msg::SolverDiagnostics();
        msg.header.stamp = this->now();
        msg.solve_time_ms = solve_time_ms;
        msg.status = 1;  // TODO: Get from solver
        msg.iterations = 0;  // TODO: Get from solver
        msg.pose_error_norm = 0.0;  // TODO: Calculate
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.current_config = current_config_;
        msg.target_config = target_config_;
        msg.target_ee_pose = target_pose;
        
        diagnostics_pub_->publish(msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gik9dof_msgs::msg::EndEffectorTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<gik9dof_msgs::msg::SolverDiagnostics>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    std::vector<double> current_config_;  // 9 DOF: [x, y, theta, arm1-6]
    std::vector<double> target_config_;   // 9 DOF
    gik9dof_msgs::msg::EndEffectorTrajectory::SharedPtr current_trajectory_;
    uint32_t trajectory_sequence_ = 0;
    
    // Synchronization
    std::mutex state_mutex_;
    std::mutex trajectory_mutex_;
    
    // Status flags
    bool arm_state_received_ = false;
    bool base_state_received_ = false;
    
    // Parameters
    double control_rate_;
    double max_solve_time_;
    double distance_lower_;
    double distance_weight_;
    bool publish_diagnostics_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GIK9DOFSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
