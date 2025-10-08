/**
 * @file gik9dof_solver_node.h
 * @brief ROS2 node wrapping MATLAB Coder generated IK solver - Header
 * 
 * This node subscribes to end-effector trajectory commands and publishes
 * joint-space trajectories for the 9-DOF mobile manipulator.
 * 
 * Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
 */

#ifndef GIK9DOF_SOLVER_NODE_H
#define GIK9DOF_SOLVER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "gik9dof_msgs/msg/end_effector_trajectory.hpp"
#include "gik9dof_msgs/msg/solver_diagnostics.hpp"

#include <memory>
#include <vector>
#include <deque>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// MATLAB Coder generated types (only the ones that don't conflict)
#include "velocity_controller/holisticVelocityController_types.h"
#include "purepursuit/purePursuitVelocityController_types.h"

// Forward declarations to avoid namespace conflicts
// DO NOT include GIKSolver or Stage B headers here - they conflict with planner types
namespace gik9dof {
    class GIKSolver;
    class StageBController;  // Forward declare - actual class in separate library
}

using namespace std::chrono_literals;

/**
 * @brief Main ROS2 node for 9-DOF GIK solver with staged control
 */
class GIK9DOFSolverNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes parameters, subscribers, publishers
     */
    GIK9DOFSolverNode();
    
    /**
     * @brief Destructor - cleanup velocity controllers
     */
    ~GIK9DOFSolverNode();

private:
    // ========== CALLBACK FUNCTIONS ==========
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void trajectoryCallback(const gik9dof_msgs::msg::EndEffectorTrajectory::SharedPtr msg);
    
    // ========== CONTROL LOOP ==========
    void controlLoop();
    
    // ========== STAGED CONTROL STATE MACHINE ==========
    void executeStagedControl(const geometry_msgs::msg::Pose& target_pose);
    void executeStageA(const geometry_msgs::msg::Pose& target_pose);
    void executeStageB(const geometry_msgs::msg::Pose& target_pose);
    void executeStageC(const geometry_msgs::msg::Pose& target_pose);
    void executeHolisticControl(const geometry_msgs::msg::Pose& target_pose);
    
    // ========== HELPER FUNCTIONS ==========
    bool checkArmAtHome();
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
    double computeEndEffectorDistance(const geometry_msgs::msg::Pose& target_pose);
    void publishZeroVelocity();
    
    // ========== IK SOLVER ==========
    bool solveIK(const geometry_msgs::msg::Pose& target_pose);
    
    // ========== COMMAND PUBLISHERS ==========
    void publishJointCommand();
    void publishBaseCommand();
    void publishDiagnostics(double solve_time_ms, const geometry_msgs::msg::Pose& target_pose);
    
    // ========== ROS2 COMMUNICATION ==========
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gik9dof_msgs::msg::EndEffectorTrajectory>::SharedPtr trajectory_sub_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
    rclcpp::Publisher<gik9dof_msgs::msg::SolverDiagnostics>::SharedPtr diagnostics_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // ========== STATE VARIABLES ==========
    std::mutex state_mutex_;
    std::mutex trajectory_mutex_;
    
    std::vector<double> current_config_;  // 9-DOF: [x, y, theta, q1, q2, q3, q4, q5, q6]
    std::vector<double> target_config_;   // Target configuration from IK
    
    bool arm_state_received_ = false;
    bool base_state_received_ = false;
    
    // Trajectory tracking
    std::shared_ptr<gik9dof_msgs::msg::EndEffectorTrajectory> current_trajectory_;
    uint32_t trajectory_sequence_ = 0;  // Sequence ID of current trajectory
    size_t current_waypoint_index_ = 0;  // Current waypoint being tracked
    bool trajectory_complete_ = false;   // Flag indicating trajectory completed
    double waypoint_tolerance_ = 0.05;   // Distance threshold to advance waypoint (meters)
    
    // Configuration history (for velocity estimation)
    static constexpr size_t HISTORY_SIZE = 10;
    std::deque<std::vector<double>> config_history_;
    std::deque<rclcpp::Time> time_history_;
    
    // Diagnostics
    int last_iterations_ = 0;
    double last_solve_time_ms_ = 0.0;
    bool last_solve_success_ = false;
    int last_solver_iterations_ = 0;
    std::string last_solver_status_;
    int last_exit_flag_ = 0;
    int last_random_restarts_ = 0;
    double last_pose_violation_ = 0.0;
    double last_position_error_ = 0.0;
    double last_orientation_error_ = 0.0;
    bool last_joint_limit_violation_ = false;
    bool last_distance_constraint_met_ = true;
    std::vector<double> last_constraint_violations_;
    
    // Per-constraint distance violation tracking (20 constraints)
    double last_dist_violations_[20];           // Violation value for each constraint
    bool last_dist_constraint_met_[20];         // Status for each constraint
    int last_num_active_constraints_ = 0;       // Count of enabled constraints
    int last_num_violated_constraints_ = 0;     // Count of violated constraints
    
    rclcpp::Time last_solve_start_;
    rclcpp::Time last_solve_end_;
    
    // ========== PARAMETERS ==========
    double control_rate_;
    double max_solve_time_;
    int max_solver_iterations_;
    
    // Distance constraints (20 total) - NEW 7-parameter interface
    int dist_body_indices_[20];       // Body indices for distance constraints
    int dist_ref_body_indices_[20];   // Reference body indices
    double dist_lower_bounds_[20];    // Lower distance bounds (meters)
    double dist_upper_bounds_[20];    // Upper distance bounds (meters)
    double dist_weights_[20];         // Constraint weights (0.0=disabled, >0.0=enabled)
    
    bool publish_diagnostics_;
    bool use_warm_start_;  // Enable warm-start from previous solution
    int velocity_control_mode_;  // 0=legacy 5-pt diff, 1=heading ctrl, 2=pure pursuit
    
    // ========== VELOCITY CONTROLLERS ==========
    // Simple heading controller state (mode 1)
    gik9dof_velocity::struct0_T vel_params_;  // Controller parameters
    gik9dof_velocity::struct1_T vel_state_;   // Controller state
    bool vel_controller_initialized_;         // Flag to track first call
    
    // Pure Pursuit controller state (mode 2)
    gik9dof_purepursuit::struct0_T pp_params_;  // Pure Pursuit parameters
    gik9dof_purepursuit::struct1_T pp_state_;   // Pure Pursuit state (path buffer)
    bool pp_controller_initialized_;            // Flag to track first call
    
    // ========== STAGED CONTROL STATE MACHINE ==========
    enum class ControlMode { HOLISTIC, STAGED };
    enum class ControlStage { STAGE_A, STAGE_B, STAGE_C };
    
    ControlMode control_mode_;
    ControlStage current_stage_;
    gik9dof::StageBController* stage_b_controller_;  // RAW pointer (managed manually to avoid incomplete type in unique_ptr)
    bool stage_b_enabled_;
    
    // ========== MATLAB SOLVER ==========
    // NOTE: With new codegen_inuse interface, solver is a free function (no object needed)
    // std::unique_ptr<gik9dof::GIKSolver> matlab_solver_;  // DEPRECATED
};

#endif // GIK9DOF_SOLVER_NODE_H
