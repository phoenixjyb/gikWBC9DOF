/**
 * @file stage_b_chassis_plan.hpp
 * @brief Stage B Chassis Planning Controller (Mode 2 - Staged Control)
 * 
 * Implements Stage B of staged control architecture:
 * - Stage B1: Pure Hybrid A* → Velocity Controller
 * - Stage B2: Hybrid A* → GIK 3-DOF → Velocity Controller
 * 
 * Stage B executes chassis planning while arm is in static gripping pose.
 * Transitions to Stage C (full-body tracking) when chassis reaches goal.
 * 
 * Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
 */

#ifndef STAGE_B_CHASSIS_PLAN_HPP
#define STAGE_B_CHASSIS_PLAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <deque>

// MATLAB Coder generated velocity controllers
#include "velocity_controller/holisticVelocityController.h"
#include "velocity_controller/holisticVelocityController_types.h"
#include "velocity_controller/holisticVelocityController_initialize.h"
#include "velocity_controller/holisticVelocityController_terminate.h"
#include "purepursuit/purePursuitVelocityController.h"
#include "purepursuit/purePursuitVelocityController_types.h"
#include "purepursuit/purePursuitVelocityController_initialize.h"
#include "purepursuit/purePursuitVelocityController_terminate.h"

// MATLAB Coder generated planner
#include "HybridAStarPlanner.h"
#include "OccupancyGrid2D.h"
#include "gik9dof_planHybridAStarCodegen_types.h"

// NOTE: NOT including GIKSolver.h to avoid struct0_T/struct1_T conflicts with planner types
// Stage B2 will use planner waypoints directly without invoking full GIK solver
// (The full 9-DOF GIK solver is used in Stage C, not Stage B which is chassis-only)

namespace gik9dof {

/**
 * @brief Stage B execution modes (internal detailed version)
 */
enum class StageBMode_Internal {
    B1_PURE_HYBRID_ASTAR,  // Pure Hybrid A* → Velocity
    B2_GIK_ASSISTED        // Hybrid A* → GIK 3-DOF → Velocity
};

/**
 * @brief Stage B controller parameters (internal detailed version)
 * Note: Factory uses simpler StageBParams from stage_b_factory.hpp
 */
struct StageBParams_Internal {
    // Mode selection
    StageBMode_Internal mode;
    
    // Hybrid A* planner parameters
    double grid_resolution;          // Occupancy grid resolution (m/cell)
    double max_planning_time;        // Max planning time (s)
    double robot_radius;             // Robot footprint radius (m)
    double replan_threshold;         // Replan if goal moves > threshold (m)
    
    // Goal tolerance
    double xy_tolerance;             // Position tolerance (m)
    double theta_tolerance;          // Heading tolerance (rad)
    
    // GIK 3-DOF parameters (for Stage B2)
    double distance_lower_bound;     // Min distance weight (m)
    double distance_weight;          // Distance cost weight
    bool use_warm_start;             // Use previous solution as init guess
    
    // Velocity controller selection (shared with holistic mode)
    int velocity_control_mode;       // 0=legacy, 1=simple heading, 2=pure pursuit
    
    // Pure Pursuit parameters (for mode 2)
    gik9dof_purepursuit::struct0_T pp_params;  // Pure Pursuit controller params
};

/**
 * @brief Stage B controller state
 */
struct StageBState {
    bool is_active;                  // Stage B currently executing?
    bool path_valid;                 // Valid path to goal?
    int current_waypoint_idx;        // Current waypoint in path
    int total_waypoints;             // Total waypoints in path
    
    // Current pose (base only: x, y, theta)
    Eigen::Vector3d current_pose;
    
    // Goal pose (base only: x, y, theta)
    Eigen::Vector3d goal_pose;
    
    // Planned path from Hybrid A*
    std::vector<gik9dof::struct1_T> path;
    
    // Search statistics
    gik9dof::struct2_T search_stats;
    
    // Arm static gripping configuration (6-DOF)
    std::vector<double> arm_grip_config;
    
    // Last planning time
    rclcpp::Time last_plan_time;
    
    // Occupancy grid received?
    bool grid_received;
};

/**
 * @brief Stage B Chassis Planning Controller
 * 
 * Manages chassis planning during Stage B of staged control:
 * 1. Subscribe to occupancy grid
 * 2. Plan Hybrid A* path from current base pose to goal
 * 3. Execute path:
 *    - B1: Pure Hybrid A* → Velocity Controller
 *    - B2: Hybrid A* waypoints → GIK 3-DOF → Velocity Controller
 * 4. Check goal reached
 * 5. Transition to Stage C when chassis at goal
 */
class StageBController {
public:
    /**
     * @brief Constructor
     * 
     * @param node ROS2 node pointer for publishers/subscribers
     * @param params Stage B parameters (internal detailed version)
     */
    StageBController(rclcpp::Node* node, const StageBParams_Internal& params);
    
    /**
     * @brief Destructor
     */
    ~StageBController();
    
    /**
     * @brief Activate Stage B controller
     * 
     * @param current_base_pose Current base pose [x, y, theta]
     * @param goal_base_pose Goal base pose [x, y, theta]
     * @param arm_static_config Arm gripping configuration (6-DOF)
     */
    void activate(const Eigen::Vector3d& current_base_pose,
                  const Eigen::Vector3d& goal_base_pose,
                  const std::vector<double>& arm_static_config);
    
    /**
     * @brief Deactivate Stage B controller
     */
    void deactivate();
    
    /**
     * @brief Execute one step of Stage B controller
     * 
     * Called at control loop rate (e.g., 10 Hz)
     * 
     * @param current_base_pose Current base pose [x, y, theta]
     * @param current_arm_config Current arm configuration (6-DOF)
     * @param[out] base_cmd Output base velocity command
     * @param[out] arm_cmd Output arm joint state (static during Stage B)
     * @return true if Stage B still active, false if goal reached
     */
    bool executeStep(const Eigen::Vector3d& current_base_pose,
                     const std::vector<double>& current_arm_config,
                     geometry_msgs::msg::Twist& base_cmd,
                     sensor_msgs::msg::JointState& arm_cmd);
    
    /**
     * @brief Check if chassis has reached goal
     * 
     * @param current_base_pose Current base pose [x, y, theta]
     * @return true if within tolerance of goal
     */
    bool chassisReachedGoal(const Eigen::Vector3d& current_base_pose) const;
    
    /**
     * @brief Get current Stage B state (for diagnostics)
     */
    const StageBState& getState() const { return state_; }

private:
    /**
     * @brief Occupancy grid callback
     */
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Plan Hybrid A* path
     * 
     * @return true if path found
     */
    bool planPath();
    
    /**
     * @brief Execute Stage B1 (Pure Hybrid A* → Velocity)
     * 
     * @param current_base_pose Current base pose
     * @param[out] base_cmd Output velocity command
     */
    void executeB1_PureHybridAStar(const Eigen::Vector3d& current_base_pose,
                                    geometry_msgs::msg::Twist& base_cmd);
    
    /**
     * @brief Execute Stage B2 (Hybrid A* → GIK 3-DOF → Velocity)
     * 
     * @param current_base_pose Current base pose
     * @param current_arm_config Current arm configuration
     * @param[out] base_cmd Output velocity command
     */
    void executeB2_GIKAssisted(const Eigen::Vector3d& current_base_pose,
                               const std::vector<double>& current_arm_config,
                               geometry_msgs::msg::Twist& base_cmd);
    
    /**
     * @brief Convert ROS OccupancyGrid to MATLAB Coder format
     * 
     * @param ros_grid Input ROS occupancy grid
     * @param[out] matlab_grid Output MATLAB Coder grid
     * @return true if conversion successful
     */
    bool convertOccupancyGrid(const nav_msgs::msg::OccupancyGrid& ros_grid,
                              gik9dof::OccupancyGrid2D& matlab_grid);
    
    // ROS2 node pointer
    rclcpp::Node* node_;
    
    // Parameters
    StageBParams_Internal params_;
    
    // State
    StageBState state_;
    
    // MATLAB Coder objects
    std::unique_ptr<gik9dof::HybridAStarPlanner> planner_;
    gik9dof::OccupancyGrid2D occupancy_grid_;
    // Note: GIK solver removed - Stage B is chassis-only planning, full 9-DOF GIK used in Stage C
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    
    // Occupancy grid storage
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
    std::mutex grid_mutex_;
    
    // Velocity controller (mode 1: simple heading)
    gik9dof_velocity::struct0_T vel_params_;   // Parameters: track, Vwheel_max, etc.
    gik9dof_velocity::struct1_T vel_state_;    // State with prev (x, y, theta, t)
    bool vel_controller_initialized_;
    
    // Pure Pursuit controller (mode 2)
    gik9dof_purepursuit::struct0_T pp_params_; // Parameters: lookahead, vxNominal, etc.
    gik9dof_purepursuit::struct1_T pp_state_;  // State: pathX[], pathY[], prevVx, etc.
    bool pp_controller_initialized_;
    
    // Logger
    rclcpp::Logger logger_;
};

// NOTE: Factory functions are declared in stage_b_factory.hpp, not here!
// This avoids duplicate declarations.

} // namespace gik9dof

#endif // STAGE_B_CHASSIS_PLAN_HPP
