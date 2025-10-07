/**
 * @file stage_b_chassis_plan.cpp
 * @brief Stage B Chassis Planning Controller Implementation
 * 
 * Implements Stage B of staged control architecture.
 * 
 * Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
 */

#include "stage_b_chassis_plan.hpp"
#include <cmath>
#include <algorithm>

namespace gik9dof {

StageBController::StageBController(rclcpp::Node* node, const StageBParams& params)
    : node_(node),
      params_(params),
      vel_controller_initialized_(false),
      pp_controller_initialized_(false),
      logger_(node->get_logger())
{
    // Initialize state
    state_.is_active = false;
    state_.path_valid = false;
    state_.current_waypoint_idx = 0;
    state_.total_waypoints = 0;
    state_.grid_received = false;
    state_.current_pose.setZero();
    state_.goal_pose.setZero();
    
    // Initialize MATLAB Coder planner
    planner_ = std::make_unique<gik9dof::HybridAStarPlanner>();
    
    // Initialize GIK solver for Stage B2 mode
    if (params_.mode == StageBMode::B2_GIK_ASSISTED) {
        gik_solver_ = std::make_unique<gik9dof::GIKSolver>();
        RCLCPP_INFO(logger_, "Stage B2 mode: GIK 3-DOF solver initialized");
    }
    
    // Initialize velocity controllers based on mode
    if (params_.velocity_control_mode == 1) {
        gik9dof_velocity::holisticVelocityController_initialize();
        
        // Initialize velocity controller state
        vel_state_.prev.x = 0.0;
        vel_state_.prev.y = 0.0;
        vel_state_.prev.theta = 0.0;
        vel_state_.prev.t = 0.0;
        
        RCLCPP_INFO(logger_, "Stage B: Simple heading controller initialized");
        
    } else if (params_.velocity_control_mode == 2) {
        gik9dof_purepursuit::purePursuitVelocityController_initialize();
        
        // Initialize Pure Pursuit state
        for (int i = 0; i < 30; i++) {
            pp_state_.pathX[i] = 0.0;
            pp_state_.pathY[i] = 0.0;
            pp_state_.pathTheta[i] = 0.0;
            pp_state_.pathTime[i] = 0.0;
        }
        pp_state_.numWaypoints = 0;
        pp_state_.prevVx = 0.0;
        pp_state_.prevWz = 0.0;
        pp_state_.prevPoseX = 0.0;
        pp_state_.prevPoseY = 0.0;
        pp_state_.prevPoseYaw = 0.0;
        pp_state_.lastRefTime = 0.0;
        
        RCLCPP_INFO(logger_, "Stage B: Pure Pursuit controller initialized");
    }
    
    // Subscribe to occupancy grid
    grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", 10,
        std::bind(&StageBController::occupancyGridCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(logger_, "Stage B Controller initialized");
    RCLCPP_INFO(logger_, "  Mode: %s", 
        params_.mode == StageBMode::B1_PURE_HYBRID_ASTAR ? "B1 (Pure Hybrid A*)" : "B2 (GIK-Assisted)");
    RCLCPP_INFO(logger_, "  Grid resolution: %.3f m", params_.grid_resolution);
    RCLCPP_INFO(logger_, "  Max planning time: %.0f ms", params_.max_planning_time * 1000.0);
    RCLCPP_INFO(logger_, "  Goal tolerance: xy=%.2f m, theta=%.1f deg", 
        params_.xy_tolerance, params_.theta_tolerance * 180.0 / M_PI);
}

StageBController::~StageBController()
{
    // Cleanup velocity controllers
    if (params_.velocity_control_mode == 1) {
        gik9dof_velocity::holisticVelocityController_terminate();
    } else if (params_.velocity_control_mode == 2) {
        gik9dof_purepursuit::purePursuitVelocityController_terminate();
    }
}

void StageBController::activate(const Eigen::Vector3d& current_base_pose,
                                 const Eigen::Vector3d& goal_base_pose,
                                 const std::vector<double>& arm_static_config)
{
    RCLCPP_INFO(logger_, "Activating Stage B controller");
    RCLCPP_INFO(logger_, "  Current: (%.2f, %.2f, %.1f°)", 
        current_base_pose.x(), current_base_pose.y(), current_base_pose.z() * 180.0 / M_PI);
    RCLCPP_INFO(logger_, "  Goal: (%.2f, %.2f, %.1f°)", 
        goal_base_pose.x(), goal_base_pose.y(), goal_base_pose.z() * 180.0 / M_PI);
    
    // Update state
    state_.is_active = true;
    state_.current_pose = current_base_pose;
    state_.goal_pose = goal_base_pose;
    state_.arm_grip_config = arm_static_config;
    state_.current_waypoint_idx = 0;
    state_.path_valid = false;
    state_.last_plan_time = node_->now();
    
    // Plan initial path
    if (!planPath()) {
        RCLCPP_ERROR(logger_, "Failed to plan initial path in Stage B!");
        state_.is_active = false;
        return;
    }
    
    RCLCPP_INFO(logger_, "Stage B activated successfully with %d waypoints", state_.total_waypoints);
}

void StageBController::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating Stage B controller");
    state_.is_active = false;
    state_.path_valid = false;
    state_.current_waypoint_idx = 0;
    state_.total_waypoints = 0;
    state_.path.clear();
}

bool StageBController::executeStep(const Eigen::Vector3d& current_base_pose,
                                   const std::vector<double>& current_arm_config,
                                   geometry_msgs::msg::Twist& base_cmd,
                                   sensor_msgs::msg::JointState& arm_cmd)
{
    if (!state_.is_active) {
        RCLCPP_WARN(logger_, "Stage B executeStep called but controller not active!");
        return false;
    }
    
    // Update current pose
    state_.current_pose = current_base_pose;
    
    // Check if goal reached
    if (chassisReachedGoal(current_base_pose)) {
        RCLCPP_INFO(logger_, "Stage B complete: Chassis reached goal!");
        deactivate();
        return false;  // Signal Stage B complete
    }
    
    // Check if replan needed (goal moved, or path invalid)
    auto now = node_->now();
    auto time_since_plan = (now - state_.last_plan_time).seconds();
    
    if (!state_.path_valid || time_since_plan > 5.0) {  // Replan every 5s
        RCLCPP_INFO(logger_, "Replanning path (valid=%d, time=%.1fs)", 
            state_.path_valid, time_since_plan);
        if (!planPath()) {
            RCLCPP_ERROR(logger_, "Replanning failed! Stopping.");
            base_cmd.linear.x = 0.0;
            base_cmd.angular.z = 0.0;
            return true;  // Still active, but can't move
        }
    }
    
    // Execute based on mode
    if (params_.mode == StageBMode::B1_PURE_HYBRID_ASTAR) {
        executeB1_PureHybridAStar(current_base_pose, base_cmd);
    } else {
        executeB2_GIKAssisted(current_base_pose, current_arm_config, base_cmd);
    }
    
    // Keep arm static at gripping configuration
    arm_cmd.name.resize(6);
    arm_cmd.position.resize(6);
    for (int i = 0; i < 6; i++) {
        arm_cmd.name[i] = "left_arm_joint" + std::to_string(i + 1);
        arm_cmd.position[i] = state_.arm_grip_config[i];
    }
    arm_cmd.header.stamp = node_->now();
    
    return true;  // Stage B still active
}

bool StageBController::chassisReachedGoal(const Eigen::Vector3d& current_base_pose) const
{
    // Check position tolerance
    double dx = current_base_pose.x() - state_.goal_pose.x();
    double dy = current_base_pose.y() - state_.goal_pose.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist > params_.xy_tolerance) {
        return false;
    }
    
    // Check heading tolerance
    double dtheta = current_base_pose.z() - state_.goal_pose.z();
    // Normalize to [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    
    if (std::abs(dtheta) > params_.theta_tolerance) {
        return false;
    }
    
    return true;
}

void StageBController::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    latest_grid_ = msg;
    state_.grid_received = true;
    
    RCLCPP_INFO_ONCE(logger_, "Occupancy grid received: %dx%d, resolution=%.3f m",
        msg->info.width, msg->info.height, msg->info.resolution);
}

bool StageBController::planPath()
{
    if (!state_.grid_received) {
        RCLCPP_WARN(logger_, "Cannot plan: No occupancy grid received yet");
        return false;
    }
    
    // Get latest occupancy grid
    nav_msgs::msg::OccupancyGrid::SharedPtr grid;
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid = latest_grid_;
    }
    
    // Convert to MATLAB Coder format
    if (!convertOccupancyGrid(*grid, occupancy_grid_)) {
        RCLCPP_ERROR(logger_, "Failed to convert occupancy grid");
        return false;
    }
    
    // Prepare start and goal states
    gik9dof::struct0_T start_state, goal_state;
    
    start_state.x = state_.current_pose.x();
    start_state.y = state_.current_pose.y();
    start_state.theta = state_.current_pose.z();
    start_state.Vx = 0.0;  // Will be set by planner
    start_state.Wz = 0.0;
    start_state.dt = 0.0;
    start_state.g = 0.0;
    start_state.h = 0.0;
    start_state.f = 0.0;
    start_state.parent_idx = -1;
    start_state.is_valid = true;
    
    goal_state.x = state_.goal_pose.x();
    goal_state.y = state_.goal_pose.y();
    goal_state.theta = state_.goal_pose.z();
    goal_state.Vx = 0.0;
    goal_state.Wz = 0.0;
    goal_state.dt = 0.0;
    goal_state.g = 0.0;
    goal_state.h = 0.0;
    goal_state.f = 0.0;
    goal_state.parent_idx = -1;
    goal_state.is_valid = true;
    
    // Plan path
    gik9dof::struct1_T path[500];
    gik9dof::struct2_T search_stats;
    
    auto plan_start = std::chrono::high_resolution_clock::now();
    
    planner_->b_gik9dof_planHybridAStarCodegen(&start_state, &goal_state, 
                                                &occupancy_grid_, path, &search_stats);
    
    auto plan_end = std::chrono::high_resolution_clock::now();
    double planning_time = std::chrono::duration<double>(plan_end - plan_start).count();
    
    // Store search stats
    state_.search_stats = search_stats;
    
    RCLCPP_INFO(logger_, "Hybrid A* planning complete:");
    RCLCPP_INFO(logger_, "  Success: %d", search_stats.success);
    RCLCPP_INFO(logger_, "  Planning time: %.3f s", planning_time);
    RCLCPP_INFO(logger_, "  Iterations: %.0f", search_stats.iterations);
    RCLCPP_INFO(logger_, "  Nodes expanded: %.0f", search_stats.nodes_expanded);
    RCLCPP_INFO(logger_, "  Path cost: %.2f", search_stats.path_cost);
    
    if (!search_stats.success) {
        RCLCPP_ERROR(logger_, "Hybrid A* failed to find path!");
        state_.path_valid = false;
        return false;
    }
    
    // Extract valid path waypoints (dt > 0)
    state_.path.clear();
    for (int i = 0; i < 500; i++) {
        if (path[i].dt > 0.0) {
            state_.path.push_back(path[i]);
        }
    }
    
    state_.total_waypoints = state_.path.size();
    state_.current_waypoint_idx = 0;
    state_.path_valid = true;
    state_.last_plan_time = node_->now();
    
    RCLCPP_INFO(logger_, "Valid path extracted: %d waypoints", state_.total_waypoints);
    
    return true;
}

void StageBController::executeB1_PureHybridAStar(const Eigen::Vector3d& current_base_pose,
                                                  geometry_msgs::msg::Twist& base_cmd)
{
    if (state_.path.empty()) {
        base_cmd.linear.x = 0.0;
        base_cmd.angular.z = 0.0;
        return;
    }
    
    // Stage B1: Use velocity commands directly from Hybrid A* path
    // Find nearest waypoint (simple nearest-neighbor)
    int nearest_idx = 0;
    double min_dist = 1e9;
    
    for (size_t i = state_.current_waypoint_idx; i < state_.path.size(); i++) {
        double dx = state_.path[i].x - current_base_pose.x();
        double dy = state_.path[i].y - current_base_pose.y();
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
        
        // Stop searching if we're past the robot
        if (i > state_.current_waypoint_idx + 10) break;
    }
    
    state_.current_waypoint_idx = nearest_idx;
    
    // Use velocity commands from nearest waypoint
    base_cmd.linear.x = state_.path[nearest_idx].Vx;
    base_cmd.angular.z = state_.path[nearest_idx].Wz;
    
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
        "Stage B1: waypoint %d/%d, Vx=%.2f, Wz=%.2f",
        nearest_idx, state_.total_waypoints, base_cmd.linear.x, base_cmd.angular.z);
}

void StageBController::executeB2_GIKAssisted(const Eigen::Vector3d& current_base_pose,
                                              const std::vector<double>& current_arm_config,
                                              geometry_msgs::msg::Twist& base_cmd)
{
    if (state_.path.empty()) {
        base_cmd.linear.x = 0.0;
        base_cmd.angular.z = 0.0;
        return;
    }
    
    // Stage B2: Hybrid A* waypoints → GIK 3-DOF → Velocity Controller
    // TODO: Implement GIK 3-DOF solver for base tracking
    // For now, use same approach as B1
    
    RCLCPP_WARN_ONCE(logger_, "Stage B2 (GIK-assisted) not fully implemented yet, using B1 approach");
    executeB1_PureHybridAStar(current_base_pose, base_cmd);
}

bool StageBController::convertOccupancyGrid(const nav_msgs::msg::OccupancyGrid& ros_grid,
                                             gik9dof::OccupancyGrid2D& matlab_grid)
{
    // Check grid size (max 200x200 = 40000 cells)
    if (ros_grid.info.width * ros_grid.info.height > 40000) {
        RCLCPP_ERROR(logger_, "Occupancy grid too large: %dx%d (max 200x200)",
            ros_grid.info.width, ros_grid.info.height);
        return false;
    }
    
    // Convert to bool array (true = occupied)
    bool data[40000];
    for (size_t i = 0; i < ros_grid.data.size() && i < 40000; i++) {
        // ROS: -1=unknown, 0=free, 100=occupied
        // MATLAB: true=occupied, false=free (conservative: unknown→occupied)
        data[i] = (ros_grid.data[i] > 50 || ros_grid.data[i] < 0);
    }
    
    // Fill remaining cells as free
    for (size_t i = ros_grid.data.size(); i < 40000; i++) {
        data[i] = false;
    }
    
    // Initialize MATLAB grid
    matlab_grid.init(data, 
                     ros_grid.info.resolution,
                     ros_grid.info.origin.position.x,
                     ros_grid.info.origin.position.y,
                     ros_grid.info.width,
                     ros_grid.info.height);
    
    return true;
}

} // namespace gik9dof
