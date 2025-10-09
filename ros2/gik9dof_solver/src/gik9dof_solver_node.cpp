/**
 * @file gik9dof_solver_node.cpp
 * @brief ROS2 node wrapping MATLAB Coder generated IK solver - Implementation
 * 
 * This node subscribes to end-effector trajectory commands and publishes
 * joint-space trajectories for the 9-DOF mobile manipulator.
 * 
 * Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
 */

#include "gik9dof_solver_node.h"

// MATLAB Coder generated solver (Class-based architecture)
#include "GIKSolver.h"

// MATLAB Coder generated velocity controllers
#include "velocity_controller/holisticVelocityController.h"
#include "velocity_controller/holisticVelocityController_initialize.h"
#include "velocity_controller/holisticVelocityController_terminate.h"

#include "purepursuit/purePursuitVelocityController.h"
#include "purepursuit/purePursuitVelocityController_initialize.h"
#include "purepursuit/purePursuitVelocityController_terminate.h"

// Stage B factory (creates controller without namespace conflicts)
#include "stage_b_factory.hpp"

using namespace std::chrono_literals;

GIK9DOFSolverNode::GIK9DOFSolverNode() : Node("gik9dof_solver_node")
{
        // Declare parameters
        this->declare_parameter("control_mode", "holistic");  // "holistic" or "staged"
        this->declare_parameter("control_rate", 10.0);  // Hz
        this->declare_parameter("max_solve_time", 0.05);  // 50ms
        this->declare_parameter("max_solver_iterations", 1000);  // Max solver iterations (matches MATLAB wrapper hardcoded value)
        
        // Distance constraints (20 total) - NEW 7-parameter interface
        // Default: 5 active constraints matching MATLAB validation tests
        std::vector<int64_t> default_body_indices = {
            9, 9, 9, 7, 6,  // Active: gripper, gripper, gripper, link5, link4
            9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9  // Disabled placeholders
        };
        std::vector<int64_t> default_ref_body_indices = {
            1, 0, 2, 1, 1,  // Active: chassis, base, arm_base, chassis, chassis
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // Disabled placeholders
        };
        std::vector<double> default_lower_bounds(20, 0.05);  // All 5cm minimum
        std::vector<double> default_upper_bounds(20, 10.0);  // All 10m maximum (effectively infinite)
        std::vector<double> default_weights = {
            1.0, 1.0, 1.0, 1.0, 1.0,  // Active constraints
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0   // Disabled constraints
        };
        
        this->declare_parameter("dist_body_indices", default_body_indices);
        this->declare_parameter("dist_ref_body_indices", default_ref_body_indices);
        this->declare_parameter("dist_lower_bounds", default_lower_bounds);
        this->declare_parameter("dist_upper_bounds", default_upper_bounds);
        this->declare_parameter("dist_weights", default_weights);
        
        this->declare_parameter("publish_diagnostics", true);
        this->declare_parameter("use_warm_start", true);  // Use previous solution as initial guess
        
        // Velocity controller mode selection
        // 0 = Legacy 5-point differentiation (baseline)
        // 1 = Simple heading controller (P + feedforward)
        // 2 = Pure Pursuit path following (lookahead-based)
        this->declare_parameter("velocity_control_mode", 2);
        
        // Simple heading controller parameters (mode 1)
        this->declare_parameter("vel_ctrl.track", 0.674);         // Wheel track width (m)
        this->declare_parameter("vel_ctrl.vwheel_max", 2.0);      // Max wheel speed (m/s)
        this->declare_parameter("vel_ctrl.vx_max", 1.0);          // Max forward velocity (m/s)
        this->declare_parameter("vel_ctrl.w_max", 2.0);           // Max yaw rate (rad/s)
        this->declare_parameter("vel_ctrl.yaw_kp", 2.0);          // Heading error P gain
        this->declare_parameter("vel_ctrl.yaw_kff", 0.9);         // Yaw rate feedforward gain
        
        // Pure Pursuit controller parameters (mode 2)
        this->declare_parameter("purepursuit.lookahead_base", 0.8);
        this->declare_parameter("purepursuit.lookahead_vel_gain", 0.3);
        this->declare_parameter("purepursuit.lookahead_time_gain", 0.1);
        this->declare_parameter("purepursuit.vx_nominal", 1.0);
        this->declare_parameter("purepursuit.vx_max", 1.5);
        this->declare_parameter("purepursuit.vx_min", -1.0);  // Max reverse velocity
        this->declare_parameter("purepursuit.wz_max", 2.0);
        this->declare_parameter("purepursuit.track", 0.674);
        this->declare_parameter("purepursuit.vwheel_max", 2.0);
        this->declare_parameter("purepursuit.waypoint_spacing", 0.15);
        this->declare_parameter("purepursuit.path_buffer_size", 30.0);
        this->declare_parameter("purepursuit.goal_tolerance", 0.2);
        this->declare_parameter("purepursuit.interp_spacing", 0.05);
        
        // Staged control parameters
        this->declare_parameter("staged.stage_b_mode", 1);  // 1=Pure Hybrid A*, 2=GIK-Assisted
        this->declare_parameter("staged.planner.max_iterations", 1000);
        this->declare_parameter("staged.planner.timeout_ms", 50.0);
        this->declare_parameter("staged.planner.goal_tolerance_xy", 0.15);
        this->declare_parameter("staged.planner.goal_tolerance_theta", 0.15);
        this->declare_parameter("staged.planner.use_holonomic", false);
        this->declare_parameter("staged.stage_a_timeout", 10.0);
        this->declare_parameter("staged.stage_b_timeout", 30.0);
        this->declare_parameter("staged.stage_c_timeout", 60.0);
        
        // Get parameters
        std::string control_mode_str = this->get_parameter("control_mode").as_string();
        control_mode_ = (control_mode_str == "staged") ? ControlMode::STAGED : ControlMode::HOLISTIC;
        current_stage_ = ControlStage::STAGE_A;  // Always start at Stage A
        stage_b_enabled_ = false;
        stage_b_controller_ = nullptr;  // Initialize to null
        
        control_rate_ = this->get_parameter("control_rate").as_double();
        max_solve_time_ = this->get_parameter("max_solve_time").as_double();
        max_solver_iterations_ = this->get_parameter("max_solver_iterations").as_int();
        
        // Read distance constraint parameters and copy to member arrays
        auto body_indices = this->get_parameter("dist_body_indices").as_integer_array();
        auto ref_body_indices = this->get_parameter("dist_ref_body_indices").as_integer_array();
        auto lower_bounds = this->get_parameter("dist_lower_bounds").as_double_array();
        auto upper_bounds = this->get_parameter("dist_upper_bounds").as_double_array();
        auto weights = this->get_parameter("dist_weights").as_double_array();
        
        // Copy to member arrays with type conversion
        for (size_t i = 0; i < 20; i++) {
            dist_body_indices_[i] = static_cast<int>(body_indices[i]);
            dist_ref_body_indices_[i] = static_cast<int>(ref_body_indices[i]);
            dist_lower_bounds_[i] = lower_bounds[i];
            dist_upper_bounds_[i] = upper_bounds[i];
            dist_weights_[i] = weights[i];
        }
        
        publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();
        use_warm_start_ = this->get_parameter("use_warm_start").as_bool();
        
        // Get velocity controller mode
        velocity_control_mode_ = this->get_parameter("velocity_control_mode").as_int();
        
        // Initialize state
        current_config_.resize(9, 0.0);
        target_config_.resize(9, 0.0);
        
        // Initialize velocity controllers based on mode
        if (velocity_control_mode_ == 1) {
            // Simple heading controller
            gik9dof_velocity::holisticVelocityController_initialize();
            
            // Get controller parameters
            vel_params_.track = this->get_parameter("vel_ctrl.track").as_double();
            vel_params_.Vwheel_max = this->get_parameter("vel_ctrl.vwheel_max").as_double();
            vel_params_.Vx_max = this->get_parameter("vel_ctrl.vx_max").as_double();
            vel_params_.W_max = this->get_parameter("vel_ctrl.w_max").as_double();
            vel_params_.yawKp = this->get_parameter("vel_ctrl.yaw_kp").as_double();
            vel_params_.yawKff = this->get_parameter("vel_ctrl.yaw_kff").as_double();
            
            // Initialize state
            vel_state_.prev.x = 0.0;
            vel_state_.prev.y = 0.0;
            vel_state_.prev.theta = 0.0;
            vel_state_.prev.t = 0.0;
            vel_controller_initialized_ = false;
            
        } else if (velocity_control_mode_ == 2) {
            // Pure Pursuit controller
            gik9dof_purepursuit::purePursuitVelocityController_initialize();
            
            // Get Pure Pursuit parameters
            pp_params_.lookaheadBase = this->get_parameter("purepursuit.lookahead_base").as_double();
            pp_params_.lookaheadVelGain = this->get_parameter("purepursuit.lookahead_vel_gain").as_double();
            pp_params_.lookaheadTimeGain = this->get_parameter("purepursuit.lookahead_time_gain").as_double();
            pp_params_.vxNominal = this->get_parameter("purepursuit.vx_nominal").as_double();
            pp_params_.vxMax = this->get_parameter("purepursuit.vx_max").as_double();
            pp_params_.vxMin = this->get_parameter("purepursuit.vx_min").as_double();  // Bidirectional support
            pp_params_.wzMax = this->get_parameter("purepursuit.wz_max").as_double();
            pp_params_.track = this->get_parameter("purepursuit.track").as_double();
            pp_params_.vwheelMax = this->get_parameter("purepursuit.vwheel_max").as_double();
            pp_params_.waypointSpacing = this->get_parameter("purepursuit.waypoint_spacing").as_double();
            pp_params_.pathBufferSize = this->get_parameter("purepursuit.path_buffer_size").as_double();
            pp_params_.goalTolerance = this->get_parameter("purepursuit.goal_tolerance").as_double();
            pp_params_.interpSpacing = this->get_parameter("purepursuit.interp_spacing").as_double();
            
            // Initialize Pure Pursuit state (30 waypoint buffer)
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
            pp_controller_initialized_ = false;
        }
        
        // Initialize Stage B controller if in staged mode
        if (control_mode_ == ControlMode::STAGED) {
            // Create Stage B parameters
            gik9dof::StageBParams params;
            params.mode = gik9dof::StageBMode::HybridAStar;
            params.chassis_max_vel_x = 0.5;
            params.chassis_max_vel_theta = 0.5;
            params.chassis_acc_x = 0.5;
            params.chassis_acc_theta = 0.5;
            params.lookahead_distance = 0.3;
            params.goal_tolerance_xy = 0.05;
            params.goal_tolerance_theta = 0.1;
            params.planner_time_limit_ms = 50.0;
            params.velocity_control_mode = velocity_control_mode_;
            
            // Pass Pure Pursuit params if mode 2
            if (velocity_control_mode_ == 2) {
                params.pp_params = &pp_params_;
            } else {
                params.pp_params = nullptr;
            }
            
            // Create Stage B controller via factory (returns raw pointer)
            stage_b_controller_ = gik9dof::createStageBController(this, params);
            
            stage_b_enabled_ = true;
            
            int stage_b_mode = this->get_parameter("staged.stage_b_mode").as_int();
            RCLCPP_INFO(this->get_logger(), 
                "Staged control mode enabled - Starting at Stage A (arm ramp-up)");
            RCLCPP_INFO(this->get_logger(), 
                "Stage B mode: %s", stage_b_mode == 1 ? "Pure Hybrid A*" : "GIK-Assisted");
        } else {
            stage_b_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "Holistic control mode enabled");
        }
        
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
        
        base_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        if (publish_diagnostics_) {
            diagnostics_pub_ = this->create_publisher<gik9dof_msgs::msg::SolverDiagnostics>(
                "/gik9dof/solver_diagnostics", 10);
        }
        
        // Control timer
        auto timer_period = std::chrono::duration<double>(1.0 / control_rate_);
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&GIK9DOFSolverNode::controlLoop, this));
        
        // Initialize MATLAB solver (class-based architecture)
        matlab_solver_ = std::make_unique<gik9dof::GIKSolver>();
        
        RCLCPP_INFO(this->get_logger(), "GIK9DOF Solver Node initialized");
        RCLCPP_INFO(this->get_logger(), "MATLAB solver interface: codegen_inuse (class-based, 20 constraints)");
        RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_);
        RCLCPP_INFO(this->get_logger(), "Max solve time: %.0f ms", max_solve_time_ * 1000.0);
        RCLCPP_INFO(this->get_logger(), "Max solver iterations: %d", max_solver_iterations_);
        RCLCPP_INFO(this->get_logger(), "Warm-start optimization: %s", use_warm_start_ ? "enabled" : "disabled");
        
        // Log velocity controller mode
        const char* mode_names[] = {
            "Legacy (5-point differentiation)", 
            "Simple Heading Controller (P + feedforward)", 
            "Pure Pursuit (lookahead path following)"
        };
        RCLCPP_INFO(this->get_logger(), "Velocity controller mode: %d - %s", 
            velocity_control_mode_, mode_names[velocity_control_mode_]);
        
        if (velocity_control_mode_ == 1) {
            RCLCPP_INFO(this->get_logger(), "  Heading controller params: track=%.3f, Vx_max=%.2f, W_max=%.2f, Kp=%.2f, Kff=%.2f",
                vel_params_.track, vel_params_.Vx_max, vel_params_.W_max, vel_params_.yawKp, vel_params_.yawKff);
        } else if (velocity_control_mode_ == 2) {
            RCLCPP_INFO(this->get_logger(), "  Pure Pursuit params:");
            RCLCPP_INFO(this->get_logger(), "    Lookahead: base=%.2f, vel_gain=%.2f, time_gain=%.2f",
                pp_params_.lookaheadBase, pp_params_.lookaheadVelGain, pp_params_.lookaheadTimeGain);
            RCLCPP_INFO(this->get_logger(), "    Velocity: nominal=%.2f m/s, max=%.2f m/s, wz_max=%.2f rad/s",
                pp_params_.vxNominal, pp_params_.vxMax, pp_params_.wzMax);
            RCLCPP_INFO(this->get_logger(), "    Path: buffer=%d waypoints, spacing=%.2f m, tolerance=%.2f m",
                (int)pp_params_.pathBufferSize, pp_params_.waypointSpacing, pp_params_.goalTolerance);
        }
        
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  - /motion_target/target_joint_state_arm_left (6 arm joints)");
        RCLCPP_INFO(this->get_logger(), "  - /cmd_vel (base velocities: vx, wz)");
}

GIK9DOFSolverNode::~GIK9DOFSolverNode()
{
    // Destroy Stage B controller (if created)
    if (stage_b_controller_) {
        gik9dof::destroyStageBController(stage_b_controller_);
        stage_b_controller_ = nullptr;
    }
    
    // Cleanup velocity controllers
    if (velocity_control_mode_ == 1) {
        gik9dof_velocity::holisticVelocityController_terminate();
        RCLCPP_INFO(this->get_logger(), "Simple heading controller terminated");
    } else if (velocity_control_mode_ == 2) {
        gik9dof_purepursuit::purePursuitVelocityController_terminate();
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit controller terminated");
    }
}

void GIK9DOFSolverNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "jointStateCallback FIRED: received %zu joints", msg->position.size());
        // Update arm joint states (6 DOF)
        // Assumes message contains joints in order: left_arm_joint1-6
        if (msg->position.size() >= 6) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < 6; ++i) {
                current_config_[3 + i] = msg->position[i];  // Arm starts at index 3
            }
            arm_state_received_ = true;
            RCLCPP_INFO_ONCE(this->get_logger(), "✅ Arm state received and set!");
        }
    }
    
void GIK9DOFSolverNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "odomCallback FIRED: x=%.2f, y=%.2f", 
                            msg->pose.pose.position.x, msg->pose.pose.position.y);
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
        RCLCPP_INFO_ONCE(this->get_logger(), "✅ Base odom received and set!");
    }
    
void GIK9DOFSolverNode::trajectoryCallback(const gik9dof_msgs::msg::EndEffectorTrajectory::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_ = msg;
        trajectory_sequence_ = msg->sequence_id;
        
        RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu waypoints, seq=%u",
                    msg->waypoints.size(), msg->sequence_id);
    }
    
void GIK9DOFSolverNode::controlLoop()
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
                
                // Check if trajectory completed
                if (trajectory_complete_) {
                    // Trajectory is complete - publish zero velocity and hold arm
                    publishZeroVelocity();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Trajectory complete. Chassis stopped, arm holding position.");
                    return;
                }
                
                // Reset waypoint index if new trajectory received
                static uint32_t last_sequence_id = 0;
                if (trajectory_sequence_ != last_sequence_id) {
                    current_waypoint_index_ = 0;
                    trajectory_complete_ = false;
                    last_sequence_id = trajectory_sequence_;
                    RCLCPP_INFO(this->get_logger(), "New trajectory seq=%u, reset to waypoint 0", 
                                trajectory_sequence_);
                }
                
                // Check if we have more waypoints to track
                if (current_waypoint_index_ < current_trajectory_->waypoints.size()) {
                    target_pose = current_trajectory_->waypoints[current_waypoint_index_].pose;
                    has_target = true;
                    
                    // Check if close enough to current waypoint to advance
                    double distance = computeEndEffectorDistance(target_pose);
                    
                    if (distance < waypoint_tolerance_) {
                        RCLCPP_INFO(this->get_logger(), 
                                    "Waypoint %zu reached (dist=%.3f m). Advancing...", 
                                    current_waypoint_index_, distance);
                        current_waypoint_index_++;
                        
                        // Check if this was the last waypoint
                        if (current_waypoint_index_ >= current_trajectory_->waypoints.size()) {
                            if (current_trajectory_->is_final_segment) {
                                RCLCPP_INFO(this->get_logger(), 
                                            "🎯 Final waypoint reached! Trajectory complete. Stopping chassis.");
                                trajectory_complete_ = true;
                                publishZeroVelocity();
                                return;
                            } else {
                                // Not final segment - hold at last waypoint
                                current_waypoint_index_--;
                                RCLCPP_INFO(this->get_logger(), 
                                            "Last waypoint reached but not final segment. Holding position.");
                            }
                        }
                    }
                } else {
                    // Should not happen, but handle gracefully
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Waypoint index out of bounds. Resetting to last waypoint.");
                    current_waypoint_index_ = current_trajectory_->waypoints.size() - 1;
                    target_pose = current_trajectory_->waypoints[current_waypoint_index_].pose;
                    has_target = true;
                }
            }
        }
        
        if (!has_target) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No trajectory target available");
            return;
        }
        
        // ========== STATE MACHINE CONTROL ==========
        if (control_mode_ == ControlMode::STAGED) {
            executeStagedControl(target_pose);
        } else {
            // Holistic mode: Direct 9-DOF GIK solve
            executeHolisticControl(target_pose);
        }
    }
    
void GIK9DOFSolverNode::executeStagedControl(const geometry_msgs::msg::Pose& target_pose)
    {
        switch (current_stage_) {
            case ControlStage::STAGE_A:
                // Stage A: Ramp arm to home configuration
                executeStageA(target_pose);
                break;
                
            case ControlStage::STAGE_B:
                // Stage B: Chassis planning and execution
                executeStageB(target_pose);
                break;
                
            case ControlStage::STAGE_C:
                // Stage C: Whole-body tracking
                executeStageC(target_pose);
                break;
        }
    }
    
void GIK9DOFSolverNode::executeStageA(const geometry_msgs::msg::Pose& target_pose)
    {
        // Stage A: Ramp arm to home position
        // TODO: Implement arm ramping logic (smooth trajectory to home config)
        
        // For now, check if arm is close to home configuration
        bool arm_at_home = checkArmAtHome();
        
        if (arm_at_home) {
            // Get current and goal base poses as Eigen::Vector3d (x, y, theta)
            Eigen::Vector3d current_base_pose;
            Eigen::Vector3d goal_base_pose;
            std::vector<double> arm_config(6);
            
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_base_pose << current_config_[0], current_config_[1], current_config_[2];
                
                // Extract arm configuration
                for (int i = 0; i < 6; i++) {
                    arm_config[i] = current_config_[3 + i];
                }
            }
            
            // Convert target_pose to base pose (x, y, theta)
            goal_base_pose << target_pose.position.x, target_pose.position.y, 0.0;
            
            // Extract yaw from quaternion
            double qw = target_pose.orientation.w;
            double qx = target_pose.orientation.x;
            double qy = target_pose.orientation.y;
            double qz = target_pose.orientation.z;
            goal_base_pose(2) = std::atan2(2.0 * (qw * qz + qx * qy), 
                                           1.0 - 2.0 * (qy * qy + qz * qz));
            
            // Activate Stage B controller using wrapper function
            if (stage_b_controller_) {
                gik9dof::stageBActivate(stage_b_controller_, current_base_pose, 
                                       goal_base_pose, arm_config);
            }
            
            // Transition to Stage B
            current_stage_ = ControlStage::STAGE_B;
            RCLCPP_INFO(this->get_logger(), "Stage A → B: Arm at home, starting chassis planning");
        } else {
            // Continue ramping arm
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Stage A: Ramping arm to home configuration...");
        }
    }
    
void GIK9DOFSolverNode::executeStageB(const geometry_msgs::msg::Pose& target_pose)
    {
        // Stage B: Chassis planning and execution
        if (!stage_b_controller_) {
            RCLCPP_ERROR(this->get_logger(), "Stage B controller not initialized!");
            return;
        }
        
        // Get current base pose as Eigen::Vector3d and arm config
        Eigen::Vector3d current_base_pose;
        std::vector<double> current_arm_config(6);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_base_pose << current_config_[0], current_config_[1], current_config_[2];
            
            // Extract arm configuration
            for (int i = 0; i < 6; i++) {
                current_arm_config[i] = current_config_[3 + i];
            }
        }
        
        // Execute one step of Stage B controller using wrapper function
        geometry_msgs::msg::Twist base_cmd;
        sensor_msgs::msg::JointState arm_cmd;
        gik9dof::stageBExecuteStep(stage_b_controller_, 
                                   current_base_pose,
                                   current_arm_config,
                                   base_cmd, 
                                   arm_cmd);
        
        // Publish base command
        base_cmd_pub_->publish(base_cmd);
        
        // Check for transition to Stage C using wrapper function
        if (gik9dof::stageBChassisReachedGoal(stage_b_controller_, current_base_pose)) {
            gik9dof::stageBDeactivate(stage_b_controller_);
            current_stage_ = ControlStage::STAGE_C;
            RCLCPP_INFO(this->get_logger(), "Stage B → C: Chassis at goal, starting whole-body tracking");
        }
    }
    
void GIK9DOFSolverNode::executeStageC(const geometry_msgs::msg::Pose& target_pose)
    {
        // Stage C: Whole-body tracking (same as holistic mode)
        executeHolisticControl(target_pose);
    }
    
void GIK9DOFSolverNode::executeHolisticControl(const geometry_msgs::msg::Pose& target_pose)
    {
        // Solve IK
        auto solve_start = this->now();
        bool solve_success = solveIK(target_pose);
        auto solve_end = this->now();
        double solve_time_ms = (solve_end - solve_start).seconds() * 1000.0;
        
        if (solve_success) {
            publishJointCommand();
            publishBaseCommand();  // NEW: Publish base velocity commands (vx, wz)
            
            if (publish_diagnostics_) {
                publishDiagnostics(solve_time_ms, target_pose);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "IK solve failed (%.2f ms)", solve_time_ms);
        }
    }
    
bool GIK9DOFSolverNode::checkArmAtHome()
    {
        // Define home configuration for arm (joints 3-8)
        // TODO: Load from parameters
        std::vector<double> home_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 6 arm joints
        
        double tolerance = 0.1;  // radians
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < 6; i++) {
            if (std::abs(current_config_[i + 3] - home_config[i]) > tolerance) {
                return false;
            }
        }
        
        return true;
    }
    
double GIK9DOFSolverNode::extractYaw(const geometry_msgs::msg::Quaternion& q)
    {
        // Extract yaw from quaternion using standard conversion
        // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

double GIK9DOFSolverNode::computeEndEffectorDistance(const geometry_msgs::msg::Pose& target_pose)
    {
        // Simplified approach: Use the pose violation metric from the last IK solution
        // The solver already computes position error, which is the distance we need
        // This avoids needing forward kinematics
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Use the position error from the last solver run
        // This represents how far the achieved EE pose is from the target
        // If position_error < tolerance, we've reached the waypoint
        return last_position_error_;
    }

void GIK9DOFSolverNode::publishZeroVelocity()
    {
        // Create and publish zero velocity command to stop the chassis
        geometry_msgs::msg::Twist zero_cmd;
        zero_cmd.linear.x = 0.0;
        zero_cmd.linear.y = 0.0;
        zero_cmd.linear.z = 0.0;
        zero_cmd.angular.x = 0.0;
        zero_cmd.angular.y = 0.0;
        zero_cmd.angular.z = 0.0;
        
        base_cmd_pub_->publish(zero_cmd);
        
        RCLCPP_INFO(this->get_logger(), "Published zero velocity - chassis stopped");
    }
    
bool GIK9DOFSolverNode::solveIK(const geometry_msgs::msg::Pose& target_pose)
    {
        // Prepare current configuration (9 DOF)
        // Use previous successful solution as initial guess (warm-start optimization)
        double q_current[9];
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Warm-start: Use previous target as initial guess if it was successful
            // This dramatically improves convergence for smooth trajectories
            if (use_warm_start_ && last_solver_status_ == "success") {
                std::copy(target_config_.begin(), target_config_.end(), q_current);
                RCLCPP_DEBUG(this->get_logger(), "Using warm-start from previous solution");
            } else {
                std::copy(current_config_.begin(), current_config_.end(), q_current);
                RCLCPP_DEBUG(this->get_logger(), "Using current config (cold-start)");
            }
        }
        
        // Convert ROS Pose to 4x4 homogeneous transform (column-major for MATLAB)
        double target_matrix[16];
        
        // Create quaternion and extract rotation matrix
        Eigen::Quaterniond q(target_pose.orientation.w,
                             target_pose.orientation.x,
                             target_pose.orientation.y,
                             target_pose.orientation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();
        
        // Fill column-major 4x4 matrix
        // Column 0: [R(0,0), R(1,0), R(2,0), 0]
        target_matrix[0] = R(0, 0);
        target_matrix[1] = R(1, 0);
        target_matrix[2] = R(2, 0);
        target_matrix[3] = 0.0;
        
        // Column 1: [R(0,1), R(1,1), R(2,1), 0]
        target_matrix[4] = R(0, 1);
        target_matrix[5] = R(1, 1);
        target_matrix[6] = R(2, 1);
        target_matrix[7] = 0.0;
        
        // Column 2: [R(0,2), R(1,2), R(2,2), 0]
        target_matrix[8] = R(0, 2);
        target_matrix[9] = R(1, 2);
        target_matrix[10] = R(2, 2);
        target_matrix[11] = 0.0;
        
        // Column 3: [x, y, z, 1]
        target_matrix[12] = target_pose.position.x;
        target_matrix[13] = target_pose.position.y;
        target_matrix[14] = target_pose.position.z;
        target_matrix[15] = 1.0;
        
        // Output arrays
        double q_next[9];
        gik9dof::struct0_T solver_info;
        
        // Add timeout safety: track solve start time
        auto solve_start_time = std::chrono::steady_clock::now();
        rclcpp::Time solve_start_ros = this->now();
        
        // Call MATLAB-generated solver (class-based architecture, 20 constraints)
        // Note: The solver itself has MaxTime=50ms configured in MATLAB code
        matlab_solver_->gik9dof_codegen_inuse_solveGIKStepWrapper(
            q_current,                // Current joint configuration [9]
            target_matrix,            // Target end-effector pose (4x4 homogeneous, column-major) [16]
            dist_body_indices_,       // Body indices for distance constraints [20]
            dist_ref_body_indices_,   // Reference body indices [20]
            dist_lower_bounds_,       // Lower distance bounds (meters) [20]
            dist_upper_bounds_,       // Upper distance bounds (meters) [20]
            dist_weights_,            // Constraint weights (0.0=disabled, >0.0=enabled) [20]
            q_next,                   // OUTPUT: Next joint configuration [9]
            &solver_info              // OUTPUT: Solver diagnostics (struct0_T)
        );
        
        // Check solve time and warn if exceeded expected duration
        auto solve_end_time = std::chrono::steady_clock::now();
        rclcpp::Time solve_end_ros = this->now();
        auto solve_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            solve_end_time - solve_start_time).count();
        
        if (solve_duration_ms > 50) {
            RCLCPP_WARN(this->get_logger(), 
                       "IK solve exceeded 50ms timeout: actual=%ld ms (solver should auto-terminate)",
                       solve_duration_ms);
        }
        
        // Extract status string
        std::string status_str(solver_info.Status.data, 
                              solver_info.Status.size[1]);
        
        // Enhanced diagnostics: extract constraint violations
        // ConstraintViolations is an array of struct1_T[3] with fields: Type (string), Violation (array)
        double pose_violation = 0.0;
        double position_error = 0.0;
        double orientation_error = 0.0;
        bool joint_limit_violation = false;
        bool distance_constraint_met = true;
        std::vector<double> all_violations;
        
        // Per-constraint distance violation tracking (20 constraints)
        double dist_violations[20] = {0.0};
        bool dist_constraint_met_array[20] = {false};
        int num_active_constraints = 0;
        int num_violated_constraints = 0;
        
        // Count active constraints (weight > 0)
        for (int i = 0; i < 20; i++) {
            if (dist_weights_[i] > 0.0) {
                num_active_constraints++;
            }
        }
        
        // Iterate through the 3 constraint violation structs
        for (int cv_idx = 0; cv_idx < 3; cv_idx++) {
            const auto& cv = solver_info.ConstraintViolations[cv_idx];
            std::string type_str(cv.Type.data, cv.Type.size[1]);
            
            // Extract violation values from the array
            int num_violations = cv.Violation.size(0);
            for (int i = 0; i < num_violations; i++) {
                double val = cv.Violation[i];
                all_violations.push_back(val);
                
                // Parse based on constraint type
                if (type_str.find("pose") != std::string::npos) {
                    // Pose constraints: first 3 are position, next 3 are orientation
                    if (i < 3) {
                        position_error = std::max(position_error, std::abs(val));
                    } else if (i < 6) {
                        orientation_error = std::max(orientation_error, std::abs(val));
                    }
                } else if (type_str.find("joint") != std::string::npos) {
                    // Joint limit constraints
                    if (std::abs(val) > 1e-6) {
                        joint_limit_violation = true;
                    }
                } else if (type_str.find("distance") != std::string::npos) {
                    // Distance constraints - extract per-constraint violations
                    // Violation array contains violations for each of the 20 constraints
                    if (i < 20) {
                        dist_violations[i] = val;
                        
                        // Check if this constraint is active and violated
                        if (dist_weights_[i] > 0.0) {  // Only check active constraints
                            bool is_met = (std::abs(val) < 1e-3);  // Threshold for "met"
                            dist_constraint_met_array[i] = is_met;
                            
                            if (!is_met) {
                                num_violated_constraints++;
                                distance_constraint_met = false;  // Global flag
                            }
                        } else {
                            // Inactive constraints are considered "met"
                            dist_constraint_met_array[i] = true;
                        }
                    }
                }
            }
        }
        
        pose_violation = std::max(position_error, orientation_error);

        
        // Store result and solver info
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            std::copy(q_next, q_next + 9, target_config_.begin());
            last_solver_iterations_ = static_cast<int>(solver_info.Iterations);
            last_solver_status_ = status_str;
            last_exit_flag_ = static_cast<int>(solver_info.ExitFlag);
            last_random_restarts_ = static_cast<int>(solver_info.NumRandomRestarts);
            last_pose_violation_ = pose_violation;
            last_position_error_ = position_error;
            last_orientation_error_ = orientation_error;
            last_joint_limit_violation_ = joint_limit_violation;
            last_distance_constraint_met_ = distance_constraint_met;
            last_constraint_violations_ = all_violations;
            
            // Store per-constraint distance violations
            std::copy(dist_violations, dist_violations + 20, last_dist_violations_);
            std::copy(dist_constraint_met_array, dist_constraint_met_array + 20, last_dist_constraint_met_);
            last_num_active_constraints_ = num_active_constraints;
            last_num_violated_constraints_ = num_violated_constraints;
            
            last_solve_start_ = solve_start_ros;
            last_solve_end_ = solve_end_ros;
        }
        
        // Log detailed diagnostics on failure or with DEBUG level
        if (status_str != "success") {
            RCLCPP_WARN(this->get_logger(), 
                       "Solver failed: status='%s', iterations=%d, exit_flag=%d, time=%ld ms",
                       status_str.c_str(), last_solver_iterations_, last_exit_flag_, solve_duration_ms);
            RCLCPP_WARN(this->get_logger(), 
                       "  Violations: pose=%.4f (pos=%.4f, ori=%.4f), joint_limits=%s, dist_constraint=%s (%d/%d active violated), restarts=%d",
                       pose_violation, position_error, orientation_error,
                       joint_limit_violation ? "YES" : "no",
                       distance_constraint_met ? "yes" : "NO",
                       num_violated_constraints, num_active_constraints,
                       last_random_restarts_);
        } else {
            RCLCPP_DEBUG(this->get_logger(), 
                        "Solver success: iter=%d, time=%ld ms, pose_err=%.4f, dist_constraints=%d/%d active OK, restarts=%d",
                        last_solver_iterations_, solve_duration_ms, pose_violation, 
                        num_active_constraints - num_violated_constraints, num_active_constraints,
                        last_random_restarts_);
        }
        
        return (status_str == "success");
    }
    
void GIK9DOFSolverNode::publishJointCommand()
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
    
void GIK9DOFSolverNode::publishBaseCommand()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // ═══════════════════════════════════════════════════════════════════════
        // 3-WAY RUNTIME SWITCH: Velocity Control Mode Selection
        // ═══════════════════════════════════════════════════════════════════════
        
        if (velocity_control_mode_ == 2) {
            // ═══════════════════════════════════════════════════════════════
            // MODE 2: Pure Pursuit Path Following Controller
            // ═══════════════════════════════════════════════════════════════
            
            // Extract reference from target configuration
            double refX = target_config_[0];      // Base x position (m)
            double refY = target_config_[1];      // Base y position (m)
            double refTheta = target_config_[2];  // Base orientation (rad)
            double refTime = this->now().seconds();
            
            // Current pose estimate (from odometry)
            double estX = current_config_[0];
            double estY = current_config_[1];
            double estYaw = current_config_[2];
            
            // Call MATLAB Coder generated Pure Pursuit controller
            double Vx, Wz;
            gik9dof_purepursuit::struct1_T newState;
            
            gik9dof_purepursuit::purePursuitVelocityController(
                refX, refY, refTheta, refTime,
                estX, estY, estYaw,
                &pp_params_,
                &pp_state_,
                &Vx, &Wz,
                &newState
            );
            
            // Update state for next iteration
            pp_state_ = newState;
            pp_controller_initialized_ = true;
            
            // Publish velocity command
            msg.linear.x = Vx;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = Wz;
            
            base_cmd_pub_->publish(msg);
            
            // Debug logging (throttled)
            static int log_counter_pp = 0;
            if (++log_counter_pp >= 50) {  // Log every 5 seconds at 10Hz
                RCLCPP_DEBUG(this->get_logger(), 
                    "Pure Pursuit: Vx=%.3f m/s, Wz=%.3f rad/s | Buffer: %d waypoints | ref: x=%.2f, y=%.2f, θ=%.2f", 
                    Vx, Wz, (int)pp_state_.numWaypoints, refX, refY, refTheta);
                log_counter_pp = 0;
            }
            
        } else if (velocity_control_mode_ == 1) {
            // ═══════════════════════════════════════════════════════════════
            // MODE 1: Simple Heading Controller (P + feedforward)
            // ═══════════════════════════════════════════════════════════════
            
            // Extract reference from target configuration
            double refX = target_config_[0];      // Base x position (m)
            double refY = target_config_[1];      // Base y position (m)
            double refTheta = target_config_[2];  // Base orientation (rad)
            double refTime = this->now().seconds();
            
            // Current pose estimate (from odometry)
            double estX = current_config_[0];
            double estY = current_config_[1];
            double estYaw = current_config_[2];
            
            // Call MATLAB Coder generated velocity controller
            double Vx, Wz;
            gik9dof_velocity::struct1_T newState;
            
            gik9dof_velocity::holisticVelocityController(
                refX, refY, refTheta, refTime,
                estX, estY, estYaw,
                &vel_params_,
                &vel_state_,
                &Vx, &Wz,
                &newState
            );
            
            // Update state for next iteration
            vel_state_ = newState;
            vel_controller_initialized_ = true;
            
            // Publish velocity command
            msg.linear.x = Vx;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = Wz;
            
            base_cmd_pub_->publish(msg);
            
            // Debug logging (throttled)
            static int log_counter_heading = 0;
            if (++log_counter_heading >= 50) {  // Log every 5 seconds at 10Hz
                RCLCPP_DEBUG(this->get_logger(), 
                    "Heading controller: Vx=%.3f m/s, Wz=%.3f rad/s (ref: x=%.2f, y=%.2f, θ=%.2f)", 
                    Vx, Wz, refX, refY, refTheta);
                log_counter_heading = 0;
            }
            
        } else {
            // ═══════════════════════════════════════════════════════════════
            // MODE 0: Legacy 5-point finite differentiation (open-loop)
            // ═══════════════════════════════════════════════════════════════
        
        // Add current target to history
        auto now = this->now();
        config_history_.push_back(target_config_);
        time_history_.push_back(now);
        
        // Maintain buffer size of 5
        if (config_history_.size() > HISTORY_SIZE) {
            config_history_.pop_front();
            time_history_.pop_front();
        }
        
        // Need at least 5 points for 5-point stencil, fall back to simpler methods if fewer
        size_t n = config_history_.size();
        
        if (n < 2) {
            // Not enough history - send zero velocity
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
            
            base_cmd_pub_->publish(msg);
            return;
        }
        
        // Compute velocities using 5-point stencil (or best available)
        double vx_world, vy_world, wz;
        
        if (n >= 5) {
            // Use 5-point central difference: O(h^4) accuracy
            // f'(x) ≈ [-f(x-2h) + 8f(x-h) - 8f(x+h) + f(x+2h)] / (12h)
            // For real-time: use backward stencil at current point
            // f'(x) ≈ [3f(x) - 16f(x-h) + 36f(x-2h) - 48f(x-3h) + 25f(x-4h)] / (12h)
            
            // Get time steps (assume roughly uniform for simplicity)
            double h = 0.0;
            for (size_t i = 1; i < n; ++i) {
                h += (time_history_[i] - time_history_[i-1]).seconds();
            }
            h /= (n - 1);  // Average time step
            
            if (h < 1e-6 || h > 1.0) {
                // Invalid time step - use fallback
                vx_world = vy_world = wz = 0.0;
            } else {
                // 5-point backward difference
                // Indices: [0]=oldest, [4]=newest
                const auto& q0 = config_history_[0];  // x-4h
                const auto& q1 = config_history_[1];  // x-3h
                const auto& q2 = config_history_[2];  // x-2h
                const auto& q3 = config_history_[3];  // x-h
                const auto& q4 = config_history_[4];  // x (current)
                
                // x-velocity: [25*q4 - 48*q3 + 36*q2 - 16*q1 + 3*q0] / (12*h)
                vx_world = (25.0*q4[0] - 48.0*q3[0] + 36.0*q2[0] - 16.0*q1[0] + 3.0*q0[0]) / (12.0 * h);
                
                // y-velocity
                vy_world = (25.0*q4[1] - 48.0*q3[1] + 36.0*q2[1] - 16.0*q1[1] + 3.0*q0[1]) / (12.0 * h);
                
                // Angular velocity (with angle wrapping)
                // First unwrap angles to avoid discontinuities
                std::vector<double> theta_unwrapped(5);
                theta_unwrapped[0] = q0[2];
                for (size_t i = 1; i < 5; ++i) {
                    const auto& q_prev = config_history_[i-1];
                    const auto& q_curr = config_history_[i];
                    double dtheta = q_curr[2] - q_prev[2];
                    
                    // Wrap to [-pi, pi]
                    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
                    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
                    
                    theta_unwrapped[i] = theta_unwrapped[i-1] + dtheta;
                }
                
                // Apply 5-point stencil to unwrapped angles
                wz = (25.0*theta_unwrapped[4] - 48.0*theta_unwrapped[3] + 36.0*theta_unwrapped[2] 
                      - 16.0*theta_unwrapped[1] + 3.0*theta_unwrapped[0]) / (12.0 * h);
            }
            
        } else if (n >= 3) {
            // Use 3-point backward difference: O(h^2) accuracy
            // f'(x) ≈ [3f(x) - 4f(x-h) + f(x-2h)] / (2h)
            
            double h = (time_history_[n-1] - time_history_[n-2]).seconds();
            
            if (h < 1e-6 || h > 1.0) {
                vx_world = vy_world = wz = 0.0;
            } else {
                const auto& q0 = config_history_[n-3];  // x-2h
                const auto& q1 = config_history_[n-2];  // x-h
                const auto& q2 = config_history_[n-1];  // x (current)
                
                vx_world = (3.0*q2[0] - 4.0*q1[0] + q0[0]) / (2.0 * h);
                vy_world = (3.0*q2[1] - 4.0*q1[1] + q0[1]) / (2.0 * h);
                
                // Angle unwrapping for 3 points
                double theta0 = q0[2];
                double dtheta1 = q1[2] - q0[2];
                while (dtheta1 > M_PI) dtheta1 -= 2.0 * M_PI;
                while (dtheta1 < -M_PI) dtheta1 += 2.0 * M_PI;
                double theta1 = theta0 + dtheta1;
                
                double dtheta2 = q2[2] - q1[2];
                while (dtheta2 > M_PI) dtheta2 -= 2.0 * M_PI;
                while (dtheta2 < -M_PI) dtheta2 += 2.0 * M_PI;
                double theta2 = theta1 + dtheta2;
                
                wz = (3.0*theta2 - 4.0*theta1 + theta0) / (2.0 * h);
            }
            
        } else {
            // Use simple 2-point difference: O(h) accuracy
            double dt = (time_history_[1] - time_history_[0]).seconds();
            
            if (dt < 1e-6 || dt > 1.0) {
                vx_world = vy_world = wz = 0.0;
            } else {
                const auto& q0 = config_history_[0];
                const auto& q1 = config_history_[1];
                
                vx_world = (q1[0] - q0[0]) / dt;
                vy_world = (q1[1] - q0[1]) / dt;
                
                double dtheta = q1[2] - q0[2];
                while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
                while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
                wz = dtheta / dt;
            }
        }
        
        // Transform velocities from world frame to robot frame
        double theta = current_config_[2];
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        // Rotation matrix: world → robot
        // [vx_robot]   [ cos(θ)  sin(θ)] [vx_world]
        // [vy_robot] = [-sin(θ)  cos(θ)] [vy_world]
        double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
        double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;
        
        // For differential drive: vy should be ~0, use only vx and wz
        msg.linear.x = vx_robot;
        msg.linear.y = 0.0;  // Differential drive constraint (no lateral motion)
        msg.linear.z = 0.0;  // No vertical motion
        
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = wz;
        
        // Log velocity commands for debugging (throttled)
        RCLCPP_DEBUG(this->get_logger(), 
                    "Base cmd [%zu-pt]: vx=%.3f m/s, wz=%.3f rad/s", 
                    n, vx_robot, wz);
        
        base_cmd_pub_->publish(msg);
            
        }  // End of legacy 5-point differentiation
    }  // End of publishBaseCommand()
    
void GIK9DOFSolverNode::publishDiagnostics(double solve_time_ms, const geometry_msgs::msg::Pose& target_pose)
    {
        auto msg = gik9dof_msgs::msg::SolverDiagnostics();
        msg.header.stamp = this->now();
        msg.solve_time_ms = solve_time_ms;
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.status = last_solver_status_.empty() ? "unknown" : last_solver_status_;
        msg.iterations = last_solver_iterations_;
        msg.exit_flag = last_exit_flag_;
        
        // Enhanced diagnostics
        msg.pose_error_norm = last_pose_violation_;
        msg.position_error = last_position_error_;
        msg.orientation_error = last_orientation_error_;
        msg.joint_limits_violated = last_joint_limit_violation_;
        msg.distance_constraint_met = last_distance_constraint_met_;
        
        // Distance constraints (20 total) - NEW detailed reporting
        msg.dist_body_indices.resize(20);
        msg.dist_ref_body_indices.resize(20);
        msg.dist_lower_bounds.resize(20);
        msg.dist_upper_bounds.resize(20);
        msg.dist_weights.resize(20);
        msg.dist_violations.resize(20);
        msg.dist_constraint_met.resize(20);
        
        for (int i = 0; i < 20; i++) {
            msg.dist_body_indices[i] = dist_body_indices_[i];
            msg.dist_ref_body_indices[i] = dist_ref_body_indices_[i];
            msg.dist_lower_bounds[i] = dist_lower_bounds_[i];
            msg.dist_upper_bounds[i] = dist_upper_bounds_[i];
            msg.dist_weights[i] = dist_weights_[i];
            msg.dist_violations[i] = last_dist_violations_[i];
            msg.dist_constraint_met[i] = last_dist_constraint_met_[i];
        }
        
        msg.num_active_constraints = static_cast<uint8_t>(last_num_active_constraints_);
        msg.num_violated_constraints = static_cast<uint8_t>(last_num_violated_constraints_);
        
        // Configuration
        msg.current_config = current_config_;
        msg.target_config = target_config_;
        
        // Poses (target provided, current would need FK calculation)
        msg.target_ee_pose = target_pose;
        // TODO: Compute current_ee_pose via forward kinematics if needed
        
        // Timestamps
        msg.solve_start = last_solve_start_;
        msg.solve_end = last_solve_end_;
        // Get trajectory timestamp if available
        if (current_trajectory_) {
            msg.trajectory_stamp = current_trajectory_->header.stamp;
        }
        
        diagnostics_pub_->publish(msg);
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GIK9DOFSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
