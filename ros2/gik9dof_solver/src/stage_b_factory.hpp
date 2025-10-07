/**
 * @file stage_b_factory.hpp
 * @brief Factory function to create Stage B controller (avoids namespace conflicts)
 * 
 * This header provides ONLY the factory function declaration, not the full
 * StageBController class. This allows the main node to create a Stage B controller
 * without including stage_b_chassis_plan.hpp (which conflicts with GIK solver types).
 */

#ifndef STAGE_B_FACTORY_HPP
#define STAGE_B_FACTORY_HPP

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rclcpp {
    class Node;
}

// Forward declaration for Pure Pursuit types
namespace gik9dof_purepursuit {
    struct struct0_T;
}

namespace gik9dof {

// Forward declarations
class StageBController;

enum class StageBMode {
    HybridAStar,          // Simple Hybrid A* mode (was B1)
    GIKAssisted           // GIK-assisted mode (was B2)
};

struct StageBParams {
    StageBMode mode;
    double chassis_max_vel_x;
    double chassis_max_vel_theta;
    double chassis_acc_x;
    double chassis_acc_theta;
    double lookahead_distance;
    double goal_tolerance_xy;
    double goal_tolerance_theta;
    double planner_time_limit_ms;
    int velocity_control_mode;          // 0=legacy, 1=simple, 2=pure pursuit
    gik9dof_purepursuit::struct0_T* pp_params;  // Pure Pursuit params (nullptr if not mode 2)
};

/**
 * @brief Factory function to create Stage B controller
 * 
 * Returns RAW pointer (not unique_ptr) to avoid incomplete type issues.
 * Caller is responsible for deletion.
 * 
 * @param node ROS2 node pointer for logging
 * @param params Controller parameters
 * @return Raw pointer to Stage B controller (caller must delete)
 */
gik9dof::StageBController* createStageBController(
    rclcpp::Node* node, 
    const StageBParams& params);

/**
 * @brief Destroy Stage B controller
 * 
 * @param controller Pointer to controller to destroy
 */
void destroyStageBController(gik9dof::StageBController* controller);

/**
 * @brief Wrapper: Activate Stage B controller
 * 
 * @param controller Pointer to Stage B controller
 * @param current_base_pose Current base pose [x, y, theta]
 * @param goal_base_pose Goal base pose [x, y, theta]
 * @param arm_static_config Arm gripping configuration (6-DOF)
 */
void stageBActivate(gik9dof::StageBController* controller,
                   const Eigen::Vector3d& current_base_pose,
                   const Eigen::Vector3d& goal_base_pose,
                   const std::vector<double>& arm_static_config);

/**
 * @brief Wrapper: Execute one step of Stage B controller
 * 
 * @param controller Pointer to Stage B controller
 * @param current_base_pose Current base pose [x, y, theta]
 * @param current_arm_config Current arm configuration (6-DOF)
 * @param[out] base_cmd Output base velocity command
 * @param[out] arm_cmd Output arm joint state
 * @return true if Stage B still active, false if goal reached
 */
bool stageBExecuteStep(gik9dof::StageBController* controller,
                      const Eigen::Vector3d& current_base_pose,
                      const std::vector<double>& current_arm_config,
                      geometry_msgs::msg::Twist& base_cmd,
                      sensor_msgs::msg::JointState& arm_cmd);

/**
 * @brief Wrapper: Check if chassis has reached goal
 * 
 * @param controller Pointer to Stage B controller
 * @param current_base_pose Current base pose [x, y, theta]
 * @return true if within tolerance of goal
 */
bool stageBChassisReachedGoal(gik9dof::StageBController* controller,
                             const Eigen::Vector3d& current_base_pose);

/**
 * @brief Wrapper: Deactivate Stage B controller
 * 
 * @param controller Pointer to Stage B controller
 */
void stageBDeactivate(gik9dof::StageBController* controller);

} // namespace gik9dof

#endif // STAGE_B_FACTORY_HPP
