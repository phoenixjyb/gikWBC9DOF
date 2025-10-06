# ROS2 Integration Guide - CodegenCC45

## Overview

This document provides detailed guidance for integrating MATLAB Coder-generated C++ code into ROS2 Humble nodes for deployment on NVIDIA AGX Orin.

---

## ROS2 Message Definitions

### Custom Messages Package: `gik9dof_msgs`

Create custom message types for trajectory control and diagnostics:

#### 1. EndEffectorTrajectory.msg

```msg
# End-effector trajectory message
Header header

# Array of target poses
geometry_msgs/PoseStamped[] waypoints

# Timing information
float64[] timestamps  # Time for each waypoint (relative to header.stamp)

# Constraint parameters
float64 distance_lower_bound  # Minimum distance from obstacles (m)
float64 distance_weight       # Weight for distance constraint

# Execution parameters
string control_mode  # "holistic" or "staged"
float64 timeout      # Maximum execution time (s)
```

#### 2. SolverDiagnostics.msg

```msg
# IK solver diagnostic information
Header header

# Solver status
bool converged
int32 iterations
float64 solve_time_ms

# Error metrics
float64 position_error  # m
float64 orientation_error  # rad
float64 joint_limit_violation  # rad (max violation)

# Current state
float64[] joint_positions  # Current joint configuration
geometry_msgs/Pose current_ee_pose
```

#### 3. ChassisCommand.msg

```msg
# Unified chassis command
Header header

# Velocity commands
float64 vx  # Forward velocity (m/s)
float64 vy  # Lateral velocity (m/s, always 0 for differential drive)
float64 wz  # Yaw rate (rad/s)

# Control mode
string mode  # "holistic", "staged-B", "staged-C"

# Diagnostics
float64 heading_error  # rad
bool velocity_limited   # True if command was clamped
```

#### 4. ControlMode.msg

```msg
# Control mode selection
Header header

string mode  # "holistic", "staged-A", "staged-B", "staged-C", "idle"
bool enable_arm
bool enable_base
```

---

## Package Structure Details

### Package 1: `gik9dof_msgs`

```
gik9dof_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── EndEffectorTrajectory.msg
    ├── SolverDiagnostics.msg
    ├── ChassisCommand.msg
    └── ControlMode.msg
```

**CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.8)
project(gik9dof_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndEffectorTrajectory.msg"
  "msg/SolverDiagnostics.msg"
  "msg/ChassisCommand.msg"
  "msg/ControlMode.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
```

---

### Package 2: `gik9dof_solver`

This package contains the MATLAB Coder-generated C++ code and ROS-agnostic wrapper.

```
gik9dof_solver/
├── CMakeLists.txt
├── package.xml
├── include/gik9dof_solver/
│   ├── generated/                    # MATLAB Coder headers
│   │   ├── solveGIKStepRealtime.h
│   │   ├── followTrajectory.h
│   │   ├── unifiedChassisCtrl.h
│   │   ├── buildRobotForCodegen.h
│   │   ├── RigidBodyTree.h
│   │   ├── rtwtypes.h
│   │   └── ... (other generated headers)
│   ├── solver_wrapper.hpp           # C++ wrapper around generated code
│   ├── chassis_controller.hpp       # Chassis control wrapper
│   └── robot_model_singleton.hpp    # Robot model management
└── src/
    ├── generated/                    # MATLAB Coder sources
    │   ├── solveGIKStepRealtime.cpp
    │   ├── followTrajectory.cpp
    │   ├── unifiedChassisCtrl.cpp
    │   ├── buildRobotForCodegen.cpp
    │   └── ... (other generated sources)
    ├── collisioncodegen/             # Collision detection helpers
    │   └── ... (from MATLAB)
    ├── libccd/                       # Collision library
    │   └── ... (third-party)
    ├── solver_wrapper.cpp
    ├── chassis_controller.cpp
    └── robot_model_singleton.cpp
```

**Key Implementation: `solver_wrapper.hpp`**

```cpp
#ifndef GIK9DOF_SOLVER_WRAPPER_HPP
#define GIK9DOF_SOLVER_WRAPPER_HPP

#include <array>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace gik9dof {

struct IKResult {
    std::array<double, 9> joint_positions;
    bool converged;
    int iterations;
    double solve_time_ms;
    double position_error;
    double orientation_error;
};

struct ChassisCmd {
    double vx;
    double vy;
    double wz;
    std::string mode;
    double heading_error;
    bool velocity_limited;
};

class SolverWrapper {
public:
    SolverWrapper();
    ~SolverWrapper();

    // Initialize robot model (call once)
    bool initialize();

    // Single IK step
    IKResult solveIKStep(
        const std::array<double, 9>& q_current,
        const Eigen::Matrix4d& target_pose,
        double distance_lower = 0.2,
        double distance_weight = 0.5
    );

    // Trajectory following
    IKResult followTrajectory(
        const std::array<double, 9>& q_initial,
        const std::vector<Eigen::Matrix4d>& waypoints,
        double distance_lower = 0.2,
        double distance_weight = 0.5
    );

    // Chassis control
    ChassisCmd computeChassisCommand(
        const std::array<double, 3>& ref_pose,  // [x, y, theta]
        const std::array<double, 3>& est_pose,
        const std::string& mode = "holistic"
    );

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace gik9dof

#endif // GIK9DOF_SOLVER_WRAPPER_HPP
```

**CMakeLists.txt** (key sections):

```cmake
cmake_minimum_required(VERSION 3.8)
project(gik9dof_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Collect generated sources
file(GLOB GENERATED_SOURCES 
    "src/generated/*.cpp"
    "src/collisioncodegen/*.cpp"
    "src/libccd/*.c"
)

# Create library
add_library(${PROJECT_NAME} SHARED
    ${GENERATED_SOURCES}
    src/solver_wrapper.cpp
    src/chassis_controller.cpp
    src/robot_model_singleton.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    OpenMP::OpenMP_CXX
)

# Export
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenMP)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()
```

---

### Package 3: `gik9dof_controllers`

ROS2 nodes that use the solver library:

```
gik9dof_controllers/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── controller_params.yaml
│   ├── robot_limits.yaml
│   └── solver_config.yaml
├── launch/
│   ├── full_system.launch.py
│   ├── arm_control.launch.py
│   └── chassis_control.launch.py
└── src/
    ├── gik9dof_solver_node.cpp
    ├── chassis_controller_node.cpp
    ├── trajectory_manager_node.cpp
    └── utils/
        ├── ros_conversions.hpp
        ├── ros_conversions.cpp
        ├── timing_utils.hpp
        └── timing_utils.cpp
```

#### Node 1: `gik9dof_solver_node`

**Purpose**: Compute IK solutions for arm control

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gik9dof_msgs/msg/solver_diagnostics.hpp>
#include <gik9dof_msgs/msg/end_effector_trajectory.hpp>
#include <gik9dof_solver/solver_wrapper.hpp>

class GIK9DOFSolverNode : public rclcpp::Node {
public:
    GIK9DOFSolverNode();

private:
    void trajectoryCallback(const gik9dof_msgs::msg::EndEffectorTrajectory::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    void publishArmCommand(const std::array<double, 9>& joint_positions);
    void publishDiagnostics(const gik9dof::IKResult& result);

    // ROS interfaces
    rclcpp::Subscription<gik9dof_msgs::msg::EndEffectorTrajectory>::SharedPtr traj_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<gik9dof_msgs::msg::SolverDiagnostics>::SharedPtr diag_pub_;
    
    // Solver
    std::unique_ptr<gik9dof::SolverWrapper> solver_;
    
    // State
    std::array<double, 9> current_joint_positions_;
    bool joint_state_received_{false};
};
```

**Subscriptions**:
- `/target/end_effector_trajectory` (`gik9dof_msgs::msg::EndEffectorTrajectory`)
- `/hdas/feedback_arm_left` (`sensor_msgs::msg::JointState`)

**Publications**:
- `/motion_target/target_joint_state_arm_left` (`sensor_msgs::msg::JointState`)
- `/gik9dof/solver_diagnostics` (`gik9dof_msgs::msg::SolverDiagnostics`)

**Parameters**:
```yaml
gik9dof_solver:
  ros__parameters:
    control_rate: 10.0  # Hz
    distance_lower_bound: 0.2  # m
    distance_weight: 0.5
    max_iterations: 100
    convergence_tolerance: 0.001  # m
    timeout: 1.0  # s
```

---

#### Node 2: `chassis_controller_node`

**Purpose**: Compute chassis velocity commands

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gik9dof_msgs/msg/chassis_command.hpp>
#include <gik9dof_solver/chassis_controller.hpp>

class ChassisControllerNode : public rclcpp::Node {
public:
    ChassisControllerNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void chassisCmdCallback(const gik9dof_msgs::msg::ChassisCommand::SharedPtr msg);
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gik9dof_msgs::msg::ChassisCommand>::SharedPtr chassis_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    
    // State
    std::array<double, 3> current_pose_;  // [x, y, theta]
    bool odom_received_{false};
};
```

**Subscriptions**:
- `/odom_wheel` (`nav_msgs::msg::Odometry`)
- `/gik9dof/chassis_command` (`gik9dof_msgs::msg::ChassisCommand`) - internal

**Publications**:
- `/mobile_base/commands/velocity` (`geometry_msgs::msg::Twist`)

---

#### Node 3: `trajectory_manager_node`

**Purpose**: Orchestrate arm and base coordination

```cpp
class TrajectoryManagerNode : public rclcpp::Node {
public:
    TrajectoryManagerNode();

private:
    void controlTimerCallback();
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void modeCallback(const gik9dof_msgs::msg::ControlMode::SharedPtr msg);
    
    void executeHolisticControl();
    void executeStagedControl();
    
    // ROS interfaces
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<gik9dof_msgs::msg::ControlMode>::SharedPtr mode_sub_;
    
    rclcpp::Publisher<gik9dof_msgs::msg::EndEffectorTrajectory>::SharedPtr traj_pub_;
    rclcpp::Publisher<gik9dof_msgs::msg::ChassisCommand>::SharedPtr chassis_pub_;
    
    // Solver
    std::unique_ptr<gik9dof::SolverWrapper> solver_;
    
    // State
    std::string control_mode_{"holistic"};
    geometry_msgs::msg::PoseStamped current_goal_;
};
```

---

## Launch Files

### `full_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('gik9dof_controllers'),
        'config'
    )
    
    solver_params = os.path.join(config_dir, 'solver_config.yaml')
    robot_params = os.path.join(config_dir, 'robot_limits.yaml')
    controller_params = os.path.join(config_dir, 'controller_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_rate',
            default_value='10.0',
            description='Control loop frequency (Hz)'
        ),
        
        Node(
            package='gik9dof_controllers',
            executable='gik9dof_solver_node',
            name='gik9dof_solver',
            parameters=[solver_params, robot_params],
            output='screen'
        ),
        
        Node(
            package='gik9dof_controllers',
            executable='chassis_controller_node',
            name='chassis_controller',
            parameters=[controller_params],
            output='screen'
        ),
        
        Node(
            package='gik9dof_controllers',
            executable='trajectory_manager_node',
            name='trajectory_manager',
            parameters=[
                solver_params,
                controller_params,
                {'control_rate': LaunchConfiguration('control_rate')}
            ],
            output='screen'
        ),
    ])
```

---

## Configuration Files

### `solver_config.yaml`

```yaml
gik9dof_solver:
  ros__parameters:
    # Control loop
    control_rate: 10.0  # Hz
    
    # IK solver parameters
    max_iterations: 100
    convergence_tolerance: 0.001  # m
    timeout: 1.0  # s
    
    # Constraint weights
    distance_lower_bound: 0.2  # m
    distance_weight: 0.5
    pose_weight_translation: 1.0
    pose_weight_orientation: 1.0
    joint_bounds_weight: 0.01
    
    # Performance
    enable_openmp: true
    num_threads: 4
```

### `controller_params.yaml`

```yaml
chassis_controller:
  ros__parameters:
    # Physical parameters
    wheel_base: 0.5  # m
    wheel_track: 0.45  # m
    wheel_radius: 0.1  # m
    
    # Velocity limits
    max_linear_velocity: 0.5  # m/s
    max_angular_velocity: 1.0  # rad/s
    max_wheel_velocity: 2.0  # rad/s (wheel rotation)
    
    # Control gains (unified chassis)
    yaw_kp: 2.0
    yaw_kff: 0.8
    
    # Safety
    emergency_stop_decel: 2.0  # m/s^2
```

### `robot_limits.yaml`

```yaml
robot_limits:
  ros__parameters:
    # Joint limits (9 DOF: 3 base + 6 arm)
    joint_names: 
      - "joint_x"
      - "joint_y"
      - "joint_theta"
      - "left_arm_joint1"
      - "left_arm_joint2"
      - "left_arm_joint3"
      - "left_arm_joint4"
      - "left_arm_joint5"
      - "left_arm_joint6"
    
    # Position limits [rad or m]
    position_limits_lower: [-10.0, -10.0, -3.14159, -2.9, -1.5, -2.5, -2.0, -2.0, -2.5]
    position_limits_upper: [10.0, 10.0, 3.14159, 2.9, 1.5, 2.5, 2.0, 2.0, 2.5]
    
    # Velocity limits [rad/s or m/s]
    velocity_limits: [0.5, 0.5, 1.0, 1.5, 1.5, 1.5, 2.0, 2.0, 2.0]
```

---

## Build Instructions

### Prerequisites

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install \
    build-essential \
    cmake \
    libeigen3-dev \
    libomp-dev \
    python3-colcon-common-extensions
```

### Build Sequence

```bash
# Setup ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/workspace/codegenGIKsample/Trial/gikWBC9DOF/ros2

# Build messages first
colcon build --packages-select gik9dof_msgs

# Source message package
source install/setup.bash

# Build solver library
colcon build --packages-select gik9dof_solver

# Source solver package
source install/setup.bash

# Build controller nodes
colcon build --packages-select gik9dof_controllers

# Final setup
source install/setup.bash
```

---

## Testing

### Unit Test Example (C++)

```cpp
#include <gtest/gtest.h>
#include <gik9dof_solver/solver_wrapper.hpp>
#include <Eigen/Dense>

TEST(SolverWrapperTest, InitializeRobot) {
    gik9dof::SolverWrapper solver;
    EXPECT_TRUE(solver.initialize());
}

TEST(SolverWrapperTest, SolveIKStep) {
    gik9dof::SolverWrapper solver;
    solver.initialize();
    
    std::array<double, 9> q_current = {0};
    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
    target_pose(0, 3) = 0.6;  // x
    target_pose(1, 3) = 0.2;  // y
    target_pose(2, 3) = 1.0;  // z
    
    auto result = solver.solveIKStep(q_current, target_pose);
    
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.position_error, 0.01);  // Less than 1cm error
}
```

### Integration Test (ROS2)

```bash
# Terminal 1: Launch system
ros2 launch gik9dof_controllers full_system.launch.py

# Terminal 2: Echo diagnostics
ros2 topic echo /gik9dof/solver_diagnostics

# Terminal 3: Publish test trajectory
ros2 topic pub /target/end_effector_trajectory gik9dof_msgs/msg/EndEffectorTrajectory \
    "{ ... }" --once
```

---

## Deployment Checklist

- [ ] MATLAB Coder generates ARM64 code successfully
- [ ] All generated code compiles without errors
- [ ] Custom ROS2 messages build correctly
- [ ] Solver library links against OpenMP and Eigen
- [ ] Controller nodes launch without errors
- [ ] Message interfaces match specification exactly
- [ ] Parameters load from YAML files
- [ ] Real-time performance meets 10 Hz minimum
- [ ] Hardware-in-loop testing passes
- [ ] 24-hour stress test completes

---

**Document Version**: 1.0  
**Last Updated**: 2025-10-06
