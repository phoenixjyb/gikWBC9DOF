# ROS 2 Node Sketches

This directory documents the planned ROS 2 integration for the 9-DOF mobile manipulator controller. We target two runtime modes—holistic whole-body tracking and staged control (arm ramp-up → chassis ramp-up → combined). Each node definition outlines interfaces for odometry, target trajectories, and commanded joint/base motions.

## Common Topics & Interfaces

- `/robot_state/joint_states` (`sensor_msgs/msg/JointState`): live joint angles/velocities for arm and chassis.
- `/robot_state/odometry` (`nav_msgs/msg/Odometry`): base pose/velocity.
- `/target/end_effector_trajectory` (`trajectory_msgs/msg/MultiDOFJointTrajectory` or custom Pose array): desired EE trajectory.
- `/target/home_configuration` (`sensor_msgs/msg/JointState` or a custom message): optional home pose seed.
- `/control/joint_commands` (`trajectory_msgs/msg/JointTrajectory`): arm joint commands.
- `/control/base_commands` (`geometry_msgs/msg/Twist` or custom planar command): chassis velocity commands.
- `/obstacles/floor_discs` (`visualization_msgs/msg/MarkerArray` or custom): floor obstacles for distance constraints.
- `/diagnostics/gik_status` (custom): solver exit flags, constraint violations.

## Node Definitions

### 1. `whole_body_controller` (Holistic Mode)

- **Purpose:** Run the holistic 9-DOF GIK solver at control rate.
- **Inputs:**
  - `JointState`: current arm + chassis joints.
  - `Odometry`: base pose (for logging/estimation).
  - `End-effector trajectory`: desired poses over time.
  - `Floor discs` (optional): update distance specs dynamically.
- **Outputs:**
  - `JointTrajectory`: arm joint targets.
  - `Twist` (or equivalent): planar base velocities.
  - `GIK diagnostics`: exit flag, iteration count.
- **Parameters:**
  - `control_rate` (Hz).
  - `distance_margin`, `distance_weight` defaults.
  - `solver_gains`: tuning parameters.
- **Services/Actions:**
  - `SetMode` (optional) to toggle to staged controller or reload goals.

### 2. `staged_controller`

- **Purpose:** Execute staged pipeline (Stage A/B/C) with internal sequencing.
- **Inputs:**
  - `JointState`, `Odometry`, `End-effector trajectory`, `Floor discs` (same as holistic).
  - `Stage config` (optional parameters—e.g., stage durations, ramp profiles).
- **Outputs:**
  - Stage-specific `JointTrajectory` commands (arm or base) as segments.
  - `Staged diagnostics`: stage progress, planner summaries.
- **Parameters:**
  - `stage_a_samples`, `stage_b_samples` (number of intermediate targets).
  - `stage_c_rate` (control loop for full GIK).
  - `distance_margin`, `distance_weight`.
- **Services/Actions:**
  - `StartStageSequence`, `CancelStage` to coordinate transitions.

### 3. `trajectory_manager`

- **Purpose:** Front-end node that loads/reference trajectories, home configurations, and dispatches them to either controller.
- **Inputs:**
  - `End-effector trajectory` uploads via service (`LoadTrajectory`).
  - `Home configuration` inputs.
- **Outputs:**
  - Publishes desired trajectories on `/target/end_effector_trajectory`.
- **Parameters:**
  - Path to trajectory files, staging options.

### 4. `obstacle_provider`

- **Purpose:** Convert environment sensing (or static definitions) into floor-disc messages.
- **Outputs:**
  - `MarkerArray` / custom message describing disc radii, centers, safety margins.
- **Parameters:**
  - Disc definitions (static) or sensor topics to monitor.

## Directory Layout Placeholder

```
ros2/nodes/
  README.md
  gik_solver_wrapper.hpp
  holistic_controller.cpp
  staged_controller.cpp
  trajectory_manager.cpp
  obstacle_provider.cpp
  launch/
    controller.launch.py  (future ROS 2 launch)
```

The current C++ implementations stub out solver calls via `gik_solver_wrapper.hpp`. Replace that header with calls into the generated `solveGIKStep` / `followTrajectory` library once MATLAB Coder output is available.
