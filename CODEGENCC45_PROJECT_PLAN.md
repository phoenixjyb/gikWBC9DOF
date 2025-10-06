# CodegenCC45 - Clean Slate C++ Code Generation Project

## Project Overview

This branch (`codegencc45`) represents a clean slate rebuild of the MATLAB-based mobile manipulator control system, targeting production deployment on **NVIDIA AGX Orin** running **Ubuntu 22.04** with **ROS2 Humble**.

### Target Platform
- **Hardware**: NVIDIA AGX Orin (ARM64 architecture)
- **OS**: Ubuntu 22.04 LTS (native installation or WSL2)
- **Middleware**: ROS2 Humble
- **Build System**: CMake + colcon
- **Language**: C++17 (generated from MATLAB via MATLAB Coder)

### Key Objectives
1. **Clean Architecture**: Strip out all visualization, plotting, and simulation-specific code
2. **Production-Ready**: Generate optimized C++ code for real-time embedded deployment
3. **ROS2 Integration**: Create native ROS2 Humble nodes that interface with existing message schemas
4. **Real-Time Focus**: Integrate with live SLAM, perception, and odometry systems
5. **Maintainability**: Keep MATLAB code as design/testing environment, C++ for deployment

---

## Architecture Design

### System Components

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MATLAB Development Environment                    │
│  (Design, Simulation, Algorithm Development, Code Generation)       │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                    MATLAB Coder Code Generation
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  Generated C++ Core Libraries                        │
│  • GIK Solver (9-DOF inverse kinematics)                            │
│  • Trajectory Following                                              │
│  • Unified Chassis Controller                                        │
│  • Configuration Tools                                               │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    ROS2 Humble Node Wrappers                         │
├─────────────────────────────────────────────────────────────────────┤
│  1. gik9dof_solver_node                                             │
│     - Subscribes: /hdas/feedback_arm_left (JointState)             │
│     - Subscribes: /odom_wheel (Odometry)                            │
│     - Subscribes: /target/end_effector_trajectory (custom)          │
│     - Publishes: /motion_target/target_joint_state_arm_left         │
│                                                                      │
│  2. chassis_controller_node                                         │
│     - Subscribes: /odom_wheel (Odometry)                            │
│     - Subscribes: /target/base_trajectory (custom)                  │
│     - Publishes: /mobile_base/commands/velocity (Twist)             │
│                                                                      │
│  3. trajectory_manager_node (orchestrator)                          │
│     - Coordinates arm and base motion                                │
│     - Manages holistic vs staged mode switching                     │
│     - Interfaces with perception for obstacles                       │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      External ROS2 Interfaces                        │
├─────────────────────────────────────────────────────────────────────┤
│  Inputs:                                                             │
│  • SLAM/Localization → /odom_wheel, /tf                            │
│  • Perception → /obstacles (point cloud or occupancy grid)          │
│  • Task Planner → /task/trajectory_goals                            │
│                                                                      │
│  Outputs:                                                            │
│  • Arm Commands → Hardware controller                               │
│  • Base Commands → Wheel controllers                                │
│  • Diagnostics → /diagnostics                                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ROS2 Message Interface (from attachment)

Based on the provided specifications:

### Topic 1: Arm Joint Control Signal
- **Topic**: `/motion_target/target_joint_state_arm_left`
- **Type**: `sensor_msgs::msg::JointState`
- **Content**: 
  - `position[6]`: Target positions for 6 arm DOF joints
  - Frame: Mechanical arm control signal

### Topic 2: Arm Joint Feedback
- **Topic**: `/hdas/feedback_arm_left`
- **Type**: `sensor_msgs::msg::JointState`
- **Content**: 
  - `position[6]`: Current positions of 6 arm DOF joints
  - Frame: Mechanical arm feedback signal

### Topic 3: Chassis Odometry
- **Topic**: `/odom_wheel`
- **Type**: `nav_msgs::msg::Odometry`
- **Content**: Chassis positioning information from wheel odometry

### Topic 4: Chassis Velocity Command
- **Topic**: `/mobile_base/commands/velocity`
- **Type**: `geometry_msgs::msg::Twist`
- **Content**: 
  - `cmd_vel.linear.x`: Longitudinal velocity (m/s)
  - `cmd_vel.angular.z`: Yaw rate (rad/s)

---

## Core MATLAB Functions for Code Generation

### Essential Functions (to be code-generated)

#### 1. **Inverse Kinematics Solver**
- `+codegen/solveGIKStep.m` - Single IK iteration
- `+codegen/followTrajectory.m` - Trajectory following loop
- `+codegen/loadRobotForCodegen.m` - Robot model loader
- **Dependencies**: 
  - Robot model builder (procedural, no MAT file)
  - Collision geometry handling

#### 2. **Unified Chassis Controller**
- `+control/unifiedChassisCtrl.m` - Chassis velocity control
- `+control/clampYawByWheelLimit.m` - Velocity limiting
- `+control/defaultUnifiedParams.m` - Controller parameters
- **Output**: (Vx, Wz) for base motion

#### 3. **Trajectory Control Core**
- `runTrajectoryControl.m` - Main control loop (simplified)
- `configurationTools.m` - Joint configuration utilities
- **Mode**: Holistic (full-body) control

#### 4. **Robot Model Builder**
- `createRobotModel.m` - Procedural URDF loading
- `collisionTools.m` - Mesh attachment (codegen-compatible)

### Functions to EXCLUDE (MATLAB-only)

- ❌ All `+viz/*` visualization functions
- ❌ `animate*` animation functions  
- ❌ `generateLogPlots.m`, `plotTrajectoryLog.m` - plotting
- ❌ `saveRunArtifacts.m` - artifact generation
- ❌ `environmentConfig.m` - static obstacle configuration
- ❌ `addFloorDiscs.m`, `demoFloorDiscs.m` - environment setup
- ❌ `generateHolisticRamp.m` - trajectory generation (handled externally)
- ❌ `runStagedTrajectory.m` - Hybrid A* planning (to be replaced by ROS nav stack)
- ❌ All JSON file loading/parsing

---

## MATLAB Coder Configuration

### Code Generation Settings

```matlab
% Target: Embedded Linux ARM64 (AGX Orin)
cfg = coder.config('lib', 'ecoder', false);
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.EnableOpenMP = true; % For parallel computation
cfg.TargetLangStandard = 'C++17';

% Hardware-specific optimizations
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdWordSize = 64;

% Memory and performance
cfg.DynamicMemoryAllocation = 'Off'; % For real-time safety
cfg.ConstantInputs = 'IgnoreValues';
cfg.SaturateOnIntegerOverflow = false;
```

### Entry Points for Code Generation

1. **solveGIKStepRealtime.m** - Real-time IK solver with fixed-size arrays
2. **followTrajectoryRealtime.m** - Bounded trajectory follower
3. **unifiedChassisControlRealtime.m** - Chassis controller wrapper
4. **buildRobotForCodegenARM64.m** - ARM-specific robot builder

---

## ROS2 Package Structure

```
ros2/
├── gik9dof_msgs/                    # Custom message definitions
│   └── msg/
│       ├── EndEffectorTrajectory.msg
│       ├── SolverDiagnostics.msg
│       └── ControlMode.msg
│
├── gik9dof_solver/                  # Generated C++ solver library
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/gik9dof_solver/
│   │   ├── generated/              # MATLAB Coder output (headers)
│   │   └── solver_wrapper.hpp      # ROS2 wrapper interface
│   └── src/
│       ├── generated/              # MATLAB Coder output (source)
│       ├── solver_wrapper.cpp
│       └── robot_builder_arm64.cpp # Procedural robot model
│
├── gik9dof_controllers/            # ROS2 node implementations
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── controller_params.yaml
│   │   └── robot_config.yaml
│   ├── launch/
│   │   ├── full_system.launch.py
│   │   ├── arm_only.launch.py
│   │   └── chassis_only.launch.py
│   └── src/
│       ├── gik9dof_solver_node.cpp
│       ├── chassis_controller_node.cpp
│       ├── trajectory_manager_node.cpp
│       └── utils/
│           ├── ros_conversions.cpp
│           └── timing_utils.cpp
│
└── gik9dof_bringup/                # System launch and config
    ├── launch/
    │   └── agx_orin_bringup.launch.py
    └── config/
        └── deployment_config.yaml
```

---

## Development Workflow

### Phase 1: MATLAB Code Preparation (Current)
1. ✅ Create `codegencc45` branch from `origin/main`
2. 🔄 Refactor core functions to be codegen-compatible:
   - Remove persistent variables where possible
   - Replace dynamic arrays with bounded arrays
   - Remove file I/O and JSON parsing
   - Create procedural robot builder
3. 🔄 Create new entry points for real-time deployment
4. 🔄 Test codegen locally on Windows with ARM64 target

### Phase 2: Code Generation
1. Configure MATLAB Coder for ARM64 Linux target
2. Generate C++ source for all entry points
3. Verify generated code compiles on Ubuntu 22.04
4. Optimize for AGX Orin (NEON, OpenMP, cache tuning)

### Phase 3: ROS2 Integration
1. Create custom message definitions
2. Implement ROS2 wrapper nodes
3. Setup launch files and configuration
4. Integration testing with mock hardware

### Phase 4: Deployment & Testing
1. Deploy to AGX Orin
2. Interface with real SLAM/perception
3. Hardware-in-the-loop testing
4. Performance profiling and optimization

---

## Key Decisions & Rationale

### 1. **Exclude Hybrid A* Stage B Planning**
- **Reason**: ROS2 has mature navigation stacks (Nav2) better suited for base path planning
- **Integration**: Use Nav2 planner → publish goal poses → unified chassis controller consumes

### 2. **Replace Static Obstacles with Perception**
- **Reason**: Production robots use live perception (LiDAR, cameras)
- **Integration**: Subscribe to `/obstacles` or costmap topics from Nav2

### 3. **Procedural Robot Model Building**
- **Reason**: MATLAB Coder cannot load handle classes from MAT files
- **Solution**: Rebuild `rigidBodyTree` from URDF at initialization

### 4. **Holistic Control Priority**
- **Reason**: Staged control (A/B/C) is simulation-specific
- **Deployment**: Start with holistic 9-DOF control, add modes later if needed

### 5. **Fixed-Size Arrays**
- **Reason**: Real-time systems require deterministic memory
- **Constraint**: Max 50 waypoints per trajectory (configurable)

---

## Success Criteria

### Code Quality
- ✅ All generated C++ code compiles without warnings
- ✅ No dynamic memory allocation in control loops
- ✅ Real-time safe (deterministic execution time)
- ✅ Thread-safe solver implementation

### Performance
- 🎯 Control loop: 10 Hz minimum (100 Hz target)
- 🎯 Solver convergence: <10ms per IK step
- 🎯 End-to-end latency: <100ms (perception → command)

### Integration
- ✅ ROS2 nodes launch cleanly
- ✅ Message interfaces match specification
- ✅ Graceful degradation on sensor loss
- ✅ Diagnostic publishing for monitoring

### Validation
- ✅ Matches MATLAB simulation results (within tolerance)
- ✅ Hardware tests show stable control
- ✅ No crashes or memory leaks in 24-hour stress test

---

## Requirements Clarification (Answered)

✅ **Robot URDF**: `mobile_manipulator_PPR_base_corrected.urdf` (confirmed for initial implementation, STL optimization planned for later)

✅ **Mesh Files**: Current STLs in `/meshes` are approved for development. Future optimization with sparser patches will be drop-in replacement.

✅ **Trajectory Input**: ROS2 topic-based trajectory streaming
- **Approach**: Publish next few goal points to track (rolling window)
- **Rationale**: More flexible than complete static trajectory, suitable for online processing
- **Topic**: `/target/end_effector_trajectory` (custom message with waypoint array)
- **Note**: MATLAB simulation uses static JSON (not suitable for deployment)

✅ **Control Frequency**: 10 Hz (100ms control loop period)

✅ **Sensor Frequencies**: 
- SLAM odometry: 10 Hz initially (design for 50 Hz future expansion)
- Joint state feedback: 10 Hz initially (design for 50 Hz future expansion)
- **Design requirement**: Make frequencies configurable via parameters

✅ **Navigation Integration**: Use Navigation Toolbox components where available (not full Nav2 stack initially)

✅ **Obstacle Format**: OccupancyGrid with 10cm (0.1m) grid resolution

✅ **MATLAB Version**: R2024b with Robotics System Toolbox (installed and confirmed)

✅ **Testing Approach**: 
- AGX Orin available for hardware testing
- **Dual track**: Simulation verification + hardware validation in parallel

✅ **Existing ROS2 Infrastructure**:
- Perception modules (publishing OccupancyGrid)
- Trajectory modules (task planning)
- Robot control modules (hardware interfaces)
- **Integration requirement**: Ensure message compatibility with existing stack

---

## Next Steps

Based on the current branch state, recommended immediate actions:

1. **Review this plan** - Confirm architecture aligns with your deployment goals
2. **Clarify requirements** - Answer questions in "Additional Information Needed"
3. **URDF Audit** - Verify robot model is codegen-ready
4. **Create skeleton ROS2 packages** - Setup empty package structure
5. **Refactor first entry point** - Start with `solveGIKStep` as proof-of-concept
6. **Test code generation** - Validate on Windows → compile on Ubuntu
7. **Iterate** - Expand to full solver, then chassis controller

---

## References

- [MATLAB Coder Documentation](https://www.mathworks.com/help/coder/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [NVIDIA AGX Orin Developer Guide](https://developer.nvidia.com/embedded/jetson-agx-orin)

---

**Branch**: `codegencc45`  
**Created**: 2025-10-06  
**Status**: Planning Phase  
**Target**: Production deployment on NVIDIA AGX Orin with ROS2 Humble
