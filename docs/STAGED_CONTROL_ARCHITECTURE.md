# Staged Control Architecture for 9-DOF Mobile Manipulator

**Date**: October 7, 2025  
**Status**: Implementation in Progress  
**Decision**: Integrate Hybrid A* planner into `gik9dof_solver` node

---

## Executive Summary

The 9-DOF mobile manipulator uses a **multi-mode, multi-stage control architecture** that combines:
- **Generalized IK (GIK)** for whole-body inverse kinematics (9-DOF)
- **Hybrid A*** for SE(2) path planning (chassis navigation)
- **Multiple velocity controllers** for differential-drive command generation

**Key Decision**: Integrate Hybrid A* planner into the existing `gik9dof_solver` node rather than creating a separate node, due to tight coupling requirements in Stage B2 (GIK-assisted chassis planning).

---

## Control Modes

### Mode 1: HOLISTIC (9-DOF Simultaneous)

```
Input: End-effector trajectory (task space)
   ↓
┌──────────────────────────────────────────┐
│  GIK Solver (9-DOF simultaneous)         │
│  • 6-DOF arm (shoulder, elbow, wrist)    │
│  • 3-DOF chassis (x, y, θ)               │
│  • Output: Joint commands + chassis pose │
└──────────────┬───────────────────────────┘
               │ Chassis DOF: (x, y, θ)
               ↓
┌──────────────────────────────────────────┐
│  Velocity Controller (3 methods)         │
│  • Method 0: Legacy (5-point diff)       │
│  • Method 1: Simple Heading (P + FF)     │
│  • Method 2: Pure Pursuit (lookahead)    │
│  • Output: (Vx, Wz) commands             │
└──────────────────────────────────────────┘
```

**Use Cases:**
- Coordinated manipulation while moving
- Continuous trajectory tracking
- Real-time end-effector control

**Characteristics:**
- Single-pass computation
- Tightly coupled arm-chassis motion
- No staged transitions

---

### Mode 2: STAGED (Sequential A → B → C)

Decouples arm and chassis motion into discrete stages for complex tasks requiring strategic planning.

#### **STAGE A: Arm Ramp-Up**

```
┌──────────────────────────────────────────┐
│  GIK Solver (6-DOF arm only)             │
│  • Input: Desired arm configuration      │
│  • Chassis: FROZEN at (0, 0, 0)          │
│  • Output: Arm joint commands            │
└──────────────────────────────────────────┘
```

**Purpose:**
- Position arm before chassis moves
- Avoid collision during navigation
- Prepare for manipulation task

**Completion Criteria:**
- Arm configuration error < threshold (e.g., 95% complete)
- Transition to Stage B when arm is "close enough"

---

#### **STAGE B: Chassis Planning & Navigation**

**Two Submodes:**

##### **Submode B1: Pure Hybrid A***

```
Input: Start pose, Goal pose, Occupancy grid
   ↓
┌──────────────────────────────────────────┐
│  Hybrid A* Planner (MATLAB Coder C++)    │
│  • SE(2) state lattice search            │
│  • Collision checking with robot footprint│
│  • Output: Waypoints [(x, y, θ), ...]    │
│  • Arm: FROZEN in Stage A position       │
└──────────────┬───────────────────────────┘
               │ Path waypoints
               ↓
┌──────────────────────────────────────────┐
│  Velocity Controller (user-selected)     │
│  • Pure Pursuit (recommended)            │
│  • Lookahead-based path following        │
│  • Output: (Vx, Wz) commands             │
└──────────────────────────────────────────┘
```

**Characteristics:**
- Strategic global planning
- Obstacle avoidance
- Kinematically feasible paths (nonholonomic constraints)
- Planning time: < 50ms target

##### **Submode B2: GIK-Assisted Hybrid A***

```
Input: Start pose, Goal pose, Occupancy grid
   ↓
┌──────────────────────────────────────────┐
│  Hybrid A* Planner                       │
│  • Strategic path planning               │
│  • Coarse waypoint generation            │
└──────────────┬───────────────────────────┘
               │ Waypoints
               ↓
┌──────────────────────────────────────────┐
│  GIK Solver (3-DOF chassis virtuals)     │
│  • Refine each waypoint                  │
│  • Compute chassis virtual joint values  │
│  • Consider arm configuration constraints│
└──────────────┬───────────────────────────┘
               │ Refined chassis configs
               ↓
┌──────────────────────────────────────────┐
│  Velocity Controller                     │
│  • Generate (Vx, Wz) from configs        │
└──────────────────────────────────────────┘
```

**Characteristics:**
- Combines strategic (Hybrid A*) + tactical (GIK) planning
- Respects full robot constraints
- Tighter integration with arm configuration
- **Requires both planners in same node** (tight coupling)

**Completion Criteria:**
- Chassis reaches goal region (e.g., within 0.2m + 10°)
- Path following error < threshold
- Transition to Stage C for final tracking

---

#### **STAGE C: Full-Body Tracking**

```
┌──────────────────────────────────────────┐
│  GIK Solver (9-DOF full-body)            │
│  • Track desired EE trajectory           │
│  • 6-DOF arm + 3-DOF chassis             │
│  • Similar to Holistic mode              │
└──────────────┬───────────────────────────┘
               │ Chassis DOF
               ↓
┌──────────────────────────────────────────┐
│  Velocity Controller (same 3 methods)    │
└──────────────────────────────────────────┘
```

**Purpose:**
- Final precise end-effector positioning
- Coordinated arm-chassis motion
- Execute manipulation task

**Characteristics:**
- Holistic control with warm-start from Stage B
- Convergence to final EE pose
- Task completion

---

## Architectural Decision: Integrated vs Separate Node

### Analysis

| Aspect | Separate Node | **Integrated Node** ✅ |
|--------|---------------|------------------------|
| **Stage B2 Coupling** | Complex ROS msgs | Direct function calls |
| **State Machine** | Distributed (2 nodes) | Centralized (1 node) |
| **Shared State** | Duplicated/sync issues | Single source of truth |
| **Real-time Performance** | ROS msg overhead | In-process calls |
| **Configuration** | 2 YAML files | 1 unified config |
| **Debugging** | Multi-node logs | Single node logs |
| **Warm-start** | Message passing | Shared memory |
| **Stage Transitions** | Network latency | Atomic transitions |

### Decision: **INTEGRATE into `gik9dof_solver`** ✅

**Rationale:**

1. **Stage B2 Requires Tight Coupling**
   ```cpp
   // Stage B2 pseudo-code - needs BOTH components
   Path path = hybridAStar.plan(start, goal, grid);  // Strategic
   for (waypoint in path) {
       chassisConfig = GIK.solve3DOF(waypoint);      // Tactical - SAME NODE
       velocity = purePursuit.compute(chassisConfig);
   }
   ```

2. **Centralized State Machine**
   - Mode selection (Holistic/Staged)
   - Stage transitions (A → B → C)
   - Submode selection (B1/B2)
   - Velocity method (0/1/2)
   - All controlled by single parameter set

3. **Shared Robot State**
   - Current configuration (9-DOF)
   - Chassis parameters (wheelbase, track, limits)
   - Warm-start data for GIK
   - Obstacle map
   - Previous solutions

4. **Performance**
   - No inter-node ROS message overhead
   - Direct function calls (~nanoseconds vs ~milliseconds)
   - Atomic stage transitions
   - Single-process memory sharing

5. **Existing Pattern**
   - `gik9dof_solver` already integrates:
     - GIK solver (MATLAB Coder)
     - 2 velocity controllers (MATLAB Coder)
     - Mode/stage logic
   - Adding Hybrid A* follows same pattern

---

## Implementation Architecture

### Node Structure: `gik9dof_solver`

```
GIK9DOFSolverNode
├── Components
│   ├── GIKSolver (MATLAB Coder)              ← EXISTING
│   ├── HybridAStarPlanner (MATLAB Coder)     ← NEW
│   ├── HolisticVelocityController            ← EXISTING
│   └── PurePursuitController                 ← EXISTING
│
├── State Machine
│   ├── Mode: Holistic / Staged
│   ├── Stage: A / B / C (if Staged)
│   ├── Stage B Submode: B1 / B2
│   └── Velocity Method: 0 / 1 / 2
│
├── Stage Executors
│   ├── executeHolisticMode()
│   ├── executeStageA_ArmRampUp()
│   ├── executeStageB1_PureHybridAStar()      ← NEW
│   ├── executeStageB2_GIKAssisted()          ← NEW
│   └── executeStageC_FullBodyTracking()
│
└── Subscriptions/Publications
    ├── SUB: /ee_trajectory
    ├── SUB: /joint_states
    ├── SUB: /odom
    ├── SUB: /occupancy_grid                  ← NEW (for Stage B)
    ├── PUB: /cmd_vel
    ├── PUB: /joint_commands
    └── PUB: /solver_diagnostics
```

### File Organization

```
ros2/gik9dof_solver/
├── src/
│   ├── gik9dof_solver_node.cpp               # Main node (state machine)
│   │
│   ├── generated/                             # MATLAB Coder outputs
│   │   ├── gik/                              # ← EXISTING
│   │   │   ├── GIKSolver.h/cpp
│   │   │   ├── solveGIKStepRealtime.h/cpp
│   │   │   └── ... (61 files)
│   │   │
│   │   ├── planner/                          # ← NEW: Hybrid A* (from codegen/planner_arm64/)
│   │   │   ├── planHybridAStarCodegen.h/cpp
│   │   │   ├── HybridState.h
│   │   │   ├── OccupancyGrid2D.h
│   │   │   ├── BinaryHeap.h/cpp
│   │   │   ├── checkFootprintCollision.h/cpp
│   │   │   └── ... (~40 files)
│   │   │
│   │   └── velocity_controller/              # ← EXISTING
│   │       ├── holisticVelocityController.h/cpp
│   │       └── purePursuitVelocityController.h/cpp
│   │
│   ├── stage_controllers/                    # ← NEW: Stage execution logic
│   │   ├── stage_a_arm_ramp.cpp
│   │   ├── stage_b_chassis_plan.cpp
│   │   └── stage_c_tracking.cpp
│   │
│   └── state_machine/                        # ← NEW: Mode/stage transitions
│       ├── control_mode_manager.cpp
│       └── stage_transition_logic.cpp
│
├── include/gik9dof_solver/
│   ├── stage_controllers/
│   │   ├── stage_a_arm_ramp.hpp
│   │   ├── stage_b_chassis_plan.hpp
│   │   └── stage_c_tracking.hpp
│   │
│   └── state_machine/
│       ├── control_mode_manager.hpp
│       └── stage_transition_logic.hpp
│
├── CMakeLists.txt                            # Link all generated libraries
├── package.xml
└── config/
    └── gik9dof_solver.yaml                   # Unified parameters
```

---

## ROS2 Parameters

### Configuration File: `config/gik9dof_solver.yaml`

```yaml
gik9dof_solver:
  ros__parameters:
    # ========================================
    # MODE SELECTION
    # ========================================
    control_mode: "staged"  # Options: "holistic" | "staged"
    
    # ========================================
    # HOLISTIC MODE CONFIGURATION
    # ========================================
    holistic:
      control_rate: 10.0              # Hz
      max_solve_time: 0.05            # 50ms GIK timeout
      distance_weight: 1.0
      use_warm_start: true
    
    # ========================================
    # STAGED MODE CONFIGURATION
    # ========================================
    staged:
      initial_stage: "stage_a"        # Options: "stage_a" | "stage_b" | "stage_c"
      
      # ----------------------------------------
      # STAGE A: Arm Ramp-Up
      # ----------------------------------------
      stage_a:
        target_arm_config: [0.0, 0.5, 0.0, -1.0, 0.0, 0.5]  # [rad]
        completion_threshold: 0.95    # 95% convergence
        max_duration: 5.0              # seconds (timeout)
      
      # ----------------------------------------
      # STAGE B: Chassis Planning
      # ----------------------------------------
      stage_b:
        submode: "pure_hybrid_astar"  # Options: "pure_hybrid_astar" | "gik_assisted"
        
        # Hybrid A* Planner Settings
        hybrid_astar:
          grid_resolution: 0.1        # meters
          max_planning_time: 0.05     # 50ms timeout
          state_lattice_theta_bins: 72  # 72 bins = 5° angular resolution
          goal_position_tolerance: 0.2  # meters
          goal_theta_tolerance: 0.175   # radians (~10°)
          max_iterations: 50000        # A* search limit
          enable_reverse: true         # Allow backward motion
          
        # GIK-Assisted Settings (Submode B2 only)
        gik_assisted:
          waypoint_spacing: 0.3       # meters (how often to call GIK)
          gik_timeout: 0.01           # 10ms per waypoint
          
        # Path Following
        completion_threshold: 0.90    # 90% path completion
        max_duration: 30.0            # seconds (timeout)
      
      # ----------------------------------------
      # STAGE C: Full-Body Tracking
      # ----------------------------------------
      stage_c:
        control_rate: 10.0
        max_solve_time: 0.05
        tracking_tolerance: 0.05      # meters (EE position error)
        completion_threshold: 0.95
    
    # ========================================
    # VELOCITY CONTROLLER SELECTION
    # ========================================
    velocity_control_mode: 2  # Options: 0 | 1 | 2
                              # 0 = Legacy (5-point differentiation)
                              # 1 = Simple Heading (P + feedforward)
                              # 2 = Pure Pursuit (lookahead-based)
    
    # ----------------------------------------
    # Method 1: Simple Heading Controller
    # ----------------------------------------
    simple_heading:
      track: 0.573                    # Wheel track width (m)
      vwheel_max: 2.0                 # Max wheel speed (m/s)
      vx_max: 0.8                     # Max forward velocity (m/s)
      wz_max: 2.5                     # Max yaw rate (rad/s)
      yaw_kp: 2.0                     # Heading error P gain
      yaw_kff: 0.9                    # Yaw rate feedforward gain
    
    # ----------------------------------------
    # Method 2: Pure Pursuit Controller
    # ----------------------------------------
    purepursuit:
      lookahead_base: 0.8             # Base lookahead distance (m)
      lookahead_vel_gain: 0.3         # Velocity-dependent gain
      lookahead_time_gain: 0.1        # Time-based gain
      vx_nominal: 0.8                 # Nominal forward velocity (m/s)
      vx_max: 0.8                     # Max forward velocity (m/s)
      wz_max: 2.5                     # Max yaw rate (rad/s)
      track: 0.573                    # Wheel track width (m)
      vwheel_max: 2.0                 # Max wheel speed (m/s)
      waypoint_spacing: 0.15          # Path discretization (m)
      path_buffer_size: 30.0          # Path horizon (m)
      goal_tolerance: 0.2             # Goal reach tolerance (m)
      interp_spacing: 0.05            # Interpolation resolution (m)
    
    # ========================================
    # GIK SOLVER CONFIGURATION
    # ========================================
    gik:
      distance_lower_bound: 0.1       # meters
      distance_weight: 1.0
      publish_diagnostics: true
      use_warm_start: true
    
    # ========================================
    # ROBOT PARAMETERS
    # ========================================
    robot:
      chassis_type: "fourwheel"       # "compact" | "fourwheel"
      wheelbase: 0.360                # meters
      track: 0.573                    # meters
      wheel_radius: 0.107             # meters
      robot_radius: 0.461             # Bounding circle (m)
      inflation_radius: 0.511         # Safety margin (m)
      min_turning_radius: 0.344       # Passive rear constraint (m)
```

---

## State Machine Logic

### Control Flow

```cpp
void GIK9DOFSolverNode::controlLoop() {
    // Top-level mode selection
    if (current_mode_ == ControlMode::HOLISTIC) {
        executeHolisticMode();
    } else {  // STAGED
        executeStagedControl();
    }
}

void GIK9DOFSolverNode::executeStagedControl() {
    switch(current_stage_) {
        case Stage::STAGE_A_ARM_RAMP:
            executeStageA_ArmRampUp();
            if (armRampComplete()) {
                transitionToStage(Stage::STAGE_B_CHASSIS_PLAN);
            }
            break;
            
        case Stage::STAGE_B_CHASSIS_PLAN:
            if (stage_b_submode_ == StageBSubmode::B1_PURE_HYBRID_ASTAR) {
                executeStageB1_PureHybridAStar();
            } else {
                executeStageB2_GIKAssisted();
            }
            if (chassisPlanComplete()) {
                transitionToStage(Stage::STAGE_C_TRACKING);
            }
            break;
            
        case Stage::STAGE_C_TRACKING:
            executeStageC_FullBodyTracking();
            if (trackingComplete()) {
                RCLCPP_INFO(this->get_logger(), "Task complete!");
                // Reset or wait for new task
            }
            break;
    }
}
```

### Stage Transition Criteria

| Transition | Condition | Parameters |
|------------|-----------|------------|
| A → B | Arm config error < threshold | `stage_a.completion_threshold` (0.95) |
| B → C | Path following error < threshold | `stage_b.completion_threshold` (0.90) |
| C → Done | EE tracking error < threshold | `stage_c.completion_threshold` (0.95) |

**Timeout Protection:**
- Each stage has `max_duration` parameter
- Prevents infinite loops if completion never achieved
- Logs warning and forces transition

---

## Data Flow

### Stage B1: Pure Hybrid A*

```
┌─────────────────┐
│ obstacle_provider│ 
└────────┬────────┘
         │ /occupancy_grid (nav_msgs/OccupancyGrid)
         v
┌─────────────────────────────────────┐
│  gik9dof_solver (Stage B1)          │
│                                     │
│  1. Receive grid, goal              │
│  2. Call Hybrid A*:                 │
│     path = planner_.plan(           │
│         current_pose_,              │
│         goal_pose_,                 │
│         grid_                       │
│     );                              │
│                                     │
│  3. Send to Pure Pursuit:           │
│     [vx, wz] = purepursuit_.        │
│         compute(path);              │
│                                     │
│  4. Publish /cmd_vel                │
└─────────────────────────────────────┘
```

### Stage B2: GIK-Assisted

```
┌─────────────────┐
│ obstacle_provider│
└────────┬────────┘
         │ /occupancy_grid
         v
┌─────────────────────────────────────┐
│  gik9dof_solver (Stage B2)          │
│                                     │
│  1. Strategic planning:             │
│     path = planner_.plan(...)       │
│                                     │
│  2. FOR each waypoint:              │
│     chassis_config = gik_.          │
│         solve3DOFChassis(waypoint)  │
│                                     │
│  3. Velocity from config:           │
│     [vx, wz] = velocity_.           │
│         compute(chassis_config)     │
│                                     │
│  4. Publish /cmd_vel                │
└─────────────────────────────────────┘
```

---

## Performance Targets

| Component | Target | Expected (ARM64) |
|-----------|--------|------------------|
| **GIK Solver** | < 10ms | 3-5ms |
| **Hybrid A*** | < 50ms | 10-30ms |
| **Velocity Control** | < 1ms | < 0.5ms |
| **Total (Stage B1)** | < 60ms | 15-35ms |
| **Total (Stage B2)** | < 100ms | 30-60ms |

**Speedup vs MATLAB:**
- Hybrid A* codegen: 3-4× faster than MATLAB
- ARM64 on Orin: 2-3× faster than x86 codegen
- **Total expected: 6-12× speedup**

---

## Integration Checklist

- [x] Generate ARM64 C++ code (MATLAB Coder)
- [x] Decide on architecture (integrate vs separate)
- [ ] Copy generated code to `ros2/gik9dof_solver/src/generated/planner/`
- [ ] Update `CMakeLists.txt` to link planner library
- [ ] Add Stage B execution methods
- [ ] Implement mode/stage state machine
- [ ] Add ROS2 parameters for all modes
- [ ] Subscribe to `/occupancy_grid`
- [ ] Test Stage B1 (Pure Hybrid A*)
- [ ] Test Stage B2 (GIK-Assisted)
- [ ] Deploy to NVIDIA AGX Orin
- [ ] Validate real-time performance
- [ ] Integration testing with full pipeline

---

## Future Enhancements

### Path Smoothing
- Add cubic spline fitting to Hybrid A* output
- Smooth lattice discretization artifacts
- Optimize velocity profiles
- Minimize jerk for better execution

### Dynamic Replanning
- Real-time grid updates from sensors
- Trigger replanning when new obstacles detected
- Seamless transition to new path

### Velocity Profile Optimization
- Time-optimal velocity along path
- Respect acceleration limits
- Smooth acceleration/deceleration

### Multi-Goal Planning
- Chain multiple navigation goals
- Optimize visit order
- Continuous replanning

---

## References

- **Hybrid A* Paper**: Dolgov et al. (2010) "Practical Search Techniques in Path Planning for Autonomous Driving"
- **State Lattice Planning**: Pivtoraiko & Kelly (2005) "Generating Near Minimal Spanning Control Sets for Constrained Motion Planning in Discrete State Spaces"
- **MATLAB Coder**: MathWorks Documentation - Code Generation for ARM Targets
- **ROS2 Humble**: https://docs.ros.org/en/humble/

---

**Document Version**: 1.0  
**Last Updated**: October 7, 2025  
**Author**: WHEELTEC Mobile Manipulator Team
