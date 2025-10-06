# Configuration Answers & Decisions - CodegenCC45

**Date**: 2025-10-06  
**Status**: Requirements Confirmed, Ready for Implementation

---

## Confirmed Requirements

### 1. Robot Model
- **URDF**: `mobile_manipulator_PPR_base_corrected.urdf` ✅
- **Mesh Files**: Current STLs in `/meshes/` ✅
- **Future**: Sparser mesh patches (drop-in replacement, no code changes needed)

### 2. Trajectory Input Design

**Decision**: Rolling window trajectory streaming via ROS2 topic

#### Message Design: `EndEffectorTrajectory.msg`

```msg
# Rolling window of target EE waypoints
Header header

# Array of target poses (typically 3-10 waypoints ahead)
geometry_msgs/PoseStamped[] waypoints

# Relative timestamps for each waypoint
float64[] timestamps  # Seconds from header.stamp

# Execution mode
string mode  # "tracking", "goal", "hold"

# Constraint parameters
float64 distance_lower_bound  # Minimum distance from obstacles (m)
float64 distance_weight       # Weight for distance constraint
```

#### Trajectory Streaming Strategy

**Option A: Fixed-size Rolling Window** (RECOMMENDED)
- Publisher sends next 5-10 waypoints
- Controller tracks first waypoint, removes it when reached
- Publisher continuously adds new waypoints to maintain window
- **Advantage**: Smooth tracking, predictable memory, codegen-friendly

**Option B: Goal-based**
- Publisher sends goal pose + intermediate waypoints
- Controller generates fine trajectory
- **Advantage**: Less network traffic
- **Disadvantage**: More complex trajectory generation in C++

**Selected**: Option A (rolling window) for initial implementation

#### Topic Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Topic name | `/target/end_effector_trajectory` | Clear naming |
| Message type | `gik9dof_msgs/msg/EndEffectorTrajectory` | Custom message |
| Publishing rate | 1-5 Hz | Updates every 200-1000ms |
| Window size | 5-10 waypoints | Covers 0.5-1.0 seconds ahead |
| Queue depth | 2 | Latest trajectory only |

### 3. Control & Sensor Frequencies

| Component | Initial | Target | Configurable |
|-----------|---------|--------|--------------|
| **Control loop** | 10 Hz | 10 Hz | Via YAML |
| **Odometry** | 10 Hz | 50 Hz | Via YAML |
| **Joint states** | 10 Hz | 50 Hz | Via YAML |
| **Trajectory updates** | 1-5 Hz | Variable | Via topic rate |

**Design Principle**: All frequencies configurable via ROS2 parameters

#### Configuration YAML

```yaml
gik9dof_controllers:
  ros__parameters:
    # Control rates
    control_rate: 10.0  # Hz, main control loop
    
    # Sensor timeout detection
    odom_timeout: 0.2  # seconds, 2x period at 10 Hz
    joint_state_timeout: 0.2  # seconds
    trajectory_timeout: 2.0  # seconds
    
    # Future expansion ready
    enable_high_frequency_mode: false  # Set true for 50 Hz sensors
```

### 4. Navigation Integration

**Decision**: Use MATLAB Navigation Toolbox components initially, not full ROS2 Nav2

#### Phase 1 (Current Scope)
- ✅ Unified chassis controller (from MATLAB)
- ✅ Holistic whole-body control
- ❌ No Nav2 costmap integration yet
- ❌ No Nav2 path planner yet

#### Phase 2 (Future Enhancement)
- 🔄 Integrate Nav2 costmap for obstacle representation
- 🔄 Add Nav2 path planner for staged base-only motion
- 🔄 DWB controller for local planning (if needed)

**Rationale**: 
- Start simple with holistic control
- Perception already provides OccupancyGrid
- Add Nav2 integration after core functionality proven

### 5. Obstacle Representation

**Format**: OccupancyGrid (ROS2 standard)

#### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Message type | `nav_msgs/msg/OccupancyGrid` | Standard ROS2 |
| Resolution | 0.1 m (10 cm cells) | Confirmed |
| Frame | `odom` or `map` | TBD based on SLAM setup |
| Update rate | Variable | From perception module |
| Occupancy values | 0-100 | 0=free, 100=occupied, -1=unknown |

#### Integration into IK Solver

**Option 1**: Pre-process grid → distance constraints
- Convert OccupancyGrid to distance field
- Use distance bounds constraint in GIK
- **Challenge**: Distance field computation expensive

**Option 2**: Sample-based collision checking
- Check FK poses against grid
- Reject configurations in occupied cells
- **Challenge**: Not smooth, discrete checking

**Option 3**: Inflate obstacles → safe boundary
- Inflate occupied cells by robot radius
- Add distance constraint for end-effector
- **Recommended**: Simple, codegen-friendly

**Selected**: Option 3 (inflated obstacles) for Phase 1

### 6. Testing Strategy

**Dual-track approach**: Simulation + Hardware in parallel

#### Track 1: Simulation Validation
- **Platform**: MATLAB simulation (existing)
- **Purpose**: Algorithm verification, regression testing
- **Scope**: Compare C++ output with MATLAB baseline
- **Frequency**: Continuous (every code change)

#### Track 2: Hardware Testing
- **Platform**: AGX Orin with real hardware
- **Purpose**: Real-time performance, hardware integration
- **Scope**: Full system validation
- **Frequency**: Weekly milestones

#### Testing Phases

| Week | Simulation | Hardware |
|------|------------|----------|
| 1-2 | MATLAB refactor verification | Setup AGX Orin environment |
| 3 | Code generation validation | Deploy solver library |
| 4 | ROS2 node unit tests | Basic IK on hardware |
| 5 | Full system simulation | Integrated system test |
| 6+ | Regression tests | Performance optimization |

### 7. Integration with Existing ROS2 Stack

**Confirmed interfaces**:

#### Inputs from Existing Modules

| Module | Topic | Message Type | Frequency |
|--------|-------|--------------|-----------|
| Perception | `/occupancy_grid` (TBD) | `nav_msgs/msg/OccupancyGrid` | Variable |
| Trajectory Planner | `/target/end_effector_trajectory` | `gik9dof_msgs/msg/EndEffectorTrajectory` | 1-5 Hz |
| SLAM | `/odom_wheel` | `nav_msgs/msg/Odometry` | 10 Hz |
| Arm Controller | `/hdas/feedback_arm_left` | `sensor_msgs/msg/JointState` | 10 Hz |

#### Outputs to Existing Modules

| Target Module | Topic | Message Type | Frequency |
|---------------|-------|--------------|-----------|
| Arm Controller | `/motion_target/target_joint_state_arm_left` | `sensor_msgs/msg/JointState` | 10 Hz |
| Chassis Controller | `/mobile_base/commands/velocity` | `geometry_msgs/msg/Twist` | 10 Hz |
| Monitoring | `/gik9dof/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz |

#### Message Compatibility Requirements

- ✅ Use standard ROS2 message types where possible
- ✅ Custom messages only when necessary (trajectory, diagnostics)
- ✅ Frame IDs must match existing TF tree
- ✅ Timestamps synchronized with system clock

---

## Updated Design Decisions

### Trajectory Tracking Architecture

```
┌────────────────────────────────────────────────────┐
│ Trajectory Planner (Existing)                      │
│ - Task-level planning                              │
│ - Generates EE waypoints                           │
└────────────┬───────────────────────────────────────┘
             │ Rolling window (5-10 waypoints)
             │ /target/end_effector_trajectory
             ▼
┌────────────────────────────────────────────────────┐
│ gik9dof_solver_node                                │
│ - Receives waypoint window                         │
│ - Tracks first waypoint (IK solve)                 │
│ - Requests new waypoints when nearing end          │
└────────────┬───────────────────────────────────────┘
             │ JointState commands (10 Hz)
             │ /motion_target/target_joint_state_arm_left
             ▼
┌────────────────────────────────────────────────────┐
│ Arm Controller (Existing)                          │
│ - Executes joint commands                          │
└────────────────────────────────────────────────────┘
```

### Control Loop Timing

```
┌─────────────────────────────────────────────────┐
│ 100ms Control Cycle (10 Hz)                    │
├─────────────────────────────────────────────────┤
│ 0-10ms:   Read sensors (odom, joint states)    │
│ 10-20ms:  IK solve (target: <10ms)             │
│ 20-30ms:  Chassis control computation          │
│ 30-40ms:  Collision checking (if enabled)      │
│ 40-50ms:  Publish commands                     │
│ 50-100ms: Buffer for jitter, diagnostics       │
└─────────────────────────────────────────────────┘

Target: <50ms computation, 50ms safety margin
```

### Configurable Parameter Structure

```yaml
gik9dof_system:
  ros__parameters:
    # Timing
    control_rate: 10.0
    sensor_timeout_multiplier: 2.0  # timeout = period * multiplier
    
    # Trajectory tracking
    trajectory_window_size: 5  # number of waypoints
    waypoint_reached_tolerance: 0.01  # meters
    
    # IK solver
    max_iterations: 100
    convergence_tolerance: 0.001
    distance_lower_bound: 0.2
    
    # Chassis control
    max_linear_velocity: 0.5
    max_angular_velocity: 1.0
    yaw_kp: 2.0
    yaw_kff: 0.8
    
    # Obstacle handling
    occupancy_grid_inflation_radius: 0.15  # meters
    occupancy_threshold: 50  # 0-100, cells > threshold = occupied
    
    # High-frequency mode (future)
    enable_50hz_mode: false
```

---

## Implementation Priorities

### Phase 1: Core Functionality (Weeks 1-4)

1. ✅ MATLAB code refactoring (procedural robot builder)
2. ✅ Code generation for IK solver
3. ✅ ROS2 solver library
4. ✅ Basic solver node (10 Hz control)
5. ✅ Rolling window trajectory tracking
6. ⚠️ **Defer**: OccupancyGrid integration (use distance constraint only)

### Phase 2: Integration (Weeks 5-6)

1. ✅ Integrate with existing perception (OccupancyGrid subscriber)
2. ✅ Integrate with trajectory planner
3. ✅ Hardware testing on AGX Orin
4. ✅ Performance optimization
5. ⚠️ **Defer**: Nav2 integration

### Phase 3: Enhancement (Weeks 7-8+)

1. 🔄 50 Hz mode support
2. 🔄 OccupancyGrid-based collision avoidance
3. 🔄 Staged control mode (if needed)
4. 🔄 Nav2 integration for base planning

---

## Updated Message Definitions

### `EndEffectorTrajectory.msg` (Revised)

```msg
# Rolling window trajectory for end-effector tracking

Header header

# Waypoint array (typically 5-10 points)
# First waypoint is immediate target
geometry_msgs/PoseStamped[] waypoints

# Relative time to reach each waypoint (seconds from header.stamp)
float64[] timestamps

# Execution parameters
string mode  # "tracking", "goal", "hold"
bool blocking  # Wait for completion vs continuous tracking

# Constraint parameters
float64 distance_lower_bound  # Minimum distance from obstacles (m)
float64 distance_weight       # Weight for distance constraint

# Request flag
bool request_more_waypoints  # Controller can set this to request extension
```

### `SolverDiagnostics.msg` (Revised)

```msg
# Enhanced diagnostics for monitoring

Header header

# Solver status
bool converged
int32 iterations
float64 solve_time_ms
float64 control_loop_time_ms  # Total cycle time

# Error metrics
float64 position_error_m
float64 orientation_error_rad
float64 joint_limit_violation_rad

# Current state
float64[] joint_positions  # 9 DOF
float64[] joint_velocities  # Computed rates
geometry_msgs/Pose current_ee_pose
geometry_msgs/Pose target_ee_pose

# Trajectory tracking
int32 waypoints_remaining
float64 trajectory_completion_percent

# Safety flags
bool sensor_timeout
bool velocity_limited
bool emergency_stop
```

---

## Risk Assessment Updates

### Low Risk (Mitigated)
- ✅ URDF confirmed and stable
- ✅ Control frequency reasonable (10 Hz)
- ✅ Hardware available for testing
- ✅ Existing ROS2 infrastructure present

### Medium Risk (Managed)
- ⚠️ Trajectory streaming design (new approach, needs testing)
- ⚠️ OccupancyGrid integration complexity (deferred to Phase 2)
- ⚠️ Real-time performance on AGX Orin (requires profiling)

### High Risk (Monitor Closely)
- 🔴 MATLAB Coder compatibility (incremental testing required)
- 🔴 Integration with existing modules (message compatibility critical)

---

## Action Items

### Immediate (This Week)
- [ ] Update ROS2_INTEGRATION_GUIDE.md with revised message definitions
- [ ] Create trajectory streaming protocol document
- [ ] Setup AGX Orin development environment
- [ ] Create MATLAB test harness for trajectory streaming

### Week 1
- [ ] Start MATLAB refactoring (procedural robot builder)
- [ ] Design trajectory publisher interface
- [ ] Setup ROS2 workspace on AGX Orin
- [ ] Create simulation test cases

### Week 2
- [ ] Complete code generation
- [ ] Test on Ubuntu 22.04
- [ ] Create basic ROS2 nodes
- [ ] Begin hardware integration planning

---

## Success Criteria (Updated)

### Technical
- ✅ 10 Hz control loop maintained (99% of time)
- ✅ IK solve time <10ms average
- ✅ Position error <1 cm
- ✅ Orientation error <5 deg
- ✅ Trajectory tracking latency <200ms

### Integration
- ✅ Compatible with existing perception module
- ✅ Compatible with existing trajectory planner
- ✅ Compatible with existing arm/chassis controllers
- ✅ Frame IDs consistent with TF tree

### Testing
- ✅ Passes MATLAB simulation baseline
- ✅ Passes hardware validation on AGX Orin
- ✅ 24-hour stress test without crashes
- ✅ Graceful degradation on sensor loss

---

**Document Version**: 1.1  
**Last Updated**: 2025-10-06  
**Status**: Requirements Confirmed, Implementation Ready  
**Next Review**: After Week 1 completion
