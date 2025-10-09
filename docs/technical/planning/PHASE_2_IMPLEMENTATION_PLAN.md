# Phase 2: C++ Code Generation & ROS2 Integration

**Status**: 🔄 IN PROGRESS  
**Date**: 2024-01-XX  
**Goal**: Deploy MATLAB trajectory smoothing to Jetson AGX Orin (ROS2, ARM64)

---

## Overview

Phase 2 converts the validated MATLAB prototype (Phase 1) into real-time C++ code for deployment on the mobile manipulator robot.

### Key Requirements
- ✅ **50Hz velocity output** (prevents tipping by reducing acceleration spikes)
- ✅ **10Hz GIK solving** (existing heavy computation stays at 10Hz)
- ✅ **Real-time safe** (fixed-size arrays, no dynamic memory allocation)
- ✅ **Low latency** (~75 µs per call, validated in MATLAB)

---

## Architecture Decision: Single-Timer @ 50Hz

### Original Plan vs. Revised Plan

**Original Idea**: Dual-timer architecture
```cpp
gik_timer_ @ 10Hz           velocity_control_timer_ @ 50Hz
  └─ Solve GIK (~50ms)         └─ Smooth velocity (~75µs)
  └─ Buffer waypoint           └─ Publish cmd_vel
```

**Problem**: Complexity
- Requires thread synchronization (mutex)
- Shared buffer between timers
- More code changes to existing node

**REVISED PLAN**: Single-timer @ 50Hz with conditional execution ✅
```cpp
control_timer_ @ 50Hz (20ms period)
  ├─ Every 5th tick: Solve GIK (~50ms, runs every 100ms = 10Hz)
  └─ Every tick: Smooth velocity + publish (~75µs, runs every 20ms = 50Hz)
```

### Benefits of Single-Timer Approach
1. **Simpler integration**: Uses existing timer infrastructure
2. **No threading complexity**: No mutex, no race conditions
3. **Easy configuration**: Just set `control_rate = 50.0` parameter
4. **Backward compatible**: Can still run at 10Hz if needed
5. **Incremental deployment**: Test smoothing independently of GIK changes

### Implementation Details

```cpp
class GIK9DOFSolverNode {
private:
    rclcpp::TimerBase::SharedPtr control_timer_;  // Now @ 50Hz
    
    // Velocity control mode parameter
    int velocity_control_mode_;  // 0=5-pt diff, 1=holistic, 2=pure pursuit, 3=SMOOTHING (NEW)
    
    // Waypoint buffer for smoothing (rolling window, 5 elements)
    std::deque<WaypointState> waypoint_buffer_;
    struct WaypointState {
        double x, y, theta, t;
    };
    
    // Smoothing state (persistent across calls)
    double vx_prev_, wz_prev_, ax_prev_, alpha_prev_;
    rclcpp::Time t_prev_;
    
    // Smoothing parameters (from ROS2 params)
    struct SmoothingParams {
        double vx_max = 1.5;       // m/s
        double ax_max = 1.0;       // m/s²
        double jx_max = 5.0;       // m/s³
        double wz_max = 2.0;       // rad/s
        double alpha_max = 3.0;    // rad/s²
        double jerk_wz_max = 10.0; // rad/s³
        std::string smoothing_method = "scurve";
    } smoothing_params_;
    
    // Tick counter for 10Hz GIK execution
    int control_tick_counter_ = 0;
};

void controlLoop() {
    control_tick_counter_++;
    
    // ========== 10Hz: GIK Solving (every 5th tick) ==========
    if (control_tick_counter_ % 5 == 0) {
        // Heavy: Solve IK, update waypoint buffer
        geometry_msgs::msg::Pose target_pose = getCurrentWaypointTarget();
        
        if (solveIK(target_pose)) {
            // Store new waypoint in buffer (rolling window)
            WaypointState wp;
            wp.x = target_config_[0];
            wp.y = target_config_[1];
            wp.theta = target_config_[2];
            wp.t = this->now().seconds();
            
            waypoint_buffer_.push_back(wp);
            if (waypoint_buffer_.size() > 5) {
                waypoint_buffer_.pop_front();
            }
            
            publishJointCommand();  // Arm commands
        }
    }
    
    // ========== 50Hz: Velocity Smoothing (every tick) ==========
    if (velocity_control_mode_ == 3) {  // MODE 3: Trajectory Smoothing
        publishBaseCommandSmoothed();
    } else {
        publishBaseCommand();  // Existing modes 0/1/2
    }
}
```

---

## Code Generation Configuration

### MATLAB Coder Settings
```matlab
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.EnableVariableSizing = false;        % Fixed-size arrays only
cfg.DynamicMemoryAllocation = 'Off';     % Real-time safe (no malloc)
cfg.GenCodeOnly = true;                  % Generate code only (no build)
```

### Input Types (Fixed-Size Arrays)
```matlab
MAX_WAYPOINTS = 5;

ARGS = {
    coder.typeof(0, [5, 1], [0 0]),  % waypoints_x
    coder.typeof(0, [5, 1], [0 0]),  % waypoints_y
    coder.typeof(0, [5, 1], [0 0]),  % waypoints_theta
    coder.typeof(0, [5, 1], [0 0]),  % t_waypoints
    coder.typeof(0),                 % t_current
    params_struct                    % params
};
```

### Generated Files
- `smoothTrajectoryVelocity.cpp` - Main implementation (~5-10 KB)
- `smoothTrajectoryVelocity.h` - Header file (~2 KB)
- `html/report.mldatx` - Code generation report

---

## Integration Checklist

### Step 1: Generate C++ Code (WSL)
```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
matlab -batch "run('scripts/codegen/generate_code_trajectory_smoothing.m')"
```

**Expected output**:
```
✅ Code generation successful! (X.XX seconds)
✓ smoothTrajectoryVelocity.cpp (X KB)
✓ smoothTrajectoryVelocity.h (X KB)
✓ html/report.mldatx (HTML report)
```

### Step 2: Copy Generated Files
```bash
mkdir -p ros2/gik9dof_solver/src/trajectory_smoothing
mkdir -p ros2/gik9dof_solver/include/trajectory_smoothing

cp codegen/trajectory_smoothing/*.cpp \
   ros2/gik9dof_solver/src/trajectory_smoothing/

cp codegen/trajectory_smoothing/*.h \
   ros2/gik9dof_solver/include/trajectory_smoothing/
```

### Step 3: Update CMakeLists.txt
```cmake
# Add trajectory smoothing library
file(GLOB TRAJECTORY_SMOOTHING_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_smoothing/*.cpp")

add_library(trajectory_smoothing STATIC ${TRAJECTORY_SMOOTHING_SOURCES})

target_include_directories(trajectory_smoothing PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_smoothing
)

# Link to main node
target_link_libraries(gik9dof_solver_node
  gik_matlab_solver
  trajectory_smoothing  # NEW
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
)
```

### Step 4: Modify Node Header (`gik9dof_solver_node.h`)
```cpp
#include "smoothTrajectoryVelocity.h"  // Generated C++ header

class GIK9DOFSolverNode : public rclcpp::Node {
private:
    // Waypoint buffer (rolling window)
    struct WaypointState {
        double x, y, theta, t;
    };
    std::deque<WaypointState> waypoint_buffer_;
    static constexpr size_t MAX_WAYPOINT_BUFFER_SIZE = 5;
    
    // Smoothing state
    double vx_prev_ = 0.0;
    double wz_prev_ = 0.0;
    double ax_prev_ = 0.0;
    double alpha_prev_ = 0.0;
    rclcpp::Time t_prev_;
    
    // Smoothing parameters
    struct SmoothingParams {
        double vx_max;
        double ax_max;
        double jx_max;
        double wz_max;
        double alpha_max;
        double jerk_wz_max;
        char smoothing_method[10];  // "scurve" for C++ compatibility
    } smoothing_params_;
    
    // Tick counter for 10Hz GIK
    int control_tick_counter_ = 0;
    
    // New method
    void publishBaseCommandSmoothed();
};
```

### Step 5: Modify Node Source (`gik9dof_solver_node.cpp`)

**Constructor: Declare parameters**
```cpp
// Smoothing parameters (velocity_control_mode == 3)
this->declare_parameter("smoothing.vx_max", 1.5);
this->declare_parameter("smoothing.ax_max", 1.0);
this->declare_parameter("smoothing.jx_max", 5.0);
this->declare_parameter("smoothing.wz_max", 2.0);
this->declare_parameter("smoothing.alpha_max", 3.0);
this->declare_parameter("smoothing.jerk_wz_max", 10.0);

// Read parameters
smoothing_params_.vx_max = this->get_parameter("smoothing.vx_max").as_double();
smoothing_params_.ax_max = this->get_parameter("smoothing.ax_max").as_double();
smoothing_params_.jx_max = this->get_parameter("smoothing.jx_max").as_double();
smoothing_params_.wz_max = this->get_parameter("smoothing.wz_max").as_double();
smoothing_params_.alpha_max = this->get_parameter("smoothing.alpha_max").as_double();
smoothing_params_.jerk_wz_max = this->get_parameter("smoothing.jerk_wz_max").as_double();
strcpy(smoothing_params_.smoothing_method, "scurve");

// Initialize state
t_prev_ = this->now();
```

**controlLoop: Add conditional GIK + always smooth**
```cpp
void GIK9DOFSolverNode::controlLoop() {
    // Check state...
    
    control_tick_counter_++;
    
    // ========== 10Hz: GIK Solving (every 5th tick @ 50Hz) ==========
    if (control_tick_counter_ % 5 == 0) {
        geometry_msgs::msg::Pose target_pose;
        bool has_target = getCurrentWaypointTarget(target_pose);
        
        if (has_target) {
            if (solveIK(target_pose)) {
                // Store waypoint in buffer
                WaypointState wp;
                wp.x = target_config_[0];
                wp.y = target_config_[1];
                wp.theta = target_config_[2];
                wp.t = this->now().seconds();
                
                waypoint_buffer_.push_back(wp);
                if (waypoint_buffer_.size() > MAX_WAYPOINT_BUFFER_SIZE) {
                    waypoint_buffer_.pop_front();
                }
                
                publishJointCommand();
            }
        }
    }
    
    // ========== 50Hz: Velocity Control (every tick) ==========
    if (velocity_control_mode_ == 3) {
        publishBaseCommandSmoothed();
    } else {
        publishBaseCommand();  // Existing modes
    }
}
```

**publishBaseCommandSmoothed: New method**
```cpp
void GIK9DOFSolverNode::publishBaseCommandSmoothed() {
    if (waypoint_buffer_.size() < 2) {
        // Not enough waypoints, stop
        publishZeroVelocity();
        return;
    }
    
    // Convert deque to fixed-size arrays for generated C++ function
    double waypoints_x[5] = {0};
    double waypoints_y[5] = {0};
    double waypoints_theta[5] = {0};
    double t_waypoints[5] = {0};
    
    size_t n = std::min(waypoint_buffer_.size(), (size_t)5);
    for (size_t i = 0; i < n; i++) {
        waypoints_x[i] = waypoint_buffer_[i].x;
        waypoints_y[i] = waypoint_buffer_[i].y;
        waypoints_theta[i] = waypoint_buffer_[i].theta;
        t_waypoints[i] = waypoint_buffer_[i].t;
    }
    
    double t_current = this->now().seconds();
    
    // Output variables
    double vx_cmd, wz_cmd, ax_cmd, alpha_cmd, jerk_vx_cmd, jerk_wz_cmd;
    
    // Call generated C++ function
    smoothTrajectoryVelocity(
        waypoints_x,
        waypoints_y,
        waypoints_theta,
        t_waypoints,
        t_current,
        &smoothing_params_,
        &vx_cmd, &wz_cmd, &ax_cmd, &alpha_cmd, &jerk_vx_cmd, &jerk_wz_cmd
    );
    
    // Publish velocity command
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = vx_cmd;
    msg.angular.z = wz_cmd;
    base_cmd_pub_->publish(msg);
    
    // Debug logging (throttled)
    static int log_counter = 0;
    if (++log_counter >= 250) {  // Every 5 seconds @ 50Hz
        RCLCPP_DEBUG(this->get_logger(),
            "Smoothed: vx=%.3f m/s, wz=%.3f rad/s, ax=%.3f m/s², buffer=%zu",
            vx_cmd, wz_cmd, ax_cmd, waypoint_buffer_.size());
        log_counter = 0;
    }
}
```

### Step 6: Update Config File
```yaml
# config/gik9dof_solver_params.yaml

gik9dof_solver_node:
  ros__parameters:
    control_rate: 50.0              # NEW: 50Hz for smooth velocity output
    velocity_control_mode: 3         # NEW: 3 = Trajectory Smoothing
    
    # Smoothing parameters
    smoothing:
      vx_max: 1.5                    # m/s
      ax_max: 1.0                    # m/s²
      jx_max: 5.0                    # m/s³
      wz_max: 2.0                    # rad/s
      alpha_max: 3.0                 # rad/s²
      jerk_wz_max: 10.0              # rad/s³
```

### Step 7: Build
```bash
cd ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_solver
```

### Step 8: Test
```bash
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

**Expected log output**:
```
[INFO] GIK9DOF Solver Node initialized
[INFO] Control rate: 50.0 Hz
[INFO] Velocity control mode: Trajectory Smoothing (Mode 3)
```

---

## Performance Expectations

### CPU Load
- **10Hz @ 10Hz** (original): ~5% CPU (50ms solve every 100ms)
- **50Hz @ 50Hz** (new): ~5.4% CPU (50ms solve every 100ms + 75µs smooth every 20ms)
  - GIK: 50ms / 100ms = 50% duty cycle = 5.0%
  - Smoothing: 75µs / 20ms = 0.375% duty cycle = 0.375%
  - **Total**: 5.375% CPU ✅

### Memory
- Waypoint buffer: 5 waypoints × 4 doubles = 160 bytes
- Smoothing state: 4 doubles = 32 bytes
- Generated code: ~10 KB static
- **Total**: ~10 KB + 192 bytes runtime ✅

### Latency
- Velocity command rate: 50Hz (20ms period)
- Smoothing computation: ~75µs (<< 20ms)
- **Latency improvement**: 100ms → 20ms (5× faster response) ✅

---

## Testing Plan

### Unit Tests
1. ✅ MATLAB validation (Phase 1 complete)
2. ⏳ C++ codegen syntax check (compile without errors)
3. ⏳ Node integration build (ROS2 compiles)
4. ⏳ Parameter loading test (reads config correctly)

### Integration Tests
1. ⏳ **50Hz output verification**: `ros2 topic hz /cmd_vel` (should report ~50Hz)
2. ⏳ **Acceleration limit check**: Record cmd_vel, verify max acceleration < 1.0 m/s²
3. ⏳ **Buffer behavior**: Test with 2-5 waypoints, verify smooth operation
4. ⏳ **Stop behavior**: Remove trajectory, verify smooth deceleration to zero

### Real Robot Tests
1. ⏳ Deploy to Jetson AGX Orin
2. ⏳ Run trajectory from `1_pull_world_scaled.json` (300 waypoints)
3. ⏳ Monitor for tipping (compare with/without smoothing)
4. ⏳ Verify CPU usage < 10% (target: ~5.4%)

---

## Rollback Plan

If issues arise, easy to revert:
1. Set `velocity_control_mode: 2` (Pure Pursuit) in config
2. Set `control_rate: 10.0` in config
3. Restart node

**No code changes needed** - just parameter changes ✅

---

## Next Steps

1. ✅ Create MATLAB Coder script
2. ⏳ Run code generation on WSL
3. ⏳ Copy generated files to ROS2 package
4. ⏳ Modify node source files
5. ⏳ Update CMakeLists.txt
6. ⏳ Build on WSL
7. ⏳ Test locally
8. ⏳ Deploy to Orin
9. ⏳ Validate with real trajectory

---

## Success Criteria

- [x] Phase 1: MATLAB prototype working ✅
- [ ] Phase 2A: C++ code generation successful
- [ ] Phase 2B: ROS2 node compiles without errors
- [ ] Phase 2C: 50Hz velocity output achieved
- [ ] Phase 2D: Acceleration limits verified (< 1.0 m/s²)
- [ ] Phase 3: Deployed to Orin
- [ ] Phase 4: Real robot test (no tipping)
- [ ] Phase 5: Performance validated (CPU < 10%)

---

**Status**: Ready to generate C++ code!  
**Command**: `matlab -batch "run('scripts/codegen/generate_code_trajectory_smoothing.m')"`
