# Trajectory Smoothing Module - Implementation Plan

**Date**: October 9, 2025  
**Problem**: Discrete 10Hz waypoints causing robot tipping due to lack of acceleration/jerk limits  
**Solution**: Real-time trajectory smoothing with S-curve velocity profiles

---

## Problem Statement

### Current Behavior
- GIK solver outputs waypoints at **10 Hz** (100ms intervals)
- Velocity controller directly tracks these waypoints
- **No acceleration limits** → Sudden velocity changes
- **No jerk limits** → Sudden acceleration changes
- **Result**: Robot tips over due to instantaneous velocity jumps

### Root Cause
```
Waypoint N   →   Waypoint N+1  (100ms gap)
vx = 0.5 m/s →   vx = 1.5 m/s  (Δv = 1.0 m/s)
Acceleration = 1.0 m/s / 0.1s = 10 m/s²  ← WAY TOO HIGH!
```

**Typical safe limits**:
- Forward acceleration: `ax_max = 1.0 m/s²` (prevents tipping)
- Angular acceleration: `alpha_max = 3.0 rad/s²`
- Forward jerk: `jx_max = 5.0 m/s³` (smooth acceleration changes)
- Angular jerk: `jerk_wz_max = 10.0 rad/s³`

---

## Proposed Solution: S-Curve Velocity Profiling

### Strategy
**Input**: Discrete waypoints at 10Hz (from GIK)  
**Output**: Smooth velocity commands at 50-100Hz (to motor controller)  
**Method**: S-curve acceleration profile with jerk limiting

### Key Features
1. ✅ **Respects waypoint sequence** (doesn't skip or reorder)
2. ✅ **Respects velocity limits** (vx_max, wz_max)
3. ✅ **Enforces acceleration limits** (ax_max, alpha_max)
4. ✅ **Enforces jerk limits** (jx_max, jerk_wz_max)
5. ✅ **Real-time capable** (low computational cost)
6. ⚠️ **Time-flexible** (may take longer to reach waypoint safely)

### S-Curve Profile
```
Acceleration (a)
    ^
    |     ╱‾‾‾╲
    |    ╱     ╲___
    |___╱          ╲___
    +-------------------> Time
    Phase 1  Phase 2  Phase 3
    (jerk↑)  (const)  (jerk↓)
```

- **Phase 1**: Jerk-up (acceleration increases linearly)
- **Phase 2**: Constant acceleration
- **Phase 3**: Jerk-down (acceleration decreases linearly)

Result: **Smooth velocity transition** without sudden jumps

---

## Implementation Plan

### Phase 1: MATLAB Prototype ✅ **DONE**

**Files Created**:
1. `matlab/+gik9dof/+control/smoothTrajectoryVelocity.m` - Main smoothing function
2. `matlab/test_trajectory_smoothing.m` - Test script

**Testing** (Windows MATLAB):
```matlab
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab
test_trajectory_smoothing
```

**Expected Output**:
- Plots showing smooth velocity/acceleration/jerk profiles
- Verification that all limits are respected
- No sudden jumps in velocity or acceleration

**Validation Criteria**:
- ✅ Max acceleration ≤ 1.0 m/s²
- ✅ Max jerk ≤ 5.0 m/s³
- ✅ Smooth transitions between waypoints
- ✅ No oscillations or overshoot

---

### Phase 2: Code Generation (WSL MATLAB) 🔄 **NEXT**

**Objective**: Generate C++ code for real-time execution

**Script**: Create `scripts/codegen/generate_code_trajectory_smoothing.m`

**Commands** (from WSL):
```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
matlab -batch "run('scripts/codegen/generate_code_trajectory_smoothing.m')"
```

**Expected Output**:
```
codegen/trajectory_smoothing/
├── smoothTrajectoryVelocity.cpp
├── smoothTrajectoryVelocity.h
├── applySCurve.cpp
├── applySCurve.h
├── decelerateToStop.cpp
└── decelerateToStop.h
```

**Code Generation Settings**:
```matlab
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenCodeOnly = false;
cfg.GenerateReport = true;
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.BuildConfiguration = 'Faster Runs';
cfg.EnableOpenMP = false;  % Avoid complexity
```

---

### Phase 3: ROS2 Integration (WSL Build)

**Objective**: Integrate smoothing into `gik9dof_solver_node`

#### Step 3.1: Add Generated Code to ROS2 Package

**Copy Files**:
```bash
mkdir -p ros2/gik9dof_solver/src/trajectory_smoothing
cp codegen/trajectory_smoothing/*.cpp ros2/gik9dof_solver/src/trajectory_smoothing/
cp codegen/trajectory_smoothing/*.h ros2/gik9dof_solver/include/trajectory_smoothing/
```

#### Step 3.2: Update CMakeLists.txt

Add to `ros2/gik9dof_solver/CMakeLists.txt`:
```cmake
# Trajectory smoothing library
file(GLOB TRAJECTORY_SMOOTHING_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_smoothing/*.cpp"
)

add_library(trajectory_smoothing STATIC ${TRAJECTORY_SMOOTHING_SOURCES})
target_include_directories(trajectory_smoothing PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_smoothing
)

# Link to main node
target_link_libraries(gik9dof_solver_node
  gik_matlab_solver
  stage_b_controller
  trajectory_smoothing  # NEW
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
```

#### Step 3.3: Modify Node Code

**In `gik9dof_solver_node.h`**:
```cpp
#include "trajectory_smoothing/smoothTrajectoryVelocity.h"

class GIK9DOFSolverNode : public rclcpp::Node {
private:
    // Trajectory smoothing state
    struct SmoothingState {
        double vx_prev = 0.0;
        double wz_prev = 0.0;
        double ax_prev = 0.0;
        double alpha_prev = 0.0;
        rclcpp::Time t_prev;
    };
    SmoothingState smoothing_state_;
    
    // Waypoint buffer (5 waypoints = 500ms lookahead)
    std::deque<geometry_msgs::msg::Pose> waypoint_buffer_;
    std::deque<rclcpp::Time> waypoint_times_;
    static constexpr size_t WAYPOINT_BUFFER_SIZE = 5;
    
    // Smoothing parameters
    struct SmoothingParams {
        double vx_max = 1.5;
        double ax_max = 1.0;
        double jx_max = 5.0;
        double wz_max = 2.0;
        double alpha_max = 3.0;
        double jerk_wz_max = 10.0;
    };
    SmoothingParams smoothing_params_;
};
```

**In `gik9dof_solver_node.cpp` - Update Velocity Control Loop**:
```cpp
void GIK9DOFSolverNode::publishBaseCommand()
{
    // ... existing code ...
    
    // NEW: Apply trajectory smoothing
    if (use_trajectory_smoothing_) {
        auto now = this->now();
        
        // Add current reference to waypoint buffer
        waypoint_buffer_.push_back(reference_pose_);
        waypoint_times_.push_back(now);
        
        // Maintain buffer size
        if (waypoint_buffer_.size() > WAYPOINT_BUFFER_SIZE) {
            waypoint_buffer_.pop_front();
            waypoint_times_.pop_front();
        }
        
        // Extract waypoint arrays
        std::vector<double> x_waypoints, y_waypoints, theta_waypoints, t_waypoints;
        for (size_t i = 0; i < waypoint_buffer_.size(); ++i) {
            x_waypoints.push_back(waypoint_buffer_[i].position.x);
            y_waypoints.push_back(waypoint_buffer_[i].position.y);
            // Extract yaw from quaternion
            double yaw = extractYaw(waypoint_buffer_[i].orientation);
            theta_waypoints.push_back(yaw);
            t_waypoints.push_back(waypoint_times_[i].seconds());
        }
        
        // Call smoothing function
        double vx_smooth, wz_smooth, ax_smooth, alpha_smooth;
        smoothTrajectoryVelocity(
            x_waypoints.data(), y_waypoints.data(), theta_waypoints.data(),
            t_waypoints.data(), now.seconds(), waypoint_buffer_.size(),
            smoothing_params_,
            &vx_smooth, &wz_smooth, &ax_smooth, &alpha_smooth);
        
        // Use smoothed velocities instead of raw
        vx_robot = vx_smooth;
        wz = wz_smooth;
        
        // Store for next iteration
        smoothing_state_.vx_prev = vx_smooth;
        smoothing_state_.wz_prev = wz_smooth;
        smoothing_state_.ax_prev = ax_smooth;
        smoothing_state_.alpha_prev = alpha_smooth;
        smoothing_state_.t_prev = now;
    }
    
    // ... rest of existing code ...
}
```

#### Step 3.4: Add ROS2 Parameters

**In `gik9dof_solver_params.yaml`**:
```yaml
gik9dof_solver_node:
  ros__parameters:
    # ... existing parameters ...
    
    # ========== TRAJECTORY SMOOTHING ==========
    use_trajectory_smoothing: true  # Enable/disable smoothing
    
    # Acceleration limits (prevent tipping)
    smoothing:
      vx_max: 1.5          # Max forward velocity (m/s)
      ax_max: 1.0          # Max forward acceleration (m/s²)
      jx_max: 5.0          # Max forward jerk (m/s³)
      wz_max: 2.0          # Max angular velocity (rad/s)
      alpha_max: 3.0       # Max angular acceleration (rad/s²)
      jerk_wz_max: 10.0    # Max angular jerk (rad/s³)
      method: 'scurve'     # 'scurve' or 'exponential'
```

---

### Phase 4: Build and Test (WSL)

#### Step 4.1: Build on WSL
```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --packages-select gik9dof_msgs
source install/setup.bash
colcon build --packages-select gik9dof_solver
```

**Expected**: ✅ Build SUCCESS

#### Step 4.2: Unit Test (C++)
Create `ros2/gik9dof_solver/test/test_trajectory_smoothing.cpp`:
```cpp
#include <gtest/gtest.h>
#include "trajectory_smoothing/smoothTrajectoryVelocity.h"

TEST(TrajectorySmoothing, AccelerationLimits) {
    // Test that acceleration stays within limits
    // ...
}

TEST(TrajectorySmoothing, JerkLimits) {
    // Test that jerk stays within limits
    // ...
}
```

---

### Phase 5: Deploy to Orin

#### Step 5.1: Sync to Orin
```bash
# From WSL
rsync -avz --delete /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2/ \
    cr@192.168.100.150:/home/nvidia/temp_gikrepo/ros2/
```

#### Step 5.2: Build on Orin
```bash
ssh cr@192.168.100.150
cd /home/nvidia/temp_gikrepo/ros2
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --packages-select gik9dof_msgs gik9dof_solver
```

#### Step 5.3: Runtime Test
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file install/gik9dof_solver/share/gik9dof_solver/config/gik9dof_solver_params.yaml
```

**Monitor**:
```bash
# In another terminal
ros2 topic echo /gik9dof/solver_diagnostics
ros2 topic echo /cmd_vel
```

**Verify**:
- ✅ Smooth velocity transitions (no jumps)
- ✅ Robot doesn't tip over
- ✅ Waypoints still followed (within tolerance)
- ✅ Control loop maintains 50-100Hz

---

## Testing Strategy

### Test 1: MATLAB Prototype (Phase 1) ✅
**Run**: `test_trajectory_smoothing.m`
**Check**: Plots show smooth profiles, limits respected

### Test 2: Code Generation (Phase 2)
**Run**: WSL MATLAB codegen
**Check**: C++ files generated, compilable

### Test 3: WSL Build (Phase 3-4)
**Run**: colcon build on WSL
**Check**: No build errors, links successfully

### Test 4: Orin Deployment (Phase 5)
**Run**: On actual robot
**Check**: Robot moves smoothly, no tipping

### Test 5: Stress Test
**Scenario**: Rapid waypoint changes (worst case)
**Check**: Still respects limits, doesn't crash

---

## Alternative Approaches Considered

### Option 1: S-Curve (CHOSEN) ⭐
**Pros**: Best smoothness, explicit jerk control, proven in robotics
**Cons**: Slightly more complex (but still real-time capable)

### Option 2: Exponential Smoothing
**Pros**: Simplest to implement, fastest
**Cons**: No explicit jerk control, harder to tune

### Option 3: Cubic Spline
**Pros**: Mathematically optimal path
**Cons**: May overshoot waypoints, expensive to compute

### Option 4: Time-Optimal Planning
**Pros**: Fastest possible motion
**Cons**: Complex, may require solving optimization problem

**Decision**: S-curve offers best balance of performance and implementation simplicity.

---

## Parameters to Tune

### Critical Parameters (Prevent Tipping)
- `ax_max = 1.0 m/s²` - **Start conservative**, increase if stable
- `alpha_max = 3.0 rad/s²` - Rotational acceleration
- `jx_max = 5.0 m/s³` - Smoothness of acceleration changes

### Performance Parameters
- `vx_max = 1.5 m/s` - Already configured
- `wz_max = 2.0 rad/s` - Already configured

### Tuning Procedure
1. Start with conservative limits (ax_max=0.5 m/s²)
2. Test on robot, verify no tipping
3. Gradually increase limits until robot becomes slightly unstable
4. Back off 20% from unstable limit → final value

---

## Success Criteria

### Must Have ✅
- [ ] Robot does NOT tip over during normal operation
- [ ] Acceleration ≤ ax_max (1.0 m/s²)
- [ ] Jerk ≤ jx_max (5.0 m/s³)
- [ ] Waypoints still followed (within 0.3m tolerance)

### Should Have ✅
- [ ] Smooth velocity transitions (no visible jerks)
- [ ] Real-time performance (< 1ms per smoothing call)
- [ ] Works with existing Pure Pursuit controller

### Nice to Have
- [ ] Configurable via ROS2 parameters
- [ ] Diagnostics showing actual accel/jerk
- [ ] Automatic tuning based on robot dynamics

---

## Timeline

**Phase 1 (MATLAB Test)**: ✅ **DONE** (30 minutes)
**Phase 2 (Codegen)**: 1-2 hours
**Phase 3 (ROS2 Integration)**: 2-3 hours
**Phase 4 (WSL Build & Test)**: 1 hour
**Phase 5 (Orin Deploy & Test)**: 2 hours

**Total Estimated Time**: **1 day** (6-8 hours)

---

## Next Steps

**IMMEDIATE**:
1. ✅ Run `test_trajectory_smoothing.m` on Windows MATLAB
2. ✅ Verify plots show smooth profiles
3. ✅ Confirm limits are respected

**THEN**:
4. Create codegen script for WSL
5. Generate C++ code
6. Integrate into ROS2 node
7. Build on WSL
8. Deploy to Orin
9. Test on real robot

---

## Risk Mitigation

### Risk 1: Robot still tips over
**Mitigation**: Start with very conservative limits (ax_max=0.5), increase gradually

### Risk 2: Too slow to follow trajectory
**Mitigation**: Acceptable trade-off - safety > speed. GIK can adapt to slower motion.

### Risk 3: Smoothing adds latency
**Mitigation**: Use small buffer (5 waypoints = 500ms), minimal impact

### Risk 4: Code generation issues
**Mitigation**: Keep MATLAB function simple, avoid persistent variables if needed

---

**Status**: Phase 1 COMPLETE ✅  
**Ready for**: Phase 2 - Code Generation

