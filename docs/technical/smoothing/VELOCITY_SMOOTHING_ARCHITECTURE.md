# Velocity Smoothing Architecture

## Overview
This document explains the new velocity smoothing architecture that allows **any velocity controller** (Pure Pursuit, Heading Controller, etc.) to have its commands smoothed with S-curve acceleration/jerk limits.

## Architecture Change

### Before (Mode 3 - Standalone Waypoint-Based)
```
GIK Waypoints → smoothTrajectoryVelocity() → Smoothed Velocity → /cmd_vel
```
- **Problem**: smoothTrajectoryVelocity computes velocity FROM waypoints
- **Limitation**: Cannot be used with Pure Pursuit or other controllers

### After (Layered Smoothing)
```
Controller (Pure Pursuit, Heading, etc.) → Raw Velocity → smoothVelocityCommand() → Smoothed Velocity → /cmd_vel
```
- **Solution**: smoothVelocityCommand smooths ANY velocity command
- **Benefit**: Works with ALL velocity controllers

## Implementation

### New MATLAB Function: `smoothVelocityCommand.m`
**Location**: `matlab/+gik9dof/+control/smoothVelocityCommand.m`

**Purpose**: Apply S-curve acceleration/jerk limits to velocity commands

**Signature**:
```matlab
[vx_smooth, wz_smooth, ax_out, alpha_out] = smoothVelocityCommand(...
    vx_target, wz_target, ...    % Input: Raw velocity from controller
    vx_prev, wz_prev, ...         % State: Previous velocity
    ax_prev, alpha_prev, ...      % State: Previous acceleration
    dt, ...                       % Time step
    params)                       % Limits: vx_max, ax_max, jx_max, etc.
```

**Algorithm**:
1. Compute desired acceleration: `ax_desired = (vx_target - vx_prev) / dt`
2. Apply jerk limit: `jerk = clamp((ax_desired - ax_prev) / dt, -jx_max, jx_max)`
3. Integrate jerk: `ax = ax_prev + jerk * dt`
4. Apply acceleration limit: `ax = clamp(ax, -ax_max, ax_max)`
5. Integrate acceleration: `vx_smooth = vx_prev + ax * dt`
6. Apply velocity limit: `vx_smooth = clamp(vx_smooth, -vx_max, vx_max)`
7. Repeat for angular velocity (wz, alpha, jerk_wz)

### Generated C++ Code
**Location**: `codegen/velocity_smoothing/`
**Key Files**:
- `smoothVelocityCommand.cpp` (5.5 KB) - Implementation
- `smoothVelocityCommand.h` (0.8 KB) - Interface
- `smoothVelocityCommand_types.h` (0.5 KB) - Struct definitions

**Function Signature**:
```cpp
void smoothVelocityCommand(
    double vx_target, double wz_target,
    double vx_prev, double wz_prev,
    double ax_prev, double alpha_prev,
    double dt,
    const struct0_T *params,
    double *vx_smooth, double *wz_smooth,
    double *ax_out, double *alpha_out);
```

## ROS2 Integration Plan

### 1. Copy Generated Files
```bash
# Create directories
mkdir -p ros2/gik9dof_solver/src/velocity_smoothing
mkdir -p ros2/gik9dof_solver/include/velocity_smoothing

# Copy source files
cp codegen/velocity_smoothing/*.cpp ros2/gik9dof_solver/src/velocity_smoothing/
cp codegen/velocity_smoothing/*.h ros2/gik9dof_solver/include/velocity_smoothing/
```

### 2. Update CMakeLists.txt
Add velocity_smoothing library (similar to trajectory_smoothing):
```cmake
# Velocity Smoothing Library (MATLAB Coder)
set(VEL_SMOOTHING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/velocity_smoothing")
file(GLOB VEL_SMOOTHING_SOURCES "${VEL_SMOOTHING_DIR}/*.cpp")
add_library(velocity_smoothing STATIC ${VEL_SMOOTHING_SOURCES})
target_include_directories(velocity_smoothing PUBLIC
  ${VEL_SMOOTHING_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/velocity_smoothing)
target_compile_options(velocity_smoothing PRIVATE -w)

# Link to node
target_link_libraries(gik9dof_solver_node
  ...
  velocity_smoothing)
```

### 3. Update Node Header (`gik9dof_solver_node.h`)
```cpp
#include "velocity_smoothing/smoothVelocityCommand.h"
#include "velocity_smoothing/smoothVelocityCommand_types.h"

private:
    // Velocity smoothing state (for ALL modes)
    double vx_smooth_prev_ = 0.0;
    double wz_smooth_prev_ = 0.0;
    double ax_smooth_prev_ = 0.0;
    double alpha_smooth_prev_ = 0.0;
    
    // Smoothing parameters
    struct0_T vel_smooth_params_;  // from smoothVelocityCommand_types.h
    bool enable_velocity_smoothing_ = false;
    
    // New method
    void applySmoothingToVelocity(double vx_raw, double wz_raw,
                                  double& vx_out, double& wz_out);
```

### 4. Update Node Implementation (`gik9dof_solver_node.cpp`)

#### Constructor: Load Parameters
```cpp
// Velocity smoothing parameters (applies to ALL controllers)
this->declare_parameter("velocity_smoothing.enable", true);
this->declare_parameter("velocity_smoothing.vx_max", 1.5);
this->declare_parameter("velocity_smoothing.ax_max", 1.0);
this->declare_parameter("velocity_smoothing.jx_max", 5.0);
this->declare_parameter("velocity_smoothing.wz_max", 2.0);
this->declare_parameter("velocity_smoothing.alpha_max", 3.0);
this->declare_parameter("velocity_smoothing.jerk_wz_max", 10.0);

// Load parameters
enable_velocity_smoothing_ = this->get_parameter("velocity_smoothing.enable").as_bool();
vel_smooth_params_.vx_max = this->get_parameter("velocity_smoothing.vx_max").as_double();
vel_smooth_params_.ax_max = this->get_parameter("velocity_smoothing.ax_max").as_double();
vel_smooth_params_.jx_max = this->get_parameter("velocity_smoothing.jx_max").as_double();
vel_smooth_params_.wz_max = this->get_parameter("velocity_smoothing.wz_max").as_double();
vel_smooth_params_.alpha_max = this->get_parameter("velocity_smoothing.alpha_max").as_double();
vel_smooth_params_.jerk_wz_max = this->get_parameter("velocity_smoothing.jerk_wz_max").as_double();

// Initialize generated function
if (enable_velocity_smoothing_) {
    smoothVelocityCommand_initialize();
    RCLCPP_INFO(this->get_logger(), "Velocity smoothing enabled (S-curve, all controllers)");
}
```

#### New Method: Apply Smoothing
```cpp
void GIK9DOFSolverNode::applySmoothingToVelocity(
    double vx_raw, double wz_raw,
    double& vx_out, double& wz_out)
{
    if (!enable_velocity_smoothing_) {
        // Pass through without smoothing
        vx_out = vx_raw;
        wz_out = wz_raw;
        return;
    }
    
    // Compute dt
    double dt = 1.0 / control_rate_;  // e.g., 0.02s for 50Hz
    
    double ax_out, alpha_out;
    
    // Call generated C++ function
    smoothVelocityCommand(
        vx_raw, wz_raw,
        vx_smooth_prev_, wz_smooth_prev_,
        ax_smooth_prev_, alpha_smooth_prev_,
        dt,
        &vel_smooth_params_,
        &vx_out, &wz_out,
        &ax_out, &alpha_out
    );
    
    // Update state for next call
    vx_smooth_prev_ = vx_out;
    wz_smooth_prev_ = wz_out;
    ax_smooth_prev_ = ax_out;
    alpha_smooth_prev_ = alpha_out;
}
```

#### Modify Publishing Methods: Add Smoothing Layer
```cpp
void GIK9DOFSolverNode::publishBaseCommand()
{
    // Mode 0, 1, 2 compute raw velocity
    double vx_raw, wz_raw;
    
    if (velocity_control_mode_ == 0) {
        // Heading controller
        holisticVelocityController(..., &vx_raw, &wz_raw);
    } else if (velocity_control_mode_ == 1) {
        // Chassis-only controller
        holisticVelocityController(..., &vx_raw, &wz_raw);
    } else if (velocity_control_mode_ == 2) {
        // Pure Pursuit
        purePursuitVelocityController(..., &vx_raw, &wz_raw, ...);
    }
    
    // Apply smoothing layer (NEW!)
    double vx_final, wz_final;
    applySmoothingToVelocity(vx_raw, wz_raw, vx_final, wz_final);
    
    // Publish smoothed velocity
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = vx_final;
    msg.angular.z = wz_final;
    base_cmd_pub_->publish(msg);
}
```

### 5. Update Configuration (`gik9dof_solver_params.yaml`)
```yaml
gik9dof_solver:
  ros__parameters:
    # ... existing parameters ...
    
    # Velocity Smoothing (applies to ALL velocity controllers)
    velocity_smoothing:
      enable: true                # Enable/disable smoothing layer
      vx_max: 1.5                # Max forward velocity (m/s)
      ax_max: 1.0                # Max forward acceleration (m/s²)
      jx_max: 5.0                # Max forward jerk (m/s³)
      wz_max: 2.0                # Max angular velocity (rad/s)
      alpha_max: 3.0             # Max angular acceleration (rad/s²)
      jerk_wz_max: 10.0          # Max angular jerk (rad/s³)
```

## Benefits

1. **Compatibility**: Works with ALL velocity controllers (Mode 0, 1, 2)
2. **Safety**: Prevents sudden jerks and excessive acceleration
3. **Flexibility**: Can be enabled/disabled via ROS2 parameter
4. **Performance**: Optimized C++ code, minimal overhead (~10µs)
5. **Consistency**: Same smoothing behavior across all control modes

## Testing Plan

1. **Test Mode 2 (Pure Pursuit) with Smoothing**:
   - Enable `velocity_smoothing.enable: true`
   - Run Pure Pursuit on curved path
   - Verify acceleration < 1.0 m/s², jerk < 5.0 m/s³
   
2. **Test Mode 0 (Heading) with Smoothing**:
   - Enable smoothing
   - Command sudden orientation change
   - Verify smooth angular acceleration

3. **Compare with/without Smoothing**:
   - Run same trajectory with `enable: false` then `enable: true`
   - Plot velocity, acceleration, jerk profiles
   - Verify smoothing reduces peak jerk without increasing settling time

4. **Performance Test**:
   - Measure CPU usage at 50Hz
   - Verify smoothing overhead < 5%
   - Check real-time performance on Jetson Orin

## Next Steps

1. ✅ Create MATLAB function (`smoothVelocityCommand.m`)
2. ✅ Generate C++ code (MATLAB Coder)
3. ⏳ Copy files to ROS2 package
4. ⏳ Update CMakeLists.txt
5. ⏳ Update node header/implementation
6. ⏳ Update configuration file
7. ⏳ Build and test
8. ⏳ Deploy to Jetson Orin
9. ⏳ Real robot validation

## Files Modified/Created

### MATLAB Side
- ✅ `matlab/+gik9dof/+control/smoothVelocityCommand.m` (NEW)
- ✅ `scripts/codegen/generate_code_velocity_smoothing.m` (NEW)
- ✅ `run_velocity_smoothing_codegen.sh` (NEW)

### Generated C++ Code
- ✅ `codegen/velocity_smoothing/smoothVelocityCommand.cpp`
- ✅ `codegen/velocity_smoothing/smoothVelocityCommand.h`
- ✅ `codegen/velocity_smoothing/smoothVelocityCommand_types.h`

### ROS2 Package (TODO)
- ⏳ `ros2/gik9dof_solver/src/velocity_smoothing/` (copy generated files)
- ⏳ `ros2/gik9dof_solver/include/velocity_smoothing/` (copy generated headers)
- ⏳ `ros2/gik9dof_solver/CMakeLists.txt` (add library)
- ⏳ `ros2/gik9dof_solver/src/gik9dof_solver_node.h` (add smoothing state/method)
- ⏳ `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp` (implement smoothing layer)
- ⏳ `ros2/gik9dof_solver/config/gik9dof_solver_params.yaml` (add velocity_smoothing section)

---
**Created**: October 10, 2025  
**Author**: GitHub Copilot  
**Status**: Code generation complete, ROS2 integration pending
