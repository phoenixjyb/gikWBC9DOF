# AGX Orin MATLAB Solver Integration Guide

**Date**: October 6, 2025  
**Status**: Build successful on Orin, ready for MATLAB integration

## Current State

✅ **Completed:**
- ROS2 packages built successfully on Orin
  - `gik9dof_msgs`: 9.07s
  - `gik9dof_solver`: 38.6s
- MATLAB-generated code copied to Orin: `~/gikWBC9DOF/gik_codegen_20251006_170613.zip`

🔄 **Next Step:** Integrate MATLAB solver code into ROS2 node

---

## Integration Steps (Run on Orin)

### 1. Extract MATLAB-Generated Code

```bash
cd ~/gikWBC9DOF
unzip gik_codegen_20251006_170613.zip -d matlab_solver
```

This creates:
```
~/gikWBC9DOF/matlab_solver/
├── include/
│   └── gik9dof/
│       ├── GIKSolver.h
│       ├── solveGIKStepRealtime.h
│       ├── RigidBodyTree.h
│       └── ... (61 headers total)
└── lib/
    └── libgik9dof_solver.a  (or .so)
```

### 2. Update CMakeLists.txt

Edit `~/gikWBC9DOF/ros2/gik9dof_solver/CMakeLists.txt`:

**Add after `find_package(gik9dof_msgs REQUIRED)`:**
```cmake
# MATLAB-generated solver library
set(MATLAB_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../matlab_solver")
include_directories(${MATLAB_SOLVER_DIR}/include)

# Find the MATLAB solver library
find_library(GIK_SOLVER_LIB
  NAMES gik9dof_solver libgik9dof_solver
  PATHS ${MATLAB_SOLVER_DIR}/lib
  NO_DEFAULT_PATH
)

if(NOT GIK_SOLVER_LIB)
  message(FATAL_ERROR "MATLAB solver library not found in ${MATLAB_SOLVER_DIR}/lib")
endif()

message(STATUS "Found MATLAB solver library: ${GIK_SOLVER_LIB}")
```

**Update `target_link_libraries`:**
```cmake
target_link_libraries(gik9dof_solver_node
  ${GIK_SOLVER_LIB}  # Add this line
)
```

### 3. Update Solver Node Code

Edit `~/gikWBC9DOF/ros2/gik9dof_solver/src/gik9dof_solver_node.cpp`:

**Uncomment line 25:**
```cpp
#include "gik9dof/GIKSolver.h"  // Uncomment this
```

**Add persistent solver object as class member** in the node's private section:

```cpp
// In GIK9DOFSolverNode class (around line 140):
private:
  // ... existing members ...
  
  // MATLAB solver instance (persistent)
  std::unique_ptr<gik9dof::GIKSolver> matlab_solver_;
```

**Initialize solver in constructor** (after `rclcpp::Node("gik9dof_solver")`):

```cpp
GIK9DOFSolverNode() : Node("gik9dof_solver") {
    // ... existing initialization ...
    
    // Create MATLAB solver instance
    matlab_solver_ = std::make_unique<gik9dof::GIKSolver>();
    RCLCPP_INFO(this->get_logger(), "MATLAB solver initialized");
}
```

**Replace the TODO block (lines ~196-210) with actual solver calls:**

```cpp
// Current state from odometry (base: x, y, theta)
double q_current[9] = {
    current_odom_.pose.pose.position.x,
    current_odom_.pose.pose.position.y,
    tf2::getYaw(current_odom_.pose.pose.orientation),
    current_arm_state_.position[0],  // Joint 1
    current_arm_state_.position[1],  // Joint 2
    current_arm_state_.position[2],  // Joint 3
    current_arm_state_.position[3],  // Joint 4
    current_arm_state_.position[4],  // Joint 5
    current_arm_state_.position[5]   // Joint 6
};

// Next joint configuration (output)
double q_next[9];

// Target pose as 4x4 homogeneous transformation matrix (column-major)
double target_matrix[16];
tf2::Transform target_tf;
tf2::fromMsg(waypoint.pose, target_tf);

// Convert to column-major 4x4 matrix for MATLAB
tf2::Matrix3x3 rotation = target_tf.getBasis();
tf2::Vector3 translation = target_tf.getOrigin();

// Column 0 (rotation matrix column 0 + translation.x)
target_matrix[0] = rotation[0][0];
target_matrix[1] = rotation[1][0];
target_matrix[2] = rotation[2][0];
target_matrix[3] = 0.0;

// Column 1 (rotation matrix column 1 + translation.y)
target_matrix[4] = rotation[0][1];
target_matrix[5] = rotation[1][1];
target_matrix[6] = rotation[2][1];
target_matrix[7] = 0.0;

// Column 2 (rotation matrix column 2 + translation.z)
target_matrix[8] = rotation[0][2];
target_matrix[9] = rotation[1][2];
target_matrix[10] = rotation[2][2];
target_matrix[11] = 0.0;

// Column 3 (translation)
target_matrix[12] = translation.x();
target_matrix[13] = translation.y();
target_matrix[14] = translation.z();
target_matrix[15] = 1.0;

// Solver parameters (from MATLAB validation)
double distance_lower = 0.5;   // meters (from validation)
double distance_weight = 1.0;  // unitless weight

// Solver output structure
gik9dof::struct0_T solver_info;

// Call MATLAB-generated solver
auto start_time = this->now();
matlab_solver_->gik9dof_codegen_realtime_solveGIKStepWrapper(
    q_current,
    target_matrix,
    distance_lower,
    distance_weight,
    q_next,
    &solver_info
);
auto end_time = this->now();

double solve_time_ms = (end_time - start_time).seconds() * 1000.0;

// Extract status string from solver_info
std::string status_str(solver_info.Status.data, 
                       solver_info.Status.size[1]);

// Update diagnostics
diag.status = status_str;
diag.solve_time_ms = solve_time_ms;
diag.iterations_used = static_cast<int32_t>(solver_info.Iterations);
diag.exit_flag = static_cast<int32_t>(solver_info.ExitFlag);

// Publish next configuration (if successful)
if (status_str == "success") {
    // q_next[0:2] = base (x, y, theta)
    // q_next[3:8] = arm joints
    
    // TODO: Publish base velocity commands (convert position to velocity)
    // TODO: Publish arm joint trajectory
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Solver success: solve_time=%.2fms, iterations=%d, exit_flag=%d", 
                 solve_time_ms, diag.iterations_used, diag.exit_flag);
} else {
    RCLCPP_WARN(this->get_logger(), 
                "Solver failed: status='%s', solve_time=%.2fms, iterations=%d, exit_flag=%d", 
                status_str.c_str(), solve_time_ms, 
                diag.iterations_used, diag.exit_flag);
}
```

### 4. Rebuild

```bash
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select gik9dof_solver
```

**Expected output:**
- Build time: ~40-50s (similar to previous build)
- No errors
- Solver library linked successfully

### 5. Test Node

```bash
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

**Monitor for:**
- Node starts without crashes
- No library loading errors
- Ready to receive trajectory messages

---

## Verification Checklist

After integration:

- [ ] Build completes with 0 errors
- [ ] Node starts without library errors
- [ ] Solver receives trajectory messages
- [ ] Diagnostics show `status="success"`
- [ ] `solve_time_ms < 50` (target performance)
- [ ] `iterations_used < 100`

---

## Troubleshooting

### Library Not Found

If you see: `error while loading shared libraries: libgik9dof_solver.so`

**Solution:** Add library path to `LD_LIBRARY_PATH`:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/gikWBC9DOF/matlab_solver/lib
```

Add to `~/.bashrc` for persistence.

### Function Signature Mismatch

If you get compilation errors about function signatures:

**Check actual signature** in `~/gikWBC9DOF/matlab_solver/include/gik9dof/solveGIKStepRealtime.h`:
```bash
cat ~/gikWBC9DOF/matlab_solver/include/gik9dof/solveGIKStepRealtime.h | grep -A 5 "solveGIKStepRealtime"
```

Adjust the function call in the node accordingly.

### Solver Returns Failure

If `solver_status != 0`:

1. Check joint bounds (robot URDF limits)
2. Verify target pose is reachable
3. Check distance constraints (max 100m)
4. Increase solver iterations (if configurable)

---

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Solve time | < 50 ms | For 10 Hz control |
| Iterations | < 100 | Typical for reachable poses |
| Success rate | > 95% | For valid trajectories |

---

## Next Steps After Integration

1. **Launch with test trajectory:**
   ```bash
   ros2 launch gik9dof_solver test_solver.launch.py
   ```

2. **Connect to real robot topics:**
   ```bash
   ros2 run gik9dof_solver gik9dof_solver_node
   ```
   
   Ensure topics are mapped:
   - `/hdas/feedback_arm_left` → `arm_joint_states`
   - `/odom_wheel` → `odom`
   - `/gik9dof/target_trajectory` → `target_trajectory`

3. **Monitor diagnostics:**
   ```bash
   ros2 topic echo /gik9dof/diagnostics
   ```

---

**Ready to integrate? Let's do this! 🚀**
