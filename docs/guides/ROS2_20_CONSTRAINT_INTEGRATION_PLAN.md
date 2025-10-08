# ROS2 Integration Plan - 20-Constraint GIK Solver

## New C++ Interface (from generated code)

```cpp
namespace gik9dof {
namespace codegen_inuse {
extern void solveGIKStepWrapper(
    const double qCurrent[9],              // Current joint config
    const double targetPose[16],           // 4x4 homogeneous transform (column-major)
    const int distBodyIndices[20],         // Body indices (gripper, link5, etc.)
    const int distRefBodyIndices[20],      // Reference body indices (chassis, base, etc.)
    const double distBoundsLower[20],      // Lower bounds for each constraint
    const double distBoundsUpper[20],      // Upper bounds for each constraint  
    const double distWeights[20],          // Weights (0.0=disabled, >0.0=enabled)
    double qNext[9],                       // OUTPUT: Next joint configuration
    struct0_T *solverInfo);                // OUTPUT: Solver status/diagnostics
}
}
```

## Body Index Mapping

From MATLAB code generation, the robot bodies are indexed:
```
0  = base
1  = chassis  
2  = arm_base_link
3  = link1
4  = link2
5  = link3
6  = link4
7  = link5
8  = end_effector
9  = gripper
```

## Default Constraint Configuration

Based on test_gik_20constraints.m:

| Index | Body1 (distBodyIndices) | Body2 (distRefBodyIndices) | Bounds [m] | Weight | Purpose |
|-------|-------------------------|----------------------------|------------|--------|---------|
| 0     | gripper (9)            | chassis (1)                | [0.05, inf]| 1.0    | Keep gripper away from chassis |
| 1     | gripper (9)            | base (0)                   | [0.05, inf]| 1.0    | Keep gripper away from base |
| 2     | gripper (9)            | arm_base_link (2)          | [0.05, inf]| 1.0    | Keep gripper away from arm base |
| 3     | link5 (7)              | chassis (1)                | [0.05, inf]| 1.0    | Keep link5 away from chassis |
| 4     | link4 (6)              | chassis (1)                | [0.05, inf]| 1.0    | Keep link4 away from chassis |
| 5-19  | gripper (9)            | base (0)                   | [0.05, inf]| 0.0    | **DISABLED** (weight=0) |

**Note**: Indices 5-19 are placeholders, disabled by default (weight=0.0)

## ROS2 Parameter Design

### YAML Configuration (`config/gik_solver_params.yaml`)

```yaml
gik9dof_solver_node:
  ros__parameters:
    # Distance constraints (20 total)
    distance_constraints:
      # Constraint 0: gripper -> chassis
      - body1_index: 9        # gripper
        body2_index: 1        # chassis
        lower_bound: 0.05     # minimum 5cm
        upper_bound: 10.0     # effectively infinite
        weight: 1.0           # enabled
        
      # Constraint 1: gripper -> base  
      - body1_index: 9
        body2_index: 0
        lower_bound: 0.05
        upper_bound: 10.0
        weight: 1.0
        
      # Constraint 2: gripper -> arm_base_link
      - body1_index: 9
        body2_index: 2
        lower_bound: 0.05
        upper_bound: 10.0
        weight: 1.0
        
      # Constraint 3: link5 -> chassis
      - body1_index: 7
        body2_index: 1
        lower_bound: 0.05
        upper_bound: 10.0
        weight: 1.0
        
      # Constraint 4: link4 -> chassis
      - body1_index: 6
        body2_index: 1
        lower_bound: 0.05
        upper_bound: 10.0
        weight: 1.0
        
      # Constraints 5-19: Disabled placeholders
      - body1_index: 9
        body2_index: 0
        lower_bound: 0.05
        upper_bound: 10.0
        weight: 0.0  # DISABLED
      # ... repeat for indices 6-19
```

### Alternative: Flat Parameter Arrays

```yaml
gik9dof_solver_node:
  ros__parameters:
    dist_body_indices:     [9, 9, 9, 7, 6, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9]
    dist_ref_body_indices: [1, 0, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    dist_lower_bounds:     [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    dist_upper_bounds:     [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    dist_weights:          [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**Recommendation**: Use flat arrays for simplicity and performance (no parsing overhead)

## Code Changes Required

### 1. Update Member Variables (gik9dof_solver_node.h)

```cpp
// OLD (4 constraints)
double distance_lower_[4];
double distance_weight_[4];

// NEW (20 constraints)
int dist_body_indices_[20];
int dist_ref_body_indices_[20];
double dist_lower_bounds_[20];
double dist_upper_bounds_[20];
double dist_weights_[20];
```

### 2. Update Parameter Declaration

```cpp
// Declare ROS parameters
this->declare_parameter("dist_body_indices", std::vector<int64_t>(20, 9));
this->declare_parameter("dist_ref_body_indices", std::vector<int64_t>(20, 0));
this->declare_parameter("dist_lower_bounds", std::vector<double>(20, 0.05));
this->declare_parameter("dist_upper_bounds", std::vector<double>(20, 10.0));
this->declare_parameter("dist_weights", std::vector<double>(20, 0.0));
```

### 3. Update Parameter Reading

```cpp
auto body_indices = this->get_parameter("dist_body_indices").as_integer_array();
auto ref_body_indices = this->get_parameter("dist_ref_body_indices").as_integer_array();
auto lower_bounds = this->get_parameter("dist_lower_bounds").as_double_array();
auto upper_bounds = this->get_parameter("dist_upper_bounds").as_double_array();
auto weights = this->get_parameter("dist_weights").as_double_array();

// Copy to member arrays
for (size_t i = 0; i < 20; i++) {
    dist_body_indices_[i] = static_cast<int>(body_indices[i]);
    dist_ref_body_indices_[i] = static_cast<int>(ref_body_indices[i]);
    dist_lower_bounds_[i] = lower_bounds[i];
    dist_upper_bounds_[i] = upper_bounds[i];
    dist_weights_[i] = weights[i];
}
```

### 4. Update Solver Call

```cpp
// OLD
matlab_solver_->gik9dof_codegen_realtime_solveGIKStepWrapper(
    q_current,
    target_matrix,
    distance_lower_,
    distance_weight_,
    q_next,
    &solver_info
);

// NEW
matlab_solver_->solveGIKStepWrapper(  // Note: new namespace!
    q_current,
    target_matrix,
    dist_body_indices_,
    dist_ref_body_indices_,
    dist_lower_bounds_,
    dist_upper_bounds_,
    dist_weights_,
    q_next,
    &solver_info
);
```

### 5. Update Include

```cpp
// OLD
#include "gik9dof_codegen_realtime_solveGIKStepWrapper.h"

// NEW  
#include "gik9dof_codegen_inuse_solveGIKStepWrapper.h"
```

## Testing Strategy

### Phase 1: Default Configuration (Constraints 0-4 enabled)
```bash
cd ros2
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

Expected: 5 active constraints, ~15ms solve time

### Phase 2: All Disabled (baseline)
Modify config: set all weights to 0.0
Expected: No distance constraints, faster solve

### Phase 3: Selective Enable
Enable constraints 1,2,4 (matching MATLAB Test 3)
Expected: Same behavior as C++ standalone test

### Phase 4: Dynamic Reconfigure (optional)
Add parameter callback to update constraints at runtime
- Useful for debugging
- Can enable/disable specific constraints on-the-fly

## Validation Checklist

- [ ] Code compiles without errors
- [ ] All 20 constraint parameters loaded from YAML
- [ ] Solver call uses correct 7-parameter interface
- [ ] Default config matches MATLAB test behavior
- [ ] Solve time ~15ms (within expected range)
- [ ] Robot respects distance bounds
- [ ] Disabled constraints (weight=0) have no effect
- [ ] ROS2 diagnostics show constraint violations correctly
- [ ] No crashes or memory issues

## Backward Compatibility

**BREAKING CHANGE**: This is a major API update. Old parameter files will not work.

Migration path:
1. Create new parameter file with 20-constraint structure
2. Map old 4 constraints to new indices 0-3
3. Set indices 4-19 weights to 0.0 (disabled)

## Performance Expectations

- Default (5 active constraints): ~15-20ms
- All disabled: ~10-15ms  
- All enabled (20 constraints): ~30-40ms (still within 50ms timeout)

## Next Steps

1. ✅ Copy generated code to ROS2 package
2. ✅ Verify CMakeLists.txt configuration
3. 🔄 Update gik9dof_solver_node.cpp interface
4. 🔄 Create parameter YAML file
5. 🔄 Build in WSL
6. 🔄 Test locally
7. 🔄 Deploy to Orin

---
**Status**: Ready to implement Step 3 (Update solver node)
**Estimated Time**: 30-45 minutes for code + config + testing
