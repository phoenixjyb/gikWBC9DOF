# GIK9DOF Configuration Files Summary

**Date**: October 9, 2025  
**Commit**: 4033838

## Configuration Files Overview

The ROS2 GIK9DOF solver package maintains **three YAML configuration files**, each serving different use cases:

### 1. `gik9dof_solver_params.yaml` (9,486 bytes)
**Purpose**: **Primary configuration for holistic mode with comprehensive tuning documentation**

**Key Features**:
- ✅ **Best documentation** - Extensive tuning guide (100+ lines)
- ✅ Detailed Pure Pursuit parameter explanations
- ✅ Trade-off discussions and validation procedures
- ✅ Recommended for production holistic mode deployment
- ❌ No 20-constraint arrays (uses old single parameters)
- ❌ No staged control parameters

**Use Cases**:
- Production deployment on Jetson Orin (holistic mode)
- Pure Pursuit controller tuning
- Learning how to tune velocity controller parameters
- Default configuration for most users

**Max Iterations**: 1000 (matches MATLAB wrapper)

---

### 2. `gik_solver_params.yaml` (6,267 bytes)
**Purpose**: **20-constraint configuration with full YAML parameter arrays**

**Key Features**:
- ✅ **Only file with 20-constraint arrays in YAML**
- ✅ Detailed body index documentation (0-9 mapping)
- ✅ Performance expectations for different constraint counts
- ✅ Constraint configuration examples (Example A/B/C)
- ✅ Basic staged control parameters
- ✅ Bidirectional Pure Pursuit support (`vx_min: -1.0`)

**Constraint Configuration**:
```yaml
dist_body_indices: [9, 9, 9, 7, 6, ...]        # 20 elements
dist_ref_body_indices: [1, 0, 2, 1, 1, ...]    # 20 elements
dist_lower_bounds: [0.05, 0.05, ...]           # 20 elements (5cm minimum)
dist_upper_bounds: [10.0, 10.0, ...]           # 20 elements (10m maximum)
dist_weights: [1.0, 1.0, 1.0, 1.0, 1.0, 0.0...]  # 5 active, 15 disabled
```

**Use Cases**:
- Testing/tuning 20 distance constraints
- Modifying constraint weights via YAML (without recompiling)
- Performance testing with different constraint counts
- Research and development

**Max Iterations**: 1000 (matches MATLAB wrapper)

---

### 3. `gik9dof_solver.yaml` (5,525 bytes) ⭐ **KEPT PER USER REQUEST**
**Purpose**: **Staged control mode with detailed Hybrid A* planner parameters**

**Key Features**:
- ✅ **Most detailed staged control configuration**
- ✅ Stage A parameters (arm ramp-up)
- ✅ Stage B parameters (chassis planning with Hybrid A*)
- ✅ Stage C parameters (full-body tracking)
- ✅ Grid resolution, planning timeouts, goal tolerances
- ✅ GIK 3-DOF parameters for Stage B submode B2
- ✅ Default control mode: `"staged"` (vs "holistic" in others)
- ❌ No 20-constraint arrays
- ❌ Minimal documentation

**Staged Control Configuration**:
```yaml
control_mode: "staged"  # Different from other files!

staged:
  stage_a:  # Arm ramp-up
    enabled: true
    max_arm_velocity: 0.5
    position_tolerance: 0.01
    timeout: 10.0
  
  stage_b:  # Chassis planning
    submode: "pure_hybrid_astar"  # or "gik_assisted"
    grid_resolution: 0.1
    max_planning_time: 0.05
    robot_radius: 0.5
    replan_threshold: 0.5
    xy_tolerance: 0.15
    theta_tolerance: 0.175
    
    gik_3dof:  # For B2 submode
      distance_lower_bound: 0.1
      distance_weight: 1.0
  
  stage_c:  # Full-body tracking
    enabled: true
    max_tracking_error: 0.1
    timeout: 60.0
```

**Use Cases**:
- **Staged control mode testing** (Stage A → B → C)
- **Hybrid A* planner integration**
- Testing GIK-assisted base planning (Stage B submode B2)
- Research on multi-stage mobile manipulation

**Max Iterations**: 1000 (matches MATLAB wrapper)

---

## Common Parameters (All 3 Files)

### Solver Limits
```yaml
control_rate: 10.0               # 10 Hz control loop
max_solve_time: 0.05             # 50ms timeout (real-time constraint)
max_solver_iterations: 1000      # Matches MATLAB wrapper hardcoded value
```

⚠️ **Important**: `max_solver_iterations` is declared in ROS2 for consistency, but the actual solver uses the **hardcoded value in MATLAB** (`solveGIKStepWrapper.m`):
```matlab
solver.SolverParameters.MaxIterations = 1000;
```

### Pure Pursuit Controller (All 3 Files)
```yaml
velocity_control_mode: 2  # Pure Pursuit (RECOMMENDED)

purepursuit:
  lookahead_base: 0.8         # Base lookahead (m)
  lookahead_vel_gain: 0.3     # Velocity-dependent gain
  lookahead_time_gain: 0.1    # Time-dependent gain
  vx_nominal: 1.0             # Nominal speed (m/s)
  vx_max: 1.5                 # Max speed (m/s) - per specification
  wz_max: 2.0                 # Max angular velocity (rad/s)
  track: 0.674                # Wheel track width (m)
  vwheel_max: 2.0             # Max wheel speed (m/s)
```

### Simple Heading Controller (Mode 1)
```yaml
vel_ctrl:
  yaw_kp: 2.0    # Heading error P gain
  yaw_kff: 0.9   # Yaw rate feedforward gain
  # (same physical parameters as Pure Pursuit)
```

---

## File Selection Guide

| **Scenario** | **Use This File** | **Rationale** |
|-------------|------------------|---------------|
| **Production holistic mode** | `gik9dof_solver_params.yaml` | Best documentation, tuning guide |
| **Testing 20 constraints** | `gik_solver_params.yaml` | Only file with constraint arrays |
| **Staged control / Hybrid A*** | `gik9dof_solver.yaml` | Most complete staged parameters |
| **Pure Pursuit tuning** | `gik9dof_solver_params.yaml` | Comprehensive tuning instructions |
| **Research / multi-stage** | `gik9dof_solver.yaml` | Stage A/B/C detailed config |

---

## Launch File Usage

To specify which config file to use:

```bash
# Use holistic mode with best documentation
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml

# Use 20-constraint configuration
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik_solver_params.yaml

# Use staged control mode
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver.yaml
```

---

## MaxIterations Parameter Fix (Commit 4033838)

**Problem**: ROS2 parameter declared `max_solver_iterations: 50`, but MATLAB wrapper hardcodes `MaxIterations = 1000`.

**Solution**: Updated all files to declare `max_solver_iterations: 1000` for consistency.

**Files Changed**:
1. `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp` - Parameter declaration
2. `ros2/gik9dof_solver/config/gik9dof_solver_params.yaml`
3. `ros2/gik9dof_solver/config/gik_solver_params.yaml`
4. `ros2/gik9dof_solver/config/gik9dof_solver.yaml`

**Note**: This parameter is currently declared for documentation purposes but **not actively passed to the solver**. The actual `MaxIterations = 1000` is hardcoded in the MATLAB wrapper file:
- `matlab/+gik9dof/+codegen_inuse/solveGIKStepWrapper.m`

Future work could make this parameter runtime-configurable if needed.

---

## Key Differences Summary

| **Feature** | **File 1 (params)** | **File 2 (solver_params)** | **File 3 (solver.yaml)** |
|------------|---------------------|---------------------------|-------------------------|
| **Default Mode** | holistic (implicit) | holistic (explicit) | **staged** ⚠️ |
| **20-Constraint Arrays** | ❌ | ✅ | ❌ |
| **Staged Control Params** | ❌ | Basic | ✅ **Detailed** |
| **Documentation** | ✅ **Extensive** | Good | Minimal |
| **Bidirectional Support** | ❌ | ✅ `vx_min: -1.0` | ✅ `vx_min: -1.0` |
| **Tuning Guide** | ✅ **100+ lines** | Constraint-focused | ❌ |
| **Hybrid A* Params** | ❌ | Basic | ✅ **Detailed** |
| **Use Case** | Production holistic | 20-constraint testing | Staged mode research |

---

## Recommendations

### For Production Deployment (Jetson Orin)
**Use**: `gik9dof_solver_params.yaml`
- Most comprehensive documentation
- Production-ready defaults
- Easy to tune velocity controller

### For Constraint Research
**Use**: `gik_solver_params.yaml`
- Modify constraint weights via YAML
- Test different body pair combinations
- No recompilation needed for constraint changes

### For Staged Control / Hybrid A* Research
**Use**: `gik9dof_solver.yaml`
- Complete staged control pipeline
- Stage A/B/C timeout tuning
- Hybrid A* planner integration
- Multi-stage mobile manipulation

---

## Future Improvements

1. **Merge best features** into a single comprehensive config (optional)
2. **Make MaxIterations runtime-configurable** by passing ROS2 parameter to solver
3. **Add constraint arrays to File 1** while keeping excellent documentation
4. **Add staged parameters to File 1** for completeness
5. **Consolidate to 2 files**: "holistic.yaml" + "staged.yaml"

---

## Related Documentation

- `GIK_20CONSTRAINTS_CODEGEN_SUCCESS.md` - 20-constraint implementation
- `CLASS_BASED_ARCHITECTURE_FIX.md` - Round 6 deployment fix
- `PUREPURSUIT_CODEGEN_SUCCESS.md` - Pure Pursuit implementation
- `PUREPURSUIT_HYBRID_ASTAR_INTEGRATION.md` - Staged control integration

---

**Status**: ✅ All 3 config files updated with `max_solver_iterations: 1000`  
**Commit**: 4033838 - "Fix MaxIterations parameter: Update from 50 to 1000 to match MATLAB wrapper"  
**Branch**: codegencc45-main  
**Deployment Ready**: ✅ Yes - sync to Orin for testing
