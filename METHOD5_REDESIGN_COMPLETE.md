# Method 5 Redesign Complete: Base-Only → Whole-Body MPC

**Date:** October 13, 2025  
**Status:** ✅ Complete - Ready for Testing  
**Branch:** mpc-dev-stageC

---

## Summary

Successfully redesigned Method 5 (pureMPC) from **incorrect base-only MPC with separate IK** to **correct whole-body MPC with unified optimization**.

### Critical Architectural Change

**❌ Phase 1 (Incorrect):**
```
Base MPC: [v, ω] → Simulate base → Solve arm IK (decoupled)
Problem: Sequential optimization loses unified constraint satisfaction
```

**✅ Phase 2 (Correct):**
```
Whole-body MPC: [v, ω, q̇_arm] → Unified optimization with FK in cost
Benefit: Differential IK embedded in MPC, all constraints satisfied simultaneously
```

### Expert Validation

User provided expert reference:
> "No—you don't need a standalone IK step for the MPC approach. The MPC already decides the base velocities (v,ω) and joint velocities q̇ by minimizing end‑effector pose error using forward kinematics + Jacobians. That gives you 'IK‑like' behavior but inside the constrained optimizer."

---

## What Changed

### 1. State Space Expansion
- **Before:** 3 states [x, y, θ] - base only
- **After:** 9 states [x, y, θ, q_arm(6)] - full configuration

### 2. Control Space Expansion
- **Before:** 2 inputs [v, ω] - base velocities only
- **After:** 8 inputs [v, ω, q̇_arm(6)] - base + arm velocities

### 3. Cost Function Redesign
- **Before:** Tracking base position/heading
- **After:** Custom FK-based end-effector tracking
  - Position error via `getTransform(robot, q_full, eeBodyName)`
  - Orientation error via Frobenius norm `||R_ee - R_ref||_F`
  - No separate IK step

### 4. NMPC Architecture
- **Before:** `nlmpc(3, 3, 2)` - base tracking
- **After:** `nlmpc(9, 12, 8)` - whole-body with EE pose output

### 5. Reference Trajectory Format
- **Before:** [3 × (p+1)] base path [x, y, θ]
- **After:** [12 × (p+1)] EE poses [p_ee(3); R_ee_vec(9)]

---

## Files Modified

### ✅ Completed

1. **`matlab/+gik9dof/+mpc/unicycleStateFcn.m`**
   - Expanded from 3 to 9 states
   - Added arm joint velocity integration: `q_arm + Ts*q̇_arm`

2. **`matlab/+gik9dof/+mpc/unicycleStateJacobian.m`**
   - Expanded Jacobians: (3×3, 3×2) → (9×9, 9×8)
   - State Jacobian A: 9×9 identity with coupling terms
   - Input Jacobian B: 9×8 with base and arm blocks

3. **`matlab/+gik9dof/+mpc/eeTrackingCostFcn.m`** *(NEW)*
   - Custom cost function with forward kinematics
   - Position error: `||p_ee - p_ref||²`
   - Orientation error: `||R_ee - R_ref||²_F`
   - Input effort: `||u||²`
   - Rate smoothness: `||Δu||²`
   - Terminal cost: 5× weight on final error

4. **`matlab/+gik9dof/+mpc/wheelSpeedConstraints.m`**
   - Updated to handle 8 inputs (extracts only first 2 for wheel constraints)
   - Constraints: `|v ± ω·W/2| ≤ v_wheel_max`

5. **`matlab/+gik9dof/runStageCPureMPC.m`** *(MAJOR REDESIGN)*
   - Changed to `nlmpc(9, 12, 8)` - whole-body system
   - Custom cost function: `eeTrackingCostFcn` with FK
   - **Removed:** All `solveArmIKForBase` calls (no longer needed)
   - **Removed:** `ArmIKParams` option (no longer needed)
   - **Added:** Joint velocity and limit constraints
   - **Added:** Orientation error tracking
   - Reference format: [12×(p+1)] EE poses instead of base path

---

## Files No Longer Used

### ⚠️ Deprecated

- **`matlab/+gik9dof/+mpc/solveArmIKForBase.m`**
  - Purpose: Solve arm IK with fixed base
  - Status: No longer needed - IK behavior embedded in MPC
  - Action: Keep for historical reference, but not used in Method 5

- **`matlab/+gik9dof/+mpc/configureNMPCWeights.m`**
  - Purpose: Configure tracking weights
  - Status: Replaced by custom cost function
  - Action: Not used in Method 5 (still used by other methods if needed)

---

## Configuration Updates Needed

### `config/pipeline_profiles.yaml`

The `pureMPC` profile needs to be updated with:

```yaml
pureMPC:
  nmpc:
    # ... existing base constraints ...
    
    # ADD: Arm joint velocity constraints
    q_dot_arm_min: -0.5   # rad/s
    q_dot_arm_max: 0.5    # rad/s
    
    # ADD: Arm joint limit constraints (robot-specific)
    q_arm_min: [-2.967, -2.094, -2.967, -2.094, -2.967, -2.094]  # Example
    q_arm_max: [2.967, 2.094, 2.967, 2.094, 2.967, 2.094]
    
  weights:
    # ADD: Arm tracking weights
    arm_velocity: 0.01  # Penalty on arm joint velocities
    arm_rate: 0.1       # Penalty on arm acceleration
```

---

## Technical Details

### NMPC Controller Structure

```matlab
nlobj = nlmpc(9, 12, 8);

% States: [x, y, θ, q1, q2, q3, q4, q5, q6]
% Outputs: [px, py, pz, R11, R12, ..., R33]  (EE pose)
% Inputs: [v, ω, q̇1, q̇2, q̇3, q̇4, q̇5, q̇6]

% State dynamics
nlobj.Model.StateFcn = 'gik9dof.mpc.unicycleStateFcn';
nlobj.Jacobian.StateFcn = 'gik9dof.mpc.unicycleStateJacobian';

% Custom cost function (FK-based EE tracking)
nlobj.Optimization.CustomCostFcn = @(...) eeTrackingCostFcn(...);

% Constraints
nlobj.MV(1:2) = [v_min, v_max; ω_min, ω_max];       % Base
nlobj.MV(3:8) = [q̇_min, q̇_max; ...];                % Arm
nlobj.States(4:9) = [q_min, q_max; ...];            % Joint limits
nlobj.Optimization.CustomIneqConFcn = @(...) wheelSpeedConstraints(...);
```

### Cost Function Computation

At each MPC step, for each point in horizon:
```matlab
% 1. Extract state: q_full = [x, y, θ, q_arm]
% 2. Compute FK: T_ee = getTransform(robot, q_full, eeBodyName)
% 3. Extract pose: p_ee = T_ee(1:3,4), R_ee = T_ee(1:3,1:3)
% 4. Compute errors:
%    e_pos = ||p_ee - p_ref||²
%    e_ori = ||R_ee - R_ref||²_F
% 5. Compute cost:
%    J = w_pos*e_pos + w_ori*e_ori + w_input*||u||² + w_rate*||Δu||²
```

This **replaces** the separate IK step. The optimizer finds `[v, ω, q̇_arm]` that minimize EE error.

---

## Why This Is Correct

### ❌ Problem with Base-Only + IK

1. **Decoupled optimization:** Base MPC ignores arm kinematics
2. **Suboptimal:** IK solves for arm given fixed base (sequential)
3. **Constraint violation:** Can't guarantee joint limits during base motion
4. **Not true MPC:** IK is open-loop, not part of receding horizon

### ✅ Advantages of Whole-Body MPC

1. **Unified optimization:** Single optimizer handles all DOF
2. **Optimal:** All DOF optimized simultaneously for same objective
3. **Constraint satisfaction:** All constraints (base, arm, wheel speeds) checked together
4. **True receding horizon:** All DOF planned over full prediction horizon
5. **Differential IK embedded:** Optimizer naturally finds `q̇` minimizing EE error

---

## Next Steps

### 1. Configuration Update (REQUIRED)
- Add `q_dot_arm_min/max` to `pureMPC` profile
- Add `q_arm_min/max` (robot-specific joint limits)
- Tune weights: `arm_velocity`, `arm_rate`

### 2. Testing
Create `test_method5_wholebodyMPC.m`:
```matlab
% Test whole-body MPC on simple trajectory
robot = gik9dof.createRobotModel();
traj = gik9dof.loadJsonTrajectory('1_pull_world_scaled.json');
q0 = [0; 0; 0; zeros(6,1)];

configTools = gik9dof.ConfigTools(...);
nmpcParams = configTools.getStageConfig('c', 'nmpc');

log = gik9dof.runStageCPureMPC(robot, traj, q0, ...
    'NMPCParams', nmpcParams, 'VerboseLevel', 2);

% Verify:
% - MPC converges (exitFlag > 0)
% - EE tracking error acceptable
% - No IK step involved
% - All DOF controlled
```

### 3. Validation
- Compare with Method 4 (PP-first hybrid IK/QP)
- Check constraint satisfaction: `all(log.mpcExitFlag > 0)`
- Verify EE tracking: `mean(log.positionErrorNorm) < 5mm`
- Confirm smooth trajectories: `diff(log.qTraj, 1, 2)` bounded

### 4. Documentation Update
- ✅ `METHOD5_IMPLEMENTATION_PLAN.md` - Update to reflect whole-body
- ✅ `METHOD5_IMPLEMENTATION_STATUS.md` - Mark redesign complete
- ✅ `METHOD5_MATLAB_MPC_APPROACH.md` - Add FK-based cost explanation

---

## Key Takeaways

1. **No IK in MPC loop** - Differential IK is embedded via FK in cost function
2. **Control all DOF simultaneously** - [v, ω, q̇_arm] optimized together
3. **Use FK, not IK** - `getTransform` in cost function, not `inverseKinematics`
4. **Unified constraints** - Base, arm, wheels all checked in single optimization
5. **True receding horizon** - All 9 DOF planned over full prediction horizon

---

## References

**User-provided expert explanation:**
> "The MPC already decides the base velocities (v,ω) and joint velocities q̇ by minimizing end‑effector pose error using forward kinematics + Jacobians. That gives you 'IK‑like' behavior but inside the constrained optimizer."

**MATLAB Documentation:**
- `nlmpc` - Nonlinear Model Predictive Control
- Custom cost functions: `nlobj.Optimization.CustomCostFcn`
- State functions: `nlobj.Model.StateFcn`

---

## Status

- ✅ **Architecture redesigned** - Whole-body MPC with unified optimization
- ✅ **State functions updated** - 9 states, 8 inputs
- ✅ **Cost function created** - FK-based EE tracking
- ✅ **Main controller redesigned** - No IK, uses custom cost
- ✅ **Constraints updated** - Wheel speeds handle 8 inputs
- ⏳ **Configuration pending** - Need to add arm limits/weights
- ⏳ **Testing pending** - Need to validate on test trajectory
- ⏳ **Documentation pending** - Need to update all METHOD5_*.md files

**Ready for configuration update and testing!** 🚀
