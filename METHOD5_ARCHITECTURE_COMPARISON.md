# Method 5 Architecture Comparison

## Phase 1 (INCORRECT) - Base-Only MPC + Separate IK

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROL LOOP (each step)                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. Base MPC                                                    │
│     ┌──────────────────────────────────┐                       │
│     │ nlmpc(3, 3, 2)                   │                       │
│     │ States: [x, y, θ]                │                       │
│     │ Inputs: [v, ω]                   │                       │
│     │ Cost: Track base position        │                       │
│     └──────────┬───────────────────────┘                       │
│                ↓                                                │
│     Optimal: [v*, ω*]                                           │
│                ↓                                                │
│  2. Simulate Base                                               │
│     ┌──────────────────────────────────┐                       │
│     │ x_next = unicycle(x, [v*, ω*])   │                       │
│     └──────────┬───────────────────────┘                       │
│                ↓                                                │
│     Base state: [x', y', θ']                                    │
│                ↓                                                │
│  3. Solve Arm IK (DECOUPLED!)                                   │
│     ┌──────────────────────────────────┐                       │
│     │ inverseKinematics(robot, ...)    │                       │
│     │ Fixed base: [x', y', θ']         │                       │
│     │ Target: EE pose                  │                       │
│     └──────────┬───────────────────────┘                       │
│                ↓                                                │
│     Arm config: q_arm*                                          │
│                ↓                                                │
│  4. Full state: [x', y', θ', q_arm*]                            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

PROBLEM:
❌ Sequential optimization (base first, then arm)
❌ IK doesn't know about base motion constraints
❌ Base MPC doesn't consider arm reachability
❌ Not true whole-body optimization
```

## Phase 2 (CORRECT) - Whole-Body MPC

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROL LOOP (each step)                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. Whole-Body MPC                                              │
│     ┌──────────────────────────────────────────────────────┐   │
│     │ nlmpc(9, 12, 8)                                       │   │
│     │ States: [x, y, θ, q_arm(6)]                           │   │
│     │ Inputs: [v, ω, q̇_arm(6)]                              │   │
│     │                                                        │   │
│     │ Custom Cost Function:                                 │   │
│     │   For each step in horizon:                           │   │
│     │     1. FK: T_ee = getTransform(robot, q_full, ...)    │   │
│     │     2. Extract: p_ee, R_ee                            │   │
│     │     3. Compute: e_pos = ||p_ee - p_ref||²             │   │
│     │                 e_ori = ||R_ee - R_ref||²_F           │   │
│     │     4. Cost: J = w_pos*e_pos + w_ori*e_ori +          │   │
│     │                  w_input*||u||² + w_rate*||Δu||²       │   │
│     │                                                        │   │
│     │ Constraints:                                           │   │
│     │   - Base velocities: v ∈ [v_min, v_max]               │   │
│     │   - Angular velocity: ω ∈ [ω_min, ω_max]              │   │
│     │   - Wheel speeds: |v ± ω*W/2| ≤ v_wheel_max           │   │
│     │   - Arm velocities: q̇_arm ∈ [q̇_min, q̇_max]            │   │
│     │   - Joint limits: q_arm ∈ [q_min, q_max]              │   │
│     │                                                        │   │
│     │ Optimizer finds [v*, ω*, q̇_arm*] that:                │   │
│     │   - Minimizes EE tracking error (via FK)              │   │
│     │   - Satisfies ALL constraints simultaneously          │   │
│     │   - Plans over full prediction horizon                │   │
│     └──────────┬───────────────────────────────────────────┘   │
│                ↓                                                │
│     Optimal: [v*, ω*, q̇_arm*]  (8 control variables)           │
│                ↓                                                │
│  2. Simulate Full-Body Motion                                   │
│     ┌──────────────────────────────────────────────────────┐   │
│     │ x_next = unicycleStateFcn(x, u, Ts)                   │   │
│     │   Base: [x, y, θ] + Ts*[v*cos(θ), v*sin(θ), ω*]      │   │
│     │   Arm: q_arm + Ts*q̇_arm*                              │   │
│     └──────────┬───────────────────────────────────────────┘   │
│                ↓                                                │
│  3. Full state: [x', y', θ', q_arm']                            │
│     (No IK step - arm motion from MPC directly!)                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

ADVANTAGES:
✅ Unified optimization (all DOF together)
✅ FK in cost = differential IK embedded in optimizer
✅ All constraints checked simultaneously
✅ True receding horizon for full system
✅ Optimal coordination between base and arm
```

## Key Insight

### ❌ Incorrect: "MPC for base, IK for arm"
- Two separate optimization problems
- IK doesn't see MPC's future plans
- MPC doesn't know if arm can reach

### ✅ Correct: "MPC for whole body with FK"
- Single optimization problem
- Optimizer uses FK to compute EE error
- Finds [v, ω, q̇_arm] that minimize EE error
- **This IS differential IK, but inside MPC!**

## Mathematical Comparison

### Phase 1 (Base-Only + IK)
```
Step 1: MPC for base
  minimize    ||[x,y,θ] - [x_ref,y_ref,θ_ref]||²
  subject to  v ∈ [v_min, v_max]
              ω ∈ [ω_min, ω_max]
              |v ± ω*W/2| ≤ v_wheel_max
  variables   v, ω

Step 2: IK for arm (given fixed base [x*,y*,θ*])
  minimize    ||FK(q_arm) - EE_target||²
  subject to  q_arm ∈ [q_min, q_max]
  variables   q_arm
```
**Problem:** Two sequential optimizations, no coordination!

### Phase 2 (Whole-Body)
```
Single MPC:
  minimize    Σ[||p_ee(q) - p_ref||² + ||R_ee(q) - R_ref||²_F 
              + w_input*||u||² + w_rate*||Δu||²]
  subject to  v ∈ [v_min, v_max]
              ω ∈ [ω_min, ω_max]
              |v ± ω*W/2| ≤ v_wheel_max
              q̇_arm ∈ [q̇_min, q̇_max]
              q_arm ∈ [q_min, q_max]
  variables   v, ω, q̇_arm(6)
  
  where q = [x, y, θ, q_arm]
        p_ee(q) = FK(robot, q, eeBodyName)[1:3, 4]
        R_ee(q) = FK(robot, q, eeBodyName)[1:3, 1:3]
```
**Advantage:** Single unified optimization, all DOF coordinated!

## Control Flow Comparison

### Phase 1 (Sequential)
```
Reference → Base MPC → [v, ω] → Simulate base → [x, y, θ]
                                                     ↓
                                         EE target → Arm IK → q_arm
```

### Phase 2 (Unified)
```
Reference → Whole-Body MPC → [v, ω, q̇_arm] → Simulate → [x, y, θ, q_arm]
   (EE)          ↑                                           ↓
                 └──────────── FK (in cost) ←───────────────┘
```

## Implementation Highlights

### Cost Function (eeTrackingCostFcn.m)
```matlab
% For each step in prediction horizon:
for i = 1:p+1
    % Extract state
    q_full = X(:, i);  % [x, y, θ, q_arm(6)]
    
    % Forward kinematics (replaces IK!)
    T_ee = getTransform(robot, q_full, eeBodyName);
    p_ee = T_ee(1:3, 4);
    R_ee = T_ee(1:3, 1:3);
    
    % Position error
    e_pos = p_ee - p_ref(:, i);
    J_pos = weights.position * (e_pos' * e_pos);
    
    % Orientation error
    e_ori = R_ee - R_ref(:, :, i);
    J_ori = weights.orientation * norm(e_ori, 'fro')^2;
    
    % Accumulate
    J = J + J_pos + J_ori;
end
```

**This is differential IK!** The optimizer finds `q̇_arm` that reduces EE error via gradient of FK.

---

**Summary:** Phase 1 was architecturally flawed (decoupled optimization). Phase 2 is correct (unified optimization with FK in cost = differential IK embedded in MPC).
