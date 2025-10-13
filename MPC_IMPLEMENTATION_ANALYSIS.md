# MPC Implementation Analysis & Plan

**Date:** October 13, 2025  
**Branch:** mpc-dev-stageC  
**Document Purpose:** Analyze existing Method 2/3 proposals and plan true MPC implementation

---

## Executive Summary

After reviewing `projectDiagnosis.md` lines 4380-4986, I've identified **three distinct approaches** that have been discussed:

| Approach | Name | Status | Key Insight |
|----------|------|--------|-------------|
| **Method 2** | Iterative Feedback (MPC-style) | 💡 Proposed | Per-waypoint GIK with prediction loop |
| **Method 3** | Differential IK with QP | ⏳ Deferred | Embeds nonholonomic constraint in optimization |
| **True MPC** | NMPC from g5wbcMpcDesign.md | 📋 Your design | Receding horizon with full dynamics |

### Critical Finding

**Methods 2 and 3 in projectDiagnosis.md are NOT true MPC!**

- **Method 2**: Iterative GIK with prediction (offline planning, still uses GIK)
- **Method 3**: Differential IK QP (per-waypoint optimization, but not receding horizon)
- **True MPC**: Your g5wbcMpcDesign.md (receding horizon, online control, replaces GIK entirely)

---

## Detailed Analysis of Existing Proposals

### Method 2: Iterative Feedback (from projectDiagnosis.md)

**Architecture:**
```
For each waypoint (OFFLINE):
  Iteration loop (max 5):
    1. GIK solve → propose (x,y,θ) + arm
    2. Predict chassis → get actual (x,y,θ)
    3. Forward kinematics → check EE error
    4. If error > 50mm: adjust base, retry
    5. Else: accept, next waypoint
```

**Strengths:**
- ✅ Adds feedback to current GIK-based system
- ✅ Per-waypoint error checking
- ✅ Guaranteed <50mm error bound
- ✅ Could run at 10 Hz (real-time capable)

**Weaknesses:**
- ❌ **Still uses GIK** - can request sideways motion
- ❌ **Still offline planning** - not true closed-loop
- ❌ 3-10× slower than Method 1
- ❌ Iterative "band-aid" solution

**Comparison to Method 4:**
```
Method 2 (Proposed):     Iterate(GIK → Predict → Check)
Method 4 (Implemented):  Predict → Constrain(GIK) → Check → Fallback

Method 4 is essentially a non-iterative version of Method 2!
```

**Conclusion:** Method 4 already achieves most of Method 2's goals.

---

### Method 3: Differential IK with QP (from projectDiagnosis.md)

**Architecture:**
```
For each waypoint (OFFLINE):
  Solve QP:
    minimize: ‖J_aug·u - V_d‖² + λ‖u‖²
    subject to:
      - Nonholonomic: q̇_base = S(θ)·[v; ω]
      - Wheel speeds: |v ± ω·W/2| ≤ v_max
      - Joint rates: q̇_min ≤ q̇_arm ≤ q̇_max
  
  where u = [v, ω, q̇_arm]ᵀ ∈ ℝ^8
```

**Key Innovation:**
```matlab
% Augmented Jacobian embeds nonholonomic constraint
S(θ) = [cos(θ)  0 ]
       [sin(θ)  0 ]  % Cannot move sideways!
       [  0     1 ]

J_aug = [J_base·S(θ) | J_arm]  % 6×8 (not 6×9)
```

**Strengths:**
- ✅ **Fundamentally nonholonomic** - v_y ≡ 0 in formulation
- ✅ **Convex QP** - global optimum, fast solve
- ✅ **Unified optimization** - base and arm coupled
- ✅ **Explicit constraints** - wheel speeds directly enforced

**Weaknesses:**
- ⚠️ **Still per-waypoint** - not receding horizon
- ⚠️ **Offline planning** - not online control
- ⚠️ **No lookahead** - only current waypoint
- ⚠️ High implementation effort

**Status:** Deferred - Method 4 provides sufficient improvement

**Conclusion:** Better than Methods 1/2, but not true MPC.

---

## True MPC Approach (from g5wbcMpcDesign.md)

### Fundamental Differences

| Aspect | Method 2 | Method 3 | **True MPC** |
|--------|----------|----------|--------------|
| **Horizon** | 1 waypoint | 1 waypoint | **N=20 steps** |
| **Online/Offline** | Offline | Offline | **Online (20-50 Hz)** |
| **Receding** | No | No | **Yes** |
| **Lookahead** | None | None | **1-2 seconds** |
| **Execution** | Batch planning | Batch planning | **Real-time control** |
| **Solver** | GIK + iteration | QP per waypoint | **NMPC per cycle** |

### Architecture Comparison

**Method 2/3 (Per-Waypoint):**
```
Planning Phase (Offline):
  For each waypoint k:
    Solve optimization
    q_trajectory[k] = result
    
Execution Phase (Online):
  For each q_trajectory[k]:
    Send to robot (open-loop)
```

**True MPC (Receding Horizon):**
```
Planning + Execution (Online, simultaneous):
  At each cycle (50 Hz):
    Solve NMPC over horizon N=20
    u_trajectory[0:19] = result
    Apply u[0] to robot (closed-loop)
    Measure new state
    Shift horizon, repeat
```

**Key Insight:**  
True MPC **plans and executes simultaneously** with feedback!

---

## MPC Mathematical Formulation

### State and Control

**State:** x_k = [p_base, q_arm] ∈ ℝ^9
```
p_base = [x, y, θ]ᵀ  (base pose)
q_arm  ∈ ℝ^6         (arm joints)
```

**Control:** u_k = [v, ω, q̇_arm]ᵀ ∈ ℝ^8
```
v     = forward velocity
ω     = yaw rate
q̇_arm = arm joint velocities
```

### Dynamics

**Base (nonholonomic unicycle):**
```
x_{k+1}     = x_k + h·v_k·cos(θ_k)
y_{k+1}     = y_k + h·v_k·sin(θ_k)
θ_{k+1}     = θ_k + h·ω_k
```

**Arm (integrator):**
```
q_arm,{k+1} = q_arm,k + h·q̇_arm,k
```

**Combined:**
```
x_{k+1} = f(x_k, u_k) = x_k + h·g(x_k, u_k)
```

### End-Effector Forward Kinematics

```
p_ee,k = FK_position(x_k)     ∈ ℝ^3
R_ee,k = FK_orientation(x_k)  ∈ SO(3)
```

**Tracking Error:**
```
e_p,k = p_ee,k - p_ref,k                 (position)
e_R,k = Log(R_ref,k^T · R_ee,k)          (orientation)
```

### NMPC Objective

```
min_{u_0,...,u_{N-1}, ṡ_0,...,ṡ_{N-1}}  J

J = Σ_{k=0}^{N-1} [
      ‖e_p,k‖²_{W_p}                    # Position tracking
    + ‖e_R,k‖²_{W_R}                    # Orientation tracking
    + λ_u·‖u_k‖²                        # Control effort
    + λ_Δu·‖u_k - u_{k-1}‖²             # Smoothness
    + λ_q·‖q_arm,k - q_nom‖²            # Posture regularization
    + λ_s·(ṡ_k - 1)²                    # Time-scaling penalty
    ]
  + ‖e_p,N‖²_{W_pf} + ‖e_R,N‖²_{W_Rf}  # Terminal cost
```

**Progress Variable (Time-Scaling):**
```
s_{k+1} = s_k + h·ṡ_k
0 ≤ ṡ_k ≤ ṡ_max  (typically 1.2)

p_ref,k = EE_trajectory(s_k)
```

**Purpose:** Allows MPC to slow down instead of failing!

### Constraints

**1. Nonholonomic Dynamics (Equality):**
```
x_{k+1} = f(x_k, u_k)  for all k=0..N-1
```

**2. Control Bounds (Box):**
```
-v_max ≤ v_k ≤ v_max           # ±1.5 m/s
-ω_max ≤ ω_k ≤ ω_max           # ±2.0 rad/s
q̇_min ≤ q̇_arm,k ≤ q̇_max       # Joint rate limits
```

**3. Wheel Speed Feasibility (Linear):**
```
v_L,k = v_k - ω_k·W/2
v_R,k = v_k + ω_k·W/2

|v_L,k| ≤ v_wheel_max  →  4 inequalities:
   v_k - ω_k·W/2 ≤  v_wheel_max
  -v_k + ω_k·W/2 ≤  v_wheel_max
   v_k + ω_k·W/2 ≤  v_wheel_max
  -v_k - ω_k·W/2 ≤  v_wheel_max
```

**4. Smoothness (Change rate):**
```
|v_k - v_{k-1}| ≤ a_v·h      # Acceleration limit
|ω_k - ω_{k-1}| ≤ a_ω·h      # Angular acceleration
```

**5. Joint Limits:**
```
q_min ≤ q_arm,k ≤ q_max
```

**6. Collision Avoidance (Optional):**
```
φ(p_base,k) ≥ d_safe - σ_k^base      # Base footprint
D(p_link,k) ≥ d_safe - σ_k^link      # Arm links

where σ ≥ 0 are slack variables with penalty ρ·Σσ
```

### Solution Method

**Option 1: Direct NMPC (IPOPT)**
```
Variables: [u_0,...,u_{N-1}, x_1,...,x_N, ṡ_0,...,ṡ_{N-1}]
Total:     8·N + 9·N + N = 18·N variables (360 for N=20)
Constraints: Dynamics, bounds, collisions
Solver:    CasADi + IPOPT (supports nonlinear constraints)
Speed:     ~50-200ms per solve (depends on horizon)
```

**Option 2: Sequential Convex Programming (SCP/LMPC)**
```
Iteration:
  1. Linearize dynamics around current trajectory
  2. Linearize collision constraints (SDF gradients)
  3. Solve convex QP (very fast, <10ms)
  4. Update trajectory
  5. Repeat 2-5 times until convergence
  
Advantages: Faster per iteration
Disadvantages: Multiple iterations needed
Total speed: ~20-50ms typical
```

**Recommendation for Implementation:**  
Start with **Direct NMPC** (easier to debug), optimize later if needed.

---

## Implementation Comparison

### What We Have Now

```
Method 0 (pureIk):     ✅ Baseline
Method 1 (ppForIk):    ✅ Production (3-pass feed-forward)
Method 4 (ppFirst):    ✅ NEW (per-waypoint with fallback)
```

### What's in projectDiagnosis.md

```
Method 2 (Iterative):  💡 Proposed (offline, iterative GIK)
Method 3 (Diff IK QP): ⏳ Deferred (offline, per-waypoint QP)
```

### What's in g5wbcMpcDesign.md

```
True MPC (NMPC):       📋 Design doc (online, receding horizon)
```

### The Gap

**Methods 2 & 3 are intermediate steps, NOT the final MPC!**

```
Current State:
  Methods 0, 1, 4 → All GIK-based, offline planning
  
Proposed (projectDiagnosis.md):
  Method 2 → Still GIK, adds iteration
  Method 3 → Better (QP), but still per-waypoint
  
Ultimate Goal (g5wbcMpcDesign.md):
  True MPC → Receding horizon, online control, replaces GIK
```

---

## Recommended Implementation Path

### Phase 0: Validate Current Methods (Week 1) ✅
- [x] Method 4 integration test passed
- [ ] **TODO:** Run Method 4 on full 148-waypoint trajectory
- [ ] **TODO:** Benchmark Method 1 vs Method 4
- [ ] **Decision:** If Method 4 sufficient → skip Methods 2/3, go direct to MPC

### Phase 1: Method 3 as Stepping Stone (Weeks 2-3)

**Why Method 3 first?**
- Introduces QP-based optimization
- Embeds nonholonomic constraint
- Tests augmented Jacobian approach
- Simpler than full MPC (no horizon, no time-scaling)

**Implementation:**
```matlab
% Create: matlab/+gik9dof/+control/differentialIK.m

function u_opt = differentialIK(robot, q_current, T_target, options)
    % 1. Compute augmented Jacobian
    J_aug = buildAugmentedJacobian(robot, q_current);
    
    % 2. Compute desired twist
    V_d = computeDesiredTwist(robot, q_current, T_target, options.K_p);
    
    % 3. Setup QP
    [H, f] = setupQPCost(J_aug, V_d, options);
    [A_ineq, b_ineq] = buildConstraints(q_current, options);
    
    % 4. Solve
    u_opt = quadprog(H, f, A_ineq, b_ineq);
end
```

**Testing:**
- 2-waypoint simple trajectory
- Verify constraints satisfied
- Compare to Method 1 & 4

**Expected Results:**
- EE error: 2-5mm (better than Methods 1/4)
- Base motion: Smooth, feasible by construction
- Solve time: ~30ms per waypoint

### Phase 2: True MPC Implementation (Weeks 4-8)

**Step 1: CasADi Setup (Week 4)**
```matlab
% Install CasADi for MATLAB
% Test basic NMPC example
% Verify IPOPT working
```

**Step 2: NMPC Core (Week 5)**
```matlab
% Create: matlab/+gik9dof/+mpc/nmpcController.m

classdef nmpcController < handle
    properties
        N           % Horizon length (20)
        dt          % Time step (0.05s)
        opti        % CasADi Opti stack
        solver      % IPOPT solver
    end
    
    methods
        function obj = nmpcController(robot, options)
            obj.N = options.horizonLength;
            obj.dt = options.timeStep;
            obj.buildOptimization(robot, options);
        end
        
        function [u_opt, x_opt] = solve(obj, x_current, traj_ref)
            % Set initial state
            % Set reference trajectory
            % Solve NMPC
            % Return first control + predicted trajectory
        end
    end
end
```

**Step 3: Dynamics Implementation (Week 5)**
```matlab
function x_next = dynamics(x, u, dt)
    % Unpack state
    x_base = x(1);
    y_base = x(2);
    theta = x(3);
    q_arm = x(4:9);
    
    % Unpack control
    v = u(1);
    omega = u(2);
    q_dot_arm = u(3:8);
    
    % Unicycle integration
    x_next(1) = x_base + dt * v * cos(theta);
    x_next(2) = y_base + dt * v * sin(theta);
    x_next(3) = theta + dt * omega;
    
    % Arm integration
    x_next(4:9) = q_arm + dt * q_dot_arm;
end
```

**Step 4: Cost Function (Week 6)**
```matlab
% EE tracking cost
for k = 1:N
    T_ee = FK(x(:,k));
    e_p = T_ee(1:3,4) - p_ref(:,k);
    e_R = rotationError(T_ee(1:3,1:3), R_ref(:,:,k));
    
    cost = cost + e_p' * W_p * e_p + e_R' * W_R * e_R;
    cost = cost + lambda_u * (u(:,k)' * u(:,k));
    cost = cost + lambda_du * ((u(:,k) - u(:,k-1))' * (u(:,k) - u(:,k-1)));
end
```

**Step 5: Constraints (Week 6)**
```matlab
% Dynamics
for k = 1:N-1
    opti.subject_to(x(:,k+1) == dynamics(x(:,k), u(:,k), dt));
end

% Control bounds
for k = 1:N-1
    opti.subject_to(-v_max <= u(1,k) <= v_max);
    opti.subject_to(-omega_max <= u(2,k) <= omega_max);
end

% Wheel speeds
for k = 1:N-1
    v_k = u(1,k);
    omega_k = u(2,k);
    opti.subject_to(v_k - omega_k*W/2 <= v_wheel_max);
    opti.subject_to(v_k + omega_k*W/2 <= v_wheel_max);
    % ... (4 total)
end
```

**Step 6: Integration with Pipeline (Week 7)**
```matlab
% Create: matlab/+gik9dof/runStageCMPC.m

function logC = runStageCMPC(robot, trajStruct, qStart, options)
    % Initialize MPC controller
    mpc = gik9dof.mpc.nmpcController(robot, options.mpc);
    
    % Initialize trajectory reference
    traj_ref = prepareTrajectoryReference(trajStruct);
    
    % Control loop
    q_current = qStart;
    t = 0;
    k = 1;
    
    while ~reachedGoal(q_current, trajStruct)
        % Get reference over horizon
        ref_horizon = getReferenceHorizon(traj_ref, t, mpc.N, mpc.dt);
        
        % Solve NMPC
        [u_opt, x_pred] = mpc.solve(q_current, ref_horizon);
        
        % Apply first control
        u_apply = u_opt(:,1);
        q_next = dynamics(q_current, u_apply, mpc.dt);
        
        % Log
        logC.qTraj(:,k) = q_next;
        logC.uTraj(:,k) = u_apply;
        logC.time(k) = t;
        
        % Update
        q_current = q_next;
        t = t + mpc.dt;
        k = k + 1;
    end
end
```

**Step 7: Testing & Validation (Week 8)**
- Simple straight line (5 waypoints)
- Full 148-waypoint trajectory
- Compare to Methods 1, 4, 3
- Tune weights (W_p, W_R, λ_u, λ_Δu)

### Phase 3: Optimization & Deployment (Weeks 9-12)

**Performance Targets:**
- Solve time: <50ms per cycle (20 Hz capable)
- EE tracking: <10mm RMS
- Base motion: Smooth, feasible
- Collision avoidance: Guaranteed

**Optimizations:**
- Warm starting (use previous solution)
- Horizon reduction (N=20 → N=10 if needed)
- SCP alternative if NMPC too slow
- Code generation (CasADi → C code)

---

## Decision Tree

```
Start: Do we need MPC?
  │
  ├─ Is Method 4 sufficient? (Test on full trajectory)
  │  ├─ YES → Stick with Method 4, done! ✅
  │  └─ NO → Continue
  │
  ├─ Is the issue sideways base motion?
  │  ├─ YES → Implement Method 3 (Diff IK QP)
  │  │         ├─ Does Method 3 solve it?
  │  │         │   ├─ YES → Done! ✅
  │  │         │   └─ NO → Need true MPC
  │  └─ NO → Continue
  │
  ├─ Do we need real-time closed-loop?
  │  ├─ YES → Must implement true MPC
  │  └─ NO → Method 3 sufficient
  │
  └─ Implement true MPC (8-12 weeks effort)
```

---

## Summary & Recommendations

### Key Findings

1. **Method 2 (Iterative):** Offline planning, iterative GIK - **Method 4 already achieves this**
2. **Method 3 (Diff IK QP):** Better formulation, but still per-waypoint - **Good stepping stone**
3. **True MPC:** Your g5wbcMpcDesign.md - **Ultimate goal, significant effort**

### Recommended Path Forward

**Option A: Conservative (Recommended)**
```
Week 1:   Test Method 4 on full trajectory
Week 2-3: Implement Method 3 if needed
Week 4-8: Implement true MPC if Method 3 insufficient
```

**Option B: Aggressive (If confident)**
```
Week 1:   Test Method 4 (in parallel with MPC setup)
Week 1-8: Go directly to true MPC implementation
          Skip Methods 2 & 3 entirely
```

**My Recommendation:** **Option A**
- Less risk
- Method 3 is valuable even if we do MPC later
- Learn QP formulation before full NMPC
- Test hypotheses incrementally

### Next Immediate Steps

1. ✅ **Complete:** Merge with Method 4 implementation
2. 📊 **Run:** Method 4 on full 148-waypoint trajectory
3. 📈 **Analyze:** Compare Method 1 vs Method 4 performance
4. 🎯 **Decide:** 
   - If Method 4 good enough → Done
   - If not → Implement Method 3 first
   - If Method 3 insufficient → Full MPC

Would you like me to:
1. Start implementing Method 3 (Differential IK QP)?
2. Start implementing true MPC directly?
3. First run benchmark tests of Method 4?
4. Create detailed pseudocode for either approach?

---

**Status:** Analysis Complete ✅  
**Ready for:** Implementation decision
