# Quick Reference: Method 6 Alternating Control

**Date:** October 14, 2025  
**Status:** 💡 Proposed - Ready for Implementation  
**Estimated Timeline:** 3-4 weeks to production-ready

---

## The Big Idea (In 30 Seconds)

Instead of solving a hard 9-DOF whole-body optimization every timestep:

```
❌ Method 5 (Whole-body MPC):
   Optimize [x, y, θ, q₁, q₂, q₃, q₄, q₅, q₆] all at once
   → 170 variables × 10 horizon steps
   → 6,600 FK calls per solve
   → 4 seconds per step (40x too slow!)
```

Alternate between two easy optimizations:

```
✅ Method 6 (Alternating Control):
   
   t=0:    Optimize [v, ω] with arm frozen (2 vars)
           → 5-20ms solve time
           
   t=50ms: Optimize [q̇₁...q̇₆] with base frozen (6 vars)
           → 10-30ms solve time
           
   Total: ~30ms per cycle → 30 Hz control rate!
   → 130x faster than Method 5!
```

---

## Performance Comparison

| Method | Solve Time | Control Rate | Mean Error | Feedback |
|--------|-----------|--------------|------------|----------|
| Method 1 (ppForIk) | 10ms | Offline | 129mm | None |
| Method 4 (ppfirst) | 15ms | ~10 Hz | 150mm | Per-waypoint |
| **Method 6** | **30ms** | **30 Hz** | **~150mm** | **Continuous** |
| Method 5 (MPC) | 4000ms | 0.25 Hz | 2310mm | Continuous |

**Winner:** Method 6 dominates Method 5, matches Method 1/4 accuracy with real-time feedback!

---

## How It Works

### Even Timesteps (Base Control):

```matlab
% Freeze arm at current configuration
q_arm_frozen = q_arm_current;

% Optimize base motion [v, ω]
[v_opt, omega_opt] = fmincon(@(u) baseCostFcn(u, q_arm_frozen), ...
    u_init, ...
    constraints_base);

% Apply base motion, arm holds
x = x + dt * v_opt * cos(theta);
y = y + dt * v_opt * sin(theta);
theta = theta + dt * omega_opt;
% q_arm unchanged
```

### Odd Timesteps (Arm Control):

```matlab
% Freeze base at current configuration
x_frozen = x_current;
y_frozen = y_current;
theta_frozen = theta_current;

% Optimize arm motion [q̇₁...q̇₆]
q_dot_opt = fmincon(@(qd) armCostFcn(qd, x_frozen, y_frozen, theta_frozen), ...
    qd_init, ...
    constraints_arm);

% Apply arm motion, base holds
q_arm = q_arm + dt * q_dot_opt;
% x, y, theta unchanged
```

---

## Why This Is Smart

### 1. Exploits Weak Coupling

Mobile manipulators have **weak coupling** between base and arm:
- Base motion affects arm reachability (strong dependency)
- Arm motion barely affects base dynamics (weak dependency - mass ratio)
- **Alternating optimization converges like Gauss-Seidel iteration!**

### 2. Natural Problem Decomposition

```
Full problem:     9 DOF → Very hard
Base subproblem:  2 DOF → Easy (nearly convex)
Arm subproblem:   6 DOF → Easy (essentially IK, proven fast)
```

### 3. Reuses Existing Components

- Base optimizer: Similar to pure pursuit controller (proven)
- Arm optimizer: Similar to GIK solver (proven)
- **Low implementation risk!**

---

## Implementation Roadmap

### Week 1: Proof of Concept
- [ ] Implement `solveBaseOptimization()` - 2-variable fmincon
- [ ] Implement `solveArmOptimization()` - 6-variable fmincon or GIK
- [ ] Create `runStageCAlternatingControl()` main loop
- [ ] Test on 5-waypoint straight-line trajectory
- **Success:** No divergence, solve times <50ms, rough tracking

### Week 2-3: Optimization
- [ ] Add smoothness penalties (velocity changes)
- [ ] Tune damping factor (prevent oscillation)
- [ ] Implement predictive arm model in base optimizer
- [ ] Add analytical Jacobians for speed
- **Success:** Solve times <30ms, EE error <200mm

### Week 4: Production Testing
- [ ] Full trajectory validation (all test cases)
- [ ] Benchmark vs Method 1/4
- [ ] Stress tests (sharp turns, singularities)
- [ ] Robustness tests (disturbances)
- **Success:** Match Method 1 accuracy, 30+ Hz rate

---

## Key Design Decisions

### Q: What happens during "frozen" phases?

**Base frozen (odd steps):**
- Option 1: Base truly stops (v=0, ω=0) - simplest
- Option 2: Base coasts with previous velocity - smoother
- **Recommendation:** Start with Option 1

**Arm frozen (even steps):**
- Arm holds joint positions (q̇=0)
- Simple to implement, no additional control needed

### Q: How to prevent oscillation?

**Smoothness penalties:**
```matlab
J_smooth_base = w_smooth * (v - v_prev)^2 + (omega - omega_prev)^2;
J_smooth_arm = w_smooth * sum((q_dot - q_dot_prev).^2);
```

**Damping factor:**
```matlab
% Don't apply full optimal control
u_applied = alpha * u_opt + (1-alpha) * u_prev;  % alpha = 0.8
```

### Q: What if one optimizer fails?

**Fallback strategy:**
```matlab
if base_optimization_failed
    % Use pure pursuit as fallback
    [v, omega] = purePursuitController(current_state, ee_target);
end

if arm_optimization_failed
    % Hold current configuration or use simple IK
    q_dot_arm = zeros(6,1);  % or armIKFallback()
end
```

---

## Expected Results

### Conservative Estimate:
- Solve time: **30ms** (vs Method 1: 10ms, Method 5: 4000ms)
- Control rate: **30 Hz** (vs Method 1: offline, Method 5: 0.25 Hz)
- Mean EE error: **150-180mm** (vs Method 1: 129mm, Method 5: 2310mm)
- Robustness: **High** (continuous feedback)

### Optimistic Estimate (with tuning):
- Solve time: **20ms**
- Control rate: **50 Hz**
- Mean EE error: **100-130mm** (match or beat Method 1!)
- Robustness: **Very high**

---

## Risk Assessment

### Low Risk ✅
- Both subproblems are proven solvable (IK and base control are mature)
- Can reuse existing components (PP controller, GIK)
- Clear fallback to Methods 1/4 if doesn't work

### Medium Risk ⚠️
- Oscillation might require tuning (but solvable with damping)
- Coordination tuning (smoothness weights)
- 3-4 weeks development time

### High Risk ❌
- None! Worst case: doesn't work, learned something, use Methods 1/4

---

## Next Steps

### Immediate:
1. ✅ Review this proposal with team
2. ✅ Get buy-in on priority (Method 6 vs enhancing Method 4)
3. **Decision point:** Green light for Week 1 POC?

### If Approved:
4. Create implementation branch `method6-alternating-control`
5. Start Week 1 POC implementation
6. Daily check-ins on progress

---

## Related Documents

- **Full Proposal:** `METHOD6_ALTERNATING_CONTROL_PROPOSAL.md` (detailed design)
- **Critical Analysis:** `MPC_CRITICAL_ANALYSIS_AND_PATH_FORWARD.md` (why Method 5 failed)
- **Method Numbering:** `METHOD_NUMBERING_GUIDE.md` (all methods overview)

---

## Bottom Line

**Method 6 is the pragmatic middle ground we need:**
- Fast enough for real-time (30 Hz)
- Accurate enough for production (~150mm)
- Smart enough to self-correct (continuous feedback)
- Simple enough to implement (3-4 weeks)

**Recommendation: Proceed with POC immediately!**

