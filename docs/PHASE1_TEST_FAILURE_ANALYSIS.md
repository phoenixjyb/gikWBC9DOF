# Phase 1 Test Failure Analysis - Critical Issues Found

**Date:** October 14, 2025  
**Test:** `test_method4_phase1_improvements.m`  
**Result:** ❌ **ALL 4 SUCCESS CRITERIA FAILED**

---

## Executive Summary

The Phase 1 test revealed **catastrophic performance degradation** in the enhanced version:

| Metric | Baseline | Enhanced | Change | Status |
|--------|----------|----------|--------|--------|
| **Mean EE Error** | 0.0 mm | 5310.3 mm | +1,810,316,600% | ❌ FAIL |
| **Fallback Rate** | 0.0% | 51.0% | Infinite | ❌ FAIL |
| **Convergence** | 95.7% | 46.7% | -51.2% | ❌ FAIL |
| **Lateral Velocity** | N/A | 41.37 m/s | **2,068x over limit** | ❌ FAIL |

### 🚨 Critical Finding: Nonholonomy Violation

**The enhanced version produces physically impossible lateral velocities:**
- Mean |v_lat|: **41.37 m/s** (target: <0.02 m/s)
- Max |v_lat|: **157.55 m/s** (impossible for differential drive)
- Violations: **207/210 waypoints (98.6%)**

This indicates a **fundamental bug** in the enhanced implementation, not a tuning issue.

---

## Root Cause Analysis

### Issue #1: ⚠️ **Unfair Comparison - Different Baselines**

**Problem:** The test compared two DIFFERENT configurations:

#### Baseline Configuration (Working)
```matlab
'YawTolerance', deg2rad(20),    % ±20° (aggressive)
'PositionTolerance', 0.20,      % ±20cm (relaxed)
'EEErrorTolerance', 0.015,      % 15mm fallback threshold
'MaxIterations', 1500
```

#### Enhanced Configuration (Broken)
```matlab
% From runStageCPPFirst_enhanced.m lines 115-124
options.SolverOptions.ConstraintTolerance = 1e-4;   % Relaxed
options.SolverOptions.StepTolerance = 1e-10;        % Tight
options.SolverOptions.MaxIterations = 2000;          % Increased

% Corridors (lines 228-241)
yawToler = deg2rad(15);          % ±15° (TIGHTER than baseline!)
eps_long = max(0.01, |v|*dt+0.01);  % Velocity-limited
eps_lat = 0.015;                 % 15mm lateral (MUCH tighter!)
```

**The enhanced version used TIGHTER constraints than baseline**, so it's expected to perform worse!

### Issue #2: 🐛 **Lateral Velocity Calculation Bug**

The lateral velocity diagnostic produced **impossible values** (41 m/s mean, 157 m/s max).

**Location:** `matlab/+gik9dof/runStageCPPFirst_enhanced.m` lines 268-281

```matlab
% Current (BUGGY) implementation:
dx = log.qTraj(1, k+1) - log.qTraj(1, k);  % World frame dx
dy = log.qTraj(2, k+1) - log.qTraj(2, k);  % World frame dy
dt_actual = log.timeVec(k+1) - log.timeVec(k);

theta_mid = (log.qTraj(3, k) + log.qTraj(3, k+1)) / 2;

% Transform to body frame
v_lat = (-sin(theta_mid) * dx + cos(theta_mid) * dy) / dt_actual;
```

**Root Cause:** The formula is mathematically correct BUT:
1. **`dt_actual` is likely 0.0** (timesteps not properly set)
2. Division by zero or very small `dt` → explosive values
3. Guide's formula expects `dt = 0.1s` (control rate), not actual timestep

**Evidence:**
- Execution time: 1.63s for 210 waypoints = **0.0078s per waypoint**
- This is 12.8x faster than expected (0.1s), suggesting `dt` issue
- Lateral velocities growing over time (0.057 → 35.87 m/s) suggests accumulation

### Issue #3: 🔧 **Velocity-Limited Corridor Misconfiguration**

**Location:** lines 228-241

```matlab
vx_cmd = ppFollower.CurrentVelocity(1);
eps_long = max(0.01, abs(vx_cmd) * dt + 0.01);  % Guide-exact formula
eps_lat = 0.015;  % 15mm lateral (tight for diff-drive)
```

**Problem:** The guide's formula is velocity-dependent, but:
1. `ppFollower.CurrentVelocity` may not be updated correctly
2. If `vx_cmd = 0`, then `eps_long = 0.02m` (very tight!)
3. This creates **tighter corridor than baseline's 0.20m**

### Issue #4: 🔍 **Warm-Starting May Be Broken**

**Location:** lines 245-263

```matlab
if options.UseWarmStarting && ~isempty(prevSolution) && ...
        ~log.fallbackUsed(k-1) && ppStatus.IsOnPath
    q_init = prevSolution;
else
    q_init = q_current;
end

% ... solver ...

if ~log.fallbackUsed(k)
    prevSolution = qSolution(baseAndArm);  % Store for next iteration
end
```

**Potential Issues:**
1. `prevSolution` only stores **base+arm** (9 DOF), but solver might expect full state
2. If GIK fails repeatedly, warm-starting from failed solutions causes drift
3. The 46.7% convergence rate suggests solver is struggling

### Issue #5: 📊 **Adaptive Lookahead Not Actually Adapting**

**Test Output:**
```
Adaptive lookahead: mean 0.80 m (range 0.80-0.80 m)
```

**Problem:** Lookahead stayed constant at 0.80m (nominal value), never adapted!

**Location:** lines 183-191

```matlab
d_to_next = hypot(dx_to_next, dy_to_next);
Ld_eff = min(options.LookaheadDistance, max(d_to_next, options.LookaheadMin));
```

**Root Cause:** 
- `d_to_next` is distance to **next waypoint**, not PP goal point
- For dense trajectory (210 waypoints, 34m = 0.16m spacing), `d_to_next < Ld_min`
- So `Ld_eff = min(0.80, max(0.16, 0.15)) = min(0.80, 0.16) = 0.16m`
- But output shows 0.80m → **logic is inverted or not being used!**

---

## Why Baseline Performed So Well

The baseline achieved **near-perfect performance** (0.0mm error, 0% fallback, 95.7% convergence) because:

1. **Trajectory is actually easy:** 210 waypoints, only 33.33m total length = 0.158m spacing
2. **Aggressive parameters:** 20° yaw corridor + 0.20m position box = very relaxed
3. **Standard implementation** has no bugs

This is **NOT representative of the original 319mm error baseline** we analyzed from Oct 13 logs!

**Hypothesis:** The original Oct 13 comparison used:
- Longer/more complex trajectory
- Default parameters (15° yaw, 0.15m position tolerance)
- Different chassis profile or control rate

---

## Critical Bugs to Fix (Priority Order)

### 🔴 Priority 1: Fix Lateral Velocity Calculation

**File:** `runStageCPPFirst_enhanced.m` lines 268-281

**Fix:**
```matlab
% Use sample time, not actual elapsed time
dt_sample = options.SampleTime;  % Should be 0.1s

% Get positions
x_k = log.qTraj(1, k);
y_k = log.qTraj(2, k);
x_kp1 = log.qTraj(1, k+1);
y_kp1 = log.qTraj(2, k+1);

% World frame velocity
dx = (x_kp1 - x_k) / dt_sample;
dy = (y_kp1 - y_k) / dt_sample;

% Body frame transformation
theta_mid = (log.qTraj(3, k) + log.qTraj(3, k+1)) / 2;
v_long = dx * cos(theta_mid) + dy * sin(theta_mid);
v_lat  = -dx * sin(theta_mid) + dy * cos(theta_mid);

log.lateralVelocity(k) = v_lat;
```

### 🔴 Priority 2: Match Baseline Parameters

**File:** `test_method4_phase1_improvements.m` lines 70-76

**Change baseline to use DEFAULT parameters** (not aggressive):
```matlab
log_baseline = gik9dof.runStageCPPFirst(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...      % DEFAULT (not 20)
    'PositionTolerance', 0.15, ...        % DEFAULT (not 0.20)
    'EEErrorTolerance', 0.010, ...        % DEFAULT (not 0.015)
    'MaxIterations', 1500, ...
    'VerboseLevel', 1);
```

This will show if enhanced version improves over **fair baseline**.

### 🟡 Priority 3: Fix Velocity-Limited Corridor

**File:** `runStageCPPFirst_enhanced.m` lines 228-241

**Option A: Use larger minimum** (safer)
```matlab
eps_long = max(0.05, abs(vx_cmd) * dt + 0.02);  % Minimum 5cm (vs 1cm)
eps_lat = 0.015;
```

**Option B: Start with baseline width, then adapt** (progressive)
```matlab
if k <= 20
    % Use baseline corridor for first 20 waypoints
    eps_long = options.PositionTolerance;  % 0.15m (baseline)
    eps_lat = 0.015;
else
    % Gradually tighten corridor
    eps_long = max(0.05, abs(vx_cmd) * dt + 0.02);
    eps_lat = 0.015;
end
```

### 🟡 Priority 4: Fix Adaptive Lookahead Logic

**File:** `runStageCPPFirst_enhanced.m` lines 183-191

**Current (BUGGY):**
```matlab
Ld_eff = min(options.LookaheadDistance, max(d_to_next, options.LookaheadMin));
ppFollower.LookaheadDistance = Ld_eff;  % May not be applied!
```

**Fixed:**
```matlab
% Distance to PP goal point (not next waypoint)
[~, ~, ppInfo] = ppFollower.step(pose_current', dt);
d_to_goal = hypot(ppInfo.LookaheadPoint(1) - pose_current(1), ...
                  ppInfo.LookaheadPoint(2) - pose_current(2));

% Adapt: reduce lookahead as we approach goal
Ld_eff = max(options.LookaheadMin, min(options.LookaheadDistance, d_to_goal * 1.5));

% CRITICAL: Actually update PP controller
ppFollower.LookaheadDistance = Ld_eff;
```

### 🟢 Priority 5: Review Warm-Starting

**File:** `runStageCPPFirst_enhanced.m` lines 245-263

**Add safety check:**
```matlab
if options.UseWarmStarting && ~isempty(prevSolution) && ...
        ~log.fallbackUsed(k-1) && ...
        ppStatus.IsOnPath && ...
        k > 1 && log.convergence(k-1)  % Only if previous step converged
    q_init = prevSolution;
else
    q_init = q_current;
end
```

---

## Recommended Action Plan

### Phase 1 (Debug, 2-3 hours)

1. ✅ **Fix lateral velocity calculation** (Priority 1)
   - Use `options.SampleTime` instead of actual elapsed time
   - Validate formula against guide's reference implementation
   - Expected: |v_lat| < 0.02 m/s for all waypoints

2. ✅ **Match baseline parameters** (Priority 2)
   - Change baseline to use default (15°, 0.15m)
   - Ensure apples-to-apples comparison

3. ✅ **Rerun test** with fixes
   - Expected: Enhanced should at least match baseline
   - If still failing, proceed to Priority 3-5

### Phase 2 (If still failing, 2-3 hours)

4. **Fix velocity-limited corridor** (Priority 3)
   - Use larger minimum (5cm vs 1cm)
   - Add progressive tightening logic

5. **Fix adaptive lookahead** (Priority 4)
   - Use distance to goal, not next waypoint
   - Ensure PP controller actually updates

6. **Review warm-starting** (Priority 5)
   - Add convergence check before reusing solution
   - Consider disabling temporarily to isolate issue

### Phase 3 (If passing, proceed to original plan)

7. **Run on ACTUAL challenging trajectory**
   - Use the Oct 13 test configuration (319mm baseline error)
   - This will show real improvement potential

8. **Proceed to Phase 2 improvements** (if Phase 1 successful)
   - Orientation+Z nominal posture (6 hours)
   - Arm-aware Pure Pursuit (14 hours)

---

## Questions to Answer

1. **What trajectory did Oct 13 comparison use?**
   - File: `1_pull_world_scaled.json` (same)
   - But what configuration parameters?

2. **Why did baseline perform so well here but poorly on Oct 13?**
   - Hypothesis: Different trajectory or parameters

3. **Can we reproduce the 319mm baseline error?**
   - This would validate our improvement plan

4. **Should we continue with Phase 1 improvements?**
   - Not until we fix the critical bugs
   - Current implementation is fundamentally broken

---

## Lessons Learned

1. ⚠️ **Always validate diagnostic formulas** with unit tests before integration
2. ⚠️ **Match baseline configuration** exactly for fair comparison
3. ⚠️ **Test on representative workload** (not artificially easy trajectories)
4. ⚠️ **Implement one improvement at a time** for easier debugging
5. ⚠️ **Add logging for intermediate values** (dt, dx, dy, vx_cmd) to catch bugs early

---

## Immediate Next Steps

**Option A: Fix bugs and retest (recommended)**
1. Apply Priority 1 & 2 fixes (30 min)
2. Rerun test (15 min)
3. Analyze results and decide next steps

**Option B: Investigate Oct 13 configuration**
1. Find exact parameters used for 319mm baseline
2. Reproduce that result with current baseline
3. Then apply fixes and compare

**Option C: Start over with simpler approach**
1. Abandon Phase 1 improvements temporarily
2. Focus on reproducing Oct 13 results
3. Implement improvements one at a time with validation

---

**Recommendation:** **Option A** (fix bugs first) - We have clear root causes identified, should be quick to fix and validate.
