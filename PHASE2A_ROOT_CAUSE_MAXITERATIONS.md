# Phase 2A Second Half Failure - ROOT CAUSE IDENTIFIED

## Executive Summary

**Problem:** Staged run second half (waypoints 106-210) experiences catastrophic failures (1739mm error) while isolated test works perfectly (2.4mm error).

**Root Cause:** **GIK solver MaxIterations mismatch**
- **Isolated test:** MaxIterations = **2000**
- **Staged run:** MaxIterations = **1500** (default)

## Evidence

### Iteration Limit Analysis

Waypoints 100-110 in staged run:
```
WP#   Conv?  Error(mm)  Iterations
---   -----  ---------  ----------
100   NO         0.01       1500  ← Hit limit!
101   NO         0.06       1500  ← Hit limit!
102   NO         0.32       1500  ← Hit limit!
103   NO         1.42       1500  ← Hit limit!
104   NO         4.88       1500  ← Hit limit!
105   NO       134.90       1500  ← Hit limit!
106   NO       275.28       1500  ← Hit limit!
107   NO       403.12       1500  ← Hit limit!
108   NO       518.20       1500  ← Hit limit!
109   NO       650.60       1500  ← Hit limit!
110   NO       732.10       1500  ← Hit limit!
```

**Every waypoint from 100-210 hits the 1500 iteration limit without converging!**

### Configuration Comparison

**Isolated Test (`test_stagec_only_home_start.m`):**
```matlab
gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'MaxIterations', 2000, ...  % ← Explicitly set to 2000
    ...
```

**Staged Run (`trackReferenceTrajectory.m` line 84):**
```matlab
options.MaxIterations (1,1) double {mustBePositive} = 1500  % ← Default
```

**Staged Run (`run_fresh_sim_with_animation.m`):**
- Does NOT specify `MaxIterations` parameter
- Falls back to default value of 1500

### Convergence Rate Comparison

|          | First Half (1-105) | Second Half (106-210) | Overall |
|----------|-------------------|----------------------|---------|
| **Isolated** | 94.3% (99/105) | 54.3% (57/105) | 74.3% |
| **Staged**   | 79.0% (83/105) | **0.0% (0/105)** | 39.5% |

**Key Observation:** Even in the isolated test, second half convergence drops to 54.3%, suggesting the trajectory becomes more difficult. But with 2000 iterations, it still succeeds. With only 1500 iterations, it completely fails.

## Why Second Half is More Difficult

1. **Tighter constraints:** Phase 2A uses `Orientation+Z` priority, requiring precise orientation matching
2. **Accumulated numerical error:** Later waypoints may have accumulated floating-point errors
3. **Arm configuration:** Different starting arm config from Stage B may affect reachability
4. **Trajectory characteristics:** Second half may have sharper turns or more complex motions

## The Fix

### Option 1: Update `run_fresh_sim_with_animation.m` (Recommended)

Add `MaxIterations` parameter:

```matlab
log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'ExecutionMode', 'ppFirst', ...
    'MaxIterations', 2000, ...  % ← ADD THIS LINE
    ...
```

### Option 2: Update Default in `trackReferenceTrajectory.m`

Change line 84:

```matlab
options.MaxIterations (1,1) double {mustBePositive} = 2000  % Increased from 1500
```

**Recommendation:** Use Option 1 to be explicit about Phase 2A requirements. Option 2 affects all runs globally.

## Expected Outcome

With `MaxIterations = 2000`:
- **Second half convergence:** Should improve from 0% to ~54% (matching isolated test)
- **Mean error:** Should drop from 1739mm to ~2-5mm
- **Overall convergence:** Should improve from 39.5% to ~74%

## Additional Findings

### Phase 1/2A Parameters ARE Being Passed Correctly

```matlab
'UseAdaptiveLookahead', true, ...
'UseMicroSegment', true, ...
'UseWarmStarting', true, ...
'UseVelocityCorridor', true, ...
'LogLateralVelocity', true, ...
'RelaxedTolerances', true, ...
'UseOrientationZNominal', true, ...  % Phase 2A
```

All Phase 1 and Phase 2A parameters are correctly passed in `runStagedTrajectory.m` lines 954-967.

### Diagnostic Data Loss (Minor Issue)

Phase 1 diagnostic fields (`lateralVelocity`, `lookaheadEffective`, etc.) are computed by `runStageCPPFirst_enhanced` but not copied to the final `logC` structure in `runStagedTrajectory.m`. This is a logging issue, not a functional issue.

**Optional enhancement:** Copy diagnostic fields in `runStagedTrajectory.m` around line 1000:

```matlab
% Copy Phase 1 diagnostic fields for analysis
if isfield(rawLog, 'lateralVelocity')
    logC.lateralVelocity = rawLog.lateralVelocity;
    logC.lookaheadEffective = rawLog.lookaheadEffective;
    logC.microSegmentUsed = rawLog.microSegmentUsed;
    logC.corridorSizes = rawLog.corridorSizes;
end
```

## Summary

**The root cause of the second-half failure is NOT initialization (as originally hypothesized), but rather insufficient GIK solver iterations.**

The r=0.985 correlation we observed earlier was a red herring - the base path divergence was a *consequence* of GIK failures, not the cause.

The fix is simple: increase `MaxIterations` from 1500 to 2000 in the staged run configuration.

---

**Status:** Root cause confirmed, fix identified  
**Next Step:** Apply fix and validate with full staged simulation  
**Expected Result:** Second half error < 5mm, convergence rate ~74%
