# Issue Analysis and Recommended Fixes

**Date:** October 10, 2025  
**Context:** Parameter sweep animations reveal three critical issues

---

## Issue #1: Arm Tip Not Aligned with Stage C Reference EE

### Observation
In animations, the executed arm tip (green) does not align well with the Stage C reference EE waypoints (magenta dash-dot).

### Root Cause Analysis

**Likely NOT an animation bug** - the diagnostics confirm this is real:
```
eeErrorMean: 0.0488m (48.8mm average deviation)
eeErrorMax:  0.1332m (133.2mm max deviation)
```

**The Issue:** Stage C reference EE path (first GIK pass with locked base) is being computed correctly, but the executed arm (second GIK pass) deviates because the base has drifted significantly.

**Evidence from Diagnostics:**
```
basePosDeviationMean: 1.3255m  ← Base drifted 1.3m from reference!
basePosDeviationMax:  2.6499m  ← Max 2.6m deviation
baseYawDriftMean:     1.5837rad ← 90.7° yaw drift
baseYawDriftMax:      3.0473rad ← 174.6° yaw drift
```

### Why This Happens

1. **Stage C Two-Pass Algorithm:**
   - **Pass 1:** GIK with "ideal" base states (from pure pursuit on Stage B path)
   - **Pass 2:** GIK with "executed" base states (actual chassis motion)

2. **Pure Pursuit Can't Track Reference:**
   - Reference base path from Pass 1 is fed to pure pursuit
   - Pure pursuit tries to follow it but FAILS badly (1.3m mean deviation)
   - Second GIK pass uses the deviated base, causing arm to reach differently

3. **Why Pure Pursuit Fails:**
   - Lookahead too large (0.8m) for tight maneuvers
   - Reference path may not be dynamically feasible
   - Acceleration limits too restrictive
   - Curvature too high at turns

### Recommended Fixes

**Option A: Improve Pure Pursuit Tracking** (Recommended)
```matlab
% In runStagedReference or sweep parameters:
'ChassisOverrides', struct( ...
    'lookahead_base', 0.4,        % Reduce from 0.8m (tighter tracking)
    'accel_limit', 1.0,           % Increase from 0.8 (more responsive)
    'heading_kp', 1.5,            % Increase from 1.2 (stronger correction)
    'feedforward_gain', 0.95)     % Increase from 0.9 (better curvature following)
```

**Option B: Enable Adaptive Lookahead**
```matlab
'StageCLookaheadVelGain', 0.3,    % Scale lookahead with velocity
'StageCLookaheadTimeGain', 0.2    % Time-based lookahead component
```

**Option C: Skip First Pass, Use Stage B Path Directly**
- Don't recompute base via GIK first pass
- Use Stage B Hybrid A* path directly for Stage C
- Apply Stage C refinement to smooth it further

---

## Issue #2: Stage C Reference Paths Jumpy/Kinky

### Observation
Stage C reference EE and base paths show jerkiness, though no collisions occur.

### Root Cause Analysis

**Primary Cause: Solver Hitting Max Iterations**
```
solverIterationsMean: 150  ← Every waypoint hits cap
solverIterationsMax:  150
solverIterationsStd:  0    ← No variation = consistent failure
```

**What This Means:**
- GIK solver is not converging within 150 iterations
- Each waypoint solution is "best effort" but not optimal
- Accumulated error causes jerky motion between waypoints

**Secondary Cause: Stage C Refinement Ineffective**
```
refinementApplied: true
refinementDelta: (probably zeros or minimal)
```
- Refinement is running but not improving anything
- RS/clothoid smoothing on Stage C base not helping

### Why Solver Can't Converge

**Possible Reasons:**

1. **Distance Constraints Too Tight**
   - DistanceBounds lower = 0.1m (our Phase 1 tuning)
   - May be infeasible for some arm configurations

2. **Target Poses Difficult to Reach**
   - JSON reference trajectory may have aggressive motions
   - 10Hz control rate limits solver time per step

3. **Initial Guess Poor Quality**
   - Previous waypoint solution used as initial guess
   - If previous was bad, next inherits the problem

### Recommended Fixes

**Fix #1: Increase Max Iterations** (Quick)
```matlab
'MaxIterations', 300  % Increase from 150
```

**Fix #2: Relax Distance Constraints** (If Fix #1 insufficient)
```matlab
'DistanceSpecs', {{obstacleDiscs, [0.15, Inf]}}  % Increase lower bound
```

**Fix #3: Improve Initial Guess** (Advanced)
- Use velocity-based extrapolation instead of previous pose
- Add intermediate waypoints in high-curvature regions

**Fix #4: Reduce Control Rate for Planning** (Trade speed for quality)
```matlab
'RateHz', 5  % Plan at 5Hz, then interpolate to 10Hz for execution
```

---

## Issue #3: Stage B Cusps Still Obvious

### Observation
Despite using Hybrid A*, visible cusps in Stage B base paths.

### Root Cause Analysis

**Smoking Gun in Diagnostics:**
```
rsIterations: 200           ← RS attempted 200 iterations
rsImprovements: 0           ← BUT ZERO improvements accepted!
rsPathLengthImprovement: 0  ← No length reduction
clothoidApplied: 1          ← Clothoid WAS applied
clothoidSegments: 1         ← But only 1 segment fitted
```

**What This Tells Us:**

1. **RS Smoothing Failing Completely**
   - 200 attempts, 0 successes
   - Why? Likely:
     a) `lambdaCusp` too restrictive (still rejecting all shortcuts)
     b) Occupancy map too tight (shortcuts collide)
     c) Path already optimal per RS criteria

2. **Clothoid Smoothing Weak**
   - Only 1 segment fitted (should be multiple for complex paths)
   - Probably fitting entire path as one spline (minimal smoothing)

3. **Hybrid A* Primitives Have Cusps**
   - Hybrid A* uses discrete motion primitives
   - These primitives CAN include reverse motions (cusps)
   - Smoothing is supposed to remove them, but it's not working

### Why RS Smoothing Fails

**Check defaultReedsSheppParams:**
```matlab
% Current settings (from Phase 1 tuning):
lambdaCusp = 1.0          % Lower than original 3.0
allowReverse = true       % Enabled
iters = 200               % Reduced from 600
```

**Hypothesis:** Even with `lambdaCusp=1.0`, shortcuts are being rejected due to:

1. **Collision Checking Too Conservative**
   - `validationDistance = 0.035m` (very dense checking)
   - `inflationRadius` may be too large
   - Shortcuts appear to collide even though Hybrid A* path doesn't

2. **Cost Function Penalizing Improvements**
   - RS path may be shorter but have more cusps
   - Cost = length + lambdaCusp × cusps
   - If cusps increase too much, improvement rejected

3. **Occupancy Map Resolution Issues**
   - Hybrid A* uses different collision model than RS
   - RS validation may be stricter

### Why Clothoid Smoothing Weak

**Only 1 Segment Fitted:**
- Clothoid expects gear changes (forward/reverse transitions) to split path
- If Hybrid A* path is all-forward or has internal cusps not detected, it's treated as single segment
- Single clothoid spline can't remove cusps, only smooth curvature

### Recommended Fixes

**Fix #1: Make RS Acceptance More Lenient** (Most Important)

```matlab
% In defaultReedsSheppParams.m or as parameter override:
params.lambdaCusp = 0.5;           % Even lower (was 1.0)
params.reverseCost = 3.0;          % Lower (was 5.0)
params.validationDistance = 0.08;  % Coarser (was 0.035m)
```

**Fix #2: Adjust Occupancy Map**

```matlab
% In Hybrid A* call, reduce inflation:
plannerInfo.inflationRadius = 0.08;  % Match RS validation
```

**Fix #3: Force Clothoid Segmentation**

```matlab
% In rsClothoidRefine options:
options.ForceSegmentation = true;   % Split at high curvature points
options.MaxSegmentLength = 2.0;     % Force splits every 2m
```

**Fix #4: Post-Process Cusps Explicitly**

Add custom cusp detection and local refinement:
```matlab
% After Hybrid A*, before RS:
cuspIdx = detectCusps(path);  % Find direction reversals
for each cusp:
    localPath = refineCuspWithBezier(path, cuspIdx, radius=0.5);
end
```

**Fix #5: Alternative - Use Only Clothoid, Skip RS**

```matlab
% If RS keeps failing:
options.UseStageBReedsShepp = false;   % Disable RS
options.UseStageBClothoid = true;       % Keep clothoid
options.ClothoidForceSmooth = true;     % Aggressive smoothing
```

---

## Recommended Action Plan

### Immediate Actions (Quick Wins)

1. **Increase MaxIterations to 300** (Fix Issue #2)
   ```matlab
   % In test scripts or sweep:
   'MaxIterations', 300
   ```

2. **Reduce Stage C Lookahead** (Fix Issue #1)
   ```matlab
   'ChassisOverrides', struct('lookahead_base', 0.4)
   ```

3. **Relax RS Acceptance** (Fix Issue #3)
   ```matlab
   % Edit defaultReedsSheppParams.m:
   params.lambdaCusp = 0.5;
   params.validationDistance = 0.08;
   ```

### Medium-Term Actions (1-2 hours)

4. **Create Diagnostic Visualization Script**
   - Plot Stage B curvature profile
   - Highlight cusp locations
   - Show RS attempted vs accepted shortcuts
   - Visualize collision margins

5. **Run Targeted Experiments**
   - Test MaxIters: [150, 200, 300, 500]
   - Test lambdaCusp: [0.1, 0.5, 1.0, 2.0]
   - Test lookahead: [0.3, 0.4, 0.6, 0.8]

6. **Implement Curvature-Based Smoothing**
   - Detect high-curvature regions (>2.0 [1/m])
   - Apply local Bezier/spline smoothing
   - Validate against obstacles

### Long-Term Actions (1-2 days)

7. **Redesign Stage C Two-Pass**
   - Single-pass GIK with MPC-style base planning
   - Or skip first pass, use Stage B path directly

8. **Implement Adaptive Pure Pursuit**
   - Dynamic lookahead based on curvature
   - Acceleration limit adaptation
   - Predictive base drift compensation

9. **Alternative Smoothing Algorithms**
   - B-spline fitting with obstacle avoidance
   - Cubic Hermite splines with C2 continuity
   - Trajectory optimization (e.g., CHOMP, TrajOpt)

---

## Testing Script

Here's a quick test to validate fixes:

```matlab
% test_issue_fixes.m
result = gik9dof.runStagedReference( ...
    'RunLabel', 'ISSUE_FIXES_test', ...
    'RateHz', 10, ...
    'MaxIterations', 300, ...                    % FIX #1
    'DistanceMargin', 0.10, ...
    'StageBReedsSheppParams', struct( ...        % FIX #3
        'lambdaCusp', 0.5, ...
        'validationDistance', 0.08), ...
    'ChassisOverrides', struct( ...              % FIX #2
        'lookahead_base', 0.4, ...
        'accel_limit', 1.0, ...
        'heading_kp', 1.5));

% Check diagnostics
stageBDiag = result.log.stageLogs.stageB.diagnostics;
stageCDiag = result.log.stageLogs.stageC.diagnostics;

fprintf('Stage B:\n');
fprintf('  RS improvements: %d / %d (%.1f%%)\n', ...
    stageBDiag.rsImprovements, stageBDiag.rsIterations, ...
    100*stageBDiag.rsAcceptanceRate);
fprintf('  Cusp count: %d\n', stageBDiag.cuspCount);
fprintf('  Path smoothness: %.3f\n', stageBDiag.pathSmoothness);

fprintf('\nStage C:\n');
fprintf('  Solver iters: %.1f ± %.1f\n', ...
    stageCDiag.solverIterationsMean, stageCDiag.solverIterationsStd);
fprintf('  EE error: %.4fm mean, %.4fm max\n', ...
    stageCDiag.eeErrorMean, stageCDiag.eeErrorMax);
fprintf('  Base deviation: %.3fm mean, %.3fm max\n', ...
    stageCDiag.basePosDeviationMean, stageCDiag.basePosDeviationMax);
```

---

## Summary

| Issue | Root Cause | Primary Fix | Expected Improvement |
|-------|------------|-------------|----------------------|
| **#1: EE Misalignment** | Pure pursuit can't track reference (1.3m deviation) | Reduce lookahead to 0.4m | Deviation <0.3m |
| **#2: Jumpy Reference** | Solver hitting max iters (150), not converging | Increase MaxIters to 300 | Std dev >0, mean <250 |
| **#3: Stage B Cusps** | RS rejecting all shortcuts (0/200 accepted) | Relax lambdaCusp to 0.5 | >10% acceptance rate |

**All three issues are interconnected:**
- Issue #3 (Stage B cusps) → causes kinky base path
- Kinky base path → GIK struggles (Issue #2)  
- GIK struggles → poor reference → pure pursuit fails (Issue #1)

**Therefore: Fix Stage B first, then re-evaluate Stage C issues.**

---

**Next Step:** Run `test_issue_fixes.m` with recommended parameters and compare animations.
