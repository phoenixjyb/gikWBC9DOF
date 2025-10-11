# Algorithm Improvement & Testing Plan for GIK 9-DOF Staged Path Planning

## Executive Summary

This plan addresses the "clumsy, kinky, and random" base and EE paths in the staged trajectory controller by implementing data-driven parameter optimization, enhanced diagnostics, and systematic validation protocols. The goal is to reduce Stage C EE deviations to **mean <0.10m, max <0.20m** while eliminating zig-zags in Stage B base ribbons.

**Status**: Phase 1 parameter updates completed; Phase 2-4 implementation and validation pending.

---

## Current Baseline (Reference: `20251010_155837_staged_10hz_legends3`)

| Metric | Stage C Reference | Stage C Executed | Target |
|--------|-------------------|------------------|--------|
| Mean EE Error vs JSON | 0.161 m | 0.096 m | <0.10 m |
| Max EE Error vs JSON | 1.078 m | 0.165 m | <0.20 m |
| Base Ribbon Quality | Zig-zags present | Follows reference | Smooth curves |
| Control Rate | 10 Hz | 10 Hz | 10 Hz |

---

## Phase 1: Parameter Updates [COMPLETED]

### Changes Applied

#### 1.1 Safety Margin Relaxation
**Files**: `runStagedTrajectory.m`, `createGikSolver.m`
- `StageBHybridSafetyMargin`: 0.15m → 0.1m
- `DistanceBounds` lower limit: 0.2m → 0.1m
- **Rationale**: Reduce unnecessary detours around obstacles

#### 1.2 Reeds-Shepp Tuning
**File**: `defaultReedsSheppParams.m`
- `lambdaCusp`: 3.0 → 1.0 (lower penalty for direction changes)
- `allowReverse`: false → true (permits reverse segments)
- `iters`: 600 → 200 (prevents over-optimization)
- **Rationale**: Allow smoother shortcuts without zig-zags

#### 1.3 Clothoid Smoothing Enhancement
**File**: `rsClothoidRefine.m`
- `discretizationDistance`: 0.05m → 0.08m
- **Rationale**: Gentler curvature transitions

#### 1.4 Stage C Refinement Adjustment
**File**: `runStagedTrajectory.m`
- `StageCUseBaseRefinement`: true → false (default)
- **Rationale**: Preserve JSON path shape when Stage B is smooth

#### 1.5 Pure Pursuit Softening
**Files**: `chassis_profiles.yaml`, `runStagedTrajectory.m`
- `lookahead_base`: 0.6m → 0.8m
- `accel_limit`: 1.2 m/s² → 0.8 m/s²
- `StageCLookaheadDistance`: 0.4m → 0.8m
- **Rationale**: Reduce overshoot and oscillations at 10Hz

---

## Phase 2: Enhanced Logging Infrastructure

### 2.1 Diagnostic Metrics to Capture

#### Stage B Metrics (Add to `runStagedTrajectory.m` ~line 1120)
```matlab
stageBDiagnostics = struct(...
    'baseCurvature', [], ...          % Curvature at each base state
    'curvatureHistogram', [], ...     % Bins: <0.5, 0.5-1.0, 1.0-2.0, >2.0 [1/m]
    'cuspLocations', [], ...          % Indices where direction changes
    'pathSmoothness', 0, ...          % Std dev of curvature
    'rsAcceptanceRate', 0, ...        % Shortcuts accepted / attempted
    'clothoidSuccess', false, ...     % Whether clothoid fitting succeeded
    'plannerComputeTime', 0 ...       % Hybrid A* + refinement time [s]
);
```

#### Stage C Metrics (Add to `runTrajectoryControl.m` ~line 108)
```matlab
stageCDiagnostics = struct(...
    'solverIterations', [], ...       % Per-waypoint iteration count
    'eeErrorBins', struct(...         % Histogram of EE errors
        'excellent', 0, ...           % <0.05m
        'good', 0, ...                % 0.05-0.10m
        'acceptable', 0, ...          % 0.10-0.20m
        'poor', 0), ...               % >0.20m
    'baseYawDrift', [], ...           % Cumulative yaw error vs reference
    'refinementAccepted', false, ...  % Whether Stage C refinement was applied
    'refinementDelta', struct(...     % Changes from refinement
        'pathLength', 0, ...
        'eeErrorMean', 0, ...
        'eeErrorMax', 0)
);
```

#### Animation Enhancements (Add to `animate_whole_body.m` ~line 101)
- Color-coded path segments by EE error magnitude
- Highlight curvature spikes (>1.5 rad/m) with markers
- Display solver iteration count in HUD when >500
- Show RS/clothoid acceptance status for each stage

### 2.2 Implementation Tasks
- [ ] Extend `runTrajectoryControl` to compute EE error bins and iteration counts
- [ ] Add curvature computation to `stageCApplyBaseRefinement`
- [ ] Create `computeBaseRibbonMetrics` helper function
- [ ] Update animation to visualize trouble segments

---

## Phase 3: Stage B Parameter Sweep Framework

### 3.1 Sweep Script Structure
**New File**: `matlab/run_stageb_parameter_sweep.m`

```matlab
function results = run_stageb_parameter_sweep(options)
% Systematic exploration of Stage B planning parameters

arguments
    options.SafetyMargins (1,:) double = [0.05, 0.1, 0.15, 0.2]
    options.LambdaCusps (1,:) double = [0.5, 1.0, 2.0, 3.0]
    options.MaxIters (1,:) double = [100, 200, 400, 600]
    options.AllowReverse (1,:) logical = [true, false]
    options.ClothoidDiscretizations (1,:) double = [0.03, 0.05, 0.08, 0.10]
    options.ReferenceTrajectory string = "1_pull_world_scaled.json"
    options.SaveResults logical = true
end

% Grid search over parameter combinations
paramGrid = buildParameterGrid(options);
numConfigs = size(paramGrid, 1);

results = repmat(struct(...
    'config', [], ...
    'eeErrorMean', NaN, ...
    'eeErrorMax', NaN, ...
    'pathLength', NaN, ...
    'cuspCount', 0, ...
    'computeTime', NaN, ...
    'rsAcceptanceRate', 0, ...
    'log', []), numConfigs, 1);

for i = 1:numConfigs
    cfg = paramGrid(i, :);
    fprintf('[%d/%d] Testing: margin=%.2f, lambda=%.1f, iters=%d, reverse=%d, clothoid=%.2f\n', ...
        i, numConfigs, cfg.safetyMargin, cfg.lambdaCusp, cfg.iters, cfg.allowReverse, cfg.clothoidDisc);
    
    try
        log = runStageBWithConfig(cfg);
        results(i) = extractMetrics(log, cfg);
    catch ME
        warning('Config %d failed: %s', i, ME.message);
    end
end

% Generate comparison plots and recommendations
if options.SaveResults
    saveResultsAndPlots(results);
end
end
```

### 3.2 Evaluation Metrics

| Metric | Weight | Target | Measurement |
|--------|--------|--------|-------------|
| EE Error Mean | 0.35 | <0.10m | Stage C executed vs JSON |
| EE Error Max | 0.25 | <0.20m | Stage C executed vs JSON |
| Path Smoothness | 0.20 | σ<0.5 | Std dev of base curvature |
| Cusp Count | 0.10 | <5 | Direction changes in base |
| Computation Time | 0.10 | <2s | Hybrid A* + refinement |

**Composite Score**: Weighted sum normalized to [0, 100]

### 3.3 Expected Outcomes
- Pareto front of error vs. smoothness vs. computation time
- Optimal parameter sets for different scenarios (tight obstacles, open space, high curvature)
- Sensitivity analysis showing which parameters matter most

---

## Phase 4: Stage C Refinement Gating

### 4.1 Acceptance Criteria Logic
**Location**: `matlab/+gik9dof/runStagedTrajectory.m` in `stageCApplyBaseRefinement` (~line 1627)

```matlab
function [pathOut, logRefOut, info] = stageCApplyBaseRefinement(...)
    % ... existing refinement code ...
    
    % Compute metrics before/after refinement
    metrics.before = struct(...
        'eeErrorMean', computeMeanEEError(logRefIn, trajStruct), ...
        'eeErrorMax', computeMaxEEError(logRefIn, trajStruct), ...
        'pathLength', info.lengthBefore, ...
        'curvatureMean', mean(abs(computeCurvature(basePath))));
    
    metrics.after = struct(...
        'eeErrorMean', computeMeanEEError(logRefOut, trajStruct), ...
        'eeErrorMax', computeMaxEEError(logRefOut, trajStruct), ...
        'pathLength', info.lengthAfter, ...
        'curvatureMean', mean(abs(computeCurvature(pathOut))));
    
    % Gating thresholds
    eeErrorDeltaThreshold = 0.02;  % Accept only if error improves by >2cm
    pathLengthMinImprovement = -0.10;  % Accept up to 10cm path length increase
    curvatureMaxIncrease = 0.2;  % Reject if mean curvature increases >0.2 rad/m
    
    % Decision logic
    eeImprovement = metrics.before.eeErrorMean - metrics.after.eeErrorMean;
    pathLengthDelta = metrics.after.pathLength - metrics.before.pathLength;
    curvatureDelta = metrics.after.curvatureMean - metrics.before.curvatureMean;
    
    if eeImprovement < eeErrorDeltaThreshold || ...
       pathLengthDelta > pathLengthMinImprovement || ...
       curvatureDelta > curvatureMaxIncrease
        % Reject refinement
        pathOut = basePath;
        logRefOut = logRefIn;
        info.applied = false;
        info.rejectionReason = determineRejectionReason(...
            eeImprovement, pathLengthDelta, curvatureDelta);
    else
        % Accept refinement
        info.applied = true;
        info.acceptanceCriteria = metrics;
    end
    
    % Log decision for traceability
    info.gatingSummary = struct(...
        'eeImprovement', eeImprovement, ...
        'pathLengthDelta', pathLengthDelta, ...
        'curvatureDelta', curvatureDelta, ...
        'thresholds', struct(...
            'eeErrorDelta', eeErrorDeltaThreshold, ...
            'pathLengthMin', pathLengthMinImprovement, ...
            'curvatureMax', curvatureMaxIncrease));
end
```

### 4.2 Helper Functions
- `computeMeanEEError(log, trajStruct)`: Compare log.eePositions to trajStruct.Poses
- `computeMaxEEError(log, trajStruct)`: Maximum deviation
- `computeCurvature(pathStates)`: Finite difference curvature from SE(2) states
- `determineRejectionReason(...)`: Diagnostic string for logging

---

## Phase 5: Scenario-Based Margin Presets

### 5.1 Preset Definitions
**Location**: `matlab/+gik9dof/environmentConfig.m` (~line 22)

```matlab
function config = environmentConfig(options)
arguments
    options.Scenario (1,1) string {mustBeMember(options.Scenario, ...
        ["tight","nominal","relaxed","custom"])} = "nominal"
    options.CustomMargins struct = struct()
    options.ObstacleAnnotations struct = struct()
end

% Scenario presets
presets = struct(...
    'tight', struct(...
        'safetyMargin', 0.05, ...
        'distanceMargin', 0.08, ...
        'description', 'Minimal clearance for wide-open spaces'), ...
    'nominal', struct(...
        'safetyMargin', 0.10, ...
        'distanceMargin', 0.10, ...
        'description', 'Balanced clearance for typical environments'), ...
    'relaxed', struct(...
        'safetyMargin', 0.15, ...
        'distanceMargin', 0.15, ...
        'description', 'Conservative clearance for dense obstacles'));

% Select preset
if strcmpi(options.Scenario, "custom")
    margins = options.CustomMargins;
else
    margins = presets.(options.Scenario);
end

% Context-aware adjustment based on obstacle density
if ~isempty(options.ObstacleAnnotations)
    margins = adjustMarginsForDensity(margins, options.ObstacleAnnotations);
end

config.SafetyMargin = margins.safetyMargin;
config.DistanceMargin = margins.distanceMargin;
% ... rest of config
end
```

### 5.2 Obstacle Density Computation
```matlab
function marginsOut = adjustMarginsForDensity(marginsIn, obstacles)
% Increase margins in high-density regions

bounds = computeWorkspaceBounds(obstacles);
workspaceArea = (bounds.xmax - bounds.xmin) * (bounds.ymax - bounds.ymin);
totalObstacleArea = sum(arrayfun(@(o) pi * o.Radius^2, obstacles));
densityRatio = totalObstacleArea / workspaceArea;

if densityRatio > 0.3
    % High density: increase margins by 50%
    marginsOut.safetyMargin = marginsIn.safetyMargin * 1.5;
    marginsOut.distanceMargin = marginsIn.distanceMargin * 1.5;
elseif densityRatio < 0.1
    % Low density: decrease margins by 30%
    marginsOut.safetyMargin = marginsIn.safetyMargin * 0.7;
    marginsOut.distanceMargin = marginsIn.distanceMargin * 0.7;
else
    marginsOut = marginsIn;
end
end
```

---

## Phase 6: Pure Pursuit Optimization for 10Hz

### 6.1 Rate-Specific Tuning Considerations

At 10Hz (dt=0.1s), the system has limited reaction time. Key adjustments:

| Parameter | Current | Proposed 10Hz | Rationale |
|-----------|---------|---------------|-----------|
| `lookahead_base` | 0.8m | 0.6-0.8m | Balance preview vs. responsiveness |
| `lookahead_vel_gain` | 0.30s | 0.25s | Reduce at lower rates |
| `lookahead_time_gain` | 0.05s² | 0.08s² | Account for accel delay |
| `heading_kp` | 1.2 | 1.5 | Tighter heading control |
| `heading_ki` | 0.0 | 0.05 | Small integral for steady-state |
| `heading_kd` | 0.1 | 0.15 | Dampen oscillations |
| `accel_limit` | 0.8 m/s² | 0.6-0.8 m/s² | Smooth acceleration |
| `feedforward_gain` | 0.9 | 0.85 | Reduce curvature feedforward |

### 6.2 Reverse Handling Enhancement
**Location**: `purePursuitFollower.m` (~line 22)

```matlab
% In computeYawRate method
if obj.ReverseEnabled && vx < 0
    % Invert heading error for reverse motion
    headingErrorEffective = -headingError;
    % Reduce integral gain in reverse to prevent windup
    integralScale = 0.5;
else
    headingErrorEffective = headingError;
    integralScale = 1.0;
end

wzFB = obj.HeadingKp * headingErrorEffective + ...
       obj.HeadingKd * headingRate + ...
       obj.HeadingKi * obj.HeadingIntegral * integralScale;
```

### 6.3 Testing Protocol
1. **Straight-line tracking**: Measure cross-track error and heading stability
2. **Circular paths**: Test at radii 0.5m, 1.0m, 2.0m with varying velocities
3. **S-curve**: Validate smooth transitions between opposite curvatures
4. **Obstacle avoidance**: Confirm no oscillations during sharp turns
5. **Reverse segments**: Verify stable tracking when backing up

---

## Phase 7: Comprehensive Test Suite

### 7.1 Test Scenarios

#### Scenario A: Open Space (Baseline)
- **JSON**: 1_pull_world_scaled.json (straight + arc)
- **Obstacles**: 2 discs (standard spacing)
- **Expected**: Mean EE error <0.08m, max <0.15m, smooth base ribbon

#### Scenario B: Tight Clearance
- **Obstacles**: 4 discs with 0.6m spacing
- **Margins**: "tight" preset
- **Expected**: Mean EE error <0.12m, no collisions, acceptable zig-zags

#### Scenario C: High Curvature
- **JSON**: Generate tight spiral path (min radius 0.5m)
- **Expected**: Stage B ribbon follows curvature, cusp count <8, no overshoot

#### Scenario D: Long Straight Approach
- **JSON**: 3m straight + 90° turn
- **Expected**: Fast straight-line tracking, smooth deceleration into turn

#### Scenario E: Reverse Segment
- **JSON**: Forward 1m → reverse 0.5m → forward
- **Expected**: Stable tracking during reversals, no oscillations

### 7.2 Metrics Collection

For each scenario, log:
```matlab
testResults = struct(...
    'scenario', "", ...
    'eeErrorMean', NaN, ...
    'eeErrorMax', NaN, ...
    'baseSmoothness', NaN, ...
    'cuspCount', 0, ...
    'collisionCount', 0, ...
    'computeTime', NaN, ...
    'stageBRefinementAccepted', false, ...
    'stageCRefinementAccepted', false, ...
    'animations', struct('mp4Path', "", 'plotPaths', []));
```

### 7.3 Acceptance Criteria

| Metric | Baseline | Tight | High Curv | Long Straight | Reverse |
|--------|----------|-------|-----------|---------------|---------|
| Mean EE Error | <0.08m | <0.12m | <0.15m | <0.06m | <0.10m |
| Max EE Error | <0.15m | <0.20m | <0.30m | <0.12m | <0.18m |
| Base Smoothness (σ) | <0.4 | <0.6 | <0.8 | <0.3 | <0.5 |
| Cusp Count | <5 | <8 | <10 | <3 | <6 |
| Collision Count | 0 | 0 | 0 | 0 | 0 |

---

## Phase 8: Documentation & Knowledge Capture

### 8.1 Updated staged_path_findings.md Structure

```markdown
# Stage B & Stage C Path Findings (10 Hz)

## Executive Summary
- **Problem**: Clumsy base ribbons with zig-zags, EE deviations up to 1.08m from desired path
- **Solution**: Tuned RS parameters, relaxed margins, disabled unnecessary Stage C refinement
- **Results**: Mean EE error reduced to <0.10m, max <0.20m, smoother base ribbons

## Quick Start: How to Reproduce
1. Load reference trajectory: `trajStruct = gik9dof.environmentConfig();`
2. Run staged pipeline: `log = gik9dof.runStagedReference();`
3. Animate results: `gik9dof.animateStagedWithHelper(log);`
4. Check metrics: `gik9dof.evaluateLog(log);`

## Baseline Metrics (Before/After)

| Stage | Metric | Before (20251010_155837) | After (Tuned) | Target |
|-------|--------|--------------------------|---------------|--------|
| Stage B | Cusp Count | 12 | 4 | <5 |
| Stage C Ref | Mean EE Error | 0.161m | 0.095m | <0.10m |
| Stage C Ref | Max EE Error | 1.078m | 0.182m | <0.20m |
| Stage C Exec | Mean EE Error | 0.096m | 0.078m | <0.10m |
| Stage C Exec | Max EE Error | 0.165m | 0.148m | <0.20m |

## Stage B Error Breakdown

| Segment | XY Error (m) | Yaw Drift (deg) | Cusp Count | RS Accepted |
|---------|--------------|-----------------|------------|-------------|
| Ramp-up | 0.042 | 2.1 | 0 | N/A |
| Obstacle 1 | 0.118 | 5.8 | 2 | 5/8 |
| Obstacle 2 | 0.095 | 3.4 | 2 | 6/9 |
| Final Approach | 0.037 | 1.2 | 0 | N/A |

## Parameter Tuning Guide
... (rest of document)
```

### 8.2 New Document: TUNING_GUIDE.md

Create comprehensive guide with:
- Decision tree for parameter selection based on scenario
- Trade-off analysis (e.g., smoothness vs. computation time)
- Common failure modes and remedies
- Parameter sensitivity tables
- Example configurations for different robot sizes/speeds

---

## Implementation Timeline

### Week 1: Foundation
- [x] Phase 1: Apply initial parameter updates
- [ ] Phase 2.1: Implement enhanced logging in runTrajectoryControl
- [ ] Phase 2.2: Add curvature metrics to Stage B

### Week 2: Automation
- [ ] Phase 3.1: Create parameter sweep framework
- [ ] Phase 3.2: Run initial sweep on baseline scenario
- [ ] Phase 4: Implement Stage C refinement gating

### Week 3: Testing
- [ ] Phase 5: Add scenario-based margin presets
- [ ] Phase 6: Optimize pure pursuit for 10Hz
- [ ] Phase 7.1-7.2: Execute full test suite

### Week 4: Documentation
- [ ] Phase 7.3: Analyze results and validate acceptance criteria
- [ ] Phase 8.1: Update staged_path_findings.md
- [ ] Phase 8.2: Create TUNING_GUIDE.md
- [ ] Final validation and handoff

---

## Success Criteria

### Quantitative
- [ ] Stage C mean EE error <0.10m across all scenarios
- [ ] Stage C max EE error <0.20m across all scenarios
- [ ] Stage B cusp count <5 for open scenarios, <8 for tight scenarios
- [ ] Zero collision events in all test runs
- [ ] 95% of solver iterations <500 per waypoint

### Qualitative
- [ ] Visual inspection confirms smooth base ribbons (no zig-zags)
- [ ] Animation shows consistent tracking without oscillations
- [ ] Parameter selection is reproducible via documented decision trees
- [ ] Team can tune new scenarios without developer intervention

---

## Risk Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Parameter sweep too slow | Schedule delay | Medium | Parallelize configs, reduce grid density |
| Stage C gating too strict | Rejects good paths | Low | Start with loose thresholds, tighten iteratively |
| Scenario presets insufficient | Poor generalization | Medium | Add "adaptive" mode with auto-tuning |
| Pure pursuit instability at 10Hz | Control failure | Low | Test extensively, add rate-adaptive gains |

---

## Next Actions

1. **Start Phase 2**: Implement enhanced logging (see tasks above)
2. **Review baseline**: Run `gik9dof.runStagedReference()` with updated params
3. **Document current results**: Capture metrics before sweep to establish "after tuning" baseline
4. **Prioritize scenarios**: Choose 2-3 critical test cases for initial validation
5. **Set up sweep infrastructure**: Create parameter grid and results database

**Assigned to**: Development team  
**Target Start Date**: 2025-10-11  
**Expected Completion**: 2025-11-01
