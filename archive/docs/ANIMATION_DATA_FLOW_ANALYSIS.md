# Animation System: Data Flow and Asset Analysis

**Date:** October 12, 2025  
**Purpose:** Comprehensive inspection of animation data sources, time management, and EE trajectory discrepancy

---

## Executive Summary

### Critical Finding: Potential Data Source Confusion ⚠️

**Observed Issue:**
- **Actual robot EE** follows **desired EE trajectory** (JSON) nicely ✅
- **Reference EE trajectory** shows large deviation from JSON source ❌
- **Concern:** Are we visualizing IDEAL tracking or ACTUAL simulated motion?

**Root Cause Analysis:**
The animation system has **multiple EE trajectory sources** with unclear precedence, potentially showing reference (ideal) paths instead of actual (simulated) paths.

---

## Animation Architecture Overview

### Main Components

```
┌─────────────────────────────────────────────────────────────┐
│                    ANIMATION PIPELINE                         │
└────────────────────────────────┬────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Data Preparation Layer                                   │
│     animateStagedWithHelper.m                                │
│     - Extracts qTraj from log                                │
│     - Separates base (x,y,θ) and arm joints                  │
│     - Resolves EE trajectory sources                         │
│     - Applies sampling                                       │
└────────────────────────────────┬────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────┐
│  2. Rendering Layer                                          │
│     viz.animate_whole_body.m                                 │
│     - Interpolates base onto arm timeline                    │
│     - Performs forward kinematics on qTraj                   │
│     - Renders robot meshes                                   │
│     - Overlays reference paths                               │
└────────────────────────────────┬────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Visual Output                                            │
│     - Dual view (3D perspective + top view)                  │
│     - Multiple trajectory overlays                           │
│     - Real-time FK computation                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Data Sources for Animation

### 1. **Primary Data: qTraj (Joint Trajectory)**

**Source:** `log.qTraj` [9×N] matrix  
**Content:** ACTUAL joint angles from simulation  
**Usage:** **THIS IS WHAT THE ROBOT ACTUALLY DID**

```matlab
% From log structure
qTraj = log.qTraj;              % [9×N] ACTUAL joint trajectory
baseIdx = [1, 2, 3];            % joint_x, joint_y, joint_theta
armIdx = [4, 5, 6, 7, 8, 9];    % left_arm_joint1-6

% Extracted for animation
basePose = qTraj(baseIdx, :)';  % [N×3] ACTUAL base states
armTrajectory = qTraj(armIdx, :)'; % [N×6] ACTUAL arm joints
```

**✅ This is correct - we're using ACTUAL simulation results**

---

### 2. **EE Trajectory Sources (Multiple Overlays)**

The animation system supports **multiple EE trajectory overlays**, which is where confusion arises:

#### a) **Actual EE Path** (Green, computed in real-time)

```matlab
% In animate_whole_body.m lines 441-456
% Computed DURING animation loop via forward kinematics
for k = 1:numSteps
    % Apply arm joints to robot
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    
    % Compute base transform
    Tbase = trvec2tform([baseX(k), baseY(k), 0]) * axang2tform([0 0 1 baseYaw(k)]);
    
    % Get EE transform via FK
    Tee = getTransform(robot, config, eeName);
    TeeWorld = Tbase * Tee;  % Transform to world frame
    
    actualEE(k, :) = TeeWorld(1:3,4)';  // ✅ ACTUAL EE position
end
```

**Source:** Forward kinematics on `qTraj`  
**This is CORRECT** - Shows where the robot EE actually is given the simulated joint angles.

---

#### b) **Desired EE Path** (White dashed, from JSON)

```matlab
% In animateStagedWithHelper.m lines 33-40
if isfield(logStaged, 'referenceTrajectory') && ...
   isfield(logStaged.referenceTrajectory, 'EndEffectorPositions') && ...
   ~isempty(logStaged.referenceTrajectory.EndEffectorPositions)
    eePathDesired = logStaged.referenceTrajectory.EndEffectorPositions;
end

helperOptions.DesiredEEPath = eePathDesired.';  % Pass to animator
```

**Source:** `1_pull_world_scaled.json` (148 waypoints)  
**This is the GOAL** - What we want the EE to track.

---

#### c) **Stage C Reference EE Path** (Magenta dash-dot, RED DOT marker)

⚠️ **THIS IS WHERE THE PROBLEM LIES**

```matlab
% In animateStagedWithHelper.m lines 42-65
eePathStageCRef = [];

if isfield(logStaged, 'stageLogs') && ...
   isfield(logStaged.stageLogs, 'stageC')
    stageC = logStaged.stageLogs.stageC;
    
    // Priority 1: referenceInitialIk.eePositions (PASS 1 REFERENCE)
    if isfield(stageC, 'referenceInitialIk') && ...
       isfield(stageC.referenceInitialIk, 'eePositions') && ...
       ~isempty(stageC.referenceInitialIk.eePositions)
        eePathStageCRef = stageC.referenceInitialIk.eePositions;  // ⚠️ PROBLEM!
    
    // Priority 2: targetPositions (DESIRED from JSON)
    elseif isfield(stageC, 'targetPositions') && ...
           ~isempty(stageC.targetPositions)
        eePathStageCRef = stageC.targetPositions;
    
    // Priority 3: eePositions (ACTUAL Stage C EE)
    elseif isfield(stageC, 'eePositions') && ...
           ~isempty(stageC.eePositions)
        eePathStageCRef = stageC.eePositions;  // ✅ This is correct
    end
end

helperOptions.StageCReferenceEEPath = eePoses;  // Pass to red dot marker
```

**🔴 CRITICAL ISSUE IDENTIFIED:**

**Priority 1** uses `referenceInitialIk.eePositions` which is the **PASS 1 REFERENCE** from ppForIk three-pass architecture. This is the IDEAL trajectory before chassis simulation, NOT the actual executed trajectory!

---

## The Discrepancy Explained

### Stage C ppForIk Three-Pass Architecture

```
Pass 1: Reference IK (bundleRef)
  ├─ Full-body IK with free base
  ├─ Gets IDEAL base trajectory (may not be feasible)
  └─ Produces: logRef.qTraj, referenceBaseStates, referenceInitialIk.eePositions
     ⚠️ This EE path assumes perfect tracking (no chassis constraints)

Pass 2: Chassis Simulation
  ├─ Takes referenceBaseStates from Pass 1
  ├─ Simulates pure pursuit controller
  ├─ Applies velocity limits, acceleration limits
  └─ Produces: executedBase (REALISTIC base trajectory)
     ✅ This accounts for chassis dynamics

Pass 3: Final IK (bundleFinal)
  ├─ Base locked to executedBase (Pass 2)
  ├─ Arm solves IK to track EE
  └─ Produces: log.qTraj (FINAL simulation result)
     ✅ This is what the robot ACTUALLY did
```

### What You're Seeing

**Green "Actual EE Path":**
- Computed from `log.qTraj` (Pass 3 final result)
- Includes effects of chassis constraints
- **This is CORRECT** ✅

**White "Desired EE Path":**
- From JSON source (`1_pull_world_scaled.json`)
- The goal trajectory
- **This is CORRECT** ✅

**Magenta "Stage C Reference EE Path" + Red Dot:**
- Currently showing `referenceInitialIk.eePositions` (Pass 1)
- This is the IDEAL trajectory WITHOUT chassis constraints
- **This is MISLEADING** ❌

### Why There's a Discrepancy

The red dot (Stage C reference) is showing the **IDEAL Pass 1** EE path, which assumes:
- Perfect base tracking
- No velocity/acceleration limits
- No pure pursuit deviations

The green line (actual EE) is showing the **REALISTIC Pass 3** EE path, which reflects:
- Actual chassis dynamics
- Pure pursuit tracking errors
- Velocity/acceleration constraints

**The deviation you see is EXPECTED** - it's the difference between ideal and realistic motion!

---

## Time Management in Animation

### Time Vector Construction

```matlab
% In animateStagedWithHelper.m lines 168-178
function t = resolveTimes(logStruct, numSamples)
    % Priority 1: Explicit time field
    if isfield(logStruct, 'time') && numel(logStruct.time) == numSamples
        t = logStruct.time;
    
    % Priority 2: Timestamps field (N-1 samples)
    elseif isfield(logStruct, 'timestamps') && ...
           numel(logStruct.timestamps) == numSamples - 1
        t = [0, logStruct.timestamps];
    
    % Priority 3: Compute from rate
    elseif isfield(logStruct, 'rateHz') && logStruct.rateHz > 0
        t = (0:numSamples-1) / logStruct.rateHz;
    
    % Fallback: Frame indices
    else
        t = 0:numSamples-1;
    end
end
```

### Time Synchronization

```matlab
% In animate_whole_body.m lines 225-228
% Interpolate base pose onto arm timeline
baseX = interp1(baseTimes, basePose(:,1), armTimes, 'linear', 'extrap');
baseY = interp1(baseTimes, basePose(:,2), armTimes, 'linear', 'extrap');
baseYawUnwrapped = interp1(baseTimes, unwrap(basePose(:,3)), armTimes, 'linear', 'extrap');
baseYaw = wrapToPi(baseYawUnwrapped);
```

**✅ Time management is CORRECT:**
- Base and arm use same timeline (both from log.timestamps)
- Interpolation ensures synchronization
- Unwrap/wrap prevents yaw discontinuities

---

## Sampling Adjustments

### Frame Sampling

```matlab
% In animateStagedWithHelper.m lines 27-30
armTrajectory = armTrajectory(1:opts.SampleStep:end, :);
basePose = basePose(1:opts.SampleStep:end, :);
t = t(1:opts.SampleStep:end);
```

### Stage Boundary Sampling

```matlab
% In animateStagedWithHelper.m lines 143-151
if opts.SampleStep > 1 && ~isempty(stageBoundaries)
    % Convert cumulative boundaries to per-stage counts
    stageCounts = [stageBoundaries(1), diff(stageBoundaries)];
    
    % Apply sampling to each stage count
    sampledCounts = arrayfun(@(c) length(1:opts.SampleStep:c), stageCounts);
    
    // Reconstruct cumulative boundaries in sampled space
    stageBoundaries = cumsum(sampledCounts);
end
```

**✅ Sampling is CORRECT** - boundaries adjusted for downsampling

---

## Rendering Assets Used

### 1. **Robot Meshes**

```matlab
% In animate_whole_body.m lines 337-380
meshPath = "";
chassisCandidates = {
    fullfile(meshOutputs, 'base_link_reduced.STL');
    fullfile(meshOutputs, 'base_link_reduced.stl');
    fullfile(meshRoot, 'base_link.STL')
};

% Arm meshes
meshDir = fullfile(matlabDir, '..', '..', 'meshes');
candidates = {
    fullfile(outputsDir, [bodyName, '_reduced.STL']),
    fullfile(outputsDir, [bodyName, '_reduced.stl']),
    fullfile(outputsDir, [bodyName, '.STL']),
    fullfile(outputsDir, [bodyName, '.stl'])
};
```

**Assets:**
- `meshes/outputs/base_link_reduced.STL` (chassis)
- `meshes/stl_output/*_reduced.STL` (arm links)
- `meshes/left_arm_joint*.STL` (fallback)

---

### 2. **Path Overlays**

| Path Type | Color | Style | Source | Correctness |
|-----------|-------|-------|--------|-------------|
| **Executed Base** | Blue | Solid | `log.qTraj(baseIdx, :)` | ✅ ACTUAL |
| **Reference Base** | Yellow | Dashed | `stageC.referenceBaseStates` | ✅ Pass 1 ideal |
| **Stage B Base** | Gray | Dotted | `stageB.execBaseStates` | ✅ ACTUAL |
| **Desired EE** | White | Dashed | JSON waypoints | ✅ GOAL |
| **Stage C Ref EE** | Magenta | Dash-dot | `referenceInitialIk.eePositions` | ⚠️ Pass 1 ideal |
| **Actual EE** | Green | Solid | FK on `log.qTraj` | ✅ ACTUAL |

---

### 3. **Markers**

```matlab
% Base marker (blue circle)
baseMarkerPersp = plot3(axPersp, baseX(1), baseY(1), 0, 'o', ...
    'Color', executedColor, 'MarkerFaceColor', executedColor);

% Heading arrow (blue line)
headingLinePersp = plot3(axPersp, [baseX, baseX+headingLen*cos(baseYaw)], ...
                         [baseY, baseY+headingLen*sin(baseYaw)], [0 0], '-', ...
                         'Color', executedColor, 'LineWidth', 2);

// Red dot for Stage C reference EE (PROBLEM HERE)
if ~isempty(stageCPath)
    eeMarkerPersp = plot3(axPersp, stageCPath(1,1), stageCPath(1,2), stageCPath(1,3), ...
                          'ro', 'MarkerFaceColor', 'r', ...
                          'DisplayName', 'Stage C reference EE waypoint');
    // ⚠️ stageCPath = referenceInitialIk.eePositions (Pass 1, not Pass 3!)
end

// Green square for actual EE (CORRECT)
actualMarkerPersp = plot3(axPersp, baseX(1), baseY(1), 0, 's', ...
                          'Color', [0.0 0.6 0.2], 'MarkerFaceColor', [0.0 0.6 0.2], ...
                          'DisplayName', 'Actual EE waypoint');
```

---

## The Fix: What Should Be Visualized

### Current Priority Order (INCORRECT ❌)

```matlab
// Priority 1: referenceInitialIk.eePositions (Pass 1 - IDEAL)
if isfield(stageC, 'referenceInitialIk') && ...
   isfield(stageC.referenceInitialIk, 'eePositions')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;  // ❌ WRONG!
```

### Recommended Priority Order (CORRECT ✅)

```matlab
// Priority 1: eePositions (ACTUAL Stage C EE from Pass 3)
if isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
    eePathStageCRef = stageC.eePositions;  // ✅ This is ACTUAL

// Priority 2: targetPositions (DESIRED from JSON)
elseif isfield(stageC, 'targetPositions') && ~isempty(stageC.targetPositions)
    eePathStageCRef = stageC.targetPositions;  // ✅ This is GOAL

// Priority 3: referenceInitialIk.eePositions (Pass 1 - for debugging only)
elseif isfield(stageC, 'referenceInitialIk') && ...
       isfield(stageC.referenceInitialIk, 'eePositions')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;  // Only for debug
```

---

## Verification: What Data is Logged

### Stage C Log Structure (ppForIk mode)

```matlab
logC = struct(
    'qTraj',            % [9×NC] ACTUAL joint angles (Pass 3 result) ✅
    'timestamps',       % [1×NC] time stamps ✅
    'eePositions',      % [3×NC] ACTUAL EE positions (FK on Pass 3) ✅
    'targetPositions',  % [3×NC] DESIRED EE (from JSON) ✅
    
    // Pure pursuit data
    'purePursuit', struct(
        'referencePath',    % [N×3] Pass 1 ideal base ✅
        'executedPath',     % [N×3] Pass 2 realistic base ✅
        'simulation',       % Full simulation results ✅
    ),
    
    // Reference IK data (Pass 1)
    'referenceInitialIk', struct(
        'qTraj',           % [9×N] Pass 1 joint angles (IDEAL) ⚠️
        'eePositions',     % [3×N] Pass 1 EE positions (IDEAL) ⚠️
    ),
    
    'referenceBaseStates',  % [N×3] Pass 1 base (IDEAL) ⚠️
    'execBaseStates',       % [N×3] Pass 2 base (REALISTIC) ✅
);
```

### What Should Be Visualized

| Component | Should Show | Currently Shows | Status |
|-----------|-------------|-----------------|--------|
| **Robot body** | `log.qTraj` (Pass 3) | `log.qTraj` | ✅ CORRECT |
| **Green EE path** | FK on `log.qTraj` | FK on `log.qTraj` | ✅ CORRECT |
| **White EE path** | JSON waypoints | JSON waypoints | ✅ CORRECT |
| **Red EE dot** | `stageC.eePositions` (Pass 3) | `referenceInitialIk.eePositions` (Pass 1) | ❌ WRONG |
| **Magenta EE path** | `stageC.eePositions` (Pass 3) | `referenceInitialIk.eePositions` (Pass 1) | ❌ WRONG |
| **Blue base path** | `log.qTraj(baseIdx)` | `log.qTraj(baseIdx)` | ✅ CORRECT |
| **Yellow base path** | `stageC.referenceBaseStates` (Pass 1) | `stageC.referenceBaseStates` | ✅ CORRECT (for reference) |

---

## Recommendations

### 1. **Fix EE Reference Priority** (High Priority) 🔴

**File:** `matlab/+gik9dof/animateStagedWithHelper.m` (lines 42-65)

**Change:**
```matlab
// OLD (WRONG):
if isfield(stageC, 'referenceInitialIk') && ...
   isfield(stageC.referenceInitialIk, 'eePositions')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;  // Pass 1 IDEAL

// NEW (CORRECT):
if isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
    eePathStageCRef = stageC.eePositions;  // Pass 3 ACTUAL ✅
```

**Rationale:** The red dot should track the ACTUAL EE during Stage C, not the ideal reference from Pass 1.

---

### 2. **Add Debug Option for Pass 1 Visualization** (Medium Priority)

Add option to show Pass 1 ideal trajectory for debugging:

```matlab
addParameter(parser, 'ShowPass1Reference', false, @islogical);

if opts.ShowPass1Reference && isfield(stageC, 'referenceInitialIk')
    helperOptions.Pass1ReferenceEEPath = stageC.referenceInitialIk.eePositions;
    helperOptions.Pass1ReferenceLabel = "Stage C Pass 1 (Ideal, before chassis sim)";
end
```

---

### 3. **Update Legend Labels** (Low Priority)

Current labels are ambiguous:

```matlab
// Current:
'Stage C reference EE path'  // Ambiguous - reference to what?

// Suggested:
'Stage C actual EE path'     // Clear - this is what was executed
'Stage C ideal EE path (Pass 1)'  // Clear - for debugging only
```

---

### 4. **Add EE Tracking Error Metric** (Medium Priority)

Current error calculation is confusing:

```matlab
// In animate_whole_body.m lines 457-469
if k <= size(stageCFull,1) && all(~isnan(stageCFull(k,1:3)))
    desiredEE = stageCFull(k,1:3);  // This is Pass 1 ideal, not JSON desired!
    errPos = norm(actualEE(k,:) - desiredEE);
```

**Should compute TWO errors:**
1. Tracking error: `actualEE` vs `JSON desired`
2. Chassis impact: `actualEE` vs `Pass 1 ideal`

---

## Summary Table: Data Correctness

| Data Item | Source | Represents | Animation Usage | Correct? |
|-----------|--------|------------|-----------------|----------|
| `log.qTraj` | Pass 3 final | ACTUAL robot motion | Robot body rendering | ✅ YES |
| `log.timestamps` | Simulation | Real time | Time synchronization | ✅ YES |
| `actualEE` (green) | FK on `log.qTraj` | ACTUAL EE position | Green path + square | ✅ YES |
| JSON waypoints (white) | Input file | DESIRED EE goal | White dashed path | ✅ YES |
| `referenceInitialIk.eePositions` | Pass 1 | IDEAL EE (no chassis) | Red dot + magenta path | ❌ NO - misleading |
| `stageC.eePositions` | Pass 3 | ACTUAL EE (Stage C) | NOT USED | ❌ Should be used! |
| `referenceBaseStates` | Pass 1 | IDEAL base path | Yellow dashed | ✅ YES (for comparison) |
| `execBaseStates` | Pass 2 | ACTUAL base path | Blue solid | ✅ YES |

---

## Conclusion

### The Discrepancy is REAL and MEANINGFUL

The deviation you observed between:
- **Green "Actual EE"** (follows JSON nicely) ✅
- **Magenta "Stage C Reference"** (shows large deviation) ❌

Is caused by:
1. **Correct**: Green uses Pass 3 actual trajectory (accounts for chassis)
2. **Incorrect**: Magenta uses Pass 1 ideal trajectory (ignores chassis)

The **large deviation** you see is the **impact of chassis dynamics** (velocity limits, pure pursuit tracking errors, acceleration constraints) on the base motion, which propagates to the EE position.

### Action Required

**Fix `animateStagedWithHelper.m` line 48-50** to prioritize `stageC.eePositions` (actual) over `referenceInitialIk.eePositions` (ideal).

This will make the red dot and magenta path show the ACTUAL Stage C EE trajectory, which should match the green line much more closely.

---

## See Also

- **Three-Pass Architecture:** `HOLISTIC_STAGEC_EQUIVALENCE.md`
- **Stage C ppForIk:** `projectDiagnosis.md` - Section "Stage C: ppForIk Mode"
- **Animation Code:** `matlab/+gik9dof/+viz/animate_whole_body.m`
- **Data Preparation:** `matlab/+gik9dof/animateStagedWithHelper.m`
