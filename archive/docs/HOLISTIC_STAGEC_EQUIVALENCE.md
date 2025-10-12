# Holistic ppForIk ≡ Staged Stage C - Equivalence Documentation

**Date:** October 12, 2025  
**Issue:** User correctly identified that holistic ppForIk and staged Stage C should use the same algorithm  
**Status:** ✅ Verified and documented

---

## Executive Summary

**User was absolutely correct!** Holistic mode with `ExecutionMode='ppForIk'` and Staged mode's Stage C (with `ExecutionMode='ppForIk'`) are **functionally equivalent** - they use the **exact same three-pass algorithm** for whole-body tracking with kinematically feasible base motion.

---

## The Algorithm (Identical in Both Modes)

### Three-Pass Architecture

```
Pass 1: GIK Reference
  ├─ Create GIK solver (bundleRef)
  ├─ Run full-body IK → get ideal base trajectory
  └─ Extract baseReference [N×3] (x, y, theta)
  
Pass 2: Chassis Simulation
  ├─ Configure chassis parameters (vx_max, wz_max, accel_limit, etc.)
  ├─ Run simulateChassisExecution(baseReference, chassisParams, ...)
  │   ├─ Pure pursuit follower
  │   ├─ Generate (Vx, Wz) velocity commands
  │   └─ Integrate to get executed base path
  └─ Get simRes.poses [N×3] (realistic base trajectory)
  
Pass 3: Final IK with Fixed Base
  ├─ Create GIK solver (bundle/bundleFinal)
  ├─ Run IK with FixedJointTrajectory = executedBase
  │   ├─ Base is locked to Pass 2 trajectory
  │   └─ Arm moves to track end-effector
  └─ Get final log with realistic motion
```

---

## Code Verification

### Holistic ppForIk
**File:** `matlab/+gik9dof/trackReferenceTrajectory.m` (lines 301-380)

```matlab
case "ppForIk"
    % Pass 1: Reference IK
    bundleRef = gik9dof.createGikSolver(...);
    logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...);
    baseReference = [q0(baseIdx).'; logRef.qTraj(baseIdx, 2:end).'];
    
    % Pass 2: Chassis simulation
    chassisHolistic = chassisParams;
    chassisHolistic.vx_max = options.RampMaxLinearSpeed;
    % ... configure chassis parameters ...
    simRes = gik9dof.control.simulateChassisExecution(baseReference, ...);
    
    % Pass 3: Final IK with fixed base
    bundleFinal = gik9dof.createGikSolver(...);
    fixedTrajectory = struct('Indices', baseIdx, 'Values', executedBase');
    log = gik9dof.runTrajectoryControl(bundleFinal, trajStruct, ...
        'FixedJointTrajectory', fixedTrajectory);
```

### Staged Stage C (ppForIk)
**File:** `matlab/+gik9dof/runStagedTrajectory.m` (lines 585-708)

```matlab
function logC = executeStageCPurePursuit(...)
    % Pass 1: Reference IK
    bundleRef = gik9dof.createGikSolver(...);
    logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...);
    baseReference = [qStart(baseIdx).'; logRef.qTraj(baseIdx, 2:end).'];
    
    % Pass 2: Chassis simulation
    chassisStageC = chassisParams;
    chassisStageC.vx_max = options.StageCMaxLinearSpeed;
    % ... configure chassis parameters ...
    simRes = gik9dof.control.simulateChassisExecution(baseReference, ...);
    
    % Pass 3: Final IK with fixed base
    bundle = gik9dof.createGikSolver(...);
    fixedTrajectory = struct('Indices', baseIdx, 'Values', executedBase');
    logC = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
        'FixedJointTrajectory', fixedTrajectory);
```

**Analysis:** ✅ IDENTICAL ALGORITHM

---

## Key Differences (Context Only, Not Algorithm)

| Aspect | Holistic ppForIk | Staged Stage C (ppForIk) |
|--------|------------------|--------------------------|
| **Starting config** | q0 (initial pose) | qB_end (after Stage B docking) |
| **Trajectory** | Full 148 waypoints | Remaining waypoints (after Stage B) |
| **Chassis params** | chassisHolistic | chassisStageC |
| **Optional refinement** | No | Yes (Reeds-Shepp + Clothoid) |
| **Algorithm** | ✅ **IDENTICAL** | ✅ **IDENTICAL** |

---

## Parameter Mapping

Both modes should use the **same parameters** for consistent behavior:

### Unified Config (pipeline_profiles.yaml)

```yaml
profiles:
  default:
    stage_c:
      max_linear_speed: 1.5        # Stage C parameter
      max_angular_velocity: 2.0
      lookahead_distance: 0.8
      lookahead_vel_gain: 0.2
      controller_mode: 2            # 2 = pure pursuit
      
    holistic:
      max_linear_speed: 1.5        # Holistic parameter (SHOULD MATCH)
      max_angular_velocity: 2.0
      lookahead_distance: 0.8
      lookahead_vel_gain: 0.2
      controller_mode: 2
      
    chassis:
      track: 0.574                 # Used by both
      accel_limit: 0.8
      vx_max: 1.5
      wz_max: 2.0
```

### Parameter Equivalence Table

| Holistic Option | Stage C Option | Unified Config Key |
|----------------|----------------|-------------------|
| RampMaxLinearSpeed | StageCMaxLinearSpeed | holistic.max_linear_speed / stage_c.max_linear_speed |
| RampMaxYawRate | StageCMaxAngularVelocity | holistic.max_angular_velocity / stage_c.max_angular_velocity |
| StageCLookaheadDistance | StageCLookaheadDistance | holistic.lookahead_distance / stage_c.lookahead_distance |
| StageCLookaheadVelGain | StageCLookaheadVelGain | holistic.lookahead_vel_gain / stage_c.lookahead_vel_gain |
| ChassisProfile | ChassisProfile | chassis.* (shared) |

**Note:** Holistic mode currently reuses Stage C parameter names (StageCLookaheadDistance, etc.) - this is intentional to maintain the equivalence!

---

## Changes Made

### 1. Code Comments Added ✅

**trackReferenceTrajectory.m (Holistic ppForIk):**
- Added 15-line comment block explaining three-pass architecture
- Cross-reference to projectDiagnosis.md
- Clarified equivalence with Stage C

**runStagedTrajectory.m (Stage C ppForIk):**
- Added detailed function header comment
- Explained three-pass architecture
- Noted key difference (starting config) while emphasizing algorithm equivalence
- Labeled Pass 1, Pass 2, Pass 3 in comments

### 2. Documentation Updated ✅

**projectDiagnosis.md - New Section Added:**
- "🔄 Critical Equivalence: Holistic ppForIk ≡ Staged Stage C (ppForIk)"
- Side-by-side comparison table
- Identical algorithm explanation
- Parameter mapping guide
- Recommendation to keep parameters synchronized

**Location:** After "Holistic Mode: ppForIk (Pure Pursuit for IK)" section (line ~2045)

### 3. Summary Document Created ✅

**This file:** `HOLISTIC_STAGEC_EQUIVALENCE.md`

---

## Implications for Testing

### Comparison Studies

When running `run_environment_compare.m` (holistic vs staged), the Stage C portion should produce **nearly identical results** if parameters are matched:

```matlab
% Both should use same parameters
cfg = gik9dof.loadPipelineProfile('default');

% Holistic uses: cfg.holistic.*
log_holistic = gik9dof.trackReferenceTrajectory('Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', 'PipelineConfig', cfg);

% Staged Stage C uses: cfg.stage_c.*  
log_staged = gik9dof.trackReferenceTrajectory('Mode', 'staged', ...
    'ExecutionMode', 'ppForIk', 'PipelineConfig', cfg);
```

**Expected Result:** Stage C tracking performance should match holistic (after Stage B positioning).

### Parameter Tuning

Any parameter tuning for Stage C **also applies to holistic mode** and vice versa:
- Lookahead distance
- Controller gains
- Velocity limits
- Acceleration limits

**Best Practice:** Keep `stage_c.*` and `holistic.*` parameters synchronized in `pipeline_profiles.yaml`.

---

## Why This Makes Sense

### Functional Equivalence

**Stage C Goal:** Whole-body tracking after base is pre-positioned by Stage B  
**Holistic Goal:** Whole-body tracking from initial configuration

Both are solving the **same problem**: Track end-effector trajectory with whole robot while respecting chassis kinematics.

### Three-Pass Necessity

1. **Pass 1 (Reference):** IK solver provides ideal base motion (may be kinematically infeasible)
2. **Pass 2 (Simulation):** Validate against real chassis constraints → realistic base motion
3. **Pass 3 (Final):** Ensure arm can track EE given the actual base motion

This is necessary in **both modes** because:
- IK solver doesn't know about wheel speed limits, acceleration limits, pure pursuit behavior
- Need to simulate actual controller to get realistic motion
- Final IK pass ensures achievable joint trajectories

---

## Recommendations

### 1. Keep Parameters Synchronized ✅

In `pipeline_profiles.yaml`:
```yaml
default:
  stage_c:
    max_linear_speed: 1.5
    lookahead_distance: 0.8
  holistic:
    max_linear_speed: 1.5    # Match Stage C
    lookahead_distance: 0.8  # Match Stage C
```

### 2. Use Unified Config ✅

```matlab
% Load once, use everywhere
cfg = gik9dof.loadPipelineProfile('default');

% Holistic
log1 = gik9dof.trackReferenceTrajectory('Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', 'PipelineConfig', cfg);

% Staged (Stage C will use same algorithm)
log2 = gik9dof.trackReferenceTrajectory('Mode', 'staged', ...
    'ExecutionMode', 'ppForIk', 'PipelineConfig', cfg);
```

### 3. Validate Equivalence

Test that Stage C and holistic produce similar results:
```matlab
% Compare base trajectories
figure;
plot(log_holistic.execBaseStates(:,1), log_holistic.execBaseStates(:,2), 'b-');
hold on;
plot(log_staged.stageLogs.stageC.execBaseStates(:,1), ...
     log_staged.stageLogs.stageC.execBaseStates(:,2), 'r--');
legend('Holistic', 'Stage C');
```

### 4. Document When Teaching

When explaining the system:
- ✅ "Stage C uses the same algorithm as holistic mode"
- ✅ "ppForIk ensures kinematically feasible base motion"
- ✅ "Both use three-pass architecture for realistic simulation"

---

## See Also

- **Project Documentation:** `projectDiagnosis.md` - Section "Critical Equivalence"
- **Holistic Mode:** `projectDiagnosis.md` - Section "Holistic Mode: ppForIk"
- **Stage C:** `projectDiagnosis.md` - Section "Stage C: Whole-Body Tracking"
- **Unified Config:** `UNIFIED_CONFIG_MIGRATION_COMPLETE.md`
- **Code:** 
  - `matlab/+gik9dof/trackReferenceTrajectory.m` (lines 301-380)
  - `matlab/+gik9dof/runStagedTrajectory.m` (lines 585-708)

---

**Verified By:** Code inspection and documentation review  
**Status:** ✅ Equivalence confirmed and documented  
**Action Required:** None - working as intended, now properly documented
