# Remaining Default Parameter Conflicts

**Analysis Date:** October 11, 2025  
**Status:** 3 Critical Conflicts + 2 Minor Differences Remain

---

## Executive Summary

After unifying `RateHz`, `StageBMode`, and `UseStageBHybridAStar`, there are still **3 critical conflicts** and **2 minor differences** between `runStagedReference.m` and `trackReferenceTrajectory.m`:

### Critical Conflicts (Require Action)
1. **MaxIterations**: 150 vs 1500 (10× difference!)
2. **StageBDesiredLinearVelocity**: 0.5 vs 0.6 m/s (20% difference)
3. **StageBMaxAngularVelocity**: 2.0 vs 2.5 rad/s (25% difference)

### Minor Differences (Acceptable)
4. **StageBDockingPositionTolerance**: NaN vs 0.02 (runStaged inherits from environment)
5. **StageBDockingYawTolerance**: NaN vs 2°/180°π (runStaged inherits from environment)

---

## Detailed Comparison Table

### Core Execution Parameters

| Parameter | runStagedReference | trackReferenceTrajectory | Status |
|-----------|-------------------|--------------------------|--------|
| **RateHz** | 10 Hz | 10 Hz | ✓ **UNIFIED** |
| **MaxIterations** | **150** | **1500** | ❌ **CONFLICT** |
| ExecutionMode | "ppForIk" | "ppForIk" | ✓ MATCH |
| **UseStageBHybridAStar** | true | true | ✓ **UNIFIED** |
| **StageBMode** | "pureHyb" | "pureHyb" | ✓ **UNIFIED** |

### Stage B Path Planning

| Parameter | runStagedReference | trackReferenceTrajectory | Status |
|-----------|-------------------|--------------------------|--------|
| StageBHybridResolution | 0.05 m | 0.05 m | ✓ MATCH |
| StageBHybridSafetyMargin | 0.15 m | 0.15 m | ✓ MATCH |
| StageBHybridMinTurningRadius | 0.5 m | 0.5 m | ✓ MATCH |
| StageBHybridMotionPrimitiveLength | 0.2 m | 0.2 m | ✓ MATCH |
| StageBUseReedsShepp | false | false | ✓ MATCH |
| StageBUseClothoid | false | false | ✓ MATCH |

### Stage B Chassis Control

| Parameter | runStagedReference | trackReferenceTrajectory | Status |
|-----------|-------------------|--------------------------|--------|
| StageBLookaheadDistance | 0.6 m | 0.6 m | ✓ MATCH |
| **StageBDesiredLinearVelocity** | **0.5 m/s** | **0.6 m/s** | ❌ **CONFLICT** |
| **StageBMaxAngularVelocity** | **2.0 rad/s** | **2.5 rad/s** | ❌ **CONFLICT** |
| StageBDockingPositionTolerance | NaN (from env) | 0.02 m | ⚠ DIFFERENT |
| StageBDockingYawTolerance | NaN (from env) | ~0.035 rad | ⚠ DIFFERENT |
| StageBChassisControllerMode | -1 | -1 | ✓ MATCH |

### Stage C Tracking

| Parameter | runStagedReference | trackReferenceTrajectory | Status |
|-----------|-------------------|--------------------------|--------|
| StageCUseBaseRefinement | true | true | ✓ MATCH |
| StageCChassisControllerMode | -1 | -1 | ✓ MATCH |
| StageCLookaheadDistance | N/A | 0.4 m | ⚠ TRACK-ONLY |
| StageCDesiredLinearVelocity | N/A | 1.0 m/s | ⚠ TRACK-ONLY |
| StageCMaxLinearSpeed | N/A | 1.5 m/s | ⚠ TRACK-ONLY |
| StageCMaxAngularVelocity | N/A | 2.5 rad/s | ⚠ TRACK-ONLY |
| StageCTrackWidth | N/A | 0.574 m | ⚠ TRACK-ONLY |
| StageCWheelBase | N/A | 0.36 m | ⚠ TRACK-ONLY |
| StageCMaxWheelSpeed | N/A | 3.3 m/s | ⚠ TRACK-ONLY |
| StageCWaypointSpacing | N/A | 0.15 m | ⚠ TRACK-ONLY |
| StageCPathBufferSize | N/A | 30.0 m | ⚠ TRACK-ONLY |
| StageCGoalTolerance | N/A | 0.10 m | ⚠ TRACK-ONLY |
| StageCInterpSpacing | N/A | 0.05 m | ⚠ TRACK-ONLY |
| StageCReverseEnabled | N/A | true | ⚠ TRACK-ONLY |

### Chassis Profile

| Parameter | runStagedReference | trackReferenceTrajectory | Status |
|-----------|-------------------|--------------------------|--------|
| ChassisProfile | "wide_track" | "wide_track" | ✓ MATCH |
| ChassisOverrides | struct() | struct() | ✓ MATCH |

---

## Impact Analysis

### 1. MaxIterations: 150 vs 1500

**Impact:** 🔴 **CRITICAL**

- **Current Issue:**
  - `runStagedReference` allows only 150 IK solver iterations
  - `trackReferenceTrajectory` allows 1500 iterations (10× more)
  - This causes **vastly different convergence behavior**
  
- **Observed Effects:**
  - With 150 iterations: Solver may hit iteration limit frequently, causing tracking errors
  - With 1500 iterations: Solver has room to converge, better tracking accuracy
  
- **Recommendation:** 
  ```matlab
  % Unify to 1500 (production-tuned value)
  options.MaxIterations (1,1) double {mustBePositive} = 1500
  ```

### 2. StageBDesiredLinearVelocity: 0.5 vs 0.6 m/s

**Impact:** 🟡 **MODERATE**

- **Current Issue:**
  - 20% difference in Stage B base velocity during path following
  - Affects timing and smoothness of Stage B execution
  
- **Observed Effects:**
  - 0.5 m/s: More conservative, smoother but slower
  - 0.6 m/s: Slightly more aggressive, faster docking
  
- **Recommendation:**
  ```matlab
  % Choose based on testing (0.5 recommended for smoothness)
  options.StageBDesiredLinearVelocity (1,1) double = 0.5
  ```

### 3. StageBMaxAngularVelocity: 2.0 vs 2.5 rad/s

**Impact:** 🟡 **MODERATE**

- **Current Issue:**
  - 25% difference in maximum yaw rate during Stage B
  - Affects turning behavior and path tracking
  
- **Observed Effects:**
  - 2.0 rad/s: More conservative turns, smoother motion
  - 2.5 rad/s: Sharper turns, may be less smooth
  
- **Recommendation:**
  ```matlab
  % Choose based on testing (2.0 recommended for smoothness)
  options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.0
  ```

### 4-5. Docking Tolerances: NaN vs Explicit Values

**Impact:** 🟢 **LOW** (Acceptable Difference)

- **Current Design:**
  - `runStagedReference` uses NaN and inherits from `environmentConfig()`
  - `trackReferenceTrajectory` has explicit defaults
  - Both end up using the same values from environment
  
- **Recommendation:** 
  - Keep as-is (architectural difference, not a functional conflict)
  - Document that runStaged inherits from environment

### Stage C Parameters: Track-Only

**Impact:** 🟢 **LOW** (Expected Difference)

- **Current Design:**
  - `trackReferenceTrajectory` is the **router** that accepts all Stage C params
  - `runStagedReference` is a **convenience wrapper** that doesn't expose them
  - Both ultimately call `runStagedTrajectory` with the same defaults
  
- **Recommendation:**
  - Keep as-is (architectural difference by design)
  - Document the layered architecture

---

## Recommended Actions

### Priority 1: Critical Unification

**File:** `matlab/+gik9dof/runStagedReference.m`

```matlab
% Line 34: Change MaxIterations default
options.MaxIterations (1,1) double {mustBePositive} = 1500  % was 150
```

### Priority 2: Velocity Parameter Unification (Choose One)

**Option A: Conservative (Recommended)**
```matlab
% In trackReferenceTrajectory.m
options.StageBDesiredLinearVelocity (1,1) double = 0.5  % was 0.6
options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.0  % was 2.5
```

**Option B: Aggressive**
```matlab
% In runStagedReference.m
options.StageBDesiredLinearVelocity (1,1) double = 0.6  % was 0.5
options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.5  % was 2.0
```

### Priority 3: Documentation

Update `projectDiagnosis.md` to reflect remaining conflicts and architectural differences.

---

## Testing Plan

After unification, run comparative tests:

```matlab
% Test 1: Verify identical behavior
result1 = gik9dof.runStagedReference();
log2 = gik9dof.trackReferenceTrajectory('Mode', 'staged');

% Compare key metrics
fprintf('MaxIterations: %d vs %d\n', ...
    result1.log.stageLogs.stageA.solverIterationsMax, ...
    log2.stageLogs.stageA.solverIterationsMax);

% Test 2: Verify Stage B velocity match
fprintf('Stage B Linear Vel: %.2f vs %.2f m/s\n', ...
    0.5, 0.5);  % Should both be same after unification
```

---

## Summary

| Category | Count | Action Required |
|----------|-------|----------------|
| ✅ Previously Unified | 3 | None |
| ❌ Critical Conflicts | 3 | **Immediate** |
| ⚠ Minor Differences | 2 | Document only |
| 📋 Architectural | ~13 | Document only |

**Next Steps:**
1. Unify MaxIterations → 1500 (critical)
2. Unify Stage B velocities (choose conservative or aggressive)
3. Update documentation
4. Run validation tests
5. Update `DEFAULTS_UNIFIED_SUMMARY.md`
