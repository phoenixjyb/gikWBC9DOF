# Phase 2A Debugging Session - October 14, 2025

## Problem Statement

Phase 2A (Orientation+Z nominal pose) works perfectly in isolated tests (1.2mm error) but fails catastrophically in staged runs (779mm error, second half: 1739mm).

## Investigation Summary

### Initial Hypothesis (INCORRECT)
- **Theory:** Sequential warm-starting from Stage B ending configuration causes drift
- **Evidence:** r=0.985 correlation between base path divergence and EE error
- **Conclusion:** This was a red herring - the correlation was real but not causal

### Second Hypothesis (PARTIALLY CORRECT)
- **Theory:** MaxIterations=1500 (staged) vs 2000 (isolated) causes failures
- **Evidence:** All waypoints 100-210 in staged run hit 1500 iteration limit
- **Test:** Ran isolated test with MaxIterations=1500
- **Result:** **HYPOTHESIS REJECTED** - Still achieved 2.3mm error in second half!

### Key Findings

#### 1. MaxIterations is NOT the Root Cause

**Test with MaxIterations=1500 (isolated):**
```
First Half (1-105):
  Mean Error: 0.0 mm ✅
  Convergence: 93.3%
  Hitting limit: 7 waypoints (6.7%)

Second Half (106-210):
  Mean Error: 2.3 mm ✅ (NOT 1739mm!)
  Convergence: 51.4%
  Hitting limit: 54 waypoints (51.4%)
```

**Key Observation:** Even when hitting the 1500 iteration limit, GIK still produces good solutions (~2mm error).

#### 2. Phase 1/2A Parameters ARE Being Passed Correctly

Verified in `runStagedTrajectory.m` lines 954-967:
```matlab
'UseAdaptiveLookahead', true, ...
'UseMicroSegment', true, ...
'UseWarmStarting', true, ...
'UseVelocityCorridor', true, ...
'LogLateralVelocity', true, ...
'RelaxedTolerances', true, ...
'UseOrientationZNominal', true, ...  % Phase 2A
```

All parameters are correctly passed to `runStageCPPFirst_enhanced`.

#### 3. Starting Configuration Difference

**Isolated test:**
- Base: [0.236, 0.062, -7.6°]
- Arm: [-1.087, 0.690, -2.082, 1.079, -1.410, -1.353] (home config)
- Result: 2.3mm second half ✅

**Staged run:**
- Base: [0.485, 0.153, -1.3°]
- Arm: [-1.813, 0.000, -0.976, -1.281, 0.996, 1.432] (Stage B ending)
- Result: 1739mm second half ❌

**Configuration Difference:**
- Base position: 0.266m
- Base heading: 6.3°
- **Arm joints: 4.62 rad** ← MAJOR DIFFERENCE!

## Current Leading Hypothesis

**The Stage B ending arm configuration is suboptimal for Stage C execution:**

1. **Reachability Issues:** Stage B leaves the arm in a configuration that makes Stage C waypoints difficult/impossible to reach
2. **Joint Limit Proximity:** Stage B ending might place joints near their limits
3. **Workspace Region Mismatch:** Stage B operates in different workspace region than Stage C requires
4. **IK Solution Basin:** Stage B ending is in a different solution basin, making Stage C IK convergence difficult

## Evidence Supporting Current Hypothesis

1. **Arm config norm difference:** 4.62 rad (very large!)
2. **GIK convergence pattern in staged run:**
   - First half: 79.0% (struggles but works)
   - Second half: 0.0% (complete failure)
3. **Same trajectory works from home:** test_stagec_only_home_start.m succeeds
4. **MaxIterations not the issue:** test_method4_phase2a_maxiter1500.m succeeds with 1500 iters

## Files Created During Investigation

### Test Scripts
- `test_stagec_only_home_start.m` - Stage C from home config (validates Phase 2A works)
- `test_method4_phase2a_maxiter1500.m` - Tests if MaxIterations=1500 causes failure

### Analysis Scripts
- `compare_execution_paths.m` - Comprehensive comparison of isolated vs staged
- `compare_parameters_detailed.m` - Deep parameter comparison
- `compare_base_paths.m` - Base path divergence analysis (r=0.985 correlation)
- `compare_isolated_vs_staged.m` - Initial comparison showing 636x error difference
- `compare_first_second_half.m` - First/second half split analysis
- `analyze_error_pattern.m` - Waypoint-by-waypoint error analysis
- `check_fix.m` - Quick error check script
- `check_mismatch.m` - Starting configuration mismatch analysis
- `verify_phase1_running.m` - Confirms Phase 1 improvements are active

### Documentation
- `PHASE2A_ROOT_CAUSE_CONFIRMED.md` - Initial (incorrect) diagnosis about initialization
- `PHASE2A_FIX_OPTION1.md` - Fix strategy document (now obsolete)
- `PHASE2A_INTEGRATION_DIAGNOSIS.md` - Stage B→C transition hypothesis (incorrect)
- `PHASE2A_ROOT_CAUSE_MAXITERATIONS.md` - MaxIterations hypothesis (disproven)
- `PHASE2A_DEBUGGING_SESSION_20251014.md` - This document

## Changes Made to Code

### 1. Added MaxIterations to run_fresh_sim_with_animation.m (Line 90)
```matlab
'MaxIterations', 2000, ...  % Phase 2A requires more iterations for tight orientation constraints
```
**Status:** This change can be kept but is NOT the fix for the problem.

### 2. Attempted fix in runStagedTrajectory.m (REVERTED)
```matlab
% Attempted to use home config for base seed path
qHomeForSeedPath = homeConfiguration(robot);
rawLog = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, qHomeForSeedPath, ...
```
**Status:** REVERTED - This caused the robot to teleport and didn't fix the problem.

## Test Results Summary

| Test | MaxIter | Start Config | First Half | Second Half | Status |
|------|---------|--------------|------------|-------------|--------|
| Isolated (original) | 2000 | Home | 0.0mm | 2.4mm | ✅ PASS |
| Staged run | 1500 | Stage B end | 0.0mm | 1739mm | ❌ FAIL |
| Isolated (iter1500) | 1500 | Home | 0.0mm | 2.3mm | ✅ PASS |
| Staged with fix | 2000 | Stage B end | 0.0mm | 1739mm | ❌ FAIL |

**Key Insight:** MaxIterations is NOT the differentiator. Starting configuration IS.

## Next Steps for Future Debugging

### Option 1: Investigate Stage B Ending Configuration
- Check if Stage B ending is near joint limits
- Analyze workspace region of Stage B vs Stage C
- Check IK solution quality at Stage B ending
- Compare reachability from Stage B end vs home

### Option 2: Improve Stage B to Stage C Transition
- Modify Stage B to end in a configuration more suitable for Stage C
- Add intermediate waypoint to transition from Stage B to Stage C smoothly
- Use different alignment strategy for Stage B ending

### Option 3: Add Configuration Reset at Stage C Start
- Accept discontinuity and reset to better starting configuration
- Use IK to find optimal starting config for Stage C first waypoint
- Blend from Stage B ending to optimal Stage C start

### Option 4: Analyze First Stage C Waypoint
- Check if first Stage C waypoint is reachable from Stage B ending
- May need to modify trajectory or add transition segment

## Recommended Next Action

**Analyze the arm configuration at Stage B ending:**
1. Check joint limits proximity
2. Calculate manipulability ellipsoid
3. Test reachability of Stage C waypoints from Stage B ending
4. Compare IK solution quality between home and Stage B ending for Stage C waypoints

## Conclusion

The root cause is NOT:
- ❌ Initialization/sequential warm-starting
- ❌ MaxIterations limit
- ❌ Missing Phase 1/2A parameters

The root cause IS:
- ✅ **Stage B ending arm configuration is incompatible with Stage C requirements**

Further investigation needed to determine:
- Why Stage B ending causes problems
- How to fix the Stage B→C transition
- Whether to modify Stage B, Stage C, or add transition segment

---

**Session Date:** October 14, 2025  
**Time Spent:** ~4 hours  
**Status:** Root cause narrowed down, specific fix pending further investigation
