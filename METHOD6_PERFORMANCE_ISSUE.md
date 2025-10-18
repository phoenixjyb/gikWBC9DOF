# Method 6 Performance Issue - Investigation Notes

**Date:** October 15, 2025  
**Status:** ⚠️ Performance Issue Identified

## Observed Performance (First Run)

### Timing
- **Mean solve time:** 393ms (expected ~11ms) ❌ **35x slower!**
- **Base PP steps:** 0.2ms average ✅ (as expected)
- **Arm GIK steps:** 786ms average ❌ **78x slower than expected!**
- **Control rate:** 2.5 Hz (expected ~90 Hz) ❌

### Accuracy  
- **Mean EE error:** 1820mm (expected ~150mm) ❌ **12x worse!**
- **Max EE error:** 2302mm ❌
- **Progress:** Only 6/210 waypoints completed in 84 seconds

## Root Cause Analysis

The GIK solver is extremely slow (786ms vs expected 10ms). Possible causes:

### 1. Max Iterations Too High ⚠️ LIKELY
- Config shows: `MaxIterations: 1500`
- GIK hitting iteration limit without converging
- Should tune down to 100-200 for real-time

### 2. Base Not Actually Frozen 🤔
- Code says base frozen but GIK might still be optimizing it
- Need to verify base joint constraints are properly set
- Check if we need to call `updateBaseJointBounds` with tight limits

### 3. Initial Guess Poor ❓
- Using current state as initial guess
- But base might be far from good position for EE reach
- Base path generation might need tuning (offset = 0.5m may be wrong)

### 4. Distance Specs Too Strict ❓
- DistanceSpecs not set in our code (optional parameter)
- Default constraints might be too tight
- Causing GIK to struggle

## Comparison with Method 4 (PPFirst)

Method 4 also uses GIK with base constraints:
```matlab
gik9dof.updateBaseJointBounds(gikBundle, options.BaseIndices, q_base_pred, ...
    base_yaw_lower, base_yaw_upper, options.StageCMaxLinearSpeed*dt);
[q_gik, solInfo] = gikBundle.solve(q_current, 'TargetPose', T_ee_target);
```

Method 6 does NOT call `updateBaseJointBounds` - we just rely on initial guess!

## Recommended Fixes

### Fix 1: Add Base Joint Bounds (HIGH PRIORITY)
Lock base joints tightly around current position:
```matlab
% Before GIK solve
gik9dof.updateBaseJointBounds(gikBundle, baseIdx, state(baseIdx), ...
    state(3), state(3), 0.001);  % Tiny tolerance on position & yaw
```

### Fix 2: Reduce Max Iterations (HIGH PRIORITY)  
```matlab
% In createGikSolver call
'MaxIterations', 200  % Not 1500!
```

### Fix 3: Improve Base Path Generation (MEDIUM)
Current code offsets base 0.5m behind EE. Better approach:
- Use inverse kinematics to find good base positions
- Or reuse Pure Pursuit path planning logic from Method 4

### Fix 4: Add Convergence Timeout (LOW)
```matlab
% In GIK loop, add max solve time check
if solve_time > 0.05  % 50ms timeout
    % Skip or use fallback
end
```

## Action Items

1. ✅ Fix `successMask` field (done - enables log merging)
2. ⏳ Add `updateBaseJointBounds` before GIK solve
3. ⏳ Reduce MaxIterations to 100-200
4. ⏳ Test and validate performance improvements
5. ⏳ Compare with Method 1/4 solve times

## Expected After Fixes

- **GIK solve time:** ~10-20ms (vs current 786ms)
- **Total solve time:** ~11-21ms (vs current 393ms)  
- **Control rate:** 50-90 Hz (vs current 2.5 Hz)
- **EE error:** ~150-200mm (vs current 1820mm)

---
*Investigation started: Oct 15, 2025 after first run*
