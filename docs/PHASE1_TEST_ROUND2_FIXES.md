# Phase 1 Test - Second Round of Fixes

**Date:** October 14, 2025  
**Issue:** First round of fixes didn't work - lateral velocity still 41.57 m/s!

---

## Root Cause Discovered

### Issue #1: Anisotropic Corridor Too Conservative ❌

The yaw-aligned anisotropic corridor in `updateBaseJointBounds()` was creating **overly tight constraints**:

```matlab
// World-frame bounds from body-frame corridor
dx_max = abs(eps_long * cos_th) + abs(eps_lat * sin_th);
dy_max = abs(eps_long * sin_th) + abs(eps_lat * cos_th);
```

**Problem:**
- At 0°: dx_max = eps_long (144mm), dy_max = eps_lat (15mm) ✓
- At 45°: dx_max = dy_max ≈ (144+15)/√2 = 112mm 
- At 90°: dx_max = eps_lat (15mm), dy_max = eps_long (144mm) ✗

This creates a **rotating rectangular hull** that's:
1. Too conservative (maps body-frame to world-frame incorrectly)
2. Tighter than baseline (150mm isotropic)
3. Causes 51% fallback rate + 46.7% convergence

**Fix Applied:**
```matlab
// Use isotropic corridor (eps_long for both x and y)
gik9dof.updateBaseJointBounds(gikBundle, options.BaseIndices, q_base_pred, ...
    options.YawTolerance, eps_long);  // Only 2 args = isotropic
```

Now corridor is ~144mm in both x and y (comparable to baseline's 150mm).

---

### Issue #2: Lateral Velocity Measured Before Fallback ❌

**Problem:**
```matlab
// OLD order:
[q_gik, solInfo] = gikBundle.solve(...);  // GIK solution (may fail)
v_lat = measure_lateral_velocity(q_gik);  // Measure FAILED solution
if (ee_error > threshold)
    q_final = [q_base_pred; q_arm_fallback];  // Apply fallback
```

When GIK failed (51% of time), we measured lateral velocity of the **failed GIK solution**, not the **actual applied motion** (PP prediction).

This explains the correlation:
- Fallback 0% → v_lat = 0.057 m/s (measuring successful GIK)
- Fallback 51% → v_lat = 41.57 m/s (measuring failed GIK)

**Fix Applied:**
```matlab
// NEW order:
[q_gik, solInfo] = gikBundle.solve(...);
if (ee_error > threshold)
    q_final = [q_base_pred; q_arm_fallback];  // Apply fallback FIRST
else
    q_final = q_gik;
v_lat = measure_lateral_velocity(q_final);  // Measure ACTUAL motion
```

Now we measure what's actually applied, which should be PP prediction (differential-drive feasible).

---

## Expected Results (This Run)

### Baseline (same as before):
- Mean EE Error: 556mm
- Fallback: 15.2%
- Convergence: 66.2%
- Lateral vel: N/A

### Enhanced (with corridor fix):
- Mean EE Error: **~400-600mm** (much better than 5310mm!)
- Fallback: **~10-20%** (better than 51%)
- Convergence: **~70-80%** (better than 46.7%)
- Lateral vel: **<0.05 m/s** (should be reasonable now)

**Rationale:** With isotropic corridor (~144mm), enhanced version now has similar constraint width to baseline (150mm). The velocity-based corridor doesn't help yet (still tight), but at least it's not hurting.

---

## Why First Fixes Didn't Work

### What We Fixed (Round 1):
1. ✅ Lateral velocity calculation (sample time) - **FORMULA WAS CORRECT**
2. ✅ Baseline parameters (15°/0.15m) - **WORKED** (baseline now 556mm, not 0mm)
3. ✅ Corridor minimum (10mm → 50mm) - **HELPED A BIT**
4. ✅ Warm-starting safety - **GOOD SAFEGUARD**

### What We Missed (Round 1):
1. ❌ **Anisotropic corridor** - This was the KILLER (way too tight)
2. ❌ **Diagnostic timing** - Measuring wrong thing (GIK not final)

### Lessons Learned:
- ⚠️ **Conservative transformations can be too conservative** - The yaw-aligned mapping was mathematically sound but practically too tight
- ⚠️ **Measure what's actually applied** - Diagnostics before fallback are misleading
- ⚠️ **Fallback rate is a red flag** - 51% meant something was fundamentally wrong
- ⚠️ **Low convergence rate (46.7%)** - Solver couldn't find solutions in tight corridor

---

## Fundamental Design Questions

### Q1: Do we even need anisotropic corridors?

**Hypothesis:** Differential drive constraint (no lateral motion) should be enforced by:
1. **PP predictions are inherently feasible** (diff-drive controller)
2. **Tight yaw corridor (±15°)** keeps base aligned with PP
3. **Position box just needs to be large enough for solver to converge**

**Conclusion:** Anisotropic corridor was **premature optimization**. We tried to enforce differential-drive constraint at the corridor level, but:
- It made solver fail (51% fallback)
- PP already guarantees feasibility
- Yaw corridor is the key constraint

### Q2: What's the right corridor strategy?

**Option A: Fixed isotropic** (baseline approach)
```matlab
eps = 0.15m  // Same for x, y
```
- ✅ Simple, proven to work
- ✅ Solver converges (66% baseline)
- ❌ Doesn't adapt to velocity

**Option B: Velocity-based isotropic** (current fix)
```matlab
eps_long = max(0.05m, |v|*dt + 0.02)  // 50-160mm
eps_lat = eps_long  // Same value (isotropic)
```
- ✅ Adapts to speed
- ✅ Should help at low speeds
- ✅ Still simple (isotropic)
- ⚠️ Need to test if it actually helps

**Option C: Anisotropic in body frame** (future, if needed)
```matlab
// Directly constrain body-frame velocities
v_long ∈ [-0.6, 0.6] m/s
v_lat ∈ [-0.02, 0.02] m/s
```
- ✅ Physically meaningful
- ✅ Direct constraint on differential drive
- ❌ Requires different constraint type (not joint bounds)
- ❌ More complex

**Recommendation:** Start with Option B (velocity-based isotropic). If successful, can explore Option C in Phase 2.

---

## Files Modified (Round 2)

1. **`matlab/+gik9dof/runStageCPPFirst_enhanced.m`**
   - Line 250: Changed `updateBaseJointBounds()` call to use isotropic corridor (2 args instead of 3)
   - Lines 280-310: Moved lateral velocity diagnostic AFTER fallback decision

---

## Confidence Level (Round 2)

**Overall:** 🟢 **75%** (Moderate-High)

| Fix | Confidence | Risk |
|-----|------------|------|
| Isotropic corridor | 85% | Should match baseline performance |
| Diagnostic timing | 95% | Obvious fix, low risk |
| Overall success | 75% | Solver should converge better |

**Risk Factors:**
- Velocity-based corridor might not help much (still ~144mm like baseline)
- Warm-starting might not be effective if solver still struggles
- Adaptive lookahead still broken

**Best Case:** Match or slightly beat baseline (556mm)  
**Expected Case:** Close to baseline (~600mm), lower fallback  
**Worst Case:** Still worse than baseline but much better than 5310mm

---

## Next Steps After This Test

### If PASSES (3-4 criteria):
1. 🎉 Celebrate fixing the bugs!
2. 📊 Analyze which improvements actually helped
3. 🚀 Consider Phase 2 or tune further

### If 2/4 PASS (partial success):
1. 🔍 Check which criterion failed
2. 🔧 Targeted fixes:
   - If error high: Increase corridor or improve PP
   - If fallback high: Relax EE threshold or improve warm-start
   - If convergence low: Relax solver tolerances more
   - If v_lat high: Debug formula again (should be fixed now)

### If 0-1/4 PASS (still failing):
1. 🚨 **Disable ALL improvements** and test individually:
   ```matlab
   UseAdaptiveLookahead = false
   UseMicroSegment = false
   UseWarmStarting = false
   UseVelocityCorridor = false  // Use baseline corridor
   RelaxedTolerances = false
   ```
2. Test baseline vs enhanced-with-single-improvement
3. Find which improvement helps/hurts
4. Build up incrementally

---

## Timeline

- **11:00 AM** - First fixes applied (lateral vel calc, baseline params, etc.)
- **11:15 AM** - First test run → FAILED (all criteria)
- **11:30 AM** - Root cause analysis (anisotropic corridor, diagnostic timing)
- **11:45 AM** - Second fixes applied (isotropic corridor, moved diagnostic)
- **12:00 PM** - Second test run started (CURRENT)
- **12:15 PM** - Results expected

**Total Debug Time:** ~2 hours  
**Fixes Applied:** 6 changes across 2 files  
**Bugs Found:** 5 (2 critical, 3 minor)

---

**Status:** ⏳ Test running... (15 min)

**Key Insight:** The anisotropic corridor was a **design flaw**, not a tuning issue. Sometimes "improvements" make things worse!
