# Quick Start: Method 4 Comparison

**Date:** October 13, 2025  
**Status:** Ready to run

---

## What's Been Fixed

✅ **Path refinement bug** in `initPPFromBasePath.m` - Fixed  
✅ **Comparison script variable error** - Fixed  
✅ **Performance test trajectory loading** - Fixed  
✅ **Comparison script now has skip flags** - Can reuse existing results

---

## Option 1: Run Performance Diagnostic (Recommended First)

**Purpose:** Find out why Method 1 takes 40 minutes

```bash
cd /Users/yanbo/Projects/gikWBC9DOF
matlab -batch "test_method1_performance"
```

**Time:** 5-15 minutes  
**Output:** 
- Time per waypoint for 10, 20, 30 waypoints
- Extrapolated estimate for full 148 waypoints
- Diagnosis of bottleneck (Stage B vs Stage C)

**What to look for:**
- If ~0.5 s/waypoint → Normal (148 wp = ~1.2 min total) ✓
- If >2 s/waypoint → Problem! (148 wp = ~5+ min)
- If >15 s/waypoint → Serious issue (148 wp = 40+ min) ← Your current issue

---

## Option 2: Skip Method 1, Just Run Method 4

**Purpose:** Complete Method 4 testing without waiting

1. **Edit `compare_method1_vs_method4.m` (lines 7-8):**
```matlab
RUN_METHOD1 = false;  % Skip Method 1
RUN_METHOD4 = true;   % Run Method 4

EXISTING_METHOD1_LOG = 'results/20251013_155120_method_comparison/log_method1_ppForIk.mat';
```

2. **Run comparison:**
```bash
matlab -batch "compare_method1_vs_method4"
```

**Time:** ~5-10 minutes (Method 4 only)  
**Output:** Full comparison report using your existing Method 1 data

---

## Option 3: Quick Method 4 Standalone Test

**Purpose:** Just verify Method 4 works

```matlab
% In MATLAB:
robot = gik9dof.createRobotModel();
env = struct('BaseHome', [-2,-2,0], 'FloorDiscs', [], 'DistanceMargin', 0.3, ...
    'DistanceWeight', 5.0, 'StageBMode', 'pureHyb');

tic;
log4 = gik9dof.trackReferenceTrajectory('Mode', 'staged', ...
    'ExecutionMode', 'ppFirst', 'Verbose', false, ...
    'EnvironmentConfig', env, 'UseStageBHybridAStar', false);
elapsed = toc;

fprintf('Method 4 completed in %.2f seconds (%.1f min)\n', elapsed, elapsed/60);
```

**Expected:** 5-10 minutes for full trajectory

---

## Quick Fixes for Slow Method 1

If performance test shows Method 1 is still too slow, try these:

### Fix 1: Disable Hybrid A* (Stage B bottleneck)
```matlab
% In compare_method1_vs_method4.m or your test:
'UseStageBHybridAStar', false,  % Use simple path instead
```

### Fix 2: Reduce GIK Iterations (Solver bottleneck)
```matlab
'MaxIterations', 500,  % Down from 1500 (for testing)
```

### Fix 3: Remove Obstacles (Collision checking bottleneck)
```matlab
env.FloorDiscs = [];  % No obstacles
```

### Fix 4: Lower Control Rate (More waypoints generated)
```matlab
'RateHz', 5,  % Down from 10 Hz (fewer waypoints)
```

---

## Understanding the Performance Test Output

```
Test 1/3: 10 waypoints
  ✓ Total time: 12.5 s
  • Time per waypoint: 1.250 s
  📊 Extrapolated time for 148 waypoints: 3.1 min
```

**Interpretation:**
- **1.25 s/waypoint** is reasonable for MATLAB
- **3.1 min total** is acceptable
- If it says **40+ min**, there's a major bottleneck

**Common Bottlenecks:**
1. **Stage B: Hybrid A*** - Can take 5-10 min if environment is complex
2. **Stage C: GIK solver** - Can take long if not converging
3. **Collision checking** - Slow if many obstacles with tight margins
4. **Path interpolation** - Might be creating too many waypoints

---

## What's Next After Performance Test

### If Method 1 is Fast (<10 min):
✓ Great! Run full comparison:
```bash
matlab -batch "compare_method1_vs_method4"
```

### If Method 1 is Medium (10-20 min):
⚠️ Acceptable but slow. Consider:
- Disabling Hybrid A* for testing
- Reducing GIK iterations to 1000
- Then run comparison

### If Method 1 is Slow (>20 min):
🔴 Problem! Try quick fixes above, then:
1. Apply the fastest fix
2. Re-run performance test to verify
3. Then run comparison

---

## Expected Timeline

| Task | Time | Status |
|------|------|--------|
| Performance diagnostic | 5-15 min | ⏳ Ready to run |
| Apply performance fix (if needed) | 10-30 min | Conditional |
| Run Method 4 only | 5-10 min | ⏳ Ready |
| Run full comparison | 10-20 min | After fixes |
| **Total** | **15-75 min** | Depends on Method 1 |

---

## My Recommendation

**Step 1:** Run performance diagnostic NOW
```bash
matlab -batch "test_method1_performance"
```

**Step 2:** Based on results:
- If <10 min estimated → Run full comparison
- If >10 min estimated → Apply quick fix, test again
- If still slow → Skip Method 1, use existing results

**Step 3:** Complete Method 4 comparison with working setup

---

## Files Ready to Use

1. ✅ `test_method1_performance.m` - Performance diagnostic (FIXED)
2. ✅ `compare_method1_vs_method4.m` - Full comparison (ENHANCED with skip flags)
3. ✅ `initPPFromBasePath.m` - Path refinement bug fixed
4. 📋 `METHOD4_CONSOLIDATION_EXECUTION_PLAN.md` - Detailed consolidation plan
5. 📋 `METHOD4_CONSOLIDATION_SUMMARY.md` - Quick overview

---

## Need Help?

**Run into errors?** Check:
- `startup.m` is run (paths configured)
- Robot model loads: `robot = gik9dof.createRobotModel()`
- Trajectory file exists: `dir refEETrajs/1_pull_world_scaled.json`

**Still slow?** Try the quick fixes above or ask for help with the performance test output.

**Ready to proceed?** Run the performance diagnostic and share the results!
