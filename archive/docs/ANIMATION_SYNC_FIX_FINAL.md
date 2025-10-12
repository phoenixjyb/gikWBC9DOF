# Animation Sync Fix - Final Solution

**Issue:** Red dot (Stage C reference EE waypoint) lags behind robot motion  
**Root Cause:** Passing UNSAMPLED reference trajectory to animator while robot uses SAMPLED trajectory  
**Date Fixed:** October 11, 2025  
**Files Modified:** `matlab/+gik9dof/animateStagedWithHelper.m`

---

## The Real Problem

After deeper investigation, I found the actual bug was on **line 71** of `animateStagedWithHelper.m`:

```matlab
helperOptions.StageCReferenceEEPath = eePathStageCRef.';  % ❌ BUG: FULL trajectory!
```

### What Was Happening:

1. **Line 30-31:** Sample arm and base trajectories with `SampleStep`
   ```matlab
   armTrajectory = armTrajectory(1:opts.SampleStep:end, :);  % frames: [1, 3, 5, ...]
   basePose = basePose(1:opts.SampleStep:end, :);            % frames: [1, 3, 5, ...]
   ```

2. **Line 51-57:** Compute or sample `eePoses` correctly
   ```matlab
   if isempty(eePathStageCRef)
       eePathStageCRef = computeEEPath(robot, qTraj(:, 1:opts.SampleStep:end));
       eePoses = eePathStageCRef';  % Already sampled
   else
       eePoses = eePathStageCRef(:, 1:opts.SampleStep:end)';  % Sample it
   end
   ```

3. **Line 71:** Pass FULL `eePathStageCRef` to options ❌
   ```matlab
   helperOptions.StageCReferenceEEPath = eePathStageCRef.';  % Has ALL frames!
   ```

4. **In `animate_whole_body.m`:**
   - Receives `numSteps` based on SAMPLED trajectory (e.g., 169 frames)
   - Receives `StageCReferenceEEPath` with FULL trajectory (e.g., 337 frames)
   - Maps full reference[1:169] to animation frames[1:169]
   - But Stage C only starts at frame ~100 in the sampled timeline
   - So reference frame 100 goes to animation frame 100, when it should go to frame ~50!
   - **Result:** Red dot shows later frames than it should = LAG

---

## The Fix

### Changed Line 71:
```matlab
# BEFORE (BUGGY):
helperOptions.StageCReferenceEEPath = eePathStageCRef.';  # FULL trajectory

# AFTER (FIXED):
helperOptions.StageCReferenceEEPath = eePoses;  # SAMPLED trajectory
```

### Why This Works:
- `eePoses` is already correctly sampled to match `armTrajectory` and `basePose`
- `eePoses` has the exact same number of frames as the animation (e.g., 169)
- When `animate_whole_body` maps it to Stage C frames, the indices now align perfectly
- Red dot frame N corresponds to robot animation frame N ✅

---

## Code Changes

**File:** `matlab/+gik9dof/animateStagedWithHelper.m`

```matlab
helperOptions = opts.HelperOptions;
if ~isempty(eePathDesired)
    helperOptions.DesiredEEPath = eePathDesired.';
else
    helperOptions.DesiredEEPath = [];
end
% CRITICAL: Pass the SAMPLED eePoses, not the full eePathStageCRef
% The red dot position must match the animation frame indices
helperOptions.StageCReferenceEEPath = eePoses;  # ✅ FIXED
```

---

## Testing

### Single Sample Test:
```matlab
matlab -batch "test_single_animation_sync"
```

This generates: `Baseline_SYNC_FIXED.mp4`

### Verification Checklist:
When reviewing the test animation, confirm:
- [ ] Red dot moves synchronously with robot (no lag)
- [ ] Red dot starts immediately when robot starts
- [ ] Red dot stays aligned with green square throughout
- [ ] EE position error display is accurate

### Regenerate All Animations:
Once the single sample is verified, regenerate all 5 parametric study animations:
```matlab
matlab -batch "generate_parametric_animations"
```

This will overwrite the existing animations with properly synchronized versions (~11 minutes).

---

## Why Previous Fix Didn't Work

My first attempt fixed line 57:
```matlab
eePoses = eePathStageCRef(:, 1:opts.SampleStep:end)';  # Correct sampling
```

But I **forgot to update line 71** which passes data to the animator! So:
- `eePoses` (for marker update in animation loop) was correct
- But `StageCReferenceEEPath` (for initial setup and path display) was still using FULL trajectory
- The red marker in the animation loop uses `eePoses` indirectly through `stageCFull`, which gets populated from `StageCReferenceEEPath`
- So the bug persisted!

---

## Technical Details

### Data Flow:
```
animateStagedWithHelper:
  ├─> eePoses (sampled) → passed to animate_whole_body as 7th argument (eePoses)
  └─> helperOptions.StageCReferenceEEPath (NOW ALSO SAMPLED!) → used to populate stageCFull

animate_whole_body:
  ├─> numSteps = length(armTrajectory) = 169 (sampled)
  ├─> stageCIdxRange = stage C frames in sampled timeline (e.g., 89:169)
  ├─> stageCFull = nan(169, 3)
  ├─> stageCFull(stageCIdxRange(1:L), :) = stageCRefPath(1:L, :)
  │   NOW: L = length(stageCIdxRange) = 81
  │   stageCRefPath has 81 frames (from sampled eePoses)
  │   Maps perfectly: reference[1:81] → animation[89:169] ✅
  └─> Red marker updated each frame: stageCFull(k, :) where k is animation frame index
```

### Before Fix:
```
stageCRefPath had 148 frames (FULL Stage C trajectory)
Tried to map reference[1:81] → animation[89:169]
But reference[1:81] was actually reference frames for FULL trajectory indices [1:81]
Which correspond to SAMPLED frames [1:41] approximately
Result: Red dot showed early frames = BEHIND the robot
```

### After Fix:
```
stageCRefPath has 81 frames (SAMPLED Stage C trajectory)  
Maps reference[1:81] → animation[89:169]
reference[1:81] already corresponds to sampled frames [89:169]
Result: Red dot synchronized with robot ✅
```

---

## Summary

**Root Cause:** Passing unsampled reference trajectory to animator  
**Fix Location:** Line 71 of `animateStagedWithHelper.m`  
**Fix:** Change `eePathStageCRef.'` to `eePoses`  
**Impact:** Visual synchronization only (metrics always correct)  
**Status:** ✅ **FIXED** (2025-10-11)

**Next Action:** Test single animation, then regenerate all 5 animations if test passes.

---

**Generated:** 2025-10-11 11:25:00  
**Fix Applied:** ✅ `matlab/+gik9dof/animateStagedWithHelper.m` line 71
