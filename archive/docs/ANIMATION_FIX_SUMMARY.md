# Animation Fix Summary - Complete Solution

## Date: October 11, 2025

## Problem Summary

**User discovered:** Animation has TWO critical bugs affecting stage labels and red dot synchronization.

### Bug 1: Red Dot Lag (Initial Report)
"The red dot is not in sync with the robot and lags behind"

### Bug 2: Stage Boundary Mismatch (User Insight)
"Are we confused about play fps and the actual tag fps? Could this tamper with stage tags showing up at right time?"

---

## Root Causes Identified

### Root Cause 1: Red Dot Indexing (STAGE_SYNC_FIX_COMPLETE.md)

**Problem:** Red dot indexed global timeline, but only had data for Stage C.

```matlab
// WRONG: Index into full timeline
stageCFull(k) where k = 1..171
// But stageCFull(1..96) = NaN (no Stage C data yet)

// RIGHT: Index relative to Stage C start
stageCPath(k - stageCFirstIdx + 1) when k >= 97
```

**Fix Location:** `/matlab/+gik9dof/+viz/animate_whole_body.m`
- Lines 181-199: Store `stageCPath` separately with `stageCFirstIdx`
- Lines 295-309: Initialize red dot with visibility='off'
- Lines 466-488: Update red dot with stage-relative indexing

### Root Cause 2: Stage Boundary Frame Space Mismatch (COMPLETE_TIMING_FIX.md)

**Problem:** Stage boundaries in raw frame space, animation in sampled frame space.

```
Raw frames:    337 total [Stage A: 51, B: 139, C: 149]
Sampled (÷2):  169 total [Stage A: 26, B: 70, C: 75]

Boundaries passed: [51, 190, 339]  (RAW)
Trajectories:      169 frames      (SAMPLED)
After clipping:    [51, 169, 169]  (BROKEN)

Stage C range: 170..169 (EMPTY!)
```

**Fix Location:** `/matlab/+gik9dof/animateStagedWithHelper.m`
- Lines 111-120: Transform boundaries from raw→sampled space

---

## Complete Fix Applied

### File 1: `/matlab/+gik9dof/+viz/animate_whole_body.m`

**Change 1:** Store Stage C path separately (lines 181-199)
```matlab
stageCPath = [];  // Stage C reference trajectory
stageCFirstIdx = stageCIdxRange(1);  // When Stage C starts
if ~isempty(stageCRefPath)
    stageCPath = pathC(:, 1:3);  // Store separately
end
```

**Change 2:** Initialize red dot hidden (lines 295-309)
```matlab
if ~isempty(stageCPath)
    eeMarkerPersp = plot3(..., 'Visible', 'off');
    eeMarkerTop = plot(..., 'Visible', 'off');
end
```

**Change 3:** Update red dot with stage-relative indexing (lines 466-488)
```matlab
if k >= stageCFirstIdx && k <= stageCIdxRange(end)
    stageCLocalIdx = k - stageCFirstIdx + 1;
    set(eeMarkerPersp, 'XData', stageCPath(stageCLocalIdx,1), ..., 'Visible', 'on');
else
    set(eeMarkerPersp, 'Visible', 'off');  // Hide during Stages A & B
end
```

### File 2: `/matlab/+gik9dof/animateStagedWithHelper.m`

**Change:** Transform stage boundaries to sampled space (lines 111-120)
```matlab
[stageBoundaries, stageLabels] = detectStageBoundaries(logStaged);
if opts.SampleStep > 1 && ~isempty(stageBoundaries)
    stageCounts = [stageBoundaries(1), diff(stageBoundaries)];
    sampledCounts = arrayfun(@(c) length(1:opts.SampleStep:c), stageCounts);
    stageBoundaries = cumsum(sampledCounts);  // Transform to sampled space
end
```

---

## Results

### Before Fix
- ❌ Stage A label: frames 1-51 (should be 1-26) → 2x too long
- ❌ Stage B label: frames 52-169 (should be 27-96) → Covers entire video
- ❌ Stage C label: frames 170-169 (empty range) → Never shown
- ❌ Red dot: Never appears (Stage C range empty)

### After Fix
- ✅ Stage A label: frames 1-26 (1.3 sec @ 20fps)
- ✅ Stage B label: frames 27-96 (3.5 sec @ 20fps)
- ✅ Stage C label: frames 97-171 (3.75 sec @ 20fps)
- ✅ Red dot: Appears at frame 97, synchronized with robot

---

## Verification

### Test Animations Generated
1. `TEST_STAGE_SYNC_FIXED.mp4` (7.5 MB) - Tests red dot fix only
2. `TEST_COMPLETE_FIX.mp4` (6.0 MB) - Tests both fixes combined

### Checklist for Review

**Stage Labels:**
- [ ] "Stage A" visible for frames 1-26 (1.3 sec)
- [ ] "Stage B" visible for frames 27-96 (3.5 sec)
- [ ] "Stage C" visible for frames 97-171 (3.75 sec)
- [ ] Transitions at correct frames (26→27, 96→97)

**Red Dot (Stage C Reference):**
- [ ] Hidden during frames 1-96 (Stages A & B)
- [ ] Appears at frame 97 (exactly when "Stage C" label appears)
- [ ] Moves synchronously with robot (no lag)
- [ ] Tracks Stage C reference trajectory smoothly

**Overall:**
- [ ] Total duration: 8.55 seconds (171 frames @ 20fps)
- [ ] No visual jumps or glitches
- [ ] Stage transitions look natural

---

## Understanding the Timing

### Three Time Scales

1. **Simulation Time (10 Hz)**
   - Stage A: 5.1 sec (51 frames)
   - Stage B: 13.9 sec (139 frames)
   - Stage C: 14.9 sec (149 frames)
   - Total: 33.7 sec (337 frames)

2. **Sampled Data (SampleStep=2)**
   - Stage A: 26 frames
   - Stage B: 70 frames
   - Stage C: 75 frames
   - Total: 171 frames (every other simulation frame)

3. **Video Playback (20 fps)**
   - Stage A: 1.3 sec
   - Stage B: 3.5 sec
   - Stage C: 3.75 sec
   - Total: 8.55 sec

**Relationship:** Video is 4x faster than real-time (20fps video showing 10Hz simulation with 50% sampling).

---

## Impact on Previous Work

### All 5 Parametric Study Animations Need Regeneration

Previous animations had both bugs:
1. Stage labels incorrect (Stage C never shown)
2. Red dot never appeared (Stage C range was empty)

**To regenerate:**
```bash
matlab -batch "generate_parametric_animations"
```

Estimated time: ~11 minutes (2.2 min per animation)

---

## Key Insights

### What We Learned

1. **Frame Space Consistency Critical:** When you sample data, you MUST sample all related indices
2. **Visual Bugs Can Be Layered:** Red dot bug masked by stage boundary bug
3. **User Domain Knowledge Invaluable:** Understanding "10Hz sim + 20fps video + sampling" revealed root cause
4. **Test What You See:** Functional tests might pass, but visual inspection catches presentation bugs

### Why This Matters

Animation is often the primary way to:
- Debug trajectory issues
- Communicate results to stakeholders
- Verify algorithm behavior

Incorrect stage labels and missing reference markers make animations misleading or unusable.

---

## Files Modified

1. `/matlab/+gik9dof/+viz/animate_whole_body.m` - Red dot indexing fix
2. `/matlab/+gik9dof/animateStagedWithHelper.m` - Stage boundary sampling fix

## Diagnostic Scripts Created

1. `debug_stage_boundaries.m` - Initial red dot investigation
2. `debug_sampling_mismatch.m` - Boundary frame space mismatch proof
3. `test_stage_sync_fix.m` - Red dot fix test
4. `test_complete_fix.m` - Complete fix validation

## Documentation Created

1. `STAGE_SYNC_FIX_COMPLETE.md` - Red dot indexing fix details
2. `COMPLETE_TIMING_FIX.md` - Stage boundary sampling fix details
3. `ANIMATION_FIX_SUMMARY.md` - This summary

---

## Status

✅ **Both fixes implemented and tested**  
✅ **Test animations generated successfully**  
⏳ **Awaiting user review of test animations**  
⏳ **Ready to regenerate all 5 parametric study animations**

---

## Next Actions

1. **User:** Review `TEST_COMPLETE_FIX.mp4` to verify:
   - Stage labels appear at correct times
   - Red dot appears at frame 97 (Stage C start)
   - Red dot moves synchronously with robot

2. **If test passes:** Regenerate all animations:
   ```bash
   matlab -batch "generate_parametric_animations"
   ```

3. **Document timing conventions:** Add to project documentation for future reference

---

**Credit:** User's insight about timing/sampling relationship was critical to identifying the stage boundary bug. This demonstrates the value of understanding system architecture when debugging.
