# Stage C EE Reference Path Fix - Complete Solution

## Date: October 11, 2025

## Problem Discovered by User

**User Report:** "Stage C reference EE path somehow starts from the homebase position, and very similar to Chassis path in Stage B. This is confusing that holds red dot tracking from the wrong place."

**Root Cause:** Red dot was tracking the **entire robot EE trajectory** (Stages A+B+C from homebase), not the **Stage C desired trajectory** from JSON.

---

## Investigation

### What Was Happening

**In `animateStagedWithHelper.m` (lines 42-58, before fix):**

```matlab
eePathStageCRef = [];
if isfield(logStaged.stageLogs.stageC, 'referenceInitialIk') && ...
        isfield(logStaged.stageLogs.stageC.referenceInitialIk, 'eePositions')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;
end

if isempty(eePathStageCRef)
    // FALLBACK: Compute from FULL qTraj!
    eePathStageCRef = computeEEPath(robot, qTraj(:, 1:opts.SampleStep:end));
end
```

**The Problem:**
1. `referenceInitialIk.eePositions` field **does NOT exist** in logs
2. Falls back to `computeEEPath(robot, qTraj)` 
3. `qTraj` contains **ALL stages** (A+B+C, 337 frames total)
4. Result: Red dot shows entire robot motion from homebase!

### What Should Happen

**Stage C desired trajectory is stored in:** `stageC.targetPositions`

This is the **JSON desired EE trajectory** that Stage C should track (148 waypoints).

---

## The Fix

### File Modified: `/matlab/+gik9dof/animateStagedWithHelper.m`

**Location:** Lines 42-74

**New Priority-Based Data Source Selection:**

```matlab
eePathStageCRef = [];
% Try to get Stage C reference EE path (desired trajectory for tracking)
if isfield(logStaged, 'stageLogs') && isstruct(logStaged.stageLogs) && ...
        isfield(logStaged.stageLogs, 'stageC')
    stageC = logStaged.stageLogs.stageC;
    
    % Priority 1: referenceInitialIk.eePositions (if available)
    if isfield(stageC, 'referenceInitialIk') && ...
            isfield(stageC.referenceInitialIk, 'eePositions') && ...
            ~isempty(stageC.referenceInitialIk.eePositions)
        eePathStageCRef = stageC.referenceInitialIk.eePositions;
        
    % Priority 2: targetPositions (desired EE trajectory from JSON)
    elseif isfield(stageC, 'targetPositions') && ~isempty(stageC.targetPositions)
        eePathStageCRef = stageC.targetPositions;
        
    % Priority 3: eePositions (actual Stage C EE, not reference but better than nothing)
    elseif isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
        eePathStageCRef = stageC.eePositions;
    end
end

if isempty(eePathStageCRef)
    % Last resort: Use full trajectory reference if available
    if isfield(logStaged, 'referenceTrajectory') && ...
            isfield(logStaged.referenceTrajectory, 'EndEffectorPositions') && ...
            ~isempty(logStaged.referenceTrajectory.EndEffectorPositions)
        eePathStageCRef = logStaged.referenceTrajectory.EndEffectorPositions;
    end
end

% Apply sampling
if ~isempty(eePathStageCRef)
    eePoses = eePathStageCRef(:, 1:opts.SampleStep:end)';
else
    eePoses = [];
end
```

### Key Changes

1. **Removed fallback to `computeEEPath(robot, qTraj)`** - This was computing from full trajectory!

2. **Added priority-based selection:**
   - **Priority 1:** `referenceInitialIk.eePositions` (ideal, but rarely exists)
   - **Priority 2:** `stageC.targetPositions` (JSON desired trajectory) ✅ **Used in our case**
   - **Priority 3:** `stageC.eePositions` (actual EE, not reference but better than full trajectory)
   - **Last resort:** `referenceTrajectory.EndEffectorPositions` (full trajectory reference, not Stage C specific)

3. **For our data:** Uses `stageC.targetPositions` which is exactly the desired Stage C trajectory from JSON

---

## Verification

### Data Source Confirmed

**Test output:**
```
✓ Will use: stageC.targetPositions (JSON desired trajectory)
  This is the CORRECT source!

Selected data:
  Size: 3 × 148
  First point: [1.6500, 0.0800, 0.8600]
  Last point:  [1.5675, 0.0833, 0.8598]

Comparison with referenceTrajectory.EndEffectorPositions:
  Match: true
  ✓ Confirmed: Using Stage C desired trajectory!
```

### Test Animation Generated

**File:** `TEST_STAGEC_PATH_FIX.mp4` (9.0 MB) ✅

**Expected Behavior:**
- Red dot appears at frame 97 (Stage C start)
- Red dot starts at **[1.65, 0.08, 0.86]** (first Stage C waypoint)
- Red dot does NOT start from homebase
- Red dot tracks JSON desired trajectory
- Red dot is clearly different from Stage B chassis path (which is on ground, Z≈0)
- Red dot is ABOVE chassis (Z≈0.86 m, end-effector height)

---

## Before vs After

### Before Fix

**Red dot trajectory:**
- Started at homebase [~0.0, 0.0, ~0.0]
- Followed entire robot EE motion (Stages A+B+C)
- Looked identical to chassis path
- Confused with Stage B executed base path

**Visual issues:**
- Red dot "travels" from homebase during Stages A & B (when hidden)
- Red dot appears at wrong position when Stage C starts (frame 97)
- User sees red dot tracking from wrong place

### After Fix

**Red dot trajectory:**
- Starts at first Stage C waypoint [1.65, 0.08, 0.86]
- Follows Stage C desired trajectory from JSON (148 points)
- Clearly distinct from chassis path (Z≈0.86 vs Z≈0.0)
- Shows what Stage C is supposed to track

**Visual clarity:**
- Red dot hidden during Stages A & B ✅
- Red dot appears at frame 97 at correct position ✅
- Red dot tracks Stage C desired trajectory ✅
- Clear distinction from chassis path ✅

---

## All Three Fixes Combined

This is the **third fix** in the animation debugging session:

### Fix 1: Stage Boundary Sampling (COMPLETE_TIMING_FIX.md)
- **Problem:** Stage boundaries in raw frame space, animation in sampled space
- **Fix:** Transform boundaries: `[51, 190, 339]` → `[26, 96, 171]`
- **Result:** Stage labels appear at correct times

### Fix 2: Red Dot Stage-Relative Indexing (STAGE_SYNC_FIX_COMPLETE.md)
- **Problem:** Red dot indexed global timeline, only had Stage C data
- **Fix:** Use `stageCPath(k - stageCFirstIdx + 1)` for stage-relative indexing
- **Result:** Red dot appears at frame 97, moves synchronously

### Fix 3: Stage C EE Path Data Source (This Fix)
- **Problem:** Red dot tracked full trajectory from homebase, not Stage C desired path
- **Fix:** Use `stageC.targetPositions` instead of `computeEEPath(full qTraj)`
- **Result:** Red dot shows correct Stage C desired trajectory

---

## Impact on Previous Animations

### All Previous Test Animations Need Regeneration

1. `TEST_STAGE_SYNC_FIXED.mp4` - Has Fix 1 & 2, missing Fix 3 ❌
2. `TEST_COMPLETE_FIX.mp4` - Has Fix 1 & 2, missing Fix 3 ❌
3. `TEST_STAGEC_PATH_FIX.mp4` - Has ALL THREE fixes ✅

### All 5 Parametric Study Animations

All need regeneration with all three fixes applied.

---

## Technical Details

### Data Structure in Logs

```matlab
log.stageLogs.stageC:
  - qTraj: 9 × 149 (joint trajectory)
  - eePositions: 3 × 148 (actual EE positions achieved)
  - targetPositions: 3 × 148 (desired EE from JSON) ✅ THIS ONE!
  - positionError: 3 × 148 (tracking error)
  - referenceInitialIk: (doesn't exist in our logs)

log.referenceTrajectory:
  - EndEffectorPositions: 3 × 148 (same as targetPositions)
```

**Note:** `log.referenceTrajectory.EndEffectorPositions` and `stageC.targetPositions` are identical in our case, both representing the JSON desired trajectory.

### Why `computeEEPath(qTraj)` Was Wrong

```matlab
computeEEPath(robot, qTraj)
  Input: qTraj = full trajectory (9 × 337, all stages A+B+C)
  Output: EE path for all 337 frames
  Result: [homebase] → [Stage A] → [Stage B] → [Stage C]
  
  When shown as "Stage C reference":
    - Starts from homebase (wrong!)
    - Includes Stages A & B motion (wrong!)
    - Only last ~149 frames are Stage C (correct)
```

**Correct approach:**
```matlab
Use stageC.targetPositions
  Input: Desired EE trajectory from JSON (3 × 148)
  Output: Exactly what Stage C should track
  Result: Stage C waypoints only [1.65, 0.08, 0.86] → ...
```

---

## Lessons Learned

### Why This Was Confusing

1. **Multiple EE trajectory sources:** 
   - Full trajectory (A+B+C)
   - Stage C desired (JSON)
   - Stage C actual (achieved)
   - Computed from joints

2. **Fallback logic was too aggressive:** Computed from full trajectory when Stage C data was actually available

3. **Field naming ambiguity:** `referenceInitialIk` doesn't exist, but `targetPositions` does (different name for same concept)

### Design Principle

**"Reference" for a stage should show what that stage is trying to achieve, not what happened before it.**

For Stage C visualization:
- ✅ **Show:** Stage C desired trajectory (what to track)
- ❌ **Don't show:** Full robot motion from homebase

---

## Testing

### Test Script: `test_stagec_path_fix.m`

**Validates:**
- Correct data source selected (`stageC.targetPositions`)
- Data matches `referenceTrajectory.EndEffectorPositions`
- First waypoint is at correct position [1.65, 0.08, 0.86]
- Generates test animation

### Verification Checklist

**In `TEST_STAGEC_PATH_FIX.mp4`:**
- [ ] Red dot appears at frame 97 (Stage C start)
- [ ] Red dot starts at [1.65, 0.08, 0.86] (NOT homebase)
- [ ] Red dot does NOT follow chassis path
- [ ] Red dot tracks JSON desired trajectory
- [ ] Red dot is ABOVE chassis (Z ≈ 0.86 m)
- [ ] Green square (actual EE) follows red dot

---

## Files Modified

1. `/matlab/+gik9dof/animateStagedWithHelper.m` (lines 42-74)
   - Replaced fallback logic with priority-based selection
   - Now uses `stageC.targetPositions` for Stage C reference

## Test Scripts Created

1. `debug_stagec_ee_path.m` - Investigates data sources
2. `test_stagec_path_fix.m` - Validates fix with test animation

## Documentation

1. `STAGEC_PATH_FIX.md` - This document

---

## Status

✅ **Fix implemented and tested**  
✅ **Test animation generated (9.0 MB)**  
⏳ **Awaiting user review**  
⏳ **Ready to regenerate all animations with all 3 fixes**

---

## Next Steps

1. **User:** Review `TEST_STAGEC_PATH_FIX.mp4` to confirm:
   - Red dot starts at Stage C first waypoint (not homebase)
   - Red dot is distinct from chassis path
   - Red dot tracks desired trajectory

2. **If all 3 fixes look good:** Regenerate parametric study animations:
   ```bash
   matlab -batch "generate_parametric_animations"
   ```

All 5 animations will now have:
- ✅ Correct stage boundary timing
- ✅ Red dot synchronized indexing
- ✅ Red dot showing Stage C desired trajectory

---

**User Contribution:** 🌟 **Critical observation that red dot was following full trajectory from homebase, not Stage C desired path**
