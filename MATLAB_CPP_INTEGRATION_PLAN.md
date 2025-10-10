# 🎯 MATLAB→C++ Integration Plan

**Date:** October 10, 2025  
**Branch:** merge-matlab-features → codegencc45-main  
**Objective:** Integrate new MATLAB features into C++ codegen pipeline

---

## 📋 Phase 1: Analysis & Assessment ✅ COMPLETE

### New MATLAB Features Merged:
1. ✅ **simulateChassisController.m** - Multi-mode controller wrapper
2. ✅ **rsClothoidRefine.m** - RS curve smoothing
3. ✅ **rsRefinePath.m** - Path refinement
4. ✅ **preparePathForFollower.m** - Path preparation
5. ✅ **loadChassisProfile.m** - Parameter loading
6. ✅ **defaultReedsSheppParams.m** - RS defaults
7. ✅ **visualizeStageBOccupancy.m** - Visualization (MATLAB only)
8. ✅ **plotJsonPath.m** - Plotting utility (MATLAB only)

### Modified Functions:
1. ✅ **unifiedChassisCtrl.m** - ⚠️ API CHANGE (params/state order)
2. ✅ **purePursuitFollower.m** - Enhanced tuning
3. ✅ **simulatePurePursuitExecution.m** - Better logic
4. ✅ **runStagedTrajectory.m** - 10Hz loop rate

---

## 📊 Phase 2: Codegen Impact Assessment ⏳ IN PROGRESS

### Current Codegen Components:
1. **arm64_realtime/** (196 files) - GIK solver
   - Source: `+codegen_inuse/solveGIKStepWrapper.m`
   - Status: ✅ Likely unaffected (core solver unchanged)
   - Action: **VERIFY ONLY** - no regeneration needed

2. **planner_arm64/** (50 files) - Path planner
   - Source: Hybrid A* planner functions
   - Status: ⚠️ **UNKNOWN** - planning functions were deleted on main
   - Action: **INVESTIGATE** - check if planner source still exists

3. **trajectory_smoothing/** (10 files) - Trajectory smoothing
   - Source: Previously `smoothTrajectoryVelocity.m` (DELETED on main)
   - Status: ⚠️ **AFFECTED** - source function deleted
   - Action: **INVESTIGATE** - is this component still needed?

4. **velocity_smoothing/** (30 files) - Velocity smoothing
   - Source: Previously `smoothVelocityCommand.m` (DELETED on main)
   - Status: ⚠️ **AFFECTED** - source function deleted
   - Action: **INVESTIGATE** - is this component still needed?

### New Components to Consider:
1. **Multi-mode chassis controller** - simulateChassisController.m
   - Codegen candidate? **YES** - If used in real-time control
   - Priority: **MEDIUM** - Depends on deployment needs

2. **RS path smoothing** - rsClothoidRefine.m, rsRefinePath.m
   - Codegen candidate? **YES** - If used in path planning
   - Priority: **HIGH** - Stage B improvement feature

3. **Path preparation** - preparePathForFollower.m
   - Codegen candidate? **YES** - If used in control loop
   - Priority: **MEDIUM** - Support function

---

## 🔍 Phase 3: Investigation Tasks ⏳ CURRENT PHASE

### Task 3.1: Check Planner Source Status
**Question:** How does planner codegen work after `planHybridAStarCodegen.m` deletion?

**Actions:**
1. Search for planner source in merged branch:
   ```bash
   find matlab/+gik9dof -name "*plan*.m" -o -name "*Astar*.m"
   ```

2. Check what planner codegen script uses:
   ```bash
   cat scripts/codegen/run_planner_codegen_wsl.sh
   ```

3. Examine planner codegen MATLAB script:
   ```bash
   # Look for generate_code_planner_arm64.m or similar
   ```

**Expected Outcomes:**
- A) Planner source moved to package namespace ✅ GOOD - keep using it
- B) Planner now codegen-only (no MATLAB source) ⚠️ OK - no changes needed
- C) Planner source completely removed ❌ BAD - need to restore or find alternative

---

### Task 3.2: Check Smoothing Function Status
**Question:** Why were smoothing functions deleted? What replaced them?

**Actions:**
1. Check if smoothing moved into controller:
   ```bash
   grep -r "smoothVelocity\|smoothTrajectory" matlab/+gik9dof/+control/
   ```

2. Check simulateChassisController implementation:
   ```matlab
   % Open simulateChassisController.m and check if it has built-in smoothing
   ```

3. Compare with old smoothVelocityCommand.m functionality:
   ```bash
   git show origin/main~8:matlab/+gik9dof/+control/smoothVelocityCommand.m
   ```

**Expected Outcomes:**
- A) Smoothing integrated into new controller ✅ GOOD - use new controller
- B) Smoothing no longer needed ⚠️ OK - verify with user
- C) Smoothing accidentally deleted ❌ BAD - need to restore

---

### Task 3.3: Verify API Breaking Changes
**Question:** Which code calls `unifiedChassisCtrl` with old API?

**Actions:**
1. Search for all callers:
   ```bash
   grep -rn "unifiedChassisCtrl" matlab/+gik9dof/
   ```

2. Check each caller's argument order:
   ```matlab
   % OLD: unifiedChassisCtrl(mode, ref, estPose, params, state)
   % NEW: unifiedChassisCtrl(mode, ref, estPose, state, params)
   ```

3. Update callers if found

**Expected Outcomes:**
- Identify all callers
- Update them to new API
- Test that changes work

---

### Task 3.4: Assess New Feature Codegen Needs
**Question:** Which new features need C++ codegen?

**Decision Matrix:**

| Feature | Real-time? | Codegen? | Priority |
|---------|-----------|----------|----------|
| simulateChassisController | YES | YES | HIGH |
| rsClothoidRefine | Depends | MAYBE | MEDIUM |
| rsRefinePath | Depends | MAYBE | MEDIUM |
| preparePathForFollower | Depends | MAYBE | LOW |
| loadChassisProfile | NO | NO | N/A |
| visualizeStageBOccupancy | NO | NO | N/A |
| plotJsonPath | NO | NO | N/A |

**Actions:**
1. Consult with user on deployment needs
2. Prioritize based on real-time requirements
3. Plan codegen schedule

---

## 🛠️ Phase 4: Codegen Regeneration Plan ⏳ PENDING

### Step 4.1: Verify Existing Components (PRIORITY 1)
**Objective:** Ensure current codegen still works after MATLAB merge

**Actions:**
```bash
# 1. Test ARM64 solver (should be unaffected)
wsl bash scripts/codegen/run_codegen_wsl_with_version.sh

# 2. Test planner (needs investigation first)
.\scripts\codegen\run_planner_codegen.ps1

# 3. Check trajectory smoothing (source may be deleted)
wsl bash scripts/codegen/run_trajectory_smoothing_codegen.sh

# 4. Check velocity smoothing (source may be deleted)
wsl bash scripts/codegen/run_velocity_smoothing_codegen.sh
```

**Success Criteria:**
- All 4 components generate without errors
- Build times similar to before (~27 min total)
- No MATLAB errors about missing functions

**If failures occur:**
- Document which component failed
- Identify root cause (missing source? API change?)
- Decide: fix, restore, or deprecate

---

### Step 4.2: Generate New Components (PRIORITY 2)
**Objective:** Create C++ codegen for new MATLAB features (if needed)

**Candidate 1: Multi-Mode Chassis Controller**
```matlab
% scripts/codegen/generate_code_chassis_controller.m
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenCodeOnly = true;
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';

codegen -config cfg ...
    gik9dof.control.simulateChassisController ...
    -args {zeros(100,3), struct(...)} ...
    -d codegen/chassis_controller_arm64 ...
    -report
```

**Candidate 2: RS Path Smoothing**
```matlab
% scripts/codegen/generate_code_rs_smoothing.m
% Generate rsClothoidRefine and rsRefinePath
```

**Decision Points:**
- Are these functions called in real-time control loop?
- Do they need to run on Jetson Orin?
- Can they run in MATLAB pre-processing instead?

---

### Step 4.3: Update Existing Components (PRIORITY 3)
**Objective:** Regenerate components affected by MATLAB changes

**Components to update:**
1. **planner_arm64** - If planner functions changed
2. **trajectory_smoothing** - If still using old source
3. **velocity_smoothing** - If still using old source

**Process:**
```bash
# For each affected component:
# 1. Backup current version
cp -r codegen/planner_arm64 codegen/planner_arm64_backup_$(date +%Y%m%d_%H%M%S)

# 2. Regenerate
.\scripts\codegen\run_planner_codegen.ps1

# 3. Verify output
diff -r codegen/planner_arm64 codegen/planner_arm64_backup_*/

# 4. Test build
wsl bash scripts/deployment/check_build_current_wsl.sh
```

---

## 🧪 Phase 5: Testing & Validation ⏳ PENDING

### Test 5.1: MATLAB Function Tests
**Objective:** Verify MATLAB code works correctly

**Actions:**
```matlab
% In MATLAB:
addpath(genpath('matlab'))

% Test new functions
result1 = gik9dof.control.simulateChassisController([0 0 0], struct(...));
result2 = gik9dof.control.rsClothoidRefine([...], struct(...));
result3 = gik9dof.control.preparePathForFollower([...]);

% Test modified functions
[cmd, state] = gik9dof.control.unifiedChassisCtrl("holistic", ref, [0 0 0], state, params);
```

**Success Criteria:**
- No MATLAB errors
- Functions return expected structures
- Help documentation works

---

### Test 5.2: Codegen Build Tests
**Objective:** Verify all codegen builds successfully

**Actions:**
```bash
# Test all 4 existing components
wsl bash scripts/codegen/run_codegen_wsl_with_version.sh
.\scripts\codegen\run_planner_codegen.ps1
wsl bash scripts/codegen/run_trajectory_smoothing_codegen.sh
wsl bash scripts/codegen/run_velocity_smoothing_codegen.sh

# Record build times
# Compare with baseline (~27 min total)
```

**Success Criteria:**
- All builds complete without errors
- Build times within 20% of baseline
- No new warnings

---

### Test 5.3: Integration Tests (if applicable)
**Objective:** Test C++ code integrates with ROS2

**Actions:**
```bash
# Copy to ROS2 workspace
bash scripts/deployment/deploy_to_ros2_workspace.sh

# Build ROS2 packages
cd ros2_ws
colcon build --packages-select gik9dof_solver

# Run tests
colcon test --packages-select gik9dof_solver
```

**Success Criteria:**
- ROS2 package builds
- No linking errors
- Tests pass (if any)

---

## 📝 Phase 6: Documentation & Finalization ⏳ PENDING

### Doc 6.1: Update Build Guides
**Files to update:**
- `docs/guides/COMPLETE_BUILD_GUIDE.md` - Add new components
- `docs/technical/codegen/CODEGEN_FOLDER_STRUCTURE.md` - Document new folders
- `README.md` - Update component list

**Changes needed:**
- Add new component build instructions
- Update component count (4 → 5 or 6?)
- Document new build times

---

### Doc 6.2: Create Feature Integration Guide
**New file:** `docs/guides/MATLAB_FEATURE_INTEGRATION.md`

**Contents:**
- Summary of new MATLAB features
- Which features have C++ codegen
- How to use new controller modes
- API breaking changes and migration guide
- Examples and usage patterns

---

### Doc 6.3: Update Session Summary
**File:** `MATLAB_FEATURE_INTEGRATION_COMPLETE.md`

**Contents:**
- What was integrated
- Which components regenerated
- Build times comparison
- Known issues
- Next steps

---

## 🎯 Phase 7: Merge to Main ⏳ PENDING

### Step 7.1: Final Verification
```bash
# 1. All tests pass
# 2. Documentation updated
# 3. No uncommitted changes
git status
```

### Step 7.2: Merge Branch
```bash
git checkout codegencc45-main
git merge --no-ff merge-matlab-features \
  -m "Integrate MATLAB feature improvements with C++ codegen

Merged new MATLAB features from origin/main:
- Multi-mode chassis controller
- RS clothoid path smoothing
- Enhanced pure pursuit control
- Better visualization utilities

Codegen Status:
- Verified existing 4 components still work
- Generated [X] new components
- Total build time: ~[X] minutes

API Breaking Changes:
- unifiedChassisCtrl argument order changed

All tests passing. Documentation updated."
```

### Step 7.3: Push to Remote
```bash
git push origin codegencc45-main
```

---

## 🚀 Execution Checklist

### Phase 3: Investigation (CURRENT) ⏳
- [ ] Task 3.1: Check planner source status
- [ ] Task 3.2: Check smoothing function status  
- [ ] Task 3.3: Verify API breaking changes
- [ ] Task 3.4: Assess new feature codegen needs

### Phase 4: Codegen ⏳
- [ ] Step 4.1: Verify existing 4 components
- [ ] Step 4.2: Generate new components (if needed)
- [ ] Step 4.3: Update affected components

### Phase 5: Testing ⏳
- [ ] Test 5.1: MATLAB function tests
- [ ] Test 5.2: Codegen build tests
- [ ] Test 5.3: Integration tests

### Phase 6: Documentation ⏳
- [ ] Doc 6.1: Update build guides
- [ ] Doc 6.2: Create feature integration guide
- [ ] Doc 6.3: Update session summary

### Phase 7: Merge ⏳
- [ ] Step 7.1: Final verification
- [ ] Step 7.2: Merge to codegencc45-main
- [ ] Step 7.3: Push to remote

---

## ⚡ Quick Start - Next Actions

**IMMEDIATE (Now):**
```bash
# 1. Start investigation - find planner source
find matlab/+gik9dof -name "*plan*.m" -o -name "*Astar*.m" | head -20

# 2. Check what our planner script uses
cat matlab/+gik9dof/planHybridAStarCodegen.m 2>/dev/null || echo "File not found"

# 3. Search for smoothing in new controller
grep -n "smooth\|jerk\|accel" matlab/+gik9dof/+control/simulateChassisController.m
```

**SHORT TERM (Today):**
- Complete Phase 3 investigation
- Decide which components need regeneration
- Test existing codegen components

**MEDIUM TERM (This session):**
- Regenerate affected components
- Generate new components (if decided)
- Run all tests

**LONG TERM (Next session):**
- Final integration testing
- Documentation updates
- Merge to main branch

---

**Status:** 📋 **PLAN READY - STARTING PHASE 3 INVESTIGATION**
