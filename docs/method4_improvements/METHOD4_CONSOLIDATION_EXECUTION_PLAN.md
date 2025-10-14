# Method 4 (ppFirst) Consolidation - Execution Plan

**Date:** October 13, 2025  
**Status:** 🚧 READY FOR EXECUTION  
**Priority:** HIGH

---

## Executive Summary

Method 4 (ppFirst mode in Stage C) is **functionally complete** but requires consolidation to be production-ready. This plan addresses:

1. ✅ **Configuration Integration** - YAML parameters already exist, code needs updating
2. 🔧 **Code Bugs Fixed** - Path refinement argument mismatch resolved
3. ⚠️ **Performance Issue** - Method 1 taking 40 minutes (needs investigation)
4. 📊 **Comparison Framework** - Script exists but needs Method 1 performance fix
5. 📝 **Documentation** - Update guides to reflect ppFirst availability

**Key Finding:** The YAML configuration for ppFirst parameters is **ALREADY IMPLEMENTED** in `pipeline_profiles.yaml` (lines 110-117), but the code still uses hardcoded values!

---

## Current Status Assessment

### ✅ What's Already Done

1. **YAML Configuration Structure** (✅ COMPLETE)
   - `config/pipeline_profiles.yaml` has `stage_c.ppfirst` section
   - Parameters defined: `yaw_corridor_deg`, `position_tolerance`, `ee_error_threshold`
   - Profile variants: default (15°), aggressive (20°), conservative (10°)

2. **Core Implementation** (✅ COMPLETE)
   - 5 helper functions implemented and tested
   - Integration in `runStagedTrajectory.m` working
   - `executeStageCPPFirst` wrapper functional

3. **Comparison Script** (✅ EXISTS, needs Method 1 optimization)
   - `compare_method1_vs_method4.m` created
   - Metrics computation implemented
   - Plotting and reporting functional

### 🔧 Issues Fixed Today

1. **Path Refinement Bug** (✅ FIXED)
   - `initPPFromBasePath.m` calling `rsClothoidRefine` with wrong arguments
   - Fixed: Added proper environment and clothoid parameters
   
2. **Comparison Script Variable Error** (✅ FIXED)
   - Method 1 section was commented out causing `elapsed1` undefined
   - Fixed: Uncommented Method 1 execution block

### ⚠️ Critical Issues Requiring Attention

1. **Method 1 Performance** (🔴 URGENT)
   - **Problem:** Method 1 takes ~40 minutes to complete
   - **Expected:** Should take 5-10 minutes max
   - **Impact:** Blocks comparison studies
   - **Root Cause:** Unknown - needs profiling

2. **Hardcoded Parameters in Code** (🟡 HIGH PRIORITY)
   - **Problem:** `executeStageCPPFirst` uses hardcoded values despite YAML existing
   - **Location:** `runStagedTrajectory.m` lines 879-882
   - **Impact:** Users cannot tune ppFirst parameters via profiles

---

## Execution Plan

### Phase 1: URGENT - Fix Method 1 Performance (Priority: 🔴 CRITICAL)

**Estimated Time:** 2-4 hours (investigation + fix)

#### Step 1.1: Profile Method 1 Execution
```matlab
% Add profiling to identify bottleneck
profile on
log1 = gik9dof.trackReferenceTrajectory('Mode', 'staged', 'ExecutionMode', 'ppForIk', ...);
profile viewer
```

**Potential Causes:**
- Stage B planning taking excessive time (Hybrid A*)
- GIK solver not converging (hitting max iterations repeatedly)
- Excessive logging or visualization overhead
- Memory allocation issues in large trajectory

#### Step 1.2: Quick Performance Test
Create a short trajectory test to isolate the issue:

```matlab
% test_method1_performance.m
% Test with first 20 waypoints only
robot = gik9dof.createRobotModel();
traj = loadTrajectory('1_pull_world_scaled.json');
traj.Poses = traj.Poses(:, :, 1:20);  % First 20 waypoints only

tic;
log_short = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppForIk', 'Verbose', true);
elapsed_short = toc;

fprintf('Time for 20 waypoints: %.2f seconds\n', elapsed_short);
fprintf('Estimated time for 148 waypoints: %.2f minutes\n', ...
    elapsed_short * 148 / 20 / 60);
```

#### Step 1.3: Common Performance Issues & Fixes

| Issue | Symptom | Solution |
|-------|---------|----------|
| Hybrid A* timeout | Stage B takes >5 min | Reduce `hybrid_resolution` or disable |
| GIK not converging | Hitting 1500 iterations | Check initial conditions, reduce `MaxIterations` for testing |
| Verbose output | Console spam | Set `Verbose=false` |
| Animation during sim | Real-time rendering | Disable live animation |
| Large distance specs | Many obstacles | Reduce `DistanceMargin` or obstacle count |

**Action Items:**
- [ ] Run profiler on Method 1
- [ ] Test with 20 waypoints to isolate issue
- [ ] Check if Stage B or Stage C is the bottleneck
- [ ] Implement fix based on findings

---

### Phase 2: Integrate YAML Configuration into Code (Priority: 🟡 HIGH)

**Estimated Time:** 1-2 hours

The YAML configuration **already exists**, we just need to wire it into the code!

#### Step 2.1: Update `executeStageCPPFirst` to Read from Config

**File:** `matlab/+gik9dof/runStagedTrajectory.m`  
**Lines:** 879-882

**Current Code (HARDCODED):**
```matlab
% Method 4 specific: yaw tolerance (corridor half-width) and position tolerance
ppFirstOpts.YawTolerance = deg2rad(15);  % ±15° corridor around PP prediction
ppFirstOpts.PositionTolerance = 0.15;   % ±15cm box around PP prediction
ppFirstOpts.EEErrorTolerance = 0.01;    % 10mm threshold for fallback trigger
```

**Updated Code (FROM CONFIG):**
```matlab
% Method 4 specific: Extract from PipelineConfig if available, else use defaults
if isfield(options, 'PipelineConfig') && ...
   isfield(options.PipelineConfig, 'stage_c') && ...
   isfield(options.PipelineConfig.stage_c, 'ppfirst')
    ppfCfg = options.PipelineConfig.stage_c.ppfirst;
    ppFirstOpts.YawTolerance = deg2rad(ppfCfg.yaw_corridor_deg);
    ppFirstOpts.PositionTolerance = ppfCfg.position_tolerance;
    ppFirstOpts.EEErrorTolerance = ppfCfg.ee_error_threshold;
    ppFirstOpts.ApplyRefinement = ppfCfg.enable_refinement;
else
    % Fallback to defaults if config not provided
    ppFirstOpts.YawTolerance = deg2rad(15);
    ppFirstOpts.PositionTolerance = 0.15;
    ppFirstOpts.EEErrorTolerance = 0.01;
end
```

#### Step 2.2: Add Direct Override Arguments (Optional)

For users who want to override without creating new profiles:

**Add to `runStagedTrajectory` arguments block (~line 60):**
```matlab
% Stage C: PP-First (Method 4) parameters
options.StageCPPFirstYawCorridor (1,1) double = NaN      % deg (NaN = use config)
options.StageCPPFirstPositionTolerance (1,1) double = NaN  % m
options.StageCPPFirstEEErrorThreshold (1,1) double = NaN   % m
```

**In executeStageCPPFirst, add override logic:**
```matlab
% Direct overrides take precedence over config
if ~isnan(options.StageCPPFirstYawCorridor)
    ppFirstOpts.YawTolerance = deg2rad(options.StageCPPFirstYawCorridor);
elseif isfield(options, 'PipelineConfig') && ...
       isfield(options.PipelineConfig.stage_c, 'ppfirst')
    ppFirstOpts.YawTolerance = deg2rad(options.PipelineConfig.stage_c.ppfirst.yaw_corridor_deg);
else
    ppFirstOpts.YawTolerance = deg2rad(15);  % Default
end
```

**Action Items:**
- [ ] Update `executeStageCPPFirst` to read from PipelineConfig
- [ ] Add argument validation for direct overrides (optional)
- [ ] Test with all three profiles (default, aggressive, conservative)
- [ ] Verify fallback to defaults when config missing

---

### Phase 3: Testing & Validation (Priority: 🟢 MEDIUM)

**Estimated Time:** 2-3 hours

#### Test Matrix

| Test Case | Config Source | Expected Yaw Corridor | Pass? |
|-----------|---------------|----------------------|-------|
| Default profile | YAML default | 15° | ☐ |
| Aggressive profile | YAML aggressive | 20° | ☐ |
| Conservative profile | YAML conservative | 10° | ☐ |
| Direct override | options.StageCPPFirstYawCorridor | 25° | ☐ |
| No config provided | Fallback | 15° | ☐ |

#### Test Script
```matlab
% test_method4_config_integration.m
robot = gik9dof.createRobotModel();
traj = loadTrajectory('1_pull_world_scaled.json');
traj.Poses = traj.Poses(:, :, 1:10);  % Short test

%% Test 1: Default profile
cfg_default = gik9dof.loadPipelineProfile('default');
result1 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', 'PipelineConfig', cfg_default);
fprintf('Test 1 - Default: Yaw corridor = %.1f°\n', ...
    rad2deg(result1.stageLogs.stageC.diagnostics.parameters.yawTolerance));

%% Test 2: Aggressive profile
cfg_agg = gik9dof.loadPipelineProfile('aggressive');
result2 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', 'PipelineConfig', cfg_agg);
fprintf('Test 2 - Aggressive: Yaw corridor = %.1f°\n', ...
    rad2deg(result2.stageLogs.stageC.diagnostics.parameters.yawTolerance));

%% Test 3: Direct override
result3 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', 'StageCPPFirstYawCorridor', 25.0);
fprintf('Test 3 - Override: Yaw corridor = %.1f°\n', ...
    rad2deg(result3.stageLogs.stageC.diagnostics.parameters.yawTolerance));
```

**Action Items:**
- [ ] Create test script
- [ ] Run all test cases
- [ ] Verify parameters logged correctly in diagnostics
- [ ] Check fallback behavior when config missing

---

### Phase 4: Documentation Updates (Priority: 🟢 MEDIUM)

**Estimated Time:** 1-2 hours

#### Files to Update

1. **METHOD4_CONSOLIDATION_PLAN.md** ✅ Already exists
   - Mark sections as COMPLETE
   - Update status to "Configuration integrated"

2. **METHOD4_IMPLEMENTATION_STATUS.md** ✅ Already exists
   - Update "Configuration Consolidation" section
   - Change status to "COMPLETE"

3. **projectDiagnosis.md** (Section 10)
   - Add note about ppFirst parameters in YAML
   - Document direct override options

4. **SIMULATION_WORKFLOW_GUIDE.md** (if exists)
   - Add Method 4 usage examples
   - Show how to tune ppFirst parameters

5. **Create: METHOD4_PARAMETER_TUNING_GUIDE.md** (NEW)
   ```markdown
   # Method 4 (ppFirst) Parameter Tuning Guide
   
   ## Quick Start
   Use profiles:
   - `default`: 15° corridor, balanced
   - `aggressive`: 20° corridor, faster
   - `conservative`: 10° corridor, precise
   
   ## Parameters
   
   ### yaw_corridor_deg (default: 15°)
   - **What:** Half-width of yaw constraint corridor
   - **Impact:** Wider = more freedom, higher fallback risk
   - **Tune:** Increase if fallback rate >20%, decrease if base drifts
   
   ### position_tolerance (default: 0.15m)
   - **What:** XY box size around PP prediction
   - **Impact:** Larger = more spatial freedom
   - **Tune:** Decrease in tight spaces, increase in open areas
   
   ### ee_error_threshold (default: 0.010m = 10mm)
   - **What:** EE error trigger for fallback
   - **Impact:** Lower = stricter, more fallbacks
   - **Tune:** Decrease for precision tasks, increase for speed
   ```

**Action Items:**
- [ ] Update existing documentation
- [ ] Create tuning guide
- [ ] Add usage examples to README
- [ ] Update method comparison table in projectDiagnosis.md

---

### Phase 5: Method 1 vs Method 4 Comparison (Priority: 🟢 LOW - After Method 1 fixed)

**Estimated Time:** 30 minutes (once Method 1 performance fixed)

#### Run Comparison
```bash
cd /Users/yanbo/Projects/gikWBC9DOF
matlab -batch "compare_method1_vs_method4"
```

#### Expected Output
- Results in `results/<timestamp>_method_comparison/`
- Comparison plots (EE error, base paths)
- Metrics report (text file)

#### Success Criteria
- Both methods complete in <10 minutes each
- EE error difference <2mm RMS
- Fallback rate <25% for Method 4
- No crashes or errors

**Action Items:**
- [ ] Fix Method 1 performance first (Phase 1)
- [ ] Run full comparison
- [ ] Generate plots and report
- [ ] Add comparison results to METHOD4_IMPLEMENTATION_STATUS.md

---

## Implementation Checklist

### Immediate (Today)
- [x] Fix path refinement bug in initPPFromBasePath.m ✅
- [x] Fix comparison script variable error ✅
- [ ] 🔴 **URGENT:** Profile and fix Method 1 performance issue
- [ ] Test Method 1 with 20 waypoints to isolate bottleneck

### Short-term (This Week)
- [ ] Update executeStageCPPFirst to read from PipelineConfig
- [ ] Add direct override arguments (optional)
- [ ] Run configuration integration tests
- [ ] Update documentation

### Medium-term (Next Week)
- [ ] Run full Method 1 vs Method 4 comparison
- [ ] Create parameter tuning guide
- [ ] Generate comparison animations
- [ ] Write final status report

---

## Performance Investigation: Method 1 Taking 40 Minutes

### Hypothesis 1: Stage B Planning Bottleneck
**Check:** 
```matlab
% In trackReferenceTrajectory, check Stage B time
tic; stageB = runStageB(...); tStageB = toc;
```
**Expected:** <1 minute for Hybrid A*  
**If >5 minutes:** Disable Hybrid A* or reduce resolution

### Hypothesis 2: GIK Not Converging
**Check:**
```matlab
% After Stage C, check convergence rate
convergenceRate = sum(log.stageLogs.stageC.successMask) / length(log.stageLogs.stageC.successMask);
fprintf('Convergence: %.1f%%\n', convergenceRate * 100);
```
**Expected:** >90% convergence  
**If <80%:** Reduce MaxIterations or check initial configuration

### Hypothesis 3: Pure Pursuit Simulation Slow
**Check:**
```matlab
% In runStagedTrajectory, time Pass 2 (chassis simulation)
tic; chassisSim = simulateChassisExecution(...); tPass2 = toc;
```
**Expected:** <30 seconds  
**If >5 minutes:** Check path complexity or controller settings

### Hypothesis 4: Excessive Waypoints in Pass Iterations
**Check:**
```matlab
% Count waypoints in each pass
fprintf('Pass 1 waypoints: %d\n', size(pass1.qTraj, 2));
fprintf('Pass 2 waypoints: %d\n', size(pass2.basePath, 1));
fprintf('Pass 3 waypoints: %d\n', size(pass3.qTraj, 2));
```
**Expected:** ~150 waypoints per pass  
**If >500:** Path interpolation or resampling issue

---

## Quick Fixes to Try First

### Fix 1: Disable Verbose Output
```matlab
log1 = gik9dof.trackReferenceTrajectory('Mode', 'staged', 'Verbose', false, ...);
```

### Fix 2: Reduce GIK Iterations (for testing)
```matlab
log1 = gik9dof.trackReferenceTrajectory('Mode', 'staged', 'MaxIterations', 500, ...);
```

### Fix 3: Simplify Stage B
```matlab
log1 = gik9dof.trackReferenceTrajectory('Mode', 'staged', ...
    'UseStageBHybridAStar', false, ...  % Use simple path
    'StageBMode', 'pureHyb', ...);
```

### Fix 4: Test with Subset of Trajectory
```matlab
% Only test first 30 waypoints
traj_short = trajStruct;
traj_short.Poses = trajStruct.Poses(:, :, 1:30);
log1 = gik9dof.runStagedTrajectory(robot, traj_short, 'ExecutionMode', 'ppForIk');
```

---

## Files Modified/Created

### Modified
1. ✅ `compare_method1_vs_method4.m` - Uncommented Method 1, fixed variable names
2. ✅ `matlab/+gik9dof/initPPFromBasePath.m` - Fixed path refinement arguments
3. ⏳ `matlab/+gik9dof/runStagedTrajectory.m` - Will update executeStageCPPFirst

### Created
1. ✅ This document: `METHOD4_CONSOLIDATION_EXECUTION_PLAN.md`
2. ⏳ `test_method4_config_integration.m` - Configuration tests
3. ⏳ `test_method1_performance.m` - Performance investigation
4. ⏳ `METHOD4_PARAMETER_TUNING_GUIDE.md` - User guide

---

## Timeline Summary

| Phase | Task | Priority | Time | Status |
|-------|------|----------|------|--------|
| 1 | Fix Method 1 performance | 🔴 URGENT | 2-4h | ⏳ TODO |
| 2 | Integrate YAML config | 🟡 HIGH | 1-2h | ⏳ TODO |
| 3 | Testing & validation | 🟢 MEDIUM | 2-3h | ⏳ TODO |
| 4 | Documentation updates | 🟢 MEDIUM | 1-2h | ⏳ TODO |
| 5 | Full comparison study | 🟢 LOW | 0.5h | ⏳ BLOCKED |

**Total Estimated Time:** 6.5-11.5 hours (1-2 days)  
**Critical Path:** Fix Method 1 performance → Enable comparison studies

---

## Next Actions

### For You (User):
1. **Immediate:** Run performance profiler on Method 1
   ```matlab
   profile on
   log1 = gik9dof.trackReferenceTrajectory('Mode', 'staged', 'ExecutionMode', 'ppForIk', 'Verbose', false);
   profile viewer
   ```

2. **Quick Test:** Try Method 1 with only 20-30 waypoints to see if it's linear scaling
   ```matlab
   test_method1_performance  % Script I'll create
   ```

3. **Decision Point:** Once Method 1 is fixed, decide consolidation priority:
   - Option A: Complete config integration first (Phase 2)
   - Option B: Run comparison with current state, then integrate config
   - **Recommendation:** Option A (clean up code before comparison)

### For Me (Assistant):
1. Create performance investigation script
2. Create config integration test script
3. Prepare code changes for Phase 2 (pending approval)

---

**Status:** 📋 PLAN COMPLETE, READY FOR EXECUTION  
**Blocking Issue:** Method 1 performance (40 minutes → should be <10 minutes)  
**Priority:** Fix Method 1 first, then proceed with consolidation

