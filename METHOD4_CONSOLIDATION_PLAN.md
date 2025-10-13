# Method 4 Consolidation Plan

**Purpose:** Migrate Method 4 from "functionally complete" to "production ready"  
**Timeline:** 2-3 days  
**Status:** 📋 PLANNED

---

## Overview

Method 4 implementation is functionally complete but has **3 configuration parameters hardcoded** in `executeStageCPPFirst.m`. This plan consolidates those parameters into `pipeline_profiles.yaml` for consistency with the rest of the system.

---

## Current State

### Hardcoded Parameters (Lines 879-882 in runStagedTrajectory.m)

```matlab
function logC = executeStageCPPFirst(...)
    % ...
    
    % Method 4 specific: yaw tolerance (corridor half-width) and position tolerance
    ppFirstOpts.YawTolerance = deg2rad(15);  % ±15° corridor around PP prediction
    ppFirstOpts.PositionTolerance = 0.15;    % ±15cm box around PP prediction
    ppFirstOpts.EEErrorTolerance = 0.01;     % 10mm threshold for fallback trigger
    
    % ...
end
```

### Issues

1. **Not tunable:** Users cannot adjust corridor width without editing code
2. **Inconsistent:** Other stage parameters live in `pipeline_profiles.yaml`
3. **Profile-agnostic:** "aggressive" profile can't use wider corridor
4. **Undiscoverable:** Users unaware these parameters exist

---

## Target State

### YAML Configuration

```yaml
# In config/pipeline_profiles.yaml

profiles:
  default:
    # ... existing params ...
    
    stage_c:
      # ... existing Stage C params ...
      
      # PP-First (Method 4) specific parameters
      ppfirst:
        yaw_corridor_deg: 15.0          # ±15° around PP prediction
        position_tolerance: 0.15        # ±0.15m box around PP position (m)
        ee_error_threshold: 0.010       # 10mm fallback trigger (m)
        enable_refinement: false        # Apply RS+Clothoid to seed path
        adaptive_corridor: false        # Scale corridor by curvature (future)
        adaptive_scale_factor: 2.0      # Curvature multiplier (future)

  aggressive:
    inherits: "default"
    overrides:
      stage_c:
        ppfirst:
          yaw_corridor_deg: 20.0        # Wider corridor for faster motion
          position_tolerance: 0.20      # Larger position box
          ee_error_threshold: 0.015     # More tolerant fallback

  conservative:
    inherits: "default"
    overrides:
      stage_c:
        ppfirst:
          yaw_corridor_deg: 10.0        # Tighter corridor for precision
          position_tolerance: 0.10      # Smaller position box
          ee_error_threshold: 0.008     # Stricter fallback threshold
```

### Code Changes

```matlab
% In executeStageCPPFirst (runStagedTrajectory.m)

function logC = executeStageCPPFirst(robot, trajStruct, qStart, baseIdx, armIdx, 
                                     velLimits, chassisParams, alignmentInfo, 
                                     options, stageBResult)
    % ...
    
    % Extract PP-First parameters from options (with fallbacks)
    if isfield(options, 'StageCPPFirstYawCorridor')
        yawCorridorDeg = options.StageCPPFirstYawCorridor;
    elseif isfield(options, 'PipelineConfig') && ...
           isfield(options.PipelineConfig, 'stage_c') && ...
           isfield(options.PipelineConfig.stage_c, 'ppfirst')
        yawCorridorDeg = options.PipelineConfig.stage_c.ppfirst.yaw_corridor_deg;
    else
        yawCorridorDeg = 15.0;  % Default fallback
    end
    
    if isfield(options, 'StageCPPFirstPositionTolerance')
        positionTolerance = options.StageCPPFirstPositionTolerance;
    elseif isfield(options, 'PipelineConfig') && ...
           isfield(options.PipelineConfig, 'stage_c') && ...
           isfield(options.PipelineConfig.stage_c, 'ppfirst')
        positionTolerance = options.PipelineConfig.stage_c.ppfirst.position_tolerance;
    else
        positionTolerance = 0.15;  % Default fallback
    end
    
    if isfield(options, 'StageCPPFirstEEErrorThreshold')
        eeErrorThreshold = options.StageCPPFirstEEErrorThreshold;
    elseif isfield(options, 'PipelineConfig') && ...
           isfield(options.PipelineConfig, 'stage_c') && ...
           isfield(options.PipelineConfig.stage_c, 'ppfirst')
        eeErrorThreshold = options.PipelineConfig.stage_c.ppfirst.ee_error_threshold;
    else
        eeErrorThreshold = 0.01;  % Default fallback
    end
    
    ppFirstOpts.YawTolerance = deg2rad(yawCorridorDeg);
    ppFirstOpts.PositionTolerance = positionTolerance;
    ppFirstOpts.EEErrorTolerance = eeErrorThreshold;
    
    % Refinement flag
    if isfield(options, 'StageCPPFirstApplyRefinement')
        ppFirstOpts.ApplyRefinement = options.StageCPPFirstApplyRefinement;
    elseif isfield(options, 'PipelineConfig') && ...
           isfield(options.PipelineConfig, 'stage_c') && ...
           isfield(options.PipelineConfig.stage_c, 'ppfirst')
        ppFirstOpts.ApplyRefinement = options.PipelineConfig.stage_c.ppfirst.enable_refinement;
    else
        ppFirstOpts.ApplyRefinement = options.StageCUseBaseRefinement;
    end
    
    % ... rest of function unchanged ...
end
```

### runStagedTrajectory.m Argument Additions

```matlab
function pipeline = runStagedTrajectory(robot, trajStruct, options)
arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    
    % ... existing arguments ...
    
    % Stage C: PP-First (Method 4) parameters
    options.StageCPPFirstYawCorridor (1,1) double = 15.0      % deg
    options.StageCPPFirstPositionTolerance (1,1) double = 0.15  % m
    options.StageCPPFirstEEErrorThreshold (1,1) double = 0.01   % m
    options.StageCPPFirstApplyRefinement (1,1) logical = false
end
```

---

## Implementation Steps

### Step 1: Update pipeline_profiles.yaml (30 min)

**File:** `config/pipeline_profiles.yaml`

**Action:**
1. Open file in editor
2. Locate `stage_c:` section in `default` profile
3. Add `ppfirst:` subsection with 5 parameters
4. Add overrides in `aggressive` and `conservative` profiles
5. Save and validate YAML syntax

**Validation:**
```matlab
% Test YAML parsing
cfg = gik9dof.loadPipelineProfile('default');
assert(isfield(cfg.stage_c, 'ppfirst'), 'Missing ppfirst section');
assert(cfg.stage_c.ppfirst.yaw_corridor_deg == 15.0);
```

---

### Step 2: Update executeStageCPPFirst (1 hour)

**File:** `matlab/+gik9dof/runStagedTrajectory.m`

**Action:**
1. Locate `executeStageCPPFirst` function (line 856)
2. Replace hardcoded values (lines 879-882) with config extraction logic
3. Add fallback chain: options → PipelineConfig → defaults
4. Test all 3 fallback paths

**Test Cases:**
```matlab
% Case 1: Direct option override
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', ...
    'StageCPPFirstYawCorridor', 20.0);  % Should use 20°

% Case 2: Profile-based
cfg = gik9dof.loadPipelineProfile('aggressive');
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', ...
    'PipelineConfig', cfg);  % Should use 20° from aggressive profile

% Case 3: Default fallback
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst');  % Should use 15° default
```

---

### Step 3: Add Arguments to runStagedTrajectory (15 min)

**File:** `matlab/+gik9dof/runStagedTrajectory.m`

**Action:**
1. Locate `arguments` block (line ~60)
2. Add 4 new options entries (see "Target State" above)
3. Document in function header

**Documentation:**
```matlab
%   options.StageCPPFirstYawCorridor (1,1) double = 15.0
%       Yaw corridor half-width for Method 4 (degrees).
%       GIK constrained to θ ∈ [θ_pp - corridor, θ_pp + corridor].
%
%   options.StageCPPFirstPositionTolerance (1,1) double = 0.15
%       Position box half-width for Method 4 (m).
%       GIK constrained to [x, y] ∈ [x_pp ± tol, y_pp ± tol].
%
%   options.StageCPPFirstEEErrorThreshold (1,1) double = 0.01
%       EE tracking error threshold for fallback (m).
%       If GIK solution exceeds this, triggers arm-only fallback.
%
%   options.StageCPPFirstApplyRefinement (1,1) logical = false
%       Apply RS+Clothoid smoothing to base seed path.
```

---

### Step 4: Update loadPipelineProfile Validation (30 min)

**File:** `matlab/+gik9dof/loadPipelineProfile.m`

**Action:**
1. Add validation check for `stage_c.ppfirst` section existence
2. Add warning if `ppfirst` params missing (for old YAML files)
3. Optionally add defaults if missing

**Code:**
```matlab
% In loadPipelineProfile.m, after loading config

% Validate ppfirst section exists (optional, for backward compatibility)
if ~isfield(cfg.stage_c, 'ppfirst')
    warning('loadPipelineProfile:MissingPPFirst', ...
        'Profile "%s" missing stage_c.ppfirst section. Using defaults.', profileName);
    cfg.stage_c.ppfirst = struct(...
        'yaw_corridor_deg', 15.0, ...
        'position_tolerance', 0.15, ...
        'ee_error_threshold', 0.010, ...
        'enable_refinement', false, ...
        'adaptive_corridor', false, ...
        'adaptive_scale_factor', 2.0);
end

% Validate numeric ranges
assert(cfg.stage_c.ppfirst.yaw_corridor_deg > 0 && ...
       cfg.stage_c.ppfirst.yaw_corridor_deg <= 90, ...
       'yaw_corridor_deg must be in (0, 90] degrees');
assert(cfg.stage_c.ppfirst.position_tolerance > 0 && ...
       cfg.stage_c.ppfirst.position_tolerance <= 1.0, ...
       'position_tolerance must be in (0, 1.0] meters');
assert(cfg.stage_c.ppfirst.ee_error_threshold > 0 && ...
       cfg.stage_c.ppfirst.ee_error_threshold <= 0.1, ...
       'ee_error_threshold must be in (0, 0.1] meters (100mm)');
```

---

### Step 5: Update Documentation (1 hour)

**Files to Update:**

1. **codexMethod4Plan.md**:
   - Mark Section 1 as COMPLETE
   - Add "IMPLEMENTED" status at top
   - Document parameter locations

2. **SIMULATION_WORKFLOW_GUIDE.md** (if exists):
   - Add Method 4 parameter tuning section
   - Example: "Adjusting PP-First corridor width"

3. **README.md**:
   - Mention Method 4 availability
   - Link to configuration guide

4. **UNIFIED_CONFIG_QUICK_REF.md** (if exists):
   - Add `stage_c.ppfirst.*` entries

**Example Documentation Addition:**
```markdown
## Method 4 (PP-First) Configuration

### Parameters

Located in `config/pipeline_profiles.yaml` under `stage_c.ppfirst`:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaw_corridor_deg` | double | 15.0 | Yaw corridor half-width (degrees) |
| `position_tolerance` | double | 0.15 | Position box half-width (meters) |
| `ee_error_threshold` | double | 0.010 | Fallback trigger threshold (meters) |
| `enable_refinement` | bool | false | Apply RS+Clothoid to seed path |

### Tuning Guide

**Tight maneuvers:** Increase `yaw_corridor_deg` to 20-25° to reduce fallback rate.

**High-precision tasks:** Decrease `ee_error_threshold` to 0.005m (5mm).

**Obstacle-dense environments:** Decrease `position_tolerance` to 0.10m for tighter base constraints.

### Usage

```matlab
% Method 1: Use profile
cfg = gik9dof.loadPipelineProfile('aggressive');
result = gik9dof.runStagedReference('PipelineConfig', cfg, 'ExecutionMode', 'ppFirst');

% Method 2: Direct override
result = gik9dof.runStagedReference('ExecutionMode', 'ppFirst', ...
    'StageCPPFirstYawCorridor', 20.0);
```
```

---

### Step 6: Testing (1 hour)

**Test Matrix:**

| Test Case | Config Source | Expected Yaw Corridor | Pass? |
|-----------|---------------|----------------------|-------|
| Default profile | YAML default | 15° | ☐ |
| Aggressive profile | YAML aggressive | 20° | ☐ |
| Conservative profile | YAML conservative | 10° | ☐ |
| Direct override | options | 25° | ☐ |
| Override + profile | options (priority) | 25° | ☐ |
| Missing YAML section | Fallback | 15° | ☐ |
| Invalid value (yaw=100°) | Validation error | N/A | ☐ |

**Test Script:**
```matlab
%% Test Method 4 Configuration Consolidation

%% Setup
robot = gik9dof.createRobotModel();
traj = create_short_test_trajectory(5);  % Helper function
q0 = homeConfiguration(robot);

%% Test 1: Default profile
fprintf('Test 1: Default profile\n');
cfg = gik9dof.loadPipelineProfile('default');
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'InitialConfiguration', q0, ...
    'ExecutionMode', 'ppFirst', ...
    'PipelineConfig', cfg, ...
    'Verbose', false);
assert(result.stageLogs.stageC.parameters.yawTolerance == deg2rad(15.0));
fprintf('  ✓ Yaw corridor: %.1f°\n', rad2deg(result.stageLogs.stageC.parameters.yawTolerance));

%% Test 2: Aggressive profile
fprintf('\nTest 2: Aggressive profile\n');
cfg = gik9dof.loadPipelineProfile('aggressive');
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'InitialConfiguration', q0, ...
    'ExecutionMode', 'ppFirst', ...
    'PipelineConfig', cfg, ...
    'Verbose', false);
assert(result.stageLogs.stageC.parameters.yawTolerance == deg2rad(20.0));
fprintf('  ✓ Yaw corridor: %.1f°\n', rad2deg(result.stageLogs.stageC.parameters.yawTolerance));

%% Test 3: Direct override
fprintf('\nTest 3: Direct override\n');
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'InitialConfiguration', q0, ...
    'ExecutionMode', 'ppFirst', ...
    'StageCPPFirstYawCorridor', 25.0, ...
    'Verbose', false);
assert(result.stageLogs.stageC.parameters.yawTolerance == deg2rad(25.0));
fprintf('  ✓ Yaw corridor: %.1f°\n', rad2deg(result.stageLogs.stageC.parameters.yawTolerance));

%% Test 4: Fallback (no config)
fprintf('\nTest 4: Fallback to hardcoded default\n');
result = gik9dof.runStagedTrajectory(robot, traj, ...
    'InitialConfiguration', q0, ...
    'ExecutionMode', 'ppFirst', ...
    'Verbose', false);
assert(result.stageLogs.stageC.parameters.yawTolerance == deg2rad(15.0));
fprintf('  ✓ Yaw corridor: %.1f° (default fallback)\n', ...
    rad2deg(result.stageLogs.stageC.parameters.yawTolerance));

fprintf('\n========================================\n');
fprintf('✅ ALL CONFIGURATION TESTS PASSED!\n');
fprintf('========================================\n');
```

---

## Rollout Checklist

- [ ] **Step 1:** Update `pipeline_profiles.yaml` with ppfirst section
- [ ] **Step 2:** Update `executeStageCPPFirst` to read from config
- [ ] **Step 3:** Add arguments to `runStagedTrajectory`
- [ ] **Step 4:** Update `loadPipelineProfile` validation
- [ ] **Step 5:** Update documentation (4 files)
- [ ] **Step 6:** Run test matrix (7 test cases)
- [ ] **Verification:** Run full trajectory with each profile
- [ ] **Verification:** Generate animation with consolidated config
- [ ] **Commit:** Create git commit with clear message

---

## Verification Criteria

### ✅ Success Criteria

1. All test cases pass
2. No hardcoded values in `executeStageCPPFirst`
3. `aggressive` profile uses wider corridor (20°)
4. `conservative` profile uses tighter corridor (10°)
5. Direct overrides take precedence over profile
6. Fallback to defaults works when YAML missing
7. Full trajectory runs with each profile variant
8. Documentation updated and accurate

### ❌ Failure Scenarios

1. YAML parsing error (syntax issue)
2. Config extraction fails (missing field access)
3. Default fallback not working (crash when YAML missing)
4. Override not taking precedence (config chain broken)
5. Invalid values not caught (negative corridor, etc.)

---

## Timeline

| Step | Duration | Dependencies | Assignee | Status |
|------|----------|--------------|----------|--------|
| 1. Update YAML | 30 min | None | TBD | ☐ |
| 2. Update executeStageCPPFirst | 1 hour | Step 1 | TBD | ☐ |
| 3. Add arguments | 15 min | Step 2 | TBD | ☐ |
| 4. Update loadPipelineProfile | 30 min | Step 2 | TBD | ☐ |
| 5. Update docs | 1 hour | Steps 1-4 | TBD | ☐ |
| 6. Testing | 1 hour | Steps 1-4 | TBD | ☐ |
| **Total** | **4 hours** | - | - | - |

**Estimated Completion:** 1 working day (with buffer)

---

## Post-Consolidation Benefits

1. **User-tunable:** Parameters adjustable without code changes
2. **Profile-aware:** Different settings for aggressive/conservative modes
3. **Consistent:** All pipeline params in single YAML file
4. **Discoverable:** Users can browse YAML to see available options
5. **Maintainable:** Single source of truth for defaults
6. **Testable:** Easy to validate different parameter combinations
7. **Production-ready:** No hardcoded magic numbers

---

## Related Documentation

- **Implementation Status:** `METHOD4_IMPLEMENTATION_STATUS.md`
- **Original Plan:** `codexMethod4Plan.md`
- **Project Diagnosis:** `projectDiagnosis.md` (Section 2, Method 4 details)
- **Config System:** `docs/UNIFIED_CONFIG_IMPLEMENTATION.md`
- **Quick Reference:** `docs/UNIFIED_CONFIG_QUICK_REF.md`

---

**Document Status:** 📋 PLANNED  
**Next Action:** Assign implementer and schedule work  
**Priority:** HIGH (blocks "production ready" status)  
**Effort:** 4 hours (~0.5 day)
