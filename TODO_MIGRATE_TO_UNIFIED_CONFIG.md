# TODO: Migrate to Unified Config System

This document tracks scripts and files that should be migrated to use the new unified configuration system when you're ready to switch from the backward-compatible individual parameters.

**Status:** ✅ System implemented, backward compatibility maintained  
**Action Required:** Migration is OPTIONAL and can be done gradually  
**Priority:** Low (existing code still works perfectly)

---

## What is the Unified Config System?

Instead of passing dozens of individual parameters:
```matlab
log = gik9dof.trackReferenceTrajectory(...
    'MaxIterations', 150, ...
    'StageBDesiredLinearVelocity', 0.6, ...
    'StageBHybridSafetyMargin', 0.1, ...
    'StageCUseBaseRefinement', true, ...
    'ChassisProfile', 'wide_track', ...
    % ... 20 more parameters ...
);
```

You can now load a profile with all parameters:
```matlab
cfg = gik9dof.loadPipelineProfile('default');  % or 'aggressive', 'conservative'
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
```

---

## Files with TODO Comments Added

These files now have inline TODO comments reminding you to migrate:

### 1. ✅ `run_staged_reference.m`
**Location:** Lines 1-6  
**Current:** Uses 10+ individual parameters  
**Migration:**
```matlab
% Instead of:
result = gik9dof.runStagedReference(...
    'MaxIterations', 150, ...
    'StageBDesiredLinearVelocity', 0.5, ...
    'StageBMode', 'pureHyb', ...
    'DistanceMargin', 0.30);

% Use:
cfg = gik9dof.loadPipelineProfile('default');
cfg.gik.max_iterations = 150;  % Override if needed
cfg.stage_b.desired_linear_velocity = 0.5;
cfg.stage_b.mode = "pureHyb";
result = gik9dof.runStagedReference('PipelineConfig', cfg, ...
    'RunLabel', runLabel);  % Non-config params still passed individually
```

### 2. ✅ `run_fresh_sim_with_animation.m`
**Location:** Lines 1-13  
**Current:** Manual config struct + individual parameters  
**Migration:**
```matlab
% Instead of:
config.SafetyMargin = 0.10;
config.LambdaCusp = 1.0;
config.Lookahead = 0.80;
log = gik9dof.trackReferenceTrajectory(...
    'StageBHybridSafetyMargin', config.SafetyMargin, ...
    'StageBLookaheadDistance', config.Lookahead, ...);

% Use:
cfg = gik9dof.loadPipelineProfile('default');
cfg.stage_b.hybrid_safety_margin = 0.10;
cfg.stage_b.lookahead_distance = 0.80;
cfg.stage_b.reeds_shepp.lambda_cusp = 1.0;
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg, ...
    'Mode', 'staged');
```

### 3. ✅ `run_parametric_study.m`
**Location:** Lines 1-6  
**Current:** Parameter sweep with custom combinations  
**Migration:**
```matlab
% Instead of building custom configs manually:
config1.SafetyMargin = 0.10;
config2.SafetyMargin = 0.05;

% Use profile inheritance:
cfg_baseline = gik9dof.loadPipelineProfile('default');
cfg_aggressive = gik9dof.loadPipelineProfile('aggressive');

% Or override specific parameters:
cfg_custom = gik9dof.loadPipelineProfile('default', ...
    'stage_b', struct('hybrid_safety_margin', 0.05));
```

### 4. ✅ `run_environment_compare.m`
**Location:** Lines 1-6  
**Current:** Individual parameters for both holistic and staged  
**Migration:**
```matlab
% Instead of:
summary = gik9dof.runEnvironmentCompare(...
    'MaxIterations', 1500, ...
    'ChassisProfile', 'wide_track', ...);

% Use:
cfg = gik9dof.loadPipelineProfile('default');
cfg.gik.max_iterations = 1500;
summary = gik9dof.runEnvironmentCompare('PipelineConfig', cfg, ...);
```

---

## Other Scripts That May Benefit (No TODO Added Yet)

### Test Scripts
These might benefit from migration but are typically one-off tests:

- `test_*.m` - Various test scripts (24 files)
- `debug_*.m` - Debug scripts (7 files)
- `generate_*.m` - Animation generation scripts (7 files)
- `matlab/run_*.m` - Additional run scripts in matlab/ subdirectory

### When to Migrate These:
- If you're running them regularly
- If you want consistent parameters across multiple tests
- If you're creating new test scripts (use unified config from the start)

---

## Migration Priority Guide

### High Priority (Do First)
✅ Already done - added TODO comments to main entry points:
- `run_staged_reference.m`
- `run_fresh_sim_with_animation.m`
- `run_parametric_study.m`
- `run_environment_compare.m`

### Medium Priority (Migrate When Convenient)
- Frequently used test scripts
- Scripts you're actively debugging
- New scripts you create (use unified config from day 1)

### Low Priority (Migrate Only If Needed)
- One-off debug scripts
- Legacy scripts that work fine
- Scripts scheduled for deletion

---

## Benefits of Migrating

### 1. Single Source of Truth
All parameters in one place (`config/pipeline_profiles.yaml`)

### 2. Easy Experimentation
```matlab
% Quick profile switching:
cfg_default = gik9dof.loadPipelineProfile('default');
cfg_fast = gik9dof.loadPipelineProfile('aggressive');
cfg_safe = gik9dof.loadPipelineProfile('conservative');

% Run comparisons easily:
log1 = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg_default);
log2 = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg_fast);
log3 = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg_safe);
```

### 3. Parameter Inheritance
```yaml
# In pipeline_profiles.yaml - define once, inherit everywhere:
my_experiment:
  inherits: "aggressive"
  overrides:
    stage_b: {desired_linear_velocity: 0.9}
```

### 4. Consistency
No more parameter mismatches between different function calls

### 5. Validation
Automatic checks (e.g., `stage_c.track_width == chassis.track`)

---

## Migration Checklist Template

When you're ready to migrate a script:

- [ ] Read current parameters from the script
- [ ] Choose base profile (default/aggressive/conservative) or create custom
- [ ] Map parameters to unified config structure
- [ ] Test that results are identical
- [ ] Update script documentation
- [ ] Remove old parameter passing code

---

## Need Help?

See these files for examples and documentation:
- **Migration Guide:** `UNIFIED_CONFIG_MIGRATION_COMPLETE.md`
- **Design Doc:** `docs/PARAMETER_CONSOLIDATION_PLAN.md`
- **Implementation:** `docs/UNIFIED_CONFIG_IMPLEMENTATION.md`
- **Test Script:** `test_pipeline_profiles.m`
- **System Architecture:** `projectDiagnosis.md` - Section 6

---

## Timeline

**No deadline** - migrate at your own pace:
- Backward compatibility is maintained indefinitely
- Old parameter style will continue to work
- New scripts should use unified config
- Gradually update existing scripts as you touch them

**Recommended approach:** "Boy Scout Rule"
- Whenever you modify a script, migrate it to unified config
- New features/tests: use unified config from day 1
- Don't migrate scripts that are working and untouched
