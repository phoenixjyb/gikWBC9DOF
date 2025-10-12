# Chassis Profiles vs Pipeline Profiles - Deprecation Guide

**Question:** "Does `chassis_profiles.yaml` become obsolete since we have `pipeline_profiles.yaml`?"

**Short Answer:** Yes, effectively. `chassis_profiles.yaml` is now **legacy/deprecated** but kept for backward compatibility.

---

## The Situation

### Before (Old System)
```
config/
  └── chassis_profiles.yaml     ← ONLY chassis parameters

Parameters scattered across:
  - chassis_profiles.yaml (chassis only)
  - runStagedTrajectory.m defaults (Stage B/C/GIK)
  - runStagedReference.m defaults (different values!)
  - trackReferenceTrajectory.m defaults (yet another set!)
```

### After (New Unified System)
```
config/
  ├── chassis_profiles.yaml     ← ⚠️ DEPRECATED (kept for compatibility)
  └── pipeline_profiles.yaml    ← ✅ USE THIS (everything in one place)
```

---

## What's in Each File?

### chassis_profiles.yaml (LEGACY)
**Contains:** Chassis parameters ONLY
- Kinematic limits (track, wheelbase, vx_max, wz_max)
- Controller gains (heading_kp, feedforward_gain)
- Pure pursuit parameters (lookahead_base, lookahead_vel_gain)
- Path processing (interp_spacing, discontinuity_threshold)

**Limitations:**
- ❌ No Stage B parameters (hybrid A*, Reeds-Shepp, velocity)
- ❌ No Stage C parameters (base refinement, controller mode)
- ❌ No GIK parameters (max iterations, distance weight)
- ❌ No holistic mode parameters
- ❌ No profile inheritance
- ❌ No automatic validation

**Who uses it:**
- `gik9dof.control.loadChassisProfile("wide_track")`
- `gik9dof.control.defaultUnifiedParams()` (calls loadChassisProfile internally)
- Old code that hasn't migrated yet

### pipeline_profiles.yaml (NEW - RECOMMENDED)
**Contains:** EVERYTHING
- ✅ Chassis parameters (same as chassis_profiles.yaml)
- ✅ Stage B parameters (mode, velocity, hybrid A*, Reeds-Shepp, clothoid)
- ✅ Stage C parameters (track_width, base refinement, controller mode)
- ✅ GIK parameters (max_iterations, distance_weight, enable_aiming)
- ✅ Pure pursuit parameters (lookahead, gains)
- ✅ Holistic mode parameters

**Extra Features:**
- ✅ Profile inheritance (aggressive/conservative inherit from default)
- ✅ Automatic validation (checks track_width == chassis.track)
- ✅ Deep merge of overrides
- ✅ Metadata (profile name, validation warnings)
- ✅ 4 predefined profiles (default, aggressive, conservative, compact_track)

**Who should use it:**
- ✅ All new code
- ✅ Anyone wanting consistent parameters across pipeline
- ✅ Anyone doing parameter sweeps or experiments

---

## Parameter Comparison

| Parameter | chassis_profiles.yaml | pipeline_profiles.yaml |
|-----------|----------------------|------------------------|
| track | ✅ 0.574 m | ✅ 0.574 m (in chassis section) |
| vx_max | ✅ 1.5 m/s | ✅ 1.5 m/s (chassis.vx_max) |
| accel_limit | ✅ 0.8 m/s² | ✅ 0.8 m/s² (chassis.accel_limit) |
| lookahead_base | ✅ 0.80 m | ✅ 0.80 m (pure_pursuit.lookahead_base) |
| heading_kp | ✅ 1.2 | ✅ 1.2 (chassis.heading_kp) |
| **Stage B velocity** | ❌ Missing | ✅ 0.6 m/s (stage_b.desired_linear_velocity) |
| **Stage B mode** | ❌ Missing | ✅ "pureHyb" (stage_b.mode) |
| **Stage B safety margin** | ❌ Missing | ✅ 0.1 m (stage_b.hybrid_safety_margin) |
| **Stage C refinement** | ❌ Missing | ✅ true (stage_c.use_base_refinement) |
| **GIK max iterations** | ❌ Missing | ✅ 150 (gik.max_iterations) |
| **Profile inheritance** | ❌ No | ✅ Yes (aggressive inherits from default) |
| **Validation** | ❌ No | ✅ Yes (automatic consistency checks) |

---

## Migration Path

### Option 1: Keep Using chassis_profiles.yaml (Not Recommended)
```matlab
% Old way - still works but limited
chassisParams = gik9dof.control.loadChassisProfile("wide_track");

% You still need to specify Stage B/C/GIK parameters separately:
log = gik9dof.trackReferenceTrajectory(...
    'ChassisProfile', 'wide_track', ...
    'MaxIterations', 150, ...
    'StageBDesiredLinearVelocity', 0.6, ...
    'StageBHybridSafetyMargin', 0.1, ...
    % ... many more parameters ...
);
```

### Option 2: Use pipeline_profiles.yaml (Recommended)
```matlab
% New way - everything in one place
cfg = gik9dof.loadPipelineProfile('default');

% All parameters loaded at once:
% - cfg.chassis (all chassis params)
% - cfg.stage_b (all Stage B params)
% - cfg.stage_c (all Stage C params)
% - cfg.gik (GIK solver params)
% - cfg.pure_pursuit (controller params)
% - cfg.holistic (holistic mode params)

log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
```

### Extracting Chassis Params from Pipeline Config
```matlab
% If you need just chassis params from unified config:
cfg = gik9dof.loadPipelineProfile('default');
chassisParams = cfg.chassis;  % Same structure as loadChassisProfile returns

% Then pass to functions that expect chassis params:
follower = gik9dof.control.purePursuitFollower(...
    'ChassisParams', chassisParams, ...);
```

---

## Still Using chassis_profiles.yaml?

These functions/files still call `loadChassisProfile()`:

1. **`gik9dof.control.defaultUnifiedParams()`**
   - Calls `loadChassisProfile()` internally
   - Used by `unifiedChassisCtrl` if no params provided
   - **Recommendation:** Pass chassis params from pipeline config instead

2. **`trackReferenceTrajectory.m` (line 151)**
   - Still uses `ChassisProfile` option
   - But also accepts `PipelineConfig` now (recommended)
   - Backward compatible

3. **`runStagedTrajectory.m` (line 116)**
   - Still uses `ChassisProfile` option
   - But also accepts `PipelineConfig` now (recommended)
   - Backward compatible

---

## Should We Delete chassis_profiles.yaml?

**No, not yet.** Here's why:

### Reasons to Keep (For Now)
1. **Backward Compatibility:** Existing scripts that call `loadChassisProfile()` still work
2. **Gradual Migration:** Users can migrate at their own pace
3. **Documented Deprecation:** Clear warnings guide users to new system
4. **Low Maintenance Cost:** File is stable, rarely needs updates

### When to Consider Removal
- [ ] All internal code migrated to `pipeline_profiles.yaml`
- [ ] All test scripts updated
- [ ] No external users relying on `loadChassisProfile()`
- [ ] At least 6 months deprecation notice given
- [ ] Version 2.0 or major release

### Alternative: Redirect Implementation
Instead of deleting, `loadChassisProfile()` could be updated to:
```matlab
function params = loadChassisProfile(profileName, options)
    % Redirect to unified config system
    warning('loadChassisProfile is deprecated. Use loadPipelineProfile instead.');
    
    % Map chassis profile name to pipeline profile
    cfg = gik9dof.loadPipelineProfile('default');
    params = cfg.chassis;
    
    % Apply overrides if provided
    if nargin > 1 && ~isempty(options.Overrides)
        params = mergeStructs(params, options.Overrides);
    end
end
```

---

## Recommendation Summary

### For New Code
✅ **Use `pipeline_profiles.yaml` exclusively**
```matlab
cfg = gik9dof.loadPipelineProfile('default');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
```

### For Existing Code
🔄 **Migrate gradually using TODO comments**
- Scripts marked with TODO comments should migrate when modified
- Old code that works can stay as-is for now
- No rush - backward compatibility maintained

### For chassis_profiles.yaml
⚠️ **Keep but mark as deprecated**
- ✅ Already added deprecation notice (see file header)
- ✅ Document in README/guides
- ❌ Don't add new profiles to chassis_profiles.yaml
- ❌ Don't recommend it in new documentation
- ✅ Consider removal in future major version

---

## Quick Reference

| Scenario | Use This |
|----------|----------|
| **New simulation script** | `loadPipelineProfile('default')` |
| **Parameter sweep study** | `loadPipelineProfile()` with overrides |
| **Testing different speeds** | Load profile, modify `cfg.stage_b.desired_linear_velocity` |
| **Custom robot config** | Create new profile in `pipeline_profiles.yaml` |
| **Legacy script maintenance** | Keep using `loadChassisProfile()` (still works) |
| **Migrating old script** | Replace `loadChassisProfile()` → `loadPipelineProfile()` |

---

## See Also

- **Quick Reference:** `UNIFIED_CONFIG_QUICK_REF.md`
- **Migration Guide:** `TODO_MIGRATE_TO_UNIFIED_CONFIG.md`
- **Full Documentation:** `projectDiagnosis.md` - Section 6
- **Implementation Details:** `docs/UNIFIED_CONFIG_IMPLEMENTATION.md`

---

**Last Updated:** January 2025  
**Status:** chassis_profiles.yaml is deprecated, pipeline_profiles.yaml is recommended  
**Action Required:** Use pipeline_profiles.yaml for all new code
