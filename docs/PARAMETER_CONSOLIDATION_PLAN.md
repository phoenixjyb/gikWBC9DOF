# Parameter Consolidation Plan

## Problem Statement

Currently, pipeline parameters are scattered across multiple locations with inconsistent defaults, creating maintenance nightmares and risk of parameter mismatches:

### Current Parameter Locations

#### 1. **config/chassis_profiles.yaml**
- Chassis kinematic parameters (track, wheel_base, wheel_speed_max)
- Velocity limits (vx_max, vx_min, wz_max)
- Acceleration limits (accel_limit, decel_limit, jerk_limit)
- Pure pursuit parameters (lookahead_base, heading_kp, feedforward_gain)
- Controller modes (stageB_controller_mode, stageC_controller_mode)

#### 2. **runStagedTrajectory.m** (lines 37-76)
- Stage B parameters with **different defaults** than runStagedReference:
  - `StageBDesiredLinearVelocity = 0.6` (vs 0.5 in runStagedReference)
  - `StageBHybridSafetyMargin = 0.1` (vs 0.15 in runStagedReference)
  - `StageBMode = "gikInLoop"` (vs "pureHyb" in runStagedReference)
- Stage C parameters (StageCTrackWidth, StageCMaxLinearSpeed, etc.)
- These are **hardcoded** in the function signature

#### 3. **runStagedReference.m** (lines 37-55)
- Similar Stage B/C parameters with **different defaults**
- Duplication of chassis-related parameters that already exist in YAML
- No clear reason why defaults differ from runStagedTrajectory

#### 4. **trackReferenceTrajectory.m** (lines 50-93)
- Yet another set of Stage B/C parameters
- Mixes pipeline options with chassis parameters
- Some parameters loaded from ChassisProfile, others hardcoded

### Specific Inconsistencies Found

| Parameter | chassis_profiles.yaml | runStagedTrajectory.m | runStagedReference.m | trackReferenceTrajectory.m |
|-----------|----------------------|----------------------|---------------------|---------------------------|
| **StageBDesiredLinearVelocity** | N/A | 0.6 m/s | 0.5 m/s | 0.5 m/s |
| **StageBHybridSafetyMargin** | N/A | 0.1 m | 0.15 m | 0.15 m |
| **StageBLookaheadDistance** | N/A | 0.6 m | 0.6 m | 0.6 m |
| **StageBMode** | N/A | "gikInLoop" | "pureHyb" | "pureHyb" |
| **StageCLookaheadDistance** | N/A | 0.8 m | N/A | 0.4 m |
| **StageCTrackWidth** | `track: 0.574` | 0.574 m | N/A | 0.574 m |
| **track** | 0.574 m | N/A (from YAML) | N/A (from YAML) | N/A (from YAML) |
| **vx_max** | 1.5 m/s | N/A (from YAML) | N/A (from YAML) | N/A (from YAML) |

### Problems This Creates

1. **Inconsistency Risk**: Different functions have different defaults for the same parameter
2. **Maintenance Burden**: Changing a parameter requires editing multiple files
3. **Parameter Drift**: No single source of truth; values can diverge over time
4. **Confusion**: Unclear which default is "correct" when they differ
5. **Testing Issues**: Hard to ensure all configurations are tested consistently
6. **Documentation Drift**: Parameter tables in docs can't stay synchronized

---

## Proposed Solution

### Architecture: Unified Configuration System

Create a **single source of truth** for all pipeline parameters using a hierarchical YAML configuration system.

```
config/
├── pipeline_profiles.yaml       ← NEW: Unified configuration file
└── chassis_profiles.yaml        ← DEPRECATED: Merge into pipeline_profiles.yaml
```

### New File Structure: `config/pipeline_profiles.yaml`

```yaml
profiles:
  default:
    # Metadata
    description: "Default configuration for wide-track mobile manipulator"
    
    # Chassis parameters (replaces chassis_profiles.yaml)
    chassis:
      track: 0.574                    # m (wheel separation)
      wheel_base: 0.36                # m
      wheel_speed_max: 3.3            # m/s per wheel
      vx_max: 1.5                     # m/s forward cap
      vx_min: -0.4                    # m/s reverse cap
      wz_max: 2.5                     # rad/s yaw cap
      accel_limit: 0.8                # m/s^2
      decel_limit: 1.8                # m/s^2
      jerk_limit: 5.0                 # m/s^3
    
    # Stage B parameters
    stage_b:
      mode: "pureHyb"                 # "gikInLoop" | "pureHyb"
      lookahead_distance: 0.6         # m
      desired_linear_velocity: 0.5    # m/s (conservative default)
      max_linear_speed: 1.5           # m/s
      max_angular_velocity: 2.5       # rad/s
      max_yaw_rate: 3.0               # rad/s (legacy)
      max_joint_speed: 1.0            # rad/s
      
      # Docking tolerances
      docking_position_tolerance: 0.02   # m
      docking_yaw_tolerance: 0.0349      # rad (2 degrees)
      
      # Hybrid A* parameters
      use_hybrid_astar: true
      hybrid_resolution: 0.05         # m
      hybrid_safety_margin: 0.15      # m
      hybrid_min_turning_radius: 0.5  # m
      hybrid_motion_primitive_length: 0.2  # m
      
      # Path smoothing
      use_reeds_shepp: false
      use_clothoid: false
      reeds_shepp_params:
        Rmin: 0.5                     # Will be overridden by chassis limit
        reverseCost: 5.0
        inflationRadius: -1.0         # Auto-fill from environment
        validationDistance: 0.035
        step: 0.05
        iters: 200
        maxSpan: 160
        lambdaCusp: 1.0
        seed: 0
        allowReverse: true
      
      clothoid_params:
        discretizationDistance: 0.08
        maxNumWaypoints: 0            # 0 = auto
      
      # Controller
      chassis_controller_mode: -1     # -1=auto from chassis profile
    
    # Stage C parameters
    stage_c:
      lookahead_distance: 0.4         # m (base lookahead)
      lookahead_vel_gain: 0.2         # s (velocity-dependent)
      lookahead_time_gain: 0.05       # s^2 (acceleration-dependent)
      desired_linear_velocity: 1.0    # m/s
      max_linear_speed: 1.5           # m/s
      min_linear_speed: -0.4          # m/s
      max_angular_velocity: 2.5       # rad/s
      
      # Path following
      track_width: 0.574              # m (should match chassis.track)
      wheel_base: 0.36                # m (should match chassis.wheel_base)
      max_wheel_speed: 3.3            # m/s (should match chassis.wheel_speed_max)
      waypoint_spacing: 0.15          # m
      path_buffer_size: 30.0          # m
      goal_tolerance: 0.10            # m
      interp_spacing: 0.05            # m
      reverse_enabled: true
      
      # Refinement
      use_base_refinement: true
      
      # Controller
      chassis_controller_mode: -1     # -1=auto from chassis profile
    
    # GIK parameters
    gik:
      max_iterations: 1500
      tolerance: 1e-5
      distance_margin: 0.02           # m
      
    # Pure pursuit parameters (legacy, consider moving to chassis)
    pure_pursuit:
      lookahead_base: 0.80            # m
      lookahead_vel_gain: 0.30        # s
      lookahead_time_gain: 0.05       # s^2
      heading_kp: 1.2
      heading_ki: 0.0
      heading_kd: 0.1
      feedforward_gain: 0.9
      goal_tolerance: 0.10            # m
      reverse_enabled: false
      interp_spacing_min: 0.05        # m
      interp_spacing_max: 0.20        # m
      discontinuity_threshold: 0.35   # m
      curvature_slowdown:
        kappa_threshold: 0.9          # 1/m
        vx_reduction: 0.6             # scale factor
  
  # Additional profiles
  aggressive:
    description: "Fast execution with tighter tolerances"
    inherits: "default"
    overrides:
      stage_b:
        desired_linear_velocity: 0.8
        hybrid_safety_margin: 0.10
      stage_c:
        desired_linear_velocity: 1.2
        max_linear_speed: 1.8
      chassis:
        vx_max: 1.8
        accel_limit: 1.2
  
  conservative:
    description: "Slow, safe execution with larger margins"
    inherits: "default"
    overrides:
      stage_b:
        desired_linear_velocity: 0.3
        hybrid_safety_margin: 0.20
      stage_c:
        desired_linear_velocity: 0.5
        max_linear_speed: 0.8
      chassis:
        vx_max: 0.8
        accel_limit: 0.5
```

---

## Implementation Plan

### Phase 1: Create New Infrastructure (Non-Breaking)

1. **Create `config/pipeline_profiles.yaml`** with consolidated parameters ✓
2. **Create loader function** `matlab/+gik9dof/loadPipelineProfile.m`:
   ```matlab
   function config = loadPipelineProfile(profileName, options)
   % Load unified pipeline configuration
   % Supports inheritance and overrides
   ```
3. **Keep existing functions working** - don't break anything yet

### Phase 2: Migration Functions

Create helper functions to bridge old → new:

```matlab
function options = migrateOptionsToProfile(options, profileName)
% Convert old options struct to new profile-based config
```

### Phase 3: Update Main Functions

Modify in order:
1. `trackReferenceTrajectory.m` - simplest, already uses ChassisProfile
2. `runStagedReference.m` - wrapper function
3. `runStagedTrajectory.m` - core function

New signatures:
```matlab
% Old way (deprecated but still works):
log = gik9dof.runStagedTrajectory(robot, refTraj, ...
    'StageBDesiredLinearVelocity', 0.6, ...
    'StageCTrackWidth', 0.574, ...
    ... 50+ more parameters)

% New way (recommended):
log = gik9dof.runStagedTrajectory(robot, refTraj, ...
    'Profile', 'default', ...
    'ProfileOverrides', struct('stage_b', struct('desired_linear_velocity', 0.6)))
```

### Phase 4: Update Documentation & Tests

1. Update all test scripts to use new profile system
2. Update documentation to reference unified config
3. Add migration guide for existing code

---

## Benefits

1. ✅ **Single Source of Truth**: All parameters in one place
2. ✅ **Consistency**: No more conflicting defaults
3. ✅ **Inheritance**: Share common settings across profiles
4. ✅ **Easy Experimentation**: Create new profiles without code changes
5. ✅ **Version Control**: Config changes tracked separately from code
6. ✅ **Documentation**: Auto-generate parameter tables from YAML
7. ✅ **Validation**: Centralized parameter validation
8. ✅ **Testing**: Easy to test multiple configurations

---

## Migration Guide for Existing Code

### Before (scattered parameters):
```matlab
log = gik9dof.trackReferenceTrajectory(robot, jsonPath, ...
    'StageBDesiredLinearVelocity', 0.5, ...
    'StageBHybridSafetyMargin', 0.15, ...
    'StageCTrackWidth', 0.574, ...
    'StageCMaxLinearSpeed', 1.5, ...
    'ChassisProfile', 'wide_track', ...
    'ChassisOverrides', struct('vx_max', 1.8));
```

### After (unified profile):
```matlab
% Use default profile as-is
log = gik9dof.trackReferenceTrajectory(robot, jsonPath, ...
    'Profile', 'default');

% Or override specific parameters
log = gik9dof.trackReferenceTrajectory(robot, jsonPath, ...
    'Profile', 'default', ...
    'ProfileOverrides', struct( ...
        'stage_b', struct('desired_linear_velocity', 0.5), ...
        'chassis', struct('vx_max', 1.8)));

% Or use a pre-configured profile
log = gik9dof.trackReferenceTrajectory(robot, jsonPath, ...
    'Profile', 'aggressive');
```

---

## Open Questions

1. Should we deprecate `chassis_profiles.yaml` or keep it separate?
   - **Recommendation**: Merge into `pipeline_profiles.yaml` for true unification
   
2. How to handle backward compatibility?
   - **Recommendation**: Keep old parameter names working with deprecation warnings
   
3. Should profiles support multi-level inheritance?
   - **Recommendation**: Yes, but limit to 2 levels to avoid complexity

4. How to validate parameter consistency (e.g., stage_c.track_width == chassis.track)?
   - **Recommendation**: Loader function validates and issues warnings

5. Should we support per-environment profiles?
   - **Recommendation**: Yes, future enhancement to include environment-specific configs

---

## Next Steps

1. Review this plan and get approval
2. Create `config/pipeline_profiles.yaml`
3. Implement `loadPipelineProfile.m`
4. Test with existing scripts
5. Migrate functions one by one
6. Update documentation
