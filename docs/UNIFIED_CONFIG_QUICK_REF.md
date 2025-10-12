# Unified Config Quick Reference

**TL;DR:** All pipeline parameters are now in `config/pipeline_profiles.yaml`. Load with `loadPipelineProfile()`, use everywhere.

---

## 🚀 Quick Start

### Old Way (Still Works)
```matlab
log = gik9dof.trackReferenceTrajectory(...
    'MaxIterations', 150, ...
    'StageBDesiredLinearVelocity', 0.6, ...
    'StageBHybridSafetyMargin', 0.1);
```

### New Way (Recommended)
```matlab
cfg = gik9dof.loadPipelineProfile('default');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
```

---

## 📦 Available Profiles

| Profile | Description | Use Case |
|---------|-------------|----------|
| `default` | Balanced parameters | General purpose, starting point |
| `aggressive` | Faster motion (1.8 m/s max) | Open environments, speed tests |
| `conservative` | Slower, safer (1.0 m/s max) | Cluttered spaces, safety tests |
| `compact_track` | Narrower wheelbase (0.4 m) | Different robot configuration |

---

## 🔧 Common Overrides

### Quick Override (Runtime)
```matlab
cfg = gik9dof.loadPipelineProfile('default', ...
    'stage_b', struct('desired_linear_velocity', 0.8), ...
    'gik', struct('max_iterations', 200));
```

### Modify Loaded Config
```matlab
cfg = gik9dof.loadPipelineProfile('aggressive');
cfg.stage_b.hybrid_safety_margin = 0.15;
cfg.chassis.accel_limit = 1.5;
```

### Create Custom Profile (YAML)
```yaml
# In config/pipeline_profiles.yaml
my_profile:
  inherits: "default"
  overrides:
    stage_b: {desired_linear_velocity: 0.7}
    chassis: {accel_limit: 1.0}
```

Then:
```matlab
cfg = gik9dof.loadPipelineProfile('my_profile');
```

---

## 📊 Config Structure

```matlab
cfg = struct(
    'chassis',      % Kinematic limits, track width, controller gains
    'stage_b',      % Navigation: mode, velocity, hybrid A*, Reeds-Shepp
    'stage_c',      % Tracking: refinement, controller mode
    'gik',          % Solver: max iterations, distance weight
    'pure_pursuit', % Controller: lookahead, gains
    'holistic',     % Holistic mode parameters
    'meta'          % Metadata: profile name, warnings
);
```

### Key Parameters Reference

| Category | Parameter | Default | Description |
|----------|-----------|---------|-------------|
| **chassis** | track | 0.574 m | Wheel separation (FIXED) |
| | vx_max | 1.5 m/s | Max forward speed |
| | accel_limit | 0.8 m/s² | Acceleration limit |
| **stage_b** | mode | "pureHyb" | Base planning mode |
| | desired_linear_velocity | 0.6 m/s | Nominal speed |
| | hybrid_safety_margin | 0.1 m | Obstacle margin |
| | lookahead_distance | 0.6 m | Pure pursuit lookahead |
| **stage_c** | track_width | 0.574 m | Must match chassis.track |
| | use_base_refinement | true | RS/clothoid smoothing |
| | controller_mode | 2 | 0=legacy, 1=heading, 2=PP |
| **gik** | max_iterations | 150 | Solver iteration cap |
| | distance_weight | 5.0 | Obstacle avoidance weight |

---

## 🎯 Use with Main Functions

### trackReferenceTrajectory
```matlab
cfg = gik9dof.loadPipelineProfile('aggressive');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg, ...
    'Mode', 'staged');  % Non-config params still passed individually
```

### runStagedReference
```matlab
cfg = gik9dof.loadPipelineProfile('conservative');
result = gik9dof.runStagedReference('PipelineConfig', cfg, ...
    'RunLabel', 'my_test');
```

### runStagedTrajectory
```matlab
cfg = gik9dof.loadPipelineProfile('default');
log = gik9dof.runStagedTrajectory(robot, 'PipelineConfig', cfg);
```

---

## 🔍 Debugging

### Check What's Loaded
```matlab
cfg = gik9dof.loadPipelineProfile('aggressive');
disp(cfg.meta.profile);           % Shows: 'aggressive'
disp(cfg.stage_b.desired_linear_velocity);  % Shows: 0.8
```

### Validation Warnings
```matlab
cfg = gik9dof.loadPipelineProfile('default');
if ~isempty(cfg.meta.validationWarnings)
    fprintf('Warnings:\n');
    cellfun(@(x) fprintf('  - %s\n', x), cfg.meta.validationWarnings);
end
```

### Compare Profiles
```matlab
cfg1 = gik9dof.loadPipelineProfile('default');
cfg2 = gik9dof.loadPipelineProfile('aggressive');

fprintf('Default vx_max: %.2f m/s\n', cfg1.chassis.vx_max);
fprintf('Aggressive vx_max: %.2f m/s\n', cfg2.chassis.vx_max);
```

---

## ⚠️ Common Pitfalls

### 1. String vs Char
```matlab
% YAML uses strings, MATLAB often expects char
cfg.stage_b.mode = "pureHyb";  % string (OK)
cfg.stage_b.mode = 'pureHyb';  % char (also OK)

% But be consistent when comparing:
if cfg.stage_b.mode == "pureHyb"  % string comparison
```

### 2. Track Width Consistency
```matlab
% These MUST match (automatically validated):
cfg.chassis.track          % 0.574 m
cfg.stage_c.track_width    # 0.574 m

% If mismatch, you'll see warning in cfg.meta.validationWarnings
```

### 3. Overriding Nested Structs
```matlab
% WRONG - this replaces entire stage_b struct:
cfg = gik9dof.loadPipelineProfile('default', ...
    'stage_b', struct('desired_linear_velocity', 0.8));
% Now cfg.stage_b.mode is MISSING!

% RIGHT - modify after loading:
cfg = gik9dof.loadPipelineProfile('default');
cfg.stage_b.desired_linear_velocity = 0.8;  # Preserves all other fields
```

---

## 📚 See Also

- **Full Documentation:** `projectDiagnosis.md` - Section 6
- **Migration Guide:** `TODO_MIGRATE_TO_UNIFIED_CONFIG.md`
- **Design Details:** `docs/PARAMETER_CONSOLIDATION_PLAN.md`
- **Implementation:** `docs/UNIFIED_CONFIG_IMPLEMENTATION.md`
- **Test Example:** `test_pipeline_profiles.m`

---

## 💡 Pro Tips

### 1. Profile Experimentation
```matlab
% Run same simulation with different profiles:
profiles = ["default", "aggressive", "conservative"];
results = cell(size(profiles));

for i = 1:length(profiles)
    cfg = gik9dof.loadPipelineProfile(profiles(i));
    results{i} = gik9dof.runStagedReference('PipelineConfig', cfg);
end
```

### 2. Parameter Sweeps
```matlab
% Easy to sweep parameters:
velocities = [0.4, 0.6, 0.8, 1.0];
logs = cell(size(velocities));

for i = 1:length(velocities)
    cfg = gik9dof.loadPipelineProfile('default');
    cfg.stage_b.desired_linear_velocity = velocities(i);
    logs{i} = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
end
```

### 3. Save Custom Config
```matlab
% Load and customize:
cfg = gik9dof.loadPipelineProfile('default');
cfg.stage_b.desired_linear_velocity = 0.75;
cfg.chassis.accel_limit = 1.1;

% Save for later:
save('my_custom_config.mat', 'cfg');

% Load in another script:
load('my_custom_config.mat', 'cfg');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
```

---

**Last Updated:** January 2025  
**Version:** 1.0
