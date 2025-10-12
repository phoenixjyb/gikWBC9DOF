# Quick Start: Staged Simulation with Animation

**Goal**: Run a simple staged simulation and generate an animation to visualize the results.

---

## ⚡ The Simplest Way (One Command)

```matlab
cd /path/to/gikWBC9DOF
addpath(genpath('matlab'))
scripts/run_fresh_sim_with_animation
```

**That's it!** This will:
1. ✅ Run a staged simulation (ppForIk mode)
2. ✅ Generate an animation automatically
3. ✅ Save results to `results/YYYYMMDD_HHMMSS_staged_reference/`

---

## 📋 Step-by-Step Method

If you want more control, follow these steps:

### Step 1: Navigate to Project Root
```matlab
cd /path/to/gikWBC9DOF
addpath(genpath('matlab'))
```

### Step 2: Run Staged Simulation
```matlab
% Option A: Use the default configuration
log = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                        'Mode', 'staged');

% Option B: Load a specific pipeline profile
profile = gik9dof.config.loadPipelineProfile('ppForIk_tuned');
log = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                        'PipelineConfig', profile);

% Option C: Use the higher-level wrapper (recommended)
result = gik9dof.runStagedReference('RunLabel', 'my_test');
log = result.log;
```

### Step 3: Generate Animation
```matlab
% Simple animation (displays in figure window)
gik9dof.animateStagedWithHelper(log);

% Export animation to MP4 video
gik9dof.animateStagedWithHelper(log, 'ExportVideo', 'my_animation.mp4');

% Export with custom settings
gik9dof.animateStagedWithHelper(log, ...
    'ExportVideo', 'my_animation.mp4', ...
    'FrameRate', 30, ...           % Video frame rate
    'SampleStep', 1);              % 1=all frames, 2=every other frame
```

---

## 🎯 What Happens in a Staged Simulation?

A **staged simulation** consists of three phases:

| Stage | Name | Description | Base | Arm |
|-------|------|-------------|------|-----|
| **A** | Arm Ramp-Up | Move arm to starting position | 🔒 Locked | ✅ Moving |
| **B** | Base Navigation | Navigate base to tracking start pose | ✅ Moving | 🔒 Locked |
| **C** | Coordinated Tracking | Track trajectory with coordinated motion | ✅ Moving | ✅ Moving |

**Stage C Execution Modes:**
- **`ppForIk` (recommended)**: 3-pass approach
  - Pass 1: Reference IK (ideal trajectory)
  - Pass 2: Chassis simulation (realistic base path)
  - Pass 3: Final IK (arm tracks with locked base)
- **`pureIk`**: Single-pass full-body IK

---

## 📂 Where Are My Results?

Results are automatically saved to:
```
results/YYYYMMDD_HHMMSS_staged_reference/
├── log_staged_ppForIk.mat       # Full trajectory log (main output)
├── run_report.json              # Performance metrics
├── run_animation.mp4            # Video (if generated)
└── run_plot.png                 # Tracking error plots (if generated)
```

---

## 🔧 Common Customizations

### Change Pipeline Profile
```matlab
% Available profiles in config/pipeline_profiles.yaml:
% - ppForIk_default    : Standard 3-pass
% - ppForIk_tuned      : Optimized (recommended)
% - pureIk_default     : Single-pass holistic IK
% - pureHyb_default    : Pure pursuit with Hybrid A*

profile = gik9dof.config.loadPipelineProfile('ppForIk_tuned');
log = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                        'PipelineConfig', profile);
```

### Change Control Rate
```matlab
result = gik9dof.runStagedReference('RunLabel', 'test_10hz', ...
                                    'RateHz', 10);  % 10Hz (recommended) or 30Hz
```

### Use Different Trajectory
```matlab
% List available trajectories
ls refEETrajs/*.json

% Use a specific trajectory
log = gik9dof.trackReferenceTrajectory('refEETrajs/your_trajectory.json', ...
                                        'Mode', 'staged');
```

### Customize Animation
```matlab
helperOpts = struct();
helperOpts.ArrowLength = 0.25;         % Heading arrow size (m)
helperOpts.PlaybackSpeed = 0.5;        % 0.5=half speed, 1.0=realtime, 2.0=double
helperOpts.VisualAlpha = 0.6;          % Robot transparency
helperOpts.ChassisAlpha = 0.35;        % Chassis transparency

gik9dof.animateStagedWithHelper(log, ...
    'HelperOptions', helperOpts, ...
    'ExportVideo', 'slow_motion.mp4');
```

---

## 📊 Analyze Your Results

### Quick Performance Check
```matlab
report = gik9dof.evaluateLog(log, 'Verbose', true);

% Key metrics:
fprintf('Mean EE error: %.3f m (%.1f mm)\n', ...
    report.positionMean, report.positionMean * 1000);
fprintf('Max EE error: %.3f m (%.1f mm)\n', ...
    report.positionMax, report.positionMax * 1000);
fprintf('Success rate: %.1f%%\n', report.successRate * 100);
```

### Plot Tracking Errors
```matlab
% Display interactive plot
gik9dof.plotTrajectoryLog(log);

% Export plot to file
gik9dof.plotTrajectoryLog(log, 'ExportPath', 'tracking_errors.png');
```

### Inspect Specific Stages
```matlab
% Stage A (arm ramp-up)
stageA = log.stageLogs.stageA;
fprintf('Stage A: %d waypoints, %.2f seconds\n', ...
    size(stageA.qTraj, 2), stageA.qTraj(end));

% Stage B (base navigation)
stageB = log.stageLogs.stageB;
basePath = stageB.execBaseStates;
fprintf('Stage B: Navigated %.2f meters\n', ...
    sum(vecnorm(diff(basePath(:,1:2), 1, 1), 2, 2)));

% Stage C (coordinated tracking)
stageC = log.stageLogs.stageC;
fprintf('Stage C: %d waypoints tracked\n', size(stageC.qTraj, 2));
if isfield(stageC, 'eePositions')
    eeErrors = vecnorm(stageC.eePositions - stageC.targetPositions, 2, 1);
    fprintf('  Mean EE error: %.1f mm\n', mean(eeErrors) * 1000);
end
```

---

## 🆚 Load and Compare Existing Results

```matlab
% Load a previous log
logPath = 'results/20251012_101530_staged_reference/log_staged_ppForIk.mat';
loaded = load(logPath);
log = loaded.log;

% Regenerate animation from saved log
gik9dof.animateStagedWithHelper(log, 'ExportVideo', 'regenerated.mp4');

% Compare with new run
log_new = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                            'Mode', 'staged');

report_old = gik9dof.evaluateLog(log, 'Verbose', false);
report_new = gik9dof.evaluateLog(log_new, 'Verbose', false);

fprintf('\nComparison:\n');
fprintf('Old: %.3fm mean, %.3fm max\n', ...
    report_old.positionMean, report_old.positionMax);
fprintf('New: %.3fm mean, %.3fm max\n', ...
    report_new.positionMean, report_new.positionMax);
```

---

## 🎬 Animation Features

The staged animation shows:

**Visual Elements:**
- 🤖 **Robot meshes** (arm + chassis)
- 🟢 **Green line**: Actual end-effector trajectory (FK from executed joints)
- 🟣 **Purple dashed**: Target end-effector trajectory (from JSON)
- 🔴 **Red dots**: Reference trajectory waypoints (Pass 3 actual for ppForIk)
- ➡️ **Blue arrow**: Base heading direction
- 🎯 **Yellow marker**: Current target waypoint
- 🔵 **Blue discs**: Floor obstacles

**Stage Indicators:**
- **Stage A**: Blue arm ramp-up
- **Stage B**: Green base navigation
- **Stage C**: Red coordinated tracking

**Legend Colors:**
- `stageA` boundaries in the plot
- `stageB` boundaries in the plot
- `stageC` boundaries in the plot

---

## ⚙️ Advanced Options

### Full Artifact Package
```matlab
% Generate everything at once
artifacts = gik9dof.saveRunArtifacts(log, ...
    'RunLabel', 'my_run', ...
    'GeneratePlot', true, ...         % PNG plots
    'GenerateVideo', true, ...        % MP4 animation
    'SampleStep', 1, ...              % All frames
    'FrameRate', 30);                 % 30 FPS

% Outputs:
% - run_log.mat
% - run_report.json
% - run_plot.png
% - run_animation.mp4
```

### Custom Pipeline Configuration
```matlab
% Start with a profile
profile = gik9dof.config.loadPipelineProfile('ppForIk_tuned');

% Modify parameters
profile.execution.rateHz = 10;
profile.stageC.lookahead_distance = 0.8;
profile.chassis.vx_max = 1.5;

% Run with custom config
log = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                        'PipelineConfig', profile);
```

---

## 📚 More Information

| Document | Purpose |
|----------|---------|
| **README.md** | Project overview and structure |
| **projectDiagnosis.md** | ⭐ Comprehensive technical reference |
| **docs/SIMULATION_WORKFLOW_GUIDE.md** | Detailed simulation guide |
| **docs/ANIMATION_GENERATION_GUIDE.md** | Animation customization |
| **docs/UNIFIED_CONFIG_QUICK_REF.md** | Configuration system reference |

---

## 🐛 Troubleshooting

### "Cannot find function gik9dof"
```matlab
addpath(genpath('matlab'))  % Add package to MATLAB path
```

### "File not found: refEETrajs/..."
```matlab
cd /path/to/gikWBC9DOF  % Make sure you're in project root
ls refEETrajs/*.json    # Verify trajectory files exist
```

### Animation window is too small
```matlab
helperOpts = struct();
helperOpts.FigureScale = 1.5;  % Scale up window size

gik9dof.animateStagedWithHelper(log, 'HelperOptions', helperOpts);
```

### Video export fails
```matlab
% Try without video export first
gik9dof.animateStagedWithHelper(log);

% If that works, check MATLAB video codecs:
VideoWriter.getProfiles()
```

### High memory usage during animation
```matlab
% Subsample frames (every other frame)
gik9dof.animateStagedWithHelper(log, ...
    'ExportVideo', 'output.mp4', ...
    'SampleStep', 2);  % 2=every other, 3=every third, etc.
```

---

## ✅ Success Criteria

Your simulation is successful if:

- ✅ No errors during execution
- ✅ Mean EE error < 100mm (0.10m)
- ✅ Max EE error < 200mm (0.20m)
- ✅ Success rate > 95%
- ✅ Animation shows smooth motion
- ✅ Green line (actual) tracks purple dashed (target)

---

**Happy simulating! 🚀**

For questions, check `projectDiagnosis.md` or `HANDOVER.md`.
