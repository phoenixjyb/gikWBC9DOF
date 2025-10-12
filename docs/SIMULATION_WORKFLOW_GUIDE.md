# Simulation Workflow & Tools Reference

## Table of Contents
1. [Quick Start](#quick-start)
2. [Configuration Knobs](#configuration-knobs)
3. [Running Simulations](#running-simulations)
4. [Generating Outputs](#generating-outputs)
5. [Analysis & Debugging](#analysis--debugging)
6. [File Structure](#file-structure)

---

## Quick Start

### Minimal Staged Simulation
```matlab
% Add MATLAB paths
addpath(genpath('matlab'));

% Run with defaults
result = gik9dof.runStagedReference();

% Results saved to: results/YYYYMMDD_HHMMSS_staged_reference/
% - log_staged_ppForIk.mat    % Full trajectory log
% - run_report.json            % Metrics summary
% - run_animation.mp4          % Video export
```

### Load and Analyze Existing Run
```matlab
% Load log from results folder
logPath = 'results/20251010_155837_staged_10hz_legends3/log_staged_ppForIk.mat';
loaded = load(logPath);
log = loaded.log;

% Evaluate metrics
report = gik9dof.evaluateLog(log, 'Verbose', true);

% Regenerate animation
gik9dof.animateStagedWithHelper(log, 'ExportVideo', 'my_animation.mp4');

% Plot tracking errors
gik9dof.plotTrajectoryLog(log, 'ExportPath', 'tracking_plot.png');
```

---

## Configuration Knobs

### Entry Point: `run_staged_reference.m`
Located at project root. Modify these variables before running:

```matlab
% Identification
runLabel = "staged_purehyb_iter150_margin30";  % Results folder suffix

% Execution modes
executionMode = "ppForIk";         % "ppForIk" or "pureIk"
stageBMode = "pureHyb";            % "pureHyb" or "gikInLoop"

% Control parameters
rateHz = 10;                       % Control loop rate (10Hz or 30Hz tested)
maxIterations = 150;               % GIK solver iteration cap

% Planning parameters
useStageBHybridAStar = true;       % Use Hybrid A* planner (vs interpolation)
distanceMargin = 0.10;             % Obstacle inflation margin (meters)

% Pure pursuit tuning
stageBLookaheadDistance = 0.6;     % Lookahead for Stage B (meters)
stageBDesiredLinearVelocity = 0.5; % Target velocity Stage B (m/s)
stageBMaxAngularVelocity = 2.0;    % Yaw rate limit Stage B (rad/s)
```

### Execution Mode Matrix

| `executionMode` | `stageBMode` | Description | Use Case |
|----------------|--------------|-------------|----------|
| `"ppForIk"` | `"pureHyb"` | Pure pursuit drives base, GIK tracks arm | **Recommended** for smooth tracking |
| `"ppForIk"` | `"gikInLoop"` | GIK plans base, PP refines, GIK tracks arm | More conservative, slower |
| `"pureIk"` | `"pureHyb"` | GIK only, base from Stage B planner | Debugging base planning |
| `"pureIk"` | `"gikInLoop"` | Full GIK pipeline | Legacy mode, not recommended |

### Advanced Options (via `gik9dof.runStagedReference`)

#### Stage B Planning
```matlab
result = gik9dof.runStagedReference(...
    'StageBHybridResolution', 0.05, ...          % Grid resolution (m)
    'StageBHybridSafetyMargin', 0.1, ...         % Safety inflation (m)
    'StageBHybridMinTurningRadius', 0.5, ...     % Min turn radius (m)
    'StageBHybridMotionPrimitiveLength', 0.2, ... % Primitive length (m)
    'StageBUseReedsShepp', true, ...             % Enable RS shortcuts
    'StageBUseClothoid', true);                  % Enable clothoid smoothing
```

#### Reeds-Shepp Tuning
```matlab
rsParams = gik9dof.control.defaultReedsSheppParams();
rsParams.lambdaCusp = 1.0;        % Cusp penalty (lower = more cusps allowed)
rsParams.allowReverse = true;     % Permit reverse segments
rsParams.iters = 200;             % Shortcut attempts

result = gik9dof.runStagedReference(...
    'StageBReedsSheppParams', rsParams);
```

#### Stage C Refinement
```matlab
result = gik9dof.runStagedReference(...
    'StageCUseBaseRefinement', false, ...  % Disable Stage C base smoothing
    'StageCLookaheadDistance', 0.8, ...    % Stage C lookahead (m)
    'StageCMaxLinearSpeed', 1.5);          % Stage C max velocity (m/s)
```

#### Chassis Control Modes
```matlab
% -1 = use chassis profile default
%  0 = legacy differential drive
%  1 = heading control
%  2 = pure pursuit (recommended)

result = gik9dof.runStagedReference(...
    'StageBChassisControllerMode', 2, ...
    'StageCChassisControllerMode', 2);
```

### Chassis Profiles (`config/chassis_profiles.yaml`)

Edit `wide_track` profile to adjust:
```yaml
profiles:
  wide_track:
    track: 0.574                # m (track width)
    wheel_base: 0.36            # m
    vx_max: 1.5                 # m/s forward limit
    accel_limit: 0.8            # m/s^2 (changed from 1.2)
    lookahead_base: 0.8         # m (changed from 0.6)
    lookahead_vel_gain: 0.30    # s (velocity-dependent lookahead)
    heading_kp: 1.2             # Proportional heading gain
    curvature_slowdown:
      kappa_threshold: 0.9      # 1/m (curvature for slowdown)
      vx_reduction: 0.6         # velocity scale at high curvature
```

---

## Running Simulations

### Standard Staged Run
```matlab
% At project root
run_staged_reference
```

### Programmatic Run with Custom Config
```matlab
result = gik9dof.runStagedReference(...
    'RunLabel', 'my_test_run', ...
    'RateHz', 10, ...
    'ExecutionMode', 'ppForIk', ...
    'StageBMode', 'pureHyb', ...
    'UseStageBHybridAStar', true, ...
    'StageBUseReedsShepp', true, ...
    'StageBUseClothoid', true, ...
    'DistanceMargin', 0.1, ...
    'SaveLog', true);

% Access results
log = result.log;
fprintf('Results in: %s\n', result.resultsDir);
```

### Environment Customization
```matlab
env = gik9dof.environmentConfig();

% Modify obstacles
env.FloorDiscs(1).Radius = 0.3;
env.FloorDiscs(1).SafetyMargin = 0.05;

% Adjust margins
env.DistanceMargin = 0.12;
env.DistanceWeight = 0.5;

% Run with custom environment
log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'staged', ...
    'EnvironmentConfig', env);
```

---

## Generating Outputs

### Animation Generation

#### From Log Struct
```matlab
% Simple animation (no video export)
gik9dof.animateStagedWithHelper(log);

% Export to MP4
gik9dof.animateStagedWithHelper(log, ...
    'ExportVideo', 'my_animation.mp4', ...
    'FrameRate', 30, ...
    'SampleStep', 1);  % 1=all frames, 2=every other, etc.
```

#### Custom Animation Options
```matlab
helperOpts = struct();
helperOpts.ArrowLength = 0.25;         % Heading arrow size (m)
helperOpts.PlaybackSpeed = 1.0;        % 1=realtime, 0.5=half speed
helperOpts.VisualAlpha = 0.6;          % Robot mesh transparency
helperOpts.ChassisAlpha = 0.35;        % Chassis mesh transparency
helperOpts.FigureScale = 1.0;          % Window size scale

gik9dof.animateStagedWithHelper(log, ...
    'HelperOptions', helperOpts, ...
    'ExportVideo', 'output.mp4');
```

#### Stage-Specific Animation
```matlab
% Animate only Stage C (tracking phase)
gik9dof.viz.animate_whole_body(robot, armJointNames, ...
    armTrajectory, armTimes, basePose, baseTimes, eePoses, ...
    'StageSelection', 'tracking', ...  % 'all', 'arm', 'chassis', 'tracking'
    'VideoFile', 'stage_c_only.mp4');
```

### Plotting

#### Tracking Error Plots
```matlab
% Generate and display
gik9dof.plotTrajectoryLog(log);

% Export to PNG
gik9dof.plotTrajectoryLog(log, 'ExportPath', 'tracking.png');

% Plot specific joints only
gik9dof.plotTrajectoryLog(log, 'JointIndices', [4 5 6 7 8 9]);  % Arm only
```

#### Custom Analysis Plots
```matlab
% Extract metrics from log
time = log.time;
eeError = log.positionErrorNorm;
baseStates = log.stageLogs.stageC.execBaseStates;

% Plot EE error over time
figure; plot(time(2:end), eeError * 1000);  % Convert to mm
xlabel('Time (s)'); ylabel('EE Position Error (mm)');
title('End-Effector Tracking Error');
grid on;

% Plot base path
figure; plot(baseStates(:,1), baseStates(:,2), 'b-', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)');
title('Base Trajectory');
axis equal; grid on;
```

### Full Artifact Package
```matlab
artifacts = gik9dof.saveRunArtifacts(log, ...
    'RunLabel', 'my_run', ...
    'GeneratePlot', true, ...
    'GenerateVideo', true, ...
    'SampleStep', 2, ...          % Subsample animation frames
    'FrameRate', 30);

% Outputs:
% - run_log.mat          % Full log
% - run_report.json      % Metrics JSON
% - run_plot.png         % Tracking plots
% - run_animation.mp4    % Video
```

---

## Analysis & Debugging

### Metrics Evaluation
```matlab
% Detailed report printed to console
report = gik9dof.evaluateLog(log, 'Verbose', true);

% Key metrics
fprintf('Mean EE error: %.3f m\n', report.positionMean);
fprintf('Max EE error: %.3f m\n', report.positionMax);
fprintf('Mean solver iters: %.1f\n', report.solverIterationsMean);
fprintf('Success rate: %.1f%%\n', report.successRate * 100);

% Access raw data
eeErrors = log.positionErrorNorm;           % Per-waypoint EE error
solverIters = log.iterations;               % Per-waypoint solver iterations
exitFlags = log.exitFlags;                  % Solver exit status
```

### Stage-Specific Diagnostics

#### Stage B Analysis
```matlab
stageB = log.stageLogs.stageB;

% Path quality
basePath = stageB.execBaseStates;
curvature = computeCurvature(basePath);  % Helper function needed
fprintf('Stage B mean curvature: %.2f rad/m\n', mean(abs(curvature)));

% Planning metrics (if available)
if isfield(stageB, 'planner')
    planner = stageB.planner;
    if isfield(planner, 'rsSmoothing')
        rsInfo = planner.rsSmoothing;
        fprintf('RS improvements: %d/%d\n', ...
            rsInfo.improvements, rsInfo.iterationsExecuted);
    end
end

% Base goal vs achieved
if isfield(stageB, 'goalBase') && isfield(stageB, 'achievedBase')
    posErr = norm(stageB.achievedBase(1:2) - stageB.goalBase(1:2));
    yawErr = rad2deg(abs(wrapToPi(stageB.achievedBase(3) - stageB.goalBase(3))));
    fprintf('Stage B docking error: %.3f m, %.2f deg\n', posErr, yawErr);
end
```

#### Stage C Analysis
```matlab
stageC = log.stageLogs.stageC;

% Compare reference vs executed
refBase = stageC.referenceBaseStates;
execBase = stageC.execBaseStates;
posDeltas = vecnorm(execBase(:,1:2) - refBase(:,1:2), 2, 2);
fprintf('Stage C base tracking error: %.3f ± %.3f m\n', ...
    mean(posDeltas), std(posDeltas));

% EE tracking (if reference available)
if isfield(stageC, 'referenceInitialIk')
    refEE = stageC.referenceInitialIk.eePositions;
    execEE = stageC.eePositions;
    eeDeltas = vecnorm(execEE - refEE, 2, 1);
    fprintf('Stage C EE ref vs exec: %.3f m mean, %.3f m max\n', ...
        mean(eeDeltas), max(eeDeltas));
end

% Pure pursuit diagnostics
if isfield(stageC, 'purePursuit')
    pp = stageC.purePursuit;
    if isfield(pp, 'simulation') && isfield(pp.simulation, 'status')
        statuses = pp.simulation.status;
        finished = sum([statuses.isFinished]);
        fprintf('Pure pursuit finished: %d/%d waypoints\n', ...
            finished, numel(statuses));
    end
end
```

### Comparing Runs
```matlab
% Load two runs
log1 = load('results/run_A/log_staged_ppForIk.mat').log;
log2 = load('results/run_B/log_staged_ppForIk.mat').log;

% Compare key metrics
report1 = gik9dof.evaluateLog(log1, 'Verbose', false);
report2 = gik9dof.evaluateLog(log2, 'Verbose', false);

fprintf('Run A: %.3fm mean, %.3fm max\n', ...
    report1.positionMean, report1.positionMax);
fprintf('Run B: %.3fm mean, %.3fm max\n', ...
    report2.positionMean, report2.positionMax);

% Side-by-side plot
figure;
subplot(1,2,1);
plot(log1.time(2:end), log1.positionErrorNorm * 1000);
title('Run A'); xlabel('Time (s)'); ylabel('EE Error (mm)');
subplot(1,2,2);
plot(log2.time(2:end), log2.positionErrorNorm * 1000);
title('Run B'); xlabel('Time (s)'); ylabel('EE Error (mm)');
```

### Regenerating Animations from Batch Results
```matlab
% Regenerate all animations in a results folder
regenerate_stageb_comparison_animations('results/20251009_ctrl_mode_compare');

% Regenerate with custom options
regenerate_iter_study_animations('results/20251007_122040_gik_iter_study_iterCompare', ...
    'FrameRate', 30, ...
    'SampleStep', 1);
```

---

## File Structure

### Results Folder Layout
```
results/YYYYMMDD_HHMMSS_run_label/
├── log_staged_ppForIk.mat       # Full trajectory log (primary output)
├── run_log.mat                  # Copy of log (if using saveRunArtifacts)
├── run_report.json              # Metrics summary (JSON format)
├── run_plot.png                 # Tracking error plots
├── run_animation.mp4            # Whole-body animation
└── anim/                        # Additional animation variants
    ├── staged_run.mp4           # Stage-specific animation
    └── plots/                   # Frame-by-frame exports
```

### Log Structure (Staged Mode)
```matlab
log = struct(...
    % Unified trajectory
    'qTraj',            [9×N],           % Joint trajectory (9 DOF)
    'time',             [1×N],           % Timestamps (seconds)
    'timestamps',       [1×(N-1)],       % Step timestamps
    'rateHz',           10,              % Control rate
    
    % Tracking metrics
    'eePositions',      [3×(N-1)],       % Executed EE positions
    'targetPositions',  [3×(N-1)],       % Target EE positions
    'positionErrorNorm',[1×(N-1)],       % Per-waypoint position error
    'orientationErrorAngle', [1×(N-1)],  % Per-waypoint orientation error
    
    % Solver diagnostics
    'iterations',       [1×(N-1)],       % Solver iterations per waypoint
    'exitFlags',        [1×(N-1)],       % Exit status codes
    'successMask',      [1×(N-1)],       % Boolean success per waypoint
    
    % Stage-specific logs
    'stageLogs', struct(...
        'stageA', struct(...             % Arm ramp-up
            'qTraj', [9×NA], ...
            'eePositions', [3×NA], ...
            ...),
        'stageB', struct(...             % Base alignment
            'qTraj', [9×NB], ...
            'pathStates', [NB×3], ...    % Planned base path
            'execBaseStates', [NB×3], ... % Executed base path
            'goalBase', [1×3], ...       % Target base pose [x y yaw]
            'achievedBase', [1×3], ...   % Final base pose
            'planner', struct(...        % Planner diagnostics
                'statesRaw', [...],
                'rsSmoothing', struct(...), ...
                'hcSmoothing', struct(...)), ...
            ...),
        'stageC', struct(...             % Full tracking
            'qTraj', [9×NC], ...
            'referenceBaseStates', [NC×3], ... % Reference base from GIK
            'execBaseStates', [NC×3], ...      % Executed base from PP
            'referenceInitialIk', struct(...   % Stage C reference GIK pass
                'eePositions', [3×NC], ...
                ...),
            'purePursuit', struct(...
                'referencePath', [NC×3], ...
                'executedPath', [(NC-1)×3], ...
                'commands', [(NC-1)×2], ...    % [Vx, Wz]
                'wheelSpeeds', [(NC-1)×2], ... % [left, right]
                'status', struct([...]), ...
                'refinement', struct(...)), ... % RS/clothoid info
            ...)),
    
    % Reference trajectory
    'referenceTrajectory', struct(...
        'Poses', [4×4×M], ...            % Original JSON waypoints
        'EndEffectorPositions', [3×M], ...
        'EndEffectorName', 'left_gripper_link'),
    
    % Environment
    'floorDiscs', struct([...]),         % Obstacle definitions
    'distanceSpecs', struct([...]),      % Distance constraints
    
    % Metadata
    'mode', 'staged', ...
    'simulationMode', 'ppForIk', ...     % Execution mode
    'stageBMode', 'pureHyb', ...         % Stage B mode
    'chassisParams', struct(...));       % Chassis configuration
```

### Key Functions Reference

| Function | Purpose | Example |
|----------|---------|---------|
| `gik9dof.runStagedReference()` | Main entry point for staged runs | `result = gik9dof.runStagedReference('RunLabel', 'test');` |
| `gik9dof.trackReferenceTrajectory()` | Lower-level trajectory tracking | `log = gik9dof.trackReferenceTrajectory('Mode', 'staged');` |
| `gik9dof.evaluateLog()` | Compute metrics from log | `report = gik9dof.evaluateLog(log);` |
| `gik9dof.animateStagedWithHelper()` | Generate staged animation | `gik9dof.animateStagedWithHelper(log, 'ExportVideo', 'out.mp4');` |
| `gik9dof.plotTrajectoryLog()` | Plot tracking errors | `gik9dof.plotTrajectoryLog(log, 'ExportPath', 'plot.png');` |
| `gik9dof.saveRunArtifacts()` | Save complete artifact package | `artifacts = gik9dof.saveRunArtifacts(log, 'RunLabel', 'test');` |
| `gik9dof.environmentConfig()` | Load environment configuration | `env = gik9dof.environmentConfig();` |

---

## Common Workflows

### Workflow 1: Parameter Tuning Iteration
```matlab
% 1. Edit parameters in run_staged_reference.m
% 2. Run simulation
run_staged_reference

% 3. Quick check of results
report = gik9dof.evaluateLog(result.log);

% 4. If promising, regenerate with full animation
gik9dof.animateStagedWithHelper(result.log, ...
    'ExportVideo', fullfile(result.resultsDir, 'full_animation.mp4'));
```

### Workflow 2: Batch Parameter Sweep
```matlab
% Define parameter grid
margins = [0.05, 0.10, 0.15];
lambdas = [0.5, 1.0, 2.0];

results = cell(numel(margins), numel(lambdas));

for i = 1:numel(margins)
    for j = 1:numel(lambdas)
        rsParams = gik9dof.control.defaultReedsSheppParams();
        rsParams.lambdaCusp = lambdas(j);
        
        results{i,j} = gik9dof.runStagedReference(...
            'RunLabel', sprintf('margin%.2f_lambda%.1f', margins(i), lambdas(j)), ...
            'DistanceMargin', margins(i), ...
            'StageBReedsSheppParams', rsParams, ...
            'SaveLog', true);
        
        fprintf('[%d,%d] Complete: %s\n', i, j, results{i,j}.resultsDir);
    end
end

% Compare results
for i = 1:numel(margins)
    for j = 1:numel(lambdas)
        rep = gik9dof.evaluateLog(results{i,j}.log, 'Verbose', false);
        fprintf('margin=%.2f, lambda=%.1f: %.3fm mean, %.3fm max\n', ...
            margins(i), lambdas(j), rep.positionMean, rep.positionMax);
    end
end
```

### Workflow 3: Debug Failed Run
```matlab
% Load log from failed run
log = load('results/failed_run/log_staged_ppForIk.mat').log;

% Check solver diagnostics
failIdx = find(~log.successMask);
fprintf('Failed at waypoints: %s\n', mat2str(failIdx));

% Inspect solver iterations
fprintf('Iterations at failures:\n');
disp(log.iterations(failIdx));

% Check exit flags
fprintf('Exit flags at failures:\n');
disp(log.exitFlags(failIdx));

% Visualize failed segment
if ~isempty(failIdx)
    firstFail = failIdx(1);
    % Extract trajectory up to failure
    logPartial = log;
    logPartial.qTraj = log.qTraj(:, 1:firstFail+1);
    gik9dof.animateStagedWithHelper(logPartial);
end
```

---

## Tips & Best Practices

1. **Always start with defaults** from `run_staged_reference.m` before tuning.

2. **Use 10Hz for production**, 30Hz for high-speed testing (but less smooth).

3. **Stage B mode recommendations**:
   - Use `"pureHyb"` for best performance
   - Use `"gikInLoop"` only for debugging GIK base planning

4. **RS/Clothoid tuning**:
   - Lower `lambdaCusp` for smoother paths (accepts more cusps)
   - Enable `allowReverse` if path allows backing up
   - Increase `clothoidDiscretization` for gentler curves

5. **Animation performance**:
   - Set `SampleStep=2` or higher for faster rendering
   - Use `PlaybackSpeed=0.5` to slow down visualization

6. **Storage management**:
   - Results folders can be large (50-200 MB each with video)
   - Delete old test runs: `rm -rf results/2025100*_test*`

7. **Metric targets**:
   - Mean EE error: <0.10m (good), <0.08m (excellent)
   - Max EE error: <0.20m (good), <0.15m (excellent)
   - Solver iterations: <500 (good), <300 (excellent)

---

## Troubleshooting

### Animation doesn't show all stages
**Solution**: Check `StageBoundaries` detection in `animateStagedWithHelper`. Ensure `stageLogs` field exists.

### Video export fails
**Solution**: Check MATLAB Video I/O support. Try different codec or reduce `FrameRate`.

### Solver fails at specific waypoints
**Solution**: 
1. Check `log.exitFlags` at failure indices
2. Increase `maxIterations` or relax constraints
3. Visualize failed segment for physical issues

### Base path has zig-zags
**Solution**: 
1. Lower `lambdaCusp` (e.g., 3.0 → 1.0)
2. Enable `StageBUseReedsShepp` and `StageBUseClothoid`
3. Increase `StageBHybridMotionPrimitiveLength` for smoother primitives

### EE error too high
**Solution**:
1. Disable `StageCUseBaseRefinement` to preserve reference
2. Lower `accel_limit` in chassis profile
3. Increase `StageCLookaheadDistance` for better preview

---

## Next Steps

See `ALGORITHM_IMPROVEMENT_PLAN.md` for systematic parameter optimization and testing protocols.
