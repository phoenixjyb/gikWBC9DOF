%% GIK 9-DOF Toolbox - Command Reference
% =========================================
% Quick reference guide for running simulations, generating animations,
% and analyzing results for the 9-DOF mobile manipulator project.
%
% Last updated: 2025-10-11

%% =============================================================================
%% 1. VIEWING & VISUALIZATION
%% =============================================================================

% --- View trajectory waypoints from JSON ---
% Visualizes the end-effector trajectory in 3D (X, Y, Z)
% Displays statistics and multiple view angles
view_json_waypoints

% Output: Figure with 4 subplots (3D, XY, XZ, YZ views)
% Shows: First/last 10 waypoints, statistics, trajectory path

%% =============================================================================
%% 2. RUNNING SIMULATIONS
%% =============================================================================

% --- Single simulation with baseline parameters (NO obstacles) ---
% Runs fresh simulation with floor discs disabled
% Generates animation with all three fixes applied
run_fresh_sim_with_animation

% Output: 
%   - Log: results/YYYYMMDD_HHMMSS_fresh_sim_no_discs/log_staged_ppForIk.mat
%   - Video: results/YYYYMMDD_HHMMSS_fresh_sim_no_discs/animation_no_discs.mp4
% Duration: ~2-3 min simulation + ~2 min animation = ~5 min total
% Features:
%   - Floor discs disabled (no obstacles)
%   - Baseline parameters (SafetyMargin=0.10, etc.)
%   - All three animation fixes applied

% --- Single simulation using runStagedReference (WITH obstacles) ---
% High-level wrapper that uses environment config (includes obstacles)
gik9dof.runStagedReference( ...
    'RunLabel', 'test_run', ...
    'RateHz', 10, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'SaveLog', true)

% Output: results/YYYYMMDD_HHMMSS_test_run/log_staged_ppForIk.mat

% --- Direct simulation with custom parameters ---
% Lower-level control for specific parameter testing
env = gik9dof.environmentConfig();
env.FloorDiscs = struct([]);  % Disable obstacles if desired

log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'RateHz', 10, ...
    'EnvironmentConfig', env, ...
    'FloorDiscs', env.FloorDiscs, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'StageBHybridSafetyMargin', 0.10);

%% =============================================================================
%% 3. PARAMETRIC STUDIES & SWEEPS
%% =============================================================================

% --- Stage B parameter sweep ---
% Sweeps over hybrid A*/RS/clothoid parameters
% Tests multiple configurations in parallel
% Located in: results/YYYYMMDD_HHMMSS_SWEEP_STAGEB_quick_test/

% To view sweep results:
matlab -batch "load('results/[FOLDER]/sweep_results.mat'); disp(results(1))"

% Common sweep parameters:
%   - SafetyMargin: [0.05, 0.10, 0.15, 0.20]
%   - LambdaCusp: [0.5, 1.0, 1.5, 2.0]
%   - MaxRSIterations: [100, 150, 200, 300]
%   - AllowReverse: [true, false]
%   - ClothoidDiscretization: [0.05, 0.08, 0.10]

% --- Comprehensive parametric study ---
% Full factorial design across multiple parameters
% Located in: results/parametric_study_YYYYMMDD_HHMMSS/
% Contains: parametric_study_results.mat with all configurations

% To load and analyze:
load('results/parametric_study_[TIMESTAMP]/parametric_study_results.mat');
% Structure: results(i).log, results(i).params, results(i).metrics

%% =============================================================================
%% 4. ANIMATION GENERATION
%% =============================================================================

% --- Generate animation from existing log ---
% Option 1: Using wrapper (recommended)
gik9dof.animateStagedWithHelper(log, ...
    'SampleStep', 2, ...        % Downsample by 2x
    'FrameRate', 20, ...         % 20 fps video
    'ExportVideo', 'output.mp4');

% Option 2: Direct call with more control
gik9dof.animate_whole_body(log.qTraj, robot, ...
    'SampleStep', 2, ...
    'FrameRate', 20, ...
    'VideoFile', 'output.mp4', ...
    'StageBoundaries', [26, 96, 171], ...  % Sampled boundaries
    'StageC_EE_Path', log.stageLogs.stageC.targetPositions);

% --- Generate animation from saved log file ---
generate_animation_from_saved_log  % Loads specific log and generates video

% --- Regenerate animations for parametric study ---
generate_parametric_animations  % Batch generation for all study results

%% =============================================================================
%% 5. DEBUGGING & DIAGNOSTICS
%% =============================================================================

% --- Debug stage boundary timing ---
debug_stage_boundaries
% Output: Prints raw and sampled frame boundaries for each stage

% --- Debug sampling mismatch ---
debug_sampling_mismatch
% Output: Shows transformation from raw to sampled frame space

% --- Debug Stage C EE path source ---
debug_stagec_ee_path
% Output: Compares different EE path data sources (targetPositions vs eePositions)

% --- Test individual animation fixes ---
test_stage_sync_fix          % Test Fix 1: Stage boundary sampling
test_complete_fix            % Test Fixes 1+2: Boundaries + red dot indexing
test_stagec_path_fix         % Test Fix 3: Stage C path data source

% --- Analyze stage collisions ---
analyze_stage_collisions(log)
% Output: Collision statistics, clearance analysis for each stage

%% =============================================================================
%% 6. DATA INSPECTION
%% =============================================================================

% --- Load and inspect simulation log ---
logPath = 'results/[FOLDER]/log_staged_ppForIk.mat';
data = load(logPath);
log = data.log;

% Key log fields:
%   log.qTraj                    - Full joint trajectory [15 × N]
%   log.stageLogs.stageA         - Stage A specific data
%   log.stageLogs.stageB         - Stage B specific data
%   log.stageLogs.stageC         - Stage C specific data
%   log.stageLogs.stageC.targetPositions  - Desired EE trajectory [3 × M]
%   log.stageLogs.stageC.eePositions      - Actual EE positions [3 × M]
%   log.stageLogs.stageC.positionError    - Tracking error [3 × M]

% --- Check Stage C tracking error ---
eeError = log.stageLogs.stageC.positionError;
errorNorms = sqrt(sum(eeError.^2, 1));
fprintf('Mean error: %.4f m\n', mean(errorNorms));
fprintf('Max error: %.4f m\n', max(errorNorms));
fprintf('RMS error: %.4f m\n', rms(errorNorms));

% --- Inspect stage frame counts ---
stageA_frames = size(log.stageLogs.stageA.qTraj, 2);
stageB_frames = size(log.stageLogs.stageB.qTraj, 2);
stageC_frames = size(log.stageLogs.stageC.qTraj, 2);
fprintf('Stage A: %d frames (%.1f sec @ 10Hz)\n', stageA_frames, stageA_frames/10);
fprintf('Stage B: %d frames (%.1f sec @ 10Hz)\n', stageB_frames, stageB_frames/10);
fprintf('Stage C: %d frames (%.1f sec @ 10Hz)\n', stageC_frames, stageC_frames/10);

%% =============================================================================
%% 7. USEFUL TERMINAL COMMANDS
%% =============================================================================

% --- Find recent results folders ---
% ls -lt results/ | head -20

% --- Find specific log files ---
% find results -name "*.mat" -type f | grep -E "(test_comprehensive|SWEEP|staged_reference)" | head -20

% --- Find recently created files ---
% find results -name "*.mat" -type f -mmin -10

% --- Check video files ---
% ls -lh results/**/*.mp4 | tail -10

% --- Check MATLAB process ---
% ps aux | grep -i matlab | grep -v grep

%% =============================================================================
%% 8. COMMON WORKFLOWS
%% =============================================================================

% --- Workflow 1: Quick single test run ---
% 1. Run simulation with animation
run_fresh_sim_with_animation
% 2. Check output video in results folder

% --- Workflow 2: Test specific parameters ---
% 1. Modify parameters in script or call directly
env = gik9dof.environmentConfig();
env.FloorDiscs = struct([]);  % Disable obstacles
log = gik9dof.trackReferenceTrajectory('Mode', 'staged', ...
    'EnvironmentConfig', env, 'StageBHybridSafetyMargin', 0.15);
% 2. Generate animation
gik9dof.animateStagedWithHelper(log, 'ExportVideo', 'test.mp4');

% --- Workflow 3: Parametric study ---
% 1. Run parametric sweep (use existing sweep script)
% 2. Load results
load('results/parametric_study_[TIMESTAMP]/parametric_study_results.mat');
% 3. Analyze metrics
for i = 1:length(results)
    fprintf('Config %d: mean error = %.4f m\n', i, results(i).metrics.meanError);
end
% 4. Generate animations for best configs
generate_parametric_animations

% --- Workflow 4: Debug animation issues ---
% 1. Load log
data = load('results/[FOLDER]/log_staged_ppForIk.mat');
log = data.log;
% 2. Check data sources
size(log.stageLogs.stageC.targetPositions)  % Should be 3×148
size(log.stageLogs.stageC.eePositions)      % Should be 3×N
% 3. Run debug scripts
debug_stage_boundaries
debug_stagec_ee_path
% 4. Test fixes individually
test_stagec_path_fix

%% =============================================================================
%% 9. KEY PARAMETERS REFERENCE
%% =============================================================================

% --- Baseline tuned parameters (from parametric study) ---
params.SafetyMargin = 0.10;              % Stage B hybrid A* safety margin
params.LambdaCusp = 1.0;                 % RS cusp weight
params.MaxRSIterations = 200;            % RS shortcut max iterations
params.AllowReverse = true;              % Allow reverse motions
params.ClothoidDiscretization = 0.08;    % Clothoid sampling resolution
params.Lookahead = 0.80;                 % Pure pursuit lookahead distance
params.AccelLimit = 0.80;                % Linear velocity limit
params.HeadingKp = 1.0;                  % Heading controller gain

% --- Animation parameters ---
anim.SampleStep = 2;                     % Downsample factor (1=no downsample)
anim.FrameRate = 20;                     % Video frame rate (fps)

% --- Simulation control ---
sim.RateHz = 10;                         % Control loop frequency
sim.MaxIterations = 150;                 % IK solver max iterations
sim.UseStageBHybridAStar = true;         % Enable hybrid A* for Stage B
sim.StageBMode = 'pureHyb';              % 'pureHyb' or 'gikInLoop'
sim.StageCUseBaseRefinement = true;      % Enable RS/clothoid smoothing

%% =============================================================================
%% 10. ANIMATION FIX SUMMARY
%% =============================================================================

% Three fixes applied to animation system:

% FIX 1: Stage Boundary Sampling Transformation
%   Problem: Boundaries in raw frame space [51, 190, 339], animation in sampled
%   Solution: Transform to sampled space [26, 96, 171] using SampleStep
%   Location: matlab/+gik9dof/animateStagedWithHelper.m lines 111-120

% FIX 2: Red Dot Stage-Relative Indexing
%   Problem: Red dot indexed globally but Stage C data starts at frame 97
%   Solution: Use stageCLocalIdx = k - stageCFirstIdx + 1
%   Location: matlab/+gik9dof/animate_whole_body.m lines 181-199, 466-488

% FIX 3: Stage C EE Path Data Source
%   Problem: Used full trajectory from homebase instead of Stage C desired
%   Solution: Priority: referenceInitialIk.eePositions > stageC.targetPositions
%   Location: matlab/+gik9dof/animateStagedWithHelper.m lines 42-74

%% =============================================================================
%% 11. FILE STRUCTURE REFERENCE
%% =============================================================================

% Key files and their purposes:
%
% Simulation & Control:
%   matlab/+gik9dof/runStagedReference.m          - High-level simulation wrapper
%   matlab/+gik9dof/trackReferenceTrajectory.m    - Core trajectory controller
%   matlab/+gik9dof/runStagedTrajectory.m         - Staged execution logic
%   matlab/+gik9dof/environmentConfig.m           - Environment & obstacle config
%
% Animation:
%   matlab/+gik9dof/animateStagedWithHelper.m     - Animation wrapper (with fixes)
%   matlab/+gik9dof/animate_whole_body.m          - Core animation renderer
%   matlab/renderWholeBodyAnimation.m             - Alternative renderer
%
% Analysis & Debugging:
%   matlab/analyze_stage_collisions.m             - Collision analysis
%   debug_stage_boundaries.m                      - Stage timing diagnostics
%   debug_sampling_mismatch.m                     - Frame space debugging
%   debug_stagec_ee_path.m                        - EE path source debugging
%
% Visualization:
%   view_json_waypoints.m                         - View trajectory waypoints
%   matlab/plotJsonPath.m                         - Plot JSON trajectory
%
% Batch Processing:
%   generate_parametric_animations.m              - Batch animation generation
%   generate_animation_from_saved_log.m           - Single log animation
%
% Testing:
%   test_stage_sync_fix.m                         - Test Fix 1
%   test_complete_fix.m                           - Test Fixes 1+2
%   test_stagec_path_fix.m                        - Test Fix 3
%
% Main Scripts:
%   run_fresh_sim_with_animation.m                - Single run with animation
%   run_environment_compare.m                     - Compare configurations
%   run_staged_reference.m                        - Reference run script

%% =============================================================================
%% 12. DATA FLOW & VERIFICATION
%% =============================================================================

% See DATA_FLOW_ANALYSIS.m for comprehensive explanation of:
%   - What trajectory the robot follows in simulation
%   - What data is used to render the robot in animation
%   - What the red dot (Stage C reference) represents
%   - Why robot EE and red dot may not align (tracking error)

% Quick verification: Check if animation matches simulation
logPath = 'results/[FOLDER]/log_staged_ppForIk.mat';
data = load(logPath);
log = data.log;

% Recompute EE from qTraj and compare with logged eePositions
robot = gik9dof.createRobotModel();
stageC = log.stageLogs.stageC;
T = getTransform(robot, stageC.qTraj(:,1), 'left_gripper_link');
eeFromFK = T(1:3,4);
eeLogged = stageC.eePositions(:,1);
fprintf('FK vs Logged: %.6f m difference\n', norm(eeFromFK - eeLogged));
% Should be ~0, confirming qTraj → FK → eePositions consistency

% Check tracking error
trackingError = stageC.targetPositions - stageC.eePositions;
errorNorms = sqrt(sum(trackingError.^2, 1));
fprintf('Tracking: mean=%.4f max=%.4f RMS=%.4f m\n', ...
    mean(errorNorms), max(errorNorms), rms(errorNorms));
% This is the gap you see between robot EE and red dot!

%% =============================================================================
%% 13. QUICK REFERENCE COMMANDS
%% =============================================================================

% View trajectory:                view_json_waypoints
% Run single simulation:          run_fresh_sim_with_animation
% Generate animation from log:    generate_animation_from_saved_log
% Debug stage boundaries:         debug_stage_boundaries
% Test animation fixes:           test_stagec_path_fix
% Analyze results:                analyze_stage_collisions(log)
% Understand data flow:           edit DATA_FLOW_ANALYSIS.m

fprintf('\n=== GIK 9-DOF Toolbox Command Reference ===\n');
fprintf('This file contains documentation only.\n');
fprintf('Copy and paste commands to run them.\n');
fprintf('See sections 1-12 above for details.\n\n');
