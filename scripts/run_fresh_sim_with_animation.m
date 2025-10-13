%% Run Fresh Simulation and Generate Animation
% This script:
% 1. Runs a fresh simulation with baseline tuned parameters
% 2. Generates animation with all three fixes applied
%
% All fixes included:
% - Fix 1: Stage boundary sampling correction
% - Fix 2: Red dot stage-relative indexing  
% - Fix 3: Stage C EE path from targetPositions (not full trajectory)
%
% TODO: Migrate to unified config system (see UNIFIED_CONFIG_MIGRATION_COMPLETE.md)
%   New recommended approach:
%     cfg = gik9dof.loadPipelineProfile('default');
%     % Customize if needed:
%     cfg.stage_b.hybrid_safety_margin = 0.10;
%     cfg.chassis.lookahead_base = 0.80;
%     log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg, 'Mode', 'staged');
%   Benefits: Single source of truth, easier to maintain

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('========================================\n');
fprintf('  Fresh Simulation + Animation\n');
fprintf('========================================\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Configuration
jsonFile = '1_pull_world_scaled.json';
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
resultsDir = sprintf('results/%s_fresh_sim_no_discs', timestamp);
videoFile = sprintf('%s/animation_no_discs.mp4', resultsDir);

% Baseline tuned parameters (from parametric study)
config = struct();
config.SafetyMargin = 0.10;
config.LambdaCusp = 1.0;
config.MaxRSIterations = 200;
config.AllowReverse = true;
config.ClothoidDiscretization = 0.08;
config.Lookahead = 0.80;
config.AccelLimit = 0.80;
config.HeadingKp = 1.0;

fprintf('--- Configuration ---\n');
fprintf('  JSON: %s\n', jsonFile);
fprintf('  Results: %s\n', resultsDir);
fprintf('  Video: %s\n\n', videoFile);

fprintf('--- Parameters ---\n');
fprintf('  Safety Margin: %.2f\n', config.SafetyMargin);
fprintf('  Lambda Cusp: %.1f\n', config.LambdaCusp);
fprintf('  Max RS Iterations: %d\n', config.MaxRSIterations);
fprintf('  Allow Reverse: %s\n', mat2str(config.AllowReverse));
fprintf('  Clothoid Discretization: %.2f\n', config.ClothoidDiscretization);
fprintf('  Pure Pursuit Lookahead: %.2f\n', config.Lookahead);
fprintf('  Accel Limit: %.2f\n', config.AccelLimit);
fprintf('  Heading Kp: %.1f\n\n', config.HeadingKp);

%% Create results directory
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Step 1: Run Simulation
fprintf('========================================\n');
fprintf('  STEP 1: Running Simulation\n');
fprintf('========================================\n');
fprintf('⚠️  FLOOR DISCS DISABLED - Running without obstacles\n');
fprintf('Please wait... (this may take a few minutes)\n\n');

tic;
try
    % Get base environment config
    env = gik9dof.environmentConfig();
    
    % Save original disc count for reporting
    originalDiscCount = length(env.FloorDiscs);
    
    % Disable floor discs
    env.FloorDiscs = struct([]);
    
    fprintf('Disabled %d floor disc obstacles\n\n', originalDiscCount);
    
    % Run staged trajectory control directly with disabled floor discs
    log = gik9dof.trackReferenceTrajectory( ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', true, ...
        'EnvironmentConfig', env, ...
        'FloorDiscs', env.FloorDiscs, ...
        'DistanceMargin', env.DistanceMargin, ...
        'DistanceWeight', env.DistanceWeight, ...
        'UseStageBHybridAStar', true, ...
        'StageBMode', 'pureHyb', ...
        'StageBHybridSafetyMargin', config.SafetyMargin, ...
        'StageCUseBaseRefinement', true, ...
        'StageBLookaheadDistance', config.Lookahead, ...
        'StageBDesiredLinearVelocity', config.AccelLimit, ...
        'StageBMaxAngularVelocity', 2.0);
    
    % Save log manually
    runDir = gik9dof.internal.createResultsFolder('fresh_no_discs');
    logPath = fullfile(runDir, 'log_staged_ppForIk.mat');
    save(logPath, 'log', '-v7.3');
    logFile = char(logPath);
    
    simTime = toc;
    fprintf('✅ Simulation completed in %.1f seconds (%.1f minutes)\n\n', simTime, simTime/60);
    fprintf('   Log saved: %s\n\n', logFile);
    
catch ME
    simTime = toc;
    fprintf('❌ Simulation FAILED after %.1f seconds\n', simTime);
    fprintf('   Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    return;
end

%% Step 2: Analyze Results
fprintf('========================================\n');
fprintf('  STEP 2: Analyzing Results\n');
fprintf('========================================\n');

% Stage information
if isfield(log, 'stageLogs')
    fprintf('--- Stage Frame Counts ---\n');
    stageA_raw = size(log.stageLogs.stageA.qTraj, 2);
    stageB_raw = size(log.stageLogs.stageB.qTraj, 2);
    stageC_raw = size(log.stageLogs.stageC.qTraj, 2);
    fprintf('  Stage A: %d frames (%.1f sec @ 10Hz)\n', stageA_raw, stageA_raw/10);
    fprintf('  Stage B: %d frames (%.1f sec @ 10Hz)\n', stageB_raw, stageB_raw/10);
    fprintf('  Stage C: %d frames (%.1f sec @ 10Hz)\n', stageC_raw, stageC_raw/10);
    fprintf('  Total: %d frames (%.1f sec @ 10Hz)\n\n', stageA_raw+stageB_raw+stageC_raw, (stageA_raw+stageB_raw+stageC_raw)/10);
    
    % Stage C EE tracking
    if isfield(log.stageLogs.stageC, 'positionError')
        eeError = log.stageLogs.stageC.positionError;
        errorNorms = sqrt(sum(eeError.^2, 1));
        fprintf('--- Stage C EE Tracking ---\n');
        fprintf('  Mean error: %.4f m\n', mean(errorNorms));
        fprintf('  Max error: %.4f m\n', max(errorNorms));
        fprintf('  RMS error: %.4f m\n\n', rms(errorNorms));
    end
    
    % Stage C reference path
    if isfield(log.stageLogs.stageC, 'targetPositions')
        targetPos = log.stageLogs.stageC.targetPositions;
        fprintf('--- Stage C Reference (for Red Dot) ---\n');
        fprintf('  Source: stageC.targetPositions (JSON desired trajectory)\n');
        fprintf('  Waypoints: %d\n', size(targetPos, 2));
        fprintf('  First: [%.4f, %.4f, %.4f]\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
        fprintf('  Last:  [%.4f, %.4f, %.4f]\n\n', targetPos(1,end), targetPos(2,end), targetPos(3,end));
    end
end

%% Step 3: Generate Animation
fprintf('========================================\n');
fprintf('  STEP 3: Generating Animation\n');
fprintf('========================================\n');
fprintf('Settings:\n');
fprintf('  SampleStep: 2 (50%% frame reduction)\n');
fprintf('  FrameRate: 20 fps\n');
fprintf('  Output: %s\n\n', videoFile);

% Expected sampled frame structure
stageA_sampled = ceil(stageA_raw / 2);
stageB_sampled = ceil(stageB_raw / 2);
stageC_sampled = ceil(stageC_raw / 2);
fprintf('Expected stage structure (sampled):\n');
fprintf('  Stage A: frames 1-%d (%.1f sec @ 20fps)\n', stageA_sampled, stageA_sampled/20);
fprintf('  Stage B: frames %d-%d (%.1f sec @ 20fps)\n', stageA_sampled+1, stageA_sampled+stageB_sampled, stageB_sampled/20);
fprintf('  Stage C: frames %d-%d (%.1f sec @ 20fps)\n', stageA_sampled+stageB_sampled+1, stageA_sampled+stageB_sampled+stageC_sampled, stageC_sampled/20);
fprintf('  Total video: %d frames (%.1f sec @ 20fps)\n\n', stageA_sampled+stageB_sampled+stageC_sampled, (stageA_sampled+stageB_sampled+stageC_sampled)/20);

fprintf('Generating animation (please wait ~2 minutes)...\n');

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    animTime = toc;
    fprintf('\n✅ Animation generated in %.1f seconds (%.1f minutes)\n', animTime, animTime/60);
    
    % Check file
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('   File: %s\n', videoFile);
        fprintf('   Size: %.2f MB\n\n', fileInfo.bytes / 1024 / 1024);
    else
        fprintf('⚠ WARNING: Video file is empty or not found!\n\n');
    end
    
catch ME
    animTime = toc;
    fprintf('\n❌ Animation FAILED after %.1f seconds\n', animTime);
    fprintf('   Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    return;
end

%% Summary
fprintf('========================================\n');
fprintf('  COMPLETE!\n');
fprintf('========================================\n');
fprintf('Total time: %.1f minutes\n', (simTime + animTime)/60);
fprintf('  Simulation: %.1f min\n', simTime/60);
fprintf('  Animation: %.1f min\n\n', animTime/60);

fprintf('Output files:\n');
fprintf('  Log: %s\n', logFile);
fprintf('  Video: %s\n\n', videoFile);

fprintf('=== VERIFICATION CHECKLIST ===\n');
fprintf('Please review the animation:\n\n');
fprintf('Stage Labels:\n');
fprintf('  [ ] Stage A label for first %.1f seconds\n', stageA_sampled/20);
fprintf('  [ ] Stage B label transitions at %.1f seconds\n', stageA_sampled/20);
fprintf('  [ ] Stage C label transitions at %.1f seconds\n', (stageA_sampled+stageB_sampled)/20);
fprintf('\nRed Dot (Stage C Reference):\n');
fprintf('  [ ] Hidden during Stages A & B (first %.1f sec)\n', (stageA_sampled+stageB_sampled)/20);
fprintf('  [ ] Appears when Stage C starts (at %.1f sec)\n', (stageA_sampled+stageB_sampled)/20);
fprintf('  [ ] Starts at [%.2f, %.2f, %.2f] (NOT homebase)\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
fprintf('  [ ] Moves synchronously with robot\n');
fprintf('  [ ] Is ABOVE chassis (Z ≈ %.2f m)\n', targetPos(3,1));
fprintf('\nAll Three Fixes Applied:\n');
fprintf('  ✓ Stage boundaries adjusted for sampling\n');
fprintf('  ✓ Red dot stage-relative indexing\n');
fprintf('  ✓ Red dot from Stage C desired trajectory\n');

fprintf('\n========================================\n');
