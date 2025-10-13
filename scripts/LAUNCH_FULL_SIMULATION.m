%% Interactive Full Simulation Launcher
% Run this script from MATLAB's command window or editor
% This avoids batch mode issues and provides better feedback

clear; clc; close all;

fprintf('========================================\n');
fprintf('  FULL SIMULATION - Interactive Mode\n');
fprintf('========================================\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Setup paths (if not already done)
if ~exist('gik9dof.trackReferenceTrajectory', 'file')
    fprintf('Setting up paths...\n');
    addpath(genpath('matlab'));
    fprintf('Paths added.\n\n');
else
    fprintf('Paths already configured.\n\n');
end

%% Configuration
jsonFile = '1_pull_world_scaled.json';
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
resultsDir = sprintf('results/%s_full_interactive', timestamp);
videoFile = sprintf('%s/animation_full.mp4', resultsDir);

config = struct();
config.SafetyMargin = 0.10;
config.Lookahead = 0.80;
config.AccelLimit = 0.80;

fprintf('--- Configuration ---\n');
fprintf('  JSON: %s (all 148 waypoints)\n', jsonFile);
fprintf('  Results: %s\n', resultsDir);
fprintf('  Mode: Staged, ppForIk\n');
fprintf('  Rate: 10 Hz\n');
fprintf('  Verbose: true\n\n');

fprintf('--- Parameters ---\n');
fprintf('  Safety Margin: %.2f\n', config.SafetyMargin);
fprintf('  Pure Pursuit Lookahead: %.2f\n', config.Lookahead);
fprintf('  Desired Velocity: %.2f\n\n', config.AccelLimit);

%% Create results directory
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Step 1: Run Simulation
fprintf('========================================\n');
fprintf('  STEP 1: Running Full Simulation\n');
fprintf('========================================\n');
fprintf('This will take 1-2 hours...\n');
fprintf('You can monitor progress via the [GIK] step messages.\n');
fprintf('Press Ctrl+C to abort if needed.\n\n');

tic;
try
    % Get base environment config
    env = gik9dof.environmentConfig();
    env.FloorDiscs = struct([]);  % No obstacles for this test
    
    fprintf('Starting simulation...\n\n');
    
    % Run staged trajectory
    log = gik9dof.trackReferenceTrajectory( ...
        'JsonPath', jsonFile, ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', true, ...
        'EnvironmentConfig', env, ...
        'FloorDiscs', env.FloorDiscs, ...
        'UseStageBHybridAStar', true, ...
        'StageBMode', 'pureHyb', ...
        'StageCUseBaseRefinement', true, ...
        'StageBHybridSafetyMargin', config.SafetyMargin, ...
        'StageBLookaheadDistance', config.Lookahead, ...
        'StageBDesiredLinearVelocity', config.AccelLimit, ...
        'StageBMaxAngularVelocity', 2.0);
    
    % Save log
    logPath = fullfile(resultsDir, 'log_full_staged.mat');
    save(logPath, 'log', '-v7.3');
    
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('✅ SIMULATION COMPLETED!\n');
    fprintf('========================================\n');
    fprintf('Time: %.1f seconds (%.1f minutes)\n', simTime, simTime/60);
    fprintf('Log: %s\n\n', logPath);
    
    % Print stats
    if isfield(log, 'stageLogs')
        fprintf('--- Stage Information ---\n');
        fprintf('  Stage A: %d frames (%.1f sec)\n', ...
            size(log.stageLogs.stageA.qTraj, 2), ...
            size(log.stageLogs.stageA.qTraj, 2)/10);
        fprintf('  Stage B: %d frames (%.1f sec)\n', ...
            size(log.stageLogs.stageB.qTraj, 2), ...
            size(log.stageLogs.stageB.qTraj, 2)/10);
        fprintf('  Stage C: %d frames (%.1f sec)\n', ...
            size(log.stageLogs.stageC.qTraj, 2), ...
            size(log.stageLogs.stageC.qTraj, 2)/10);
        fprintf('\n');
    end
    
catch ME
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('❌ SIMULATION FAILED\n');
    fprintf('========================================\n');
    fprintf('Time elapsed: %.1f seconds (%.1f minutes)\n', simTime, simTime/60);
    fprintf('Error: %s\n\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %d. %s (line %d)\n', i, ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\n');
    error('Simulation failed. See error above.');
end

%% Step 2: Generate Animation
fprintf('========================================\n');
fprintf('  STEP 2: Generating Animation\n');
fprintf('========================================\n');
fprintf('Output: %s\n', videoFile);
fprintf('This will take ~5-10 minutes...\n\n');

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    animTime = toc;
    fprintf('\n✅ ANIMATION COMPLETED!\n');
    fprintf('Time: %.1f seconds (%.1f minutes)\n', animTime, animTime/60);
    
    % Check file
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('File: %s\n', videoFile);
        fprintf('Size: %.2f MB\n\n', fileInfo.bytes / 1024 / 1024);
    end
    
catch ME
    animTime = toc;
    fprintf('\n❌ ANIMATION FAILED\n');
    fprintf('Time elapsed: %.1f seconds\n', animTime);
    fprintf('Error: %s\n', ME.message);
    warning('Animation failed but simulation log is saved.');
end

%% Summary
fprintf('========================================\n');
fprintf('  ALL DONE!\n');
fprintf('========================================\n');
fprintf('Total time: %.1f minutes\n', (simTime + animTime)/60);
fprintf('\nOutput files:\n');
fprintf('  Log: %s\n', logPath);
if exist(videoFile, 'file')
    fprintf('  Video: %s\n', videoFile);
end
fprintf('\n');
