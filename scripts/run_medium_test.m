%% Medium-Length Test - 30 Waypoints
% This script runs a simulation with 30 waypoints to identify issues
% before running the full 148-waypoint simulation

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('========================================\n');
fprintf('  MEDIUM TEST (30 waypoints)\n');
fprintf('========================================\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Configuration
jsonFile = '1_pull_world_scaled.json';
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
resultsDir = sprintf('results/%s_medium_test', timestamp);

fprintf('--- Configuration ---\n');
fprintf('  JSON: %s (first 30 waypoints)\n', jsonFile);
fprintf('  Results: %s\n', resultsDir);
fprintf('  Mode: Staged, ppForIk\n');
fprintf('  Rate: 10 Hz\n');
fprintf('  Verbose: true\n\n');

%% Create results directory
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Load and truncate trajectory
jsonPath = jsonFile;
if ~exist(jsonPath, 'file')
    error('JSON file not found: %s', jsonPath);
end

% Read full trajectory
fid = fopen(jsonPath, 'r');
raw = fread(fid, inf, 'uint8=>char')';
fclose(fid);
fullTraj = jsondecode(raw);

% Keep first 30 waypoints
numWaypoints = min(30, length(fullTraj));
truncatedTraj = fullTraj(1:numWaypoints);
fprintf('Trajectory: %d -> %d waypoints\n\n', length(fullTraj), numWaypoints);

% Save truncated trajectory
tempJson = 'temp_medium_test_traj.json';
fid = fopen(tempJson, 'w');
fprintf(fid, '%s', jsonencode(truncatedTraj));
fclose(fid);

%% Run Simulation
fprintf('========================================\n');
fprintf('  Running Simulation\n');
fprintf('========================================\n');
fprintf('Expected duration: 5-10 minutes\n\n');

tic;
try
    % Get base environment config (no obstacles)
    env = gik9dof.environmentConfig();
    env.FloorDiscs = struct([]);
    
    fprintf('Disabled floor disc obstacles\n\n');
    
    % Run staged trajectory with VERBOSE enabled
    log = gik9dof.trackReferenceTrajectory( ...
        'JsonPath', tempJson, ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', true, ...
        'EnvironmentConfig', env, ...
        'FloorDiscs', env.FloorDiscs, ...
        'UseStageBHybridAStar', true, ...
        'StageBMode', 'pureHyb', ...
        'StageCUseBaseRefinement', true, ...
        'StageBHybridSafetyMargin', 0.10, ...
        'StageBLookaheadDistance', 0.80, ...
        'StageBDesiredLinearVelocity', 0.80, ...
        'StageBMaxAngularVelocity', 2.0);
    
    % Clean up temp file
    if exist(tempJson, 'file')
        delete(tempJson);
    end
    
    % Save log
    logPath = fullfile(resultsDir, 'log_medium_test.mat');
    save(logPath, 'log', '-v7.3');
    
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('✅ MEDIUM TEST PASSED!\n');
    fprintf('========================================\n');
    fprintf('Simulation time: %.1f seconds (%.1f minutes)\n', simTime, simTime/60);
    fprintf('Log saved: %s\n\n', logPath);
    
    % Print stats
    if isfield(log, 'stageLogs')
        fprintf('--- Stage Information ---\n');
        if isfield(log.stageLogs, 'stageA')
            fprintf('  Stage A: %d frames\n', size(log.stageLogs.stageA.qTraj, 2));
        end
        if isfield(log.stageLogs, 'stageB')
            fprintf('  Stage B: %d frames\n', size(log.stageLogs.stageB.qTraj, 2));
        end
        if isfield(log.stageLogs, 'stageC')
            fprintf('  Stage C: %d frames\n', size(log.stageLogs.stageC.qTraj, 2));
        end
        fprintf('\n');
    end
    
    fprintf('System working with 30 waypoints! ✓\n');
    fprintf('Ready to try full 148-waypoint simulation.\n\n');
    
catch ME
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('❌ MEDIUM TEST FAILED\n');
    fprintf('========================================\n');
    fprintf('Time elapsed: %.1f seconds\n', simTime);
    fprintf('Error: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  at %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\n');
    
    % Clean up temp file
    if exist('tempJson', 'var') && exist(tempJson, 'file')
        delete(tempJson);
    end
    
    rethrow(ME);
end

%% Generate Animation
fprintf('========================================\n');
fprintf('  Generating Animation\n');
fprintf('========================================\n');

videoFile = fullfile(resultsDir, 'animation_medium_test.mp4');
fprintf('Output: %s\n\n', videoFile);

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    animTime = toc;
    fprintf('\n✅ Animation generated in %.1f seconds\n', animTime);
    
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
end

fprintf('========================================\n');
fprintf('  COMPLETE!\n');
fprintf('========================================\n');
