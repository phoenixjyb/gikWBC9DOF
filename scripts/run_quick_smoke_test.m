%% Quick Smoke Test - Minimal Simulation
% This script runs a very short simulation to verify the system works
% Uses only the first 10 waypoints and simplified settings

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('========================================\n');
fprintf('  QUICK SMOKE TEST\n');
fprintf('========================================\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Configuration
jsonFile = '1_pull_world_scaled.json';
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
resultsDir = sprintf('results/%s_smoke_test', timestamp);

fprintf('--- Configuration ---\n');
fprintf('  JSON: %s (first 10 waypoints only)\n', jsonFile);
fprintf('  Results: %s\n', resultsDir);
fprintf('  Mode: Staged, ppForIk\n');
fprintf('  Rate: 10 Hz\n\n');

%% Create results directory
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Run Minimal Simulation
fprintf('========================================\n');
fprintf('  Running Simulation (VERBOSE)\n');
fprintf('========================================\n');
fprintf('This should take 1-2 minutes...\n\n');

tic;
try
    % Load only first 10 waypoints from JSON
    jsonPath = jsonFile;
    if ~exist(jsonPath, 'file')
        error('JSON file not found: %s', jsonPath);
    end
    
    % Read and truncate trajectory
    fid = fopen(jsonPath, 'r');
    raw = fread(fid, inf, 'uint8=>char')';
    fclose(fid);
    fullTraj = jsondecode(raw);
    
    % Keep only first 10 waypoints
    if length(fullTraj) > 10
        truncatedTraj = fullTraj(1:10);
        fprintf('Truncated trajectory: %d -> %d waypoints\n\n', length(fullTraj), length(truncatedTraj));
    else
        truncatedTraj = fullTraj;
    end
    
    % Save truncated trajectory temporarily
    tempJson = 'temp_smoke_test_traj.json';
    fid = fopen(tempJson, 'w');
    fprintf(fid, '%s', jsonencode(truncatedTraj));
    fclose(fid);
    
    % Get base environment config (no obstacles)
    env = gik9dof.environmentConfig();
    env.FloorDiscs = struct([]);
    
    % Run staged trajectory with VERBOSE enabled
    log = gik9dof.trackReferenceTrajectory( ...
        'JsonPath', tempJson, ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', true, ...
        'EnvironmentConfig', env, ...
        'FloorDiscs', env.FloorDiscs, ...
        'UseStageBHybridAStar', false, ...
        'StageBMode', 'pureHyb', ...
        'StageCUseBaseRefinement', false);
    
    % Clean up temp file
    if exist(tempJson, 'file')
        delete(tempJson);
    end
    
    % Save log
    logPath = fullfile(resultsDir, 'log_smoke_test.mat');
    save(logPath, 'log', '-v7.3');
    
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('✅ SMOKE TEST PASSED!\n');
    fprintf('========================================\n');
    fprintf('Simulation time: %.1f seconds\n', simTime);
    fprintf('Log saved: %s\n\n', logPath);
    
    % Print basic stats
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
    
    fprintf('System is working correctly! ✓\n');
    fprintf('You can now run the full simulation.\n\n');
    
catch ME
    simTime = toc;
    fprintf('\n========================================\n');
    fprintf('❌ SMOKE TEST FAILED\n');
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

fprintf('========================================\n');
