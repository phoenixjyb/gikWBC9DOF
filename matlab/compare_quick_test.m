%% Quick Comparison: Method 1 vs Method 5 (First 20 waypoints only)
% Fast test using a subset of the trajectory

clear; clc;

fprintf('=== Quick Comparison: Method 1 vs Method 5 ===\n');
fprintf('Testing on first 20 waypoints only for speed\n\n');

% Setup
addpath(genpath('matlab'));
env = gik9dof.environmentConfig();
env.FloorDiscs = struct([]);

% Load full trajectory and subsample
fprintf('Loading trajectory...\n');
jsonPath = fullfile(fileparts(pwd), '1_pull_world_scaled.json');
jsonText = fileread(jsonPath);
trajData = jsondecode(jsonText);
numFull = length(trajData.poses);
numTest = min(20, numFull);
trajData.poses = trajData.poses(1:numTest);
fprintf('  Using %d/%d waypoints\n\n', numTest, numFull);

% Save temporary test trajectory
testJsonFile = 'test_traj_short.json';
fid = fopen(testJsonFile, 'w');
fprintf(fid, '%s', jsonencode(trajData));
fclose(fid);

%% Test Method 1
fprintf('=== Method 1 (ppForIk) ===\n');
tic;
try
    log1 = gik9dof.trackReferenceTrajectory(testJsonFile, ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', false, ...
        'EnvironmentConfig', env, ...
        'UseStageBHybridAStar', true, ...
        'StageBMode', 'pureHyb', ...
        'ExecutionMode', 'ppForIk');
    time1 = toc;
    logC1 = log1.stageLogs.stageC;
    fprintf('✅ Complete: %.1f s\n', time1);
    if isfield(logC1, 'positionError')
        err1 = sqrt(sum(logC1.positionError.^2,1))*1000;
        fprintf('   Mean error: %.2f mm\n', mean(err1));
    end
catch ME
    fprintf('❌ Failed: %s\n', ME.message);
    time1 = NaN;
end

%% Test Method 5
fprintf('\n=== Method 5 (pureMPC) ===\n');
tic;
try
    log5 = gik9dof.trackReferenceTrajectory(testJsonFile, ...
        'Mode', 'staged', ...
        'RateHz', 10, ...
        'Verbose', false, ...
        'EnvironmentConfig', env, ...
        'UseStageBHybridAStar', true, ...
        'StageBMode', 'pureHyb', ...
        'ExecutionMode', 'pureMPC');
    time5 = toc;
    logC5 = log5.stageLogs.stageC;
    fprintf('✅ Complete: %.1f s\n', time5);
    if isfield(logC5, 'stageCDiagnostics')
        d = logC5.stageCDiagnostics;
        fprintf('   Mean error: %.2f mm\n', d.meanEEPosError*1000);
        fprintf('   MPC convergence: %.1f%%\n', d.mpcConvergenceRate*100);
        fprintf('   Solve time: %.1f ms\n', d.meanSolveTime*1000);
    end
catch ME
    fprintf('❌ Failed: %s\n', ME.message);
    time5 = NaN;
end

%% Cleanup
delete(testJsonFile);

%% Summary
fprintf('\n=== Summary ===\n');
fprintf('Method 1: %.1f s (three-pass: 2×GIK + chassis sim)\n', time1);
fprintf('Method 5: %.1f s (single-pass: whole-body MPC)\n', time5);
if ~isnan(time1) && ~isnan(time5)
    fprintf('\n');
    if time5 < time1
        fprintf('✅ Method 5 is %.1fx faster\n', time1/time5);
    else
        fprintf('⚠️  Method 1 is %.1fx faster\n', time5/time1);
    end
end

fprintf('\n✅ Quick test complete!\n');
