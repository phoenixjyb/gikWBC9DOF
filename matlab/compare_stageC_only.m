%% Compare Stage C Only: Method 1 (ppForIk) vs Method 5 (pureMPC)
% This script:
% 1. Loads pre-saved Method 1 log (includes Stage B results)
% 2. Runs ONLY Stage C with pureMPC using Method 1's Stage B path
% 3. Compares Stage C execution

clear; clc;

fprintf('=== Stage C Comparison: Method 1 vs Method 5 ===\n');
fprintf('Comparing Stage C execution only (using saved Stage B path)\n\n');

%% Load Method 1 log
fprintf('Step 1: Loading Method 1 log...\n');
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
log1Path = fullfile(projectRoot, 'logToCompare', 'log_method1_ppForIk.mat');

if ~exist(log1Path, 'file')
    error('Method 1 log not found at: %s', log1Path);
end

load(log1Path, 'log1');
logC1 = log1.stageLogs.stageC;
logB1 = log1.stageLogs.stageB;

fprintf('  ✅ Method 1 loaded\n');
fprintf('  Stage C: %d waypoints\n\n', size(logC1.qTraj, 2));

%% Run Stage C with pureMPC
fprintf('Step 2: Running Stage C with pureMPC...\n');
addpath(genpath('matlab'));

% Load pureMPC configuration
pipelineConfig = gik9dof.loadPipelineProfile('pureMPC');

% Create robot
robot = gik9dof.createRobotModel();

% Load full trajectory for Stage C waypoints
jsonPath = fullfile(projectRoot, '1_pull_world_scaled.json');
trajData = gik9dof.loadJsonTrajectory(jsonPath);

% Get initial configuration from Method 1's Stage B end
q0 = logB1.qTraj(:, end);

% Extract base index
jointNames = robot.homeConfiguration;
baseIdx = [1, 2, 3];  % joint_x, joint_y, joint_theta
armIdx = 4:9;

% Get chassis parameters - use the first available profile
chassisParams = gik9dof.control.loadChassisProfile('wide_track');

% Call Stage C pureMPC directly
fprintf('  Calling runStageCPureMPC...\n');
tic;
try
    % Extract NMPC parameters from pipeline config
    nmpcParams = pipelineConfig.stage_c.nmpc;
    
    logC5 = gik9dof.runStageCPureMPC(robot, trajData, q0, ...
        'NMPCParams', nmpcParams, ...
        'BaseIndices', baseIdx, ...
        'ArmIndices', armIdx, ...
        'VerboseLevel', 2);
    time5 = toc;
    
    fprintf('\n  ✅ Method 5 complete: %.1f s\n', time5);
catch ME
    fprintf('\n  ❌ Error: %s\n', ME.message);
    rethrow(ME);
end

%% Extract metrics
fprintf('\n=== Comparison Results (Stage C Only) ===\n\n');

% Method 1 metrics
if isfield(logC1, 'positionError')
    err1 = sqrt(sum(logC1.positionError.^2, 1));
    meanErr1 = mean(err1) * 1000;
    maxErr1 = max(err1) * 1000;
    rmsErr1 = rms(err1) * 1000;
else
    meanErr1 = NaN;
    maxErr1 = NaN;
    rmsErr1 = NaN;
end

% Method 5 metrics
if isfield(logC5, 'stageCDiagnostics')
    d5 = logC5.stageCDiagnostics;
    meanErr5 = d5.meanEEPosError * 1000;
    maxErr5 = d5.maxEEPosError * 1000;
    rmsErr5 = d5.rmsEEPosError * 1000;
    meanOriErr5 = d5.meanEEOriError;
    convergence5 = d5.mpcConvergenceRate * 100;
    controlFreq5 = d5.controlFrequency;
    meanSolve5 = d5.meanSolveTime * 1000;
elseif isfield(logC5, 'positionError')
    err5 = sqrt(sum(logC5.positionError.^2, 1));
    meanErr5 = mean(err5) * 1000;
    maxErr5 = max(err5) * 1000;
    rmsErr5 = rms(err5) * 1000;
else
    meanErr5 = NaN;
    maxErr5 = NaN;
    rmsErr5 = NaN;
end

%% Print comparison
fprintf('1. End-Effector Tracking Accuracy:\n');
fprintf('   %-25s | %12s | %12s\n', 'Metric', 'Method 1', 'Method 5');
fprintf('   %s\n', repmat('-', 53, 1));
fprintf('   %-25s | %9.2f mm | %9.2f mm\n', 'Mean Position Error', meanErr1, meanErr5);
fprintf('   %-25s | %9.2f mm | %9.2f mm\n', 'Max Position Error', maxErr1, maxErr5);
fprintf('   %-25s | %9.2f mm | %9.2f mm\n', 'RMS Position Error', rmsErr1, rmsErr5);

if exist('meanOriErr5', 'var')
    fprintf('   %-25s | %12s | %12.3f\n', 'Mean Orientation Error', 'N/A', meanOriErr5);
end

if ~isnan(meanErr5) && ~isnan(meanErr1)
    improvement = (meanErr1 - meanErr5) / meanErr1 * 100;
    if improvement > 0
        fprintf('\n   ✅ Method 5 is %.1f%% more accurate\n', improvement);
    else
        fprintf('\n   ⚠️  Method 1 is %.1f%% more accurate\n', -improvement);
    end
end

fprintf('\n2. Performance Metrics (Stage C only):\n');
if exist('convergence5', 'var')
    fprintf('   MPC Convergence Rate:  %.1f%%\n', convergence5);
    fprintf('   Control Frequency:     %.1f Hz\n', controlFreq5);
    fprintf('   Mean Solve Time:       %.1f ms\n', meanSolve5);
    
    if meanSolve5 < 100
        fprintf('   ✅ Real-time capable (< 100ms per step)\n');
    end
end

fprintf('\n✅ Stage C comparison complete!\n');
