%% Compare Method 5 vs Saved Method 1 Log
% This script:
% 1. Loads pre-saved Method 1 (ppForIk) log from logToCompare/
% 2. Runs Method 5 (pureMPC) on the same trajectory
% 3. Compares tracking accuracy, performance, and trajectories

clear; clc;

fprintf('=== Method 5 vs Method 1 Comparison ===\n');
fprintf('Loading saved Method 1 log and running Method 5 fresh\n\n');

%% Load Method 1 (ppForIk) from saved log
fprintf('Step 1: Loading saved Method 1 log...\n');
% Get the script's directory, then go up one level to project root
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
log1Path = fullfile(projectRoot, 'logToCompare', 'log_method1_ppForIk.mat');

if ~exist(log1Path, 'file')
    error('Method 1 log not found at: %s', log1Path);
end

load(log1Path, 'log1');

fprintf('  ✅ Method 1 log loaded\n');
if isfield(log1, 'stageLogs') && isfield(log1.stageLogs, 'stageC')
    logC1 = log1.stageLogs.stageC;
    fprintf('  Stage C: %d waypoints\n', size(logC1.qTraj, 2));
else
    error('Invalid log structure - missing stageLogs.stageC');
end

%% Run Method 5 (pureMPC) fresh
fprintf('\nStep 2: Running Method 5 (pureMPC)...\n');
addpath(genpath('matlab'));

% Create robot and load trajectory
robot = gik9dof.createRobotModel();
jsonPath = fullfile(projectRoot, '1_pull_world_scaled.json');

% Read JSON trajectory using the internal function
jsonText = fileread(jsonPath);
trajData = jsondecode(jsonText);
numWaypoints = numel(trajData.poses);
poses = repmat(eye(4), 1, 1, numWaypoints);

for k = 1:numWaypoints
    entry = trajData.poses(k);
    position = reshape(entry.position, [], 1);
    quatXYZW = reshape(entry.orientation, 1, []);
    quatWXYZ = [quatXYZW(4), quatXYZW(1:3)];
    quatWXYZ = quatWXYZ ./ norm(quatWXYZ);
    T = quat2tform(quatWXYZ);
    T(1:3,4) = position;
    poses(:,:,k) = T;
end

traj = struct('Poses', poses);

% Load configuration and get initial state from Method 1
pipelineConfig = gik9dof.loadPipelineProfile('pureMPC');
q0 = logC1.qTraj(:, 1);

% Get Stage B result from Method 1 log to skip re-running Stage B
if isfield(log1, 'stageLogs') && isfield(log1.stageLogs, 'stageB')
    stageBResult = log1.stageLogs.stageB;
else
    error('Method 1 log missing Stage B data - cannot skip Stage B');
end

fprintf('  Using Method 1 Stage B result to skip planning\n');
fprintf('  Running Stage C with pureMPC...\n');

tic;
% Call runStagedTrajectory with Stage B already completed
pipeline5 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'pureMPC', ...
    'PipelineConfig', pipelineConfig, ...
    'InitialConfiguration', q0, ...
    'StageBResult', stageBResult, ...
    'Verbose', true);
time5 = toc;

logC5 = pipeline5.logC;
fprintf('\n  ✅ Method 5 complete: %.1f s\n', time5);

%% Extract metrics
fprintf('\n=== Comparison Results ===\n\n');

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

fprintf('\n2. Performance Metrics:\n');
if exist('convergence5', 'var')
    fprintf('   MPC Convergence Rate:  %.1f%%\n', convergence5);
    fprintf('   Control Frequency:     %.1f Hz\n', controlFreq5);
    fprintf('   Mean Solve Time:       %.1f ms\n', meanSolve5);
    
    if meanSolve5 < 100
        fprintf('   ✅ Real-time capable (< 100ms per step)\n');
    end
end

fprintf('\n3. Trajectory Statistics:\n');
fprintf('   Method 1 waypoints:    %d\n', size(logC1.qTraj, 2));
fprintf('   Method 5 waypoints:    %d\n', size(logC5.qTraj, 2));

%% Visualization
fprintf('\n4. Generating comparison plots...\n');

figure('Name', 'Method 5 vs Method 1 (Saved)', 'Position', [50 50 1400 900]);

% Subplot 1: Base trajectories
subplot(2, 3, 1);
plot(logC1.qTraj(1, :), logC1.qTraj(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Method 1');
hold on;
plot(logC5.qTraj(1, :), logC5.qTraj(2, :), 'r-', 'LineWidth', 2, 'DisplayName', 'Method 5');
xlabel('X (m)'); ylabel('Y (m)');
title('Base Trajectory (Top View)');
legend('Location', 'best');
grid on; axis equal;

% Subplot 2: Position error over time
subplot(2, 3, 2);
if isfield(logC1, 'positionError')
    err1 = sqrt(sum(logC1.positionError.^2, 1)) * 1000;
    plot(logC1.time, err1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
    hold on;
end
if isfield(logC5, 'positionError')
    err5 = sqrt(sum(logC5.positionError.^2, 1)) * 1000;
    plot(logC5.time, err5, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
end
xlabel('Time (s)'); ylabel('Position Error (mm)');
title('EE Tracking Error');
legend('Location', 'best');
grid on;

% Subplot 3: Error histogram
subplot(2, 3, 3);
if isfield(logC1, 'positionError')
    err1 = sqrt(sum(logC1.positionError.^2, 1)) * 1000;
    histogram(err1, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Method 1');
    hold on;
end
if isfield(logC5, 'positionError')
    err5 = sqrt(sum(logC5.positionError.^2, 1)) * 1000;
    histogram(err5, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Method 5');
end
xlabel('Position Error (mm)'); ylabel('Frequency');
title('Error Distribution');
legend('Location', 'best');
grid on;

% Subplot 4: Base velocities
subplot(2, 3, 4);
dt1 = diff(logC1.time);
v1 = sqrt(diff(logC1.qTraj(1, :)).^2 + diff(logC1.qTraj(2, :)).^2) ./ dt1;
dt5 = diff(logC5.time);
v5 = sqrt(diff(logC5.qTraj(1, :)).^2 + diff(logC5.qTraj(2, :)).^2) ./ dt5;
plot(logC1.time(1:end-1), v1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.time(1:end-1), v5, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Base Linear Velocity');
legend('Location', 'best');
grid on;

% Subplot 5: Arm joint trajectories
subplot(2, 3, 5);
plot(logC1.time, logC1.qTraj(4:9, :)', 'b-', 'LineWidth', 1);
hold on;
plot(logC5.time, logC5.qTraj(4:9, :)', 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Joint Angle (rad)');
title('Arm Joint Trajectories');
grid on;

% Subplot 6: Orientation error (Method 5 only)
subplot(2, 3, 6);
if isfield(logC5, 'orientationErrorAngle')
    plot(logC5.time, rad2deg(logC5.orientationErrorAngle), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Orientation Error (deg)');
    title('EE Orientation Error (Method 5)');
    grid on;
elseif exist('meanOriErr5', 'var')
    text(0.5, 0.5, sprintf('Mean Ori Error: %.3f', meanOriErr5), ...
        'HorizontalAlignment', 'center', 'Units', 'normalized');
    title('Orientation Error (Method 5)');
else
    text(0.5, 0.5, 'N/A', 'HorizontalAlignment', 'center', 'Units', 'normalized');
    title('Orientation Error');
end

%% Final Summary
fprintf('\n=== Final Summary ===\n');
fprintf('Method 1 (ppForIk):  Feed-forward, three-pass IK\n');
fprintf('Method 5 (pureMPC):  Whole-body NMPC optimization\n\n');

if exist('convergence5', 'var')
    if convergence5 > 90
        fprintf('✅ Method 5 MPC convergence: %.1f%% (excellent)\n', convergence5);
    else
        fprintf('⚠️  Method 5 MPC convergence: %.1f%% (needs tuning)\n', convergence5);
    end
end

if ~isnan(meanErr5) && ~isnan(meanErr1)
    if meanErr5 < meanErr1 * 1.1
        fprintf('✅ Method 5 tracking accuracy: comparable or better\n');
    else
        fprintf('⚠️  Method 5 tracking accuracy: needs improvement\n');
    end
end

if exist('meanSolve5', 'var') && meanSolve5 < 100
    fprintf('✅ Method 5 computational performance: real-time capable\n');
end

fprintf('\n✅ Comparison complete!\n');
