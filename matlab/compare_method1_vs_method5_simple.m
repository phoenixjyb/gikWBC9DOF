%% Simple Comparison Test: Method 1 (ppForIk) vs Method 5 (pureMPC)
% Runs both methods on the same trajectory and compares results

clear; clc;

fprintf('=== Method Comparison: Method 1 vs Method 5 ===\n\n');

% Setup environment
fprintf('Setting up environment...\n');
addpath(genpath('matlab'));
env = gik9dof.environmentConfig();
env.FloorDiscs = struct([]);  % Disable obstacles for cleaner test

%% Run Method 1 (ppForIk)
fprintf('\n=== Running Method 1 (ppForIk) ===\n');
tic;
log1 = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'RateHz', 10, ...
    'Verbose', true, ...
    'EnvironmentConfig', env, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'ExecutionMode', 'ppForIk');
time1 = toc;
logC1 = log1.stageLogs.stageC;
fprintf('✅ Method 1 Complete: %.1f s\n', time1);

%% Run Method 5 (pureMPC)
fprintf('\n=== Running Method 5 (pureMPC) ===\n');
tic;
log5 = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'RateHz', 10, ...
    'Verbose', true, ...
    'EnvironmentConfig', env, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'ExecutionMode', 'pureMPC');
time5 = toc;
logC5 = log5.stageLogs.stageC;
fprintf('✅ Method 5 Complete: %.1f s\n', time5);

%% Compare Results
fprintf('\n=== Comparison Results ===\n\n');

% EE tracking errors
if isfield(logC1, 'positionError')
    err1 = sqrt(sum(logC1.positionError.^2, 1));
    fprintf('Method 1: Mean EE error = %.2f mm, Max = %.2f mm\n', ...
        mean(err1)*1000, max(err1)*1000);
end

if isfield(logC5, 'stageCDiagnostics')
    d = logC5.stageCDiagnostics;
    fprintf('Method 5: Mean EE error = %.2f mm, Max = %.2f mm\n', ...
        d.meanEEPosError*1000, d.maxEEPosError*1000);
    fprintf('          Mean ori error = %.3f\n', d.meanEEOriError);
    fprintf('          MPC convergence = %.1f%%\n', d.mpcConvergenceRate*100);
    fprintf('          Control rate = %.1f Hz\n', d.controlFrequency);
    fprintf('          Solve time = %.1f ms avg\n', d.meanSolveTime*1000);
elseif isfield(logC5, 'positionError')
    err5 = sqrt(sum(logC5.positionError.^2, 1));
    fprintf('Method 5: Mean EE error = %.2f mm, Max = %.2f mm\n', ...
        mean(err5)*1000, max(err5)*1000);
end

fprintf('\nTotal time: Method 1 = %.1f s, Method 5 = %.1f s\n', time1, time5);

%% Simple Visualization
fprintf('\nGenerating comparison plots...\n');

figure('Name', 'Method 1 vs Method 5', 'Position', [50 50 1200 800]);

% Base trajectory
subplot(2, 2, 1);
plot(logC1.qTraj(1, :), logC1.qTraj(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Method 1');
hold on;
plot(logC5.qTraj(1, :), logC5.qTraj(2, :), 'r-', 'LineWidth', 2, 'DisplayName', 'Method 5');
xlabel('X (m)'); ylabel('Y (m)');
title('Base Trajectory');
legend; grid on; axis equal;

% Position error
subplot(2, 2, 2);
if isfield(logC1, 'positionError')
    plot(logC1.time, sqrt(sum(logC1.positionError.^2,1))*1000, 'b-', 'LineWidth', 1.5);
    hold on;
end
if isfield(logC5, 'positionError')
    plot(logC5.time, sqrt(sum(logC5.positionError.^2,1))*1000, 'r-', 'LineWidth', 1.5);
end
xlabel('Time (s)'); ylabel('EE Error (mm)');
title('Tracking Error');
legend('Method 1', 'Method 5'); grid on;

% Base velocity
subplot(2, 2, 3);
v1 = sqrt(diff(logC1.qTraj(1,:)).^2 + diff(logC1.qTraj(2,:)).^2) ./ diff(logC1.time);
v5 = sqrt(diff(logC5.qTraj(1,:)).^2 + diff(logC5.qTraj(2,:)).^2) ./ diff(logC5.time);
plot(logC1.time(1:end-1), v1, 'b-', 'LineWidth', 1.5);
hold on;
plot(logC5.time(1:end-1), v5, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Base Linear Velocity');
legend('Method 1', 'Method 5'); grid on;

% Arm joints
subplot(2, 2, 4);
plot(logC1.time, logC1.qTraj(4:9, :)', 'b-', 'LineWidth', 1);
hold on;
plot(logC5.time, logC5.qTraj(4:9, :)', 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Joint Angle (rad)');
title('Arm Joint Trajectories');
grid on;

fprintf('\n✅ Comparison complete!\n');
