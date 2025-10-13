%% Comparison Test: Method 1 (ppForIk) vs Method 5 (pureMPC)
% This script runs both methods on the same trajectory and compares:
%   - End-effector tracking accuracy
%   - Computational performance  
%   - Success rates
%
% Method 1: PP-For-IK (Production default, three-pass feed-forward)
% Method 5: Pure MPC (Whole-body NMPC with 9 states, 8 inputs)

clear; clc;

%% Setup
fprintf('=== Method Comparison: Method 1 vs Method 5 ===\n\n');

% Setup environment
fprintf('Step 1: Setting up environment...\n');
addpath(genpath('matlab'));
env = gik9dof.environmentConfig();

% Disable floor discs for cleaner comparison
env.FloorDiscs = struct([]);
fprintf('  Environment configured (obstacles disabled)\n');

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

% Extract Stage C log
logC1 = log1.stageLogs.stageC;
fprintf('\nMethod 1 Complete:\n');
fprintf('  Total time: %.1f s\n', time1);
if isfield(logC1, 'positionError')
    eeErrors1 = sqrt(sum(logC1.positionError.^2, 1));
    fprintf('  Mean EE error: %.2f mm\n', mean(eeErrors1) * 1000);
    fprintf('  Max EE error: %.2f mm\n', max(eeErrors1) * 1000);
end

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

% Extract Stage C log
logC5 = log5.stageLogs.stageC;
fprintf('\nMethod 5 Complete:\n');
fprintf('  Total time: %.1f s\n', time5);
if isfield(logC5, 'stageCDiagnostics')
    fprintf('  Mean EE pos error: %.2f mm\n', logC5.stageCDiagnostics.meanEEPosError * 1000);
    fprintf('  Max EE pos error: %.2f mm\n', logC5.stageCDiagnostics.maxEEPosError * 1000);
    fprintf('  Mean EE ori error: %.3f\n', logC5.stageCDiagnostics.meanEEOriError);
    fprintf('  MPC convergence: %.1f%%\n', logC5.stageCDiagnostics.mpcConvergenceRate * 100);
elseif isfield(logC5, 'positionError')
    eeErrors5 = sqrt(sum(logC5.positionError.^2, 1));
    fprintf('  Mean EE error: %.2f mm\n', mean(eeErrors5) * 1000);
    fprintf('  Max EE error: %.2f mm\n', max(eeErrors5) * 1000);
end

%% Comparison Analysis
fprintf('\n=== Comparison Results ===\n\n');

% Extract error metrics
if isfield(logC1, 'positionError')
    eeErrors1 = sqrt(sum(logC1.positionError.^2, 1));
    meanErr1 = mean(eeErrors1) * 1000;
    maxErr1 = max(eeErrors1) * 1000;
else
    meanErr1 = NaN;
    maxErr1 = NaN;
end

if isfield(logC5, 'stageCDiagnostics')
    meanErr5 = logC5.stageCDiagnostics.meanEEPosError * 1000;
    maxErr5 = logC5.stageCDiagnostics.maxEEPosError * 1000;
elseif isfield(logC5, 'positionError')
    eeErrors5 = sqrt(sum(logC5.positionError.^2, 1));
    meanErr5 = mean(eeErrors5) * 1000;
    maxErr5 = max(eeErrors5) * 1000;
else
    meanErr5 = NaN;
    maxErr5 = NaN;
end

% 1. Tracking Accuracy
fprintf('1. End-Effector Tracking Accuracy:\n');
fprintf('   Method 1 (ppForIk):\n');
fprintf('     Mean error: %.2f mm\n', meanErr1);
fprintf('     Max error:  %.2f mm\n', maxErr1);
fprintf('   Method 5 (pureMPC):\n');
fprintf('     Mean error: %.2f mm\n', meanErr5);
fprintf('     Max error:  %.2f mm\n', maxErr5);
fprintf('   Comparison:\n');
if ~isnan(meanErr5) && ~isnan(meanErr1)
    if meanErr5 < meanErr1
        fprintf('     ✅ Method 5 is %.1f%% more accurate (mean)\n', ...
            100 * (meanErr1 - meanErr5) / meanErr1);
    else
        fprintf('     ⚠️  Method 1 is %.1f%% more accurate (mean)\n', ...
            100 * (meanErr5 - meanErr1) / meanErr5);
    end
end

% 2. Computational Performance
fprintf('\n2. Computational Performance:\n');
fprintf('   Method 1: %.1f s total\n', time1);
fprintf('   Method 5: %.1f s total\n', time5);
if isfield(logC5, 'stageCDiagnostics')
    fprintf('   Method 5 control rate: %.1f Hz\n', logC5.stageCDiagnostics.controlFrequency);
    fprintf('   Method 5 solve time: %.1f ms average\n', logC5.stageCDiagnostics.meanSolveTime * 1000);
end
if time5 < time1
    fprintf('   ✅ Method 5 is %.1f%% faster\n', 100 * (time1 - time5) / time1);
else
    fprintf('   ⚠️  Method 1 is %.1f%% faster\n', 100 * (time5 - time1) / time5);
end

% 3. Trajectory Smoothness
fprintf('\n3. Trajectory Smoothness:\n');
baseVel1 = diff(logC1.qTraj(1:3, :), 1, 2) ./ diff(logC1.time);
baseVel5 = diff(logC5.qTraj(1:3, :), 1, 2) ./ diff(logC5.time);

smoothness1 = mean(vecnorm(diff(baseVel1, 1, 2), 2, 1));  % Mean acceleration
smoothness5 = mean(vecnorm(diff(baseVel5, 1, 2), 2, 1));

fprintf('   Method 1 base acceleration: %.3f m/s²\n', smoothness1);
fprintf('   Method 5 base acceleration: %.3f m/s²\n', smoothness5);
if smoothness5 < smoothness1
    fprintf('   ✅ Method 5 is %.1f%% smoother\n', ...
        100 * (smoothness1 - smoothness5) / smoothness1);
else
    fprintf('   ⚠️  Method 1 is %.1f%% smoother\n', ...
        100 * (smoothness5 - smoothness1) / smoothness5);
end

% 4. Success Rate
fprintf('\n4. Success Rate:\n');
success1 = sum(logC1.successMask) / length(logC1.successMask);
success5 = logC5.stageCDiagnostics.mpcConvergenceRate;
fprintf('   Method 1: %.1f%% (IK convergence)\n', success1 * 100);
fprintf('   Method 5: %.1f%% (MPC convergence)\n', success5 * 100);

%% Visualization
fprintf('\n5. Generating comparison plots...\n');

figure('Name', 'Method 1 vs Method 5 Comparison', 'Position', [50 50 1400 900]);

% Subplot 1: EE position error
subplot(3, 3, 1);
plot(logC1.time, logC1.positionErrorNorm * 1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.time, logC5.positionErrorNorm * 1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
xlabel('Time (s)');
ylabel('Position Error (mm)');
title('EE Position Tracking Error');
legend('Location', 'best');
grid on;

% Subplot 2: EE trajectory (XY)
subplot(3, 3, 2);
plot(logC1.eePositions(1, :), logC1.eePositions(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.eePositions(1, :), logC5.eePositions(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
plot(eePositions(1, :), eePositions(2, :), 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
xlabel('X (m)');
ylabel('Y (m)');
title('EE Path (Top View)');
legend('Location', 'best');
grid on;
axis equal;

% Subplot 3: Base trajectory (XY)
subplot(3, 3, 3);
plot(logC1.qTraj(1, :), logC1.qTraj(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.qTraj(1, :), logC5.qTraj(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
xlabel('X (m)');
ylabel('Y (m)');
title('Base Trajectory (Top View)');
legend('Location', 'best');
grid on;
axis equal;

% Subplot 4: Base linear velocity
subplot(3, 3, 4);
dt1 = diff(logC1.time);
vx1 = diff(logC1.qTraj(1, :)) ./ dt1;
vy1 = diff(logC1.qTraj(2, :)) ./ dt1;
v1 = sqrt(vx1.^2 + vy1.^2);
dt5 = diff(logC5.time);
vx5 = diff(logC5.qTraj(1, :)) ./ dt5;
vy5 = diff(logC5.qTraj(2, :)) ./ dt5;
v5 = sqrt(vx5.^2 + vy5.^2);
plot(logC1.time(1:end-1), v1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.time(1:end-1), v5, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Base Linear Velocity');
legend('Location', 'best');
grid on;

% Subplot 5: Base angular velocity
subplot(3, 3, 5);
omega1 = diff(logC1.qTraj(3, :)) ./ dt1;
omega5 = diff(logC5.qTraj(3, :)) ./ dt5;
plot(logC1.time(1:end-1), omega1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1');
hold on;
plot(logC5.time(1:end-1), omega5, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 5');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Base Angular Velocity');
legend('Location', 'best');
grid on;

% Subplot 6: Arm joint trajectories
subplot(3, 3, 6);
plot(logC1.time, logC1.qTraj(4:9, :)', 'b-', 'LineWidth', 1);
hold on;
plot(logC5.time, logC5.qTraj(4:9, :)', 'r--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Arm Joint Trajectories');
legend('Method 1', '', '', '', '', '', 'Method 5', 'Location', 'best');
grid on;

% Subplot 7: Error histogram
subplot(3, 3, 7);
histogram(logC1.positionErrorNorm * 1000, 20, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Method 1');
hold on;
histogram(logC5.positionErrorNorm * 1000, 20, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Method 5');
xlabel('Position Error (mm)');
ylabel('Frequency');
title('Error Distribution');
legend('Location', 'best');
grid on;

% Subplot 8: Orientation error (Method 5 only - Method 1 doesn't track this)
subplot(3, 3, 8);
if isfield(logC5, 'orientationErrorAngle')
    plot(logC5.time, rad2deg(logC5.orientationErrorAngle), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Orientation Error (deg)');
    title('EE Orientation Error (Method 5)');
    grid on;
else
    text(0.5, 0.5, 'N/A', 'HorizontalAlignment', 'center', 'FontSize', 14);
    title('Orientation Error (N/A)');
end

% Subplot 9: Computation time comparison
subplot(3, 3, 9);
if isfield(logC5, 'solveTime')
    bar([time1, time5]);
    set(gca, 'XTickLabel', {'Method 1', 'Method 5'});
    ylabel('Total Time (s)');
    title('Computation Time');
    grid on;
    
    % Add solve time annotation for Method 5
    if isfield(logC5.stageCDiagnostics, 'meanSolveTime')
        text(2, time5, sprintf('%.1f ms avg', ...
            logC5.stageCDiagnostics.meanSolveTime * 1000), ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    end
end

%% Summary Statistics Table
fprintf('\n=== Summary Statistics ===\n\n');
fprintf('%-30s | %15s | %15s\n', 'Metric', 'Method 1', 'Method 5');
fprintf('%s\n', repmat('-', 65, 1));
fprintf('%-30s | %12.2f mm | %12.2f mm\n', 'Mean EE Position Error', meanErr1, meanErr5);
fprintf('%-30s | %12.2f mm | %12.2f mm\n', 'Max EE Position Error', maxErr1, maxErr5);
if isfield(logC5.stageCDiagnostics, 'meanEEOriError')
    fprintf('%-30s | %15s | %15.3f\n', 'Mean EE Orientation Error', 'N/A', logC5.stageCDiagnostics.meanEEOriError);
end
fprintf('%-30s | %12.1f s  | %12.1f s\n', 'Total Computation Time', time1, time5);
fprintf('%-30s | %15s | %12.1f ms\n', 'Avg Solve Time per Step', 'N/A', logC5.stageCDiagnostics.meanSolveTime * 1000);
fprintf('%-30s | %13.1f %% | %13.1f %%\n', 'Success/Convergence Rate', success1 * 100, success5 * 100);
fprintf('%-30s | %12.3f   | %12.3f\n', 'Mean Base Acceleration', smoothness1, smoothness5);
fprintf('%s\n', repmat('-', 65, 1));

%% Final Verdict
fprintf('\n=== Final Verdict ===\n');
fprintf('Method 1 (ppForIk): Feed-forward, fast, proven production method\n');
fprintf('Method 5 (pureMPC): Whole-body optimization, considers all DOF\n\n');

if meanErr5 < meanErr1 * 1.1  % Within 10% or better
    fprintf('✅ Method 5 achieves comparable or better accuracy\n');
end
if logC5.stageCDiagnostics.mpcConvergenceRate > 0.9
    fprintf('✅ Method 5 has good convergence rate (>90%%)\n');
end
if logC5.stageCDiagnostics.controlFrequency > 5
    fprintf('✅ Method 5 is capable of real-time control (>5 Hz)\n');
end

fprintf('\nComparison complete! Check the figure for detailed visualizations.\n');
