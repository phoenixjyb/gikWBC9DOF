%% test_stagec_ppfirst_simple.m
% Simple test of Method 4 (PP-First) on a short trajectory
%
% This script tests the runStageCPPFirst function on a simple 5-waypoint
% straight-line trajectory to verify basic functionality before running
% on the full trajectory.

clear; close all; clc;

fprintf('===== Method 4 (PP-First) Simple Test =====\n\n');

%% 1. Setup robot and environment
fprintf('1. Setting up robot and environment...\n');
robot = gik9dof.createRobotModel();
chassisParams = gik9dof.control.loadChassisProfile('wide_track');

% Starting configuration
q0 = homeConfiguration(robot);
q0 = gik9dof.configurationTools(robot).column(q0);

fprintf('   Robot: %d DOF\n', robot.NumBodies);
fprintf('   Chassis profile: wide_track (%.3f m track width)\n', chassisParams.track);

%% 2. Create simple test trajectory (5 waypoints, straight line)
fprintf('\n2. Creating simple test trajectory...\n');

% Get current EE position
T_start = getTransform(robot, q0, 'left_gripper_link');
p_start = T_start(1:3, 4);
R_start = T_start(1:3, 1:3);

% Create 5 waypoints moving forward in X direction
nWaypoints = 5;
targetPositions = zeros(3, nWaypoints);
for i = 1:nWaypoints
    targetPositions(:, i) = p_start + [0.1 * (i-1); 0; 0]; % 10cm steps forward
end

% Build trajectory struct
trajStruct = struct();
trajStruct.Poses = zeros(4, 4, nWaypoints);
trajStruct.EndEffectorPositions = targetPositions;
trajStruct.EndEffectorName = 'left_gripper_link';

for i = 1:nWaypoints
    trajStruct.Poses(:, :, i) = [R_start, targetPositions(:, i); 0 0 0 1];
end

fprintf('   Trajectory: %d waypoints\n', nWaypoints);
fprintf('   Start position: [%.2f, %.2f, %.2f]\n', p_start);
fprintf('   End position: [%.2f, %.2f, %.2f]\n', targetPositions(:, end));
fprintf('   Total distance: %.2f m\n', norm(targetPositions(:, end) - p_start));

%% 3. Run Method 4 (PP-First)
fprintf('\n3. Running Method 4 (PP-First)...\n');

tic;
log = gik9dof.runStageCPPFirst(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'MaxIterations', 1500, ...
    'YawTolerance', deg2rad(15), ...
    'PositionTolerance', 0.15, ...
    'EEErrorTolerance', 0.01, ...
    'DesiredVelocity', 0.3, ...
    'LookaheadDistance', 0.5, ...
    'ApplyRefinement', false, ...
    'VerboseLevel', 2);
elapsedTime = toc;

fprintf('\n   Total execution time: %.2f seconds\n', elapsedTime);

%% 4. Analyze results
fprintf('\n4. Analyzing results...\n');

fprintf('   Waypoints processed: %d\n', size(log.qTraj, 2));
fprintf('   Fallback rate: %.1f%% (%d/%d)\n', ...
    100*log.fallbackRate, sum(log.fallbackUsed), nWaypoints);
fprintf('   EE tracking error:\n');
fprintf('     Mean: %.3f mm\n', log.avgEEError * 1000);
fprintf('     Max: %.3f mm\n', log.maxEEError * 1000);
fprintf('     RMS: %.3f mm\n', rms(log.positionErrorNorm) * 1000);
fprintf('   Base deviation from PP:\n');
baseDeviation = vecnorm(log.baseActual - log.basePredicted, 2, 1);
fprintf('     Mean: %.3f mm\n', mean(baseDeviation) * 1000);
fprintf('     Max: %.3f mm\n', max(baseDeviation) * 1000);
fprintf('   GIK convergence: %.1f%% (%d/%d)\n', ...
    100*sum(log.successMask)/nWaypoints, sum(log.successMask), nWaypoints);
fprintf('   Solve time per waypoint: %.3f s (mean), %.3f s (max)\n', ...
    log.meanSolveTime, max(log.solveTime));

%% 5. Visualize results
fprintf('\n5. Creating visualization...\n');

figure('Name', 'Method 4 (PP-First) Simple Test', 'Position', [100 100 1400 600]);

% Subplot 1: 3D trajectory
subplot(2, 3, 1);
hold on; grid on; axis equal;
title('3D Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% Plot target EE path
plot3(log.targetPositions(1, :), log.targetPositions(2, :), log.targetPositions(3, :), ...
    'ro-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Target EE');

% Plot actual EE path
plot3(log.eePositions(1, :), log.eePositions(2, :), log.eePositions(3, :), ...
    'b*-', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'Actual EE');

legend('Location', 'best');
view(3);

% Subplot 2: Top-down view of base paths
subplot(2, 3, 2);
hold on; grid on; axis equal;
title('Base Paths (Top View)');
xlabel('X (m)'); ylabel('Y (m)');

% Base seed path
plot(log.baseSeedPath(:, 1), log.baseSeedPath(:, 2), ...
    'k--', 'LineWidth', 1, 'DisplayName', 'Seed Path');

% PP predicted path
plot(log.basePredicted(1, :), log.basePredicted(2, :), ...
    'g.-', 'LineWidth', 1.5, 'MarkerSize', 10, 'DisplayName', 'PP Predicted');

% Actual GIK base path
plot(log.baseActual(1, :), log.baseActual(2, :), ...
    'b*-', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'GIK Actual');

% Mark fallback points
if any(log.fallbackUsed)
    fallbackIdx = find(log.fallbackUsed);
    plot(log.baseActual(1, fallbackIdx), log.baseActual(2, fallbackIdx), ...
        'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Fallback');
end

legend('Location', 'best');

% Subplot 3: EE tracking error
subplot(2, 3, 3);
plot(1:nWaypoints, log.positionErrorNorm * 1000, 'b-o', 'LineWidth', 1.5);
hold on; grid on;
yline(log.parameters.eeErrorTolerance * 1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Fallback Threshold');
title('EE Tracking Error');
xlabel('Waypoint'); ylabel('Error (mm)');
legend('Location', 'best');

% Subplot 4: PP commands
subplot(2, 3, 4);
yyaxis left;
plot(1:nWaypoints, log.ppCommands(:, 1), 'b-o', 'LineWidth', 1.5);
ylabel('Linear Velocity (m/s)');
xlabel('Waypoint');
yyaxis right;
plot(1:nWaypoints, rad2deg(log.ppCommands(:, 2)), 'r-s', 'LineWidth', 1.5);
ylabel('Angular Velocity (deg/s)');
title('Pure Pursuit Commands');
grid on;

% Subplot 5: GIK iterations
subplot(2, 3, 5);
bar(1:nWaypoints, log.gikIterations);
hold on; grid on;
yline(log.parameters.maxIterations, 'r--', 'LineWidth', 2, 'DisplayName', 'Max Iterations');
title('GIK Iterations per Waypoint');
xlabel('Waypoint'); ylabel('Iterations');
legend('Location', 'best');

% Subplot 6: Solve time
subplot(2, 3, 6);
bar(1:nWaypoints, log.solveTime * 1000);
hold on; grid on;
title('Solve Time per Waypoint');
xlabel('Waypoint'); ylabel('Time (ms)');

%% 6. Test assessment
fprintf('\n6. Test assessment...\n');

allPassed = true;

% Test 1: All waypoints processed
if size(log.qTraj, 2) == nWaypoints
    fprintf('   ✓ PASS: All %d waypoints processed\n', nWaypoints);
else
    fprintf('   ✗ FAIL: Expected %d waypoints, got %d\n', nWaypoints, size(log.qTraj, 2));
    allPassed = false;
end

% Test 2: EE tracking error below threshold
if log.maxEEError < 0.015 % 15mm tolerance for simple trajectory
    fprintf('   ✓ PASS: Max EE error %.2f mm < 15 mm\n', log.maxEEError * 1000);
else
    fprintf('   ✗ FAIL: Max EE error %.2f mm exceeds 15 mm\n', log.maxEEError * 1000);
    allPassed = false;
end

% Test 3: Low fallback rate for simple trajectory
if log.fallbackRate < 0.5 % Expect <50% fallback for straight line
    fprintf('   ✓ PASS: Fallback rate %.1f%% < 50%%\n', 100*log.fallbackRate);
else
    fprintf('   ✗ WARN: Fallback rate %.1f%% high for simple trajectory\n', 100*log.fallbackRate);
    % Not a hard fail, just a warning
end

% Test 4: GIK convergence
if sum(log.successMask) / nWaypoints > 0.8
    fprintf('   ✓ PASS: GIK convergence rate %.1f%% > 80%%\n', 100*sum(log.successMask)/nWaypoints);
else
    fprintf('   ✗ FAIL: GIK convergence rate %.1f%% < 80%%\n', 100*sum(log.successMask)/nWaypoints);
    allPassed = false;
end

% Test 5: Reasonable solve time
if log.meanSolveTime < 1.0 % Less than 1 second per waypoint
    fprintf('   ✓ PASS: Mean solve time %.3f s < 1.0 s\n', log.meanSolveTime);
else
    fprintf('   ✗ WARN: Mean solve time %.3f s > 1.0 s\n', log.meanSolveTime);
    % Not a hard fail
end

fprintf('\n');
if allPassed
    fprintf('===== ALL TESTS PASSED ✓ =====\n');
else
    fprintf('===== SOME TESTS FAILED ✗ =====\n');
end

fprintf('\nSimple test complete. Ready for full trajectory testing.\n');
