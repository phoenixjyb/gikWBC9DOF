%% Quick Method 1 vs Method 4 Comparison
time4 = toc;
fprintf('✓ Method 4 done in %.1f sec (%.1f min)\n\n', time4, time4/60);

% Extract logs
logC1 = log1.stageLogs.stageC;
logC4 = log4.stageLogs.stageC;
and paste this into MATLAB command window

clear; close all; clc;

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  Method 1 (ppForIk) vs Method 4 (ppFirst) Comparison\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

% Get environment config
env = gik9dof.environmentConfig();

fprintf('Running Method 1 (ppForIk)...\n');
tic;
log1 = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'RateHz', 10, ...
    'Verbose', false, ...
    'EnvironmentConfig', env, ...
    'FloorDiscs', env.FloorDiscs, ...
    'DistanceMargin', env.DistanceMargin, ...
    'DistanceWeight', env.DistanceWeight, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'ExecutionMode', 'ppForIk');
time1 = toc;
fprintf('✓ Method 1 done in %.1f sec (%.1f min)\n\n', time1, time1/60);

fprintf('Running Method 4 (ppFirst)...\n');
tic;
log4 = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'RateHz', 10, ...
    'Verbose', false, ...
    'EnvironmentConfig', env, ...
    'FloorDiscs', env.FloorDiscs, ...
    'DistanceMargin', env.DistanceMargin, ...
    'DistanceWeight', env.DistanceWeight, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'ExecutionMode', 'ppFirst');
time4 = toc;
fprintf('✓ Method 4 done in %.1f sec (%.1f min)\n\n', time4, time4/60);

% Extract logs
log1 = result1.log.stageLogs.stageC;
log4 = result4.log.stageLogs.stageC;

% Compute metrics
fprintf('═══════════════════════════════════════════════════════\n');
fprintf('                  COMPARISON RESULTS\n');
fprintf('═══════════════════════════════════════════════════════\n\n');
fprintf('METRIC                  │  METHOD 1  │  METHOD 4  │  DELTA\n');
fprintf('────────────────────────┼────────────┼────────────┼─────────\n');
fprintf('Total Time (s)          │   %6.1f   │   %6.1f   │ %+6.1f\n', time1, time4, time4-time1);
fprintf('Waypoints               │   %6d   │   %6d   │  %5d\n', size(logC1.qTraj,2), size(logC4.qTraj,2), 0);
fprintf('────────────────────────┼────────────┼────────────┼─────────\n');
fprintf('EE Error Mean (mm)      │   %6.2f   │   %6.2f   │ %+6.2f\n', ...
    mean(logC1.positionErrorNorm)*1000, mean(logC4.positionErrorNorm)*1000, ...
    (mean(logC4.positionErrorNorm)-mean(logC1.positionErrorNorm))*1000);
fprintf('EE Error Max (mm)       │   %6.2f   │   %6.2f   │ %+6.2f\n', ...
    max(logC1.positionErrorNorm)*1000, max(logC4.positionErrorNorm)*1000, ...
    (max(logC4.positionErrorNorm)-max(logC1.positionErrorNorm))*1000);
fprintf('────────────────────────┼────────────┼────────────┼─────────\n');
fprintf('GIK Iterations (mean)   │   %6.1f   │   %6.1f   │ %+6.1f\n', ...
    mean(logC1.iterations), mean(logC4.iterations), mean(logC4.iterations)-mean(logC1.iterations));
fprintf('Convergence Rate (%%)    │   %6.1f   │   %6.1f   │ %+6.1f\n', ...
    sum(logC1.successMask)/length(logC1.successMask)*100, ...
    sum(logC4.successMask)/length(logC4.successMask)*100, ...
    (sum(logC4.successMask)/length(logC4.successMask)-sum(logC1.successMask)/length(logC1.successMask))*100);
fprintf('════════════════════════════════════════════════════════\n');

% Quick plots
figure('Name', 'EE Error Comparison', 'Position', [100 100 1200 500]);
subplot(1,2,1);
plot(logC1.timestamps, logC1.positionErrorNorm*1000, 'b-', 'LineWidth', 1.5); hold on;
plot(logC4.timestamps, logC4.positionErrorNorm*1000, 'r-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('EE Error (mm)');
title('End-Effector Tracking Error');
legend('Method 1 (ppForIk)', 'Method 4 (ppFirst)');

subplot(1,2,2);
plot(logC1.qTraj(1,:), logC1.qTraj(2,:), 'b-', 'LineWidth', 2); hold on;
plot(logC4.qTraj(1,:), logC4.qTraj(2,:), 'r-', 'LineWidth', 2);
axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)');
title('Base Paths');
legend('Method 1', 'Method 4');

fprintf('\nDone! Logs available as: log1, log4\n');
