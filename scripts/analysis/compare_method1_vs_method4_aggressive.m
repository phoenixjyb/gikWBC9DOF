%% Compare Method 1 vs Method 4 (with AGGRESSIVE profile)
% This script runs Method 4 with aggressive profile parameters:
%   - 20° yaw corridor (vs 15° default) → More GIK freedom
%   - 0.20m position tolerance (vs 0.15m) → Larger search space
%   - 0.015m EE threshold (vs 0.010m) → More tolerant fallback
%
% Hypothesis: Relaxed constraints should improve convergence rate

clear; close all; clc;

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Method 1 vs Method 4 Comparison (Aggressive Profile)\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% Configuration
RUN_METHOD1 = false;  % Skip Method 1 (use existing results)
RUN_METHOD4 = true;   % Run Method 4 with aggressive profile

% Paths
EXISTING_METHOD1_LOG = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
TRAJECTORY_FILE = '1_pull_world_scaled.json';
TIMESTAMP = datestr(now, 'yyyymmdd_HHMMSS');
RESULTS_DIR = fullfile('results', [TIMESTAMP '_method_comparison_aggressive']);

% Create results directory
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

%% [1/6] Setup Environment
fprintf('[1/6] Setting up environment...\n');
addpath('matlab');

robot = gik9dof.createRobotModel();
configTools = gik9dof.configurationTools(robot);
q0 = homeConfiguration(robot);

% Load 2-obstacle environment (same as previous test)
floorDiscs = struct([]);
floorDiscs(1).center = [-1.0, -1.0, 0.0];
floorDiscs(1).radius = 0.3;
floorDiscs(2).center = [1.0, 1.0, 0.0];
floorDiscs(2).radius = 0.3;

fprintf('  ✓ Robot model created\n');
fprintf('  ✓ Environment: 2 obstacles\n');
fprintf('  ✓ Base home: [%.1f, %.1f, %.1f]\n\n', q0(1:3));

%% [2/6] Load or Run Method 1
fprintf('[2/6] Method 1 (ppForIk - PP as base initializer)...\n');

if RUN_METHOD1
    fprintf('Running Method 1 (this takes ~37 minutes)...\n');
    tic;
    traj1 = gik9dof.loadReferenceTrajectory(TRAJECTORY_FILE);
    result1 = gik9dof.trackReferenceTrajectory(robot, traj1, ...
        'InitialConfiguration', q0, ...
        'ConfigTools', configTools, ...
        'ExecutionMode', 'ppForIk', ...
        'FloorDiscs', floorDiscs, ...
        'Verbose', false);
    elapsed1 = toc;
    
    % Save
    log1 = result1.stageLogs.stageC;
    save(fullfile(RESULTS_DIR, 'log_method1_ppForIk.mat'), 'log1', 'elapsed1');
    fprintf('  ✓ Method 1 completed in %.2f seconds (%.1f min)\n', elapsed1, elapsed1/60);
else
    fprintf('Loading existing Method 1 results...\n');
    if ~exist(EXISTING_METHOD1_LOG, 'file')
        error('Method 1 log not found: %s', EXISTING_METHOD1_LOG);
    end
    data = load(EXISTING_METHOD1_LOG);
    log1 = data.log1;
    elapsed1 = data.elapsed1;
    fprintf('  ✓ Loaded from: %s\n', EXISTING_METHOD1_LOG);
    fprintf('  ✓ Method 1 runtime: %.2f seconds (%.1f min)\n', elapsed1, elapsed1/60);
end
fprintf('  ✓ Method 1 waypoints: %d\n\n', length(log1.qLog));

%% [3/6] Run Method 4 with AGGRESSIVE Profile
fprintf('[3/6] Running Method 4 (ppFirst - AGGRESSIVE profile)...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

if RUN_METHOD4
    % Load aggressive profile
    cfg = gik9dof.loadPipelineProfile('aggressive');
    
    % Display parameters
    fprintf('Configuration:\n');
    fprintf('  • Yaw corridor:     %.1f° (default: 15°)\n', cfg.stage_c.ppfirst.yaw_corridor_deg);
    fprintf('  • Position tol:     %.2fm (default: 0.15m)\n', cfg.stage_c.ppfirst.position_tolerance);
    fprintf('  • EE threshold:     %.3fm (default: 0.010m)\n', cfg.stage_c.ppfirst.ee_error_threshold);
    fprintf('\nPlease wait... (this may take a few minutes)\n\n');
    
    tic;
    traj4 = gik9dof.loadReferenceTrajectory(TRAJECTORY_FILE);
    result4 = gik9dof.trackReferenceTrajectory(robot, traj4, ...
        'InitialConfiguration', q0, ...
        'ConfigTools', configTools, ...
        'ExecutionMode', 'ppFirst', ...
        'PipelineConfig', cfg, ...
        'FloorDiscs', floorDiscs, ...
        'Verbose', false);
    elapsed4 = toc;
    
    log4 = result4.stageLogs.stageC;
    save(fullfile(RESULTS_DIR, 'log_method4_ppFirst_aggressive.mat'), 'log4', 'elapsed4');
    fprintf('✓ Method 4 completed in %.2f seconds (%.1f min)\n', elapsed4, elapsed4/60);
    fprintf('  Saved to: %s/log_method4_ppFirst_aggressive.mat\n\n', RESULTS_DIR);
else
    fprintf('  (Skipped - RUN_METHOD4 = false)\n\n');
    return;
end

%% [4/6] Compute Comparison Metrics
fprintf('[4/6] Computing comparison metrics...\n');

% Extract EE positions
eePos1 = cell2mat(arrayfun(@(i) log1.qLog(i).EndEffector.ActualPosition, 1:length(log1.qLog), 'UniformOutput', false)');
eePos4 = cell2mat(arrayfun(@(i) log4.qLog(i).EndEffector.ActualPosition, 1:length(log4.qLog), 'UniformOutput', false)');

eeRef1 = cell2mat(arrayfun(@(i) log1.qLog(i).EndEffector.RefPosition, 1:length(log1.qLog), 'UniformOutput', false)');
eeRef4 = cell2mat(arrayfun(@(i) log4.qLog(i).EndEffector.RefPosition, 1:length(log4.qLog), 'UniformOutput', false)');

% Compute errors
eeErr1 = vecnorm(eePos1 - eeRef1, 2, 2) * 1000;  % mm
eeErr4 = vecnorm(eePos4 - eeRef4, 2, 2) * 1000;  % mm

% GIK iterations
gikIter1 = arrayfun(@(i) log1.qLog(i).IterationsTaken, 1:length(log1.qLog));
gikIter4 = arrayfun(@(i) log4.qLog(i).IterationsTaken, 1:length(log4.qLog));

% Convergence
converged1 = sum(gikIter1 < 1500);
converged4 = sum(gikIter4 < 1500);

% Method 4 fallback detection
fallbackCount = 0;
if isfield(log4.qLog(1), 'FallbackTriggered')
    fallbackCount = sum(arrayfun(@(i) log4.qLog(i).FallbackTriggered, 1:length(log4.qLog)));
end

% Store metrics
metrics = struct();
metrics.time1 = elapsed1;
metrics.time4 = elapsed4;
metrics.waypoints1 = length(log1.qLog);
metrics.waypoints4 = length(log4.qLog);
metrics.eeErrMean1 = mean(eeErr1);
metrics.eeErrMean4 = mean(eeErr4);
metrics.eeErrMax1 = max(eeErr1);
metrics.eeErrMax4 = max(eeErr4);
metrics.eeErrRMS1 = sqrt(mean(eeErr1.^2));
metrics.eeErrRMS4 = sqrt(mean(eeErr4.^2));
metrics.gikIterMean1 = mean(gikIter1);
metrics.gikIterMean4 = mean(gikIter4);
metrics.gikIterMax1 = max(gikIter1);
metrics.gikIterMax4 = max(gikIter4);
metrics.convergenceRate1 = 100 * converged1 / length(log1.qLog);
metrics.convergenceRate4 = 100 * converged4 / length(log4.qLog);
metrics.fallbackRate4 = 100 * fallbackCount / length(log4.qLog);
metrics.fallbackCount4 = fallbackCount;

save(fullfile(RESULTS_DIR, 'comparison_metrics.mat'), 'metrics');
fprintf('  ✓ Metrics computed and saved\n\n');

%% [5/6] Display Results
fprintf('[5/6] Comparison Results:\n');
fprintf('═════════════════════════════════════════════════════════\n\n');
fprintf('METRIC                       │  METHOD 1  │  METHOD 4  │   DELTA\n');
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('Total Time (s)               │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.time1, metrics.time4, metrics.time4 - metrics.time1);
fprintf('Waypoints                    │     %4d   │     %4d   │     %+4d\n', metrics.waypoints1, metrics.waypoints4, metrics.waypoints4 - metrics.waypoints1);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('EE Error Mean (mm)           │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrMean1, metrics.eeErrMean4, metrics.eeErrMean4 - metrics.eeErrMean1);
fprintf('EE Error Max (mm)            │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrMax1, metrics.eeErrMax4, metrics.eeErrMax4 - metrics.eeErrMax1);
fprintf('EE Error RMS (mm)            │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrRMS1, metrics.eeErrRMS4, metrics.eeErrRMS4 - metrics.eeErrRMS1);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('GIK Iter Mean                │  %7.1f   │  %7.1f   │  %+7.1f\n', metrics.gikIterMean1, metrics.gikIterMean4, metrics.gikIterMean4 - metrics.gikIterMean1);
fprintf('GIK Iter Max                 │     %4d   │     %4d   │     %+4d\n', metrics.gikIterMax1, metrics.gikIterMax4, metrics.gikIterMax4 - metrics.gikIterMax1);
fprintf('Convergence Rate (%%)         │    %5.1f   │    %5.1f   │    %+5.1f\n', metrics.convergenceRate1, metrics.convergenceRate4, metrics.convergenceRate4 - metrics.convergenceRate1);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('Fallback Rate (%%)            │     N/A    │    %5.1f   │     ---\n', metrics.fallbackRate4);
fprintf('Fallback Count               │     N/A    │      %3d   │     ---\n', metrics.fallbackCount4);
fprintf('═════════════════════════════════════════════════════════\n\n');

% Interpretation
fprintf('INTERPRETATION:\n');
fprintf('──────────────\n');
if metrics.eeErrMean4 < metrics.eeErrMean1
    fprintf('✓ EE tracking accuracy: METHOD 4 BETTER by %.2f mm\n', metrics.eeErrMean1 - metrics.eeErrMean4);
else
    fprintf('⚠ EE tracking accuracy: METHOD 1 BETTER by %.2f mm\n', metrics.eeErrMean4 - metrics.eeErrMean1);
end

if metrics.time4 < metrics.time1
    fprintf('✓ Computation time: METHOD 4 FASTER by %.2f s\n', metrics.time1 - metrics.time4);
else
    fprintf('⚠ Computation time: METHOD 1 FASTER by %.2f s\n', metrics.time4 - metrics.time1);
end

if metrics.fallbackRate4 > 20
    fprintf('✗ Fallback rate: HIGH (%.1f%% > 20%%)\n', metrics.fallbackRate4);
elseif metrics.fallbackRate4 > 10
    fprintf('⚠ Fallback rate: MODERATE (%.1f%%)\n', metrics.fallbackRate4);
else
    fprintf('✓ Fallback rate: LOW (%.1f%% < 10%%)\n', metrics.fallbackRate4);
end

if metrics.convergenceRate4 > 90
    fprintf('✓ Convergence rate: EXCELLENT (%.1f%%)\n', metrics.convergenceRate4);
elseif metrics.convergenceRate4 > 70
    fprintf('⚠ Convergence rate: ACCEPTABLE (%.1f%%)\n', metrics.convergenceRate4);
else
    fprintf('✗ Convergence rate: POOR (%.1f%%)\n', metrics.convergenceRate4);
end

fprintf('\n\n');

%% [6/6] Generate Plots
fprintf('[6/6] Generating comparison plots...\n');

% Plot 1: EE Error Over Time
figure('Position', [100, 100, 1200, 400]);
subplot(1,2,1);
hold on; grid on;
plot(eeErr1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Method 1 (ppForIk)');
plot(eeErr4, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Method 4 (ppFirst Aggressive)');
xlabel('Waypoint Index');
ylabel('EE Error (mm)');
title('End-Effector Tracking Error');
legend('Location', 'best');
ylim([0, max([max(eeErr1), max(eeErr4)]) + 50]);

% Plot 2: Error Distribution
subplot(1,2,2);
hold on; grid on;
histogram(eeErr1, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Method 1');
histogram(eeErr4, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Method 4');
xlabel('EE Error (mm)');
ylabel('Frequency');
title('Error Distribution');
legend('Location', 'best');

saveas(gcf, fullfile(RESULTS_DIR, 'comparison_ee_error_aggressive.png'));
fprintf('  ✓ Saved: %s/comparison_ee_error_aggressive.png\n', RESULTS_DIR);

% Plot 3: Base Paths
figure('Position', [100, 100, 800, 600]);
hold on; grid on; axis equal;

basePos1 = cell2mat(arrayfun(@(i) log1.qLog(i).Configuration(1:2), 1:length(log1.qLog), 'UniformOutput', false)');
basePos4 = cell2mat(arrayfun(@(i) log4.qLog(i).Configuration(1:2), 1:length(log4.qLog), 'UniformOutput', false)');

plot(basePos1(:,1), basePos1(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Method 1');
plot(basePos4(:,1), basePos4(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Method 4 (Aggressive)');

% Plot obstacles
for i = 1:length(floorDiscs)
    viscircles(floorDiscs(i).center(1:2), floorDiscs(i).radius, 'Color', 'k', 'LineWidth', 1.5);
end

xlabel('X (m)'); ylabel('Y (m)');
title('Base Path Comparison');
legend('Location', 'best');

saveas(gcf, fullfile(RESULTS_DIR, 'comparison_base_path_aggressive.png'));
fprintf('  ✓ Saved: %s/comparison_base_path_aggressive.png\n\n', RESULTS_DIR);

%% Write Report
fprintf('Writing comparison report...\n');
reportFile = fullfile(RESULTS_DIR, 'comparison_report_aggressive.txt');
fid = fopen(reportFile, 'w');

fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Method 1 vs Method 4 Comparison (Aggressive Profile)\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
fprintf(fid, 'Generated: %s\n', datestr(now));
fprintf(fid, 'Results Directory: %s\n\n', RESULTS_DIR);

fprintf(fid, 'CONFIGURATION:\n');
fprintf(fid, '──────────────\n');
fprintf(fid, '• Control Rate: 10 Hz\n');
fprintf(fid, '• GIK Max Iterations: 1500\n');
fprintf(fid, '• Trajectory: %s\n', TRAJECTORY_FILE);
fprintf(fid, '• Method 4 Profile: AGGRESSIVE\n');
fprintf(fid, '  - Yaw corridor: 20° (vs 15° default)\n');
fprintf(fid, '  - Position tolerance: 0.20m (vs 0.15m)\n');
fprintf(fid, '  - EE error threshold: 0.015m (vs 0.010m)\n\n');

fprintf(fid, 'QUANTITATIVE RESULTS:\n');
fprintf(fid, '─────────────────────\n\n');
fprintf(fid, 'METRIC                       │  METHOD 1  │  METHOD 4  │   DELTA\n');
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'Total Time (s)               │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.time1, metrics.time4, metrics.time4 - metrics.time1);
fprintf(fid, 'Waypoints                    │     %4d   │     %4d   │     %+4d\n', metrics.waypoints1, metrics.waypoints4, metrics.waypoints4 - metrics.waypoints1);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'EE Error Mean (mm)           │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrMean1, metrics.eeErrMean4, metrics.eeErrMean4 - metrics.eeErrMean1);
fprintf(fid, 'EE Error Max (mm)            │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrMax1, metrics.eeErrMax4, metrics.eeErrMax4 - metrics.eeErrMax1);
fprintf(fid, 'EE Error RMS (mm)            │  %7.2f   │  %7.2f   │  %+7.2f\n', metrics.eeErrRMS1, metrics.eeErrRMS4, metrics.eeErrRMS4 - metrics.eeErrRMS1);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'GIK Iter Mean                │  %7.1f   │  %7.1f   │  %+7.1f\n', metrics.gikIterMean1, metrics.gikIterMean4, metrics.gikIterMean4 - metrics.gikIterMean1);
fprintf(fid, 'GIK Iter Max                 │     %4d   │     %4d   │     %+4d\n', metrics.gikIterMax1, metrics.gikIterMax4, metrics.gikIterMax4 - metrics.gikIterMax1);
fprintf(fid, 'Convergence Rate (%%)         │    %5.1f   │    %5.1f   │    %+5.1f\n', metrics.convergenceRate1, metrics.convergenceRate4, metrics.convergenceRate4 - metrics.convergenceRate1);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'Fallback Rate (%%)            │     N/A    │    %5.1f   │     ---\n', metrics.fallbackRate4);
fprintf(fid, 'Fallback Count               │     N/A    │      %3d   │     ---\n', metrics.fallbackCount4);
fprintf(fid, '═════════════════════════════════════════════════════════════════\n\n');

fprintf(fid, 'CONCLUSION:\n');
fprintf(fid, '───────────\n');
fprintf(fid, 'Method 4 (ppFirst) with AGGRESSIVE profile provides relaxed constraints\n');
fprintf(fid, 'that allow the GIK solver more freedom to find solutions.\n\n');

if metrics.convergenceRate4 > 90 && metrics.fallbackRate4 < 10
    fprintf(fid, 'RECOMMENDATION: Method 4 with aggressive profile is PRODUCTION READY.\n');
elseif metrics.convergenceRate4 > 70 && metrics.fallbackRate4 < 20
    fprintf(fid, 'RECOMMENDATION: Method 4 with aggressive profile shows promise but may\n');
    fprintf(fid, 'need further tuning for challenging trajectories.\n');
else
    fprintf(fid, 'RECOMMENDATION: Further parameter tuning or architectural changes needed.\n');
end

fclose(fid);
fprintf('  ✓ Saved: %s/comparison_report_aggressive.txt\n\n', RESULTS_DIR);

%% Summary
fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  COMPARISON COMPLETE\n');
fprintf('═════════════════════════════════════════════════════════\n\n');

fprintf('Results saved to: %s\n\n', RESULTS_DIR);

fprintf('Generated files:\n');
fprintf('  • log_method1_ppForIk.mat (or loaded from existing)\n');
fprintf('  • log_method4_ppFirst_aggressive.mat\n');
fprintf('  • comparison_metrics.mat\n');
fprintf('  • comparison_ee_error_aggressive.png\n');
fprintf('  • comparison_base_path_aggressive.png\n');
fprintf('  • comparison_report_aggressive.txt\n\n');

fprintf('To view report:\n');
fprintf('  type %s/comparison_report_aggressive.txt\n\n', RESULTS_DIR);
