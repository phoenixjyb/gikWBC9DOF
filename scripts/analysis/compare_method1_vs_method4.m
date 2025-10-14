%COMPARE_METHOD1_VS_METHOD4 Compare Method 1 (ppForIk) vs Method 4 (ppFirst)
%   Runs both methods on the same trajectory and generates comparison report.
%
%   This script:
%   1. Runs Method 1 (ppForIk) - default three-pass architecture
%   2. Runs Method 4 (ppFirst) - PP-first with constrained GIK
%   3. Compares performance metrics
%   4. Generates side-by-side plots
%   5. Saves comparison report
%
%   Output: results/<timestamp>_method_comparison/

%% Setup
clear; close all; clc;

% ═══════════════════════════════════════════════════════════
% CONFIGURATION FLAGS
% ═══════════════════════════════════════════════════════════
RUN_METHOD1 = false;  % Set to false to skip Method 1 (if already have results)
RUN_METHOD4 = true;   % Set to false to skip Method 4 (if already have results)

% If skipping methods, specify existing log file paths:
EXISTING_METHOD1_LOG = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
EXISTING_METHOD4_LOG = 'results/20251013_151157_method_comparison/log_method4_ppFirst.mat';
% ═══════════════════════════════════════════════════════════

fprintf('═════════════════════════════════════════════════════════\n');
fprintf('  Method 1 (ppForIk) vs Method 4 (ppFirst) Comparison\n');
fprintf('═════════════════════════════════════════════════════════\n\n');

% Create results directory
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
resultsDir = fullfile('results', sprintf('%s_method_comparison', timestamp));
mkdir(resultsDir);
fprintf('Results will be saved to: %s\n\n', resultsDir);

%% Load environment configuration
fprintf('[1/6] Loading environment and robot model...\n');
env = gik9dof.environmentConfig();
fprintf('  ✓ Environment: %d obstacles\n', length(env.FloorDiscs));
fprintf('  ✓ Base home: [%.2f, %.2f, %.2f]\n', env.BaseHome);

%% Common parameters
commonParams = struct();
commonParams.RateHz = 10;
commonParams.MaxIterations = 1500;

fprintf('Common parameters:\n');
fprintf('  • Control rate: %d Hz\n', commonParams.RateHz);
fprintf('  • GIK max iterations: %d\n', commonParams.MaxIterations);

%% Run Method 1 (ppForIk)
if RUN_METHOD1
    fprintf('\n[2/6] Running Method 1 (ppForIk - three-pass)...\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('Please wait... (this may take a few minutes)\n\n');

    tic;
    try
        % Run using trackReferenceTrajectory (proven method)
        log1 = gik9dof.trackReferenceTrajectory( ...
            'Mode', 'staged', ...
            'RateHz', commonParams.RateHz, ...
            'Verbose', false, ...
            'EnvironmentConfig', env, ...
            'FloorDiscs', env.FloorDiscs, ...
            'DistanceMargin', env.DistanceMargin, ...
            'DistanceWeight', env.DistanceWeight, ...
            'UseStageBHybridAStar', true, ...
            'StageBMode', 'pureHyb', ...
            'ExecutionMode', 'ppForIk');  % Method 1
        
        elapsed1 = toc;
        fprintf('\n✓ Method 1 completed in %.2f seconds (%.1f min)\n', elapsed1, elapsed1/60);
        
        % Extract Stage C log
        logC1 = log1.stageLogs.stageC;
        
        % Save log
        logPath1 = fullfile(resultsDir, 'log_method1_ppForIk.mat');
        save(logPath1, 'log1', 'elapsed1', '-v7.3');
        fprintf('  Saved to: %s\n', logPath1);
        
    catch ME
        fprintf('\n✗ Method 1 FAILED: %s\n', ME.message);
        fprintf('  File: %s\n', ME.stack(1).file);
        fprintf('  Line: %d\n', ME.stack(1).line);
        rethrow(ME);
    end
else
    fprintf('\n[2/6] Loading existing Method 1 results...\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    if exist(EXISTING_METHOD1_LOG, 'file')
        load(EXISTING_METHOD1_LOG, 'log1', 'elapsed1');
        logC1 = log1.stageLogs.stageC;
        fprintf('✓ Loaded from: %s\n', EXISTING_METHOD1_LOG);
        fprintf('  Original runtime: %.2f seconds (%.1f min)\n', elapsed1, elapsed1/60);
    else
        error('Method 1 log file not found: %s\nSet RUN_METHOD1=true to run it.', EXISTING_METHOD1_LOG);
    end
end
%% Run Method 4 (ppFirst)
if RUN_METHOD4
    fprintf('\n[3/6] Running Method 4 (ppFirst - PP-first constrained)...\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('Please wait... (this may take a few minutes)\n\n');

    tic;
    try
        % Run using trackReferenceTrajectory (proven method)
        log4 = gik9dof.trackReferenceTrajectory( ...
            'Mode', 'staged', ...
            'RateHz', commonParams.RateHz, ...
            'Verbose', false, ...
            'EnvironmentConfig', env, ...
            'FloorDiscs', env.FloorDiscs, ...
            'DistanceMargin', env.DistanceMargin, ...
            'DistanceWeight', env.DistanceWeight, ...
            'UseStageBHybridAStar', true, ...
            'StageBMode', 'pureHyb', ...
            'ExecutionMode', 'ppFirst');  % Method 4
        
        elapsed4 = toc;
        fprintf('\n✓ Method 4 completed in %.2f seconds (%.1f min)\n', elapsed4, elapsed4/60);
        
        % Extract Stage C log
        logC4 = log4.stageLogs.stageC;
        
        % Save log
        logPath4 = fullfile(resultsDir, 'log_method4_ppFirst.mat');
        save(logPath4, 'log4', 'elapsed4', '-v7.3');
        fprintf('  Saved to: %s\n', logPath4);
        
    catch ME
        fprintf('\n✗ Method 4 FAILED: %s\n', ME.message);
        fprintf('  File: %s\n', ME.stack(1).file);
        fprintf('  Line: %d\n', ME.stack(1).line);
        rethrow(ME);
    end
else
    fprintf('\n[3/6] Loading existing Method 4 results...\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    if exist(EXISTING_METHOD4_LOG, 'file')
        load(EXISTING_METHOD4_LOG, 'log4', 'elapsed4');
        logC4 = log4.stageLogs.stageC;
        fprintf('✓ Loaded from: %s\n', EXISTING_METHOD4_LOG);
        fprintf('  Original runtime: %.2f seconds (%.1f min)\n', elapsed4, elapsed4/60);
    else
        error('Method 4 log file not found: %s\nSet RUN_METHOD4=true to run it.', EXISTING_METHOD4_LOG);
    end
end

%% Compute Metrics
fprintf('\n[4/6] Computing comparison metrics...\n');

metrics = struct();

% Method 1 metrics
metrics.method1.totalTime = elapsed1;
metrics.method1.nWaypoints = size(logC1.qTraj, 2);
metrics.method1.eeErrorMean = mean(logC1.positionErrorNorm) * 1000;  % mm
metrics.method1.eeErrorMax = max(logC1.positionErrorNorm) * 1000;   % mm
metrics.method1.eeErrorRMS = sqrt(mean(logC1.positionErrorNorm.^2)) * 1000;  % mm
metrics.method1.gikIterMean = mean(logC1.iterations);
metrics.method1.gikIterMax = max(logC1.iterations);
metrics.method1.convergenceRate = sum(logC1.successMask) / length(logC1.successMask) * 100;

% Method 4 metrics
metrics.method4.totalTime = elapsed4;
metrics.method4.nWaypoints = size(logC4.qTraj, 2);
metrics.method4.eeErrorMean = mean(logC4.positionErrorNorm) * 1000;  % mm
metrics.method4.eeErrorMax = max(logC4.positionErrorNorm) * 1000;   % mm
metrics.method4.eeErrorRMS = sqrt(mean(logC4.positionErrorNorm.^2)) * 1000;  % mm
metrics.method4.gikIterMean = mean(logC4.iterations);
metrics.method4.gikIterMax = max(logC4.iterations);
metrics.method4.convergenceRate = sum(logC4.successMask) / length(logC4.successMask) * 100;

if isfield(logC4.diagnostics, 'fallbackRate')
    metrics.method4.fallbackRate = logC4.diagnostics.fallbackRate * 100;  % percent
    metrics.method4.fallbackCount = round(logC4.diagnostics.fallbackRate * metrics.method4.nWaypoints);
else
    metrics.method4.fallbackRate = 0;
    metrics.method4.fallbackCount = 0;
end

% Compute deltas
metrics.delta.eeErrorMean = metrics.method4.eeErrorMean - metrics.method1.eeErrorMean;
metrics.delta.eeErrorMax = metrics.method4.eeErrorMax - metrics.method1.eeErrorMax;
metrics.delta.eeErrorRMS = metrics.method4.eeErrorRMS - metrics.method1.eeErrorRMS;
metrics.delta.gikIterMean = metrics.method4.gikIterMean - metrics.method1.gikIterMean;
metrics.delta.totalTime = metrics.method4.totalTime - metrics.method1.totalTime;
metrics.delta.convergenceRate = metrics.method4.convergenceRate - metrics.method1.convergenceRate;

% Save metrics
metricsPath = fullfile(resultsDir, 'comparison_metrics.mat');
save(metricsPath, 'metrics');
fprintf('  ✓ Metrics computed and saved\n');

%% Print Comparison Table
fprintf('\n[4/4] Comparison Results:\n');
fprintf('═════════════════════════════════════════════════════════\n\n');

fprintf('METRIC                       │  METHOD 1  │  METHOD 4  │   DELTA\n');
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('Total Time (s)               │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.totalTime, metrics.method4.totalTime, metrics.delta.totalTime);
fprintf('Waypoints                    │   %6d   │   %6d   │    %+5d\n', ...
    metrics.method1.nWaypoints, metrics.method4.nWaypoints, 0);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('EE Error Mean (mm)           │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorMean, metrics.method4.eeErrorMean, metrics.delta.eeErrorMean);
fprintf('EE Error Max (mm)            │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorMax, metrics.method4.eeErrorMax, metrics.delta.eeErrorMax);
fprintf('EE Error RMS (mm)            │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorRMS, metrics.method4.eeErrorRMS, metrics.delta.eeErrorRMS);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('GIK Iter Mean                │   %6.1f   │   %6.1f   │ %+7.1f\n', ...
    metrics.method1.gikIterMean, metrics.method4.gikIterMean, metrics.delta.gikIterMean);
fprintf('GIK Iter Max                 │   %6d   │   %6d   │    %+5d\n', ...
    metrics.method1.gikIterMax, metrics.method4.gikIterMax, ...
    metrics.method4.gikIterMax - metrics.method1.gikIterMax);
fprintf('Convergence Rate (%%)         │   %6.1f   │   %6.1f   │ %+7.1f\n', ...
    metrics.method1.convergenceRate, metrics.method4.convergenceRate, metrics.delta.convergenceRate);
fprintf('─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf('Fallback Rate (%%)            │     N/A    │   %6.1f   │     ---\n', ...
    metrics.method4.fallbackRate);
fprintf('Fallback Count               │     N/A    │   %6d   │     ---\n', ...
    metrics.method4.fallbackCount);
fprintf('═════════════════════════════════════════════════════════\n\n');

% Interpretation
fprintf('INTERPRETATION:\n');
fprintf('──────────────\n');
if abs(metrics.delta.eeErrorMean) < 1.0
    fprintf('✓ EE tracking accuracy: EQUIVALENT (Δ %.2f mm)\n', metrics.delta.eeErrorMean);
elseif metrics.delta.eeErrorMean < 0
    fprintf('✓ EE tracking accuracy: METHOD 4 BETTER by %.2f mm\n', -metrics.delta.eeErrorMean);
else
    fprintf('⚠ EE tracking accuracy: METHOD 1 BETTER by %.2f mm\n', metrics.delta.eeErrorMean);
end

if abs(metrics.delta.totalTime) < 5.0
    fprintf('✓ Computation time: EQUIVALENT (Δ %.2f s)\n', metrics.delta.totalTime);
elseif metrics.delta.totalTime < 0
    fprintf('✓ Computation time: METHOD 4 FASTER by %.2f s\n', -metrics.delta.totalTime);
else
    fprintf('⚠ Computation time: METHOD 1 FASTER by %.2f s\n', metrics.delta.totalTime);
end

if metrics.method4.fallbackRate < 10.0
    fprintf('✓ Fallback rate: EXCELLENT (%.1f%% < 10%% target)\n', metrics.method4.fallbackRate);
elseif metrics.method4.fallbackRate < 20.0
    fprintf('⚠ Fallback rate: ACCEPTABLE (%.1f%% < 20%%)\n', metrics.method4.fallbackRate);
else
    fprintf('✗ Fallback rate: HIGH (%.1f%% > 20%%)\n', metrics.method4.fallbackRate);
end

fprintf('\n');

%% Generate Comparison Plots
fprintf('\nGenerating comparison plots...\n');

% Figure 1: EE Error Comparison
fig1 = figure('Name', 'EE Error Comparison', 'Position', [100 100 1400 800]);

subplot(2,2,1);
plot(logC1.timestamps, logC1.positionErrorNorm * 1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(logC4.timestamps, logC4.positionErrorNorm * 1000, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('EE Position Error (mm)');
title('End-Effector Tracking Error');
legend('Method 1 (ppForIk)', 'Method 4 (ppFirst)', 'Location', 'best');

subplot(2,2,2);
edges = 0:0.5:20;
histogram(logC1.positionErrorNorm * 1000, edges, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
hold on;
histogram(logC4.positionErrorNorm * 1000, edges, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
grid on;
xlabel('EE Error (mm)');
ylabel('Count');
title('Error Distribution');
legend('Method 1', 'Method 4');

subplot(2,2,3);
plot(logC1.timestamps, logC1.iterations, 'b-', 'LineWidth', 1.5);
hold on;
plot(logC4.timestamps, logC4.iterations, 'r-', 'LineWidth', 1.5);
yline(commonParams.MaxIterations, 'k--', 'Max Iterations');
grid on;
xlabel('Time (s)');
ylabel('GIK Iterations');
title('Solver Iterations per Waypoint');
legend('Method 1', 'Method 4', 'Location', 'best');

subplot(2,2,4);
data = [metrics.method1.eeErrorMean, metrics.method4.eeErrorMean; ...
        metrics.method1.eeErrorMax, metrics.method4.eeErrorMax; ...
        metrics.method1.eeErrorRMS, metrics.method4.eeErrorRMS];
bar(data);
set(gca, 'XTickLabel', {'Mean', 'Max', 'RMS'});
ylabel('EE Error (mm)');
title('EE Error Summary');
legend('Method 1', 'Method 4', 'Location', 'best');
grid on;

figPath1 = fullfile(resultsDir, 'comparison_ee_error.png');
saveas(fig1, figPath1);
fprintf('  ✓ Saved: %s\n', figPath1);

% Figure 2: Base Path Comparison
fig2 = figure('Name', 'Base Path Comparison', 'Position', [150 150 1400 600]);

subplot(1,2,1);
% Method 1
basePath1 = logC1.qTraj(1:2, :);
plot(basePath1(1,:), basePath1(2,:), 'b-', 'LineWidth', 2);
hold on;
scatter(basePath1(1,1), basePath1(2,1), 100, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter(basePath1(1,end), basePath1(2,end), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Method 1: Base Path (ppForIk)');
legend('Base path', 'Start', 'End', 'Location', 'best');

subplot(1,2,2);
% Method 4
basePath4 = logC4.qTraj(1:2, :);
plot(basePath4(1,:), basePath4(2,:), 'r-', 'LineWidth', 2);
hold on;
scatter(basePath4(1,1), basePath4(2,1), 100, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter(basePath4(1,end), basePath4(2,end), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
% Mark fallback points if available
if isfield(logC4, 'fallbackUsed') && any(logC4.fallbackUsed)
    fbIdx = find(logC4.fallbackUsed);
    scatter(basePath4(1,fbIdx), basePath4(2,fbIdx), 50, 'y', 'filled', 'MarkerEdgeColor', 'k');
end
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Method 4: Base Path (ppFirst)');
if isfield(logC4, 'fallbackUsed') && any(logC4.fallbackUsed)
    legend('Base path', 'Start', 'End', 'Fallback points', 'Location', 'best');
else
    legend('Base path', 'Start', 'End', 'Location', 'best');
end

figPath2 = fullfile(resultsDir, 'comparison_base_path.png');
saveas(fig2, figPath2);
fprintf('  ✓ Saved: %s\n', figPath2);

%% Write Text Report
fprintf('\nWriting comparison report...\n');

reportPath = fullfile(resultsDir, 'comparison_report.txt');
fid = fopen(reportPath, 'w');

fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Method 1 (ppForIk) vs Method 4 (ppFirst) Comparison Report\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
fprintf(fid, 'Generated: %s\n', datestr(now));
fprintf(fid, 'Results Directory: %s\n\n', resultsDir);

fprintf(fid, 'CONFIGURATION:\n');
fprintf(fid, '──────────────\n');
fprintf(fid, '• Control Rate: %d Hz\n', commonParams.RateHz);
fprintf(fid, '• GIK Max Iterations: %d\n', commonParams.MaxIterations);
fprintf(fid, '• Trajectory: 1_pull_world_scaled.json\n\n');

fprintf(fid, 'QUANTITATIVE RESULTS:\n');
fprintf(fid, '─────────────────────\n\n');
fprintf(fid, 'METRIC                       │  METHOD 1  │  METHOD 4  │   DELTA\n');
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'Total Time (s)               │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.totalTime, metrics.method4.totalTime, metrics.delta.totalTime);
fprintf(fid, 'Waypoints                    │   %6d   │   %6d   │    %+5d\n', ...
    metrics.method1.nWaypoints, metrics.method4.nWaypoints, 0);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'EE Error Mean (mm)           │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorMean, metrics.method4.eeErrorMean, metrics.delta.eeErrorMean);
fprintf(fid, 'EE Error Max (mm)            │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorMax, metrics.method4.eeErrorMax, metrics.delta.eeErrorMax);
fprintf(fid, 'EE Error RMS (mm)            │   %6.2f   │   %6.2f   │ %+7.2f\n', ...
    metrics.method1.eeErrorRMS, metrics.method4.eeErrorRMS, metrics.delta.eeErrorRMS);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'GIK Iter Mean                │   %6.1f   │   %6.1f   │ %+7.1f\n', ...
    metrics.method1.gikIterMean, metrics.method4.gikIterMean, metrics.delta.gikIterMean);
fprintf(fid, 'GIK Iter Max                 │   %6d   │   %6d   │    %+5d\n', ...
    metrics.method1.gikIterMax, metrics.method4.gikIterMax, ...
    metrics.method4.gikIterMax - metrics.method1.gikIterMax);
fprintf(fid, 'Convergence Rate (%%)         │   %6.1f   │   %6.1f   │ %+7.1f\n', ...
    metrics.method1.convergenceRate, metrics.method4.convergenceRate, metrics.delta.convergenceRate);
fprintf(fid, '─────────────────────────────┼────────────┼────────────┼──────────\n');
fprintf(fid, 'Fallback Rate (%%)            │     N/A    │   %6.1f   │     ---\n', ...
    metrics.method4.fallbackRate);
fprintf(fid, 'Fallback Count               │     N/A    │   %6d   │     ---\n', ...
    metrics.method4.fallbackCount);
fprintf(fid, '═════════════════════════════════════════════════════════════════\n\n');

fprintf(fid, 'INTERPRETATION:\n');
fprintf(fid, '───────────────\n');
if abs(metrics.delta.eeErrorMean) < 1.0
    fprintf(fid, '✓ EE tracking accuracy: EQUIVALENT\n');
elseif metrics.delta.eeErrorMean < 0
    fprintf(fid, '✓ EE tracking accuracy: METHOD 4 BETTER\n');
else
    fprintf(fid, '⚠ EE tracking accuracy: METHOD 1 BETTER\n');
end
if abs(metrics.delta.totalTime) < 5.0
    fprintf(fid, '✓ Computation time: EQUIVALENT\n');
elseif metrics.delta.totalTime < 0
    fprintf(fid, '✓ Computation time: METHOD 4 FASTER\n');
else
    fprintf(fid, '⚠ Computation time: METHOD 1 FASTER\n');
end
if metrics.method4.fallbackRate < 10.0
    fprintf(fid, '✓ Fallback rate: EXCELLENT\n');
elseif metrics.method4.fallbackRate < 20.0
    fprintf(fid, '⚠ Fallback rate: ACCEPTABLE\n');
else
    fprintf(fid, '✗ Fallback rate: HIGH\n');
end

fprintf(fid, '\nCONCLUSION:\n');
fprintf(fid, '───────────\n');
fprintf(fid, 'Method 4 (ppFirst) provides an alternative architecture that integrates\n');
fprintf(fid, 'Pure Pursuit predictions directly into the GIK solve process, reducing\n');
fprintf(fid, 'the decoupling between base planning and arm motion seen in Method 1.\n\n');

if abs(metrics.delta.eeErrorMean) < 2.0 && metrics.method4.fallbackRate < 25.0
    fprintf(fid, 'RECOMMENDATION: Method 4 is ready for production consideration.\n');
    fprintf(fid, 'EE tracking accuracy is comparable and fallback rate is acceptable.\n');
else
    fprintf(fid, 'RECOMMENDATION: Further tuning recommended before production.\n');
    fprintf(fid, 'Consider adjusting yaw corridor or position tolerance parameters.\n');
end

fprintf(fid, '\nGENERATED FILES:\n');
fprintf(fid, '────────────────\n');
fprintf(fid, '• log_method1_ppForIk.mat       - Method 1 simulation log\n');
fprintf(fid, '• log_method4_ppFirst.mat       - Method 4 simulation log\n');
fprintf(fid, '• comparison_metrics.mat        - Numerical comparison metrics\n');
fprintf(fid, '• comparison_ee_error.png       - EE error plots\n');
fprintf(fid, '• comparison_base_path.png      - Base path visualization\n');
fprintf(fid, '• comparison_report.txt         - This file\n\n');

fclose(fid);
fprintf('  ✓ Saved: %s\n', reportPath);

%% Summary
fprintf('\n═════════════════════════════════════════════════════════\n');
fprintf('  COMPARISON COMPLETE\n');
fprintf('═════════════════════════════════════════════════════════\n\n');
fprintf('Results saved to: %s\n', resultsDir);
fprintf('\nGenerated files:\n');
fprintf('  • log_method1_ppForIk.mat\n');
fprintf('  • log_method4_ppFirst.mat\n');
fprintf('  • comparison_metrics.mat\n');
fprintf('  • comparison_ee_error.png\n');
fprintf('  • comparison_base_path.png\n');
fprintf('  • comparison_report.txt\n');
fprintf('\nTo view report:\n');
fprintf('  type %s\n', reportPath);
fprintf('\n');
