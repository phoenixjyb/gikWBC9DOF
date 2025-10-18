%% Test Script: Method 6 (Alternating Control) - Proof of Concept
% This script tests the newly implemented Method 6 (alternating control)
% and compares it against Method 1 (ppForIk) baseline.
%
% Expected performance:
%   - Solve time: ~30ms per cycle (vs Method 1: 10ms, Method 5: 4000ms)
%   - Control rate: 20-50 Hz (vs Method 1: offline, Method 5: 0.25 Hz)
%   - Mean EE error: ~150-180mm (vs Method 1: 129mm, Method 5: 2310mm)
%
% Date: October 15, 2025
% Branch: mpc-dev-stageC

clear; close all; clc;

%% Configuration
fprintf('=== Method 6 (Alternating Control) - POC Test ===\n\n');

% Use default pipeline profile
cfg = gik9dof.loadPipelineProfile('default');

% Test parameters
test_rate_hz = 20;  % Start conservative (50ms timesteps)
cfg.execution.rate_hz = test_rate_hz;

fprintf('Configuration:\n');
fprintf('  Control rate: %d Hz (%.1f ms timesteps)\n', test_rate_hz, 1000/test_rate_hz);
fprintf('  Max iterations: %d\n', cfg.gik.max_iterations);
fprintf('  Pipeline profile: %s\n\n', cfg.meta.profile);

%% Prepare test trajectory
projectRoot = fileparts(mfilename('fullpath'));
jsonPath = fullfile(projectRoot, '1_pull_world_scaled.json');
rawJson = jsondecode(fileread(jsonPath));
maxWaypoints = min(25, numel(rawJson.poses));
rawJson.poses = rawJson.poses(1:maxWaypoints);

% Write the trimmed trajectory to a temporary JSON file
tempJson = tempname;
tempJson = [tempJson, '.json']; %#ok<AGROW>
fid = fopen(tempJson, 'w');
if fid == -1
    error('Unable to open temporary trajectory file for writing: %s', tempJson);
end
cleanup = onCleanup(@() delete(tempJson));
fprintf(fid, '%s', jsonencode(rawJson));
fclose(fid);

%% Test 1: Method 6 (Alternating Control)
fprintf('Test 1: Running Method 6 (alternating) on %d waypoints...\n', maxWaypoints);
tic;
try
    result_method6 = gik9dof.trackReferenceTrajectory(...
        'Mode', 'staged', ...
        'ExecutionMode', 'alternating', ...
        'PipelineConfig', cfg, ...
        'JsonPath', tempJson, ...
        'Verbose', false);
    
    time_method6 = toc;
    
    % Evaluate results
    eval6 = gik9dof.evaluateLog(result_method6);
    
    fprintf('\n✅ Method 6 SUCCESS!\n');
    fprintf('  Total time: %.2f seconds\n', time_method6);
    fprintf('  Total steps: %d\n', length(result_method6.time));
    fprintf('  Mean solve time: %.1f ms\n', mean(result_method6.stageLogs.stageC.solveTime)*1000);
    fprintf('  Control rate: %.1f Hz\n', 1/mean(result_method6.stageLogs.stageC.solveTime));
    fprintf('  Mean EE error: %.1f mm\n', eval6.eePositionError.mean*1000);
    fprintf('  Max EE error: %.1f mm\n', eval6.eePositionError.max*1000);
    
    % Count base vs arm steps
    logC = result_method6.stageLogs.stageC;
    base_steps = sum(strcmp(logC.mode, 'base'));
    arm_steps = sum(strcmp(logC.mode, 'arm'));
    fprintf('  Base optimization steps: %d (%.1f ms avg)\n', base_steps, ...
        mean(logC.solveTime(strcmp(logC.mode, 'base')))*1000);
    fprintf('  Arm optimization steps: %d (%.1f ms avg)\n\n', arm_steps, ...
        mean(logC.solveTime(strcmp(logC.mode, 'arm')))*1000);
    
    method6_success = true;
catch ME
    fprintf('\n❌ Method 6 FAILED: %s\n\n', ME.message);
    method6_success = false;
    result_method6 = [];
    eval6 = [];
end

%% Test 2: Method 1 (ppForIk) - Baseline Comparison
fprintf('Test 2: Running Method 1 (ppForIk) on %d waypoints for comparison...\n', maxWaypoints);
tic;
try
    result_method1 = gik9dof.trackReferenceTrajectory(...
        'Mode', 'staged', ...
        'ExecutionMode', 'ppForIk', ...
        'PipelineConfig', cfg, ...
        'JsonPath', tempJson, ...
        'Verbose', false);
    
    time_method1 = toc;
    
    % Evaluate results
    eval1 = gik9dof.evaluateLog(result_method1);
    
    fprintf('\n✅ Method 1 SUCCESS!\n');
    fprintf('  Total time: %.2f seconds\n', time_method1);
    fprintf('  Total steps: %d\n', length(result_method1.time));
    fprintf('  Mean EE error: %.1f mm\n', eval1.eePositionError.mean*1000);
    fprintf('  Max EE error: %.1f mm\n\n', eval1.eePositionError.max*1000);
    
    method1_success = true;
catch ME
    fprintf('\n❌ Method 1 FAILED: %s\n\n', ME.message);
    method1_success = false;
    result_method1 = [];
    eval1 = [];
end

%% Comparison
if method6_success && method1_success
    fprintf('=== COMPARISON: Method 6 vs Method 1 ===\n\n');
    
    fprintf('Performance Metrics:\n');
    fprintf('  %-30s %10s %10s %10s\n', 'Metric', 'Method 6', 'Method 1', 'Ratio');
    fprintf('  %s\n', repmat('-', 1, 65));
    
    % Mean EE error
    err6 = eval6.eePositionError.mean * 1000;
    err1 = eval1.eePositionError.mean * 1000;
    fprintf('  %-30s %9.1f mm %9.1f mm %9.2fx\n', 'Mean EE Error', err6, err1, err6/err1);
    
    % Max EE error
    errMax6 = eval6.eePositionError.max * 1000;
    errMax1 = eval1.eePositionError.max * 1000;
    fprintf('  %-30s %9.1f mm %9.1f mm %9.2fx\n', 'Max EE Error', errMax6, errMax1, errMax6/errMax1);
    
    % Control rate
    logC6 = result_method6.stageLogs.stageC;
    rate6 = 1 / mean(logC6.solveTime);
    fprintf('  %-30s %9.1f Hz %10s %10s\n', 'Control Rate', rate6, 'N/A', '—');
    
    % Solve time
    solve6 = mean(logC6.solveTime) * 1000;
    fprintf('  %-30s %9.1f ms %10s %10s\n', 'Mean Solve Time', solve6, 'N/A', '—');
    
    fprintf('\n');
    
    % Assessment
    fprintf('Assessment:\n');
    if err6 < 200
        fprintf('  ✅ Method 6 accuracy is GOOD (< 200mm mean error)\n');
    else
        fprintf('  ⚠️  Method 6 accuracy needs tuning (%.1f mm > 200mm target)\n', err6);
    end
    
    if solve6 < 50
        fprintf('  ✅ Method 6 solve time is EXCELLENT (< 50ms)\n');
    else
        fprintf('  ⚠️  Method 6 solve time needs optimization (%.1f ms > 50ms target)\n', solve6);
    end
    
    if rate6 > 15
        fprintf('  ✅ Method 6 control rate is GOOD (> 15 Hz)\n');
    else
        fprintf('  ⚠️  Method 6 control rate is low (%.1f Hz < 15 Hz target)\n', rate6);
    end
    
    fprintf('\n');
end

%% Visualization (optional)
if method6_success
    fprintf('Generating diagnostic plots for Method 6...\n');
    try
        fig = gik9dof.generateLogPlots(result_method6);
        sgtitle('Method 6 (Alternating Control) - Diagnostic Plots');
    catch ME
        fprintf('  Warning: Could not generate plots: %s\n', ME.message);
    end
end

%% Summary
fprintf('\n=== SUMMARY ===\n');
if method6_success
    fprintf('✅ Method 6 implementation is WORKING!\n');
    fprintf('\nNext steps:\n');
    fprintf('  1. Tune cost weights if accuracy > 200mm\n');
    fprintf('  2. Profile solve times (base vs arm)\n');
    fprintf('  3. Test on full trajectory set\n');
    fprintf('  4. Compare against Method 4 (ppFirst)\n');
else
    fprintf('❌ Method 6 implementation has ISSUES.\n');
    fprintf('Debug the error messages above and try again.\n');
end

fprintf('\n=== Test Complete ===\n');
