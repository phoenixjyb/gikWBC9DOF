%% Compare isolated Phase 2A test vs staged pipeline run
% This script compares the working isolated test with the failed staged run
% to identify why the same code produces different results

clear; clc;

fprintf('========================================\n');
fprintf('  Comparing Isolated vs Staged Results\n');
fprintf('========================================\n\n');

%% Load isolated test results (working correctly)
isolatedDir = 'results/phase2a_orientation_z';

% Find the most recent Phase 2A log file
logFiles = dir(fullfile(isolatedDir, 'log_phase2a_*.mat'));
if isempty(logFiles)
    error('Isolated test results not found. Run test_method4_phase2a.m first');
end

% Sort by date and take the most recent
[~, idx] = max([logFiles.datenum]);
isolatedFile = fullfile(isolatedDir, logFiles(idx).name);

data = load(isolatedFile);
isolatedLog = data.log_phase2a;

fprintf('✓ Loaded isolated test: %s\n', isolatedFile);
fprintf('  Mean EE Error: %.1f mm\n', mean(isolatedLog.positionErrorNorm) * 1000);
fprintf('  Fallback Rate: %.1f%%\n\n', (1 - mean(isolatedLog.successMask)) * 100);

%% Load staged pipeline results (not working)
stagedDir = 'results/20251014_145719_fresh_no_discs';
stagedFile = fullfile(stagedDir, 'log_staged_ppForIk.mat');

if ~isfile(stagedFile)
    error('Staged test results not found at: %s', stagedFile);
end

load(stagedFile, 'log');
stagedLog = log;

fprintf('✓ Loaded staged run: %s\n', stagedFile);
fprintf('  Execution Mode: %s\n', stagedLog.simulationMode);
fprintf('  Mean EE Error (Stage C): %.1f mm\n', ...
    mean(stagedLog.stageLogs.stageC.positionErrorNorm) * 1000);
fprintf('  Fallback Rate: %.1f%%\n\n', ...
    stagedLog.stageLogs.stageC.diagnostics.fallbackRate * 100);

%% Compare key metrics
fprintf('========================================\n');
fprintf('  DETAILED COMPARISON\n');
fprintf('========================================\n\n');

fprintf('1. END-EFFECTOR TRACKING ERROR:\n');
fprintf('   Isolated:  Mean = %.1f mm, Max = %.1f mm\n', ...
    mean(isolatedLog.positionErrorNorm) * 1000, ...
    max(isolatedLog.positionErrorNorm) * 1000);
fprintf('   Staged:    Mean = %.1f mm, Max = %.1f mm\n', ...
    mean(stagedLog.stageLogs.stageC.positionErrorNorm) * 1000, ...
    max(stagedLog.stageLogs.stageC.positionErrorNorm) * 1000);
fprintf('   Ratio:     %.1fx worse in staged\n\n', ...
    mean(stagedLog.stageLogs.stageC.positionErrorNorm) / mean(isolatedLog.positionErrorNorm));

fprintf('2. FALLBACK RATE:\n');
fprintf('   Isolated:  %.1f%% (%.0f/%.0f waypoints)\n', ...
    (1 - mean(isolatedLog.successMask)) * 100, ...
    sum(~isolatedLog.successMask), length(isolatedLog.successMask));
fprintf('   Staged:    %.1f%% (%.0f/%.0f waypoints)\n', ...
    stagedLog.stageLogs.stageC.diagnostics.fallbackRate * 100, ...
    stagedLog.stageLogs.stageC.diagnostics.ppFirst.fallbackCount, ...
    stagedLog.stageLogs.stageC.diagnostics.ppFirst.totalWaypoints);
fprintf('   Ratio:     %.1fx worse in staged\n\n', ...
    stagedLog.stageLogs.stageC.diagnostics.fallbackRate / (1 - mean(isolatedLog.successMask)));

fprintf('3. CONVERGENCE RATE:\n');
fprintf('   Isolated:  %.1f%%\n', mean(isolatedLog.successMask) * 100);
fprintf('   Staged:    %.1f%%\n', ...
    stagedLog.stageLogs.stageC.diagnostics.convergenceRate * 100);
fprintf('   Change:    %.1f%%\n\n', ...
    (stagedLog.stageLogs.stageC.diagnostics.convergenceRate - mean(isolatedLog.successMask)) * 100);

fprintf('4. NUMBER OF WAYPOINTS:\n');
fprintf('   Isolated:  %d waypoints\n', length(isolatedLog.successMask));
fprintf('   Staged:    %d waypoints (Stage C only)\n', ...
    stagedLog.stageLogs.stageC.diagnostics.ppFirst.totalWaypoints);

% Check if they're tracking the same trajectory
if length(isolatedLog.successMask) == stagedLog.stageLogs.stageC.diagnostics.ppFirst.totalWaypoints
    fprintf('   ✓ Same number of waypoints\n\n');
else
    fprintf('   ⚠️  DIFFERENT number of waypoints!\n\n');
end

%% Analyze error distributions
fprintf('========================================\n');
fprintf('  ERROR DISTRIBUTION ANALYSIS\n');
fprintf('========================================\n\n');

isolatedErrors = isolatedLog.positionErrorNorm * 1000; % mm
stagedErrors = stagedLog.stageLogs.stageC.positionErrorNorm * 1000; % mm

fprintf('Isolated test error bins:\n');
fprintf('  < 5mm:    %.1f%%\n', sum(isolatedErrors < 5) / length(isolatedErrors) * 100);
fprintf('  5-50mm:   %.1f%%\n', sum(isolatedErrors >= 5 & isolatedErrors < 50) / length(isolatedErrors) * 100);
fprintf('  50-500mm: %.1f%%\n', sum(isolatedErrors >= 50 & isolatedErrors < 500) / length(isolatedErrors) * 100);
fprintf('  > 500mm:  %.1f%%\n\n', sum(isolatedErrors >= 500) / length(isolatedErrors) * 100);

fprintf('Staged run error bins:\n');
fprintf('  < 5mm:    %.1f%%\n', sum(stagedErrors < 5) / length(stagedErrors) * 100);
fprintf('  5-50mm:   %.1f%%\n', sum(stagedErrors >= 5 & stagedErrors < 50) / length(stagedErrors) * 100);
fprintf('  50-500mm: %.1f%%\n', sum(stagedErrors >= 50 & stagedErrors < 500) / length(stagedErrors) * 100);
fprintf('  > 500mm:  %.1f%%\n\n', sum(stagedErrors >= 500) / length(stagedErrors) * 100);

%% Check if initial conditions differ
fprintf('========================================\n');
fprintf('  INITIAL CONFIGURATION CHECK\n');
fprintf('========================================\n\n');

isolatedQ0 = isolatedLog.qTraj(:, 1);
stagedQ0 = stagedLog.stageLogs.stageC.qTraj(:, 1);

fprintf('Isolated test starting config:\n');
fprintf('  Base: [%.3f, %.3f, %.3f] (x, y, theta)\n', ...
    isolatedQ0(1), isolatedQ0(2), isolatedQ0(3));
fprintf('  Arm:  [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
    isolatedQ0(4), isolatedQ0(5), isolatedQ0(6), isolatedQ0(7), isolatedQ0(8), isolatedQ0(9));

fprintf('\nStaged run starting config (Stage C):\n');
fprintf('  Base: [%.3f, %.3f, %.3f] (x, y, theta)\n', ...
    stagedQ0(1), stagedQ0(2), stagedQ0(3));
fprintf('  Arm:  [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n\n', ...
    stagedQ0(4), stagedQ0(5), stagedQ0(6), stagedQ0(7), stagedQ0(8), stagedQ0(9));

% Check if starting configurations are similar
qDiff = norm(isolatedQ0 - stagedQ0);
if qDiff < 0.01
    fprintf('✓ Starting configurations are IDENTICAL\n\n');
elseif qDiff < 0.1
    fprintf('⚠️  Starting configurations are SIMILAR (diff = %.3f)\n\n', qDiff);
else
    fprintf('❌ Starting configurations are DIFFERENT! (diff = %.3f)\n', qDiff);
    fprintf('   This could explain the performance difference\n\n');
end

%% Conclusion
fprintf('========================================\n');
fprintf('  DIAGNOSIS\n');
fprintf('========================================\n\n');

if mean(stagedErrors) > 100
    fprintf('❌ STAGED RUN FAILED: Mean error %.1f mm (expected ~1.2 mm)\n\n', mean(stagedErrors));
    fprintf('Possible causes:\n');
    fprintf('  1. Phase 2A parameters NOT being passed correctly\n');
    fprintf('  2. MATLAB function caching (old code still in memory)\n');
    fprintf('  3. Different initial conditions affecting performance\n');
    fprintf('  4. Bug in executeStageCPPFirst wrapper\n\n');
    
    fprintf('Recommended actions:\n');
    fprintf('  1. Clear MATLAB cache: clear all; rehash toolboxcache\n');
    fprintf('  2. Verify executeStageCPPFirst calls runStageCPPFirst_enhanced\n');
    fprintf('  3. Add debug prints to baseSeedFromEE to verify UseOrientationZNominal\n');
    fprintf('  4. Run debug_parameter_passing.m to isolate the issue\n');
else
    fprintf('✅ Both tests working correctly!\n');
end
