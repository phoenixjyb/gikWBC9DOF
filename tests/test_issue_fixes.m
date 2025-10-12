%TEST_ISSUE_FIXES Validate fixes for animation/path quality issues.
%   Tests recommended parameter changes to address:
%   #1: Arm tip misalignment (pure pursuit tracking)
%   #2: Jumpy Stage C reference paths (solver convergence)
%   #3: Stage B cusps (RS smoothing acceptance)

clear; clc;
fprintf('=== Testing Issue Fixes ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

projRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projRoot, 'matlab')));

% ===========================================================================
% FIXES APPLIED:
% #0: Enable RS smoothing (was disabled!)
% #1: MaxIterations 150 -> 300 (allow solver to converge)
% #2: Lookahead 0.8 -> 0.4m (tighter pure pursuit tracking)
% #3: lambdaCusp 1.0 -> 0.5 (accept more RS shortcuts)
% #4: RS validationDistance 0.035 -> 0.08m (less conservative collision check)
% #5: Accel limit 0.8 -> 1.0 m/s² (more responsive chassis)
% #6: Heading Kp 1.2 -> 1.5 (stronger heading correction)
% ===========================================================================

fprintf('Running staged trajectory with ISSUE FIXES...\n');
fprintf('  CRITICAL: Enable RS + Clothoid smoothing (was OFF!)\n');
fprintf('  MaxIterations: 300 (was 150)\n');
fprintf('  Lookahead: 0.4m (was 0.8m)\n');
fprintf('  RS lambdaCusp: 0.5 (was 1.0)\n');
fprintf('  RS validation: 0.08m (was 0.035m)\n');
fprintf('  Accel limit: 1.0 m/s² (was 0.8 m/s²)\n');
fprintf('  Heading Kp: 1.5 (was 1.2)\n\n');

tic;
result = gik9dof.runStagedReference( ...
    'RunLabel', 'ISSUE_FIXES_test_10hz', ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 300, ...                    % FIX #1: Allow convergence
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'DistanceMargin', 0.10, ...
    'StageBUseReedsShepp', true, ...             % ENABLE RS smoothing!
    'StageBUseClothoid', true, ...               % ENABLE clothoid smoothing!
    'StageBReedsSheppParams', struct( ...        % FIX #3 & #4: RS tuning
        'lambdaCusp', 0.5, ...
        'validationDistance', 0.08, ...
        'allowReverse', true, ...
        'iters', 200), ...
    'ChassisOverrides', struct( ...              % FIX #2, #5, #6: Pure pursuit tuning
        'lookahead_base', 0.4, ...
        'accel_limit', 1.0, ...
        'heading_kp', 1.5, ...
        'feedforward_gain', 0.95), ...
    'SaveLog', true);
elapsedTime = toc;

fprintf('✓ Simulation complete in %.1f seconds\n', elapsedTime);
fprintf('✓ Results saved to: %s\n\n', result.resultsDir);

% Extract diagnostics
log = result.log;
stageBDiag = log.stageLogs.stageB.diagnostics;
stageCDiag = log.stageLogs.stageC.diagnostics;

% ===========================================================================
% DIAGNOSTIC COMPARISON
% ===========================================================================
fprintf('=== STAGE B DIAGNOSTICS ===\n');
fprintf('[Reeds-Shepp Smoothing]\n');
fprintf('  Iterations: %d\n', stageBDiag.rsIterations);
fprintf('  Improvements: %d\n', stageBDiag.rsImprovements);
fprintf('  Acceptance rate: %.1f%%', 100*stageBDiag.rsAcceptanceRate);
if stageBDiag.rsAcceptanceRate > 0.10
    fprintf(' ✓ IMPROVED (was 0%%)\n');
elseif stageBDiag.rsAcceptanceRate > 0
    fprintf(' ↗ BETTER (was 0%%)\n');
else
    fprintf(' ✗ STILL ZERO\n');
end
fprintf('  Path length reduction: %.3f m\n', stageBDiag.rsPathLengthImprovement);

fprintf('\n[Path Quality]\n');
fprintf('  Cusp count: %d', stageBDiag.cuspCount);
if stageBDiag.cuspCount == 0
    fprintf(' ✓ GOOD\n');
else
    fprintf(' (target: 0)\n');
end
fprintf('  Path smoothness (σ): %.3f', stageBDiag.pathSmoothness);
if stageBDiag.pathSmoothness < 0.5
    fprintf(' ✓ EXCELLENT\n');
elseif stageBDiag.pathSmoothness < 1.0
    fprintf(' ✓ GOOD\n');
else
    fprintf(' (target: <0.5)\n');
end
fprintf('  Mean curvature: %.3f [1/m]\n', stageBDiag.meanCurvature);
fprintf('  Max curvature: %.3f [1/m]\n', stageBDiag.maxCurvature);

fprintf('\n[Curvature Distribution]\n');
hist = stageBDiag.curvatureHistogram;
total = hist.low + hist.medium + hist.high + hist.veryHigh;
if total > 0
    fprintf('  Low (<0.5):       %3d (%.1f%%)\n', hist.low, 100*hist.low/total);
    fprintf('  Medium (0.5-1.0): %3d (%.1f%%)\n', hist.medium, 100*hist.medium/total);
    fprintf('  High (1.0-2.0):   %3d (%.1f%%)\n', hist.high, 100*hist.high/total);
    fprintf('  Very High (>2.0): %3d (%.1f%%)\n', hist.veryHigh, 100*hist.veryHigh/total);
end

fprintf('\n=== STAGE C DIAGNOSTICS ===\n');
fprintf('[Solver Performance]\n');
fprintf('  Mean iterations: %.1f', stageCDiag.solverIterationsMean);
if stageCDiag.solverIterationsMean < 250
    fprintf(' ✓ CONVERGING\n');
else
    fprintf(' ⚠ Near limit\n');
end
fprintf('  Max iterations: %d (limit: 300)\n', stageCDiag.solverIterationsMax);
fprintf('  Std dev: %.1f', stageCDiag.solverIterationsStd);
if stageCDiag.solverIterationsStd > 0
    fprintf(' ✓ IMPROVED (was 0)\n');
else
    fprintf(' ✗ Still hitting cap\n');
end

exitFlags = stageCDiag.exitFlagHistogram;
totalFlags = exitFlags.success + exitFlags.maxIters + exitFlags.failed;
if totalFlags > 0
    fprintf('  Exit flags:\n');
    fprintf('    Success:   %3d (%.1f%%)\n', exitFlags.success, 100*exitFlags.success/totalFlags);
    fprintf('    Max iters: %3d (%.1f%%)\n', exitFlags.maxIters, 100*exitFlags.maxIters/totalFlags);
    fprintf('    Failed:    %3d (%.1f%%)\n', exitFlags.failed, 100*exitFlags.failed/totalFlags);
end

fprintf('\n[End-Effector Tracking]\n');
fprintf('  Mean error: %.4f m', stageCDiag.eeErrorMean);
if stageCDiag.eeErrorMean < 0.05
    fprintf(' ✓ EXCELLENT\n');
elseif stageCDiag.eeErrorMean < 0.10
    fprintf(' ✓ GOOD\n');
else
    fprintf(' (target: <0.10m)\n');
end
fprintf('  Max error: %.4f m', stageCDiag.eeErrorMax);
if stageCDiag.eeErrorMax < 0.20
    fprintf(' ✓ PASS\n');
else
    fprintf(' ✗ Above target\n');
end

bins = stageCDiag.eeErrorBins;
total = bins.excellent + bins.good + bins.acceptable + bins.poor;
if total > 0
    fprintf('  Distribution:\n');
    fprintf('    Excellent (<0.05m):    %.1f%%\n', 100*bins.excellent/total);
    fprintf('    Good (0.05-0.10m):     %.1f%%\n', 100*bins.good/total);
    fprintf('    Acceptable (0.10-0.20m): %.1f%%\n', 100*bins.acceptable/total);
    fprintf('    Poor (>0.20m):         %.1f%%\n', 100*bins.poor/total);
end

fprintf('\n[Base Tracking]\n');
fprintf('  Position deviation: %.3f m mean, %.3f m max', ...
    stageCDiag.basePosDeviationMean, stageCDiag.basePosDeviationMax);
if stageCDiag.basePosDeviationMean < 0.3
    fprintf(' ✓ EXCELLENT (was 1.3m)\n');
elseif stageCDiag.basePosDeviationMean < 0.6
    fprintf(' ✓ IMPROVED (was 1.3m)\n');
else
    fprintf(' (was 1.3m)\n');
end
fprintf('  Yaw drift: %.3f rad mean, %.3f rad max (%.1f°, %.1f°)\n', ...
    stageCDiag.baseYawDriftMean, stageCDiag.baseYawDriftMax, ...
    rad2deg(stageCDiag.baseYawDriftMean), rad2deg(stageCDiag.baseYawDriftMax));

% ===========================================================================
% COMPARISON WITH PREVIOUS SWEEP
% ===========================================================================
fprintf('\n=== COMPARISON WITH PREVIOUS SWEEP ===\n');
sweepPath = 'results/20251010_211447_SWEEP_STAGEB_quick_test/sweep_results.mat';
if exist(sweepPath, 'file')
    sweep = load(sweepPath);
    baseline = sweep.results(1);
    
    fprintf('[Stage B - RS Smoothing]\n');
    fprintf('  Before: %d/%d accepted (%.1f%%)\n', ...
        baseline.diagnostics.stageB.rsImprovements, ...
        baseline.diagnostics.stageB.rsIterations, ...
        100*baseline.diagnostics.stageB.rsAcceptanceRate);
    fprintf('  After:  %d/%d accepted (%.1f%%)', ...
        stageBDiag.rsImprovements, stageBDiag.rsIterations, ...
        100*stageBDiag.rsAcceptanceRate);
    if stageBDiag.rsAcceptanceRate > baseline.diagnostics.stageB.rsAcceptanceRate
        fprintf(' ✓ IMPROVED\n');
    else
        fprintf('\n');
    end
    
    fprintf('\n[Stage C - Solver Convergence]\n');
    fprintf('  Before: %.1f ± %.1f iterations\n', ...
        baseline.diagnostics.stageC.solverIterationsMean, ...
        baseline.diagnostics.stageC.solverIterationsStd);
    fprintf('  After:  %.1f ± %.1f iterations', ...
        stageCDiag.solverIterationsMean, stageCDiag.solverIterationsStd);
    if stageCDiag.solverIterationsStd > 0
        fprintf(' ✓ CONVERGING\n');
    else
        fprintf('\n');
    end
    
    fprintf('\n[Stage C - Base Tracking]\n');
    fprintf('  Before: %.3fm mean deviation\n', ...
        baseline.diagnostics.stageC.basePosDeviationMean);
    fprintf('  After:  %.3fm mean deviation', ...
        stageCDiag.basePosDeviationMean);
    improvement = (1 - stageCDiag.basePosDeviationMean / baseline.diagnostics.stageC.basePosDeviationMean) * 100;
    if improvement > 10
        fprintf(' ✓ %.1f%% BETTER\n', improvement);
    elseif improvement > 0
        fprintf(' (%.1f%% better)\n', improvement);
    else
        fprintf('\n');
    end
end

% ===========================================================================
% GENERATE ANIMATION
% ===========================================================================
fprintf('\n=== Generating Animation ===\n');
animFile = fullfile(result.resultsDir, 'ISSUE_FIXES_animation.mp4');
fprintf('Creating: %s\n', animFile);

try
    gik9dof.animateStagedWithHelper(log, ...
        'ExportVideo', animFile, ...
        'FrameRate', 30, ...
        'SampleStep', 1);
    close all;
    fprintf('✓ Animation complete\n');
catch ME
    fprintf('⚠ Animation error: %s\n', ME.message);
    close all;
end

% ===========================================================================
% SUMMARY & RECOMMENDATIONS
% ===========================================================================
fprintf('\n=== SUMMARY ===\n');

issuesFixed = 0;
issuesRemain = 0;

fprintf('\nIssue #1 (Base Tracking): ');
if stageCDiag.basePosDeviationMean < 0.5
    fprintf('✓ FIXED (%.3fm < 0.5m target)\n', stageCDiag.basePosDeviationMean);
    issuesFixed = issuesFixed + 1;
else
    fprintf('⚠ PARTIAL (%.3fm, target <0.5m)\n', stageCDiag.basePosDeviationMean);
    issuesRemain = issuesRemain + 1;
end

fprintf('Issue #2 (Solver Convergence): ');
if stageCDiag.solverIterationsStd > 10
    fprintf('✓ FIXED (std=%.1f > 10)\n', stageCDiag.solverIterationsStd);
    issuesFixed = issuesFixed + 1;
elseif stageCDiag.solverIterationsMean < 280
    fprintf('✓ IMPROVED (mean=%.1f < 280)\n', stageCDiag.solverIterationsMean);
    issuesFixed = issuesFixed + 1;
else
    fprintf('⚠ PARTIAL (still near limit)\n');
    issuesRemain = issuesRemain + 1;
end

fprintf('Issue #3 (Stage B Cusps): ');
if stageBDiag.rsAcceptanceRate > 0.05 && stageBDiag.cuspCount == 0
    fprintf('✓ FIXED (RS working, 0 cusps)\n');
    issuesFixed = issuesFixed + 1;
elseif stageBDiag.rsAcceptanceRate > 0
    fprintf('↗ BETTER (%.1f%% acceptance)\n', 100*stageBDiag.rsAcceptanceRate);
    issuesRemain = issuesRemain + 1;
else
    fprintf('✗ UNRESOLVED (RS still failing)\n');
    issuesRemain = issuesRemain + 1;
end

fprintf('\n%d / 3 issues fixed\n', issuesFixed);

if issuesRemain == 0
    fprintf('\n🎉 ALL ISSUES RESOLVED!\n');
    fprintf('Recommended: Deploy these parameters to production.\n');
elseif issuesFixed >= 2
    fprintf('\n✓ Significant improvement!\n');
    fprintf('Recommended: Review animation and consider further tuning.\n');
else
    fprintf('\n⚠ Limited improvement.\n');
    fprintf('Recommended: Try more aggressive fixes (see ISSUE_ANALYSIS_AND_FIXES.md)\n');
end

fprintf('\nView animation:\n');
fprintf('  open %s\n', animFile);

fprintf('\n=== Test Complete ===\n');
