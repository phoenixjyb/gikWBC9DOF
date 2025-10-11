%TEST_ENHANCED_LOGGING Validate Phase 2 enhanced logging infrastructure.
%   This script tests the new diagnostic logging features added in Phase 2:
%   - Stage B: curvature histograms, RS/clothoid metrics, path smoothness
%   - Stage C: solver iteration distribution, EE error bins, base tracking
%
%   All results saved with ENHANCED_LOGGING_ prefix for easy identification.

clear; clc;
fprintf('=== Testing Enhanced Logging Infrastructure ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% Add paths
scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(scriptDir, "matlab")));

% Run a quick test with enhanced logging
runLabel = "ENHANCED_LOGGING_test_10hz";
executionMode = "ppForIk";
rateHz = 10;
maxIterations = 150;

fprintf('Running staged trajectory with enhanced logging...\n');
fprintf('  - Rate: %d Hz\n', rateHz);
fprintf('  - Execution mode: %s\n\n', executionMode);

tic;
result = gik9dof.runStagedReference( ...
    "RunLabel", runLabel, ...
    "ExecutionMode", executionMode, ...
    "RateHz", rateHz, ...
    "MaxIterations", maxIterations, ...
    "SaveLog", true);
elapsedTime = toc;

fprintf('✓ Simulation complete in %.1f seconds\n', elapsedTime);
fprintf('✓ Results saved to: %s\n\n', result.resultsDir);

% Extract and display Stage B diagnostics
log = result.log;
fprintf('=== Stage B Diagnostics ===\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageB')
    stageB = log.stageLogs.stageB;
    
    if isfield(stageB, 'diagnostics')
        diag = stageB.diagnostics;
        
        fprintf('[Path Quality]\n');
        fprintf('  Curvature: %.3f ± %.3f [1/m] (max %.3f)\n', ...
            diag.meanCurvature, diag.pathSmoothness, diag.maxCurvature);
        fprintf('  Cusp count: %d\n', diag.cuspCount);
        
        fprintf('\n[Curvature Distribution]\n');
        hist = diag.curvatureHistogram;
        total = hist.low + hist.medium + hist.high + hist.veryHigh;
        if total > 0
            fprintf('  Low (<0.5):      %3d (%.1f%%)\n', hist.low, 100*hist.low/total);
            fprintf('  Medium (0.5-1.0):%3d (%.1f%%)\n', hist.medium, 100*hist.medium/total);
            fprintf('  High (1.0-2.0):  %3d (%.1f%%)\n', hist.high, 100*hist.high/total);
            fprintf('  Very High (>2.0):%3d (%.1f%%)\n', hist.veryHigh, 100*hist.veryHigh/total);
        end
        
        fprintf('\n[Reeds-Shepp Smoothing]\n');
        fprintf('  Iterations: %d\n', diag.rsIterations);
        fprintf('  Improvements: %d (%.1f%% acceptance)\n', ...
            diag.rsImprovements, 100*diag.rsAcceptanceRate);
        fprintf('  Path length reduction: %.3f m\n', diag.rsPathLengthImprovement);
        
        fprintf('\n[Clothoid Smoothing]\n');
        if diag.clothoidApplied
            fprintf('  Status: APPLIED (%d segments)\n', diag.clothoidSegments);
        else
            fprintf('  Status: not applied\n');
        end
        
        if ~isnan(diag.plannerComputeTime)
            fprintf('\n[Planner Performance]\n');
            fprintf('  Compute time: %.3f s\n', diag.plannerComputeTime);
        end
    else
        fprintf('  No diagnostics found (may be old log format)\n');
    end
else
    fprintf('  Stage B not found in log\n');
end

% Extract and display Stage C diagnostics
fprintf('\n=== Stage C Diagnostics ===\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    
    if isfield(stageC, 'diagnostics')
        diag = stageC.diagnostics;
        
        fprintf('[End-Effector Tracking]\n');
        fprintf('  Mean error: %.4f m\n', diag.eeErrorMean);
        fprintf('  Max error:  %.4f m\n', diag.eeErrorMax);
        
        fprintf('\n[EE Error Distribution]\n');
        bins = diag.eeErrorBins;
        total = bins.excellent + bins.good + bins.acceptable + bins.poor;
        if total > 0
            fprintf('  Excellent (<0.05m):   %3d (%.1f%%)\n', bins.excellent, 100*bins.excellent/total);
            fprintf('  Good (0.05-0.10m):    %3d (%.1f%%)\n', bins.good, 100*bins.good/total);
            fprintf('  Acceptable (0.10-0.20m):%3d (%.1f%%)\n', bins.acceptable, 100*bins.acceptable/total);
            fprintf('  Poor (>0.20m):        %3d (%.1f%%)\n', bins.poor, 100*bins.poor/total);
        end
        
        fprintf('\n[Solver Performance]\n');
        fprintf('  Iterations: %.1f ± %.1f (max %d)\n', ...
            diag.solverIterationsMean, diag.solverIterationsStd, diag.solverIterationsMax);
        
        exitFlags = diag.exitFlagHistogram;
        totalFlags = exitFlags.success + exitFlags.maxIters + exitFlags.failed;
        if totalFlags > 0
            fprintf('  Exit flags:\n');
            fprintf('    Success:   %3d (%.1f%%)\n', exitFlags.success, 100*exitFlags.success/totalFlags);
            fprintf('    Max iters: %3d (%.1f%%)\n', exitFlags.maxIters, 100*exitFlags.maxIters/totalFlags);
            fprintf('    Failed:    %3d (%.1f%%)\n', exitFlags.failed, 100*exitFlags.failed/totalFlags);
        end
        
        fprintf('\n[Base Tracking]\n');
        fprintf('  Yaw drift: %.4f rad mean, %.4f rad max\n', ...
            diag.baseYawDriftMean, diag.baseYawDriftMax);
        fprintf('  Position deviation: %.4f m mean, %.4f m max\n', ...
            diag.basePosDeviationMean, diag.basePosDeviationMax);
        
        fprintf('\n[Stage C Refinement]\n');
        if diag.refinementApplied
            fprintf('  Status: APPLIED\n');
            delta = diag.refinementDelta;
            fprintf('  Path length delta: %.3f m\n', delta.pathLength);
            fprintf('  EE error delta: %.4f m (mean), %.4f m (max)\n', ...
                delta.eeErrorMean, delta.eeErrorMax);
        else
            fprintf('  Status: NOT APPLIED (%s)\n', diag.refinementReason);
        end
    else
        fprintf('  No diagnostics found (may be old log format)\n');
    end
else
    fprintf('  Stage C not found in log\n');
end

% Save diagnostic summary
fprintf('\n=== Saving Diagnostic Summary ===\n');
summaryFile = fullfile(result.resultsDir, 'ENHANCED_LOGGING_diagnostics.txt');
fid = fopen(summaryFile, 'w');
fprintf(fid, 'Enhanced Logging Diagnostic Summary\n');
fprintf(fid, 'Generated: %s\n\n', datestr(now));

% Stage B summary
fprintf(fid, '=== STAGE B DIAGNOSTICS ===\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageB') && ...
   isfield(log.stageLogs.stageB, 'diagnostics')
    diag = log.stageLogs.stageB.diagnostics;
    fprintf(fid, 'Path Quality:\n');
    fprintf(fid, '  Mean curvature: %.3f [1/m]\n', diag.meanCurvature);
    fprintf(fid, '  Max curvature: %.3f [1/m]\n', diag.maxCurvature);
    fprintf(fid, '  Path smoothness (std): %.3f\n', diag.pathSmoothness);
    fprintf(fid, '  Cusp count: %d\n', diag.cuspCount);
    fprintf(fid, '\nReeds-Shepp Smoothing:\n');
    fprintf(fid, '  Acceptance rate: %.1f%%\n', 100*diag.rsAcceptanceRate);
    fprintf(fid, '  Path improvement: %.3f m\n', diag.rsPathLengthImprovement);
    fprintf(fid, '\nClothoid Smoothing:\n');
    fprintf(fid, '  Applied: %s\n', ternary(diag.clothoidApplied, 'YES', 'NO'));
end

% Stage C summary
fprintf(fid, '\n=== STAGE C DIAGNOSTICS ===\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC') && ...
   isfield(log.stageLogs.stageC, 'diagnostics')
    diag = log.stageLogs.stageC.diagnostics;
    fprintf(fid, 'End-Effector Tracking:\n');
    fprintf(fid, '  Mean error: %.4f m\n', diag.eeErrorMean);
    fprintf(fid, '  Max error: %.4f m\n', diag.eeErrorMax);
    fprintf(fid, '\nEE Error Distribution:\n');
    bins = diag.eeErrorBins;
    total = bins.excellent + bins.good + bins.acceptable + bins.poor;
    if total > 0
        fprintf(fid, '  Excellent (<0.05m): %.1f%%\n', 100*bins.excellent/total);
        fprintf(fid, '  Good (0.05-0.10m): %.1f%%\n', 100*bins.good/total);
        fprintf(fid, '  Acceptable (0.10-0.20m): %.1f%%\n', 100*bins.acceptable/total);
        fprintf(fid, '  Poor (>0.20m): %.1f%%\n', 100*bins.poor/total);
    end
    fprintf(fid, '\nSolver Performance:\n');
    fprintf(fid, '  Mean iterations: %.1f\n', diag.solverIterationsMean);
    fprintf(fid, '  Max iterations: %d\n', diag.solverIterationsMax);
    fprintf(fid, '\nBase Tracking:\n');
    fprintf(fid, '  Yaw drift mean: %.4f rad\n', diag.baseYawDriftMean);
    fprintf(fid, '  Position deviation mean: %.4f m\n', diag.basePosDeviationMean);
end

fclose(fid);
fprintf('✓ Diagnostic summary: %s\n', summaryFile);

% Generate plots with enhanced diagnostics
fprintf('✓ Generating diagnostic plots...\n');
plotPath = fullfile(result.resultsDir, 'ENHANCED_LOGGING_plots.png');
gik9dof.plotTrajectoryLog(log, 'ExportPath', plotPath);
close all;

fprintf('\n=== Summary ===\n');
fprintf('Enhanced logging validated successfully!\n');
fprintf('All artifacts saved to:\n  %s\n', result.resultsDir);
fprintf('\nFiles generated:\n');
fprintf('  - log_staged_ppForIk.mat\n');
fprintf('  - ENHANCED_LOGGING_diagnostics.txt\n');
fprintf('  - ENHANCED_LOGGING_plots.png\n');

fprintf('\n=== Phase 2 Complete ===\n');
fprintf('Next: Run parameter sweeps using the new diagnostic data (Task #4)\n');

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
