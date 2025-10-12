%TEST_TUNED_PARAMETERS Validate parameter improvements from tuning session.
%   This script tests the updated parameters:
%   - Relaxed safety margins (0.15 -> 0.1m)
%   - Tuned RS params (lambdaCusp 3.0 -> 1.0, allowReverse true, iters 600 -> 200)
%   - Clothoid smoothing (discretization 0.05 -> 0.08m)
%   - Stage C refinement disabled by default
%   - Softened pure pursuit (lookahead 0.6 -> 0.8m, accel 1.2 -> 0.8 m/s²)

clear; clc;
fprintf('=== Testing Tuned Parameters ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% Add paths
scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(scriptDir, "matlab")));

% Configuration - clearly labeled as TUNED parameters
runLabel = "TUNED_params_10hz_validation";
executionMode = "ppForIk";            
rateHz = 10;                          % Control rate
maxIterations = 150;                  
useStageBHybridAStar = true;
stageBMode = "pureHyb";

% NEW TUNED PARAMETERS (applied in codebase)
% - StageBHybridSafetyMargin: 0.15 -> 0.1m
% - DistanceBounds: 0.2 -> 0.1m
% - lambdaCusp: 3.0 -> 1.0
% - allowReverse: false -> true
% - iters: 600 -> 200
% - clothoidDiscretization: 0.05 -> 0.08m
% - StageCUseBaseRefinement: true -> false
% - lookahead_base: 0.6 -> 0.8m
% - accel_limit: 1.2 -> 0.8 m/s²
% - StageCLookaheadDistance: 0.4 -> 0.8m

fprintf('Running staged trajectory with TUNED parameters...\n');
fprintf('  - Rate: %d Hz\n', rateHz);
fprintf('  - Stage B mode: %s\n', stageBMode);
fprintf('  - Execution mode: %s\n\n', executionMode);

tic;
result = gik9dof.runStagedReference( ...
    "RunLabel", runLabel, ...
    "ExecutionMode", executionMode, ...
    "RateHz", rateHz, ...
    "MaxIterations", maxIterations, ...
    "UseStageBHybridAStar", useStageBHybridAStar, ...
    "StageBMode", stageBMode, ...
    "SaveLog", true);
elapsedTime = toc;

fprintf('✓ Simulation complete in %.1f seconds\n', elapsedTime);
fprintf('✓ Results saved to: %s\n\n', result.resultsDir);

% Evaluate metrics
fprintf('=== Performance Metrics ===\n');
log = result.log;
report = gik9dof.evaluateLog(log, 'Verbose', true);

% Extract stage-specific metrics
fprintf('\n=== Stage-Specific Analysis ===\n');

% Stage B metrics
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageB')
    stageB = log.stageLogs.stageB;
    fprintf('\n[Stage B - Base Alignment]\n');
    
    if isfield(stageB, 'goalBase') && isfield(stageB, 'achievedBase')
        posErr = norm(stageB.achievedBase(1:2) - stageB.goalBase(1:2));
        yawErr = rad2deg(abs(wrapToPi(stageB.achievedBase(3) - stageB.goalBase(3))));
        fprintf('  Docking error: %.4f m position, %.2f° yaw\n', posErr, yawErr);
    end
    
    if isfield(stageB, 'planner') && isfield(stageB.planner, 'rsSmoothing')
        rsInfo = stageB.planner.rsSmoothing;
        if isfield(rsInfo, 'improvements') && isfield(rsInfo, 'iterationsExecuted')
            fprintf('  RS shortcuts: %d accepted / %d attempted\n', ...
                double(rsInfo.improvements), double(rsInfo.iterationsExecuted));
        end
        if isfield(rsInfo, 'initialLength') && isfield(rsInfo, 'finalLength')
            pathImprovement = rsInfo.initialLength - rsInfo.finalLength;
            fprintf('  Path length: %.2f m -> %.2f m (Δ%.2f m)\n', ...
                rsInfo.initialLength, rsInfo.finalLength, pathImprovement);
        end
    end
    
    if isfield(stageB, 'planner') && isfield(stageB.planner, 'hcSmoothing')
        hcInfo = stageB.planner.hcSmoothing;
        if isfield(hcInfo, 'applied') && hcInfo.applied
            fprintf('  Clothoid smoothing: APPLIED (%d segments fitted)\n', ...
                double(hcInfo.fittedSegments));
        else
            fprintf('  Clothoid smoothing: not applied\n');
        end
    end
end

% Stage C metrics
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    fprintf('\n[Stage C - Full Tracking]\n');
    
    % Compare reference vs executed base
    if isfield(stageC, 'referenceBaseStates') && isfield(stageC, 'execBaseStates')
        refBase = stageC.referenceBaseStates;
        execBase = stageC.execBaseStates;
        if size(refBase,1) == size(execBase,1)
            posDeltas = vecnorm(execBase(:,1:2) - refBase(:,1:2), 2, 2);
            fprintf('  Base tracking: %.4f ± %.4f m deviation\n', ...
                mean(posDeltas), std(posDeltas));
        end
    end
    
    % EE tracking against reference
    if isfield(stageC, 'referenceInitialIk') && isfield(stageC.referenceInitialIk, 'eePositions')
        refEE = stageC.referenceInitialIk.eePositions;
        execEE = stageC.eePositions;
        if size(refEE, 2) == size(execEE, 2)
            eeDeltas = vecnorm(execEE - refEE, 2, 1);
            fprintf('  EE ref vs exec: %.4f m mean, %.4f m max\n', ...
                mean(eeDeltas), max(eeDeltas));
        end
    end
    
    % Pure pursuit status
    if isfield(stageC, 'purePursuit') && isfield(stageC.purePursuit, 'refinement')
        refInfo = stageC.purePursuit.refinement;
        if isfield(refInfo, 'applied')
            if refInfo.applied
                fprintf('  Stage C refinement: APPLIED\n');
            else
                fprintf('  Stage C refinement: DISABLED (as configured)\n');
            end
        end
    end
end

% Compare against baseline
fprintf('\n=== Comparison with Baseline ===\n');
baselinePath = 'results/20251010_155837_staged_10hz_legends3/log_staged_ppForIk.mat';
if exist(baselinePath, 'file')
    baseline = load(baselinePath);
    baselineLog = baseline.log;
    baselineReport = gik9dof.evaluateLog(baselineLog, 'Verbose', false);
    
    fprintf('Baseline (20251010_155837):\n');
    fprintf('  Mean EE error: %.4f m\n', baselineReport.positionMean);
    fprintf('  Max EE error:  %.4f m\n', baselineReport.positionMax);
    
    fprintf('\nTuned (current run):\n');
    fprintf('  Mean EE error: %.4f m', report.positionMean);
    if report.positionMean < baselineReport.positionMean
        improvementPct = (1 - report.positionMean/baselineReport.positionMean) * 100;
        fprintf(' (↓ %.1f%% improvement!)\n', improvementPct);
    else
        degradationPct = (report.positionMean/baselineReport.positionMean - 1) * 100;
        fprintf(' (↑ %.1f%% worse)\n', degradationPct);
    end
    
    fprintf('  Max EE error:  %.4f m', report.positionMax);
    if report.positionMax < baselineReport.positionMax
        improvementPct = (1 - report.positionMax/baselineReport.positionMax) * 100;
        fprintf(' (↓ %.1f%% improvement!)\n', improvementPct);
    else
        degradationPct = (report.positionMax/baselineReport.positionMax - 1) * 100;
        fprintf(' (↑ %.1f%% worse)\n', degradationPct);
    end
    
    % Target assessment
    fprintf('\nTarget Achievement:\n');
    meanTarget = 0.10;
    maxTarget = 0.20;
    fprintf('  Mean EE error target (<%.2f m): %s\n', meanTarget, ...
        ternary(report.positionMean < meanTarget, '✓ PASS', '✗ FAIL'));
    fprintf('  Max EE error target (<%.2f m):  %s\n', maxTarget, ...
        ternary(report.positionMax < maxTarget, '✓ PASS', '✗ FAIL'));
else
    fprintf('Baseline file not found at: %s\n', baselinePath);
    fprintf('Skipping comparison.\n');
end

% Generate artifacts
fprintf('\n=== Generating Artifacts ===\n');

% Save detailed comparison report
comparisonFile = fullfile(result.resultsDir, 'TUNED_vs_BASELINE_comparison.txt');
fid = fopen(comparisonFile, 'w');
fprintf(fid, 'Parameter Tuning Validation Report\n');
fprintf(fid, 'Generated: %s\n\n', datestr(now));
fprintf(fid, '=== TUNED PARAMETERS ===\n');
fprintf(fid, 'StageBHybridSafetyMargin: 0.15 -> 0.10 m\n');
fprintf(fid, 'DistanceBounds lower: 0.2 -> 0.1 m\n');
fprintf(fid, 'RS lambdaCusp: 3.0 -> 1.0\n');
fprintf(fid, 'RS allowReverse: false -> true\n');
fprintf(fid, 'RS iters: 600 -> 200\n');
fprintf(fid, 'Clothoid discretization: 0.05 -> 0.08 m\n');
fprintf(fid, 'StageCUseBaseRefinement: true -> false\n');
fprintf(fid, 'lookahead_base: 0.6 -> 0.8 m\n');
fprintf(fid, 'accel_limit: 1.2 -> 0.8 m/s²\n');
fprintf(fid, 'StageCLookaheadDistance: 0.4 -> 0.8 m\n\n');
fprintf(fid, '=== RESULTS ===\n');
fprintf(fid, 'Mean EE error: %.4f m\n', report.positionMean);
fprintf(fid, 'Max EE error: %.4f m\n', report.positionMax);
fprintf(fid, 'Solver iters (mean/max): %.1f / %.1f\n', ...
    report.solverIterationsMean, report.solverIterationsMax);
fclose(fid);
fprintf('✓ Comparison report: %s\n', comparisonFile);

% Generate plots
fprintf('✓ Generating tracking plots...\n');
plotPath = fullfile(result.resultsDir, 'TUNED_tracking_plots.png');
gik9dof.plotTrajectoryLog(log, 'ExportPath', plotPath);
close all;

% Generate animation
fprintf('✓ Generating animation (this may take a minute)...\n');
videoPath = fullfile(result.resultsDir, 'TUNED_whole_body_animation.mp4');
gik9dof.animateStagedWithHelper(log, ...
    'ExportVideo', videoPath, ...
    'FrameRate', 30, ...
    'SampleStep', 1);
close all;

% Save parameter snapshot
paramFile = fullfile(result.resultsDir, 'TUNED_parameters.json');
params = struct();
params.stageBHybridSafetyMargin = 0.10;
params.distanceBoundsLower = 0.10;
params.rsLambdaCusp = 1.0;
params.rsAllowReverse = true;
params.rsIters = 200;
params.clothoidDiscretization = 0.08;
params.stageCUseBaseRefinement = false;
params.lookaheadBase = 0.80;
params.accelLimit = 0.80;
params.stageCLookaheadDistance = 0.80;
params.rateHz = rateHz;
params.executionMode = char(executionMode);
params.stageBMode = char(stageBMode);
fid = fopen(paramFile, 'w');
fprintf(fid, '%s', jsonencode(params, 'PrettyPrint', true));
fclose(fid);

fprintf('\n=== Summary ===\n');
fprintf('All artifacts saved to:\n  %s\n', result.resultsDir);
fprintf('\nFiles generated:\n');
fprintf('  - log_staged_ppForIk.mat              (full trajectory log)\n');
fprintf('  - TUNED_vs_BASELINE_comparison.txt    (text report)\n');
fprintf('  - TUNED_tracking_plots.png            (error plots)\n');
fprintf('  - TUNED_whole_body_animation.mp4      (visualization)\n');
fprintf('  - TUNED_parameters.json               (parameter snapshot)\n');

fprintf('\n=== Next Steps ===\n');
if report.positionMean < 0.10 && report.positionMax < 0.20
    fprintf('✓ Targets achieved! Parameters are performing well.\n');
    fprintf('  Consider: Enhanced logging implementation (Task #2)\n');
else
    fprintf('⚠ Targets not yet achieved. Recommendations:\n');
    if report.positionMean >= 0.10
        fprintf('  - Run parameter sweep to optimize further\n');
    end
    if report.positionMax >= 0.20
        fprintf('  - Check animation for problematic segments\n');
        fprintf('  - Consider tighter Stage C lookahead tuning\n');
    end
end

fprintf('\n=== Test Complete ===\n');

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
