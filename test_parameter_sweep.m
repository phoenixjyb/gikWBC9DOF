%TEST_PARAMETER_SWEEP Quick validation of Phase 3 parameter sweep framework.
%   This script runs a small parameter sweep (Stage B only) to validate the
%   sweep infrastructure works correctly. Uses reduced grid to keep runtime
%   under 10 minutes.
%
%   All results saved with SWEEP_ prefix for easy identification.

clear; clc;
fprintf('=== Testing Parameter Sweep Framework ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% Add paths
scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(scriptDir, "matlab")));

%% Quick Stage B sweep (reduced grid)
fprintf('Running Stage B parameter sweep (reduced grid)...\n');
fprintf('This will test 2x2x2 = 8 configurations\n');
fprintf('Estimated time: ~3-5 minutes\n\n');

tic;
results = run_parameter_sweep( ...
    'SweepMode', 'stageb', ...
    'ResultsLabel', 'quick_test', ...
    'SafetyMargins', [0.10, 0.15], ...
    'LambdaCusps', [1.0, 2.0], ...
    'MaxIters', [200], ...
    'AllowReverse', [true], ...
    'ClothoidDiscretizations', [0.08], ...
    'SaveResults', true);
elapsedTime = toc;

fprintf('\n✓ Sweep complete in %.1f minutes\n\n', elapsedTime/60);

%% Analyze results
fprintf('=== Quick Analysis ===\n');

scores = [results.score];
validResults = results(scores > 0);

if isempty(validResults)
    fprintf('⚠ No valid results!\n');
    return;
end

fprintf('Valid configurations: %d / %d\n', length(validResults), length(results));
fprintf('Score range: %.3f - %.3f\n', min(scores(scores > 0)), max(scores(scores > 0)));

% Best configuration
[bestScore, bestIdx] = max(scores);
fprintf('\n[Best Configuration] Score: %.4f\n', bestScore);
fprintf('  Safety Margin: %.3f m\n', results(bestIdx).config.safetyMargin);
fprintf('  Lambda Cusp: %.1f\n', results(bestIdx).config.lambdaCusp);
fprintf('  Max Iters: %d\n', results(bestIdx).config.maxIters);

if isfield(results(bestIdx), 'metrics') && ~isempty(results(bestIdx).metrics)
    m = results(bestIdx).metrics;
    fprintf('  Metrics:\n');
    fprintf('    EE Mean Error: %.4f m\n', m.eeErrorMean);
    fprintf('    EE Max Error: %.4f m\n', m.eeErrorMax);
    fprintf('    Path Smoothness: %.3f\n', m.pathSmoothness);
    fprintf('    Cusp Count: %d\n', m.cuspCount);
end

% Parameter sensitivity
fprintf('\n=== Parameter Sensitivity ===\n');

% Group by safety margin
margins = unique(arrayfun(@(r) r.config.safetyMargin, validResults));
fprintf('\nSafety Margin:\n');
for margin = margins
    idx = arrayfun(@(r) r.config.safetyMargin == margin, validResults);
    avgScore = mean(arrayfun(@(r) r.score, validResults(idx)));
    fprintf('  %.2f m: avg score %.3f\n', margin, avgScore);
end

% Group by lambda cusp
lambdas = unique(arrayfun(@(r) r.config.lambdaCusp, validResults));
fprintf('\nLambda Cusp:\n');
for lambda = lambdas
    idx = arrayfun(@(r) r.config.lambdaCusp == lambda, validResults);
    avgScore = mean(arrayfun(@(r) r.score, validResults(idx)));
    fprintf('  %.1f: avg score %.3f\n', lambda, avgScore);
end

%% Recommendations
fprintf('\n=== Recommendations ===\n');

% Find best lambda
lambdaScores = zeros(size(lambdas));
for i = 1:length(lambdas)
    idx = arrayfun(@(r) r.config.lambdaCusp == lambdas(i), validResults);
    lambdaScores(i) = mean(arrayfun(@(r) r.score, validResults(idx)));
end
[~, bestLambdaIdx] = max(lambdaScores);
fprintf('Best lambda cusp: %.1f (avg score %.3f)\n', lambdas(bestLambdaIdx), lambdaScores(bestLambdaIdx));

% Find best margin
marginScores = zeros(size(margins));
for i = 1:length(margins)
    idx = arrayfun(@(r) r.config.safetyMargin == margins(i), validResults);
    marginScores(i) = mean(arrayfun(@(r) r.score, validResults(idx)));
end
[~, bestMarginIdx] = max(marginScores);
fprintf('Best safety margin: %.2f m (avg score %.3f)\n', margins(bestMarginIdx), marginScores(bestMarginIdx));

% Check if current tuned params are optimal
tunedMargin = 0.10;
tunedLambda = 1.0;
tunedIdx = find(arrayfun(@(r) r.config.safetyMargin == tunedMargin && ...
                              r.config.lambdaCusp == tunedLambda, validResults), 1);

if ~isempty(tunedIdx)
    tunedScore = validResults(tunedIdx).score;
    fprintf('\nCurrent tuned params score: %.3f (rank %d / %d)\n', ...
        tunedScore, sum(scores > tunedScore) + 1, length(validResults));
    
    if tunedScore >= bestScore * 0.95
        fprintf('✓ Tuned parameters are near-optimal!\n');
    else
        fprintf('⚠ Consider switching to best configuration for %.1f%% improvement\n', ...
            (bestScore - tunedScore) / tunedScore * 100);
    end
else
    fprintf('\n⚠ Tuned params not tested in this sweep\n');
end

fprintf('\n=== Phase 3 Quick Test Complete ===\n');
fprintf('For comprehensive sweep, run:\n');
fprintf('  results = run_parameter_sweep(''SweepMode'', ''stageb'');\n');
fprintf('This will test 4x4x3x2x3 = 288 configurations (~10 hours)\n');
