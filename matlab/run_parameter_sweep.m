function results = run_parameter_sweep(varargin)
%RUN_PARAMETER_SWEEP Automated grid search over planning and control parameters.
%   results = run_parameter_sweep() runs a comprehensive parameter sweep over
%   Stage B planning parameters (Hybrid A*, Reeds-Shepp, Clothoid) and Stage C
%   control parameters (pure pursuit) to find optimal configurations.
%
%   results = run_parameter_sweep(Name, Value) specifies options:
%
%   Sweep Configuration:
%       'SweepMode'             - 'stageb' (default), 'stagec', or 'full'
%       'ParallelExecution'     - Enable parallel execution (default false)
%       'SaveResults'           - Save results to disk (default true)
%       'ResultsLabel'          - Custom label for results folder
%
%   Stage B Parameters (grid search):
%       'SafetyMargins'         - Vector of safety margins [m] (default [0.05, 0.10, 0.15, 0.20])
%       'LambdaCusps'           - Vector of RS cusp penalties (default [0.5, 1.0, 2.0, 3.0])
%       'MaxIters'              - Vector of RS iterations (default [100, 200, 400])
%       'AllowReverse'          - Vector of reverse flags (default [true, false])
%       'ClothoidDiscretizations' - Vector of clothoid steps [m] (default [0.05, 0.08, 0.10])
%
%   Stage C Parameters (grid search):
%       'LookaheadDistances'    - Vector of lookahead [m] (default [0.4, 0.6, 0.8, 1.0])
%       'AccelLimits'           - Vector of accel limits [m/s²] (default [0.6, 0.8, 1.0, 1.2])
%       'HeadingKps'            - Vector of heading gains (default [0.8, 1.0, 1.2, 1.5])
%
%   Scoring Configuration:
%       'WeightEEMean'          - Weight for mean EE error (default 0.35)
%       'WeightEEMax'           - Weight for max EE error (default 0.25)
%       'WeightSmoothness'      - Weight for path smoothness (default 0.20)
%       'WeightCusps'           - Weight for cusp count (default 0.10)
%       'WeightComputeTime'     - Weight for compute time (default 0.10)
%
%   Output:
%       results - Struct array with fields for each configuration:
%           .config         - Parameter configuration tested
%           .log            - Full trajectory log
%           .diagnostics    - Stage B and C diagnostics
%           .score          - Composite score (0-1, higher is better)
%           .metrics        - Individual metric scores
%
%   Example:
%       % Quick sweep over RS parameters only
%       results = run_parameter_sweep('SweepMode', 'stageb', ...
%           'LambdaCusps', [0.5, 1.0, 2.0], ...
%           'MaxIters', [100, 200, 400]);
%       
%       % Find best configuration
%       [~, bestIdx] = max([results.score]);
%       fprintf('Best config: lambdaCusp=%.1f, maxIters=%d, score=%.3f\n', ...
%           results(bestIdx).config.lambdaCusp, ...
%           results(bestIdx).config.maxIters, ...
%           results(bestIdx).score);

%% Parse inputs
p = inputParser;
addParameter(p, 'SweepMode', 'stageb', @(x) ismember(x, {'stageb', 'stagec', 'full'}));
addParameter(p, 'ParallelExecution', false, @islogical);
addParameter(p, 'SaveResults', true, @islogical);
addParameter(p, 'ResultsLabel', 'param_sweep', @ischar);

% Stage B parameters
addParameter(p, 'SafetyMargins', [0.05, 0.10, 0.15, 0.20], @isnumeric);
addParameter(p, 'LambdaCusps', [0.5, 1.0, 2.0, 3.0], @isnumeric);
addParameter(p, 'MaxIters', [100, 200, 400], @isnumeric);
addParameter(p, 'AllowReverse', [true, false], @islogical);
addParameter(p, 'ClothoidDiscretizations', [0.05, 0.08, 0.10], @isnumeric);

% Stage C parameters
addParameter(p, 'LookaheadDistances', [0.4, 0.6, 0.8, 1.0], @isnumeric);
addParameter(p, 'AccelLimits', [0.6, 0.8, 1.0, 1.2], @isnumeric);
addParameter(p, 'HeadingKps', [0.8, 1.0, 1.2, 1.5], @isnumeric);

% Scoring weights
addParameter(p, 'WeightEEMean', 0.35, @isnumeric);
addParameter(p, 'WeightEEMax', 0.25, @isnumeric);
addParameter(p, 'WeightSmoothness', 0.20, @isnumeric);
addParameter(p, 'WeightCusps', 0.10, @isnumeric);
addParameter(p, 'WeightComputeTime', 0.10, @isnumeric);

parse(p, varargin{:});
opts = p.Results;

%% Generate parameter grid
fprintf('=== Parameter Sweep Configuration ===\n');
fprintf('Mode: %s\n', opts.SweepMode);
fprintf('Parallel: %s\n', ternary(opts.ParallelExecution, 'enabled', 'disabled'));
fprintf('\n');

configs = generateParameterGrid(opts);
numConfigs = length(configs);

fprintf('Generated %d configurations to test\n', numConfigs);
fprintf('Estimated time: %.1f minutes (sequential)\n', numConfigs * 2.2 / 60);
fprintf('\n');

%% Run sweep
results = struct('config', {}, 'log', {}, 'diagnostics', {}, 'score', {}, 'metrics', {});

fprintf('Starting parameter sweep...\n');
fprintf('Progress: [');
progressInterval = max(floor(numConfigs / 50), 1);

for i = 1:numConfigs
    try
        % Run simulation with current config
        result = runSingleConfiguration(configs(i), opts);
        
        % Store result
        results(i).config = configs(i);
        results(i).log = result.log;
        results(i).diagnostics = extractDiagnostics(result.log);
        results(i).metrics = computeMetrics(result.log, opts);
        results(i).score = computeCompositeScore(results(i).metrics, opts);
        
        % Progress indicator
        if mod(i, progressInterval) == 0
            fprintf('.');
        end
        
    catch ME
        warning('Configuration %d failed: %s', i, ME.message);
        results(i).config = configs(i);
        results(i).log = [];
        results(i).diagnostics = struct();
        results(i).metrics = struct();
        results(i).score = 0;
    end
end

fprintf('] Complete!\n\n');

%% Save results
if opts.SaveResults
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    resultsDir = sprintf('results/%s_SWEEP_%s_%s', timestamp, upper(opts.SweepMode), opts.ResultsLabel);
    mkdir(resultsDir);
    
    % Save full results
    save(fullfile(resultsDir, 'sweep_results.mat'), 'results', 'opts', '-v7.3');
    
    % Generate summary report
    generateSummaryReport(results, opts, resultsDir);
    
    % Generate Pareto front plots
    generateParetoPlots(results, opts, resultsDir);
    
    fprintf('Results saved to: %s\n', resultsDir);
end

%% Display summary
fprintf('=== Sweep Summary ===\n');
scores = [results.score];
[bestScore, bestIdx] = max(scores);
[worstScore, worstIdx] = min(scores);

fprintf('Configurations tested: %d\n', numConfigs);
fprintf('Successful runs: %d\n', sum(scores > 0));
fprintf('Failed runs: %d\n', sum(scores == 0));
fprintf('\nBest configuration (score %.3f):\n', bestScore);
disp(results(bestIdx).config);
fprintf('\nWorst configuration (score %.3f):\n', worstScore);
disp(results(worstIdx).config);

fprintf('\nTop 5 configurations:\n');
[~, sortedIdx] = sort(scores, 'descend');
for i = 1:min(5, numConfigs)
    idx = sortedIdx(i);
    fprintf('  #%d (score %.3f): ', i, results(idx).score);
    printConfigSummary(results(idx).config);
end

end

%% Helper Functions

function configs = generateParameterGrid(opts)
%GENERATEPARAMETERGRID Create all parameter combinations to test.

configs = struct('safetyMargin', {}, 'lambdaCusp', {}, 'maxIters', {}, ...
                 'allowReverse', {}, 'clothoidDiscretization', {}, ...
                 'lookaheadDistance', {}, 'accelLimit', {}, 'headingKp', {});

switch opts.SweepMode
    case 'stageb'
        % Stage B only: Hybrid A* + RS + Clothoid
        idx = 1;
        for margin = opts.SafetyMargins
            for lambda = opts.LambdaCusps
                for iters = opts.MaxIters
                    for reverse = opts.AllowReverse
                        for clothoid = opts.ClothoidDiscretizations
                            configs(idx).safetyMargin = margin;
                            configs(idx).lambdaCusp = lambda;
                            configs(idx).maxIters = iters;
                            configs(idx).allowReverse = reverse;
                            configs(idx).clothoidDiscretization = clothoid;
                            configs(idx).lookaheadDistance = 0.8;  % Default
                            configs(idx).accelLimit = 0.8;          % Default
                            configs(idx).headingKp = 1.2;           % Default
                            idx = idx + 1;
                        end
                    end
                end
            end
        end
        
    case 'stagec'
        % Stage C only: Pure pursuit tuning
        idx = 1;
        for lookahead = opts.LookaheadDistances
            for accel = opts.AccelLimits
                for kp = opts.HeadingKps
                    configs(idx).safetyMargin = 0.10;       % Default
                    configs(idx).lambdaCusp = 1.0;          % Default
                    configs(idx).maxIters = 200;            % Default
                    configs(idx).allowReverse = true;       % Default
                    configs(idx).clothoidDiscretization = 0.08; % Default
                    configs(idx).lookaheadDistance = lookahead;
                    configs(idx).accelLimit = accel;
                    configs(idx).headingKp = kp;
                    idx = idx + 1;
                end
            end
        end
        
    case 'full'
        % Full sweep (warning: combinatorial explosion!)
        idx = 1;
        for margin = opts.SafetyMargins
            for lambda = opts.LambdaCusps
                for iters = opts.MaxIters
                    for reverse = opts.AllowReverse
                        for clothoid = opts.ClothoidDiscretizations
                            for lookahead = opts.LookaheadDistances
                                for accel = opts.AccelLimits
                                    for kp = opts.HeadingKps
                                        configs(idx).safetyMargin = margin;
                                        configs(idx).lambdaCusp = lambda;
                                        configs(idx).maxIters = iters;
                                        configs(idx).allowReverse = reverse;
                                        configs(idx).clothoidDiscretization = clothoid;
                                        configs(idx).lookaheadDistance = lookahead;
                                        configs(idx).accelLimit = accel;
                                        configs(idx).headingKp = kp;
                                        idx = idx + 1;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end

end

function result = runSingleConfiguration(config, ~)
%RUNSINGLECONFIGURATION Execute one parameter configuration.

% Prepare RS params
rsParams = gik9dof.control.defaultReedsSheppParams();
rsParams.lambdaCusp = config.lambdaCusp;
rsParams.iters = config.maxIters;
rsParams.allowReverse = config.allowReverse;

% Prepare chassis profile override
chassisOverride = struct();
chassisOverride.accel_limit = config.accelLimit;
chassisOverride.heading_kp = config.headingKp;
chassisOverride.lookahead_base = config.lookaheadDistance;

% Prepare clothoid params
clothoidParams = struct();
clothoidParams.discretizationDistance = config.clothoidDiscretization;

% Run simulation
result = gik9dof.runStagedReference( ...
    'RunLabel', sprintf('sweep_%.2f_%.1f_%d', config.safetyMargin, config.lambdaCusp, config.maxIters), ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 150, ...
    'StageBHybridSafetyMargin', config.safetyMargin, ...
    'StageBUseReedsShepp', true, ...
    'StageBReedsSheppParams', rsParams, ...
    'StageBUseClothoid', true, ...
    'StageBClothoidParams', clothoidParams, ...
    'ChassisOverrides', chassisOverride, ...
    'SaveLog', false);  % Don't save individual runs

end

function diag = extractDiagnostics(log)
%EXTRACTDIAGNOSTICS Pull diagnostics from log structure.

diag = struct();

if isfield(log, 'stageLogs')
    if isfield(log.stageLogs, 'stageB') && isfield(log.stageLogs.stageB, 'diagnostics')
        diag.stageB = log.stageLogs.stageB.diagnostics;
    end
    
    if isfield(log.stageLogs, 'stageC') && isfield(log.stageLogs.stageC, 'diagnostics')
        diag.stageC = log.stageLogs.stageC.diagnostics;
    end
end

end

function metrics = computeMetrics(log, ~)
%COMPUTEMETRICS Extract and normalize performance metrics.

metrics = struct();

% Extract Stage C EE tracking
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC') && ...
   isfield(log.stageLogs.stageC, 'diagnostics')
    diagC = log.stageLogs.stageC.diagnostics;
    metrics.eeErrorMean = diagC.eeErrorMean;
    metrics.eeErrorMax = diagC.eeErrorMax;
else
    metrics.eeErrorMean = NaN;
    metrics.eeErrorMax = NaN;
end

% Extract Stage B smoothness
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageB') && ...
   isfield(log.stageLogs.stageB, 'diagnostics')
    diagB = log.stageLogs.stageB.diagnostics;
    metrics.pathSmoothness = diagB.pathSmoothness;
    metrics.cuspCount = diagB.cuspCount;
    metrics.computeTime = diagB.plannerComputeTime;
else
    metrics.pathSmoothness = NaN;
    metrics.cuspCount = NaN;
    metrics.computeTime = NaN;
end

% Normalize metrics (0-1 scale, higher is better)
metrics.eeErrorMeanScore = max(0, 1 - metrics.eeErrorMean / 0.10);  % Target <0.10m
metrics.eeErrorMaxScore = max(0, 1 - metrics.eeErrorMax / 0.20);    % Target <0.20m
metrics.smoothnessScore = max(0, 1 - metrics.pathSmoothness / 1.0); % Target <1.0
metrics.cuspScore = max(0, 1 - metrics.cuspCount / 5);              % Target <5 cusps
metrics.computeTimeScore = max(0, 1 - metrics.computeTime / 5.0);   % Target <5s

end

function score = computeCompositeScore(metrics, opts)
%COMPUTECOMPOSITESCORE Weighted sum of normalized metrics.

if any(isnan(struct2array(metrics)))
    score = 0;
    return;
end

score = opts.WeightEEMean * metrics.eeErrorMeanScore + ...
        opts.WeightEEMax * metrics.eeErrorMaxScore + ...
        opts.WeightSmoothness * metrics.smoothnessScore + ...
        opts.WeightCusps * metrics.cuspScore + ...
        opts.WeightComputeTime * metrics.computeTimeScore;

% Clamp to [0, 1]
score = max(0, min(1, score));

end

function generateSummaryReport(results, opts, resultsDir)
%GENERATESUMMARYREPORT Create text report of sweep results.

fid = fopen(fullfile(resultsDir, 'SWEEP_summary_report.txt'), 'w');
fprintf(fid, 'Parameter Sweep Summary Report\n');
fprintf(fid, 'Generated: %s\n\n', datestr(now));
fprintf(fid, '=== CONFIGURATION ===\n');
fprintf(fid, 'Sweep mode: %s\n', opts.SweepMode);
fprintf(fid, 'Configurations tested: %d\n', length(results));
fprintf(fid, '\n=== SCORING WEIGHTS ===\n');
fprintf(fid, 'EE Mean Error: %.2f\n', opts.WeightEEMean);
fprintf(fid, 'EE Max Error: %.2f\n', opts.WeightEEMax);
fprintf(fid, 'Path Smoothness: %.2f\n', opts.WeightSmoothness);
fprintf(fid, 'Cusp Count: %.2f\n', opts.WeightCusps);
fprintf(fid, 'Compute Time: %.2f\n', opts.WeightComputeTime);

scores = [results.score];
[~, sortedIdx] = sort(scores, 'descend');

fprintf(fid, '\n=== TOP 10 CONFIGURATIONS ===\n');
for i = 1:min(10, length(results))
    idx = sortedIdx(i);
    fprintf(fid, '\n[Rank %d] Score: %.4f\n', i, results(idx).score);
    fprintf(fid, '  Safety Margin: %.3f m\n', results(idx).config.safetyMargin);
    fprintf(fid, '  Lambda Cusp: %.1f\n', results(idx).config.lambdaCusp);
    fprintf(fid, '  Max Iters: %d\n', results(idx).config.maxIters);
    fprintf(fid, '  Allow Reverse: %s\n', ternary(results(idx).config.allowReverse, 'true', 'false'));
    fprintf(fid, '  Clothoid Discretization: %.3f m\n', results(idx).config.clothoidDiscretization);
    fprintf(fid, '  Lookahead: %.2f m\n', results(idx).config.lookaheadDistance);
    fprintf(fid, '  Accel Limit: %.2f m/s²\n', results(idx).config.accelLimit);
    fprintf(fid, '  Heading Kp: %.2f\n', results(idx).config.headingKp);
    
    if isfield(results(idx), 'metrics') && ~isempty(results(idx).metrics) && ...
       isfield(results(idx).metrics, 'eeErrorMean')
        m = results(idx).metrics;
        fprintf(fid, '  Metrics:\n');
        fprintf(fid, '    EE Mean: %.4f m (score %.3f)\n', m.eeErrorMean, m.eeErrorMeanScore);
        fprintf(fid, '    EE Max: %.4f m (score %.3f)\n', m.eeErrorMax, m.eeErrorMaxScore);
        fprintf(fid, '    Smoothness: %.3f (score %.3f)\n', m.pathSmoothness, m.smoothnessScore);
        fprintf(fid, '    Cusps: %d (score %.3f)\n', m.cuspCount, m.cuspScore);
        fprintf(fid, '    Compute: %.2f s (score %.3f)\n', m.computeTime, m.computeTimeScore);
    else
        fprintf(fid, '  Metrics: Not available (run failed)\n');
    end
end

fclose(fid);

end

function generateParetoPlots(results, ~, resultsDir)
%GENERATEPARETOPLOTS Create visualization of parameter trade-offs.

scores = [results.score];
validIdx = scores > 0;
validResults = results(validIdx);

if isempty(validResults)
    warning('No valid results to plot');
    return;
end

% Extract metrics
eeErrorMean = arrayfun(@(r) r.metrics.eeErrorMean, validResults);
eeErrorMax = arrayfun(@(r) r.metrics.eeErrorMax, validResults);
smoothness = arrayfun(@(r) r.metrics.pathSmoothness, validResults);
cusps = arrayfun(@(r) r.metrics.cuspCount, validResults);
validScores = arrayfun(@(r) r.score, validResults);

% Create figure with subplots
fig = figure('Position', [100, 100, 1200, 800]);

% EE Mean vs Max
subplot(2, 3, 1);
scatter(eeErrorMean, eeErrorMax, 50, validScores, 'filled');
xlabel('Mean EE Error [m]');
ylabel('Max EE Error [m]');
title('EE Tracking Trade-off');
colorbar;
grid on;

% EE Mean vs Smoothness
subplot(2, 3, 2);
scatter(eeErrorMean, smoothness, 50, validScores, 'filled');
xlabel('Mean EE Error [m]');
ylabel('Path Smoothness (σ)');
title('Tracking vs Smoothness');
colorbar;
grid on;

% Smoothness vs Cusps
subplot(2, 3, 3);
scatter(smoothness, cusps, 50, validScores, 'filled');
xlabel('Path Smoothness (σ)');
ylabel('Cusp Count');
title('Smoothness vs Cusps');
colorbar;
grid on;

% Score distribution
subplot(2, 3, 4);
histogram(validScores, 20);
xlabel('Composite Score');
ylabel('Count');
title('Score Distribution');
grid on;

% Parameter sensitivity (if Stage B sweep)
subplot(2, 3, 5);
lambdas = arrayfun(@(r) r.config.lambdaCusp, validResults);
scatter(lambdas, validScores, 50, 'filled');
xlabel('Lambda Cusp');
ylabel('Composite Score');
title('Lambda Cusp Sensitivity');
grid on;

subplot(2, 3, 6);
margins = arrayfun(@(r) r.config.safetyMargin, validResults);
scatter(margins, validScores, 50, 'filled');
xlabel('Safety Margin [m]');
ylabel('Composite Score');
title('Safety Margin Sensitivity');
grid on;

sgtitle('Parameter Sweep Analysis');

exportgraphics(fig, fullfile(resultsDir, 'SWEEP_pareto_plots.png'), 'Resolution', 300);
close(fig);

end

function printConfigSummary(config)
%PRINTCONFIGSUMMARY One-line summary of configuration.

fprintf('margin=%.2f, lambda=%.1f, iters=%d, lookahead=%.2f\n', ...
    config.safetyMargin, config.lambdaCusp, config.maxIters, config.lookaheadDistance);

end

function result = ternary(condition, trueVal, falseVal)
%TERNARY Inline conditional operator.

if condition
    result = trueVal;
else
    result = falseVal;
end

end
