%% Comprehensive Analysis of All Test Results
%  This script loads all previous test results and performs deep analysis
%  to understand parameter sensitivity, identify issues, and provide
%  actionable recommendations.

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Comprehensive Test Results Analysis ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Load All Available Test Results
fprintf('[1/5] Loading test results...\n');

resultsFiles = {
    'results/test_comprehensive_evaluation.mat'
    'results/20251010_211447_SWEEP_STAGEB_quick_test/sweep_results.mat'
};

allResults = struct();
loadCount = 0;

for i = 1:length(resultsFiles)
    if isfile(resultsFiles{i})
        fprintf('  ✓ Loading: %s\n', resultsFiles{i});
        data = load(resultsFiles{i});
        
        if contains(resultsFiles{i}, 'comprehensive_evaluation')
            allResults.comprehensive = data.results;
            loadCount = loadCount + 1;
        elseif contains(resultsFiles{i}, 'SWEEP')
            allResults.sweep = data.results;
            loadCount = loadCount + 1;
        end
    else
        fprintf('  ✗ Not found: %s\n', resultsFiles{i});
    end
end

fprintf('  Loaded %d result sets\n\n', loadCount);

if loadCount == 0
    error('No results found to analyze!');
end

%% [2/5] Analyze Comprehensive Evaluation Results
if isfield(allResults, 'comprehensive')
    fprintf('[2/5] Analyzing comprehensive evaluation results...\n');
    
    compResults = allResults.comprehensive;
    nConfigs = numel(compResults);
    
    fprintf('  Configurations tested: %d\n', nConfigs);
    
    % Extract metrics
    configNames = {compResults.name};
    eeErrors = zeros(nConfigs, 2);  % [mean, max]
    cuspCounts = zeros(nConfigs, 1);
    overallScores = zeros(nConfigs, 1);
    
    for i = 1:nConfigs
        eeErrors(i, 1) = compResults(i).evalReport.c3_meanError;
        eeErrors(i, 2) = compResults(i).evalReport.c3_maxError;
        cuspCounts(i) = compResults(i).evalReport.c5_cuspCount;
        overallScores(i) = compResults(i).evalReport.overallScore;
    end
    
    % Statistical summary
    fprintf('\n  EE Tracking Errors:\n');
    fprintf('    Mean error: %.4f ± %.4f m (range: %.4f - %.4f m)\n', ...
            mean(eeErrors(:, 1)), std(eeErrors(:, 1)), ...
            min(eeErrors(:, 1)), max(eeErrors(:, 1)));
    fprintf('    Max error:  %.4f ± %.4f m (range: %.4f - %.4f m)\n', ...
            mean(eeErrors(:, 2)), std(eeErrors(:, 2)), ...
            min(eeErrors(:, 2)), max(eeErrors(:, 2)));
    
    fprintf('\n  Cusp Analysis:\n');
    fprintf('    Cusp count: %.1f ± %.1f (range: %d - %d)\n', ...
            mean(cuspCounts), std(cuspCounts), ...
            min(cuspCounts), max(cuspCounts));
    fprintf('    Configs with 0 cusps: %d/%d (%.1f%%)\n', ...
            sum(cuspCounts == 0), nConfigs, ...
            100 * sum(cuspCounts == 0) / nConfigs);
    
    fprintf('\n  Overall Scores:\n');
    fprintf('    Score: %.3f ± %.3f (range: %.3f - %.3f)\n', ...
            mean(overallScores), std(overallScores), ...
            min(overallScores), max(overallScores));
    
    % Check for parameter sensitivity
    fprintf('\n  Parameter Sensitivity Check:\n');
    if std(eeErrors(:, 1)) < 0.001
        fprintf('    ⚠ EE mean error shows minimal variation (<0.001m)\n');
        fprintf('      → Parameters may not be affecting trajectory significantly\n');
    end
    if std(cuspCounts) < 0.5
        fprintf('    ⚠ Cusp count shows minimal variation\n');
        fprintf('      → Stage B parameters may not be diverse enough\n');
    end
    
else
    fprintf('[2/5] No comprehensive evaluation results found\n');
end

%% [3/5] Analyze Parameter Sweep Results
if isfield(allResults, 'sweep')
    fprintf('\n[3/5] Analyzing parameter sweep results...\n');
    
    sweepResults = allResults.sweep;
    nSweep = numel(sweepResults);
    
    fprintf('  Configurations tested: %d\n', nSweep);
    
    % Extract configurations
    safetyMargins = zeros(nSweep, 1);
    lambdaCusps = zeros(nSweep, 1);
    eeErrorsMean = zeros(nSweep, 1);
    eeErrorsMax = zeros(nSweep, 1);
    smoothness = zeros(nSweep, 1);
    cusps = zeros(nSweep, 1);
    scores = zeros(nSweep, 1);
    
    for i = 1:nSweep
        safetyMargins(i) = sweepResults(i).config.safetyMargin;
        lambdaCusps(i) = sweepResults(i).config.lambdaCusp;
        
        if isfield(sweepResults(i), 'metrics') && ~isempty(sweepResults(i).metrics)
            m = sweepResults(i).metrics;
            eeErrorsMean(i) = m.eeErrorMean;
            eeErrorsMax(i) = m.eeErrorMax;
            smoothness(i) = m.pathSmoothness;
            cusps(i) = m.cuspCount;
        else
            eeErrorsMean(i) = NaN;
            eeErrorsMax(i) = NaN;
            smoothness(i) = NaN;
            cusps(i) = NaN;
        end
        
        scores(i) = sweepResults(i).score;
    end
    
    % Remove NaN entries
    validIdx = ~isnan(eeErrorsMean);
    
    if sum(validIdx) > 0
        fprintf('\n  Valid results: %d/%d\n', sum(validIdx), nSweep);
        
        fprintf('\n  Performance Metrics:\n');
        fprintf('    EE Mean Error: %.4f ± %.4f m\n', ...
                mean(eeErrorsMean(validIdx)), std(eeErrorsMean(validIdx)));
        fprintf('    EE Max Error:  %.4f ± %.4f m\n', ...
                mean(eeErrorsMax(validIdx)), std(eeErrorsMax(validIdx)));
        fprintf('    Smoothness:    %.3f ± %.3f\n', ...
                mean(smoothness(validIdx)), std(smoothness(validIdx)));
        fprintf('    Cusp Count:    %.1f ± %.1f\n', ...
                mean(cusps(validIdx)), std(cusps(validIdx)));
        
        % Parameter sensitivity analysis
        fprintf('\n  Parameter Sensitivity:\n');
        
        % Safety Margin impact
        uniqueMargins = unique(safetyMargins(validIdx));
        if length(uniqueMargins) > 1
            fprintf('    Safety Margin:\n');
            for margin = uniqueMargins'
                idx = validIdx & (safetyMargins == margin);
                avgScore = mean(scores(idx));
                avgCusps = mean(cusps(idx));
                fprintf('      %.2fm: score=%.3f, cusps=%.1f (n=%d)\n', ...
                        margin, avgScore, avgCusps, sum(idx));
            end
        else
            fprintf('    Safety Margin: single value tested (%.2fm)\n', uniqueMargins);
        end
        
        % Lambda Cusp impact
        uniqueLambdas = unique(lambdaCusps(validIdx));
        if length(uniqueLambdas) > 1
            fprintf('    Lambda Cusp:\n');
            for lambda = uniqueLambdas'
                idx = validIdx & (lambdaCusps == lambda);
                avgScore = mean(scores(idx));
                avgCusps = mean(cusps(idx));
                fprintf('      %.1f: score=%.3f, cusps=%.1f (n=%d)\n', ...
                        lambda, avgScore, avgCusps, sum(idx));
            end
        else
            fprintf('    Lambda Cusp: single value tested (%.1f)\n', uniqueLambdas);
        end
        
        % Correlation analysis
        fprintf('\n  Correlation Analysis (Pearson r):\n');
        if length(uniqueMargins) > 1
            r_margin_score = corr(safetyMargins(validIdx), scores(validIdx));
            r_margin_cusps = corr(safetyMargins(validIdx), cusps(validIdx));
            fprintf('    SafetyMargin vs Score: r=%.3f\n', r_margin_score);
            fprintf('    SafetyMargin vs Cusps: r=%.3f\n', r_margin_cusps);
        end
        if length(uniqueLambdas) > 1
            r_lambda_score = corr(lambdaCusps(validIdx), scores(validIdx));
            r_lambda_cusps = corr(lambdaCusps(validIdx), cusps(validIdx));
            fprintf('    LambdaCusp vs Score:   r=%.3f\n', r_lambda_score);
            fprintf('    LambdaCusp vs Cusps:   r=%.3f\n', r_lambda_cusps);
        end
    else
        fprintf('  ⚠ No valid results found in sweep data\n');
    end
    
else
    fprintf('\n[3/5] No parameter sweep results found\n');
end

%% [4/5] Cross-Test Comparison
fprintf('\n[4/5] Cross-test comparison...\n');

if isfield(allResults, 'comprehensive') && isfield(allResults, 'sweep')
    compMeanEE = mean(eeErrors(:, 1));
    sweepMeanEE = mean(eeErrorsMean(validIdx));
    
    fprintf('  Mean EE Error:\n');
    fprintf('    Comprehensive test: %.4f m\n', compMeanEE);
    fprintf('    Sweep test:         %.4f m\n', sweepMeanEE);
    fprintf('    Difference:         %.4f m (%.1f%%)\n', ...
            abs(compMeanEE - sweepMeanEE), ...
            100 * abs(compMeanEE - sweepMeanEE) / compMeanEE);
    
    if abs(compMeanEE - sweepMeanEE) < 0.01
        fprintf('    ✓ Results are consistent across tests\n');
    else
        fprintf('    ⚠ Significant difference detected\n');
    end
end

%% [5/5] Issue Identification and Recommendations
fprintf('\n[5/5] Issue Identification & Recommendations\n');
fprintf('======================================\n');

issues = {};
recommendations = {};

% Issue 1: Identical results across diverse configs
if isfield(allResults, 'comprehensive')
    if std(eeErrors(:, 1)) < 0.001 && std(cuspCounts) < 0.5
        issues{end+1} = 'All configs show nearly identical results (EE error, cusps)';
        recommendations{end+1} = 'Parameters not impacting trajectory. Consider:';
        recommendations{end+1} = '  - Test on more complex trajectory (more obstacles, tighter spaces)';
        recommendations{end+1} = '  - Expand parameter ranges (safety: 0.03-0.25m, lambda: 0.1-5.0)';
        recommendations{end+1} = '  - Check if Hybrid A* is dominating (try Stage B without Hybrid A*)';
    end
end

% Issue 2: Persistent cusps
if isfield(allResults, 'comprehensive') && all(cuspCounts >= 2)
    issues{end+1} = 'All configs have 2 cusps despite varying lambdaCusp penalty';
    recommendations{end+1} = 'Cusps not being eliminated by Reeds-Shepp smoothing:';
    recommendations{end+1} = '  - Visualize cusp locations (are they at same positions?)';
    recommendations{end+1} = '  - Check if cusps are in Hybrid A* output (before RS refinement)';
    recommendations{end+1} = '  - Try higher RS iteration counts (800-1000)';
    recommendations{end+1} = '  - Examine cusp severity (angle at reversal)';
end

% Issue 3: Excellent tracking but unknown reference quality
if isfield(allResults, 'comprehensive') && mean(eeErrors(:, 1)) < 0.005
    issues{end+1} = 'Tracking is excellent (mean <0.005m) but reference path quality unknown';
    recommendations{end+1} = 'Need to evaluate reference path smoothness:';
    recommendations{end+1} = '  - Implement Criterion 1 & 2 (EE/base ref path jerk/kinks)';
    recommendations{end+1} = '  - Generate animations to visually confirm smoothness';
    recommendations{end+1} = '  - Check if "kinky" observations are in reference vs execution';
end

% Issue 4: Missing comprehensive evaluation
if ~isfield(allResults, 'comprehensive') || ~isfield(compResults(1).evalReport, 'c1_score')
    issues{end+1} = 'Full 6-criteria evaluation not implemented';
    recommendations{end+1} = 'Complete comprehensiveEvaluation.m implementation:';
    recommendations{end+1} = '  - Extract reference paths from log (stageC.referenceInitialIk)';
    recommendations{end+1} = '  - Call evaluatePathSmoothness on reference trajectories';
    recommendations{end+1} = '  - Implement collision intrusion check';
    recommendations{end+1} = '  - Add sideways movement validation';
end

% Display issues and recommendations
fprintf('\n📋 ISSUES IDENTIFIED: %d\n', length(issues));
for i = 1:length(issues)
    fprintf('\n[Issue %d] %s\n', i, issues{i});
end

fprintf('\n\n💡 RECOMMENDATIONS:\n');
recIdx = 1;
for i = 1:length(recommendations)
    fprintf('%d. %s\n', recIdx, recommendations{i});
    recIdx = recIdx + 1;
end

%% Summary Statistics Table
fprintf('\n\n=== SUMMARY TABLE ===\n');
fprintf('%-25s | %s\n', 'Metric', 'Value');
fprintf('%s\n', repmat('-', 1, 60));

if isfield(allResults, 'comprehensive')
    fprintf('%-25s | %.4f m (± %.4f)\n', 'EE Mean Error (Comp)', ...
            mean(eeErrors(:, 1)), std(eeErrors(:, 1)));
    fprintf('%-25s | %.4f m (± %.4f)\n', 'EE Max Error (Comp)', ...
            mean(eeErrors(:, 2)), std(eeErrors(:, 2)));
    fprintf('%-25s | %.1f (± %.1f)\n', 'Cusp Count (Comp)', ...
            mean(cuspCounts), std(cuspCounts));
    fprintf('%-25s | %.3f (± %.3f)\n', 'Overall Score (Comp)', ...
            mean(overallScores), std(overallScores));
end

if isfield(allResults, 'sweep') && sum(validIdx) > 0
    fprintf('%-25s | %.4f m (± %.4f)\n', 'EE Mean Error (Sweep)', ...
            mean(eeErrorsMean(validIdx)), std(eeErrorsMean(validIdx)));
    fprintf('%-25s | %.1f (± %.1f)\n', 'Cusp Count (Sweep)', ...
            mean(cusps(validIdx)), std(cusps(validIdx)));
end

fprintf('\n=== Analysis Complete ===\n');
fprintf('Timestamp: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
