% Comprehensive Method 4 Performance Analysis
% Focus: Identifying improvements to minimize EE tracking error

clear; clc;

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('         METHOD 4 PERFORMANCE ANALYSIS\n');
fprintf('  Goal: Minimize End-Effector Tracking Error\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

% Load the most recent aggressive parameters test
logFile = 'results/20251013_183955_method_comparison_aggressive/log_method4_ppFirst_aggressive.mat';
fprintf('Loading: %s\n', logFile);

if ~exist(logFile, 'file')
    error('Log file not found. Please run the aggressive comparison first.');
end

data = load(logFile);
log4 = data.log4;
logC = log4.stageLogs.stageC;

fprintf('✓ Loaded successfully\n\n');

%% 1. CURRENT PERFORMANCE SUMMARY
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('1. CURRENT PERFORMANCE (Aggressive Parameters)\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

% EE tracking metrics
eeErrors = logC.positionErrorNorm * 1000;  % Convert to mm
fprintf('End-Effector Tracking Error:\n');
fprintf('  Mean:   %.2f mm\n', mean(eeErrors));
fprintf('  Median: %.2f mm\n', median(eeErrors));
fprintf('  RMS:    %.2f mm\n', sqrt(mean(eeErrors.^2)));
fprintf('  Max:    %.2f mm\n', max(eeErrors));
fprintf('  Min:    %.2f mm\n', min(eeErrors));
fprintf('  Std:    %.2f mm\n', std(eeErrors));

% Percentiles
pcts = [50, 75, 90, 95, 99];
pct_vals = prctile(eeErrors, pcts);
fprintf('\n  Percentiles:\n');
for i = 1:length(pcts)
    fprintf('    %d%%: %.2f mm\n', pcts(i), pct_vals(i));
end

% Fallback analysis
if isfield(logC.diagnostics, 'fallbackRate')
    fallbackRate = logC.diagnostics.fallbackRate * 100;
    fprintf('\nFallback Behavior:\n');
    fprintf('  Fallback rate: %.1f%%\n', fallbackRate);
    fprintf('  Fallback count: %d / %d waypoints\n', ...
        round(logC.diagnostics.fallbackRate * size(logC.qTraj,2)), size(logC.qTraj,2));
end

% Convergence
convergenceRate = 100 * sum(logC.successMask) / length(logC.successMask);
fprintf('\nSolver Performance:\n');
fprintf('  Convergence rate: %.1f%%\n', convergenceRate);
fprintf('  Mean iterations: %.1f\n', mean(logC.iterations));
fprintf('  Max iterations: %d\n', max(logC.iterations));

%% 2. IDENTIFY ERROR PATTERNS
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('2. ERROR PATTERN ANALYSIS\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

% Identify high-error waypoints
threshold_high = mean(eeErrors) + std(eeErrors);
high_error_idx = find(eeErrors > threshold_high);
fprintf('High Error Waypoints (>%.1f mm):\n', threshold_high);
fprintf('  Count: %d (%.1f%% of trajectory)\n', ...
    length(high_error_idx), 100*length(high_error_idx)/length(eeErrors));
fprintf('  Mean error in this group: %.2f mm\n', mean(eeErrors(high_error_idx)));

% Check if fallback is the issue
if isfield(logC, 'fallbackUsed') && ~isempty(logC.fallbackUsed)
    fallback_idx = find(logC.fallbackUsed);
    
    % Compare errors: fallback vs non-fallback
    if ~isempty(fallback_idx)
        fallback_errors = eeErrors(fallback_idx);
        nonfallback_errors = eeErrors(~logC.fallbackUsed);
        
        fprintf('\nFallback vs Non-Fallback Comparison:\n');
        fprintf('  Fallback waypoints:\n');
        fprintf('    Count: %d\n', length(fallback_idx));
        fprintf('    Mean error: %.2f mm\n', mean(fallback_errors));
        fprintf('    Max error: %.2f mm\n', max(fallback_errors));
        
        fprintf('  Non-fallback waypoints:\n');
        fprintf('    Count: %d\n', length(nonfallback_errors));
        fprintf('    Mean error: %.2f mm\n', mean(nonfallback_errors));
        fprintf('    Max error: %.2f mm\n', max(nonfallback_errors));
        
        fprintf('\n  🔴 ERROR CONTRIBUTION:\n');
        fprintf('    Fallback accounts for %.1f%% of total error\n', ...
            100 * sum(fallback_errors) / sum(eeErrors));
    end
end

% Check convergence failures
failed_idx = find(~logC.successMask);
if ~isempty(failed_idx)
    fprintf('\nConvergence Failures:\n');
    fprintf('  Count: %d\n', length(failed_idx));
    fprintf('  Mean error: %.2f mm\n', mean(eeErrors(failed_idx)));
    fprintf('  These account for %.1f%% of waypoints\n', ...
        100*length(failed_idx)/length(eeErrors));
end

% Temporal patterns
fprintf('\nTemporal Analysis:\n');
% Check if errors increase over time
first_third = eeErrors(1:floor(length(eeErrors)/3));
last_third = eeErrors(ceil(2*length(eeErrors)/3):end);
fprintf('  First 1/3 mean: %.2f mm\n', mean(first_third));
fprintf('  Last 1/3 mean: %.2f mm\n', mean(last_third));
if mean(last_third) > mean(first_third) * 1.2
    fprintf('  ⚠️  Error increases over trajectory (drift?)\n');
elseif mean(first_third) > mean(last_third) * 1.2
    fprintf('  ⚠️  Higher error at start (poor initialization?)\n');
else
    fprintf('  ✓ Error distribution relatively uniform\n');
end

%% 3. CONSTRAINT ANALYSIS
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('3. CONSTRAINT TIGHTNESS ANALYSIS\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

% Current parameters (from aggressive profile)
current_params = struct();
current_params.yaw_corridor = 20.0;  % degrees
current_params.position_tol = 0.20;  % meters
current_params.ee_threshold = 0.015; % meters (15mm)

fprintf('Current Parameters:\n');
fprintf('  Yaw corridor: %.1f°\n', current_params.yaw_corridor);
fprintf('  Position tolerance: %.2f m\n', current_params.position_tol);
fprintf('  EE error threshold: %.3f m (%.1f mm)\n', ...
    current_params.ee_threshold, current_params.ee_threshold*1000);

% Check how many solutions are near constraints
fprintf('\nConstraint Utilization:\n');
if isfield(logC, 'diagnostics') && isfield(logC.diagnostics, 'ppFirstUsed')
    ppFirst_idx = find(logC.diagnostics.ppFirstUsed);
    fprintf('  Pure Pursuit constraints used: %d waypoints (%.1f%%)\n', ...
        length(ppFirst_idx), 100*length(ppFirst_idx)/length(eeErrors));
end

% Iteration analysis (high iterations suggest tight constraints)
high_iter_idx = find(logC.iterations > 1000);
fprintf('  High iteration solves (>1000): %d (%.1f%%)\n', ...
    length(high_iter_idx), 100*length(high_iter_idx)/length(logC.iterations));

max_iter_idx = find(logC.iterations >= 1500);
fprintf('  Maxed out iterations (≥1500): %d (%.1f%%)\n', ...
    length(max_iter_idx), 100*length(max_iter_idx)/length(logC.iterations));

if ~isempty(max_iter_idx)
    fprintf('    Mean error for maxed iterations: %.2f mm\n', mean(eeErrors(max_iter_idx)));
end

%% 4. RECOMMENDATIONS FOR IMPROVEMENT
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('4. RECOMMENDATIONS TO IMPROVE EE TRACKING\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

recommendations = {};
priority = {};

% Analyze current performance
current_error = mean(eeErrors);
target_error = 150;  % Target: comparable to Method 1 (129mm)

fprintf('GOAL: Reduce mean EE error from %.1f mm to < %.1f mm\n\n', ...
    current_error, target_error);

% Recommendation 1: Reduce fallback rate
if exist('fallbackRate', 'var') && fallbackRate > 30
    recommendations{end+1} = 'Reduce Fallback Rate';
    priority{end+1} = '🔴 CRITICAL';
    fprintf('🔴 RECOMMENDATION 1: Reduce Fallback Rate\n');
    fprintf('   Current: %.1f%% | Target: <20%%\n\n', fallbackRate);
    fprintf('   Actions:\n');
    fprintf('   a) Relax constraints further:\n');
    fprintf('      - Try yaw_corridor: 25-30°\n');
    fprintf('      - Try position_tol: 0.25-0.30 m\n');
    fprintf('      - Try ee_threshold: 0.020 m (20mm)\n\n');
    fprintf('   b) Improve Pure Pursuit predictions:\n');
    fprintf('      - Increase lookahead distance\n');
    fprintf('      - Use curvature-adaptive lookahead\n');
    fprintf('      - Add velocity consideration\n\n');
    fprintf('   Expected impact: -30 to -50%% reduction in fallback rate\n');
    fprintf('   Expected error reduction: -50 to -100 mm\n\n');
end

% Recommendation 2: Improve convergence
if convergenceRate < 80
    recommendations{end+1} = 'Improve Solver Convergence';
    priority{end+1} = '🔴 CRITICAL';
    fprintf('🔴 RECOMMENDATION 2: Improve Solver Convergence\n');
    fprintf('   Current: %.1f%% | Target: >80%%\n\n', convergenceRate);
    fprintf('   Actions:\n');
    fprintf('   a) Warm-starting:\n');
    fprintf('      - Use previous solution as initial guess\n');
    fprintf('      - Linear interpolation for initialization\n\n');
    fprintf('   b) Relax solver tolerances:\n');
    fprintf('      - ConstraintTolerance: 1e-4 (from 1e-6)\n');
    fprintf('      - StepTolerance: 1e-10\n');
    fprintf('      - OptimalityTolerance: 1e-4\n\n');
    fprintf('   c) Increase iteration limit:\n');
    fprintf('      - Try MaxIterations: 2000 or 2500\n\n');
    fprintf('   Expected impact: +20 to +30%% convergence rate\n');
    fprintf('   Expected error reduction: -20 to -40 mm\n\n');
end

% Recommendation 3: Adaptive corridor
recommendations{end+1} = 'Implement Adaptive Corridor';
priority{end+1} = '⚠️ HIGH';
fprintf('⚠️ RECOMMENDATION 3: Implement Adaptive Corridor\n');
fprintf('   Adjust corridor width based on trajectory characteristics\n\n');
fprintf('   Actions:\n');
fprintf('   a) Curvature-based adaptation:\n');
fprintf('      - Straight sections: ±15° corridor (tight)\n');
fprintf('      - Curved sections: ±30-40° corridor (wide)\n');
fprintf('      - Compute local curvature from path\n\n');
fprintf('   b) Velocity-based adaptation:\n');
fprintf('      - Slow motion: tighter constraints\n');
fprintf('      - Fast motion: wider constraints\n\n');
fprintf('   Expected impact: -10 to -20%% fallback rate\n');
fprintf('   Expected error reduction: -20 to -40 mm\n\n');

% Recommendation 4: Better fallback strategy
if exist('fallbackRate', 'var') && fallbackRate > 20
    recommendations{end+1} = 'Improve Fallback Strategy';
    priority{end+1} = '⚠️ HIGH';
    fprintf('⚠️ RECOMMENDATION 4: Improve Fallback Strategy\n');
    fprintf('   When GIK fails, current fallback fixes base and solves arm-only\n\n');
    fprintf('   Actions:\n');
    fprintf('   a) Multi-stage fallback:\n');
    fprintf('      1. Try with wider corridor (2x current)\n');
    fprintf('      2. If still failing, relax EE constraint\n');
    fprintf('      3. Last resort: arm-only IK\n\n');
    fprintf('   b) Base refinement in fallback:\n');
    fprintf('      - Allow small base adjustments (±5cm, ±5°)\n');
    fprintf('      - Optimize base position for reachability\n\n');
    fprintf('   c) Predictive fallback:\n');
    fprintf('      - Detect upcoming difficult configurations\n');
    fprintf('      - Pre-widen corridor before failure occurs\n\n');
    fprintf('   Expected impact: -50%% error in fallback cases\n');
    fprintf('   Expected error reduction: -30 to -60 mm overall\n\n');
end

% Recommendation 5: Trajectory preprocessing
recommendations{end+1} = 'Trajectory Preprocessing';
priority{end+1} = '⚠️ MEDIUM';
fprintf('⚠️ RECOMMENDATION 5: Trajectory Preprocessing\n');
fprintf('   Analyze and optimize trajectory before execution\n\n');
fprintf('   Actions:\n');
fprintf('   a) Reachability analysis:\n');
fprintf('      - Identify waypoints with poor reachability\n');
fprintf('      - Adjust base path to improve arm workspace\n\n');
fprintf('   b) Smooth Pure Pursuit path:\n');
fprintf('      - Apply spline smoothing to PP predictions\n');
fprintf('      - Reduce sudden direction changes\n\n');
fprintf('   c) Collision-aware corridor sizing:\n');
fprintf('      - Narrow corridor in cluttered regions\n');
fprintf('      - Widen in open space\n\n');
fprintf('   Expected impact: -5 to -15%% fallback rate\n');
fprintf('   Expected error reduction: -10 to -20 mm\n\n');

% Recommendation 6: Hybrid approach
recommendations{end+1} = 'Hybrid Method 1/4 Approach';
priority{end+1} = '💡 EXPERIMENTAL';
fprintf('💡 RECOMMENDATION 6: Hybrid Method 1/4 Approach\n');
fprintf('   Combine strengths of both methods\n\n');
fprintf('   Actions:\n');
fprintf('   a) Use Method 1 for difficult sections:\n');
fprintf('      - Detect high-curvature or tight regions\n');
fprintf('      - Switch to unconstrained GIK (Method 1)\n');
fprintf('      - Resume Method 4 in easier sections\n\n');
fprintf('   b) Method 1 seeding:\n');
fprintf('      - Run fast Method 1 pass with coarse resolution\n');
fprintf('      - Use result to guide Method 4 corridor\n');
fprintf('      - Tighter corridors around known-good path\n\n');
fprintf('   Expected impact: Best of both worlds\n');
fprintf('   Expected error reduction: -50 to -100 mm\n\n');

%% 5. PARAMETER TUNING EXPERIMENTS
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('5. SUGGESTED PARAMETER EXPERIMENTS\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('Run systematic experiments to find optimal parameters:\n\n');

% Create experiment matrix
experiments = struct();

% Experiment 1: Ultra-aggressive corridor
experiments(1).name = 'Ultra-Aggressive Corridor';
experiments(1).yaw_corridor = 30.0;
experiments(1).position_tol = 0.30;
experiments(1).ee_threshold = 0.020;
experiments(1).expected_fallback = '15-25%';
experiments(1).expected_error = '200-300 mm';

% Experiment 2: Very wide corridor
experiments(2).name = 'Very Wide Corridor';
experiments(2).yaw_corridor = 40.0;
experiments(2).position_tol = 0.40;
experiments(2).ee_threshold = 0.025;
experiments(2).expected_fallback = '5-15%';
experiments(2).expected_error = '150-250 mm';

% Experiment 3: Tighter threshold, wider corridor
experiments(3).name = 'Tight Threshold + Wide Corridor';
experiments(3).yaw_corridor = 35.0;
experiments(3).position_tol = 0.35;
experiments(3).ee_threshold = 0.010;
experiments(3).expected_fallback = '10-20%';
experiments(3).expected_error = '100-200 mm';

% Experiment 4: Optimal balance (predicted)
experiments(4).name = 'Predicted Optimal';
experiments(4).yaw_corridor = 25.0;
experiments(4).position_tol = 0.25;
experiments(4).ee_threshold = 0.018;
experiments(4).expected_fallback = '15-20%';
experiments(4).expected_error = '150-200 mm';

for i = 1:length(experiments)
    exp = experiments(i);
    fprintf('Experiment %d: %s\n', i, exp.name);
    fprintf('  yaw_corridor: %.1f°\n', exp.yaw_corridor);
    fprintf('  position_tol: %.2f m\n', exp.position_tol);
    fprintf('  ee_threshold: %.3f m\n', exp.ee_threshold);
    fprintf('  Expected fallback: %s\n', exp.expected_fallback);
    fprintf('  Expected error: %s\n\n', exp.expected_error);
end

%% 6. IMPLEMENTATION PRIORITY
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('6. IMPLEMENTATION PRIORITY & TIMELINE\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('PHASE 1 (Week 1): Quick Wins\n');
fprintf('─────────────────────────────\n');
fprintf('1. Run parameter experiments (above)\n');
fprintf('   Effort: 4-6 hours\n');
fprintf('   Expected: -50 to -100 mm error reduction\n\n');

fprintf('2. Implement warm-starting\n');
fprintf('   Effort: 4 hours\n');
fprintf('   Expected: +20%% convergence, -30 mm error\n\n');

fprintf('3. Relax solver tolerances\n');
fprintf('   Effort: 1 hour\n');
fprintf('   Expected: +10%% convergence, -20 mm error\n\n');

fprintf('PHASE 2 (Week 2): Architecture Improvements\n');
fprintf('─────────────────────────────────────────────\n');
fprintf('1. Implement adaptive corridor\n');
fprintf('   Effort: 8-12 hours\n');
fprintf('   Expected: -15%% fallback, -40 mm error\n\n');

fprintf('2. Improve fallback strategy\n');
fprintf('   Effort: 6-8 hours\n');
fprintf('   Expected: -50%% error in fallback cases\n\n');

fprintf('PHASE 3 (Week 3): Advanced Features\n');
fprintf('────────────────────────────────────\n');
fprintf('1. Trajectory preprocessing\n');
fprintf('   Effort: 10-15 hours\n');
fprintf('   Expected: -10%% fallback, -20 mm error\n\n');

fprintf('2. Hybrid Method 1/4 approach\n');
fprintf('   Effort: 15-20 hours\n');
fprintf('   Expected: Best-in-class performance\n\n');

fprintf('TARGET PERFORMANCE (after all improvements):\n');
fprintf('  • Mean EE error: < 150 mm (competitive with Method 1)\n');
fprintf('  • Fallback rate: < 10%%\n');
fprintf('  • Convergence rate: > 85%%\n');
fprintf('  • Execution time: < 10 seconds\n\n');

%% 7. GENERATE DIAGNOSTIC PLOTS
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('7. GENERATING DIAGNOSTIC PLOTS\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

% Comprehensive diagnostic figure
fig = figure('Name', 'Method 4 Diagnostic Analysis', 'Position', [50, 50, 1600, 1000]);

% Error over time
subplot(3,3,1);
plot(logC.timestamps, eeErrors, 'b-', 'LineWidth', 1.5);
hold on;
yline(mean(eeErrors), 'r--', sprintf('Mean: %.1f mm', mean(eeErrors)));
yline(threshold_high, 'k--', 'High Error Threshold');
xlabel('Time (s)');
ylabel('EE Error (mm)');
title('EE Error Over Time');
grid on;

% Error distribution
subplot(3,3,2);
histogram(eeErrors, 50, 'FaceColor', [0.3 0.6 0.8]);
xlabel('EE Error (mm)');
ylabel('Count');
title('Error Distribution');
grid on;

% Error vs iterations
subplot(3,3,3);
scatter(logC.iterations, eeErrors, 20, 'filled', 'MarkerFaceAlpha', 0.6);
xlabel('GIK Iterations');
ylabel('EE Error (mm)');
title('Error vs Solver Iterations');
grid on;

% Cumulative error
subplot(3,3,4);
plot(cumsum(eeErrors)/1000, 'r-', 'LineWidth', 2);
xlabel('Waypoint Index');
ylabel('Cumulative Error (m)');
title('Cumulative EE Error');
grid on;

% Iterations over time
subplot(3,3,5);
plot(logC.timestamps, logC.iterations, 'g-', 'LineWidth', 1.5);
yline(1500, 'k--', 'Max Iterations');
yline(mean(logC.iterations), 'r--', sprintf('Mean: %.0f', mean(logC.iterations)));
xlabel('Time (s)');
ylabel('Iterations');
title('Solver Iterations');
grid on;

% Success mask
subplot(3,3,6);
plot(logC.timestamps, double(logC.successMask), 'b-', 'LineWidth', 1.5);
ylim([-0.1, 1.1]);
xlabel('Time (s)');
ylabel('Success (1) / Failure (0)');
title(sprintf('Convergence (%.1f%% success)', convergenceRate));
grid on;

% Fallback analysis
if isfield(logC, 'fallbackUsed')
    subplot(3,3,7);
    plot(logC.timestamps, double(logC.fallbackUsed), 'r-', 'LineWidth', 1.5);
    ylim([-0.1, 1.1]);
    xlabel('Time (s)');
    ylabel('Fallback Used (1/0)');
    title(sprintf('Fallback Usage (%.1f%% rate)', fallbackRate));
    grid on;
    
    % Error comparison: fallback vs non-fallback
    subplot(3,3,8);
    boxplot([eeErrors(logC.fallbackUsed)', eeErrors(~logC.fallbackUsed)'], ...
        [ones(sum(logC.fallbackUsed),1); 2*ones(sum(~logC.fallbackUsed),1)], ...
        'Labels', {'Fallback', 'Normal'});
    ylabel('EE Error (mm)');
    title('Error: Fallback vs Normal');
    grid on;
end

% Error percentiles
subplot(3,3,9);
bar(pct_vals);
set(gca, 'XTickLabel', arrayfun(@(x) sprintf('%d%%', x), pcts, 'UniformOutput', false));
ylabel('EE Error (mm)');
title('Error Percentiles');
grid on;

% Save figure
figPath = 'results/method4_diagnostic_analysis.png';
saveas(fig, figPath);
fprintf('✓ Saved diagnostic plot: %s\n', figPath);

% Save analysis results
analysisPath = 'results/method4_performance_analysis_detailed.mat';
save(analysisPath, 'logC', 'eeErrors', 'current_params', 'recommendations', ...
    'priority', 'experiments', '-v7.3');
fprintf('✓ Saved analysis data: %s\n', analysisPath);

fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('Summary of Recommendations:\n');
for i = 1:length(recommendations)
    fprintf('  %s %s\n', priority{i}, recommendations{i});
end

fprintf('\nNext Steps:\n');
fprintf('  1. Review this analysis and diagnostic plots\n');
fprintf('  2. Run parameter experiments (Phase 1)\n');
fprintf('  3. Implement high-priority improvements\n');
fprintf('  4. Re-run comparison with optimized parameters\n\n');

fprintf('Goal: Reduce mean EE error from %.1f mm to < 150 mm\n', mean(eeErrors));
fprintf('      This would make Method 4 competitive with Method 1 (129 mm)\n\n');
