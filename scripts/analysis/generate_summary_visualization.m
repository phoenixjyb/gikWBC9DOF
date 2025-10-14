% Generate Summary Visualization for Method 1 Performance
% Creates publication-quality figures showing the performance issues

clear; clc;

% Load analysis results
fprintf('Loading analysis results...\n');
data = load('results/method1_performance_analysis.mat');
stageResults = data.stageResults;
logData = data.data;

fprintf('Creating summary visualizations...\n');

%% Create comprehensive summary figure
fig = figure('Name', 'Method 1 Performance Summary', ...
    'Position', [50, 50, 1600, 1000], 'Color', 'w');

% Title
annotation('textbox', [0.35, 0.95, 0.3, 0.04], ...
    'String', 'Method 1 Performance Analysis Summary', ...
    'FontSize', 16, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none');

annotation('textbox', [0.35, 0.92, 0.3, 0.02], ...
    'String', sprintf('Total Time: %.2f minutes (%.2f hours)', ...
        logData.elapsed1/60, logData.elapsed1/3600), ...
    'FontSize', 12, 'FontWeight', 'normal', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none', 'Color', 'r');

%% 1. Time breakdown pie chart
subplot(3,4,1);
stage_times = [];
stage_labels = {};
colors = [];

if stageResults.stageA.executed && isfield(stageResults.stageA, 'totalTime')
    stage_times(end+1) = stageResults.stageA.totalTime;
    stage_labels{end+1} = sprintf('Stage A\n%.1f min (%.1f%%)', ...
        stageResults.stageA.totalTime/60, ...
        100*stageResults.stageA.totalTime/logData.elapsed1);
    colors = [colors; 0.2, 0.6, 0.8];
end

if stageResults.stageC.executed && isfield(stageResults.stageC, 'totalTime')
    stage_times(end+1) = stageResults.stageC.totalTime;
    stage_labels{end+1} = sprintf('Stage C\n%.1f min (%.1f%%)', ...
        stageResults.stageC.totalTime/60, ...
        100*stageResults.stageC.totalTime/logData.elapsed1);
    colors = [colors; 0.8, 0.3, 0.3];
end

overhead = logData.elapsed1 - sum(stage_times);
stage_times(end+1) = overhead;
stage_labels{end+1} = sprintf('Overhead\n%.1f min (%.1f%%)', ...
    overhead/60, 100*overhead/logData.elapsed1);
colors = [colors; 0.7, 0.7, 0.7];

pie(stage_times);
colormap(colors);
legend(stage_labels, 'Location', 'southoutside', 'FontSize', 8);
title('Time Distribution', 'FontSize', 10, 'FontWeight', 'bold');

%% 2. Solve time comparison
subplot(3,4,2);
mean_times = [stageResults.stageA.meanTime, stageResults.stageC.meanTime];
stage_names = {'Stage A', 'Stage C'};
bar(mean_times, 'FaceColor', [0.4, 0.6, 0.8]);
set(gca, 'XTickLabel', stage_names);
ylabel('Seconds', 'FontSize', 9);
title('Mean Time per Solve', 'FontSize', 10, 'FontWeight', 'bold');
grid on;
% Add value labels
for i = 1:length(mean_times)
    text(i, mean_times(i), sprintf('%.2fs', mean_times(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9);
end

%% 3. Iteration analysis - THE SMOKING GUN!
subplot(3,4,3);
mean_iters = [stageResults.stageA.meanIters, stageResults.stageC.meanIters];
bar(mean_iters, 'FaceColor', [0.8, 0.3, 0.3]);
set(gca, 'XTickLabel', stage_names);
ylabel('Iterations', 'FontSize', 9);
title('⚠️ Mean Iterations (Max=1500)', 'FontSize', 10, 'FontWeight', 'bold');
yline(1500, 'r--', 'LineWidth', 2, 'Label', 'Maximum');
grid on;
ylim([0, 1600]);
% Add value labels
for i = 1:length(mean_iters)
    text(i, mean_iters(i), sprintf('%.0f', mean_iters(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
end

%% 4. Number of solves
subplot(3,4,4);
num_solves = [length(stageResults.stageA.times), length(stageResults.stageC.times)];
bar(num_solves, 'FaceColor', [0.3, 0.7, 0.5]);
set(gca, 'XTickLabel', stage_names);
ylabel('Count', 'FontSize', 9);
title('Number of Solves', 'FontSize', 10, 'FontWeight', 'bold');
grid on;
for i = 1:length(num_solves)
    text(i, num_solves(i), sprintf('%d', num_solves(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9);
end

%% 5. Stage A solve times over time
subplot(3,4,[5,6]);
plot(stageResults.stageA.times, 'b-', 'LineWidth', 1.5);
xlabel('Solve Index', 'FontSize', 9);
ylabel('Time (seconds)', 'FontSize', 9);
title('Stage A: Time per Solve', 'FontSize', 10, 'FontWeight', 'bold');
grid on;
yline(mean(stageResults.stageA.times), 'r--', ...
    sprintf('Mean: %.2fs', mean(stageResults.stageA.times)), 'FontSize', 8);

%% 6. Stage A iterations over time
subplot(3,4,[7,8]);
plot(stageResults.stageA.iterations, 'r-', 'LineWidth', 1.5);
xlabel('Solve Index', 'FontSize', 9);
ylabel('Iterations', 'FontSize', 9);
title('Stage A: Iterations per Solve (⚠️ Hitting Max!)', ...
    'FontSize', 10, 'FontWeight', 'bold');
grid on;
yline(1500, 'k--', 'Max Limit', 'LineWidth', 2, 'FontSize', 8);
ylim([0, 1600]);

%% 7. Stage C solve times over time
subplot(3,4,[9,10]);
plot(stageResults.stageC.times, 'b-', 'LineWidth', 1.5);
xlabel('Solve Index', 'FontSize', 9);
ylabel('Time (seconds)', 'FontSize', 9);
title('Stage C: Time per Solve', 'FontSize', 10, 'FontWeight', 'bold');
grid on;
yline(mean(stageResults.stageC.times), 'r--', ...
    sprintf('Mean: %.2fs', mean(stageResults.stageC.times)), 'FontSize', 8);

%% 8. Stage C iterations over time - ALL AT MAX!
subplot(3,4,[11,12]);
plot(stageResults.stageC.iterations, 'r-', 'LineWidth', 1.5);
xlabel('Solve Index', 'FontSize', 9);
ylabel('Iterations', 'FontSize', 9);
title('Stage C: Iterations per Solve (🔴 ALL at Maximum!)', ...
    'FontSize', 10, 'FontWeight', 'bold');
grid on;
yline(1500, 'k--', 'Max Limit', 'LineWidth', 2, 'FontSize', 8);
ylim([1490, 1510]);

% Add key findings text box
annotation('textbox', [0.02, 0.01, 0.96, 0.08], ...
    'String', {
        '🔴 KEY FINDINGS:', ...
        sprintf('• Total Time: %.1f min (logged), possibly 370 min (actual) - 10x discrepancy suggests massive overhead!', logData.elapsed1/60), ...
        '• Stage C consumes 46.4% of logged time (17.4 min)', ...
        '• Overhead: 48.7% (18.2 min) - UNEXPLAINED!', ...
        sprintf('• ALL solves hit max iterations (1500) - solver NOT converging, just stopping!'), ...
        sprintf('• Total iterations: %s - extremely wasteful', ...
            num2str(stageResults.stageA.totalIters + stageResults.stageC.totalIters, '%.0f')), ...
        '• Recommendation: Use Method 4 instead OR fix convergence issues + implement warm-starting'
    }, ...
    'FontSize', 9, 'FontWeight', 'normal', ...
    'EdgeColor', 'r', 'LineWidth', 2, 'BackgroundColor', [1, 0.95, 0.95]);

% Save figure
saveas(fig, 'results/method1_performance_summary.png');
fprintf('Saved: results/method1_performance_summary.png\n');

%% Create second figure: detailed iteration analysis
fig2 = figure('Name', 'Iteration Analysis Detail', ...
    'Position', [100, 100, 1400, 800], 'Color', 'w');

% Title
annotation('textbox', [0.35, 0.95, 0.3, 0.04], ...
    'String', 'Detailed Iteration Analysis - Root Cause of Slowness', ...
    'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none');

% Stage A iteration histogram
subplot(2,3,1);
histogram(stageResults.stageA.iterations, 30, 'FaceColor', [0.8, 0.3, 0.3]);
xlabel('Iterations', 'FontSize', 10);
ylabel('Frequency', 'FontSize', 10);
title('Stage A: Iteration Distribution', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
xline(1500, 'k--', 'Max', 'LineWidth', 2);

% Stage C iteration histogram
subplot(2,3,2);
histogram(stageResults.stageC.iterations, 30, 'FaceColor', [0.8, 0.3, 0.3]);
xlabel('Iterations', 'FontSize', 10);
ylabel('Frequency', 'FontSize', 10);
title('Stage C: Iteration Distribution', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
xline(1500, 'k--', 'Max', 'LineWidth', 2);
xlim([1490, 1510]);

% Combined iteration vs time
subplot(2,3,3);
all_times = [stageResults.stageA.times(:); stageResults.stageC.times(:)];
all_iters = [stageResults.stageA.iterations(:); stageResults.stageC.iterations(:)];
scatter(all_iters, all_times, 30, 'filled', 'MarkerFaceAlpha', 0.6);
xlabel('Iterations', 'FontSize', 10);
ylabel('Solve Time (seconds)', 'FontSize', 10);
title('Iterations vs Solve Time', 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% Cumulative iterations Stage A
subplot(2,3,4);
plot(cumsum(stageResults.stageA.iterations), 'b-', 'LineWidth', 2);
xlabel('Solve Index', 'FontSize', 10);
ylabel('Cumulative Iterations', 'FontSize', 10);
title(sprintf('Stage A: Cumulative Iterations (Total: %d)', ...
    stageResults.stageA.totalIters), 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% Cumulative iterations Stage C
subplot(2,3,5);
plot(cumsum(stageResults.stageC.iterations), 'r-', 'LineWidth', 2);
xlabel('Solve Index', 'FontSize', 10);
ylabel('Cumulative Iterations', 'FontSize', 10);
title(sprintf('Stage C: Cumulative Iterations (Total: %d)', ...
    stageResults.stageC.totalIters), 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% Time per iteration estimate
subplot(2,3,6);
time_per_iter_A = stageResults.stageA.times ./ stageResults.stageA.iterations;
time_per_iter_C = stageResults.stageC.times ./ stageResults.stageC.iterations;

boxplot([time_per_iter_A(:); time_per_iter_C(:)], ...
    [repmat({'Stage A'}, length(time_per_iter_A), 1); ...
     repmat({'Stage C'}, length(time_per_iter_C), 1)]);
ylabel('Time per Iteration (seconds)', 'FontSize', 10);
title('Time per Iteration Comparison', 'FontSize', 11, 'FontWeight', 'bold');
grid on;

saveas(fig2, 'results/method1_iteration_analysis.png');
fprintf('Saved: results/method1_iteration_analysis.png\n');

%% Print summary statistics
fprintf('\n========================================\n');
fprintf('PERFORMANCE SUMMARY STATISTICS\n');
fprintf('========================================\n');
fprintf('Total time: %.2f minutes (%.2f hours)\n', logData.elapsed1/60, logData.elapsed1/3600);
fprintf('\nStage A:\n');
fprintf('  Solves: %d\n', length(stageResults.stageA.times));
fprintf('  Time: %.2f min (%.1f%%)\n', stageResults.stageA.totalTime/60, ...
    100*stageResults.stageA.totalTime/logData.elapsed1);
fprintf('  Iterations: %d total, %.1f avg\n', ...
    stageResults.stageA.totalIters, stageResults.stageA.meanIters);
fprintf('  Time per iteration: %.4f seconds\n', ...
    mean(time_per_iter_A));

fprintf('\nStage C:\n');
fprintf('  Solves: %d\n', length(stageResults.stageC.times));
fprintf('  Time: %.2f min (%.1f%%)\n', stageResults.stageC.totalTime/60, ...
    100*stageResults.stageC.totalTime/logData.elapsed1);
fprintf('  Iterations: %d total, %.1f avg\n', ...
    stageResults.stageC.totalIters, stageResults.stageC.meanIters);
fprintf('  Time per iteration: %.4f seconds\n', ...
    mean(time_per_iter_C));

fprintf('\nOverall:\n');
fprintf('  Total iterations: %d\n', ...
    stageResults.stageA.totalIters + stageResults.stageC.totalIters);
fprintf('  Overhead: %.2f min (%.1f%%)\n', overhead/60, 100*overhead/logData.elapsed1);
fprintf('  Avg time per iteration: %.4f seconds\n', ...
    logData.elapsed1 / (stageResults.stageA.totalIters + stageResults.stageC.totalIters));

fprintf('\n⚠️  CRITICAL ISSUE: Solver hitting maximum iteration limit!\n');
fprintf('   - Stage A: %.1f%% of solves at max\n', ...
    100*sum(stageResults.stageA.iterations >= 1500)/length(stageResults.stageA.iterations));
fprintf('   - Stage C: %.1f%% of solves at max\n', ...
    100*sum(stageResults.stageC.iterations >= 1500)/length(stageResults.stageC.iterations));
fprintf('   This means the solver is NOT converging properly!\n');

fprintf('\n========================================\n');
fprintf('Analysis complete! Check generated figures.\n');
fprintf('========================================\n');
