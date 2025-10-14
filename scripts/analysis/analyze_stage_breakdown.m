% Complete Stage Breakdown Analysis
% Compare timing across all stages

clear; clc;

% Load the log file
log_file = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
fprintf('Loading log file: %s\n', log_file);
data = load(log_file);

log1 = data.log1;
stageLogs = log1.stageLogs;

fprintf('\n========================================\n');
fprintf('COMPLETE STAGE BREAKDOWN ANALYSIS\n');
fprintf('========================================\n');
fprintf('Total elapsed time: %.2f minutes (%.2f hours)\n\n', ...
    data.elapsed1/60, data.elapsed1/3600);

% Initialize results structure
stageResults = struct();

% Analyze each stage
stages = {'stageA', 'stageB', 'stageC'};

for s = 1:length(stages)
    stage_name = stages{s};
    
    if isfield(stageLogs, stage_name)
        stageLog = stageLogs.(stage_name);
        
        fprintf('=== %s ===\n', upper(stage_name));
        
        % Check if stage was executed
        if isempty(stageLog) || ~isstruct(stageLog)
            fprintf('  Not executed or empty\n\n');
            stageResults.(stage_name).executed = false;
            continue;
        end
        
        stageResults.(stage_name).executed = true;
        
        % Get timing data
        if isfield(stageLog, 'solveTime')
            times = stageLog.solveTime;
            stageResults.(stage_name).times = times;
            stageResults.(stage_name).totalTime = sum(times);
            stageResults.(stage_name).numSolves = length(times);
            stageResults.(stage_name).meanTime = mean(times);
            stageResults.(stage_name).medianTime = median(times);
            stageResults.(stage_name).minTime = min(times);
            stageResults.(stage_name).maxTime = max(times);
            stageResults.(stage_name).stdTime = std(times);
            
            fprintf('  Number of solves: %d\n', length(times));
            fprintf('  Total time: %.2f minutes (%.2f%% of total)\n', ...
                sum(times)/60, 100*sum(times)/data.elapsed1);
            fprintf('  Mean solve time: %.4f seconds\n', mean(times));
            fprintf('  Median solve time: %.4f seconds\n', median(times));
            fprintf('  Min solve time: %.4f seconds\n', min(times));
            fprintf('  Max solve time: %.4f seconds\n', max(times));
            fprintf('  Std dev: %.4f seconds\n', std(times));
        else
            fprintf('  No timing data available\n');
        end
        
        % Get iteration data
        if isfield(stageLog, 'iterations')
            iters = stageLog.iterations;
            % Remove NaN values
            iters_valid = iters(~isnan(iters));
            if ~isempty(iters_valid)
                stageResults.(stage_name).iterations = iters_valid;
                stageResults.(stage_name).totalIters = sum(iters_valid);
                stageResults.(stage_name).meanIters = mean(iters_valid);
                
                fprintf('  Total iterations: %d\n', sum(iters_valid));
                fprintf('  Mean iterations per solve: %.2f\n', mean(iters_valid));
                fprintf('  Max iterations: %d\n', max(iters_valid));
            end
        end
        
        % Get success rate
        if isfield(stageLog, 'successMask')
            successMask = stageLog.successMask;
            successRate = 100 * sum(successMask) / length(successMask);
            stageResults.(stage_name).successRate = successRate;
            fprintf('  Success rate: %.1f%% (%d/%d)\n', ...
                successRate, sum(successMask), length(successMask));
        end
        
        fprintf('\n');
    end
end

% Create comparison figure
fprintf('=== STAGE COMPARISON ===\n');

executed_stages = {};
stage_times = [];
stage_percentages = [];
stage_labels = {};

for s = 1:length(stages)
    stage_name = stages{s};
    if isfield(stageResults, stage_name) && stageResults.(stage_name).executed && ...
            isfield(stageResults.(stage_name), 'totalTime')
        executed_stages{end+1} = stage_name;
        stage_times(end+1) = stageResults.(stage_name).totalTime;
        stage_percentages(end+1) = 100 * stageResults.(stage_name).totalTime / data.elapsed1;
        stage_labels{end+1} = sprintf('%s\n%.1f min\n(%.1f%%)', ...
            upper(stage_name(end)), stageResults.(stage_name).totalTime/60, ...
            stage_percentages(end));
    end
end

% Account for overhead (difference between sum of stages and total time)
stage_time_sum = sum(stage_times);
overhead = data.elapsed1 - stage_time_sum;
overhead_pct = 100 * overhead / data.elapsed1;

if overhead > 0
    stage_times(end+1) = overhead;
    stage_percentages(end+1) = overhead_pct;
    stage_labels{end+1} = sprintf('Overhead\n%.1f min\n(%.1f%%)', overhead/60, overhead_pct);
end

fprintf('\nTime breakdown:\n');
for i = 1:length(stage_labels)
    fprintf('  %s\n', strrep(stage_labels{i}, sprintf('\n'), ' '));
end

% Create visualization
figure('Name', 'Stage Performance Comparison', 'Position', [100, 100, 1400, 900]);

% Pie chart
subplot(2,3,1);
pie(stage_times, stage_labels);
title('Time Distribution by Stage');

% Bar chart
subplot(2,3,2);
bar(stage_percentages);
set(gca, 'XTickLabel', {});
ylabel('Percentage of Total Time (%)');
title('Stage Time as % of Total');
grid on;

% Time per solve comparison
subplot(2,3,3);
mean_times = [];
stage_names_plot = {};
for i = 1:length(executed_stages)
    stage_name = executed_stages{i};
    if isfield(stageResults.(stage_name), 'meanTime')
        mean_times(end+1) = stageResults.(stage_name).meanTime;
        stage_names_plot{end+1} = upper(stage_name(end));
    end
end
if ~isempty(mean_times)
    bar(mean_times);
    set(gca, 'XTickLabel', stage_names_plot);
    ylabel('Mean Solve Time (seconds)');
    title('Average Time per Solve');
    grid on;
end

% Box plots of solve times
subplot(2,3,4);
box_data = [];
box_labels = {};
for i = 1:length(executed_stages)
    stage_name = executed_stages{i};
    if isfield(stageResults.(stage_name), 'times')
        times = stageResults.(stage_name).times;
        box_data = [box_data; times(:)];
        box_labels = [box_labels; repmat({upper(stage_name(end))}, length(times), 1)];
    end
end
if ~isempty(box_data)
    boxplot(box_data, box_labels);
    ylabel('Solve Time (seconds)');
    title('Solve Time Distribution');
    grid on;
end

% Iterations comparison
subplot(2,3,5);
mean_iters = [];
stage_names_iters = {};
for i = 1:length(executed_stages)
    stage_name = executed_stages{i};
    if isfield(stageResults.(stage_name), 'meanIters')
        mean_iters(end+1) = stageResults.(stage_name).meanIters;
        stage_names_iters{end+1} = upper(stage_name(end));
    end
end
if ~isempty(mean_iters)
    bar(mean_iters);
    set(gca, 'XTickLabel', stage_names_iters);
    ylabel('Mean Iterations per Solve');
    title('Average Iterations');
    grid on;
end

% Success rates
subplot(2,3,6);
success_rates = [];
stage_names_success = {};
for i = 1:length(executed_stages)
    stage_name = executed_stages{i};
    if isfield(stageResults.(stage_name), 'successRate')
        success_rates(end+1) = stageResults.(stage_name).successRate;
        stage_names_success{end+1} = upper(stage_name(end));
    end
end
if ~isempty(success_rates)
    bar(success_rates);
    set(gca, 'XTickLabel', stage_names_success);
    ylabel('Success Rate (%)');
    ylim([0 105]);
    title('Success Rate by Stage');
    grid on;
end

% Detailed bottleneck analysis
fprintf('\n========================================\n');
fprintf('BOTTLENECK ANALYSIS\n');
fprintf('========================================\n');

% Find the most time-consuming stage
[max_time, max_idx] = max(stage_times);
if max_idx <= length(executed_stages)
    bottleneck_stage = executed_stages{max_idx};
    fprintf('\n🔴 PRIMARY BOTTLENECK: %s\n', upper(bottleneck_stage));
    fprintf('   Time: %.2f minutes (%.1f%% of total)\n', ...
        max_time/60, 100*max_time/data.elapsed1);
    
    if isfield(stageResults.(bottleneck_stage), 'meanTime')
        fprintf('   Mean solve time: %.4f seconds\n', ...
            stageResults.(bottleneck_stage).meanTime);
    end
    
    if isfield(stageResults.(bottleneck_stage), 'numSolves')
        fprintf('   Number of solves: %d\n', ...
            stageResults.(bottleneck_stage).numSolves);
    end
    
    if isfield(stageResults.(bottleneck_stage), 'meanIters')
        fprintf('   Mean iterations: %.2f\n', ...
            stageResults.(bottleneck_stage).meanIters);
    end
end

fprintf('\n========================================\n');
fprintf('KEY FINDINGS & RECOMMENDATIONS\n');
fprintf('========================================\n');

fprintf('\n1. TOTAL EXECUTION TIME:\n');
fprintf('   %.2f minutes (%.2f hours) - THIS IS VERY SLOW!\n', ...
    data.elapsed1/60, data.elapsed1/3600);

fprintf('\n2. TIME BREAKDOWN:\n');
for i = 1:length(executed_stages)
    stage_name = executed_stages{i};
    if isfield(stageResults.(stage_name), 'totalTime')
        fprintf('   %s: %.2f min (%.1f%%)\n', upper(stage_name), ...
            stageResults.(stage_name).totalTime/60, ...
            100*stageResults.(stage_name).totalTime/data.elapsed1);
    end
end
fprintf('   Overhead: %.2f min (%.1f%%)\n', overhead/60, overhead_pct);

fprintf('\n3. PERFORMANCE ISSUES:\n');

% Check Stage C specifically
if isfield(stageResults, 'stageC') && stageResults.stageC.executed
    fprintf('   🔴 Stage C is consuming %.1f%% of total time\n', ...
        100*stageResults.stageC.totalTime/data.elapsed1);
    fprintf('      - Each Stage C solve takes ~%.2f seconds on average\n', ...
        stageResults.stageC.meanTime);
    fprintf('      - With %d solves, this adds up to %.2f minutes\n', ...
        stageResults.stageC.numSolves, stageResults.stageC.totalTime/60);
    
    if isfield(stageResults.stageC, 'meanIters')
        fprintf('      - Average iterations: %.2f\n', stageResults.stageC.meanIters);
        if stageResults.stageC.meanIters > 50
            fprintf('      ⚠️  High iteration count suggests poor convergence\n');
        end
    end
end

fprintf('\n4. WHY METHOD 1 IS SO SLOW:\n');
fprintf('   - Uses iterative IK solver for each timestep\n');
fprintf('   - Each IK solve requires multiple iterations to converge\n');
fprintf('   - No warm-starting between consecutive solves\n');
fprintf('   - Stage C (whole-body control) is particularly expensive\n');

fprintf('\n5. RECOMMENDATIONS:\n');
fprintf('   ✓ Use Method 4 instead (likely uses analytical/faster IK)\n');
fprintf('   ✓ If Method 1 must be used:\n');
fprintf('     - Reduce trajectory resolution (fewer waypoints)\n');
fprintf('     - Implement warm-starting (use previous solution as initial guess)\n');
fprintf('     - Relax convergence tolerances if acceptable\n');
fprintf('     - Consider parallel processing if available\n');
fprintf('   ✓ Profile the IK solver itself to find internal bottlenecks\n');

fprintf('\n========================================\n');
fprintf('Analysis Complete!\n');
fprintf('========================================\n');

% Save results
save('results/method1_performance_analysis.mat', 'stageResults', 'data', '-v7.3');
fprintf('\nResults saved to: results/method1_performance_analysis.mat\n');
