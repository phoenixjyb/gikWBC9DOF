% Detailed Analysis of Method 1 Performance
% This script performs in-depth analysis of the log1 structure

clear; clc;

% Load the log file
log_file = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
fprintf('Loading log file: %s\n', log_file);
data = load(log_file);

fprintf('\n========================================\n');
fprintf('METHOD 1 PERFORMANCE ANALYSIS\n');
fprintf('========================================\n');
fprintf('\nTotal elapsed time: %.2f minutes (%.2f hours)\n', ...
    data.elapsed1/60, data.elapsed1/3600);

log1 = data.log1;

% Show all fields in log1
fprintf('\n=== Fields in log1 structure ===\n');
log_fields = fieldnames(log1);
for i = 1:length(log_fields)
    fprintf('%d. %s\n', i, log_fields{i});
end

% Analyze key fields
fprintf('\n=== Key Metrics ===\n');

% 1. Number of waypoints/samples
if isfield(log1, 'completedWaypoints')
    nWaypoints = log1.completedWaypoints;
    fprintf('Completed waypoints: %d\n', nWaypoints);
end

if isfield(log1, 'timestamps')
    nTimesteps = length(log1.timestamps);
    fprintf('Number of timesteps: %d\n', nTimesteps);
end

if isfield(log1, 'rateHz')
    fprintf('Rate: %.2f Hz\n', log1.rateHz);
end

% 2. Success rate
if isfield(log1, 'successMask')
    successMask = log1.successMask;
    successRate = 100 * sum(successMask) / length(successMask);
    fprintf('\nSuccess rate: %.1f%% (%d/%d successful)\n', ...
        successRate, sum(successMask), length(successMask));
    fprintf('Failed solutions: %d\n', sum(~successMask));
end

% 3. Iterations analysis
if isfield(log1, 'iterations')
    iterations = log1.iterations;
    fprintf('\n=== Iteration Statistics ===\n');
    fprintf('Total iterations: %d\n', sum(iterations));
    fprintf('Mean iterations per solve: %.2f\n', mean(iterations));
    fprintf('Median iterations per solve: %.2f\n', median(iterations));
    fprintf('Max iterations per solve: %d\n', max(iterations));
    fprintf('Min iterations per solve: %d\n', min(iterations));
    fprintf('Std dev: %.2f\n', std(iterations));
    
    % Time per iteration estimate
    avgTimePerSolve = data.elapsed1 / length(iterations);
    avgTimePerIter = avgTimePerSolve / mean(iterations);
    fprintf('\nEstimated time per solve: %.4f seconds\n', avgTimePerSolve);
    fprintf('Estimated time per iteration: %.6f seconds\n', avgTimePerIter);
    
    % Create figure for iterations
    figure('Name', 'Iteration Analysis', 'Position', [100, 100, 1200, 800]);
    
    subplot(2,3,1);
    plot(iterations, 'b-', 'LineWidth', 1);
    xlabel('Solve Index');
    ylabel('Iterations');
    title('Iterations per Solve');
    grid on;
    
    subplot(2,3,2);
    histogram(iterations, 50);
    xlabel('Iterations');
    ylabel('Frequency');
    title('Iteration Distribution');
    grid on;
    
    subplot(2,3,3);
    boxplot(iterations);
    ylabel('Iterations');
    title('Iteration Box Plot');
    grid on;
    
    % Check correlation with success
    if isfield(log1, 'successMask')
        subplot(2,3,4);
        hold on;
        plot(find(successMask), iterations(successMask), 'g.', 'MarkerSize', 10);
        plot(find(~successMask), iterations(~successMask), 'r.', 'MarkerSize', 10);
        xlabel('Solve Index');
        ylabel('Iterations');
        title('Iterations: Success vs Failed');
        legend('Success', 'Failed');
        grid on;
        hold off;
        
        fprintf('\nIterations for successful solves: %.2f (mean)\n', ...
            mean(iterations(successMask)));
        fprintf('Iterations for failed solves: %.2f (mean)\n', ...
            mean(iterations(~successMask)));
    end
    
    % Cumulative iterations
    subplot(2,3,5);
    plot(cumsum(iterations), 'b-', 'LineWidth', 1.5);
    xlabel('Solve Index');
    ylabel('Cumulative Iterations');
    title('Cumulative Iterations Over Time');
    grid on;
    
    % Moving average
    subplot(2,3,6);
    window = max(1, floor(length(iterations)/20));
    moving_avg = movmean(iterations, window);
    plot(moving_avg, 'r-', 'LineWidth', 1.5);
    xlabel('Solve Index');
    ylabel('Iterations (Moving Avg)');
    title(sprintf('Moving Average (window=%d)', window));
    grid on;
end

% 4. Exit flags analysis
if isfield(log1, 'exitFlags')
    exitFlags = log1.exitFlags;
    fprintf('\n=== Exit Flag Analysis ===\n');
    uniqueFlags = unique(exitFlags);
    fprintf('Unique exit flags: %s\n', mat2str(uniqueFlags));
    for i = 1:length(uniqueFlags)
        flag = uniqueFlags(i);
        count = sum(exitFlags == flag);
        fprintf('  Exit flag %d: %d occurrences (%.1f%%)\n', ...
            flag, count, 100*count/length(exitFlags));
    end
end

% 5. Constraint violation analysis
if isfield(log1, 'constraintViolationMax')
    violations = log1.constraintViolationMax;
    fprintf('\n=== Constraint Violation Statistics ===\n');
    fprintf('Max constraint violation: %.6e\n', max(violations));
    fprintf('Mean constraint violation: %.6e\n', mean(violations));
    fprintf('Median constraint violation: %.6e\n', median(violations));
    
    % Create figure for violations
    figure('Name', 'Constraint Violations', 'Position', [150, 150, 1200, 400]);
    
    subplot(1,3,1);
    semilogy(violations, 'b-');
    xlabel('Solve Index');
    ylabel('Max Constraint Violation (log scale)');
    title('Constraint Violations Over Time');
    grid on;
    
    subplot(1,3,2);
    histogram(log10(violations + eps), 50);
    xlabel('log10(Constraint Violation)');
    ylabel('Frequency');
    title('Constraint Violation Distribution');
    grid on;
    
    % Check if violations correlate with failed solves
    if isfield(log1, 'successMask')
        subplot(1,3,3);
        hold on;
        semilogy(find(successMask), violations(successMask), 'g.', 'MarkerSize', 10);
        semilogy(find(~successMask), violations(~successMask), 'r.', 'MarkerSize', 10);
        xlabel('Solve Index');
        ylabel('Max Constraint Violation (log scale)');
        title('Violations: Success vs Failed');
        legend('Success', 'Failed');
        grid on;
        hold off;
    end
end

% 6. Check for timing breakdown if available
timing_fields = {'solveTime', 'tSolve', 'computeTime', 'ikTime', 'optimTime'};
fprintf('\n=== Timing Breakdown (if available) ===\n');
found_timing = false;
for i = 1:length(timing_fields)
    if isfield(log1, timing_fields{i})
        found_timing = true;
        timing_data = log1.(timing_fields{i});
        fprintf('\n%s:\n', timing_fields{i});
        fprintf('  Total: %.2f minutes\n', sum(timing_data)/60);
        fprintf('  Mean: %.4f seconds\n', mean(timing_data));
        fprintf('  Median: %.4f seconds\n', median(timing_data));
        fprintf('  Max: %.4f seconds\n', max(timing_data));
    end
end
if ~found_timing
    fprintf('No detailed timing breakdown found in log.\n');
end

% 7. Check solutionInfo if it contains timing
if isfield(log1, 'solutionInfo')
    fprintf('\n=== Solution Info Analysis ===\n');
    solutionInfo = log1.solutionInfo;
    
    if isstruct(solutionInfo) && length(solutionInfo) > 0
        fprintf('Solution info is a struct array with %d elements\n', length(solutionInfo));
        fprintf('Fields in solutionInfo:\n');
        infoFields = fieldnames(solutionInfo(1));
        for i = 1:length(infoFields)
            fprintf('  - %s\n', infoFields{i});
        end
        
        % Try to extract timing if available
        if isfield(solutionInfo, 'solveTime') || isfield(solutionInfo, 'time')
            if isfield(solutionInfo, 'solveTime')
                solveTimes = [solutionInfo.solveTime];
            else
                solveTimes = [solutionInfo.time];
            end
            
            fprintf('\n=== Individual Solve Times ===\n');
            fprintf('Total solve time: %.2f minutes (%.2f hours)\n', ...
                sum(solveTimes)/60, sum(solveTimes)/3600);
            fprintf('Mean solve time: %.4f seconds\n', mean(solveTimes));
            fprintf('Median solve time: %.4f seconds\n', median(solveTimes));
            fprintf('Max solve time: %.4f seconds\n', max(solveTimes));
            fprintf('Min solve time: %.4f seconds\n', min(solveTimes));
            
            % Create figure for solve times
            figure('Name', 'Solve Times', 'Position', [200, 200, 1200, 600]);
            
            subplot(2,3,1);
            plot(solveTimes, 'b-', 'LineWidth', 1);
            xlabel('Solve Index');
            ylabel('Solve Time (seconds)');
            title('Time per Solve');
            grid on;
            
            subplot(2,3,2);
            histogram(solveTimes, 50);
            xlabel('Solve Time (seconds)');
            ylabel('Frequency');
            title('Solve Time Distribution');
            grid on;
            
            subplot(2,3,3);
            plot(cumsum(solveTimes)/60, 'r-', 'LineWidth', 1.5);
            xlabel('Solve Index');
            ylabel('Cumulative Time (minutes)');
            title('Cumulative Solve Time');
            grid on;
            
            subplot(2,3,4);
            boxplot(solveTimes);
            ylabel('Solve Time (seconds)');
            title('Solve Time Box Plot');
            grid on;
            
            % Check for outliers
            Q1 = quantile(solveTimes, 0.25);
            Q3 = quantile(solveTimes, 0.75);
            IQR = Q3 - Q1;
            outlier_threshold = Q3 + 1.5*IQR;
            outliers = solveTimes > outlier_threshold;
            
            subplot(2,3,5);
            hold on;
            plot(find(~outliers), solveTimes(~outliers), 'b.', 'MarkerSize', 10);
            plot(find(outliers), solveTimes(outliers), 'r.', 'MarkerSize', 15);
            yline(outlier_threshold, 'k--', 'LineWidth', 1.5);
            xlabel('Solve Index');
            ylabel('Solve Time (seconds)');
            title('Outlier Detection');
            legend('Normal', 'Outlier', 'Threshold');
            grid on;
            hold off;
            
            fprintf('\nOutlier analysis:\n');
            fprintf('  Outliers (>Q3+1.5*IQR): %d (%.1f%%)\n', ...
                sum(outliers), 100*sum(outliers)/length(outliers));
            fprintf('  Mean time for outliers: %.4f seconds\n', mean(solveTimes(outliers)));
            fprintf('  Time spent in outliers: %.2f minutes (%.1f%% of total)\n', ...
                sum(solveTimes(outliers))/60, ...
                100*sum(solveTimes(outliers))/sum(solveTimes));
            
            % Moving average
            subplot(2,3,6);
            window = max(1, floor(length(solveTimes)/20));
            moving_avg = movmean(solveTimes, window);
            plot(moving_avg, 'g-', 'LineWidth', 1.5);
            xlabel('Solve Index');
            ylabel('Solve Time (seconds, Moving Avg)');
            title(sprintf('Moving Average (window=%d)', window));
            grid on;
        end
    end
end

% Summary and recommendations
fprintf('\n========================================\n');
fprintf('SUMMARY AND POTENTIAL ISSUES\n');
fprintf('========================================\n');

fprintf('\n1. OVERALL PERFORMANCE:\n');
fprintf('   - Total time: %.2f minutes (%.2f hours)\n', data.elapsed1/60, data.elapsed1/3600);
if exist('nTimesteps', 'var')
    fprintf('   - Number of solves: %d\n', nTimesteps);
    fprintf('   - Average time per solve: %.4f seconds\n', data.elapsed1/nTimesteps);
end

if exist('iterations', 'var')
    fprintf('\n2. ITERATION ANALYSIS:\n');
    fprintf('   - Total iterations: %d\n', sum(iterations));
    fprintf('   - Average iterations per solve: %.2f\n', mean(iterations));
    if mean(iterations) > 100
        fprintf('   ⚠️  WARNING: High average iterations (>100) suggests:\n');
        fprintf('      - Poor initial guess\n');
        fprintf('      - Difficult optimization problem\n');
        fprintf('      - Need for better convergence criteria\n');
    end
end

if exist('successRate', 'var')
    fprintf('\n3. SUCCESS RATE:\n');
    fprintf('   - Success rate: %.1f%%\n', successRate);
    if successRate < 95
        fprintf('   ⚠️  WARNING: Low success rate (<95%%) indicates:\n');
        fprintf('      - Infeasible constraints\n');
        fprintf('      - Poor solver settings\n');
        fprintf('      - Numerical issues\n');
    end
end

fprintf('\n4. RECOMMENDATIONS:\n');
fprintf('   - Consider using Method 4 (appears to be much faster)\n');
fprintf('   - Review optimization solver settings\n');
fprintf('   - Check if initial guess can be improved\n');
fprintf('   - Consider relaxing constraints if appropriate\n');
fprintf('   - Profile individual solver calls to find bottlenecks\n');

fprintf('\n========================================\n');
fprintf('Analysis complete!\n');
fprintf('========================================\n');
