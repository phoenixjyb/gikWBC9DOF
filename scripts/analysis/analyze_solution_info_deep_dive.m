% Deep Dive into Solution Info
% Extract detailed timing from solutionInfo structure

clear; clc;

% Load the log file
log_file = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
fprintf('Loading log file: %s\n', log_file);
data = load(log_file);

log1 = data.log1;

fprintf('\n========================================\n');
fprintf('DEEP DIVE: SOLUTION INFO STRUCTURE\n');
fprintf('========================================\n');

if isfield(log1, 'solutionInfo')
    solutionInfo = log1.solutionInfo;
    
    fprintf('\nSolutionInfo dimensions: %s\n', mat2str(size(solutionInfo)));
    fprintf('Class: %s\n', class(solutionInfo));
    
    if ~isempty(solutionInfo)
        % If it's a struct array, examine first element
        if isstruct(solutionInfo)
            fprintf('\nFields in solutionInfo(1):\n');
            fields = fieldnames(solutionInfo(1));
            for i = 1:length(fields)
                field_name = fields{i};
                field_value = solutionInfo(1).(field_name);
                fprintf('  %d. %-30s [%s, size: %s]\n', i, field_name, ...
                    class(field_value), mat2str(size(field_value)));
                
                % If numeric and small, show value
                if isnumeric(field_value) && numel(field_value) <= 10
                    fprintf('      Value: %s\n', mat2str(field_value));
                end
            end
            
            % Extract timing information from all elements
            fprintf('\n=== Extracting Data from All Solution Infos ===\n');
            nSolutions = length(solutionInfo);
            fprintf('Total number of solution records: %d\n', nSolutions);
            
            % Check for various timing fields
            timing_fields = {'time', 'elapsed', 'solveTime', 'duration', ...
                            'computeTime', 'executionTime', 'wallTime'};
            
            for t = 1:length(timing_fields)
                if isfield(solutionInfo, timing_fields{t})
                    fprintf('\nFound timing field: %s\n', timing_fields{t});
                    try
                        times = [solutionInfo.(timing_fields{t})];
                        fprintf('  Total time: %.2f minutes\n', sum(times)/60);
                        fprintf('  Mean: %.4f seconds\n', mean(times));
                        fprintf('  Median: %.4f seconds\n', median(times));
                        fprintf('  Min: %.4f seconds\n', min(times));
                        fprintf('  Max: %.4f seconds\n', max(times));
                        
                        % Plot
                        figure('Name', sprintf('Timing: %s', timing_fields{t}));
                        subplot(2,2,1);
                        plot(times, 'b-', 'LineWidth', 1.5);
                        xlabel('Solve Index');
                        ylabel('Time (seconds)');
                        title(sprintf('%s per Solve', timing_fields{t}));
                        grid on;
                        
                        subplot(2,2,2);
                        histogram(times, 50);
                        xlabel('Time (seconds)');
                        ylabel('Frequency');
                        title('Distribution');
                        grid on;
                        
                        subplot(2,2,3);
                        plot(cumsum(times)/60, 'r-', 'LineWidth', 1.5);
                        xlabel('Solve Index');
                        ylabel('Cumulative Time (minutes)');
                        title('Cumulative Time');
                        grid on;
                        
                        subplot(2,2,4);
                        window = max(10, floor(length(times)/20));
                        moving_avg = movmean(times, window);
                        plot(moving_avg, 'g-', 'LineWidth', 1.5);
                        xlabel('Solve Index');
                        ylabel('Moving Average (seconds)');
                        title(sprintf('Moving Avg (window=%d)', window));
                        grid on;
                    catch e
                        fprintf('  Error extracting: %s\n', e.message);
                    end
                end
            end
            
            % Check for iteration counts
            iter_fields = {'iterations', 'numIterations', 'iter', 'nIter'};
            for t = 1:length(iter_fields)
                if isfield(solutionInfo, iter_fields{t})
                    fprintf('\nFound iteration field: %s\n', iter_fields{t});
                    try
                        iters = [solutionInfo.(iter_fields{t})];
                        fprintf('  Total iterations: %d\n', sum(iters));
                        fprintf('  Mean: %.2f\n', mean(iters));
                        fprintf('  Median: %.2f\n', median(iters));
                        fprintf('  Min: %d\n', min(iters));
                        fprintf('  Max: %d\n', max(iters));
                    catch e
                        fprintf('  Error extracting: %s\n', e.message);
                    end
                end
            end
            
            % Show sample of first few records
            fprintf('\n=== Sample Records (first 5) ===\n');
            for i = 1:min(5, nSolutions)
                fprintf('\nRecord %d:\n', i);
                for f = 1:length(fields)
                    field_name = fields{f};
                    field_value = solutionInfo(i).(field_name);
                    if isnumeric(field_value) && numel(field_value) <= 10
                        fprintf('  %s: %s\n', field_name, mat2str(field_value));
                    elseif ischar(field_value)
                        fprintf('  %s: %s\n', field_name, field_value);
                    else
                        fprintf('  %s: [%s, %s]\n', field_name, ...
                            class(field_value), mat2str(size(field_value)));
                    end
                end
            end
            
        elseif iscell(solutionInfo)
            fprintf('\nSolutionInfo is a cell array\n');
            fprintf('Number of cells: %d\n', numel(solutionInfo));
            
            % Examine first non-empty cell
            for i = 1:min(10, numel(solutionInfo))
                if ~isempty(solutionInfo{i})
                    fprintf('\nFirst non-empty cell (index %d):\n', i);
                    if isstruct(solutionInfo{i})
                        fields = fieldnames(solutionInfo{i});
                        for f = 1:length(fields)
                            fprintf('  %s\n', fields{f});
                        end
                    end
                    break;
                end
            end
        end
    else
        fprintf('solutionInfo is empty!\n');
    end
else
    fprintf('No solutionInfo field found!\n');
end

% Check the stageLogs field as well
fprintf('\n========================================\n');
fprintf('CHECKING STAGE LOGS\n');
fprintf('========================================\n');

if isfield(log1, 'stageLogs')
    stageLogs = log1.stageLogs;
    fprintf('\nstageLogs dimensions: %s\n', mat2str(size(stageLogs)));
    fprintf('Class: %s\n', class(stageLogs));
    
    if isstruct(stageLogs) && ~isempty(stageLogs)
        fprintf('\nFields in stageLogs:\n');
        stageFields = fieldnames(stageLogs);
        for i = 1:length(stageFields)
            fprintf('  %d. %s\n', i, stageFields{i});
        end
        
        % Check for Stage C logs (likely the bottleneck)
        if isfield(stageLogs, 'stageC')
            fprintf('\n=== Stage C Analysis ===\n');
            stageCLogs = stageLogs.stageC;
            
            if isstruct(stageCLogs) && ~isempty(stageCLogs)
                fprintf('Stage C log fields:\n');
                stageCFields = fieldnames(stageCLogs);
                for i = 1:length(stageCFields)
                    field_name = stageCFields{i};
                    field_value = stageCLogs.(field_name);
                    fprintf('  %s: [%s, size: %s]\n', field_name, ...
                        class(field_value), mat2str(size(field_value)));
                end
                
                % Look for timing in Stage C
                if isfield(stageCLogs, 'solveTime') || isfield(stageCLogs, 'time')
                    if isfield(stageCLogs, 'solveTime')
                        stageCTimes = stageCLogs.solveTime;
                        timing_name = 'solveTime';
                    else
                        stageCTimes = stageCLogs.time;
                        timing_name = 'time';
                    end
                    
                    fprintf('\n⚠️  FOUND STAGE C TIMING DATA! ⚠️\n');
                    fprintf('Stage C %s:\n', timing_name);
                    fprintf('  Total time: %.2f minutes (%.2f hours)\n', ...
                        sum(stageCTimes)/60, sum(stageCTimes)/3600);
                    fprintf('  Number of solves: %d\n', length(stageCTimes));
                    fprintf('  Mean: %.4f seconds per solve\n', mean(stageCTimes));
                    fprintf('  Median: %.4f seconds per solve\n', median(stageCTimes));
                    fprintf('  Min: %.4f seconds\n', min(stageCTimes));
                    fprintf('  Max: %.4f seconds\n', max(stageCTimes));
                    fprintf('  Std dev: %.4f seconds\n', std(stageCTimes));
                    
                    % Compare to total time
                    fprintf('\n  Percentage of total time: %.1f%%\n', ...
                        100*sum(stageCTimes)/data.elapsed1);
                    
                    % Create comprehensive figure
                    figure('Name', 'Stage C Performance Analysis', 'Position', [50, 50, 1400, 900]);
                    
                    subplot(3,3,1);
                    plot(stageCTimes, 'b-', 'LineWidth', 1.5);
                    xlabel('Solve Index');
                    ylabel('Solve Time (seconds)');
                    title('Stage C: Time per Solve');
                    grid on;
                    
                    subplot(3,3,2);
                    histogram(stageCTimes, 50);
                    xlabel('Solve Time (seconds)');
                    ylabel('Frequency');
                    title('Stage C: Time Distribution');
                    grid on;
                    
                    subplot(3,3,3);
                    plot(cumsum(stageCTimes)/60, 'r-', 'LineWidth', 2);
                    xlabel('Solve Index');
                    ylabel('Cumulative Time (minutes)');
                    title('Stage C: Cumulative Time');
                    grid on;
                    
                    subplot(3,3,4);
                    boxplot(stageCTimes);
                    ylabel('Solve Time (seconds)');
                    title('Stage C: Box Plot');
                    grid on;
                    
                    subplot(3,3,5);
                    qqplot(stageCTimes);
                    title('Stage C: Q-Q Plot');
                    
                    subplot(3,3,6);
                    window = max(10, floor(length(stageCTimes)/20));
                    moving_avg = movmean(stageCTimes, window);
                    plot(moving_avg, 'g-', 'LineWidth', 2);
                    xlabel('Solve Index');
                    ylabel('Moving Average (seconds)');
                    title(sprintf('Stage C: Moving Avg (window=%d)', window));
                    grid on;
                    
                    % Detect outliers
                    subplot(3,3,7);
                    Q1 = quantile(stageCTimes, 0.25);
                    Q3 = quantile(stageCTimes, 0.75);
                    IQR = Q3 - Q1;
                    outlier_threshold = Q3 + 1.5*IQR;
                    outliers = stageCTimes > outlier_threshold;
                    hold on;
                    plot(find(~outliers), stageCTimes(~outliers), 'b.', 'MarkerSize', 8);
                    plot(find(outliers), stageCTimes(outliers), 'r.', 'MarkerSize', 15);
                    yline(outlier_threshold, 'k--', 'LineWidth', 2);
                    xlabel('Solve Index');
                    ylabel('Solve Time (seconds)');
                    title('Stage C: Outlier Detection');
                    legend('Normal', 'Outliers', 'Threshold', 'Location', 'best');
                    grid on;
                    hold off;
                    
                    fprintf('\nOutlier Analysis:\n');
                    fprintf('  Outliers: %d (%.1f%%)\n', sum(outliers), 100*sum(outliers)/length(outliers));
                    if sum(outliers) > 0
                        fprintf('  Mean outlier time: %.4f seconds\n', mean(stageCTimes(outliers)));
                        fprintf('  Time in outliers: %.2f minutes (%.1f%% of Stage C total)\n', ...
                            sum(stageCTimes(outliers))/60, 100*sum(stageCTimes(outliers))/sum(stageCTimes));
                    end
                    
                    % Time series analysis
                    subplot(3,3,8);
                    scatter(1:length(stageCTimes), stageCTimes, 20, stageCTimes, 'filled');
                    colorbar;
                    xlabel('Solve Index');
                    ylabel('Solve Time (seconds)');
                    title('Stage C: Time Series (colored)');
                    grid on;
                    
                    % Percentiles
                    subplot(3,3,9);
                    percentiles = [50, 75, 90, 95, 99];
                    pct_values = prctile(stageCTimes, percentiles);
                    bar(pct_values);
                    set(gca, 'XTickLabel', arrayfun(@(x) sprintf('%dth', x), percentiles, 'UniformOutput', false));
                    ylabel('Time (seconds)');
                    title('Stage C: Percentiles');
                    grid on;
                    
                    fprintf('\nPercentiles:\n');
                    for i = 1:length(percentiles)
                        fprintf('  %dth: %.4f seconds\n', percentiles(i), pct_values(i));
                    end
                end
            end
        end
    end
end

fprintf('\n========================================\n');
fprintf('Analysis Complete!\n');
fprintf('========================================\n');
