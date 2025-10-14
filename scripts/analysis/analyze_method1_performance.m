% Analyze Method 1 Performance Log
% This script analyzes the log file to identify performance bottlenecks

clear; clc;

% Load the log file
log_file = 'results/20251013_151157_method_comparison/log_method1_ppForIk.mat';
fprintf('Loading log file: %s\n', log_file);
data = load(log_file);

% Display available fields
fprintf('\n=== Available Fields in Log ===\n');
disp(fieldnames(data));

% Display basic information about each field
fields = fieldnames(data);
for i = 1:length(fields)
    field_name = fields{i};
    field_value = data.(field_name);
    fprintf('\nField: %s\n', field_name);
    fprintf('  Class: %s\n', class(field_value));
    fprintf('  Size: %s\n', mat2str(size(field_value)));
    
    % If it's numeric and reasonable size, show statistics
    if isnumeric(field_value) && numel(field_value) < 1000 && numel(field_value) > 0
        fprintf('  Min: %.6f, Max: %.6f, Mean: %.6f\n', ...
            min(field_value(:)), max(field_value(:)), mean(field_value(:)));
    end
    
    % If it's a struct, show its fields
    if isstruct(field_value)
        fprintf('  Struct fields:\n');
        subfields = fieldnames(field_value);
        for j = 1:min(10, length(subfields))
            fprintf('    - %s\n', subfields{j});
        end
        if length(subfields) > 10
            fprintf('    ... and %d more fields\n', length(subfields) - 10);
        end
    end
    
    % If it's a cell array, show dimensions
    if iscell(field_value)
        fprintf('  Cell array with %d elements\n', numel(field_value));
        if numel(field_value) > 0 && numel(field_value) <= 5
            for j = 1:numel(field_value)
                fprintf('    Cell %d: %s, size %s\n', j, ...
                    class(field_value{j}), mat2str(size(field_value{j})));
            end
        end
    end
end

% Look for timing-related fields
fprintf('\n=== Timing Analysis ===\n');

% Check for common timing field names
timing_fields = {'timing', 'time', 'elapsed', 'duration', 'iterTime', ...
                 'solveTime', 'computeTime', 'tSolve', 'tCompute'};

for i = 1:length(timing_fields)
    if isfield(data, timing_fields{i})
        timing_data = data.(timing_fields{i});
        fprintf('\nFound timing field: %s\n', timing_fields{i});
        
        if isnumeric(timing_data) && numel(timing_data) > 0
            fprintf('  Total time: %.2f minutes (%.2f hours)\n', ...
                sum(timing_data(:))/60, sum(timing_data(:))/3600);
            fprintf('  Number of samples: %d\n', numel(timing_data));
            fprintf('  Mean time per sample: %.4f seconds\n', mean(timing_data(:)));
            fprintf('  Median time per sample: %.4f seconds\n', median(timing_data(:)));
            fprintf('  Max time per sample: %.4f seconds\n', max(timing_data(:)));
            fprintf('  Min time per sample: %.4f seconds\n', min(timing_data(:)));
            fprintf('  Std dev: %.4f seconds\n', std(timing_data(:)));
            
            % Show histogram of timing
            figure('Name', sprintf('Timing Distribution: %s', timing_fields{i}));
            histogram(timing_data(:), 50);
            xlabel('Time (seconds)');
            ylabel('Frequency');
            title(sprintf('Distribution of %s', timing_fields{i}));
            grid on;
            
            % Check for outliers
            Q1 = quantile(timing_data(:), 0.25);
            Q3 = quantile(timing_data(:), 0.75);
            IQR = Q3 - Q1;
            outlier_threshold = Q3 + 1.5*IQR;
            outliers = timing_data(:) > outlier_threshold;
            fprintf('  Number of outliers (>Q3+1.5*IQR): %d (%.1f%%)\n', ...
                sum(outliers), 100*sum(outliers)/numel(timing_data));
            
            if sum(outliers) > 0
                fprintf('  Mean time for outliers: %.4f seconds\n', ...
                    mean(timing_data(outliers)));
                fprintf('  Total time spent in outliers: %.2f minutes\n', ...
                    sum(timing_data(outliers))/60);
            end
        end
    end
end

% Check for iteration or step count
fprintf('\n=== Iteration/Step Analysis ===\n');
iteration_fields = {'nIter', 'iterations', 'numIter', 'iter', 'steps', 'N'};
for i = 1:length(iteration_fields)
    if isfield(data, iteration_fields{i})
        iter_data = data.(iteration_fields{i});
        fprintf('\nFound iteration field: %s\n', iteration_fields{i});
        if isnumeric(iter_data)
            fprintf('  Value: %s\n', mat2str(iter_data));
        end
    end
end

% Check for convergence-related data
fprintf('\n=== Convergence Analysis ===\n');
conv_fields = {'converged', 'success', 'exitflag', 'status', 'cost', 'residual', 'error'};
for i = 1:length(conv_fields)
    if isfield(data, conv_fields{i})
        conv_data = data.(conv_fields{i});
        fprintf('\nFound convergence field: %s\n', conv_fields{i});
        
        if isnumeric(conv_data) && numel(conv_data) > 0
            if numel(conv_data) <= 10
                fprintf('  Values: %s\n', mat2str(conv_data));
            else
                fprintf('  Size: %s\n', mat2str(size(conv_data)));
                fprintf('  Min: %.6e, Max: %.6e, Mean: %.6e\n', ...
                    min(conv_data(:)), max(conv_data(:)), mean(conv_data(:)));
                
                % If it's a binary flag, count successes
                if all(ismember(unique(conv_data(:)), [0, 1]))
                    fprintf('  Success rate: %.1f%% (%d/%d)\n', ...
                        100*sum(conv_data(:))/numel(conv_data), ...
                        sum(conv_data(:)), numel(conv_data));
                end
            end
        end
    end
end

% Try to create a comprehensive timing plot if we have the data
fprintf('\n=== Creating Comprehensive Analysis ===\n');

% Find the main timing array (usually the largest numeric array)
max_size = 0;
main_timing_field = '';
for i = 1:length(fields)
    field_name = fields{i};
    field_value = data.(field_name);
    if isnumeric(field_value) && numel(field_value) > max_size
        max_size = numel(field_value);
        main_timing_field = field_name;
    end
end

if ~isempty(main_timing_field)
    fprintf('Main timing field appears to be: %s (size: %d)\n', ...
        main_timing_field, max_size);
    
    timing_data = data.(main_timing_field);
    
    % Create cumulative time plot
    figure('Name', 'Cumulative Time Analysis');
    subplot(2,2,1);
    plot(cumsum(timing_data(:))/60);
    xlabel('Sample Index');
    ylabel('Cumulative Time (minutes)');
    title('Cumulative Time Over Samples');
    grid on;
    
    % Time per sample plot
    subplot(2,2,2);
    plot(timing_data(:));
    xlabel('Sample Index');
    ylabel('Time per Sample (seconds)');
    title('Time per Sample');
    grid on;
    
    % Box plot
    subplot(2,2,3);
    boxplot(timing_data(:));
    ylabel('Time (seconds)');
    title('Time Distribution (Box Plot)');
    grid on;
    
    % Moving average
    subplot(2,2,4);
    window_size = max(1, floor(length(timing_data)/50));
    moving_avg = movmean(timing_data(:), window_size);
    plot(moving_avg);
    xlabel('Sample Index');
    ylabel('Moving Average Time (seconds)');
    title(sprintf('Moving Average (window=%d)', window_size));
    grid on;
end

fprintf('\n=== Analysis Complete ===\n');
fprintf('Total execution time: 370 minutes (6.17 hours)\n');
fprintf('\nLook for:\n');
fprintf('  1. Large mean/median times per sample\n');
fprintf('  2. High number of iterations/samples\n');
fprintf('  3. Outliers that contribute significantly to total time\n');
fprintf('  4. Poor convergence rates\n');
fprintf('  5. Increasing time trends over samples\n');
