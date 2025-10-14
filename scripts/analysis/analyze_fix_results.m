% Analyze results after implementing the fix
% Check if Stage C still has second-half drift

clearvars; clc;

%% Load the latest results
resultsDir = 'results/20251014_164148_fresh_no_discs';
logFile = fullfile(resultsDir, 'log_staged_ppForIk.mat');

if ~isfile(logFile)
    error('Log file not found: %s', logFile);
end

fprintf('Loading results from: %s\n\n', resultsDir);
data = load(logFile);

%% Extract Stage C data
if isfield(data, 'logC')
    logC = data.logC;
    fprintf('=== Stage C Results ===\n');
    
    % Position errors
    posErr = logC.positionErrorNorm * 1000; % Convert to mm
    
    fprintf('Overall Performance:\n');
    fprintf('  Mean EE Error: %.1f mm\n', mean(posErr));
    fprintf('  Max EE Error: %.1f mm\n', max(posErr));
    fprintf('  Median EE Error: %.1f mm\n', median(posErr));
    
    % First half vs second half
    N = length(posErr);
    firstHalf = posErr(1:floor(N/2));
    secondHalf = posErr(floor(N/2)+1:end);
    
    fprintf('\nFirst Half vs Second Half:\n');
    fprintf('  First half (1-%d):  Mean=%.1fmm, Max=%.1fmm\n', ...
        floor(N/2), mean(firstHalf), max(firstHalf));
    fprintf('  Second half (%d-%d): Mean=%.1fmm, Max=%.1fmm\n', ...
        floor(N/2)+1, N, mean(secondHalf), max(secondHalf));
    
    % Failure analysis
    failures = posErr > 500;
    fprintf('\nFailure Analysis (>500mm errors):\n');
    fprintf('  Total failures: %d/%d (%.1f%%)\n', ...
        sum(failures), N, 100*sum(failures)/N);
    fprintf('  First half failures: %d\n', sum(failures(1:floor(N/2))));
    fprintf('  Second half failures: %d\n', sum(failures(floor(N/2)+1:end)));
    
    % Convergence
    if isfield(logC, 'successMask')
        convRate = 100 * sum(logC.successMask) / length(logC.successMask);
        fprintf('\nGIK Convergence Rate: %.1f%%\n', convRate);
    end
    
    % Plot error progression
    figure('Position', [100, 100, 1200, 500]);
    
    subplot(1,2,1);
    plot(posErr, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(5, 'g--', 'Target: 5mm', 'LineWidth', 1.5);
    yline(50, 'y--', '50mm', 'LineWidth', 1);
    yline(500, 'r--', 'Failure: 500mm', 'LineWidth', 1.5);
    xline(floor(N/2), 'k--', 'Half', 'LineWidth', 2);
    xlabel('Waypoint');
    ylabel('EE Error (mm)');
    title('Stage C: EE Tracking Error After Fix');
    grid on;
    ylim([0, min(max(posErr)*1.1, 1000)]);
    
    subplot(1,2,2);
    semilogy(posErr, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(5, 'g--', 'Target: 5mm', 'LineWidth', 1.5);
    yline(500, 'r--', 'Failure', 'LineWidth', 1.5);
    xline(floor(N/2), 'k--', 'Half', 'LineWidth', 2);
    xlabel('Waypoint');
    ylabel('EE Error (mm, log scale)');
    title('Stage C: EE Error (Log Scale)');
    grid on;
    
    saveas(gcf, fullfile(resultsDir, 'stagec_fix_analysis.png'));
    fprintf('\n✓ Plot saved to: %s\n', fullfile(resultsDir, 'stagec_fix_analysis.png'));
    
    % Check if fix worked
    fprintf('\n========================================\n');
    if mean(secondHalf) < 10
        fprintf('✅ FIX SUCCESSFUL! Second half error < 10mm\n');
    else
        fprintf('❌ FIX FAILED! Second half still has high errors (%.1fmm)\n', mean(secondHalf));
        fprintf('\nPossible reasons:\n');
        fprintf('  1. Fix not applied correctly - check runStagedTrajectory.m line 938\n');
        fprintf('  2. Different issue than initialization\n');
        fprintf('  3. Need to investigate actual starting config used\n');
    end
    fprintf('========================================\n');
else
    fprintf('⚠️  Stage C data not found in log file\n');
    fprintf('Available fields: %s\n', strjoin(fieldnames(data), ', '));
end
