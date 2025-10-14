% Comprehensive comparison: Isolated vs Staged execution
% We'll instrument both paths to capture function calls and parameters

clearvars; clc;

fprintf('========================================\n');
fprintf('  EXECUTION PATH COMPARISON\n');
fprintf('========================================\n\n');

%% PART 1: Load and compare log structures
fprintf('PART 1: Comparing Log Structures\n');
fprintf('=====================================\n\n');

% Isolated test
isoFile = 'results/stage_c_only_home_start/log_20251014_162314.mat';
if ~isfile(isoFile)
    error('Isolated test file not found: %s', isoFile);
end
isoData = load(isoFile);
% Isolated test stores data in .log field
if isfield(isoData, 'log')
    isoLog = isoData.log;
else
    isoLog = isoData;
end

% Staged run
stgFile = 'results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat';
if ~isfile(stgFile)
    error('Staged test file not found: %s', stgFile);
end
stgData = load(stgFile);
logC_staged = stgData.log.stageLogs.stageC;

fprintf('Isolated test fields:\n');
isoFields = fieldnames(isoLog);
for i = 1:length(isoFields)
    fprintf('  - %s\n', isoFields{i});
end

fprintf('\nStaged Stage C fields:\n');
stgFields = fieldnames(logC_staged);
for i = 1:length(stgFields)
    fprintf('  - %s\n', stgFields{i});
end

fprintf('\nFields in isolated but NOT in staged:\n');
isolatedOnly = setdiff(isoFields, stgFields);
if isempty(isolatedOnly)
    fprintf('  (none)\n');
else
    for i = 1:length(isolatedOnly)
        fprintf('  - %s\n', isolatedOnly{i});
    end
end

fprintf('\nFields in staged but NOT in isolated:\n');
stagedOnly = setdiff(stgFields, isoFields);
if isempty(stagedOnly)
    fprintf('  (none)\n');
else
    for i = 1:length(stagedOnly)
        fprintf('  - %s\n', stagedOnly{i});
    end
end

%% PART 2: Compare dimensions and data shapes
fprintf('\n\nPART 2: Comparing Data Dimensions\n');
fprintf('=====================================\n\n');

fprintf('\nNumber of waypoints:\n');
fprintf('  Isolated: %d\n', size(isoLog.qTraj, 2));
fprintf('  Staged:   %d\n', size(logC_staged.qTraj, 2));

fprintf('\nConfiguration space dimensions:\n');
fprintf('  Isolated: %d DOFs\n', size(isoLog.qTraj, 1));
fprintf('  Staged:   %d DOFs\n', size(logC_staged.qTraj, 1));

%% PART 3: Compare starting configurations
fprintf('\n\nPART 3: Comparing Starting Configurations\n');
fprintf('=====================================\n\n');

q0_iso = isoLog.qTraj(:, 1);
q0_stg = logC_staged.qTraj(:, 1);

fprintf('Starting configuration (first 3 DOFs - base):\n');
fprintf('  Isolated: [%.4f, %.4f, %.4f rad = %.2f deg]\n', ...
    q0_iso(1), q0_iso(2), q0_iso(3), rad2deg(q0_iso(3)));
fprintf('  Staged:   [%.4f, %.4f, %.4f rad = %.2f deg]\n', ...
    q0_stg(1), q0_stg(2), q0_stg(3), rad2deg(q0_stg(3)));

fprintf('\nStarting configuration (arm joints 4-9):\n');
fprintf('  Isolated: [');
for i = 4:9
    fprintf('%.4f', q0_iso(i));
    if i < 9, fprintf(', '); end
end
fprintf(']\n');
fprintf('  Staged:   [');
for i = 4:9
    fprintf('%.4f', q0_stg(i));
    if i < 9, fprintf(', '); end
end
fprintf(']\n');

fprintf('\nConfiguration difference (norm):\n');
fprintf('  Base (DOFs 1-3): %.6f\n', norm(q0_iso(1:3) - q0_stg(1:3)));
fprintf('  Arm (DOFs 4-9):  %.6f\n', norm(q0_iso(4:9) - q0_stg(4:9)));
fprintf('  Total (all DOFs): %.6f\n', norm(q0_iso - q0_stg));

%% PART 4: Compare Phase 1 diagnostics (if available)
fprintf('\n\nPART 4: Comparing Phase 1 Diagnostics\n');
fprintf('=====================================\n\n');

if isfield(isoLog, 'lateralVelocity') && isfield(logC_staged, 'lateralVelocity')
    fprintf('Lateral velocity statistics:\n');
    fprintf('  Isolated: mean=%.4f m/s, max=%.4f m/s\n', ...
        mean(abs(isoLog.lateralVelocity)), max(abs(isoLog.lateralVelocity)));
    fprintf('  Staged:   mean=%.4f m/s, max=%.4f m/s\n', ...
        mean(abs(logC_staged.lateralVelocity)), max(abs(logC_staged.lateralVelocity)));
else
    fprintf('  Lateral velocity data not available\n');
end

if isfield(isoLog, 'lookaheadEffective') && isfield(logC_staged, 'lookaheadEffective')
    fprintf('\nAdaptive lookahead statistics:\n');
    fprintf('  Isolated: mean=%.4f m, min=%.4f m, max=%.4f m\n', ...
        mean(isoLog.lookaheadEffective), min(isoLog.lookaheadEffective), max(isoLog.lookaheadEffective));
    fprintf('  Staged:   mean=%.4f m, min=%.4f m, max=%.4f m\n', ...
        mean(logC_staged.lookaheadEffective), min(logC_staged.lookaheadEffective), max(logC_staged.lookaheadEffective));
else
    fprintf('  Lookahead data not available\n');
end

%% PART 5: Compare GIK convergence patterns
fprintf('\n\nPART 5: Comparing GIK Convergence Patterns\n');
fprintf('=====================================\n\n');

if isfield(isoLog, 'successMask') && isfield(logC_staged, 'successMask')
    iso_conv = isoLog.successMask;
    stg_conv = logC_staged.successMask;
    
    fprintf('Overall convergence rates:\n');
    fprintf('  Isolated: %.1f%% (%d/%d)\n', ...
        100*sum(iso_conv)/length(iso_conv), sum(iso_conv), length(iso_conv));
    fprintf('  Staged:   %.1f%% (%d/%d)\n', ...
        100*sum(stg_conv)/length(stg_conv), sum(stg_conv), length(stg_conv));
    
    fprintf('\nFirst half convergence (waypoints 1-105):\n');
    fprintf('  Isolated: %.1f%% (%d/105)\n', ...
        100*sum(iso_conv(1:105))/105, sum(iso_conv(1:105)));
    fprintf('  Staged:   %.1f%% (%d/105)\n', ...
        100*sum(stg_conv(1:105))/105, sum(stg_conv(1:105)));
    
    fprintf('\nSecond half convergence (waypoints 106-210):\n');
    fprintf('  Isolated: %.1f%% (%d/105)\n', ...
        100*sum(iso_conv(106:end))/105, sum(iso_conv(106:end)));
    fprintf('  Staged:   %.1f%% (%d/105)\n', ...
        100*sum(stg_conv(106:end))/105, sum(stg_conv(106:end)));
end

%% PART 6: Compare solve times
fprintf('\n\nPART 6: Comparing Solve Times\n');
fprintf('=====================================\n\n');

if isfield(isoLog, 'solveTime') && isfield(logC_staged, 'solveTime')
    fprintf('Solve time statistics:\n');
    fprintf('  Isolated: mean=%.3f s, max=%.3f s\n', ...
        mean(isoLog.solveTime), max(isoLog.solveTime));
    fprintf('  Staged:   mean=%.3f s, max=%.3f s\n', ...
        mean(logC_staged.solveTime), max(logC_staged.solveTime));
    
    fprintf('\nFirst half solve times:\n');
    fprintf('  Isolated: mean=%.3f s\n', mean(isoLog.solveTime(1:105)));
    fprintf('  Staged:   mean=%.3f s\n', mean(logC_staged.solveTime(1:105)));
    
    fprintf('\nSecond half solve times:\n');
    fprintf('  Isolated: mean=%.3f s\n', mean(isoLog.solveTime(106:end)));
    fprintf('  Staged:   mean=%.3f s\n', mean(logC_staged.solveTime(106:end)));
end

%% PART 7: Compare errors progression
fprintf('\n\nPART 7: Comparing Error Progression\n');
fprintf('=====================================\n\n');

err_iso = isoLog.positionErrorNorm * 1000;  % mm
err_stg = logC_staged.positionErrorNorm * 1000;  % mm

fprintf('Error statistics (mm):\n');
fprintf('  Isolated: mean=%.2f, median=%.2f, max=%.2f\n', ...
    mean(err_iso), median(err_iso), max(err_iso));
fprintf('  Staged:   mean=%.2f, median=%.2f, max=%.2f\n', ...
    mean(err_stg), median(err_stg), max(err_stg));

% Find first waypoint where staged diverges significantly
threshold = 50;  % mm
diverge_wp = find(err_stg > threshold, 1);
if ~isempty(diverge_wp)
    fprintf('\nFirst significant divergence (>%dmm) at waypoint: %d\n', threshold, diverge_wp);
    fprintf('  Isolated error: %.2f mm\n', err_iso(diverge_wp));
    fprintf('  Staged error:   %.2f mm\n', err_stg(diverge_wp));
end

% Show errors around waypoint 105-106 transition
fprintf('\nErrors around waypoint 105-106 transition:\n');
fprintf('  WP#   Isolated(mm)  Staged(mm)  Diff(mm)\n');
fprintf('  ---   ------------  ----------  --------\n');
for k = 103:108
    if k <= length(err_iso) && k <= length(err_stg)
        fprintf('  %3d   %12.2f  %10.2f  %8.2f\n', ...
            k, err_iso(k), err_stg(k), err_stg(k) - err_iso(k));
    end
end

%% PART 8: Save comparison results
fprintf('\n\nSaving detailed comparison...\n');
comparison = struct();
comparison.isolated = isoLog;
comparison.staged = logC_staged;
comparison.analysis = struct();
comparison.analysis.start_config_diff = norm(q0_iso - q0_stg);
comparison.analysis.first_divergence_wp = diverge_wp;

save('results/execution_path_comparison.mat', 'comparison');
fprintf('✓ Saved to: results/execution_path_comparison.mat\n');

fprintf('\n========================================\n');
fprintf('  COMPARISON COMPLETE\n');
fprintf('========================================\n');
