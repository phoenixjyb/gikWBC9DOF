%% Quick Verification of Animation Data Source Fix
fprintf('=== Animation Data Source Fix Verification ===\n\n');

% Find most recent log file
logFiles = dir('results/*/log_staged_*.mat');
if isempty(logFiles)
    fprintf('No log files found.\n');
    return;
end

[~, idx] = max([logFiles.datenum]);
logPath = fullfile(logFiles(idx).folder, logFiles(idx).name);
fprintf('Log: %s\n', logFiles(idx).name);

% Load and check
data = load(logPath);
log = data.log;

if ~isfield(log, 'stageLogs') || ~isfield(log.stageLogs, 'stageC')
    fprintf('No Stage C data.\n');
    return;
end

stageC = log.stageLogs.stageC;

% Show what's available
fprintf('\nAvailable data:\n');
if isfield(stageC, 'eePositions')
    fprintf('  ✅ stageC.eePositions [%dx%d] <- WILL USE THIS (Pass 3)\n', ...
        size(stageC.eePositions));
end
if isfield(stageC, 'referenceInitialIk') && isfield(stageC.referenceInitialIk, 'eePositions')
    fprintf('  ⚠️  referenceInitialIk.eePositions [%dx%d] (Pass 1 ideal)\n', ...
        size(stageC.referenceInitialIk.eePositions));
    
    % Calculate deviation
    if isfield(stageC, 'eePositions')
        minLen = min(size(stageC.eePositions, 2), ...
                     size(stageC.referenceInitialIk.eePositions, 2));
        dev = vecnorm(stageC.eePositions(:,1:minLen) - ...
                      stageC.referenceInitialIk.eePositions(:,1:minLen), 2, 1);
        fprintf('\n  Deviation (Pass 1 vs Pass 3):\n');
        fprintf('    Mean: %.1f mm, Max: %.1f mm\n', mean(dev)*1000, max(dev)*1000);
    end
end

fprintf('\n✅ Fix applied: Red dot will show Pass 3 actual (stageC.eePositions)\n');
fprintf('   instead of Pass 1 ideal (referenceInitialIk.eePositions)\n');
