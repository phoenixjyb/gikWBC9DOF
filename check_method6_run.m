%% Check Method 6 Run Progress
% Quick script to monitor the latest Method 6 run

clear; clc;

fprintf('========================================\n');
fprintf('  Method 6 Run Monitor\n');
fprintf('========================================\n\n');

%% Check for latest results folder
resultsDir = 'results';
if ~exist(resultsDir, 'dir')
    fprintf('❌ No results folder found\n');
    return;
end

% Find latest fresh_sim_alternating folder
d = dir(fullfile(resultsDir, '*fresh_sim_alternating*'));
if isempty(d)
    fprintf('⏳ No Method 6 results yet...\n');
    fprintf('   (Simulation may still be running)\n');
    return;
end

% Sort by date and get most recent
[~, idx] = sort([d.datenum], 'descend');
latestFolder = fullfile(resultsDir, d(idx(1)).name);

fprintf('📁 Latest run: %s\n', d(idx(1)).name);
fprintf('   Modified: %s\n\n', d(idx(1)).date);

%% Check for log file
logFile = fullfile(latestFolder, 'log_staged_alternating.mat');
if ~exist(logFile, 'file')
    fprintf('⏳ No log file yet (still running...)\n');
    return;
end

fprintf('✅ Log file found!\n\n');

%% Load and analyze log
try
    data = load(logFile);
    log = data.log;
    
    % Performance metrics
    N = size(log.qTraj, 2);
    totalTime = log.time(end);
    meanSolve = mean(log.solveTime) * 1000;  % ms
    meanError = mean(log.eeError);  % mm
    maxError = max(log.eeError);  % mm
    
    % Count steps by type
    baseSteps = sum(strcmp(log.mode, 'base_pp'));
    armSteps = sum(strcmp(log.mode, 'arm_gik'));
    
    fprintf('Performance Summary:\n');
    fprintf('  Total steps: %d (%.1f seconds)\n', N, totalTime);
    fprintf('  Mean solve time: %.1f ms (%.1f Hz capable)\n', meanSolve, 1000/meanSolve);
    fprintf('  Mean EE error: %.1f mm\n', meanError);
    fprintf('  Max EE error: %.1f mm\n\n', maxError);
    
    fprintf('Step Breakdown:\n');
    fprintf('  Base steps (PP): %d\n', baseSteps);
    fprintf('  Arm steps (GIK): %d\n\n', armSteps);
    
    % Detailed timing by mode
    if baseSteps > 0
        ppTimes = log.solveTime(strcmp(log.mode, 'base_pp')) * 1000;
        fprintf('  PP timing: %.2f ms avg (%.2f max)\n', mean(ppTimes), max(ppTimes));
    end
    if armSteps > 0
        gikTimes = log.solveTime(strcmp(log.mode, 'arm_gik')) * 1000;
        fprintf('  GIK timing: %.2f ms avg (%.2f max)\n', mean(gikTimes), max(gikTimes));
    end
    
    fprintf('\n');
    
    % Check for animation
    animFile = fullfile(latestFolder, 'animation_alternating.mp4');
    if exist(animFile, 'file')
        fprintf('🎬 Animation generated!\n');
        fprintf('   %s\n', animFile);
    else
        fprintf('⏳ No animation yet...\n');
    end
    
catch ME
    fprintf('❌ Error loading log: %s\n', ME.message);
end
