%% Monitor Method 6 Animation Run Progress
% Quick script to check if Method 6 is still running and see latest results
%
% Usage: 
%   run('check_method6_progress.m')

clear; clc;

fprintf('========================================\n');
fprintf('  Method 6 Progress Monitor\n');
fprintf('========================================\n');
fprintf('Time: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Check for latest results folder
resultsRoot = 'results';
if ~exist(resultsRoot, 'dir')
    fprintf('❌ No results folder found!\n');
    return;
end

% Find most recent method 6 folder
folders = dir(fullfile(resultsRoot, '*_fresh_sim_alternating'));
if isempty(folders)
    fprintf('⏳ No Method 6 results yet. Simulation may still be starting...\n\n');
    
    % Check if any fresh sim is running
    allFresh = dir(fullfile(resultsRoot, '*_fresh_sim_*'));
    if ~isempty(allFresh)
        [~, idx] = max([allFresh.datenum]);
        latest = allFresh(idx);
        fprintf('📁 Found recent simulation: %s\n', latest.name);
        fprintf('   Started: %s\n', datestr(latest.datenum, 'yyyy-mm-dd HH:MM:SS'));
    end
    return;
end

% Get most recent
[~, idx] = max([folders.datenum]);
latestFolder = folders(idx);
resultPath = fullfile(resultsRoot, latestFolder.name);

fprintf('📁 Latest Method 6 run: %s\n', latestFolder.name);
fprintf('   Started: %s\n', datestr(latestFolder.datenum, 'yyyy-mm-dd HH:MM:SS'));
fprintf('   Folder: %s\n\n', resultPath);

%% Check for log file
logFile = fullfile(resultPath, 'log_staged_alternating.mat');
if exist(logFile, 'file')
    fprintf('✅ Log file found: %s\n', logFile);
    
    % Load and analyze
    try
        data = load(logFile, 'log');
        log = data.log;
        
        fprintf('\n--- Simulation Progress ---\n');
        
        % Stage frame counts
        if isfield(log, 'stageLogs')
            fprintf('Stage A: %d frames\n', size(log.stageLogs.stageA.qTraj, 2));
            fprintf('Stage B: %d frames\n', size(log.stageLogs.stageB.qTraj, 2));
            fprintf('Stage C: %d frames\n', size(log.stageLogs.stageC.qTraj, 2));
            
            % Method 6 specific
            if isfield(log.stageLogs.stageC, 'solveTime')
                solveTime = log.stageLogs.stageC.solveTime;
                fprintf('\n--- Method 6 Performance ---\n');
                fprintf('Mean solve time: %.1f ms\n', mean(solveTime)*1000);
                fprintf('Max solve time: %.1f ms\n', max(solveTime)*1000);
                fprintf('Control rate: %.1f Hz\n', 1/mean(solveTime));
                
                if isfield(log.stageLogs.stageC, 'mode')
                    modes = log.stageLogs.stageC.mode;
                    baseSteps = sum(strcmp(modes, 'base_pp'));
                    armSteps = sum(strcmp(modes, 'arm_gik'));
                    fprintf('Base steps: %d (%.1f ms avg)\n', baseSteps, ...
                        mean(solveTime(strcmp(modes, 'base_pp')))*1000);
                    fprintf('Arm steps: %d (%.1f ms avg)\n', armSteps, ...
                        mean(solveTime(strcmp(modes, 'arm_gik')))*1000);
                end
            end
            
            % EE error
            if isfield(log.stageLogs.stageC, 'positionError')
                eeError = log.stageLogs.stageC.positionError;
                errorNorms = sqrt(sum(eeError.^2, 1));
                fprintf('\n--- EE Tracking ---\n');
                fprintf('Mean error: %.1f mm\n', mean(errorNorms)*1000);
                fprintf('Max error: %.1f mm\n', max(errorNorms)*1000);
            end
        end
        
        fprintf('\n✅ Simulation COMPLETE!\n');
    catch ME
        fprintf('⚠️  Could not load log: %s\n', ME.message);
    end
else
    fprintf('⏳ Log file not yet created. Simulation in progress...\n');
    fprintf('   Expected: %s\n', logFile);
end

%% Check for video
videoFile = fullfile(resultPath, 'animation_alternating.mp4');
if exist(videoFile, 'file')
    fileInfo = dir(videoFile);
    fprintf('\n✅ Animation COMPLETE!\n');
    fprintf('   File: %s\n', videoFile);
    fprintf('   Size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
    fprintf('   Created: %s\n', datestr(fileInfo.datenum, 'yyyy-mm-dd HH:MM:SS'));
else
    fprintf('\n⏳ Animation not yet generated.\n');
    fprintf('   Expected: %s\n', videoFile);
end

%% Check if MATLAB is still running
fprintf('\n========================================\n');
fprintf('To check if MATLAB is still running:\n');
fprintf('  Get-Process matlab\n\n');
fprintf('To kill if needed:\n');
fprintf('  Stop-Process -Name matlab\n');
fprintf('========================================\n');
