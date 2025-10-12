%% Generate Animation from Saved Log
% The simulation has already completed and saved the log.
% This script loads it and generates the animation.

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('========================================\n');
fprintf('  Generate Animation from Saved Log\n');
fprintf('========================================\n');

% Load the saved log
logPath = 'results/20251011_134613_fresh_baseline/log_staged_ppForIk.mat';
fprintf('Loading: %s\n', logPath);

if ~exist(logPath, 'file')
    error('Log file not found: %s', logPath);
end

data = load(logPath);
log = data.log;

fprintf('✅ Log loaded successfully\n\n');

%% Analyze Results
fprintf('========================================\n');
fprintf('  Analyzing Results\n');
fprintf('========================================\n');

% Stage information
if isfield(log, 'stageLogs')
    fprintf('--- Stage Frame Counts ---\n');
    stageA_raw = size(log.stageLogs.stageA.qTraj, 2);
    stageB_raw = size(log.stageLogs.stageB.qTraj, 2);
    stageC_raw = size(log.stageLogs.stageC.qTraj, 2);
    fprintf('  Stage A: %d frames (%.1f sec @ 10Hz)\n', stageA_raw, stageA_raw/10);
    fprintf('  Stage B: %d frames (%.1f sec @ 10Hz)\n', stageB_raw, stageB_raw/10);
    fprintf('  Stage C: %d frames (%.1f sec @ 10Hz)\n', stageC_raw, stageC_raw/10);
    fprintf('  Total: %d frames (%.1f sec @ 10Hz)\n\n', stageA_raw+stageB_raw+stageC_raw, (stageA_raw+stageB_raw+stageC_raw)/10);
    
    % Stage C EE tracking
    if isfield(log.stageLogs.stageC, 'positionError')
        eeError = log.stageLogs.stageC.positionError;
        errorNorms = sqrt(sum(eeError.^2, 1));
        fprintf('--- Stage C EE Tracking ---\n');
        fprintf('  Mean error: %.4f m\n', mean(errorNorms));
        fprintf('  Max error: %.4f m\n', max(errorNorms));
        fprintf('  RMS error: %.4f m\n\n', rms(errorNorms));
    end
    
    % Stage C reference path
    if isfield(log.stageLogs.stageC, 'targetPositions')
        targetPos = log.stageLogs.stageC.targetPositions;
        fprintf('--- Stage C Reference (for Red Dot) ---\n');
        fprintf('  Source: stageC.targetPositions (JSON desired trajectory)\n');
        fprintf('  Waypoints: %d\n', size(targetPos, 2));
        fprintf('  First: [%.4f, %.4f, %.4f]\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
        fprintf('  Last:  [%.4f, %.4f, %.4f]\n\n', targetPos(1,end), targetPos(2,end), targetPos(3,end));
    end
end

%% Generate Animation
fprintf('========================================\n');
fprintf('  Generating Animation\n');
fprintf('========================================\n');

videoFile = 'results/20251011_134613_fresh_baseline/animation_baseline.mp4';

fprintf('Settings:\n');
fprintf('  SampleStep: 2 (50%% frame reduction)\n');
fprintf('  FrameRate: 20 fps\n');
fprintf('  Output: %s\n\n', videoFile);

% Expected sampled frame structure
stageA_sampled = ceil(stageA_raw / 2);
stageB_sampled = ceil(stageB_raw / 2);
stageC_sampled = ceil(stageC_raw / 2);
fprintf('Expected stage structure (sampled):\n');
fprintf('  Stage A: frames 1-%d (%.1f sec @ 20fps)\n', stageA_sampled, stageA_sampled/20);
fprintf('  Stage B: frames %d-%d (%.1f sec @ 20fps)\n', stageA_sampled+1, stageA_sampled+stageB_sampled, stageB_sampled/20);
fprintf('  Stage C: frames %d-%d (%.1f sec @ 20fps)\n', stageA_sampled+stageB_sampled+1, stageA_sampled+stageB_sampled+stageC_sampled, stageC_sampled/20);
fprintf('  Total video: %d frames (%.1f sec @ 20fps)\n\n', stageA_sampled+stageB_sampled+stageC_sampled, (stageA_sampled+stageB_sampled+stageC_sampled)/20);

fprintf('Generating animation (please wait ~2 minutes)...\n');

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    animTime = toc;
    fprintf('\n✅ Animation generated in %.1f seconds (%.1f minutes)\n', animTime, animTime/60);
    
    % Check file
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('   File: %s\n', videoFile);
        fprintf('   Size: %.2f MB\n\n', fileInfo.bytes / 1024 / 1024);
    else
        fprintf('⚠ WARNING: Video file is empty or not found!\n\n');
    end
    
catch ME
    animTime = toc;
    fprintf('\n❌ Animation FAILED after %.1f seconds\n', animTime);
    fprintf('   Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    return;
end

%% Summary
fprintf('========================================\n');
fprintf('  COMPLETE!\n');
fprintf('========================================\n');

fprintf('Output files:\n');
fprintf('  Log: %s\n', logPath);
fprintf('  Video: %s\n\n', videoFile);

fprintf('=== VERIFICATION CHECKLIST ===\n');
fprintf('Please review the animation:\n\n');
fprintf('Stage Labels:\n');
fprintf('  [ ] Stage A label for first %.1f seconds\n', stageA_sampled/20);
fprintf('  [ ] Stage B label transitions at %.1f seconds\n', stageA_sampled/20);
fprintf('  [ ] Stage C label transitions at %.1f seconds\n', (stageA_sampled+stageB_sampled)/20);
fprintf('\nRed Dot (Stage C Reference):\n');
fprintf('  [ ] Hidden during Stages A & B (first %.1f sec)\n', (stageA_sampled+stageB_sampled)/20);
fprintf('  [ ] Appears when Stage C starts (at %.1f sec)\n', (stageA_sampled+stageB_sampled)/20);
fprintf('  [ ] Starts at [%.2f, %.2f, %.2f] (NOT homebase)\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
fprintf('  [ ] Moves synchronously with robot\n');
fprintf('  [ ] Is ABOVE chassis (Z ≈ %.2f m)\n', targetPos(3,1));
fprintf('\nAll Three Fixes Applied:\n');
fprintf('  ✓ Stage boundaries adjusted for sampling\n');
fprintf('  ✓ Red dot stage-relative indexing\n');
fprintf('  ✓ Red dot from Stage C desired trajectory\n');

fprintf('\n========================================\n');
