%% Test Stage C Red Dot Synchronization Fix
% This script tests that the red dot (Stage C reference EE):
% 1. Only appears during Stage C (not during Stages A & B)
% 2. Moves synchronously with the robot during Stage C
% 3. Tracks the Stage C reference trajectory correctly

clear; clc; close all;

% Add path
addpath(genpath('matlab'));

fprintf('=== Testing Stage C Red Dot Synchronization Fix ===\n\n');

% Load baseline config
load('results/parametric_study_20251011_085252/parametric_study_results.mat');
config = results(1);
log = config.log;

fprintf('Test Configuration: %s\n', config.config.name);
fprintf('Parameters: SM=%.2f LC=%.1f Iters=%d\n', ...
    config.config.SafetyMargin, config.config.LambdaCusp, config.config.MaxRSIterations);

% Extract stage information
if isfield(log, 'stageLogs')
    stageA_frames = size(log.stageLogs.stageA.qTraj, 2);
    stageB_frames = size(log.stageLogs.stageB.qTraj, 2);
    stageC_frames = size(log.stageLogs.stageC.qTraj, 2);
    
    fprintf('\n--- Stage Frame Counts (raw) ---\n');
    fprintf('  Stage A: %d frames\n', stageA_frames);
    fprintf('  Stage B: %d frames\n', stageB_frames);
    fprintf('  Stage C: %d frames\n', stageC_frames);
    
    % With SampleStep=2
    sampledA = ceil(stageA_frames / 2);
    sampledB = ceil(stageB_frames / 2);
    sampledC = ceil(stageC_frames / 2);
    
    fprintf('\n--- After SampleStep=2 ---\n');
    fprintf('  Stage A: %d frames (1 to %d)\n', sampledA, sampledA);
    fprintf('  Stage B: %d frames (%d to %d)\n', sampledB, sampledA+1, sampledA+sampledB);
    fprintf('  Stage C: %d frames (%d to %d)\n', sampledC, sampledA+sampledB+1, sampledA+sampledB+sampledC);
    
    stageCStartFrame = sampledA + sampledB + 1;
    fprintf('\n✓ Stage C starts at frame %d\n', stageCStartFrame);
    fprintf('✓ Red dot should appear at frame %d (not before)\n', stageCStartFrame);
    fprintf('✓ Red dot should track Stage C reference EE trajectory\n\n');
end

% Generate test animation
fprintf('Generating test animation...\n');
videoFile = 'TEST_STAGE_SYNC_FIXED.mp4';

try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    fprintf('\n✓ Animation generated: %s\n', videoFile);
    fprintf('\n=== VERIFICATION CHECKLIST ===\n');
    fprintf('Please review the animation and verify:\n');
    fprintf('[ ] 1. Red dot DOES NOT appear during Stage A (frames 1-%d)\n', sampledA);
    fprintf('[ ] 2. Red dot DOES NOT appear during Stage B (frames %d-%d)\n', sampledA+1, sampledA+sampledB);
    fprintf('[ ] 3. Red dot APPEARS at frame %d (when Stage C starts)\n', stageCStartFrame);
    fprintf('[ ] 4. Red dot moves WITH the robot (no lag)\n');
    fprintf('[ ] 5. Red dot tracks Stage C reference trajectory\n');
    fprintf('[ ] 6. Green square (actual EE) follows red dot closely\n');
    fprintf('\nIf all checks pass, the fix is successful!\n');
    
    % Check file size
    fileInfo = dir(videoFile);
    if isempty(fileInfo) || fileInfo.bytes == 0
        fprintf('\n⚠ WARNING: Video file is empty or not found!\n');
    else
        fprintf('\nVideo file size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
    end
    
catch ME
    fprintf('\n❌ ERROR during animation generation:\n');
    fprintf('   %s\n', ME.message);
    fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
end

fprintf('\n=== Test Complete ===\n');
