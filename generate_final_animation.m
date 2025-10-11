%% Generate Single Animation with All Fixes
% This script generates one animation with all three fixes applied:
% Fix 1: Stage boundary sampling correction
% Fix 2: Red dot stage-relative indexing
% Fix 3: Stage C EE path from targetPositions (not full trajectory)

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('=== Generating Animation with All Fixes ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% Load baseline configuration
load('results/parametric_study_20251011_085252/parametric_study_results.mat');
config = results(1);
log = config.log;

fprintf('Configuration: %s\n', config.config.name);
fprintf('Parameters:\n');
fprintf('  Safety Margin: %.2f\n', config.config.SafetyMargin);
fprintf('  Lambda Cusp: %.1f\n', config.config.LambdaCusp);
fprintf('  Max Iterations: %d\n', config.config.MaxRSIterations);
fprintf('  Allow Reverse: %d\n', config.config.AllowReverse);
fprintf('  Clothoid Discretization: %.2f\n', config.config.ClothoidDiscretization);

fprintf('\nMetrics:\n');
fprintf('  Mean EE Error: %.4f m\n', config.metrics.meanError);
fprintf('  Max EE Error: %.4f m\n', config.metrics.maxError);
fprintf('  Cusps: %d\n', config.metrics.numCusps);
fprintf('  RMS Jerk: %.1f m/s³\n', config.metrics.rmsJerk);

% Output filename with timestamp
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
videoFile = sprintf('FINAL_BASELINE_%s.mp4', timestamp);

fprintf('\n--- Animation Settings ---\n');
fprintf('  SampleStep: 2 (50%% frame reduction)\n');
fprintf('  FrameRate: 20 fps\n');
fprintf('  Output: %s\n', videoFile);

fprintf('\n--- Expected Stage Structure ---\n');
stageA_frames = ceil(size(log.stageLogs.stageA.qTraj, 2) / 2);
stageB_frames = ceil(size(log.stageLogs.stageB.qTraj, 2) / 2);
stageC_frames = ceil(size(log.stageLogs.stageC.qTraj, 2) / 2);
fprintf('  Stage A: frames 1-%d (%.1f sec @ 20fps)\n', stageA_frames, stageA_frames/20);
fprintf('  Stage B: frames %d-%d (%.1f sec @ 20fps)\n', stageA_frames+1, stageA_frames+stageB_frames, stageB_frames/20);
fprintf('  Stage C: frames %d-%d (%.1f sec @ 20fps)\n', stageA_frames+stageB_frames+1, stageA_frames+stageB_frames+stageC_frames, stageC_frames/20);
fprintf('  Total: %d frames (%.1f sec)\n', stageA_frames+stageB_frames+stageC_frames, (stageA_frames+stageB_frames+stageC_frames)/20);

fprintf('\n--- Stage C Reference EE Path ---\n');
if isfield(log.stageLogs, 'stageC') && isfield(log.stageLogs.stageC, 'targetPositions')
    targetPos = log.stageLogs.stageC.targetPositions;
    fprintf('  Source: stageC.targetPositions (JSON desired trajectory)\n');
    fprintf('  Size: %d × %d\n', size(targetPos, 1), size(targetPos, 2));
    fprintf('  First waypoint: [%.4f, %.4f, %.4f]\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
    fprintf('  Last waypoint:  [%.4f, %.4f, %.4f]\n', targetPos(1,end), targetPos(2,end), targetPos(3,end));
    fprintf('  ✓ Red dot will track this trajectory (NOT from homebase)\n');
end

fprintf('\n--- Generating Animation ---\n');
fprintf('Please wait... (~2 minutes)\n\n');

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    elapsed = toc;
    fprintf('\n✅ Animation Generated Successfully!\n');
    fprintf('   Time elapsed: %.1f seconds (%.1f minutes)\n', elapsed, elapsed/60);
    
    % Check file
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('   File: %s\n', videoFile);
        fprintf('   Size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
        
        fprintf('\n=== VERIFICATION CHECKLIST ===\n');
        fprintf('Please review the animation and verify:\n\n');
        fprintf('Stage Labels:\n');
        fprintf('  [ ] Stage A label appears for frames 1-%d (%.1f sec)\n', stageA_frames, stageA_frames/20);
        fprintf('  [ ] Stage B label appears for frames %d-%d (%.1f sec)\n', stageA_frames+1, stageA_frames+stageB_frames, stageB_frames/20);
        fprintf('  [ ] Stage C label appears for frames %d-%d (%.1f sec)\n', stageA_frames+stageB_frames+1, stageA_frames+stageB_frames+stageC_frames, stageC_frames/20);
        
        fprintf('\nRed Dot (Stage C Reference EE):\n');
        fprintf('  [ ] Red dot HIDDEN during Stages A & B\n');
        fprintf('  [ ] Red dot APPEARS at frame %d (when Stage C starts)\n', stageA_frames+stageB_frames+1);
        fprintf('  [ ] Red dot starts at [%.2f, %.2f, %.2f] (NOT homebase)\n', targetPos(1,1), targetPos(2,1), targetPos(3,1));
        fprintf('  [ ] Red dot moves synchronously with robot (no lag)\n');
        fprintf('  [ ] Red dot is ABOVE chassis (Z ≈ %.2f m, not on ground)\n', targetPos(3,1));
        fprintf('  [ ] Red dot is DIFFERENT from Stage B chassis path\n');
        
        fprintf('\nOverall:\n');
        fprintf('  [ ] Stage transitions look natural (no jumps)\n');
        fprintf('  [ ] Total duration matches: %.1f seconds\n', (stageA_frames+stageB_frames+stageC_frames)/20);
        fprintf('  [ ] Green square (actual EE) tracks red dot closely in Stage C\n');
        
        fprintf('\n=== All Three Fixes Applied ===\n');
        fprintf('✓ Fix 1: Stage boundaries correctly adjusted for sampling\n');
        fprintf('✓ Fix 2: Red dot uses stage-relative indexing\n');
        fprintf('✓ Fix 3: Red dot shows Stage C desired trajectory (not full trajectory)\n');
        
    else
        fprintf('\n⚠ WARNING: Video file is empty or not found!\n');
    end
    
catch ME
    elapsed = toc;
    fprintf('\n❌ ERROR during animation generation:\n');
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    fprintf('   Time before error: %.1f seconds\n', elapsed);
end

fprintf('\n=== Complete ===\n');
