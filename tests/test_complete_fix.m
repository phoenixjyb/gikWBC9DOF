%% Test Complete Stage Boundary Sampling Fix
clear; clc; close all;
addpath(genpath('matlab'));

fprintf('=== Testing Complete Stage Boundary + Sampling Fix ===\n\n');

load('results/parametric_study_20251011_085252/parametric_study_results.mat');
log = results(1).log;

fprintf('Configuration: %s\n', results(1).config.name);
fprintf('Parameters: SampleStep=2, FrameRate=20\n\n');

% Show expected stage structure
stageA_raw = size(log.stageLogs.stageA.qTraj, 2);
stageB_raw = size(log.stageLogs.stageB.qTraj, 2);
stageC_raw = size(log.stageLogs.stageC.qTraj, 2);

sampledA = length(1:2:stageA_raw);
sampledB = length(1:2:stageB_raw);
sampledC = length(1:2:stageC_raw);

fprintf('--- Expected Stage Structure (Sampled) ---\n');
fprintf('Stage A: frames 1 to %d (%d frames)\n', sampledA, sampledA);
fprintf('Stage B: frames %d to %d (%d frames)\n', sampledA+1, sampledA+sampledB, sampledB);
fprintf('Stage C: frames %d to %d (%d frames)\n', sampledA+sampledB+1, sampledA+sampledB+sampledC, sampledC);

fprintf('\n--- Generating Test Animation ---\n');
videoFile = 'TEST_COMPLETE_FIX.mp4';

try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    fprintf('\n✅ Animation generated: %s\n', videoFile);
    
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('File size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
        
        fprintf('\n=== VERIFICATION CHECKLIST ===\n');
        fprintf('Please verify in the animation:\n\n');
        fprintf('[ ] 1. Stage A label shows for frames 1-%d (%.1f seconds)\n', sampledA, sampledA/20);
        fprintf('[ ] 2. Stage B label shows for frames %d-%d (%.1f seconds)\n', ...
            sampledA+1, sampledA+sampledB, sampledB/20);
        fprintf('[ ] 3. Stage C label shows for frames %d-%d (%.1f seconds)\n', ...
            sampledA+sampledB+1, sampledA+sampledB+sampledC, sampledC/20);
        fprintf('[ ] 4. Red dot appears EXACTLY when "Stage C" label appears\n');
        fprintf('[ ] 5. Red dot moves synchronously with robot (no lag)\n');
        fprintf('[ ] 6. Stage transitions look natural (no jumps)\n');
        fprintf('[ ] 7. Total video duration: %.1f seconds (%d frames @ 20fps)\n', ...
            (sampledA+sampledB+sampledC)/20, sampledA+sampledB+sampledC);
        
        fprintf('\n--- Timing Analysis ---\n');
        fprintf('At 20 fps video:\n');
        fprintf('  Stage A: 0.0 - %.1f sec (visual time in video)\n', sampledA/20);
        fprintf('  Stage B: %.1f - %.1f sec\n', sampledA/20, (sampledA+sampledB)/20);
        fprintf('  Stage C: %.1f - %.1f sec\n', (sampledA+sampledB)/20, (sampledA+sampledB+sampledC)/20);
        
        fprintf('\nOriginal simulation timing (10 Hz):\n');
        fprintf('  Stage A: 0.0 - %.1f sec (real time)\n', stageA_raw/10);
        fprintf('  Stage B: %.1f - %.1f sec\n', stageA_raw/10, (stageA_raw+stageB_raw)/10);
        fprintf('  Stage C: %.1f - %.1f sec\n', (stageA_raw+stageB_raw)/10, (stageA_raw+stageB_raw+stageC_raw)/10);
        
        fprintf('\n✓ Video is 2x real-time (20fps video, 10Hz sim, SampleStep=2)\n');
    else
        fprintf('\n⚠ WARNING: Video file is empty or not found!\n');
    end
    
catch ME
    fprintf('\n❌ ERROR during animation:\n');
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end

fprintf('\n=== Test Complete ===\n');
