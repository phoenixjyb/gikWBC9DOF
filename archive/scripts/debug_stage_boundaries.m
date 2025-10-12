%% Debug Stage Boundaries and Red Dot Indexing Issue
clear; clc;

% Load parametric study baseline config
load('results/parametric_study_20251011_085252/parametric_study_results.mat');
log = results(1).log;

fprintf('=== STAGE BOUNDARY ANALYSIS ===\n\n');

% Check stage logs structure
if isfield(log, 'stageLogs')
    fprintf('Stage Logs found:\n');
    
    % Stage A
    if isfield(log.stageLogs, 'stageA')
        stageA_frames = size(log.stageLogs.stageA.qTraj, 2);
        fprintf('  Stage A: %d frames (raw)\n', stageA_frames);
    else
        stageA_frames = 0;
        fprintf('  Stage A: NOT FOUND\n');
    end
    
    % Stage B
    if isfield(log.stageLogs, 'stageB')
        stageB_frames = size(log.stageLogs.stageB.qTraj, 2);
        fprintf('  Stage B: %d frames (raw)\n', stageB_frames);
    else
        stageB_frames = 0;
        fprintf('  Stage B: NOT FOUND\n');
    end
    
    % Stage C
    if isfield(log.stageLogs, 'stageC')
        stageC_frames = size(log.stageLogs.stageC.qTraj, 2);
        fprintf('  Stage C: %d frames (raw)\n', stageC_frames);
        
        % Check EE reference data
        if isfield(log.stageLogs.stageC, 'eePositions')
            eeRef_frames = size(log.stageLogs.stageC.eePositions, 2);
            fprintf('  Stage C EE reference: %d frames\n', eeRef_frames);
        end
    else
        stageC_frames = 0;
        fprintf('  Stage C: NOT FOUND\n');
    end
    
    % Calculate cumulative boundaries
    fprintf('\n--- Cumulative Boundaries (raw) ---\n');
    cumA = stageA_frames;
    cumB = cumA + stageB_frames;
    cumC = cumB + stageC_frames;
    fprintf('  After Stage A: frame %d\n', cumA);
    fprintf('  After Stage B: frame %d\n', cumB);
    fprintf('  After Stage C: frame %d\n', cumC);
    
    % With SampleStep=2
    fprintf('\n--- After SampleStep=2 ---\n');
    sampledA = ceil(stageA_frames / 2);
    sampledB = ceil(stageB_frames / 2);
    sampledC = ceil(stageC_frames / 2);
    fprintf('  Stage A: %d → %d frames\n', stageA_frames, sampledA);
    fprintf('  Stage B: %d → %d frames\n', stageB_frames, sampledB);
    fprintf('  Stage C: %d → %d frames\n', stageC_frames, sampledC);
    
    cumSampledA = sampledA;
    cumSampledB = cumSampledA + sampledB;
    cumSampledC = cumSampledB + sampledC;
    fprintf('\n  Stage ranges (sampled):\n');
    fprintf('    Stage A: frames 1 to %d\n', cumSampledA);
    fprintf('    Stage B: frames %d to %d\n', cumSampledA+1, cumSampledB);
    fprintf('    Stage C: frames %d to %d\n', cumSampledB+1, cumSampledC);
    
else
    fprintf('ERROR: No stageLogs field found!\n');
end

% Check total qTraj
total_frames = size(log.qTraj, 2);
fprintf('\n--- Total Combined Trajectory ---\n');
fprintf('  log.qTraj: %d frames (raw)\n', total_frames);
fprintf('  After SampleStep=2: %d frames\n', ceil(total_frames / 2));

fprintf('\n=== THE PROBLEM ===\n');
fprintf('In animate_whole_body.m line 191:\n');
fprintf('  stageCFull(stageCIdxRange(1:L), :) = pathC(1:L, 1:3)\n\n');
fprintf('This assigns:\n');
fprintf('  pathC(1) → stageCFull(stageCIdxRange(1))  [e.g., frame %d]\n', cumSampledB+1);
fprintf('  pathC(2) → stageCFull(stageCIdxRange(2))  [e.g., frame %d]\n', cumSampledB+2);
fprintf('  ...\n\n');
fprintf('But red dot reads stageCFull(k) where k starts from 1!\n');
fprintf('So red dot is NaN for frames 1 to %d (Stages A & B)\n', cumSampledB);
fprintf('Red dot only appears at frame %d (when Stage C starts)\n\n', cumSampledB+1);

fprintf('=== THE FIX ===\n');
fprintf('Red dot should track Stage C reference ONLY during Stage C.\n');
fprintf('Two options:\n');
fprintf('1. Only show red dot when k is in stageCIdxRange\n');
fprintf('2. Map pathC linearly: pathC(1) → stageCFull(1), pathC(2) → stageCFull(2)\n');
fprintf('   and only show during Stage C range\n\n');
