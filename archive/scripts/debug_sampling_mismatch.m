%% Debug Sampling vs Stage Boundary Mismatch
clear; clc;
addpath(genpath('matlab'));

fprintf('=== SAMPLING VS STAGE BOUNDARY MISMATCH ===\n\n');

load('results/parametric_study_20251011_085252/parametric_study_results.mat');
log = results(1).log;

% Simulate what animateStagedWithHelper does
qTraj = log.qTraj;
SampleStep = 2;

fprintf('--- RAW DATA (from logs) ---\n');
stageA_raw = size(log.stageLogs.stageA.qTraj, 2);
stageB_raw = size(log.stageLogs.stageB.qTraj, 2);
stageC_raw = size(log.stageLogs.stageC.qTraj, 2);
total_raw = size(qTraj, 2);

fprintf('Stage A: %d frames\n', stageA_raw);
fprintf('Stage B: %d frames\n', stageB_raw);
fprintf('Stage C: %d frames\n', stageC_raw);
fprintf('Total qTraj: %d frames\n', total_raw);

% detectStageBoundaries returns cumulative counts in RAW frame space
boundaries_raw = cumsum([stageA_raw, stageB_raw, stageC_raw]);
fprintf('\n--- Stage Boundaries (RAW) ---\n');
fprintf('boundaries = [%d, %d, %d]\n', boundaries_raw(1), boundaries_raw(2), boundaries_raw(3));
fprintf('  Stage A ends at raw frame %d\n', boundaries_raw(1));
fprintf('  Stage B ends at raw frame %d\n', boundaries_raw(2));
fprintf('  Stage C ends at raw frame %d\n', boundaries_raw(3));

% But animation samples the trajectory
fprintf('\n--- SAMPLED DATA (SampleStep=%d) ---\n', SampleStep);
sampled_qTraj = qTraj(:, 1:SampleStep:end);
total_sampled = size(sampled_qTraj, 2);
fprintf('Total sampled frames: %d\n', total_sampled);

% Expected sampled counts per stage
sampledA = length(1:SampleStep:stageA_raw);
sampledB = length(1:SampleStep:stageB_raw);
sampledC = length(1:SampleStep:stageC_raw);
fprintf('Stage A: %d → %d sampled frames\n', stageA_raw, sampledA);
fprintf('Stage B: %d → %d sampled frames\n', stageB_raw, sampledB);
fprintf('Stage C: %d → %d sampled frames\n', stageC_raw, sampledC);

% What boundaries SHOULD be (sampled space)
boundaries_sampled = cumsum([sampledA, sampledB, sampledC]);
fprintf('\n--- Stage Boundaries (SHOULD BE SAMPLED) ---\n');
fprintf('boundaries = [%d, %d, %d]\n', boundaries_sampled(1), boundaries_sampled(2), boundaries_sampled(3));
fprintf('  Stage A ends at sampled frame %d\n', boundaries_sampled(1));
fprintf('  Stage B ends at sampled frame %d\n', boundaries_sampled(2));
fprintf('  Stage C ends at sampled frame %d\n', boundaries_sampled(3));

% What animate_whole_body.m receives
fprintf('\n=== THE BUG ===\n');
fprintf('animateStagedWithHelper passes:\n');
fprintf('  StageBoundaries = [%d, %d, %d]  (RAW frame space)\n', boundaries_raw(1), boundaries_raw(2), boundaries_raw(3));
fprintf('  armTrajectory: %d × 6 (SAMPLED)\n', total_sampled);
fprintf('  basePose: %d × 3 (SAMPLED)\n', total_sampled);
fprintf('\n');
fprintf('animate_whole_body.m receives:\n');
fprintf('  numSteps = %d (from sampled trajectory)\n', total_sampled);
fprintf('  stageBoundaries = [%d, %d, %d] (from RAW counts)\n', boundaries_raw(1), boundaries_raw(2), boundaries_raw(3));
fprintf('\n');
fprintf('But stageBoundaries > numSteps!\n');
fprintf('  boundaries(1) = %d > numSteps = %d? %s\n', boundaries_raw(1), total_sampled, mat2str(boundaries_raw(1) > total_sampled));
fprintf('  boundaries(2) = %d > numSteps = %d? %s\n', boundaries_raw(2), total_sampled, mat2str(boundaries_raw(2) > total_sampled));
fprintf('  boundaries(3) = %d > numSteps = %d? %s\n', boundaries_raw(3), total_sampled, mat2str(boundaries_raw(3) > total_sampled));

fprintf('\n=== WHAT HAPPENS IN animate_whole_body.m ===\n');
fprintf('Line 115-116:\n');
fprintf('  stageBoundaries(stageBoundaries > numSteps) = numSteps\n');
fprintf('  Result: [%d, %d, %d] → all clipped to [%d, %d, %d]\n', ...
    boundaries_raw(1), boundaries_raw(2), boundaries_raw(3), ...
    min(boundaries_raw(1), total_sampled), min(boundaries_raw(2), total_sampled), min(boundaries_raw(3), total_sampled));

clipped = boundaries_raw;
clipped(clipped > total_sampled) = total_sampled;
fprintf('\n--- Stage Ranges (INCORRECT, after clipping) ---\n');
if clipped(1) <= total_sampled
    fprintf('  Stage A: frames 1 to %d\n', clipped(1));
end
if length(clipped) > 1 && clipped(2) <= total_sampled
    fprintf('  Stage B: frames %d to %d\n', clipped(1)+1, clipped(2));
end
if length(clipped) > 2
    fprintf('  Stage C: frames %d to %d\n', clipped(2)+1, clipped(3));
end

fprintf('\n--- Stage Ranges (CORRECT, if boundaries were sampled) ---\n');
fprintf('  Stage A: frames 1 to %d\n', boundaries_sampled(1));
fprintf('  Stage B: frames %d to %d\n', boundaries_sampled(1)+1, boundaries_sampled(2));
fprintf('  Stage C: frames %d to %d\n', boundaries_sampled(2)+1, boundaries_sampled(3));

fprintf('\n=== CONCLUSION ===\n');
fprintf('❌ Current: detectStageBoundaries returns RAW frame counts\n');
fprintf('❌ These get clipped to numSteps, destroying stage structure\n');
fprintf('✅ Fix: detectStageBoundaries must account for SampleStep\n');
fprintf('   OR boundaries must be adjusted after sampling\n');
