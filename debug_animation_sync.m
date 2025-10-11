%% Debug Animation Sync Issue
% This script examines the indexing problem in detail

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Animation Sync Debug Analysis ===\n\n');

% Load a test result
dataFile = 'results/parametric_study_20251011_085252/parametric_study_results.mat';
load(dataFile, 'results');
log = results(1).log;

fprintf('Full trajectory dimensions:\n');
fprintf('  qTraj: %d joints × %d frames\n', size(log.qTraj, 1), size(log.qTraj, 2));

if isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    fprintf('  stageC.qTraj: %d joints × %d frames\n', size(stageC.qTraj, 1), size(stageC.qTraj, 2));
    
    if isfield(stageC.referenceInitialIk, 'eePositions')
        eeRef = stageC.referenceInitialIk.eePositions;
        fprintf('  stageC.referenceInitialIk.eePositions: %d frames × %d dims\n', ...
                size(eeRef, 1), size(eeRef, 2));
    end
end

% Simulate what animateStagedWithHelper does
SampleStep = 2;
fprintf('\nWith SampleStep=%d:\n', SampleStep);

qTraj = log.qTraj;
numFramesFull = size(qTraj, 2);
fprintf('  Full qTraj frames: %d\n', numFramesFull);

% Sample the trajectories
sampledFrames = 1:SampleStep:numFramesFull;
numFramesSampled = length(sampledFrames);
fprintf('  Sampled frames: %d (indices: %d, %d, %d, ..., %d)\n', ...
        numFramesSampled, sampledFrames(1), sampledFrames(2), sampledFrames(3), sampledFrames(end));

% Get Stage C reference
if isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    if isfield(stageC.referenceInitialIk, 'eePositions')
        eePathStageCRef = stageC.referenceInitialIk.eePositions;
        fprintf('\n  eePathStageCRef (from log): %d × %d\n', ...
                size(eePathStageCRef, 1), size(eePathStageCRef, 2));
        
        % What animateStagedWithHelper does (CURRENT CODE):
        eePoses = eePathStageCRef(:, 1:SampleStep:end)';
        fprintf('  eePoses (current code): %d × %d\n', size(eePoses, 1), size(eePoses, 2));
        
        % What's passed to animate_whole_body:
        fprintf('\n  Passed to animate_whole_body:\n');
        fprintf('    armTrajectory: %d frames\n', numFramesSampled);
        fprintf('    basePose: %d frames\n', numFramesSampled);
        fprintf('    eePoses: %d frames\n', size(eePoses, 1));
        fprintf('    StageCReferenceEEPath (in options): %d × %d (FULL!)\n', ...
                size(eePathStageCRef, 1), size(eePathStageCRef, 2));
    end
end

% Simulate what animate_whole_body does
fprintf('\nInside animate_whole_body:\n');
fprintf('  numSteps = %d (from armTrajectory length)\n', numFramesSampled);

% Detect stage boundaries
stageLogs = log.stageLogs;
counts = [];
if isfield(stageLogs, 'stageA') && isfield(stageLogs.stageA, 'qTraj')
    counts(end+1) = size(stageLogs.stageA.qTraj, 2);
end
if isfield(stageLogs, 'stageB') && isfield(stageLogs.stageB, 'qTraj')
    counts(end+1) = size(stageLogs.stageB.qTraj, 2);
end
if isfield(stageLogs, 'stageC') && isfield(stageLogs.stageC, 'qTraj')
    counts(end+1) = size(stageLogs.stageC.qTraj, 2);
end
boundaries = cumsum(counts);
fprintf('  Stage boundaries (FULL): [%s]\n', num2str(boundaries));

% But these boundaries are for FULL trajectory!
% After sampling, they should be adjusted
boundariesSampled = ceil(boundaries / SampleStep);
fprintf('  Stage boundaries (should be SAMPLED): [%s]\n', num2str(boundariesSampled));

% Stage C range
stageStarts = [1, boundaries(1:end-1) + 1];
stageCStart = stageStarts(end);
stageCEnd = boundaries(end);
stageCRange = stageCStart:stageCEnd;
stageCLength = length(stageCRange);

fprintf('\n  Stage C range (FULL trajectory): %d:%d (length=%d)\n', ...
        stageCStart, stageCEnd, stageCLength);

% After sampling
stageCStartSampled = ceil(stageCStart / SampleStep);
stageCEndSampled = ceil(stageCEnd / SampleStep);
stageCRangeSampled = stageCStartSampled:stageCEndSampled;
fprintf('  Stage C range (SAMPLED): %d:%d (length=%d)\n', ...
        stageCStartSampled, stageCEndSampled, length(stageCRangeSampled));

% The problem
fprintf('\n🔴 THE PROBLEM:\n');
fprintf('  animate_whole_body receives stageCReferenceEEPath with %d frames (FULL)\n', ...
        size(eePathStageCRef, 1));
fprintf('  But it tries to map it to stageCIdxRange which is based on SAMPLED frames\n');
fprintf('  stageCIdxRange = %d:%d (length=%d)\n', ...
        stageCRangeSampled(1), stageCRangeSampled(end), length(stageCRangeSampled));
fprintf('  So it puts FULL reference[1:%d] → sampledFrames[%d:%d]\n', ...
        length(stageCRangeSampled), stageCRangeSampled(1), stageCRangeSampled(end));
fprintf('  This means reference frame N goes to animation frame ceil(N/2), causing LAG!\n');

fprintf('\n✅ THE FIX:\n');
fprintf('  animateStagedWithHelper should pass SAMPLED eePathStageCRef, not FULL\n');
fprintf('  helperOptions.StageCReferenceEEPath = eePathStageCRef(:, 1:%d:end).''\n', SampleStep);
fprintf('  This way, reference frame indices match animation frame indices\n');
