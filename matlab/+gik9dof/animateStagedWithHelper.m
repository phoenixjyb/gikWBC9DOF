function animateStagedWithHelper(logStaged, varargin)
%ANIMATESTAGEDWITHHELPER Visualise staged runs using internal helper.
%
parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'Robot', gik9dof.createRobotModel(), @(x) isa(x,'rigidBodyTree'));
addParameter(parser, 'SampleStep', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, 'FrameRate', 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'ExportVideo', "", @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'HelperOptions', struct(), @(x) isstruct(x));
parse(parser, varargin{:});
opts = parser.Results;

robot = opts.Robot;
tools = gik9dof.configurationTools(robot);
jointNames = string(tools.templateJointNames());
baseNames = string({'joint_x','joint_y','joint_theta'});
[~, baseIdx] = ismember(baseNames, jointNames);
baseIdx = baseIdx(baseIdx > 0);
armIdx = setdiff(1:numel(jointNames), baseIdx);
armJointNames = cellstr(jointNames(armIdx));

qTraj = logStaged.qTraj;
armTrajectory = qTraj(armIdx, :)';
basePose = qTraj(baseIdx, :)';

t = resolveTimes(logStaged, size(qTraj, 2));
t = t(:)';
t = t - t(1);
armTrajectory = armTrajectory(1:opts.SampleStep:end, :);
basePose = basePose(1:opts.SampleStep:end, :);
t = t(1:opts.SampleStep:end);
armTimes = t;
baseTimes = t;

eePathDesired = [];
if isfield(logStaged, 'referenceTrajectory') && isfield(logStaged.referenceTrajectory, 'EndEffectorPositions') && ...
        ~isempty(logStaged.referenceTrajectory.EndEffectorPositions)
    eePathDesired = logStaged.referenceTrajectory.EndEffectorPositions;
end

eePathStageCRef = [];
% Try to get Stage C reference EE path (desired trajectory for tracking)
if isfield(logStaged, 'stageLogs') && isstruct(logStaged.stageLogs) && ...
        isfield(logStaged.stageLogs, 'stageC')
    stageC = logStaged.stageLogs.stageC;
    
    % Priority 1: referenceInitialIk.eePositions (if available)
    if isfield(stageC, 'referenceInitialIk') && ...
            isfield(stageC.referenceInitialIk, 'eePositions') && ...
            ~isempty(stageC.referenceInitialIk.eePositions)
        eePathStageCRef = stageC.referenceInitialIk.eePositions;
    % Priority 2: targetPositions (desired EE trajectory from JSON)
    elseif isfield(stageC, 'targetPositions') && ~isempty(stageC.targetPositions)
        eePathStageCRef = stageC.targetPositions;
    % Priority 3: eePositions (actual Stage C EE, not reference but better than nothing)
    elseif isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
        eePathStageCRef = stageC.eePositions;
    end
end

if isempty(eePathStageCRef)
    % Last resort: Use full trajectory reference if available
    if isfield(logStaged, 'referenceTrajectory') && ...
            isfield(logStaged.referenceTrajectory, 'EndEffectorPositions') && ...
            ~isempty(logStaged.referenceTrajectory.EndEffectorPositions)
        eePathStageCRef = logStaged.referenceTrajectory.EndEffectorPositions;
    end
end

% Apply sampling
if ~isempty(eePathStageCRef)
    eePoses = eePathStageCRef(:, 1:opts.SampleStep:end)';
else
    eePoses = [];
end

helperArgs = {};
if ~isempty(opts.ExportVideo)
    helperArgs = [helperArgs, {'VideoFile', opts.ExportVideo}, {'VideoFrameRate', opts.FrameRate}]; %#ok<AGROW>
end

helperOptions = opts.HelperOptions;
if ~isempty(eePathDesired)
    helperOptions.DesiredEEPath = eePathDesired.';
else
    helperOptions.DesiredEEPath = [];
end
% CRITICAL: Pass the SAMPLED eePoses, not the full eePathStageCRef
% The red dot position must match the animation frame indices
helperOptions.StageCReferenceEEPath = eePoses;
if isfield(logStaged, 'purePursuit') && isfield(logStaged.purePursuit, 'referencePath') && ...
        ~isempty(logStaged.purePursuit.referencePath)
    refPath = logStaged.purePursuit.referencePath;
    if size(refPath,2) >= 2
        helperOptions.TargetPath = refPath(:,1:3);
    end
elseif isfield(logStaged, 'targetPositions') && ~isempty(logStaged.targetPositions)
    helperOptions.TargetPath = logStaged.targetPositions.';
end

if isfield(logStaged, 'stageLogs') && isstruct(logStaged.stageLogs)
    if isfield(logStaged.stageLogs, 'stageC') && isstruct(logStaged.stageLogs.stageC)
        stageC = logStaged.stageLogs.stageC;
        if isfield(stageC, 'referenceBaseStates') && ~isempty(stageC.referenceBaseStates)
            helperOptions.ReferenceBasePath = stageC.referenceBaseStates;
            helperOptions.ReferenceBaseLabel = "Stage C Reference (GIK)";
        end
        if isfield(stageC, 'execBaseStates') && ~isempty(stageC.execBaseStates)
            helperOptions.ExecutedBaseLabel = "Stage C Executed Base";
        end
    end
    if isfield(logStaged.stageLogs, 'stageB') && isstruct(logStaged.stageLogs.stageB)
        stageB = logStaged.stageLogs.stageB;
        if isfield(stageB, 'execBaseStates') && ~isempty(stageB.execBaseStates)
            helperOptions.StageBPath = stageB.execBaseStates;
            helperOptions.StageBLabel = "Stage B Executed Base";
        end
    end
elseif isfield(logStaged, 'referenceBaseStates') && ~isempty(logStaged.referenceBaseStates)
    helperOptions.ReferenceBasePath = logStaged.referenceBaseStates;
end

fields = fieldnames(helperOptions);
for i = 1:numel(fields)
    helperArgs = [helperArgs, {fields{i}, helperOptions.(fields{i})}]; %#ok<AGROW>
end

[stageBoundaries, stageLabels] = detectStageBoundaries(logStaged);
% CRITICAL: Adjust stage boundaries for sampling
% detectStageBoundaries returns boundaries in RAW frame space,
% but we've already sampled the trajectories with SampleStep
if opts.SampleStep > 1 && ~isempty(stageBoundaries)
    % Convert cumulative boundaries to per-stage counts
    stageCounts = [stageBoundaries(1), diff(stageBoundaries)];
    % Apply sampling to each stage count
    sampledCounts = arrayfun(@(c) length(1:opts.SampleStep:c), stageCounts);
    % Reconstruct cumulative boundaries in sampled space
    stageBoundaries = cumsum(sampledCounts);
end
helperArgs = [helperArgs, {'StageBoundaries', stageBoundaries}, {'StageLabels', stageLabels}];

if isfield(logStaged, 'floorDiscs') && ~isempty(logStaged.floorDiscs)
    discs = logStaged.floorDiscs;
    if isfield(logStaged, 'distanceSpecs') && ~isempty(logStaged.distanceSpecs)
        specs = logStaged.distanceSpecs;
        for i = 1:min(numel(discs), numel(specs))
            lb = specs(i).Bounds(1);
            safety = 0;
            if isfield(discs(i), 'SafetyMargin') && ~isempty(discs(i).SafetyMargin)
                safety = discs(i).SafetyMargin;
            end
            discs(i).DistanceMargin = max(lb - discs(i).Radius - safety, 0);
            if ~isfield(discs(i), 'Height')
                discs(i).Height = 0.05;
            end
            discs(i).height = discs(i).Height;
        end
    end
    helperArgs = [helperArgs, {'Obstacles', struct('discs', {discs})}];
end

robotStruct = copy(robot);
robotStruct.DataFormat = 'struct';

gik9dof.viz.animate_whole_body(robotStruct, armJointNames, armTrajectory, armTimes, basePose, baseTimes, eePoses, helperArgs{:});
end

function t = resolveTimes(logStruct, numSamples)
if isfield(logStruct, 'time') && numel(logStruct.time) == numSamples
    t = logStruct.time;
elseif isfield(logStruct, 'timestamps') && numel(logStruct.timestamps) == numSamples - 1
    t = [0, logStruct.timestamps];
elseif isfield(logStruct, 'rateHz') && logStruct.rateHz > 0
    t = (0:numSamples-1) / logStruct.rateHz;
else
    t = 0:numSamples-1;
end
end

function eePath = computeEEPath(robot, qSeries)
numFrames = size(qSeries, 2);
eePath = zeros(numFrames, 3);
for i = 1:numFrames
    T = getTransform(robot, qSeries(:, i), 'left_gripper_link');
    eePath(i, :) = tform2trvec(T);
end
end

function [boundaries, labels] = detectStageBoundaries(logStaged)
boundaries = size(logStaged.qTraj, 2);
labels = "Tracking";
if isfield(logStaged, 'stageLogs')
    counts = [];
    labels = strings(1,0);
    if isfield(logStaged.stageLogs, 'stageA') && isfield(logStaged.stageLogs.stageA, 'qTraj')
        counts(end+1) = size(logStaged.stageLogs.stageA.qTraj, 2);
        labels(end+1) = "Stage A";
    end
    if isfield(logStaged.stageLogs, 'stageB') && isfield(logStaged.stageLogs.stageB, 'qTraj')
        counts(end+1) = size(logStaged.stageLogs.stageB.qTraj, 2);
        labels(end+1) = "Stage B";
    end
    if isfield(logStaged.stageLogs, 'stageC') && isfield(logStaged.stageLogs.stageC, 'qTraj')
        counts(end+1) = size(logStaged.stageLogs.stageC.qTraj, 2);
        labels(end+1) = "Stage C";
    end
    if ~isempty(counts)
        boundaries = cumsum(counts);
    end
end
if isempty(labels)
    labels = "Tracking";
end
if isempty(boundaries)
    boundaries = size(logStaged.qTraj, 2);
end
end
