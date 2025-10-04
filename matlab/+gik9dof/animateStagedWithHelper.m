function animateStagedWithHelper(logStaged, varargin)
%ANIMATESTAGEDWITHHELPER Visualise staged runs using internal helper.
%
parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'Robot', gik9dof.createRobotModel(), @(x) isa(x,'rigidBodyTree'));
addParameter(parser, 'SampleStep', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, 'FrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
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

if isfield(logStaged, 'eePositions') && ~isempty(logStaged.eePositions)
    eePoses = logStaged.eePositions(:, 1:opts.SampleStep:end)';
else
    eePoses = computeEEPath(robot, qTraj(:, 1:opts.SampleStep:end));
end

helperArgs = {};
if ~isempty(opts.ExportVideo)
    helperArgs = [helperArgs, {'VideoFile', opts.ExportVideo}, {'VideoFrameRate', opts.FrameRate}]; %#ok<AGROW>
end

% Pass through any helper options provided by the caller
fields = fieldnames(opts.HelperOptions);
for i = 1:numel(fields)
    helperArgs = [helperArgs, {fields{i}, opts.HelperOptions.(fields{i})}]; %#ok<AGROW>
end

[stageBoundaries, stageLabels] = detectStageBoundaries(logStaged);
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
