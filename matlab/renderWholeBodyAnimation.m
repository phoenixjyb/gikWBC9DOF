function renderWholeBodyAnimation(log, robot, armJointNames, armIdx, baseIdx, videoPath, options)
%RENDERWHOLEBODYANIMATION Render dual-view animation using animate_whole_body.
%   renderWholeBodyAnimation(log, robot, armJointNames, armIdx, baseIdx, videoPath)
%   extracts arm and base trajectories from LOG (a GIK controller log) and
%   reuses gik9dof.viz.animate_whole_body to generate the standard dual-view
%   animation. The helper optionally accepts a chassis trajectory override
%   (BaseOverride/BaseTimesOverride) so that pure-pursuit replays can be
%   visualised without duplicating the rendering pipeline.
%
%   Required inputs:
%     log            - struct containing fields qTraj (joint x samples) and,
%                      optionally, time and rateHz.
%     robot          - rigidBodyTree used for animation.
%     armJointNames  - cell array of arm joint names (matching armIdx order).
%     armIdx         - indices of arm joints in qTraj.
%     baseIdx        - indices of chassis joints in qTraj (order [x y yaw]).
%     videoPath      - target MP4 path (string/char).
%
%   Name-value options:
%     StageLabels       - string array of stage captions.
%     StageBoundaries   - cumulative frame counts per stage (original sampling).
%     SampleStep        - positive integer subsampling factor for arm trajectory.
%     VideoFrameRate    - output MP4 frame rate (default 25).
%     FigureScale       - scale factor for figure size (default 1.0).
%     BaseOverride      - Nx3 override poses [x y yaw] (optional).
%     BaseTimesOverride - 1xN time vector for BaseOverride (required when BaseOverride used).

arguments
    log (1,1) struct
    robot (1,1) rigidBodyTree
    armJointNames (1,:) cell
    armIdx (1,:) double
    baseIdx (1,:) double
    videoPath
    options.StageLabels (1,:) string = ["Stage 1", "Stage 2", "Stage 3"]
    options.StageBoundaries double = []
    options.SampleStep (1,1) double {mustBePositive, mustBeInteger} = 1
    options.VideoFrameRate (1,1) double {mustBePositive} = 25
    options.FigureScale (1,1) double {mustBePositive} = 1.0
    options.BaseOverride double = []
    options.BaseTimesOverride double = []
    options.TargetPath double = []
    options.Obstacles = []
end

if ~isfield(log, 'qTraj') || isempty(log.qTraj)
    error('renderWholeBodyAnimation:EmptyTrajectory', ...
        'Log does not contain qTraj data suitable for animation.');
end

% Work on a copy so we can enforce struct data format for helper utilities.
robot = copy(robot);
robot.DataFormat = 'struct';

qTraj = double(log.qTraj);
numSamples = size(qTraj, 2);
if numSamples == 0
    error('renderWholeBodyAnimation:NoSamples', ...
        'Trajectory contains zero samples.');
end

timeVecFull = extractTimeVector(log, numSamples);

sampleIdx = 1:options.SampleStep:numSamples;
if sampleIdx(end) ~= numSamples
    sampleIdx(end+1) = numSamples; %#ok<AGROW>
end
sampleIdx = unique(sampleIdx);

armTrajectory = qTraj(armIdx, sampleIdx).';
armTimes = timeVecFull(sampleIdx);

if ~isempty(options.BaseOverride)
    basePose = double(options.BaseOverride);
    if size(basePose,2) ~= 3
        error('renderWholeBodyAnimation:BaseOverrideSize', ...
            'BaseOverride must be an Nx3 matrix [x y yaw].');
    end
    if isempty(options.BaseTimesOverride)
        error('renderWholeBodyAnimation:BaseTimesRequired', ...
            'BaseTimesOverride must be provided when BaseOverride is supplied.');
    end
    baseTimes = double(options.BaseTimesOverride(:)');
    if numel(baseTimes) ~= size(basePose,1)
        error('renderWholeBodyAnimation:BaseTimesLength', ...
            'BaseTimesOverride length (%d) must match BaseOverride rows (%d).', ...
            numel(baseTimes), size(basePose,1));
    end
else
    basePose = qTraj(baseIdx, :).';
    baseTimes = timeVecFull;
end

stageBoundsResampled = remapStageBoundaries(options.StageBoundaries, sampleIdx);

if isempty(options.TargetPath) && isfield(log, 'purePursuit') && ...
        isfield(log.purePursuit, 'referencePath') && ~isempty(log.purePursuit.referencePath)
    options.TargetPath = log.purePursuit.referencePath(:,1:3);
end
if isempty(options.Obstacles) && isfield(log, 'floorDiscs') && ~isempty(log.floorDiscs)
    options.Obstacles = struct('discs', {log.floorDiscs});
end

vizArgs = {'VideoFile', char(string(videoPath)), ...
    'VideoFrameRate', options.VideoFrameRate, ...
    'FigureScale', options.FigureScale, ...
    'StageLabels', options.StageLabels, ...
    'StageBoundaries', stageBoundsResampled};
if ~isempty(options.TargetPath)
    vizArgs = [vizArgs, {'TargetPath', options.TargetPath}]; %#ok<AGROW>
end
if ~isempty(options.Obstacles)
    vizArgs = [vizArgs, {'Obstacles', options.Obstacles}]; %#ok<AGROW>
end

gik9dof.viz.animate_whole_body(robot, armJointNames, armTrajectory, armTimes, ...
    basePose, baseTimes, [], vizArgs{:});
end

function timeVec = extractTimeVector(log, numSamples)
if isfield(log, 'time') && ~isempty(log.time)
    rawTime = double(log.time(:)');
    if numel(rawTime) >= numSamples
        timeVec = rawTime(1:numSamples);
        return
    end
end

rateHz = fieldOrDefault(log, 'rateHz', 25);
if ~isfinite(rateHz) || rateHz <= 0
    sampleTime = 0.1;
else
    sampleTime = 1 / rateHz;
end
timeVec = (0:numSamples-1) * sampleTime;
end

function stageBounds = remapStageBoundaries(originalBounds, sampleIdx)
if isempty(originalBounds)
    stageBounds = [];
    return
end

originalBounds = double(originalBounds(:)');
sampleIdx = double(sampleIdx(:)');
numFrames = numel(sampleIdx);

stageBounds = arrayfun(@(b) sum(sampleIdx <= b), originalBounds);
stageBounds(stageBounds < 1) = 1;
stageBounds(stageBounds > numFrames) = numFrames;
stageBounds = unique(stageBounds);
if isempty(stageBounds) || stageBounds(end) ~= numFrames
    stageBounds(end+1) = numFrames;
end
end

function val = fieldOrDefault(s, name, defaultValue)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = defaultValue;
end
end
