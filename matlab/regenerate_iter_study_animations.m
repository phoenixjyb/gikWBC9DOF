function outputs = regenerate_iter_study_animations(resultsDir, options)
%REGENERATE_ITER_STUDY_ANIMATIONS Produce dual-view animations for GIK logs.
%
%   regenerate_iter_study_animations(resultsDir) scans the specified
%   results directory for log_*.mat files (as produced by
%   run_gik_iteration_study) and renders dual-view animations using
%   gik9dof.viz.animate_whole_body. Videos are saved alongside the logs
%   using the "anim_wholebody_<mode>_iterXXXX.mp4" naming convention.
%
%   outputs = regenerate_iter_study_animations(___) returns a struct array
%   describing each generated animation (mode, iteration cap, paths).
%
%   Name-value options:
%       SampleStep     - Positive integer stride for animation frames (default 5)
%       VideoFrameRate - Frame rate for the output MP4 (default 25)
%       FigureScale    - Scale factor applied to default figure size (default 1.0)
%       StageLabels    - Optional string array used for stage captions
%
%   Example:
%       regenerate_iter_study_animations('results/20251007_122040_gik_iter_study_iterCompare');

arguments
    resultsDir (1,1) string
    options.SampleStep (1,1) double {mustBePositive, mustBeInteger} = 5
    options.VideoFrameRate (1,1) double {mustBePositive} = 25
    options.FigureScale (1,1) double {mustBePositive} = 1.0
    options.StageLabels (1,:) string = ["Phase 1", "Phase 2", "Phase 3"]
end

resultsDir = string(resultsDir);
if ~isfolder(resultsDir)
    error('regenerate_iter_study_animations:ResultsDirMissing', ...
        'Directory does not exist: %s', resultsDir);
end

logFiles = dir(fullfile(resultsDir, 'log_*.mat'));
if isempty(logFiles)
    warning('regenerate_iter_study_animations:NoLogsFound', ...
        'No log_*.mat files found in %s.', resultsDir);
    outputs = struct('Mode', {}, 'IterationCap', {}, 'LogPath', {}, 'VideoPath', {});
    return
end

robotColumn = gik9dof.createRobotModel();
robotStruct = copy(robotColumn);
robotStruct.DataFormat = 'struct';
configTools = gik9dof.configurationTools(robotColumn);
jointNames = string(configTools.templateJointNames());
baseIdx = [find(jointNames == 'joint_x', 1), ...
    find(jointNames == 'joint_y', 1), find(jointNames == 'joint_theta', 1)];
if any(isnan(baseIdx))
    error('regenerate_iter_study_animations:MissingBaseJoints', ...
        'Could not locate base joints in robot model.');
end
armMask = true(1, numel(jointNames));
armMask(baseIdx) = false;
armIdx = find(armMask);
armJointNames = cellstr(jointNames(armIdx));

outputs = repmat(struct('Mode', "", 'IterationCap', NaN, ...
    'LogPath', "", 'VideoPath', ""), numel(logFiles), 1);

for k = 1:numel(logFiles)
    logPath = fullfile(logFiles(k).folder, logFiles(k).name);
    data = load(logPath);
    if ~isfield(data, 'log')
        warning('regenerate_iter_study_animations:MissingLogVar', ...
            'File %s does not contain variable ''log''; skipping.', logFiles(k).name);
        continue
    end
    log = data.log;

    qTraj = log.qTraj;
    if isempty(qTraj)
        warning('regenerate_iter_study_animations:EmptyTrajectory', ...
            'Log %s has empty qTraj; skipping.', logFiles(k).name);
        continue
    end

    numSamples = size(qTraj, 2);
    if isfield(log, 'time') && numel(log.time) == numSamples
        timeVec = log.time(:)';
    else
        sampleTime = 1 / getfieldwithdefault(log, 'rateHz', 30);
        timeVec = (0:numSamples-1) * sampleTime;
    end

    baseTrajectory = qTraj(baseIdx, :).';
    armTrajectory = qTraj(armIdx, :).';
    armTimes = timeVec;
    baseTimes = timeVec;

    eeName = "left_gripper_link";
    eePath = zeros(numSamples, 3);
    for n = 1:numSamples
        qFrame = configTools.column(qTraj(:, n));
        T = getTransform(robotColumn, qFrame, eeName);
        eePath(n, :) = tform2trvec(T);
    end

    mode = extractBetween(string(logFiles(k).name), 'log_', '_iter');
    if isempty(mode)
        mode = "unknown";
    end
    iterStr = extractBetween(string(logFiles(k).name), 'iter', '.mat');
    if isempty(iterStr)
        iterVal = NaN;
    else
        iterVal = str2double(iterStr);
    end

    if isempty(mode)
        modeLabel = "unknown";
    else
        modeLabel = mode(1);
    end
    if isempty(iterStr)
        iterLabel = "";
    else
        iterLabel = iterStr(1);
    end

    stageLabels = options.StageLabels;
    stageBounds = [];
    if isfield(log, 'stageLogs') && isstruct(log.stageLogs)
        names = fieldnames(log.stageLogs);
        counts = zeros(1, numel(names));
        for ni = 1:numel(names)
            stageLog = log.stageLogs.(names{ni});
            if isfield(stageLog, 'qTraj') && ~isempty(stageLog.qTraj)
                counts(ni) = size(stageLog.qTraj, 2);
            end
        end
        counts = counts(counts > 0);
        if ~isempty(counts)
            stageBounds = cumsum(counts);
            if numel(stageLabels) < numel(stageBounds)
                stageLabels(end+1:numel(stageBounds)) = stageLabels(end);
            end
        end
    end

    sampleIdx = 1:options.SampleStep:numSamples;
    armTrajectory = armTrajectory(sampleIdx, :);
    armTimes = armTimes(sampleIdx);
    baseTrajectory = baseTrajectory(sampleIdx, :);
    baseTimes = baseTimes(sampleIdx);
    eePath = eePath(sampleIdx, :);
    if ~isempty(stageBounds)
        stageBounds = ceil(stageBounds / options.SampleStep);
        stageBounds(stageBounds < 1) = 1;
        stageBounds(stageBounds > size(armTrajectory,1)) = size(armTrajectory,1);
        stageBounds = unique(stageBounds);
    end

    videoName = sprintf('anim_wholebody_%s_iter%s.mp4', modeLabel, iterLabel);
    videoPath = fullfile(resultsDir, videoName);

    obstacles = [];
    if isfield(log, 'floorDiscs') && ~isempty(log.floorDiscs)
        obstacles = log.floorDiscs;
    end

    try
        gik9dof.viz.animate_whole_body(robotStruct, armJointNames, armTrajectory, armTimes, ...
            baseTrajectory, baseTimes, eePath, ...
            'VideoFile', videoPath, ...
            'VideoFrameRate', options.VideoFrameRate, ...
            'StageLabels', stageLabels, ...
            'StageBoundaries', stageBounds, ...
            'FigureScale', options.FigureScale, ...
            'TargetPath', eePath, ...
            'Obstacles', obstacles);
    catch animErr
        warning('regenerate_iter_study_animations:RenderFailed', ...
            'Failed to render %s: %s', logFiles(k).name, animErr.message);
        videoPath = "";
    end

    outputs(k).Mode = modeLabel;
    outputs(k).IterationCap = iterVal;
    outputs(k).LogPath = logPath;
    outputs(k).VideoPath = videoPath;
end

if nargout == 0
    clear outputs
end
end

function val = getfieldwithdefault(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    val = s.(fieldName);
else
    val = defaultValue;
end
end
