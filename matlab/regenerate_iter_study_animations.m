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
    options.CreatePurePursuitVideo (1,1) logical = false
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
    'LogPath', "", 'VideoPath', "", 'PurePursuitVideoPath', {strings(0,1)}, ...
    'PurePursuitCommands', {strings(0,1)}), numel(logFiles), 1);

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

    if ~exist('sampleTime','var') || isempty(sampleTime)
        if numel(timeVec) > 1
            sampleTime = mean(diff(timeVec));
        else
            sampleTime = 1 / getfieldwithdefault(log, 'rateHz', 30);
        end
    end

    basePathFull = qTraj(baseIdx, :).';
    baseTimesFull = timeVec;
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

    videoName = sprintf('anim_wholebody_%s_iter%s.mp4', modeLabel, iterLabel);
    videoPath = fullfile(resultsDir, videoName);

    obstacles = [];
    if isfield(log, 'floorDiscs') && ~isempty(log.floorDiscs)
        obstacles = log.floorDiscs;
        if isfield(log, 'environment') && isfield(log.environment, 'DistanceMargin')
            distMargin = log.environment.DistanceMargin;
        else
            distMargin = 0;
        end
        for d = 1:numel(obstacles)
            obstacles(d).DistanceMargin = distMargin;
        end
    end

    pureCmdPaths = strings(0,1);
    pureVideoPaths = strings(0,1);

    try
        renderWholeBodyAnimation(log, robotColumn, armJointNames, armIdx, baseIdx, videoPath, ...
            'StageLabels', stageLabels, 'StageBoundaries', stageBounds, ...
            'SampleStep', options.SampleStep, 'VideoFrameRate', options.VideoFrameRate, ...
            'FigureScale', options.FigureScale, 'Obstacles', obstacles);
    catch animErr
        warning('regenerate_iter_study_animations:RenderFailed', ...
            'Failed to render %s: %s', logFiles(k).name, animErr.message);
        videoPath = "";
    end

    if options.CreatePurePursuitVideo
        ppJobs = struct('Label', {}, 'Log', {}, 'Pure', {}, 'StageLabels', {}, 'StageBoundaries', {});

        if isfield(log, 'purePursuit') && ~isempty(log.purePursuit)
            ppJobs(end+1) = struct('Label', modeLabel, 'Log', log, ...
                'Pure', log.purePursuit, 'StageLabels', stageLabels, ...
                'StageBoundaries', stageBounds); %#ok<AGROW>
        end

        if isfield(log, 'stageLogs') && isstruct(log.stageLogs)
            stageNames = fieldnames(log.stageLogs);
            for si = 1:numel(stageNames)
                stageLog = log.stageLogs.(stageNames{si});
                if isfield(stageLog, 'purePursuit') && ~isempty(stageLog.purePursuit)
                    stageLabel = string(stageNames{si});
                    ppJobs(end+1) = struct('Label', stageLabel, ...
                        'Log', stageLog, 'Pure', stageLog.purePursuit, ...
                        'StageLabels', string(prettyStageLabel(stageLabel)), ...
                        'StageBoundaries', []); %#ok<AGROW>
                end
            end
        end

        for jobIdx = 1:numel(ppJobs)
            job = ppJobs(jobIdx);
            ppData = job.Pure;

            if ~isfield(ppData, 'simulation') || ~isfield(ppData.simulation, 'poses') || isempty(ppData.simulation.poses)
                continue
            end

            sampleTimePP = getfieldwithdefault(ppData, 'sampleTime', []);
            if isempty(sampleTimePP) || ~isfinite(sampleTimePP) || sampleTimePP <= 0
                rateHzJob = getfieldwithdefault(job.Log, 'rateHz', getfieldwithdefault(log, 'rateHz', 25));
                if ~isfinite(rateHzJob) || rateHzJob <= 0
                    sampleTimePP = 0.1;
                else
                    sampleTimePP = 1 / rateHzJob;
                end
            end

            executedPath = ppData.simulation.poses;
            if isfield(ppData, 'referencePath') && ~isempty(ppData.referencePath)
                executedPath(1,:) = ppData.referencePath(1,:);
            end
            baseTimesPP = (0:size(executedPath,1)-1)' * sampleTimePP;

            stageSlug = makeStageSlug(job.Label);
            videoNamePP = sprintf('anim_wholebody_%s_purepursuit_%s_iter%s.mp4', stageSlug, modeLabel, iterLabel);
            videoPathPP = fullfile(resultsDir, videoNamePP);

            try
                renderWholeBodyAnimation(job.Log, robotColumn, armJointNames, armIdx, baseIdx, videoPathPP, ...
                    'StageLabels', job.StageLabels, 'StageBoundaries', job.StageBoundaries, ...
                    'SampleStep', options.SampleStep, 'VideoFrameRate', options.VideoFrameRate, ...
                    'FigureScale', options.FigureScale, 'BaseOverride', executedPath, ...
                    'BaseTimesOverride', baseTimesPP, 'Obstacles', obstacles);
                pureVideoPaths(end+1,1) = videoPathPP;
            catch simAnimErr
                warning('regenerate_iter_study_animations:PurePursuitRenderFailed', ...
                    'Failed to render pure pursuit animation for %s (%s): %s', ...
                    logFiles(k).name, stageSlug, simAnimErr.message);
            end

            if isfield(ppData.simulation, 'commands') && ~isempty(ppData.simulation.commands)
                cmdNamePP = sprintf('purepursuit_cmd_%s_%s_iter%s.csv', stageSlug, modeLabel, iterLabel);
                cmdPathPP = fullfile(resultsDir, cmdNamePP);
                cmdTablePP = [baseTimesPP, ppData.simulation.commands];
                writematrix(cmdTablePP, cmdPathPP);
                pureCmdPaths(end+1,1) = cmdPathPP;
            end
        end
    end

    outputs(k).Mode = modeLabel;
    outputs(k).IterationCap = iterVal;
    outputs(k).LogPath = logPath;
    outputs(k).VideoPath = videoPath;
    outputs(k).PurePursuitVideoPath = pureVideoPaths;
    outputs(k).PurePursuitCommands = pureCmdPaths;
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

function label = prettyStageLabel(stageName)
stageStr = string(stageName);
if strlength(stageStr) == 0
    label = "Stage";
    return
end

stageStr = strtrim(stageStr);
lowerStage = lower(stageStr);
if startsWith(lowerStage, "stage")
    suffix = extractAfter(stageStr, strlength("stage"));
    if strlength(suffix) == 0
        label = "Stage";
    else
        suffixText = regexprep(char(suffix), '([A-Z])', ' $1');
        suffixText = regexprep(suffixText, '_', ' ');
        suffixText = strtrim(suffixText);
        if isempty(suffixText)
            suffixText = char(suffix);
        end
        if ~isempty(suffixText)
            suffixText(1) = upper(suffixText(1));
            if numel(suffixText) > 1
                suffixText(2:end) = lower(suffixText(2:end));
            end
        end
        label = "Stage " + string(suffixText);
    end
else
    text = regexprep(char(stageStr), '([A-Z])', ' $1');
    text = regexprep(text, '_', ' ');
    text = regexprep(text, '\s+', ' ');
    text = strtrim(text);
    if isempty(text)
        text = char(stageStr);
    end
    if ~isempty(text)
        text(1) = upper(text(1));
        if numel(text) > 1
            text(2:end) = lower(text(2:end));
        end
    end
    label = string(text);
end
end

function slug = makeStageSlug(stageName)
labelStr = char(string(stageName));
labelStr = lower(strtrim(labelStr));
labelStr = regexprep(labelStr, '\s+', '_');
labelStr = regexprep(labelStr, '[^a-z0-9_]', '');
if isempty(labelStr)
    slug = "full";
else
    slug = string(labelStr);
end
end
