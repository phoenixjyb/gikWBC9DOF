function report = analyze_stage_collisions(logPath, options)
%ANALYZE_STAGE_COLLISIONS Summarise collision metrics for staged GIK runs.
%
%   report = ANALYZE_STAGE_COLLISIONS(logPath) loads the specified log_*.mat
%   file (produced by gik9dof.trackReferenceTrajectory) and evaluates
%   collision statistics for Stage A and Stage C. The output struct contains
%   per-stage summaries of self-collision flags and obstacle clearances.
%
%   report = ANALYZE_STAGE_COLLISIONS(logPath, 'StageNames', ["stageA","stageC","stageB"])
%   allows custom stage selection.
%
%   Example:
%       rpt = analyze_stage_collisions('results/.../log_staged_purehyb_iter150.mat');
%
%   The function applies default collision meshes via gik9dof.collisionTools
%   before calling checkCollision.
%
%   NOTE: Obstacles are drawn from log.environment.FloorDiscs. If DistanceMargin
%   is present it is included in the clearance computations.

arguments
    logPath (1,1) string
    options.StageNames (1,:) string = ["stageA","stageC"]
end

if ~isfile(logPath)
    error('analyze_stage_collisions:MissingLog', 'File not found: %s', logPath);
end

data = load(logPath);
if ~isfield(data, 'log')
    error('analyze_stage_collisions:MissingLogVar', ...
        'File %s does not contain a variable named ''log''.', logPath);
end
log = data.log;

[robotColumn, ~] = gik9dof.createRobotModel();
configTools = gik9dof.configurationTools(robotColumn);
jointNames = string(configTools.templateJointNames());

[robotRow, ~] = gik9dof.createRobotModel("DataFormat", "row");
colTools = gik9dof.collisionTools(robotRow);
colTools.apply();

baseIdx = [find(jointNames == "joint_x", 1), ...
           find(jointNames == "joint_y", 1), ...
           find(jointNames == "joint_theta", 1)];
if numel(baseIdx) ~= 3
    error('analyze_stage_collisions:BaseIndices', 'Failed to locate base joint indices.');
end
armIdx = setdiff(1:numel(jointNames), baseIdx);

floorDiscs = [];
distanceMargin = 0;
if isfield(log, 'environment') && isstruct(log.environment)
    env = log.environment;
    if isfield(env, 'FloorDiscs')
        floorDiscs = env.FloorDiscs;
    end
    if isfield(env, 'DistanceMargin') && ~isempty(env.DistanceMargin)
        distanceMargin = double(env.DistanceMargin);
    end
end

stageList = options.StageNames;
report = struct('stage', {}, 'numSamples', {}, 'selfCollisionCount', {}, ...
    'minSelfSep', {}, 'discClearance', {}, 'notes', {});

for idx = 1:numel(stageList)
    stageName = stageList(idx);
    stageData = getStageLog(log, stageName);
    if isempty(stageData)
        entry = stageEntry(stageName, 0);
        entry.notes = "Stage not present";
        report(end+1) = entry; %#ok<AGROW>
        continue
    end

    qTraj = stageData.qTraj;
    if isempty(qTraj)
        entry = stageEntry(stageName, 0);
        entry.notes = "Empty trajectory";
        report(end+1) = entry; %#ok<AGROW>
        continue
    end

    numSamples = size(qTraj, 2);

    % --- Disc clearance ---
    discSummary = computeDiscClearance(qTraj(baseIdx, :), floorDiscs, distanceMargin);

    % --- Self collision check ---
    [selfCount, minSep] = computeSelfCollision(qTraj, robotRow);

    entry = stageEntry(stageName, numSamples);
    entry.selfCollisionCount = selfCount;
    entry.minSelfSep = minSep;
    entry.discClearance = discSummary;
    if selfCount > 0
        entry.notes = sprintf('Self collisions detected (%d samples).', selfCount);
    else
        entry.notes = "No self-collisions detected";
    end
    report(end+1) = entry; %#ok<AGROW>
end

end

function stageLog = getStageLog(logStruct, stageName)
if ~isfield(logStruct, 'stageLogs') || ~isstruct(logStruct.stageLogs)
    stageLog = [];
    return
end
if isfield(logStruct.stageLogs, stageName)
    stageLog = logStruct.stageLogs.(stageName);
else
    stageLog = [];
end
end

function entry = stageEntry(stageName, numSamples)
entry = struct('stage', string(stageName), ...
    'numSamples', double(numSamples), ...
    'selfCollisionCount', 0, ...
    'minSelfSep', NaN, ...
    'discClearance', struct(), ...
    'notes', "");
end

function summary = computeDiscClearance(baseTraj, discs, distanceMargin)
summary = struct('discName', {}, 'minClearance', {}, 'minDistance', {}, ...
    'requiredRadius', {});
if isempty(discs)
    return
end

baseX = baseTraj(1, :).';
baseY = baseTraj(2, :).';

for d = 1:numel(discs)
    disc = discs(d);
    if isfield(disc, 'Center')
        center = double(disc.Center(:).');
    elseif isfield(disc, 'center')
        center = double(disc.center(:).');
    else
        center = [0 0];
    end
    if numel(center) < 2
        center = [center, 0];
    end
    center = center(1:2);

    radius = pickDiscField(disc, 'Radius', 'radius', 0.0);
    safety = pickDiscField(disc, 'SafetyMargin', 'safetyMargin', 0.0);

    dist = hypot(baseX - center(1), baseY - center(2));
    required = radius + safety + distanceMargin;
    clearance = dist - required;

    summary(end+1) = struct( ...
        'discName', discName(disc, d), ...
        'minClearance', min(clearance), ...
        'minDistance', min(dist), ...
        'requiredRadius', required ...
    ); %#ok<AGROW>
end
end

function name = discName(disc, index)
if isfield(disc, 'Name') && ~isempty(disc.Name)
    name = string(disc.Name);
elseif isfield(disc, 'name') && ~isempty(disc.name)
    name = string(disc.name);
else
    name = "disc" + index;
end
end

function value = pickDiscField(disc, primaryField, secondaryField, defaultVal)
if isfield(disc, primaryField) && ~isempty(disc.(primaryField))
    value = double(disc.(primaryField));
elseif isfield(disc, secondaryField) && ~isempty(disc.(secondaryField))
    value = double(disc.(secondaryField));
else
    value = defaultVal;
end
if isempty(value)
    value = defaultVal;
end
value = value(1);
end

function [collisionCount, minSeparation] = computeSelfCollision(qTraj, robotRow)
numSamples = size(qTraj, 2);
collisionCount = 0;
minSeparation = Inf;

for k = 1:numSamples
    qRow = qTraj(:,k).';
    [isColliding, sepDist] = checkCollision(robotRow, qRow, ...
        'SkippedSelfCollisions', 'parent');
    if isColliding
        collisionCount = collisionCount + 1;
    end
    if ~isempty(sepDist)
        localMin = min(sepDist(:));
        minSeparation = min(minSeparation, localMin);
    end
end

if isinf(minSeparation)
    minSeparation = NaN;
end
end
