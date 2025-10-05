function pipeline = runStagedTrajectory(robot, trajStruct, options)
%RUNSTAGEDTRAJECTORY Execute staged pipeline: arm-only, base-only, combined.
%   pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, options)
%   orchestrates three sequential phases that replicate the legacy staged
%   behaviour: Stage A (arm-only ramp-up), Stage B (base-only alignment), and
%   Stage C (full-body tracking). The output pipeline struct contains the
%   combined log plus per-stage logs.
%
%   Required options:
%       InitialConfiguration - Column vector of joint positions.
%       ConfigTools         - configurationTools(robot) struct.
%
%   Optional name-value:
%       DistanceSpecs       - Distance constraint specs for Stage C (struct array).
%       RateHz              - Control loop rate (default 100).
%       Verbose             - Verbosity flag (default false).
%       DistanceWeight      - Weight used when DistanceSpecs empty (default 0.5).
%
arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    options.InitialConfiguration (:,1) double
    options.ConfigTools (1,1) struct
    options.DistanceSpecs struct = struct([])
    options.DistanceWeight (1,1) double = 0.5
    options.RateHz (1,1) double {mustBePositive} = 100
    options.Verbose (1,1) logical = false
    options.CommandFcn = []
    options.FloorDiscs struct = struct([])
    options.UseStageBHybridAStar (1,1) logical = false
    options.StageBHybridResolution (1,1) double = 0.1
    options.StageBHybridSafetyMargin (1,1) double = 0.15
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.5
    options.StageBMaxLinearSpeed (1,1) double = 1.5
    options.StageBMaxYawRate (1,1) double = 3.0
    options.StageBMaxJointSpeed (1,1) double = 1.0
    options.EnvironmentConfig struct = struct()
end

configTools = options.ConfigTools;
q0 = configTools.column(options.InitialConfiguration);
NA = 50;  % samples for Stage A
NB = 50;  % samples for Stage B

jointNames = string(configTools.templateJointNames());
idx_x = find(jointNames == "joint_x", 1);
idx_y = find(jointNames == "joint_y", 1);
idx_theta = find(jointNames == "joint_theta", 1);
baseIdx = [idx_x, idx_y, idx_theta];
armIdx = setdiff(1:numel(jointNames), baseIdx);

velLimits = struct('BaseIndices', baseIdx, ...
    'ArmIndices', armIdx, ...
    'MaxLinearSpeed', options.StageBMaxLinearSpeed, ...
    'MaxYawRate', options.StageBMaxYawRate, ...
    'MaxJointSpeed', options.StageBMaxJointSpeed);

% Extract first desired pose
TrefAll = trajStruct.Poses;
numWaypoints = size(TrefAll, 3);
T1 = TrefAll(:,:,1);

% Current pose at q0
T0 = getTransform(robot, q0, trajStruct.EndEffectorName);

%% Stage A: arm-only ramp-up (match orientation & z while base fixed)
stageAPoses = generateStagePoses(T0, T1, NA, 'arm');
trajA = struct('Poses', stageAPoses);
trajA.EndEffectorName = trajStruct.EndEffectorName;

bundleA = gik9dof.createGikSolver(robot);
lockJointBounds(bundleA.constraints.joint, baseIdx, q0);

logA = gik9dof.runTrajectoryControl(bundleA, trajA, ...
    'InitialConfiguration', q0, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

qA_end = logA.qTraj(:, end);

%% Stage B: base-only alignment of x-y while arm frozen
stageBStart = getTransform(robot, qA_end, trajStruct.EndEffectorName);
if options.UseStageBHybridAStar
    try
        stageBStates = planStageBHybridAStarPath(qA_end, baseIdx, T1, options, robot, trajStruct.EndEffectorName, options.FloorDiscs);
    catch plannerErr
        if ~isempty(plannerErr.stack)
            origin = sprintf('%s:%d', plannerErr.stack(1).name, plannerErr.stack(1).line);
        else
            origin = 'unknown origin';
        end
        warning('gik9dof:runStagedTrajectory:HybridAStarFailed', ...
            'Stage B hybrid A* failed (%s at %s); falling back to interpolation.', plannerErr.message, origin);
        stageBStates = [];
    end
    if isempty(stageBStates)
        stageBPoses = generateStagePoses(stageBStart, T1, NB, 'base');
    else
        stageBPoses = statesToEndEffectorPoses(robot, qA_end, baseIdx, stageBStates, trajStruct.EndEffectorName);
    end
else
    stageBPoses = generateStagePoses(stageBStart, T1, NB, 'base');
end
trajB = struct('Poses', stageBPoses);
trajB.EndEffectorName = trajStruct.EndEffectorName;

bundleB = gik9dof.createGikSolver(robot);
lockJointBounds(bundleB.constraints.joint, armIdx, qA_end);

logB = gik9dof.runTrajectoryControl(bundleB, trajB, ...
    'InitialConfiguration', qA_end, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

qB_end = logB.qTraj(:, end);

%% Stage C: full-body tracking of remaining reference
bundleC = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs);

trajC = trajStruct;
logC = struct();
if ~isempty(trajC.Poses)
    logC = gik9dof.runTrajectoryControl(bundleC, trajC, ...
        'InitialConfiguration', qB_end, ...
        'RateHz', options.RateHz, ...
        'CommandFcn', options.CommandFcn, ...
        'Verbose', options.Verbose, ...
        'VelocityLimits', velLimits);
else
    % No remaining waypoints; synthesise empty log
    logC = emptyLog(robot, qB_end, options.RateHz);
end

%% Combine logs
pipeline = mergeStageLogs(logA, logB, logC);
pipeline.mode = "staged";
pipeline.stageLogs = struct('stageA', logA, 'stageB', logB, 'stageC', logC);
pipeline.distanceSpecs = options.DistanceSpecs;
pipeline.rateHz = options.RateHz;
pipeline.referenceTrajectory = trajStruct;
pipeline.environment = options.EnvironmentConfig;
end

function lockJointBounds(jointConstraint, indices, values)
for idx = indices
    jointConstraint.Bounds(idx, :) = [values(idx) values(idx)];
end
end

function poses = generateStagePoses(Tstart, Tend, N, mode)
posStart = tform2trvec(Tstart);
posGoal = tform2trvec(Tend);
quatStart = tform2quat(Tstart);
quatGoal = tform2quat(Tend);
poses = repmat(eye(4), 1, 1, N);
for k = 1:N
    alpha = (k-1)/(N-1);
    switch mode
        case 'arm' % maintain x-y, blend z and orientation
            pos = [posStart(1:2), posStart(3)*(1-alpha) + posGoal(3)*alpha];
            quat = slerpQuaternion(quatStart, quatGoal, alpha);
        case 'base' % blend x-y, hold z and orientation at goal
            pos = [posStart(1)*(1-alpha) + posGoal(1)*alpha, ...
                   posStart(2)*(1-alpha) + posGoal(2)*alpha, ...
                   posGoal(3)];
            quat = quatGoal;
        otherwise
            error('Unknown stage mode: %s', mode);
    end
    poses(:,:,k) = quat2tform(quat);
    poses(1:3,4,k) = pos(:);
end
end

function q = slerpQuaternion(q0, q1, t)
q0 = quatNormalize(q0);
q1 = quatNormalize(q1);
if dot(q0, q1) < 0
    q1 = -q1;
end
cosTheta = dot(q0, q1);
if cosTheta > 0.9995
    q = quatNormalize((1-t)*q0 + t*q1);
    return
end
angle = acos(max(min(cosTheta,1),-1));
q = (sin((1-t)*angle)/sin(angle))*q0 + (sin(t*angle)/sin(angle))*q1;
q = quatNormalize(q);
end

function q = quatNormalize(q)
q = q(:).';
q = q / norm(q);
end

function sliced = sliceTrajectoryStruct(traj, startIdx)
sliced = struct();
fields = fieldnames(traj);
for i = 1:numel(fields)
    name = fields{i};
    value = traj.(name);
    if strcmp(name, 'Poses')
        if size(value,3) >= startIdx
            sliced.Poses = value(:,:,startIdx:end);
        else
            sliced.Poses = zeros(4,4,0);
        end
    elseif ndims(value) == 2 && size(value,2) == size(traj.Poses,3)
        sliced.(name) = value(:, startIdx:end);
    elseif isvector(value) && numel(value) == size(traj.Poses,3)
        sliced.(name) = value(startIdx:end);
    else
        sliced.(name) = value;
    end
end
if ~isfield(sliced, 'Poses')
    sliced.Poses = zeros(4,4,0);
end
if ~isfield(sliced, 'EndEffectorName') && isfield(traj, 'EndEffectorName')
    sliced.EndEffectorName = traj.EndEffectorName;
end
end

function log = emptyLog(robot, qFinal, rateHz)
nJ = numel(qFinal);
log.qTraj = [qFinal];
log.solutionInfo = {};
log.successMask = [];
log.timestamps = [];
log.rateHz = rateHz;
log.completedWaypoints = 0;
log.exitFlags = [];
log.iterations = [];
log.constraintViolationMax = [];
log.targetPoses = zeros(4,4,0);
log.targetPositions = zeros(3,0);
log.targetOrientations = zeros(4,0);
log.eePoses = repmat(eye(4),1,1,0);
log.eePositions = zeros(3,0);
log.eeOrientations = zeros(4,0);
log.positionError = zeros(3,0);
log.positionErrorNorm = [];
log.orientationErrorQuat = zeros(4,0);
log.orientationErrorAngle = [];
log.time = 0;
end

function combined = mergeStageLogs(logA, logB, logC)
qCombined = logA.qTraj;
if ~isempty(logB.qTraj)
    qCombined = [qCombined, logB.qTraj(:,2:end)];
end
if ~isempty(logC.qTraj)
    qCombined = [qCombined, logC.qTraj(:,2:end)];
end

successMask = [logA.successMask, logB.successMask, logC.successMask];

timestamps = logA.timestamps;
offset = 0;
if ~isempty(logA.time)
    offset = logA.time(end);
end
if ~isempty(logB.timestamps)
    timestamps = [timestamps, offset + logB.timestamps];
    offset = offset + logB.time(end);
end
if ~isempty(logC.timestamps)
    timestamps = [timestamps, offset + logC.timestamps];
    offset = offset + logC.time(end);
end

time = [0, timestamps];

exitFlags = [logA.exitFlags, logB.exitFlags, logC.exitFlags];
iterations = [logA.iterations, logB.iterations, logC.iterations];
violations = [logA.constraintViolationMax, logB.constraintViolationMax, logC.constraintViolationMax];

positionError = [logA.positionError, logB.positionError, logC.positionError];
positionErrorNorm = [logA.positionErrorNorm, logB.positionErrorNorm, logC.positionErrorNorm];
orientationErrorQuat = [logA.orientationErrorQuat, logB.orientationErrorQuat, logC.orientationErrorQuat];
orientationErrorAngle = [logA.orientationErrorAngle, logB.orientationErrorAngle, logC.orientationErrorAngle];

targetPoses = cat(3, logA.targetPoses, logB.targetPoses, logC.targetPoses);

targetPositions = [logA.targetPositions, logB.targetPositions, logC.targetPositions];

eePoses = cat(3, logA.eePoses, logB.eePoses, logC.eePoses);
eePositions = [logA.eePositions, logB.eePositions, logC.eePositions];
eeOrientations = [logA.eeOrientations, logB.eeOrientations, logC.eeOrientations];

combined = struct();
combined.qTraj = qCombined;
combined.solutionInfo = [logA.solutionInfo, logB.solutionInfo, logC.solutionInfo];
combined.successMask = successMask;
combined.timestamps = timestamps;
combined.rateHz = logA.rateHz;
combined.completedWaypoints = nnz(successMask);
combined.exitFlags = exitFlags;
combined.iterations = iterations;
combined.constraintViolationMax = violations;
combined.targetPoses = targetPoses;
combined.targetPositions = targetPositions;
combined.targetOrientations = [logA.targetOrientations, logB.targetOrientations, logC.targetOrientations];
combined.eePoses = eePoses;
combined.eePositions = eePositions;
combined.eeOrientations = eeOrientations;
combined.positionError = positionError;
combined.positionErrorNorm = positionErrorNorm;
combined.orientationErrorQuat = orientationErrorQuat;
combined.orientationErrorAngle = orientationErrorAngle;
combined.time = time;
end

function states = planStageBHybridAStarPath(qStart, baseIdx, targetPose, options, robot, eeName, floorDiscs)
%PLANSTAGEBHYBRIDASTARPATH Plan a base trajectory using Hybrid A*.
startBase = reshape(qStart(baseIdx), 1, []);
nJoints = numel(qStart);
armIdx = setdiff(1:nJoints, baseIdx);

goalBundle = gik9dof.createGikSolver(robot, "EndEffector", eeName);
lockJointBounds(goalBundle.constraints.joint, armIdx, qStart);
[qGoal, info] = goalBundle.solve(qStart, "TargetPose", targetPose);
exitFlag = nan;
if isstruct(info) && isfield(info, "ExitFlag")
    exitFlag = double(info.ExitFlag);
end
if ~(isfinite(exitFlag) && exitFlag > 0)
    error("gik9dof:runStagedTrajectory:HybridAStarGoalIK", ...
        "Failed to resolve Stage B base goal configuration (ExitFlag=%g).", exitFlag);
end
goalBase = reshape(qGoal(baseIdx), 1, []);

resolution = options.StageBHybridResolution;
safetyMargin = options.StageBHybridSafetyMargin;
occMap = buildStageBOccupancyMap(startBase, goalBase, floorDiscs, resolution, safetyMargin);

startState = [startBase(1:2), wrapToPi(startBase(3))];
goalState = [goalBase(1:2), wrapToPi(goalBase(3))];

startOcc = checkOccupancy(occMap, startState(1:2));
if startOcc < 0
    startOcc = 0;
end
if startOcc >= occMap.OccupiedThreshold
    error("gik9dof:runStagedTrajectory:HybridAStarStartOccupied", ...
        "Stage B start pose lies inside an occupied region.");
end
goalOcc = checkOccupancy(occMap, goalState(1:2));
if goalOcc < 0
    goalOcc = 0;
end
if goalOcc >= occMap.OccupiedThreshold
    error("gik9dof:runStagedTrajectory:HybridAStarGoalOccupied", ...
        "Stage B goal pose lies inside an occupied region.");
end

ss = stateSpaceSE2;
ss.StateBounds = [occMap.XWorldLimits; occMap.YWorldLimits; -pi pi];

validator = validatorOccupancyMap(ss);
validator.Map = occMap;
validator.ValidationDistance = resolution / 2;

planner = plannerHybridAStar(validator);
planner.MotionPrimitiveLength = options.StageBHybridMotionPrimitiveLength;
minTurningLowerBound = (2 * planner.MotionPrimitiveLength) / pi;
planner.MinTurningRadius = max(options.StageBHybridMinTurningRadius, minTurningLowerBound);

if isfield(options, "Verbose") && options.Verbose
    fprintf("[Stage B] Hybrid A* planning with resolution %.2f m and safety margin %.2f m...\n", resolution, safetyMargin);
end

pathObj = plan(planner, startState, goalState);
statesRaw = pathObj.States;
if isempty(statesRaw)
    error("gik9dof:runStagedTrajectory:HybridAStarEmpty", ...
        "Hybrid A* returned an empty path.");
end

rateHz = options.RateHz;
if isempty(rateHz) || ~isscalar(rateHz) || rateHz <= 0
    rateHz = 100;
end
maxLinearStep = options.StageBMaxLinearSpeed / rateHz;
maxYawStep = options.StageBMaxYawRate / rateHz;

states = densifyHybridStates(statesRaw, maxLinearStep, maxYawStep);
end

function map = buildStageBOccupancyMap(startBase, goalBase, floorDiscs, resolution, safetyMargin)
%BUILDSTAGEBOCCUPANCYMAP Construct occupancy map covering path region.
points = [startBase(1:2); goalBase(1:2)];
maxDiscRadius = 0;
if ~isempty(floorDiscs)
    centers = reshape([floorDiscs.Center], 2, []).';
    points = [points; centers]; %#ok<AGROW>
    discRadii = arrayfun(@(d) d.Radius + d.SafetyMargin + safetyMargin, floorDiscs);
    maxDiscRadius = max(discRadii);
else
    discRadii = []; %#ok<NASGU>
end

padding = max(1.0, 2*maxDiscRadius);
minXY = min(points, [], 1) - padding;
maxXY = max(points, [], 1) + padding;

width = max(maxXY(1) - minXY(1), resolution);
height = max(maxXY(2) - minXY(2), resolution);

cellsPerMeter = 1 / max(resolution, eps);
binMap = occupancyMap(width, height, cellsPerMeter);
binMap.GridLocationInWorld = minXY;
setOccupancy(binMap, zeros(binMap.GridSize));

if ~isempty(floorDiscs)
    for idx = 1:numel(floorDiscs)
        disc = floorDiscs(idx);
        inflatedRadius = disc.Radius + disc.SafetyMargin + safetyMargin;
        markDiscOnBinaryMap(binMap, disc.Center, inflatedRadius);
    end
end

map = binMap;
end

function markDiscOnBinaryMap(binMap, center, radius)
%MARKDISCONBINARYMAP Stamp a circular obstacle into the map.
if radius <= 0
    return
end

xRange = center(1) + [-radius, radius];
yRange = center(2) + [-radius, radius];

idxMinRowCol = world2grid(binMap, [xRange(1), yRange(1)]);
idxMaxRowCol = world2grid(binMap, [xRange(2), yRange(2)]);

if any(~isfinite([idxMinRowCol, idxMaxRowCol]))
    return
end

rowBounds = sort([idxMinRowCol(1), idxMaxRowCol(1)]);
colBounds = sort([idxMinRowCol(2), idxMaxRowCol(2)]);

rowLo = max(1, floor(rowBounds(1)));
rowHi = min(binMap.GridSize(1), ceil(rowBounds(2)));
colLo = max(1, floor(colBounds(1)));
colHi = min(binMap.GridSize(2), ceil(colBounds(2)));

if rowLo > rowHi || colLo > colHi
    return
end

rows = rowLo:rowHi;
cols = colLo:colHi;

if isempty(rows) || isempty(cols)
    return
end

[rowGrid, colGrid] = ndgrid(rows, cols);
worldPts = grid2world(binMap, [rowGrid(:), colGrid(:)]);
xWorld = worldPts(:,1);
yWorld = worldPts(:,2);

dx = xWorld - center(1);
dy = yWorld - center(2);
cellPadding = 0.5 / binMap.Resolution;
mask = (dx.^2 + dy.^2) <= (radius + cellPadding)^2;

if any(mask)
    occupiedPoints = [xWorld(mask), yWorld(mask)];
    setOccupancy(binMap, occupiedPoints, 1, "world");
end
end

function states = densifyHybridStates(statesIn, maxLinearStep, maxYawStep)
%DENSIFYHYBRIDSTATES Insert intermediate samples to respect rate limits.
if isempty(statesIn)
    states = statesIn;
    return
end

if ~isfinite(maxLinearStep) || maxLinearStep <= 0
    maxLinearStep = Inf;
end
if ~isfinite(maxYawStep) || maxYawStep <= 0
    maxYawStep = Inf;
end

states = zeros(0, 3);
currentState = [statesIn(1,1:2), wrapToPi(statesIn(1,3))];
states(end+1, :) = currentState; %#ok<AGROW>

for idx = 1:size(statesIn,1)-1
    nextState = [statesIn(idx+1,1:2), wrapToPi(statesIn(idx+1,3))];
    delta = nextState(1:2) - currentState(1:2);
    linearDist = hypot(delta(1), delta(2));
    yawDelta = wrapToPi(nextState(3) - currentState(3));
    stepsLinear = ceil(linearDist / maxLinearStep);
    stepsYaw = ceil(abs(yawDelta) / maxYawStep);
    numSegments = max([1, stepsLinear, stepsYaw]);

    for s = 1:numSegments-1
        tau = s / numSegments;
        interpXY = currentState(1:2) + delta * tau;
        interpYaw = wrapToPi(currentState(3) + yawDelta * tau);
        states(end+1, :) = [interpXY, interpYaw]; %#ok<AGROW>
    end

    states(end+1, :) = nextState; %#ok<AGROW>
    currentState = nextState;
end

% Remove potential duplicates introduced by zero-length segments.
if size(states,1) >= 2
    diffStates = diff(states);
    keepMask = [true; any(abs(diffStates) > 1e-9, 2)];
    states = states(keepMask, :);
end
end

function poses = statesToEndEffectorPoses(robot, qTemplate, baseIdx, states, eeName)
%STATESTOENDEFFECTORPOSES Convert base states to EE poses with frozen arm.
nStates = size(states, 1);
poses = repmat(eye(4), 1, 1, nStates);
q = qTemplate;
for k = 1:nStates
    q(baseIdx) = states(k, :).';
    poses(:,:,k) = getTransform(robot, q, eeName);
end
end
