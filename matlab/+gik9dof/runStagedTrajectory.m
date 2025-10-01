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
    'Verbose', options.Verbose);

qA_end = logA.qTraj(:, end);

%% Stage B: base-only alignment of x-y while arm frozen
stageBStart = getTransform(robot, qA_end, trajStruct.EndEffectorName);
stageBPoses = generateStagePoses(stageBStart, T1, NB, 'base');
trajB = struct('Poses', stageBPoses);
trajB.EndEffectorName = trajStruct.EndEffectorName;

bundleB = gik9dof.createGikSolver(robot);
lockJointBounds(bundleB.constraints.joint, armIdx, qA_end);

logB = gik9dof.runTrajectoryControl(bundleB, trajB, ...
    'InitialConfiguration', qA_end, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose);

qB_end = logB.qTraj(:, end);

%% Stage C: full-body tracking of remaining reference
bundleC = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs);

trajC = sliceTrajectoryStruct(trajStruct, 2);
logC = struct();
if ~isempty(trajC.Poses)
    logC = gik9dof.runTrajectoryControl(bundleC, trajC, ...
        'InitialConfiguration', qB_end, ...
        'RateHz', options.RateHz, ...
        'CommandFcn', options.CommandFcn, ...
        'Verbose', options.Verbose);
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
