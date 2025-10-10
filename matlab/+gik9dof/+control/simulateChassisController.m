function result = simulateChassisController(pathStates, options)
%SIMULATECHASSISCONTROLLER Integrate chassis using selectable control modes.
%   result = gik9dof.control.simulateChassisController(pathStates, opts)
%   runs one of the supported chassis controllers against PATHSTATES (Nx3
%   [x y yaw]) with an integration step of opts.SampleTime.  Supported modes:
%       0 - Legacy five-point differentiation (feedforward replay)
%       1 - Simple heading controller (P + feedforward yaw)
%       2 - Pure pursuit follower (delegates to simulatePurePursuitExecution)
%
%   Name-value options:
%       SampleTime          - integration step (s, default 0.1)
%       ControllerMode      - integer 0/1/2 selecting the controller (default 2)
%       FollowerOptions     - struct passed to the controller (requires
%                             ChassisParams field with track, limits, etc.)
%
%   The returned struct mirrors simulatePurePursuitExecution:
%       poses, commands, wheelSpeeds, status, follower ( [] for modes 0/1 )
%
%   See also gik9dof.control.simulatePurePursuitExecution.

arguments
    pathStates double
    options.SampleTime (1,1) double {mustBePositive} = 0.1
    options.ControllerMode (1,1) double {mustBeMember(options.ControllerMode, [0 1 2])} = 2
    options.FollowerOptions struct = struct()
end

if isempty(pathStates)
    result = emptyResult();
    result.controllerMode = options.ControllerMode;
    return
end

mode = options.ControllerMode;
sampleTime = options.SampleTime;
followers = options.FollowerOptions;

switch mode
    case 2
        result = gik9dof.control.simulatePurePursuitExecution(pathStates, ...
            'SampleTime', sampleTime, 'FollowerOptions', followers);
        result.controllerMode = mode;
        if ~isfield(result, 'follower') || isempty(result.follower)
            result.follower = [];
        end
        if ~isfield(result, 'status')
            result.status = struct([]);
        end
        return

    case 1
        result = simulateHeadingController(pathStates, sampleTime, followers);

    otherwise % mode == 0
        result = simulateDifferentiationController(pathStates, sampleTime, followers);
end

result.controllerMode = mode;
result.follower = [];
end

% -------------------------------------------------------------------------
function result = simulateHeadingController(pathStates, sampleTime, followers)
chassis = mustGetChassis(followers);

lookaheadBase = getFieldWithDefault(followers, 'LookaheadBase', ...
    getFieldWithDefault(chassis, 'lookahead_base', 0.6));
goalTolerance = getFieldWithDefault(followers, 'GoalTolerance', ...
    getFieldWithDefault(chassis, 'goal_tolerance', 0.1));
kHeading = getFieldWithDefault(followers, 'HeadingKp', ...
    getFieldWithDefault(chassis, 'heading_kp', 1.2));
kFeedforward = getFieldWithDefault(followers, 'FeedforwardGain', ...
    getFieldWithDefault(chassis, 'feedforward_gain', 0.9));

vxMax = getFieldWithDefault(chassis, 'vx_max', 1.5);
vxMin = getFieldWithDefault(chassis, 'vx_min', -0.4);
track = getFieldWithDefault(chassis, 'track', 0.573);
wheelSpeedMax = getFieldWithDefault(chassis, 'wheel_speed_max', 3.3);
wzMax = getFieldWithDefault(chassis, 'wz_max', 2.5);
reverseEnabled = getFieldWithDefault(chassis, 'reverse_enabled', false);

numPts = size(pathStates, 1);
maxSteps = max(numPts * 3, 500);

poses = zeros(maxSteps + 1, 3);
commands = zeros(maxSteps, 2);
wheelSpeeds = zeros(maxSteps, 2);
statusArray = repmat(statusTemplate("heading"), maxSteps, 1);

poses(1,:) = pathStates(1,:);
targetIdx = 2;
stepCount = 0;

for k = 1:maxSteps
    pose = poses(k,:);

    [targetIdx, targetPose, distanceToGoal] = selectTarget(pathStates, pose, ...
        targetIdx, lookaheadBase);

    if distanceToGoal <= goalTolerance && targetIdx >= numPts
        statusArray(k) = finishStatus(statusArray(k), targetIdx, distanceToGoal, [0 0]);
        poses(k+1,:) = pose;
        stepCount = k;
        break
    end

    dx = targetPose(1) - pose(1);
    dy = targetPose(2) - pose(2);
    desiredHeading = atan2(dy, dx);
    headingError = wrapToPi(desiredHeading - pose(3));
    yawFF = wrapToPi(targetPose(3) - pose(3)) / max(sampleTime, 1e-3);

    distanceAhead = hypot(dx, dy);
    vxDesired = distanceAhead / max(sampleTime, 1e-3);
    if reverseEnabled && cos(headingError) < cosd(120)
        vxDesired = -vxDesired;
    end

    Vx = clamp(vxDesired, vxMin, vxMax);
    Wz = kHeading * headingError + kFeedforward * yawFF;

    [Wz, yawCaps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, ...
        track, wheelSpeedMax, wzMax);
    Vx = clamp(Vx, vxMin, vxMax);
    wheelSpeeds(k,:) = computeWheelSpeeds(Vx, Wz, track);

    commands(k,:) = [Vx, Wz];
    poseNext = propagatePose(pose, Vx, Wz, sampleTime);
    poses(k+1,:) = poseNext;

    statusArray(k) = updateStatus(statusArray(k), targetIdx, ...
        distanceToGoal, lookaheadBase, headingError, Vx, Wz, wheelSpeeds(k,:), yawCaps);

    stepCount = k;
    if targetIdx >= numPts && distanceToGoal <= goalTolerance
        break
    end
end

poses = poses(1:stepCount+1, :);
commands = commands(1:stepCount, :);
wheelSpeeds = wheelSpeeds(1:stepCount, :);
statusArray = statusArray(1:stepCount);

result = struct('poses', poses, 'commands', commands, ...
    'wheelSpeeds', wheelSpeeds, 'status', statusArray);
end

% -------------------------------------------------------------------------
function result = simulateDifferentiationController(pathStates, sampleTime, followers)
chassis = mustGetChassis(followers);

vxMax = getFieldWithDefault(chassis, 'vx_max', 1.5);
vxMin = getFieldWithDefault(chassis, 'vx_min', -0.4);
track = getFieldWithDefault(chassis, 'track', 0.573);
wheelSpeedMax = getFieldWithDefault(chassis, 'wheel_speed_max', 3.3);
wzMax = getFieldWithDefault(chassis, 'wz_max', 2.5);

N = size(pathStates,1);
if N < 2
    result = emptyResult();
    return
end

poses = zeros(N,3);
poses(1,:) = pathStates(1,:);
commands = zeros(N-1, 2);
wheelSpeeds = zeros(N-1, 2);
statusArray = repmat(statusTemplate("legacy"), N-1, 1);

vxWorld = differentiateSeries(pathStates(:,1), sampleTime);
vyWorld = differentiateSeries(pathStates(:,2), sampleTime);
wSeries = differentiateSeries(pathStates(:,3), sampleTime);

for k = 1:N-1
    theta = poses(k,3);
    bodyVel = [cos(theta) sin(theta); -sin(theta) cos(theta)] * [vxWorld(k); vyWorld(k)];
    VxDesired = bodyVel(1);

    Vx = clamp(VxDesired, vxMin, vxMax);
    Wz = wSeries(k);
    [Wz, yawCaps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, track, wheelSpeedMax, wzMax);
    Vx = clamp(Vx, vxMin, vxMax);

    commands(k,:) = [Vx, Wz];
    wheelSpeeds(k,:) = computeWheelSpeeds(Vx, Wz, track);

    poses(k+1,:) = propagatePose(poses(k,:), Vx, Wz, sampleTime);
    statusArray(k) = updateStatus(statusArray(k), min(k+1, N), ...
        distanceToGoalEst(pathStates, poses(k,:)), 0, 0, Vx, Wz, ...
        wheelSpeeds(k,:), yawCaps);
end

result = struct('poses', poses, 'commands', commands, ...
    'wheelSpeeds', wheelSpeeds, 'status', statusArray);
end

% -------------------------------------------------------------------------
function chassis = mustGetChassis(followers)
if ~isfield(followers, 'ChassisParams') || isempty(followers.ChassisParams)
    error("gik9dof:simulateChassisController:MissingChassisParams", ...
        "FollowerOptions must include a ChassisParams struct.");
end
chassis = followers.ChassisParams;
end

function result = emptyResult()
result = struct('poses', zeros(0,3), 'commands', zeros(0,2), ...
    'wheelSpeeds', zeros(0,2), 'status', struct([]), 'follower', []);
end

function val = getFieldWithDefault(s, name, defaultValue)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = defaultValue;
end
end

function [targetIdx, targetPose, distanceToGoal] = selectTarget(pathStates, pose, idx, lookahead)
numPts = size(pathStates,1);
targetIdx = max(idx, 2);
for k = targetIdx:numPts
    d = hypot(pathStates(k,1) - pose(1), pathStates(k,2) - pose(2));
    targetIdx = k;
    if d >= lookahead || k == numPts
        break
    end
end
targetPose = pathStates(targetIdx,:);
distanceToGoal = hypot(pathStates(end,1) - pose(1), pathStates(end,2) - pose(2));
end

function val = clamp(x, minVal, maxVal)
val = min(max(x, minVal), maxVal);
end

function wheelSpeeds = computeWheelSpeeds(Vx, Wz, track)
wheelSpeeds = [Vx - 0.5 * track * Wz, Vx + 0.5 * track * Wz];
end

function status = statusTemplate(mode)
status = struct('isFinished', false, ...
    'nearestIndex', 1, ...
    'targetIndex', 1, ...
    'distanceToGoal', Inf, ...
    'lookaheadDistance', 0, ...
    'curvature', 0, ...
    'crossTrackError', 0, ...
    'headingError', 0, ...
    'vxCommand', 0, ...
    'wzCommand', 0, ...
    'acceleration', 0, ...
    'wheelSpeeds', [0 0], ...
    'controllerMode', string(mode), ...
    'warnings', strings(1,0));
end

function status = finishStatus(status, idx, distanceToGoal, wheelSpeeds)
status.isFinished = true;
status.targetIndex = idx;
status.nearestIndex = idx;
status.distanceToGoal = distanceToGoal;
status.wheelSpeeds = wheelSpeeds;
end

function status = updateStatus(status, targetIdx, distanceToGoal, lookahead, ...
    headingError, Vx, Wz, wheelSpeeds, yawCaps)
status.isFinished = false;
status.targetIndex = targetIdx;
status.nearestIndex = targetIdx;
status.distanceToGoal = distanceToGoal;
status.lookaheadDistance = lookahead;
status.headingError = headingError;
status.vxCommand = Vx;
status.wzCommand = Wz;
status.wheelSpeeds = wheelSpeeds;
status.warnings = strings(1,0);
status.controllerMode = string(status.controllerMode);
if nargin >= 9 && isstruct(yawCaps)
    status.warnings = [status.warnings, string(sprintf("yawClamp=%.3f", yawCaps.applied))];
end
end

function df = differentiateSeries(series, dt)
N = numel(series);
df = zeros(N,1);
if N < 2
    return
end
dt = max(dt, 1e-3);
if N < 5
    for i = 1:N
        if i == 1
            df(i) = (series(min(N, i+1)) - series(i)) / dt;
        elseif i == N
            df(i) = (series(i) - series(max(1, i-1))) / dt;
        else
            df(i) = (series(i+1) - series(i-1)) / (2*dt);
        end
    end
    return
end

for i = 1:N
    if i >= 3 && i <= N-2
        df(i) = (-series(i+2) + 8*series(i+1) - 8*series(i-1) + series(i-2)) / (12*dt);
    elseif i == 2 || i == N-1
        df(i) = (series(i+1) - series(i-1)) / (2*dt);
    elseif i == 1
        df(i) = (-3*series(i) + 4*series(i+1) - series(i+2)) / (2*dt);
    else % i == N
        df(i) = (3*series(i) - 4*series(i-1) + series(i-2)) / (2*dt);
    end
end
end

function dist = distanceToGoalEst(pathStates, pose)
dist = hypot(pathStates(end,1) - pose(1), pathStates(end,2) - pose(2));
end

function poseNext = propagatePose(pose, vx, wz, dt)
poseNext = zeros(1,3);
poseNext(1) = pose(1) + vx * cos(pose(3)) * dt;
poseNext(2) = pose(2) + vx * sin(pose(3)) * dt;
poseNext(3) = wrapToPi(pose(3) + wz * dt);
end
