function [qOut, qCount, stageCounts, baseStates, baseCount, baseCommands, baseCmdCount, sampleTime] = stagedFollowTrajectory( ...
    qInit, poses, distanceLower, distanceWeight, params, floorCenters, floorRadii, floorMargins, floorCount)
%STAGEDFOLLOWTRAJECTORY Execute staged pipeline (Stages A/B/C) for codegen.
%#codegen

arguments
    qInit (9,1) double
    poses (:,:,:) double
    distanceLower (1,1) double
    distanceWeight (1,1) double
    params struct
    floorCenters (:,2) double
    floorRadii (:,1) double
    floorMargins (:,1) double
    floorCount (1,1) double
end

MAX_STAGE_A = int32(80);
MAX_STAGE_B = int32(200);
MAX_STAGE_C = int32(200);
MAX_TOTAL = int32(MAX_STAGE_A + MAX_STAGE_B + MAX_STAGE_C);

qOut = zeros(9, MAX_TOTAL);
baseStates = zeros(3, MAX_STAGE_B);
baseCommands = zeros(3, MAX_STAGE_B);
qCount = int32(0);
baseCount = int32(0);
baseCmdCount = int32(0);
sampleTime = 0;
stageCounts = zeros(3,1,'int32');

baseIdx = coder.const(int32([1; 2; 3]));
armIdx = coder.const(int32([4; 5; 6; 7; 8; 9]));

persistent robot eeName
if isempty(robot)
    robot = gik9dof.codegen.loadRobotForCodegen();
    eeName = 'left_gripper_link';
end

numPoses = int32(size(poses, 3));
qCurrent = qInit;

rateHz = getFieldOr(params, 'rateHz', 100.0);
if ~(isfinite(rateHz) && rateHz > 0)
    rateHz = 100.0;
end
sampleTime = 1.0 / rateHz;
maxLinearSpeed = getFieldOr(params, 'maxLinearSpeed', 1.5);
if ~(isfinite(maxLinearSpeed) && maxLinearSpeed > 0)
    maxLinearSpeed = 1.5;
end
maxYawRate = getFieldOr(params, 'maxYawRate', 3.0);
if ~(isfinite(maxYawRate) && maxYawRate > 0)
    maxYawRate = 3.0;
end
maxPathLengthParam = getFieldOr(params, 'maxPathLength', 25.0);
if ~(isfinite(maxPathLengthParam) && maxPathLengthParam > 0)
    maxPathLengthParam = 25.0;
end
ppLookahead = getFieldOr(params, 'ppLookahead', 0.6);
ppDesiredSpeed = getFieldOr(params, 'ppDesiredSpeed', maxLinearSpeed);
ppGoalRadius = getFieldOr(params, 'ppGoalRadius', 0.15);
ppCloseHeading = getFieldOr(params, 'ppCloseOnHeading', 0);
ppReverseAllowed = getFieldOr(params, 'ppReverseAllowed', 0);
hybridResolution = getFieldOr(params, 'hybridResolution', 0.1);
hybridSafetyMargin = getFieldOr(params, 'hybridSafetyMargin', 0.15);
hybridMinTurningRadius = getFieldOr(params, 'hybridMinTurningRadius', 0.5);
if ~(isfinite(hybridMinTurningRadius) && hybridMinTurningRadius > 0.05)
    hybridMinTurningRadius = 0.5;
end
hybridMotionPrimitiveLength = getFieldOr(params, 'hybridMotionPrimitiveLength', 0.5);
if ~(isfinite(hybridMotionPrimitiveLength) && hybridMotionPrimitiveLength > 0.05)
    hybridMotionPrimitiveLength = 0.5;
end
hybridHeadingBins = getFieldOr(params, 'hybridHeadingBins', 48.0);
hybridRotationStep = getFieldOr(params, 'hybridRotationStep', 0.35);
plannerMaxWaypoints = double(MAX_STAGE_B);
if numPoses == 0
    qOut(:,1) = qInit;
    qCount = int32(1);
    return;
end

TGoal = poses(:,:,1);
TStart = getEndEffectorTransform(robot, qCurrent, eeName);

%% Stage A: arm ramp with base locked
stageASteps = int32(getFieldOr(params, 'stageA_samples', 50));
if stageASteps < 0
    stageASteps = int32(0);
elseif stageASteps > MAX_STAGE_A
    stageASteps = MAX_STAGE_A;
end

if stageASteps <= 1
    stageCounts(1) = stageASteps;
else
    lockMask = false(9,1);
    lockMask(1:3) = true;
    for k = 0:(double(stageASteps) - 1)
        alpha = 0.0;
        if stageASteps > 1
        denom = double(stageASteps) - 1.0;
        alpha = double(k) / denom;
        end
        targetPose = interpolateStagePose(TStart, TGoal, alpha, 'arm');
        qCurrent = gik9dof.codegen.solveGIKStepWithLock(qCurrent, targetPose, distanceLower, distanceWeight, lockMask);
        qCount = qCount + int32(1);
        qOut(:, double(qCount)) = qCurrent;
    end
    stageCounts(1) = stageASteps;
end

%% Stage B: plan base path with arm frozen
lockArmMask = false(9,1);
lockArmMask(4:9) = true;
qGoalBase = gik9dof.codegen.solveGIKStepWithLock(qCurrent, TGoal, distanceLower, distanceWeight, lockArmMask);
startBase = qCurrent(baseIdx);
goalBase = qGoalBase(baseIdx);

plannerParams = struct('resolution', hybridResolution, ...
    'safetyMargin', hybridSafetyMargin, ...
    'maxPathLength', maxPathLengthParam, ...
    'maxWaypoints', plannerMaxWaypoints, ...
    'minTurningRadius', hybridMinTurningRadius, ...
    'motionPrimitiveLength', hybridMotionPrimitiveLength, ...
    'headingBins', hybridHeadingBins, ...
    'rotationStep', hybridRotationStep, ...
    'sampleTime', sampleTime, ...
    'rateHz', rateHz, ...
    'maxLinearSpeed', maxLinearSpeed, ...
    'maxYawRate', maxYawRate, ...
    'ppLookahead', ppLookahead, ...
    'ppDesiredSpeed', ppDesiredSpeed, ...
    'ppGoalRadius', ppGoalRadius, ...
    'ppCloseOnHeading', ppCloseHeading, ...
    'ppReverseAllowed', ppReverseAllowed);

[baseStates, baseCount, baseCommands, baseCmdCount] = gik9dof.codegen.stageBPlanPath(startBase, goalBase, floorCenters, floorRadii, floorMargins, floorCount, plannerParams);

if baseCount < 2
    baseStates(:,1) = startBase;
    baseStates(:,2) = goalBase;
    baseCount = int32(2);
    baseCmdCount = int32(0);
end

if baseCount >= 2 && baseCmdCount > 0
    if baseCmdCount >= 1
        baseCommands(1,1) = 0;
        baseCommands(2,1) = 0;
        baseCommands(3,1) = 0;
    end
    baseCmdCount = min(baseCmdCount, baseCount);
else
    baseCommands = fallbackStageBCommands(baseStates, baseCount, sampleTime, maxLinearSpeed, maxYawRate);
    baseCmdCount = baseCount;
end

for idx = 2:double(baseCount)
    qCurrent(baseIdx) = baseStates(:, idx);
    qCount = qCount + int32(1);
    qOut(:, double(qCount)) = qCurrent;
end

stageCounts(2) = max(int32(0), baseCount - int32(1));

%% Stage C: full-body tracking for remaining poses
if numPoses > 1
    lockMask = false(9,1);
    stageCsteps = int32(0);
    for poseIdx = 2:double(numPoses)
        targetPose = poses(:,:,poseIdx);
        qCurrent = gik9dof.codegen.solveGIKStepWithLock(qCurrent, targetPose, distanceLower, distanceWeight, lockMask);
        qCount = qCount + int32(1);
        qOut(:, double(qCount)) = qCurrent;
        stageCsteps = stageCsteps + int32(1);
    end
    stageCounts(3) = stageCsteps;
else
    stageCounts(3) = int32(0);
end

% Ensure at least one sample stored
if qCount == 0
    qCount = int32(1);
    qOut(:,1) = qCurrent;
end

end

function T = getEndEffectorTransform(robot, q, eeName)
T = getTransform(robot, q, eeName);
end

function T = interpolateStagePose(Tstart, Tend, alpha, mode)
posStart = [Tstart(1,4), Tstart(2,4), Tstart(3,4)];
posGoal = [Tend(1,4), Tend(2,4), Tend(3,4)];
quatStart = rotmToQuat(Tstart(1:3,1:3));
quatGoal = rotmToQuat(Tend(1:3,1:3));

switch mode
    case 'arm'
        pos = [posStart(1), posStart(2), posStart(3)*(1-alpha) + posGoal(3)*alpha];
        quat = quatSlerp(quatStart, quatGoal, alpha);
    case 'base'
        pos = [posStart(1)*(1-alpha) + posGoal(1)*alpha, ...
               posStart(2)*(1-alpha) + posGoal(2)*alpha, ...
               posGoal(3)];
        quat = quatGoal;
    otherwise
        pos = posGoal;
        quat = quatGoal;
end

T = eye(4);
T(1:3,1:3) = quatToRotm(quat);
T(1:3,4) = pos(:);
end

function q = rotmToQuat(R)
tr = trace(R);
if tr > 0
    S = sqrt(tr + 1.0) * 2;
    w = 0.25 * S;
    x = (R(3,2) - R(2,3)) / S;
    y = (R(1,3) - R(3,1)) / S;
    z = (R(2,1) - R(1,2)) / S;
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    w = (R(3,2) - R(2,3)) / S;
    x = 0.25 * S;
    y = (R(1,2) + R(2,1)) / S;
    z = (R(1,3) + R(3,1)) / S;
elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    w = (R(1,3) - R(3,1)) / S;
    x = (R(1,2) + R(2,1)) / S;
    y = 0.25 * S;
    z = (R(2,3) + R(3,2)) / S;
else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    w = (R(2,1) - R(1,2)) / S;
    x = (R(1,3) + R(3,1)) / S;
    y = (R(2,3) + R(3,2)) / S;
    z = 0.25 * S;
end
q = quatNormalize([w, x, y, z]);
end

function R = quatToRotm(q)
q = quatNormalize(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = zeros(3,3);
R(1,1) = 1 - 2*(y*y + z*z);
R(1,2) = 2*(x*y - z*w);
R(1,3) = 2*(x*z + y*w);
R(2,1) = 2*(x*y + z*w);
R(2,2) = 1 - 2*(x*x + z*z);
R(2,3) = 2*(y*z - x*w);
R(3,1) = 2*(x*z - y*w);
R(3,2) = 2*(y*z + x*w);
R(3,3) = 1 - 2*(x*x + y*y);
end

function q = quatSlerp(q0, q1, t)
q0 = quatNormalize(q0);
q1 = quatNormalize(q1);
if dot(q0, q1) < 0
    q1 = -q1;
end
cosTheta = dot(q0, q1);
if cosTheta > 0.9995
    q = quatNormalize((1-t)*q0 + t*q1);
    return;
end
angle = acos(max(min(cosTheta, 1), -1));
q = (sin((1-t)*angle)/sin(angle))*q0 + (sin(t*angle)/sin(angle))*q1;
q = quatNormalize(q);
end

function q = quatNormalize(q)
normVal = sqrt(sum(q.^2));
if normVal > 0
    q = q ./ normVal;
else
    q = [1,0,0,0];
end
end

function val = getFieldOr(s, name, defaultValue)
if isfield(s, name)
    val = s.(name);
else
    val = defaultValue;
end
end









function commands = fallbackStageBCommands(states, count, sampleTime, maxLinearSpeed, maxYawRate)
commands = zeros(3, size(states,2));
if count <= 0
    return;
end
commands(:,1) = [0; 0; 0];
for idx = 2:double(count)
    dt = sampleTime;
    dx = states(1, idx) - states(1, idx-1);
    dy = states(2, idx) - states(2, idx-1);
    yawPrev = states(3, idx-1);
    vxWorld = dx / dt;
    vyWorld = dy / dt;
    vxBody = cos(yawPrev) * vxWorld + sin(yawPrev) * vyWorld;
    yawNext = states(3, idx);
    yawDelta = atan2(sin(yawNext - yawPrev), cos(yawNext - yawPrev));
    wCmd = yawDelta / dt;
    vxBody = max(-maxLinearSpeed, min(maxLinearSpeed, vxBody));
    wCmd = max(-maxYawRate, min(maxYawRate, wCmd));
    commands(1, idx) = vxBody;
    commands(2, idx) = wCmd;
    commands(3, idx) = double(idx - 1) * sampleTime;
end
end



