function ramp = generateHolisticRamp(robot, solverBundle, qStart, targetPose, options)
%GENERATEHOLISTICRAMP Create a bounded-speed ramp towards the first pose.
%   ramp = gik9dof.generateHolisticRamp(robot, solverBundle, qStart, targetPose)
%   interpolates the planar base and arm joints from qStart towards the
%   configuration that realises targetPose while respecting the specified
%   linear, angular, and joint-rate limits.

arguments
    robot (1,1) rigidBodyTree
    solverBundle (1,1) struct
    qStart double
    targetPose (4,4) double
    options.SampleTime double = 0.1
    options.MaxLinearSpeed double = 1.5
    options.MaxYawRate double = 3.0
    options.MaxJointSpeed double = 1.0
    options.EndEffectorName (1,1) string = "left_gripper_link"
end

configTools = gik9dof.configurationTools(robot);
qStart = configTools.column(qStart);
nJoints = numel(qStart);

jointNames = string(configTools.templateJointNames());
idxX = find(jointNames == "joint_x", 1);
idxY = find(jointNames == "joint_y", 1);
idxTheta = find(jointNames == "joint_theta", 1);
baseIdx = [idxX, idxY, idxTheta];
armIdx = setdiff(1:nJoints, baseIdx);

[qGoal, solveInfo] = solverBundle.solve(qStart, "TargetPose", targetPose);
if ~isfield(solveInfo, "ExitFlag") || solveInfo.ExitFlag <= 0
    error("gik9dof:generateHolisticRamp:IKFailed", ...
        "Unable to compute ramp goal configuration (ExitFlag=%g).", solveInfo.ExitFlag);
end

startBase = qStart(baseIdx);
goalBase = qGoal(baseIdx);
linearDist = norm(goalBase(1:2) - startBase(1:2));
yawDist = wrapToPi(goalBase(3) - startBase(3));

dt = options.SampleTime;
stepsLinear = ceil(linearDist / max(options.MaxLinearSpeed * dt, eps));
stepsYaw = ceil(abs(yawDist) / max(options.MaxYawRate * dt, eps));

if isempty(armIdx)
    stepsArm = 0;
else
    deltaArm = qGoal(armIdx) - qStart(armIdx);
    stepsArm = ceil(max(abs(deltaArm)) / max(options.MaxJointSpeed * dt, eps));
end

nSteps = max([2, stepsLinear, stepsYaw, stepsArm]);
tau = linspace(0, 1, nSteps);

qRamp = repmat(qStart, 1, nSteps);
qRamp(baseIdx, :) = startBase + (goalBase - startBase) .* tau;
if ~isempty(armIdx)
    qRamp(armIdx, :) = qStart(armIdx) + (qGoal(armIdx) - qStart(armIdx)) .* tau;
end

eeName = char(options.EndEffectorName);
poses = repmat(eye(4), 1, 1, nSteps);
eePos = zeros(3, nSteps);
for k = 1:nSteps
    poses(:,:,k) = getTransform(robot, qRamp(:,k), eeName);
    eePos(:,k) = poses(1:3,4,k);
end

ramp = struct();
ramp.SampleTime = dt;
ramp.qTraj = qRamp;
ramp.basePose = qRamp(baseIdx, :).';
ramp.Poses = poses;
ramp.EndEffectorPositions = eePos;
ramp.GoalConfiguration = qGoal;
ramp.NumSteps = nSteps;
ramp.Time = (0:nSteps-1) * dt;
ramp.BaseIndices = baseIdx;
ramp.ArmIndices = armIdx;
ramp.TargetPose = targetPose;
ramp.TargetConfiguration = qGoal;

end
