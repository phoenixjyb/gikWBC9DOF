function plotStruct = generateLogPlots(log, prefix)
%GENERATELOGPLOTS Produce standard plots for arm and chassis states/velocities.
%   plotStruct = gik9dof.generateLogPlots(log, prefix) saves two PNG figures
%   prefixed with the provided path (string/char without extension). The first
%   shows the first six arm joints, the second plots chassis x/y/theta and the
%   derived base velocity (Vx) and yaw rate.
%
%   Returns a struct with fields:
%       armJointFigure  - path to arm joint figure (if generated)
%       chassisFigure   - path to chassis figure (if generated)
%       data            - struct containing time vectors and signals
%
%   The function assumes log.qTraj uses the configuration ordering produced by
%   gik9dof.configurationTools (joint_x, joint_y, joint_theta, ...).

arguments
    log (1,1) struct
    prefix {mustBeText} = ""
end

plotStruct = struct();
plotStruct.data = struct();

if ~isfield(log, 'qTraj') || isempty(log.qTraj)
    return
end

prefix = string(prefix);
robot = gik9dof.createRobotModel();
configTools = gik9dof.configurationTools(robot);
jointNames = string(configTools.templateJointNames());

qTraj = log.qTraj;
numSamples = size(qTraj, 2);

if isfield(log, 'timestamps') && ~isempty(log.timestamps)
    timeVec = [0, log.timestamps];
else
    timeVec = linspace(0, (numSamples-1) / max(log.rateHz, 1), numSamples);
end

baseIdx = zeros(1,3);
baseIdx(1) = find(jointNames == "joint_x", 1);
baseIdx(2) = find(jointNames == "joint_y", 1);
baseIdx(3) = find(jointNames == "joint_theta", 1);
baseIdx(baseIdx==0) = [];

armIdx = setdiff(1:numel(jointNames), baseIdx);
if isempty(armIdx)
    armIdx = 1:min(6, size(qTraj,1));
else
    armIdx = armIdx(1:min(6, numel(armIdx)));
end

if ~isempty(armIdx)
    figArm = figure('Visible', 'off');
    plot(timeVec, qTraj(armIdx,:)');
    xlabel('Time (s)'); ylabel('Joint Position (rad)');
    legend(arrayfun(@(k) char(jointNames(k)), armIdx, 'UniformOutput', false), ...
        'Interpreter','none', 'Location', 'best');
    title('Arm Joint Positions'); grid on;
    armPath = char(prefix + "_arm_joints.png");
    saveas(figArm, armPath);
    close(figArm);
    plotStruct.armJointFigure = armPath;
    plotStruct.data.armTime = timeVec;
    plotStruct.data.armPositions = qTraj(armIdx,:);
end

if numel(baseIdx) == 3
    x = qTraj(baseIdx(1),:);
    y = qTraj(baseIdx(2),:);
    theta = qTraj(baseIdx(3),:);
    vx = gradient(x, timeVec);
    vy = gradient(y, timeVec);
    VxBody = hypot(vx, vy);
    wz = gradient(theta, timeVec);

    figBase = figure('Visible', 'off');
    subplot(2,1,1);
    plot(timeVec, [x; y; theta]');
    xlabel('Time (s)'); ylabel('Pose (m / rad)'); grid on;
    legend({'x','y','theta'}, 'Location', 'best');
    title('Chassis PPR States');

    subplot(2,1,2);
    plot(timeVec, VxBody, 'LineWidth', 1.2); hold on;
    plot(timeVec, wz, 'LineWidth', 1.2);
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Velocity');
    legend({'Vx','Wz'}, 'Location', 'best');
    title('Chassis Velocity Estimates');

    chassisPath = char(prefix + "_chassis.png");
    saveas(figBase, chassisPath);
    close(figBase);
    plotStruct.chassisFigure = chassisPath;
    plotStruct.data.chassisTime = timeVec;
    plotStruct.data.x = x;
    plotStruct.data.y = y;
    plotStruct.data.theta = theta;
    plotStruct.data.Vx = VxBody;
    plotStruct.data.Wz = wz;
end
end

function mustBeText(value)
if ~(isstring(value) || ischar(value))
    error('Prefix must be text.');
end
end
