function result = simulatePurePursuitExecution(pathStates, options)
%SIMULATEPUREPURSUITEXECUTION Propagate chassis pose using pure pursuit.
%   result = gik9dof.control.simulatePurePursuitExecution(pathStates, opts)
%   runs the pure pursuit follower against a reference path defined by
%   PATHSTATES (Nx3 [x y yaw]) and integrates (vx, wz) commands to recover an
%   executed base trajectory. OPTIONS supports name-value inputs:
%       SampleTime          - integration time step (s, default 0.1)
%       FollowerOptions     - struct passed to purePursuitFollower
%
%   The returned struct contains fields:
%       poses               - (N x 3) simulated poses
%       commands            - (N x 2) [vx, wz] per step
%       wheelSpeeds         - (N x 2) [vl, vr] per step
%       status              - array of status structs from the follower
%       follower            - follower object used (for inspection)
%
%   The simulation assumes vy = 0 and uses a standard differential-drive
%   integrator:
%       x_{k+1} = x_k + vx * cos(theta) * dt
%       y_{k+1} = y_k + vx * sin(theta) * dt
%       theta_{k+1} = theta_k + wz * dt

arguments
    pathStates double {mustBeReal}
    options.SampleTime (1,1) double {mustBePositive} = 0.1
    options.FollowerOptions = struct()
end

if isempty(pathStates)
    result = struct('poses', zeros(0,3), 'commands', zeros(0,2), ...
        'wheelSpeeds', zeros(0,2), 'status', struct([]), 'follower', []);
    return
end

if size(pathStates,2) < 3
    error('simulatePurePursuitExecution:PathFormat', ...
        'pathStates must contain [x y yaw] columns.');
end

follower = gik9dof.control.purePursuitFollower(pathStates, options.FollowerOptions);

sampleTime = options.SampleTime;
numSteps = size(pathStates, 1);
poses = zeros(numSteps, 3);
commands = zeros(numSteps, 2);
wheelSpeeds = zeros(numSteps, 2);
statusArray = repmat(struct('isFinished',false,'lookaheadIndex',NaN, ...
    'distanceToGoal',NaN,'lookaheadDistance',NaN,'wheelSpeeds',[0 0],'headingError',NaN), numSteps, 1);

poses(1,:) = pathStates(1,:);

for k = 1:numSteps
    pose = poses(k,:);
    [vx, wz, status] = follower.step(pose, sampleTime);
    commands(k,:) = [vx, wz];
    wheelSpeeds(k,:) = status.wheelSpeeds;
    statusArray(k) = status;

    if k < numSteps
        poses(k+1,:) = propagatePose(pose, vx, wz, sampleTime);
    end

    if status.isFinished
        poses(k+1:end,:) = repmat(poses(k,:), numSteps-k, 1);
        commands(k+1:end,:) = 0;
        wheelSpeeds(k+1:end,:) = 0;
        statusArray(k+1:end) = status;
        break
    end
end

result = struct('poses', poses, 'commands', commands, ...
    'wheelSpeeds', wheelSpeeds, 'status', statusArray, 'follower', follower);
end

function poseNext = propagatePose(pose, vx, wz, dt)
x = pose(1); y = pose(2); theta = pose(3);
poseNext = zeros(1,3);
poseNext(1) = x + vx * cos(theta) * dt;
poseNext(2) = y + vx * sin(theta) * dt;
poseNext(3) = wrapToPi(theta + wz * dt);
end
