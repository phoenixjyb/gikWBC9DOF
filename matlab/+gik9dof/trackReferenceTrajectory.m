function log = trackReferenceTrajectory(options)
%TRACKREFERENCETRAJECTORY Run GIK control on the reference JSON trajectory.
%   log = GIK9DOF.TRACKREFERENCETRAJECTORY() loads the bundled
%   1_pull_world_scaled.json end-effector trajectory, sets the mobile base
%   home configuration to (-2, -2, 0), and executes the GIK control loop
%   defined by gik9dof.runTrajectoryControl. The returned log records the
%   commanded joint configurations and solver diagnostics.
%
%   log = GIK9DOF.TRACKREFERENCETRAJECTORY(options) accepts name-value
%   options:
%       JsonPath            - Path to a trajectory JSON file. Defaults to
%                             the project reference trajectory.
%       BaseHome            - 1x3 vector [x y theta] specifying the planar
%                             base joint positions used for the initial
%                             configuration (default [-2 -2 0]).
%       RateHz              - Control loop frequency (default 100).
%       Verbose             - Logical flag enabling per-step logging
%                             (default true).
%       CommandFcn          - Optional per-iteration callback of the form
%                             @(q, info, idx).
%       EnableAiming        - Logical flag, when true reuses the reference
%                             positions as aim targets (default false).
%
%   Example:
%       log = gik9dof.trackReferenceTrajectory();
%       plot(log.timestamps, log.qTraj(1,1:end-1));
%
%   See also gik9dof.runTrajectoryControl, jsondecode.

arguments
    options.JsonPath (1,1) string = "1_pull_world_scaled.json"
    options.BaseHome (1,3) double = [-2 -2 0]
    options.RateHz (1,1) double {mustBePositive} = 100
    options.Verbose (1,1) logical = true
    options.CommandFcn = []
    options.EnableAiming (1,1) logical = false
    options.DistanceWeight (1,1) double = 0.5
    options.DistanceMargin (1,1) double = 0.1
    options.FloorDiscs (1,:) struct = struct([])
    options.BaseDistanceBody (1,1) string = "abstract_chassis_link"
    options.Mode (1,1) string {mustBeMember(options.Mode, ["holistic","staged"])} = "holistic"
    options.UseHolisticRamp (1,1) logical = false
    options.RampMaxLinearSpeed (1,1) double = 1.5
    options.RampMaxYawRate (1,1) double = 3.0
    options.RampMaxJointSpeed (1,1) double = 1.0
    options.UseStageBHybridAStar (1,1) logical = false
    options.StageBHybridResolution (1,1) double = 0.1
    options.StageBHybridSafetyMargin (1,1) double = 0.15
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.5
    options.EnvironmentConfig (1,1) struct = gik9dof.environmentConfig()
end

% Resolve assets and instantiate robot.
jsonPath = gik9dof.internal.resolvePath(options.JsonPath);
robot = gik9dof.createRobotModel("Validate", true);

% Build initial configuration with specified base pose.
configTools = gik9dof.configurationTools(robot);
homeStruct = configTools.struct(configTools.homeConfig());

envConfig = options.EnvironmentConfig;
if isfield(envConfig, "BaseHome") && numel(envConfig.BaseHome) == 3
    baseHome = envConfig.BaseHome;
else
    baseHome = options.BaseHome;
end

baseMap = struct('joint_x', baseHome(1), ...
                 'joint_y', baseHome(2), ...
                 'joint_theta', baseHome(3));

for idx = 1:numel(homeStruct)
    name = homeStruct(idx).JointName;
    if isfield(baseMap, name)
        homeStruct(idx).JointPosition = baseMap.(name);
    end
end

q0 = configTools.column(homeStruct);

jointNames = string(configTools.templateJointNames());
idxX = find(jointNames == "joint_x", 1);
idxY = find(jointNames == "joint_y", 1);
idxTheta = find(jointNames == "joint_theta", 1);
baseIdx = [idxX, idxY, idxTheta];
armIdx = setdiff(1:numel(q0), baseIdx);

if options.DistanceMargin < 0
    error("gik9dof:trackReferenceTrajectory:InvalidDistanceMargin", ...
        "DistanceMargin must be non-negative.");
end

envFloorDiscs = struct([]);
if isfield(envConfig, "FloorDiscs")
    envFloorDiscs = envConfig.FloorDiscs;
end

floorDiscSource = options.FloorDiscs;
if isempty(floorDiscSource)
    floorDiscSource = envFloorDiscs;
end

distanceMargin = options.DistanceMargin;
if isfield(envConfig, "DistanceMargin")
    distanceMargin = envConfig.DistanceMargin;
end

distanceWeight = options.DistanceWeight;
if isfield(envConfig, "DistanceWeight")
    distanceWeight = envConfig.DistanceWeight;
end

floorDiscInfo = struct('Name', {}, 'Radius', {}, 'SafetyMargin', {});
distanceSpecs = struct([]);

if ~isempty(floorDiscSource)
    floorDiscInfo = gik9dof.addFloorDiscs(robot, floorDiscSource);
    numDiscs = numel(floorDiscInfo);
    distanceSpecs = repmat(struct('Body', options.BaseDistanceBody, ...
        'ReferenceBody', "", 'Bounds', [0 0], 'Weight', distanceWeight), numDiscs, 1);
    for k = 1:numDiscs
        discName = floorDiscInfo(k).Name;
        lowerBound = floorDiscInfo(k).Radius + floorDiscInfo(k).SafetyMargin + distanceMargin;
        distanceSpecs(k).ReferenceBody = discName;
        distanceSpecs(k).Bounds = [lowerBound, Inf];
    end
end

% Attach available collision meshes for downstream checks and visualisation.
colTools = gik9dof.collisionTools(robot, 'MeshDirectory', 'meshes');
colTools.apply();

% Instantiate solver bundle.
bundle = gik9dof.createGikSolver(robot, ...
    "EnableAiming", options.EnableAiming, ...
    "DistanceSpecs", distanceSpecs, ...
    "DistanceWeight", distanceWeight);

% Prepare trajectory struct from JSON file.
trajStruct = loadJsonTrajectory(jsonPath);
trajStruct.EndEffectorName = "left_gripper_link";

if options.EnableAiming
    trajStruct.AimTargets = trajStruct.EndEffectorPositions;
end

rampInfo = struct();
if options.Mode == "holistic" && options.UseHolisticRamp
    rampInfo = gik9dof.generateHolisticRamp(robot, bundle, q0, trajStruct.Poses(:,:,1), ...
        'SampleTime', 1/options.RateHz, ...
        'MaxLinearSpeed', options.RampMaxLinearSpeed, ...
        'MaxYawRate', options.RampMaxYawRate, ...
        'MaxJointSpeed', options.RampMaxJointSpeed, ...
        'EndEffectorName', trajStruct.EndEffectorName);
    trajStruct = prependRampTrajectory(trajStruct, rampInfo, options.EnableAiming);
end

% Run control according to selected mode.
commandFcn = options.CommandFcn;
if ~isempty(commandFcn) && ~isa(commandFcn, "function_handle")
    error("gik9dof:trackReferenceTrajectory:InvalidCommandFcn", ...
        "CommandFcn must be a function handle or empty.");
end
stageTurningRadius = options.StageBHybridMinTurningRadius;
if stageTurningRadius <= 0
    stageTurningRadius = options.RampMaxLinearSpeed / max(options.RampMaxYawRate, eps);
end

switch options.Mode
    case "holistic"
        velLimits = struct('BaseIndices', baseIdx, ...
            'ArmIndices', armIdx, ...
            'MaxLinearSpeed', options.RampMaxLinearSpeed, ...
            'MaxYawRate', options.RampMaxYawRate, ...
            'MaxJointSpeed', options.RampMaxJointSpeed);
        log = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
            "InitialConfiguration", q0, ...
            "RateHz", options.RateHz, ...
            "VelocityLimits", velLimits, ...
            "CommandFcn", commandFcn, ...
            "Verbose", options.Verbose);
        log.mode = "holistic";
    case "staged"
        pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
            'InitialConfiguration', q0, ...
            'ConfigTools', configTools, ...
            'DistanceSpecs', distanceSpecs, ...
            'DistanceWeight', distanceWeight, ...
            'RateHz', options.RateHz, ...
            'Verbose', options.Verbose, ...
            'CommandFcn', commandFcn, ...
            'FloorDiscs', floorDiscInfo, ...
            'UseStageBHybridAStar', options.UseStageBHybridAStar, ...
            'StageBHybridResolution', options.StageBHybridResolution, ...
            'StageBHybridSafetyMargin', options.StageBHybridSafetyMargin, ...
            'StageBHybridMinTurningRadius', stageTurningRadius, ...
            'StageBHybridMotionPrimitiveLength', options.StageBHybridMotionPrimitiveLength, ...
            'StageBMaxLinearSpeed', options.RampMaxLinearSpeed, ...
            'StageBMaxYawRate', options.RampMaxYawRate, ...
            'StageBMaxJointSpeed', options.RampMaxJointSpeed, ...
            'EnvironmentConfig', envConfig);
        log = pipeline;
    otherwise
        error("gik9dof:trackReferenceTrajectory:UnknownMode", options.Mode);
end

log.floorDiscs = floorDiscInfo;
log.distanceSpecs = distanceSpecs;
log.environment = envConfig;
if options.Mode == "holistic"
    log.velocityLimits = struct('MaxLinearSpeed', options.RampMaxLinearSpeed, ...
        'MaxYawRate', options.RampMaxYawRate, ...
        'MaxJointSpeed', options.RampMaxJointSpeed, ...
        'SampleTime', 1/options.RateHz);
    if options.UseHolisticRamp
        log.ramp = rampInfo;
        log.rampSamples = rampInfo.NumSteps;
    end
end

end

function trajOut = prependRampTrajectory(trajIn, rampInfo, enableAiming)
%PREPENDRAMPTRAJECTORY Concatenate ramp poses ahead of trajectory.
trajOut = trajIn;

% Drop the original first pose to avoid duplication.
rampPoses = rampInfo.Poses;
rampEE = rampInfo.EndEffectorPositions;

trajOut.Poses = cat(3, rampPoses, trajIn.Poses(:,:,2:end));

if isfield(trajIn, 'EndEffectorPositions') && ~isempty(trajIn.EndEffectorPositions)
    trajOut.EndEffectorPositions = [rampEE, trajIn.EndEffectorPositions(:,2:end)];
end

if enableAiming && isfield(trajIn, 'AimTargets') && ~isempty(trajIn.AimTargets)
    trajOut.AimTargets = [rampEE, trajIn.AimTargets(:,2:end)];
end

end

function traj = loadJsonTrajectory(jsonPath)
%LOADJSONTRAJECTORY Convert reference JSON into runTrajectoryControl struct.
raw = jsondecode(fileread(jsonPath));
if ~isfield(raw, "poses")
    error("gik9dof:trackReferenceTrajectory:MissingField", ...
        "JSON file %s does not contain a 'poses' field.", jsonPath);
end

numWaypoints = numel(raw.poses);
poses = repmat(eye(4), 1, 1, numWaypoints);
posXYZ = zeros(3, numWaypoints);

for k = 1:numWaypoints
    entry = raw.poses(k);
    if ~isfield(entry, "position") || ~isfield(entry, "orientation")
        error("gik9dof:trackReferenceTrajectory:PoseFieldMissing", ...
            "Waypoint %d is missing position/orientation fields.", k);
    end

    position = reshape(entry.position, [], 1);
    quatXYZW = reshape(entry.orientation, 1, []);
    if numel(quatXYZW) ~= 4
        error("gik9dof:trackReferenceTrajectory:InvalidQuaternion", ...
            "Waypoint %d orientation does not contain four elements.", k);
    end

    % Convert from [x y z w] into MATLAB's [w x y z] convention.
    quatWXYZ = [quatXYZW(4), quatXYZW(1:3)];
    quatWXYZ = quatWXYZ ./ norm(quatWXYZ);

    T = quat2tform(quatWXYZ);
    T(1:3,4) = position;

    poses(:,:,k) = T;
    posXYZ(:,k) = position;
end

traj.Poses = poses;
traj.EndEffectorPositions = posXYZ;
end
