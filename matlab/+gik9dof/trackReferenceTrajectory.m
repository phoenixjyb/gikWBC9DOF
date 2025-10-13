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
%       PipelineConfig      - Unified configuration struct from 
%                             gik9dof.loadPipelineProfile(). When provided,
%                             all parameters are loaded from the profile
%                             unless explicitly overridden. This is the
%                             RECOMMENDED way to configure the pipeline.
%                             Example:
%                               cfg = gik9dof.loadPipelineProfile('default');
%                               log = trackReferenceTrajectory('PipelineConfig', cfg);
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
%       StageBReedsSheppParams - Struct overrides for Stage B RS smoothing
%                                (see gik9dof.control.defaultReedsSheppParams).
%       StageBHybridResolution / MotionPrimitiveLength default to 0.05 m /
%       0.20 m for dense Hybrid A* primitive sampling.
%       StageCUseBaseRefinement - Smooth Stage C base ribbon with RS + clothoid
%                                before executing pure pursuit (default true).
%       StageBChassisControllerMode/StageCChassisControllerMode select the
%       velocity controller (0 legacy diff, 1 heading, 2 pure pursuit, -1 use
%       chassis profile defaults).
%
%   Example:
%       log = gik9dof.trackReferenceTrajectory();
%       plot(log.timestamps, log.qTraj(1,1:end-1));
%
%   See also gik9dof.runTrajectoryControl, jsondecode.

arguments
    options.PipelineConfig struct = struct()  % NEW: Unified configuration from loadPipelineProfile
    options.JsonPath (1,1) string = "1_pull_world_scaled.json"
    options.BaseHome (1,3) double = [-2 -2 0]
    options.RateHz (1,1) double {mustBePositive} = 10
    options.Verbose (1,1) logical = true
    options.CommandFcn = []
    options.EnableAiming (1,1) logical = false
    options.DistanceWeight (1,1) double = 0.5
    options.DistanceMargin (1,1) double = 0.3
    options.FloorDiscs (1,:) struct = struct([])
    options.BaseDistanceBody (1,1) string = "abstract_chassis_link"
    options.Mode (1,1) string {mustBeMember(options.Mode, ["holistic","staged"])} = "holistic"
    options.ExecutionMode (1,1) string {mustBeMember(options.ExecutionMode, ["pureIk","ppForIk","ppFirst","pureMPC"])} = "ppForIk"
    options.UseHolisticRamp (1,1) logical = false
    options.RampMaxLinearSpeed (1,1) double = 1.5
    options.RampMaxYawRate (1,1) double = 3.0
    options.RampMaxJointSpeed (1,1) double = 1.0
    options.UseStageBHybridAStar (1,1) logical = true
    options.StageBHybridResolution (1,1) double = 0.05
    options.StageBHybridSafetyMargin (1,1) double = 0.15
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.2
    options.StageBUseReedsShepp (1,1) logical = false
    options.StageBReedsSheppParams struct = gik9dof.control.defaultReedsSheppParams()
    options.StageBUseClothoid (1,1) logical = false
    options.StageBClothoidParams struct = struct()
    options.StageCUseBaseRefinement (1,1) logical = true
    options.StageBChassisControllerMode (1,1) double {mustBeMember(options.StageBChassisControllerMode, [-1 0 1 2])} = -1
    options.StageCChassisControllerMode (1,1) double {mustBeMember(options.StageCChassisControllerMode, [-1 0 1 2])} = -1
    options.StageBLookaheadDistance (1,1) double {mustBePositive} = 0.6
    options.StageBDesiredLinearVelocity (1,1) double = 0.5
    options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.0
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "pureHyb"
    options.StageBDockingPositionTolerance (1,1) double {mustBeNonnegative} = 0.02
    options.StageBDockingYawTolerance (1,1) double {mustBeNonnegative} = 2*pi/180
    options.EnvironmentConfig (1,1) struct = gik9dof.environmentConfig()
    options.MaxIterations (1,1) double {mustBePositive} = 1500
    options.StageCLookaheadDistance (1,1) double {mustBePositive} = 0.4
    options.StageCLookaheadVelGain (1,1) double {mustBeNonnegative} = 0.2
    options.StageCLookaheadTimeGain (1,1) double {mustBeNonnegative} = 0.05
    options.StageCDesiredLinearVelocity (1,1) double = 1.0
    options.StageCMaxLinearSpeed (1,1) double {mustBePositive} = 1.5
    options.StageCMinLinearSpeed (1,1) double = -0.4
    options.StageCMaxAngularVelocity (1,1) double {mustBePositive} = 2.5
    options.StageCTrackWidth (1,1) double {mustBePositive} = 0.574
    options.StageCWheelBase (1,1) double {mustBePositive} = 0.36
    options.StageCMaxWheelSpeed (1,1) double {mustBePositive} = 3.3
    options.StageCWaypointSpacing (1,1) double {mustBePositive} = 0.15
    options.StageCPathBufferSize (1,1) double {mustBePositive} = 30.0
    options.StageCGoalTolerance (1,1) double {mustBePositive} = 0.10
    options.StageCInterpSpacing (1,1) double {mustBePositive} = 0.05
    options.StageCReverseEnabled (1,1) logical = true
    options.ChassisProfile (1,1) string = "wide_track"
    options.ChassisOverrides struct = struct()
end

% =========================================================================
% Apply unified PipelineConfig if provided
% =========================================================================
if ~isempty(fieldnames(options.PipelineConfig))
    options = applyPipelineConfig(options, options.PipelineConfig);
end

% Resolve assets and instantiate robot.
jsonPath = gik9dof.internal.resolvePath(options.JsonPath);
[robot, footprintInfo] = gik9dof.createRobotModel("Validate", true);

% Build initial configuration with specified base pose.
configTools = gik9dof.configurationTools(robot);
homeStruct = configTools.struct(configTools.homeConfig());

envConfig = options.EnvironmentConfig;
if isfield(envConfig, "BaseHome") && numel(envConfig.BaseHome) == 3
    baseHome = envConfig.BaseHome;
else
    baseHome = options.BaseHome;
end

usingDefaults = {};
if isfield(options, 'UsingDefaults')
    usingDefaults = options.UsingDefaults;
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

chassisParams = gik9dof.control.loadChassisProfile(options.ChassisProfile, ...
    "Overrides", options.ChassisOverrides);

stageBControllerMode = resolveControllerModeOption(options.StageBChassisControllerMode, chassisParams, "stageB_controller_mode");
stageCControllerMode = resolveControllerModeOption(options.StageCChassisControllerMode, chassisParams, "stageC_controller_mode");
options.StageBChassisControllerMode = stageBControllerMode;
options.StageCChassisControllerMode = stageCControllerMode;

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

stageBMode = string(options.StageBMode);
if ismember('StageBMode', usingDefaults) && isfield(envConfig, "StageBMode") && ~isempty(envConfig.StageBMode)
    try
        stageBMode = string(validatestring(string(envConfig.StageBMode), {"gikInLoop", "pureHyb"}));
    catch
        warning('gik9dof:trackReferenceTrajectory:InvalidStageBMode', ...
            'Environment config StageBMode "%s" is invalid; using %s.', ...
            string(envConfig.StageBMode), stageBMode);
    end
end

stageBDockPosTol = options.StageBDockingPositionTolerance;
if ismember('StageBDockingPositionTolerance', usingDefaults) && ...
        isfield(envConfig, "StageBDockingPositionTolerance") && ...
        ~isempty(envConfig.StageBDockingPositionTolerance)
    stageBDockPosTol = max(0, envConfig.StageBDockingPositionTolerance);
end

stageBDockYawTol = options.StageBDockingYawTolerance;
if ismember('StageBDockingYawTolerance', usingDefaults) && ...
        isfield(envConfig, "StageBDockingYawTolerance") && ...
        ~isempty(envConfig.StageBDockingYawTolerance)
    stageBDockYawTol = max(0, envConfig.StageBDockingYawTolerance);
end

floorDiscInfo = struct('Name', {}, 'Radius', {}, 'SafetyMargin', {});
distanceSpecs = struct([]);

if ~isempty(floorDiscSource)
    floorDiscInfo = gik9dof.addFloorDiscs(robot, floorDiscSource);
    numDiscs = numel(floorDiscInfo);

    specList = struct('Body', {}, 'ReferenceBody', {}, 'Bounds', {}, 'Weight', {});
    baseWeight = max(distanceWeight, max(5, distanceWeight));

    if strlength(options.BaseDistanceBody) > 0
        for k = 1:numDiscs
            discName = floorDiscInfo(k).Name;
            lowerBound = floorDiscInfo(k).Radius + floorDiscInfo(k).SafetyMargin + distanceMargin;
            specList(end+1) = struct( ... %#ok<AGROW>
                'Body', options.BaseDistanceBody, ...
                'ReferenceBody', discName, ...
                'Bounds', [lowerBound, Inf], ...
                'Weight', baseWeight);
        end
    end

    footprintNames = footprintInfo.Names;
    if ~isempty(footprintNames)
        fpWeight = max(baseWeight * 2, 20);
        for fp = 1:numel(footprintNames)
            for k = 1:numDiscs
                discName = floorDiscInfo(k).Name;
                lowerBound = floorDiscInfo(k).Radius + floorDiscInfo(k).SafetyMargin + distanceMargin;
                specList(end+1) = struct( ... %#ok<AGROW>
                    'Body', footprintNames(fp), ...
                    'ReferenceBody', discName, ...
                    'Bounds', [lowerBound, Inf], ...
                    'Weight', fpWeight);
            end
        end
    end

    if ~isempty(specList)
        distanceSpecs = specList;
    end
end

% Attach available collision meshes for downstream checks and visualisation.
colTools = gik9dof.collisionTools(robot, 'MeshDirectory', 'meshes');
colTools.apply();

% Instantiate solver bundle.
bundle = gik9dof.createGikSolver(robot, ...
    "EnableAiming", options.EnableAiming, ...
    "DistanceSpecs", distanceSpecs, ...
    "DistanceWeight", distanceWeight, ...
    "MaxIterations", options.MaxIterations);

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
        switch options.ExecutionMode
            case "ppForIk"
                % ===================================================================
                % HOLISTIC MODE: Pure Pursuit for IK (Three-Pass Architecture)
                % ===================================================================
                % This implementation is IDENTICAL to Staged Stage C (ppForIk mode).
                % See projectDiagnosis.md: "Critical Equivalence" section.
                %
                % Three-Pass Algorithm:
                %   Pass 1: GIK reference → ideal base trajectory (may not be feasible)
                %   Pass 2: Chassis simulation → realistic executed base trajectory
                %   Pass 3: GIK with fixed base → final arm motion with realistic base
                %
                % This ensures kinematically feasible base motion while tracking EE targets.
                % ===================================================================
                
                % Pass 1: Generate reference base path from IK
                bundleRef = gik9dof.createGikSolver(robot, ...
                    "EnableAiming", options.EnableAiming, ...
                    "DistanceSpecs", distanceSpecs, ...
                    "DistanceWeight", distanceWeight, ...
                    "MaxIterations", options.MaxIterations);

                logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...
                    "InitialConfiguration", q0, ...
                    "RateHz", options.RateHz, ...
                    "VelocityLimits", velLimits, ...
                    "CommandFcn", commandFcn, ...
                    "Verbose", options.Verbose);

                baseReferenceTail = logRef.qTraj(baseIdx, 2:end).';
                baseReference = [q0(baseIdx).'; baseReferenceTail];
                
                % Pass 2: Simulate chassis controller with kinematic constraints
                chassisHolistic = chassisParams;
                chassisHolistic.vx_max = options.RampMaxLinearSpeed;
                chassisHolistic.vx_min = max(chassisHolistic.vx_min, -options.RampMaxLinearSpeed);
                chassisHolistic.wz_max = options.RampMaxYawRate;
                chassisHolistic.wheel_speed_max = max(chassisHolistic.wheel_speed_max, options.RampMaxLinearSpeed + 0.5);
                chassisHolistic.reverse_enabled = false;
                chassisHolistic.interp_spacing_min = options.StageCInterpSpacing;
                chassisHolistic.interp_spacing_max = options.StageCWaypointSpacing;

                followerOptions = struct( ...
                    'SampleTime', 1/options.RateHz, ...
                    'ChassisParams', chassisHolistic, ...
                    'ControllerMode', "blended", ...
                    'LookaheadBase', options.StageCLookaheadDistance, ...
                    'LookaheadVelGain', options.StageCLookaheadVelGain, ...
                    'LookaheadTimeGain', options.StageCLookaheadTimeGain, ...
                    'GoalTolerance', options.StageCGoalTolerance, ...
                    'ReverseEnabled', false, ...
                    'HeadingKp', getStructFieldOrDefault(chassisHolistic, 'heading_kp', 1.2), ...
                    'FeedforwardGain', getStructFieldOrDefault(chassisHolistic, 'feedforward_gain', 0.9));

                simRes = gik9dof.control.simulateChassisExecution(baseReference, ...
                    'SampleTime', 1/options.RateHz, 'FollowerOptions', followerOptions, ...
                    'ControllerMode', stageCControllerMode);

                % Pass 3: Final IK with base locked to executed trajectory
                bundleFinal = gik9dof.createGikSolver(robot, ...
                    "EnableAiming", options.EnableAiming, ...
                    "DistanceSpecs", distanceSpecs, ...
                    "DistanceWeight", distanceWeight, ...
                    "MaxIterations", options.MaxIterations);

                baseExecutedFull = simRes.poses;
                if ~isempty(baseExecutedFull)
                    baseExecutedFull(1,:) = baseReference(1,:);
                end
                baseExecutedFull = resampleBasePath(baseExecutedFull, baseReference);
                executedBase = baseExecutedFull(2:end, :);
                fixedTrajectory = struct('Indices', baseIdx, 'Values', executedBase');
                log = gik9dof.runTrajectoryControl(bundleFinal, trajStruct, ...
                    "InitialConfiguration", q0, ...
                    "RateHz", options.RateHz, ...
                    "VelocityLimits", velLimits, ...
                    "CommandFcn", commandFcn, ...
                    "Verbose", options.Verbose, ...
                    "FixedJointTrajectory", fixedTrajectory);
                log.mode = "holistic";
                log.simulationMode = "ppForIk";
                log.purePursuit.referencePath = baseReference;
                log.purePursuit.simulation = simRes;
                log.purePursuit.executedPath = executedBase;
                log.purePursuit.sampleTime = 1/options.RateHz;
                log.purePursuit.commands = simRes.commands;
                log.purePursuit.wheelSpeeds = simRes.wheelSpeeds;
                log.purePursuit.status = simRes.status;
                log.referenceInitialIk = logRef;
                log.execBaseStates = baseExecutedFull;
                log.referenceBaseStates = baseReference;
                dt = 1 / options.RateHz;
                if ~isempty(simRes.commands)
                    timeCommand = (0:size(simRes.commands,1)-1)' * dt;
                    log.cmdLog = table(timeCommand, simRes.commands(:,1), simRes.commands(:,2), ...
                        'VariableNames', {'time','Vx','Wz'});
                else
                    log.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
                        'VariableNames', {'time','Vx','Wz'});
                end
            case "pureIk"
                bundleSingle = gik9dof.createGikSolver(robot, ...
                    "EnableAiming", options.EnableAiming, ...
                    "DistanceSpecs", distanceSpecs, ...
                    "DistanceWeight", distanceWeight, ...
                    "MaxIterations", options.MaxIterations);
                log = gik9dof.runTrajectoryControl(bundleSingle, trajStruct, ...
                    "InitialConfiguration", q0, ...
                    "RateHz", options.RateHz, ...
                    "VelocityLimits", velLimits, ...
                    "CommandFcn", commandFcn, ...
                    "Verbose", options.Verbose);
                log.mode = "holistic";
                log.simulationMode = "pureIk";
                 log.execBaseStates = log.qTraj(baseIdx, :).';
                log.referenceBaseStates = log.execBaseStates;
                log.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
                    'VariableNames', {'time','Vx','Wz'});
        end
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
            'StageBUseReedsShepp', options.StageBUseReedsShepp, ...
            'StageBReedsSheppParams', options.StageBReedsSheppParams, ...
            'StageBUseClothoid', options.StageBUseClothoid, ...
            'StageBClothoidParams', options.StageBClothoidParams, ...
            'StageBChassisControllerMode', options.StageBChassisControllerMode, ...
            'StageCUseBaseRefinement', options.StageCUseBaseRefinement, ...
            'StageCChassisControllerMode', options.StageCChassisControllerMode, ...
            'StageBMaxLinearSpeed', options.RampMaxLinearSpeed, ...
            'StageBMaxYawRate', options.RampMaxYawRate, ...
            'StageBMaxJointSpeed', options.RampMaxJointSpeed, ...
            'StageBLookaheadDistance', options.StageBLookaheadDistance, ...
            'StageBDesiredLinearVelocity', options.StageBDesiredLinearVelocity, ...
            'StageBMaxAngularVelocity', options.StageBMaxAngularVelocity, ...
            'StageBMode', stageBMode, ...
            'StageBDockingPositionTolerance', stageBDockPosTol, ...
            'StageBDockingYawTolerance', stageBDockYawTol, ...
            'EnvironmentConfig', envConfig, ...
            'MaxIterations', options.MaxIterations, ...
            'ExecutionMode', options.ExecutionMode, ...
            'PipelineConfig', options.PipelineConfig, ...
            'StageCLookaheadDistance', options.StageCLookaheadDistance, ...
            'StageCLookaheadVelGain', options.StageCLookaheadVelGain, ...
            'StageCLookaheadTimeGain', options.StageCLookaheadTimeGain, ...
            'StageCDesiredLinearVelocity', options.StageCDesiredLinearVelocity, ...
            'StageCMaxLinearSpeed', options.StageCMaxLinearSpeed, ...
            'StageCMinLinearSpeed', options.StageCMinLinearSpeed, ...
            'StageCMaxAngularVelocity', options.StageCMaxAngularVelocity, ...
            'StageCTrackWidth', options.StageCTrackWidth, ...
            'StageCWheelBase', options.StageCWheelBase, ...
            'StageCMaxWheelSpeed', options.StageCMaxWheelSpeed, ...
            'StageCWaypointSpacing', options.StageCWaypointSpacing, ...
            'StageCPathBufferSize', options.StageCPathBufferSize, ...
            'StageCGoalTolerance', options.StageCGoalTolerance, ...
            'StageCInterpSpacing', options.StageCInterpSpacing, ...
            'StageCReverseEnabled', options.StageCReverseEnabled, ...
            'ChassisProfile', options.ChassisProfile, ...
            'ChassisOverrides', options.ChassisOverrides);
        log = pipeline;
    otherwise
        error("gik9dof:trackReferenceTrajectory:UnknownMode", options.Mode);
end

log.floorDiscs = floorDiscInfo;
log.distanceSpecs = distanceSpecs;
log.environment = envConfig;
log.footprintInfo = footprintInfo;
log.chassisProfile = options.ChassisProfile;
log.chassisParams = chassisParams;
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

function resampled = resampleBasePath(executed, reference)
if isempty(executed)
    resampled = reference;
    return
end

refCount = size(reference,1);
if size(executed,1) == refCount
    resampled = executed;
    return
end

sExec = linspace(0, 1, size(executed,1));
sRef = linspace(0, 1, refCount);
resampled = zeros(refCount, 3);
resampled(:,1) = interp1(sExec, executed(:,1), sRef, 'linear', 'extrap');
resampled(:,2) = interp1(sExec, executed(:,2), sRef, 'linear', 'extrap');
yawExec = unwrap(executed(:,3));
yawInterp = interp1(sExec, yawExec, sRef, 'linear', 'extrap');
resampled(:,3) = wrapToPi(yawInterp);
end

function val = getStructFieldOrDefault(s, name, defaultValue)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = defaultValue;
end
end

function modeOut = resolveControllerModeOption(modeIn, chassisParams, fieldName)
if nargin < 3 || isempty(fieldName)
    fieldName = "";
end

if isempty(modeIn) || modeIn == -1
    profileMode = 2;
    if ~isempty(fieldName) && isfield(chassisParams, fieldName)
        profileMode = double(chassisParams.(fieldName));
    elseif isfield(chassisParams, 'controller_mode')
        profileMode = double(chassisParams.controller_mode);
    end
    modeOut = clampControllerMode(profileMode);
else
    modeOut = clampControllerMode(modeIn);
end
end

function mode = clampControllerMode(val)
validModes = [0 1 2];
if any(val == validModes)
    mode = val;
else
    mode = 2;
end
end

function options = applyPipelineConfig(options, config)
%APPLYPIPELINECONFIG Map unified PipelineConfig to trackReferenceTrajectory options.
%   This function maps fields from the unified pipeline configuration (loaded
%   via gik9dof.loadPipelineProfile) to the legacy parameter names used by
%   trackReferenceTrajectory. Only applies config values if the user didn't
%   explicitly override them.
%
%   This maintains backward compatibility while enabling the new unified
%   configuration system.

% Track which parameters were explicitly provided by user
providedParams = {};
if isfield(options, 'UsingDefaults')
    providedParams = setdiff(fieldnames(options), options.UsingDefaults);
end

% Helper function to set parameter only if not explicitly provided
    function setIfNotProvided(paramName, configValue)
        if isempty(providedParams) || ~ismember(paramName, providedParams)
            options.(paramName) = configValue;
        end
    end

% Map Stage B parameters
if isfield(config, 'stage_b')
    sb = config.stage_b;
    if isfield(sb, 'mode'), setIfNotProvided('StageBMode', string(sb.mode)); end
    if isfield(sb, 'lookahead_distance'), setIfNotProvided('StageBLookaheadDistance', sb.lookahead_distance); end
    if isfield(sb, 'desired_linear_velocity'), setIfNotProvided('StageBDesiredLinearVelocity', sb.desired_linear_velocity); end
    if isfield(sb, 'max_angular_velocity'), setIfNotProvided('StageBMaxAngularVelocity', sb.max_angular_velocity); end
    if isfield(sb, 'docking_position_tolerance'), setIfNotProvided('StageBDockingPositionTolerance', sb.docking_position_tolerance); end
    if isfield(sb, 'docking_yaw_tolerance'), setIfNotProvided('StageBDockingYawTolerance', sb.docking_yaw_tolerance); end
    if isfield(sb, 'use_hybrid_astar'), setIfNotProvided('UseStageBHybridAStar', sb.use_hybrid_astar); end
    if isfield(sb, 'hybrid_resolution'), setIfNotProvided('StageBHybridResolution', sb.hybrid_resolution); end
    if isfield(sb, 'hybrid_safety_margin'), setIfNotProvided('StageBHybridSafetyMargin', sb.hybrid_safety_margin); end
    if isfield(sb, 'hybrid_min_turning_radius'), setIfNotProvided('StageBHybridMinTurningRadius', sb.hybrid_min_turning_radius); end
    if isfield(sb, 'hybrid_motion_primitive_length'), setIfNotProvided('StageBHybridMotionPrimitiveLength', sb.hybrid_motion_primitive_length); end
    if isfield(sb, 'use_reeds_shepp'), setIfNotProvided('StageBUseReedsShepp', sb.use_reeds_shepp); end
    if isfield(sb, 'reeds_shepp_params'), setIfNotProvided('StageBReedsSheppParams', sb.reeds_shepp_params); end
    if isfield(sb, 'use_clothoid'), setIfNotProvided('StageBUseClothoid', sb.use_clothoid); end
    if isfield(sb, 'clothoid_params'), setIfNotProvided('StageBClothoidParams', sb.clothoid_params); end
    if isfield(sb, 'chassis_controller_mode'), setIfNotProvided('StageBChassisControllerMode', sb.chassis_controller_mode); end
end

% Map Stage C parameters
if isfield(config, 'stage_c')
    sc = config.stage_c;
    if isfield(sc, 'lookahead_distance'), setIfNotProvided('StageCLookaheadDistance', sc.lookahead_distance); end
    if isfield(sc, 'lookahead_vel_gain'), setIfNotProvided('StageCLookaheadVelGain', sc.lookahead_vel_gain); end
    if isfield(sc, 'lookahead_time_gain'), setIfNotProvided('StageCLookaheadTimeGain', sc.lookahead_time_gain); end
    if isfield(sc, 'desired_linear_velocity'), setIfNotProvided('StageCDesiredLinearVelocity', sc.desired_linear_velocity); end
    if isfield(sc, 'max_linear_speed'), setIfNotProvided('StageCMaxLinearSpeed', sc.max_linear_speed); end
    if isfield(sc, 'min_linear_speed'), setIfNotProvided('StageCMinLinearSpeed', sc.min_linear_speed); end
    if isfield(sc, 'max_angular_velocity'), setIfNotProvided('StageCMaxAngularVelocity', sc.max_angular_velocity); end
    if isfield(sc, 'track_width'), setIfNotProvided('StageCTrackWidth', sc.track_width); end
    if isfield(sc, 'wheel_base'), setIfNotProvided('StageCWheelBase', sc.wheel_base); end
    if isfield(sc, 'max_wheel_speed'), setIfNotProvided('StageCMaxWheelSpeed', sc.max_wheel_speed); end
    if isfield(sc, 'waypoint_spacing'), setIfNotProvided('StageCWaypointSpacing', sc.waypoint_spacing); end
    if isfield(sc, 'path_buffer_size'), setIfNotProvided('StageCPathBufferSize', sc.path_buffer_size); end
    if isfield(sc, 'goal_tolerance'), setIfNotProvided('StageCGoalTolerance', sc.goal_tolerance); end
    if isfield(sc, 'interp_spacing'), setIfNotProvided('StageCInterpSpacing', sc.interp_spacing); end
    if isfield(sc, 'reverse_enabled'), setIfNotProvided('StageCReverseEnabled', sc.reverse_enabled); end
    if isfield(sc, 'use_base_refinement'), setIfNotProvided('StageCUseBaseRefinement', sc.use_base_refinement); end
    if isfield(sc, 'chassis_controller_mode'), setIfNotProvided('StageCChassisControllerMode', sc.chassis_controller_mode); end
end

% Map Holistic parameters
if isfield(config, 'holistic')
    hol = config.holistic;
    if isfield(hol, 'use_ramp'), setIfNotProvided('UseHolisticRamp', hol.use_ramp); end
    if isfield(hol, 'ramp_max_linear_speed'), setIfNotProvided('RampMaxLinearSpeed', hol.ramp_max_linear_speed); end
    if isfield(hol, 'ramp_max_yaw_rate'), setIfNotProvided('RampMaxYawRate', hol.ramp_max_yaw_rate); end
    if isfield(hol, 'ramp_max_joint_speed'), setIfNotProvided('RampMaxJointSpeed', hol.ramp_max_joint_speed); end
end

% Map GIK parameters
if isfield(config, 'gik')
    gik = config.gik;
    if isfield(gik, 'max_iterations'), setIfNotProvided('MaxIterations', gik.max_iterations); end
    if isfield(gik, 'distance_margin'), setIfNotProvided('DistanceMargin', gik.distance_margin); end
end

% Map Chassis parameters (will be used by loadChassisProfile)
% Note: We don't override ChassisProfile/ChassisOverrides directly, but
% these will be automatically picked up by the existing chassis loading logic
if isfield(config, 'chassis')
    % Store chassis config for potential use
    if isempty(providedParams) || ~ismember('ChassisOverrides', providedParams)
        % Merge chassis config into ChassisOverrides
        options.ChassisOverrides = mergeStructs(options.ChassisOverrides, config.chassis);
    end
end

end

function merged = mergeStructs(base, overlay)
%MERGESTRUCTS Shallow merge of two structs (overlay takes precedence).
merged = base;
if isempty(overlay)
    return
end
overlayFields = fieldnames(overlay);
for i = 1:numel(overlayFields)
    merged.(overlayFields{i}) = overlay.(overlayFields{i});
end
end
