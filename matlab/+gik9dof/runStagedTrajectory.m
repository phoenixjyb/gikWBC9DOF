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
%   Recommended (NEW):
%       PipelineConfig      - Unified configuration struct from
%                             gik9dof.loadPipelineProfile(). When provided,
%                             all Stage B/C parameters are loaded from the
%                             profile. Example:
%                               cfg = gik9dof.loadPipelineProfile('default');
%                               pipeline = runStagedTrajectory(robot, traj, ...
%                                   'PipelineConfig', cfg, ...
%                                   'InitialConfiguration', q0, ...
%                                   'ConfigTools', configTools);
%
%   Optional name-value:
%       DistanceSpecs       - Distance constraint specs for Stage C (struct array).
%       RateHz              - Control loop rate (default 10).
%       Verbose             - Verbosity flag (default false).
%       DistanceWeight      - Weight used when DistanceSpecs empty (default 0.5).
%       StageBReedsSheppParams - Struct controlling Stage B RS shortcutting
%                                (see gik9dof.control.defaultReedsSheppParams).
%       StageBHybridResolution/StageBHybridMotionPrimitiveLength default to
%       0.05 m and 0.2 m respectively to provide dense Hybrid A* primitives.
%       StageCUseBaseRefinement - Apply RS/clothoid smoothing to the Stage C
%                                 base ribbon before running pure pursuit.
%
arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    options.PipelineConfig struct = struct()  % NEW: Unified configuration from loadPipelineProfile
    options.InitialConfiguration (:,1) double
    options.ConfigTools (1,1) struct
    options.DistanceSpecs struct = struct([])
    options.DistanceWeight (1,1) double = 0.5
    options.RateHz (1,1) double {mustBePositive} = 10
    options.Verbose (1,1) logical = false
    options.CommandFcn = []
    options.FloorDiscs struct = struct([])
    options.UseStageBHybridAStar (1,1) logical = false
    options.StageBHybridResolution (1,1) double = 0.05
    options.StageBHybridSafetyMargin (1,1) double = 0.1
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.2
    options.StageBUseReedsShepp (1,1) logical = false
    options.StageBReedsSheppParams struct = gik9dof.control.defaultReedsSheppParams()
    options.StageBUseClothoid (1,1) logical = false
    options.StageBClothoidParams struct = struct()
    options.StageBChassisControllerMode (1,1) double {mustBeMember(options.StageBChassisControllerMode, [-1 0 1 2])} = -1
    options.StageBMaxLinearSpeed (1,1) double = 1.5
    options.StageBMaxYawRate (1,1) double = 3.0
    options.StageBMaxJointSpeed (1,1) double = 1.0
    options.StageBLookaheadDistance (1,1) double {mustBePositive} = 0.6
    options.StageBDesiredLinearVelocity (1,1) double = 0.6
    options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.5
    options.EnvironmentConfig struct = struct()
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "gikInLoop"
    options.StageBDockingPositionTolerance (1,1) double {mustBeNonnegative} = 0.02
    options.StageBDockingYawTolerance (1,1) double {mustBeNonnegative} = 2*pi/180
    options.ChassisProfile (1,1) string = "wide_track"
    options.ChassisOverrides struct = struct()
    options.ExecutionMode (1,1) string {mustBeMember(options.ExecutionMode, ["pureIk","ppForIk"])} = "ppForIk"
    options.StageCLookaheadDistance (1,1) double {mustBePositive} = 0.8
    options.StageCLookaheadVelGain (1,1) double {mustBeNonnegative} = 0.2
    options.StageCLookaheadTimeGain (1,1) double {mustBeNonnegative} = 0.05
    options.StageCDesiredLinearVelocity (1,1) double = 1.0
    options.StageCMaxLinearSpeed (1,1) double {mustBePositive} = 1.5
    options.StageCMinLinearSpeed (1,1) double = -1.0
    options.StageCMaxAngularVelocity (1,1) double {mustBePositive} = 2.0
    options.StageCTrackWidth (1,1) double {mustBePositive} = 0.574
    options.StageCWheelBase (1,1) double {mustBePositive} = 0.36
    options.StageCMaxWheelSpeed (1,1) double {mustBePositive} = 2.0
    options.StageCWaypointSpacing (1,1) double {mustBePositive} = 0.15
    options.StageCPathBufferSize (1,1) double {mustBePositive} = 30.0
    options.StageCGoalTolerance (1,1) double {mustBePositive} = 0.05
    options.StageCInterpSpacing (1,1) double {mustBePositive} = 0.05
    options.StageCReverseEnabled (1,1) logical = true
    options.StageCChassisControllerMode (1,1) double {mustBeMember(options.StageCChassisControllerMode, [-1 0 1 2])} = -1
    options.StageCUseBaseRefinement (1,1) logical = false
    options.MaxIterations (1,1) double {mustBePositive} = 1500
end

% =========================================================================
% Apply unified PipelineConfig if provided
% =========================================================================
if ~isempty(fieldnames(options.PipelineConfig))
    options = applyPipelineConfigToStaged(options, options.PipelineConfig);
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

chassisParams = gik9dof.control.loadChassisProfile(options.ChassisProfile, ...
    "Overrides", options.ChassisOverrides);
chassisParams.reverse_enabled = chassisParams.reverse_enabled | strcmpi(options.StageBMode, "purehyb");

options.StageBChassisControllerMode = resolveChassisControllerMode( ...
    options.StageBChassisControllerMode, chassisParams, "stageB_controller_mode");
options.StageCChassisControllerMode = resolveChassisControllerMode( ...
    options.StageCChassisControllerMode, chassisParams, "stageC_controller_mode");

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

bundleA = gik9dof.createGikSolver(robot, 'MaxIterations', options.MaxIterations);
lockJointBounds(bundleA.constraints.joint, baseIdx, q0);

logA = gik9dof.runTrajectoryControl(bundleA, trajA, ...
    'InitialConfiguration', q0, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

qA_end = logA.qTraj(:, end);

%% Stage B: base-only alignment of x-y while arm frozen
stageBMode = lower(string(options.StageBMode));
dt = 1 / options.RateHz;
stageBStart = getTransform(robot, qA_end, trajStruct.EndEffectorName);
switch stageBMode
    case "gikinloop"
        stageBResult = executeStageBGikInLoop(robot, trajStruct.EndEffectorName, qA_end, baseIdx, armIdx, ...
            T1, stageBStart, NB, velLimits, chassisParams, options);
    case "purehyb"
        stageBResult = executeStageBPureHyb(robot, trajStruct.EndEffectorName, qA_end, baseIdx, armIdx, ...
            T1, NB, velLimits, chassisParams, options);
    otherwise
        error('gik9dof:runStagedTrajectory:UnsupportedStageBMode', ...
            'Unknown Stage B mode "%s".', stageBMode);
end

logB = stageBResult.log;
if isfield(stageBResult, 'goalBase') && isfield(stageBResult, 'achievedBase') && ...
        ~isempty(stageBResult.goalBase) && ~isempty(stageBResult.achievedBase)
    checkStageBDocking(stageBResult.goalBase, stageBResult.achievedBase, ...
        options.StageBDockingPositionTolerance, options.StageBDockingYawTolerance, stageBMode);
end

alignmentInfo = generateStageCAlignmentInfo(stageBResult, qA_end(baseIdx).', options, chassisParams, dt);
if alignmentInfo.applied
    stageBResult = applyStageCAlignment(stageBResult, baseIdx, alignmentInfo);
else
    logB = stageBResult.log;
    logB.alignment = alignmentInfo;
    stageBResult.log = logB;
    stageBResult.alignment = alignmentInfo;
end
logB = stageBResult.log;
qB_end = stageBResult.qFinal;

%% Stage C: full-body tracking of remaining reference
trajC = trajStruct;
logC = struct();

if ~isempty(trajC.Poses)
    switch options.ExecutionMode
        case "ppForIk"
            logC = executeStageCPurePursuit(robot, trajC, qB_end, baseIdx, armIdx, velLimits, chassisParams, alignmentInfo, options, stageBResult);
        case "pureIk"
            logC = executeStageCPureIk(robot, trajC, qB_end, baseIdx, velLimits, options);
        otherwise
            error('gik9dof:runStagedTrajectory:UnsupportedExecutionMode', ...
                'Unknown execution mode "%s".', string(options.ExecutionMode));
    end
else
    % No remaining waypoints; synthesise empty log
    logC = emptyLog(robot, qB_end, options.RateHz);
    logC.simulationMode = options.ExecutionMode;
end
logC.chassisParams = chassisParams;
logC.preStageCAlignment = alignmentInfo;

%% Combine logs
pipeline = mergeStageLogs(logA, logB, logC);
pipeline.mode = "staged";
pipeline.stageLogs = struct('stageA', logA, 'stageB', logB, 'stageC', logC);
pipeline.distanceSpecs = options.DistanceSpecs;
pipeline.rateHz = options.RateHz;
pipeline.referenceTrajectory = trajStruct;
pipeline.environment = options.EnvironmentConfig;
pipeline.stageBMode = stageBMode;
pipeline.simulationMode = options.ExecutionMode;
pipeline.chassisParams = chassisParams;
pipeline.preStageCAlignment = alignmentInfo;
if isfield(stageBResult, 'goalBase')
    pipeline.stageBGoal = stageBResult.goalBase;
end
if isfield(stageBResult, 'planner')
    pipeline.stageBPlanner = stageBResult.planner;
end
pipeline.stageBControllerMode = options.StageBChassisControllerMode;
pipeline.stageCControllerMode = options.StageCChassisControllerMode;
end

function lockJointBounds(jointConstraint, indices, values)
for idx = indices
    jointConstraint.Bounds(idx, :) = [values(idx) values(idx)];
end
end

function result = executeStageBGikInLoop(robot, eeName, qStart, baseIdx, armIdx, targetPose, stageBStartPose, nSamples, velLimits, chassisParams, options)
%EXECUTESTAGEBGIKINLOOP Stage B implementation matching legacy behaviour.
cmdLogPP = [];
stageBStates = [];
goalBase = [];
simResStageB = [];
plannerReference = [];
plannerInfo = struct();
dt = 1 / options.RateHz;

chassisStageB = chassisParams;
chassisStageB.vx_max = min(chassisStageB.vx_max, options.StageBMaxLinearSpeed);
chassisStageB.vx_min = max(chassisStageB.vx_min, -options.StageBMaxLinearSpeed);
chassisStageB.wz_max = min(chassisStageB.wz_max, options.StageBMaxAngularVelocity);
chassisStageB.reverse_enabled = false;

posTolStageB = max(options.StageBDockingPositionTolerance, 0.02);
followerOptions = struct( ...
    'SampleTime', dt, ...
    'ChassisParams', chassisStageB, ...
    'ControllerMode', "blended", ...
    'LookaheadBase', options.StageBLookaheadDistance, ...
    'LookaheadVelGain', 0.0, ...
    'LookaheadTimeGain', 0.0, ...
    'GoalTolerance', posTolStageB, ...
    'ReverseEnabled', false, ...
    'HeadingKp', getStructFieldOrDefault(chassisStageB, 'heading_kp', 1.2), ...
    'FeedforwardGain', getStructFieldOrDefault(chassisStageB, 'feedforward_gain', 0.9));

if options.UseStageBHybridAStar
    try
        [plannerStates, plannerGoalBase, plannerInfo] = planStageBHybridAStarPath(qStart, baseIdx, targetPose, options, robot, eeName, options.FloorDiscs);
        goalBase = plannerGoalBase;
    catch plannerErr
        if ~isempty(plannerErr.stack)
            origin = sprintf('%s:%d', plannerErr.stack(1).name, plannerErr.stack(1).line);
        else
            origin = 'unknown origin';
        end
        warning('gik9dof:runStagedTrajectory:HybridAStarFailed', ...
            'Stage B hybrid A* failed (%s at %s); falling back to interpolation.', plannerErr.message, origin);
        plannerStates = [];
        plannerInfo = struct();
    end

    if ~isempty(plannerStates)
        plannerReference = plannerStates;
        simResStageB = gik9dof.control.simulateChassisController(plannerStates, ...
            'SampleTime', dt, 'FollowerOptions', followerOptions, ...
            'ControllerMode', options.StageBChassisControllerMode);
        simResStageB = ensureDocking(simResStageB, goalBase, followerOptions, dt, ...
            options.StageBDockingPositionTolerance, options.StageBDockingYawTolerance, ...
            options.StageBChassisControllerMode);
        if ~isempty(simResStageB.commands)
            timeCommand = (0:size(simResStageB.commands,1)-1)' * dt;
            cmdLogPP = [timeCommand, simResStageB.commands];
        else
            cmdLogPP = zeros(0,3);
        end

        posesPP = simResStageB.poses;
        if size(posesPP,1) >= 1
            posesPP(1,:) = qStart(baseIdx).';
        end
        if size(posesPP,1) >= 2
            stageBStates = posesPP(2:end, :);
        else
            stageBStates = zeros(0,3);
        end

        if isempty(stageBStates)
            warning('gik9dof:runStagedTrajectory:PurePursuitEmpty', ...
                'Pure pursuit integration returned no states; reverting to interpolation trajectory.');
            stageBStates = [];
            cmdLogPP = [];
            stageBPoses = generateStagePoses(stageBStartPose, targetPose, nSamples, 'base');
        else
            stageBPoses = statesToEndEffectorPoses(robot, qStart, baseIdx, stageBStates, eeName);
        end
    else
        stageBPoses = generateStagePoses(stageBStartPose, targetPose, nSamples, 'base');
    end
else
    [goalBase, ~] = computeStageBGoalBase(robot, qStart, baseIdx, targetPose, eeName, options.MaxIterations);
    stageBPoses = generateStagePoses(stageBStartPose, targetPose, nSamples, 'base');
end

if isempty(goalBase)
    [goalBase, ~] = computeStageBGoalBase(robot, qStart, baseIdx, targetPose, eeName, options.MaxIterations);
end

trajB = struct('Poses', stageBPoses);
trajB.EndEffectorName = eeName;

bundleB = gik9dof.createGikSolver(robot, 'MaxIterations', options.MaxIterations);
lockJointBounds(bundleB.constraints.joint, armIdx, qStart);

logB = gik9dof.runTrajectoryControl(bundleB, trajB, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

qFinal = logB.qTraj(:, end);
achievedBase = qFinal(baseIdx).';
if isempty(goalBase)
    goalBase = achievedBase;
end

if ~isempty(plannerReference)
    baseStates = plannerReference;
else
    if isempty(stageBStates)
        baseStates = logB.qTraj(baseIdx, :)';
    else
        baseStates = stageBStates;
    end
end
logB.pathStates = baseStates;
logB.execBaseStates = baseStates;
logB.mode = "gikInLoop";
logB.goalBase = goalBase;
logB.achievedBase = achievedBase;
if ~isempty(cmdLogPP)
    logB.cmdLog = array2table(cmdLogPP, 'VariableNames', {'time','Vx','Wz'});
else
    logB.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
        'VariableNames', {'time','Vx','Wz'});
end

if ~isempty(simResStageB) && ~isempty(plannerReference)
    logB.purePursuit.referencePath = plannerReference;
    logB.purePursuit.simulation = simResStageB;
    logB.purePursuit.executedPath = simResStageB.poses;
    logB.purePursuit.sampleTime = dt;
    logB.purePursuit.chassisParams = chassisStageB;
    logB.purePursuit.controllerMode = options.StageBChassisControllerMode;
end
logB.planner = plannerInfo;

result = struct();
result.log = logB;
result.qFinal = qFinal;
result.pathStates = baseStates;
result.goalBase = goalBase;
result.achievedBase = achievedBase;
result.mode = "gikInLoop";
if isfield(logB, 'purePursuit')
    result.purePursuit = logB.purePursuit;
end
result.planner = plannerInfo;
end

function result = executeStageBPureHyb(robot, eeName, qStart, baseIdx, ~, targetPose, nSamples, ~, chassisParams, options)
%EXECUTESTAGEBPUREHYB Stage B implementation using planner + pure pursuit.

[goalBase, ~] = computeStageBGoalBase(robot, qStart, baseIdx, targetPose, eeName, options.MaxIterations);

plannerStates = [];
plannerInfo = struct();
if options.UseStageBHybridAStar
    try
    [plannerStates, plannerGoalBase, plannerInfo] = planStageBHybridAStarPath(qStart, baseIdx, targetPose, options, robot, eeName, options.FloorDiscs);
        if ~isempty(plannerGoalBase)
            goalBase = plannerGoalBase;
        end
    catch plannerErr
        if ~isempty(plannerErr.stack)
            origin = sprintf('%s:%d', plannerErr.stack(1).name, plannerErr.stack(1).line);
        else
            origin = 'unknown origin';
        end
        warning('gik9dof:runStagedTrajectory:HybridAStarFailed', ...
            'Stage B hybrid A* failed (%s at %s); falling back to interpolation.', plannerErr.message, origin);
        plannerStates = [];
        plannerInfo = struct();
    end
end

if isempty(plannerStates)
    plannerStates = interpolateBaseStates(qStart(baseIdx).', goalBase, max(nSamples, 2));
end

dt = 1 / options.RateHz;
chassisStageB = chassisParams;
chassisStageB.vx_max = min(chassisStageB.vx_max, options.StageBMaxLinearSpeed);
chassisStageB.vx_min = max(chassisStageB.vx_min, -options.StageBMaxLinearSpeed);
chassisStageB.wz_max = min(chassisStageB.wz_max, options.StageBMaxAngularVelocity);
chassisStageB.reverse_enabled = false;
posTolStageB = max(options.StageBDockingPositionTolerance, 0.02);
followerOptions = struct( ...
    'SampleTime', dt, ...
    'ChassisParams', chassisStageB, ...
    'ControllerMode', "blended", ...
    'LookaheadBase', options.StageBLookaheadDistance, ...
    'LookaheadVelGain', 0.0, ...
    'LookaheadTimeGain', 0.0, ...
    'GoalTolerance', posTolStageB, ...
    'ReverseEnabled', false, ...
    'HeadingKp', getStructFieldOrDefault(chassisStageB, 'heading_kp', 1.2), ...
    'FeedforwardGain', getStructFieldOrDefault(chassisStageB, 'feedforward_gain', 0.9));

    simRes = gik9dof.control.simulateChassisController(plannerStates, ...
        'SampleTime', dt, 'FollowerOptions', followerOptions, ...
        'ControllerMode', options.StageBChassisControllerMode);
simRes = ensureDocking(simRes, goalBase, followerOptions, dt, ...
    options.StageBDockingPositionTolerance, options.StageBDockingYawTolerance, ...
    options.StageBChassisControllerMode);

baseStatesRaw = simRes.poses;
if isempty(baseStatesRaw)
    baseStatesRaw = assembleBaseStateSequence(qStart(baseIdx).', []);
end
baseStates = baseStatesRaw;
baseStates(1,:) = qStart(baseIdx).';
if any(abs(baseStates(:,1)) > 20 | abs(baseStates(:,2)) > 20) || ...
        hypot(baseStates(end,1) - goalBase(1), baseStates(end,2) - goalBase(2)) > 5
    if ~isempty(plannerStates)
        baseStates = plannerStates;
    else
        baseStates = interpolateBaseStates(qStart(baseIdx).', goalBase, max(nSamples, 2));
    end
end
if any(abs(baseStates(:,1)) > 20 | abs(baseStates(:,2)) > 20) || ...
        hypot(baseStates(end,1) - goalBase(1), baseStates(end,2) - goalBase(2)) > 5
    if ~isempty(plannerReference)
        baseStates = plannerReference;
    else
        baseStates = interpolateBaseStates(qStart(baseIdx).', goalBase, max(nSamples, 2));
    end
end

cmdLogPP = [];
if isfield(simRes, 'commands') && ~isempty(simRes.commands)
    timeCommand = (0:size(simRes.commands,1)-1)' * dt;
    cmdLogPP = [timeCommand, simRes.commands];
else
    cmdLogPP = zeros(0,3);
end

qTraj = repmat(qStart, 1, size(baseStates,1));
for k = 1:size(baseStates,1)
    qTraj(baseIdx, k) = baseStates(k, :).';
end

executedStates = baseStates(2:end, :);
eePoses = statesToEndEffectorPoses(robot, qStart, baseIdx, executedStates, eeName);
nSteps = size(executedStates,1);
timestamps = (1:nSteps) * dt;

logB = buildSyntheticStageBLog(robot, qTraj, timestamps, eePoses, options.RateHz);
logB.pathStates = baseStates;
logB.execBaseStates = baseStates;
logB.mode = "pureHyb";
logB.goalBase = goalBase;
logB.achievedBase = baseStates(end, :);

if ~isempty(cmdLogPP)
    logB.cmdLog = array2table(cmdLogPP, 'VariableNames', {'time','Vx','Wz'});
else
    logB.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
        'VariableNames', {'time','Vx','Wz'});
end

logB.purePursuit.referencePath = plannerStates;
logB.purePursuit.simulation = simRes;
logB.purePursuit.executedPath = executedStates;
logB.purePursuit.sampleTime = dt;
logB.purePursuit.chassisParams = chassisStageB;
logB.purePursuit.controllerMode = options.StageBChassisControllerMode;
logB.planner = plannerInfo;

% === PHASE 2: Enhanced Stage B Diagnostics ===
stageBDiagnostics = struct();

% Compute base ribbon metrics (curvature, cusps, smoothness)
if size(baseStates, 1) >= 3
    ribbonMetrics = gik9dof.computeBaseRibbonMetrics(baseStates);
    stageBDiagnostics.baseCurvature = ribbonMetrics.curvature;
    stageBDiagnostics.curvatureHistogram = ribbonMetrics.curvatureHistogram;
    stageBDiagnostics.cuspLocations = ribbonMetrics.cuspLocations;
    stageBDiagnostics.cuspCount = ribbonMetrics.cuspCount;
    stageBDiagnostics.pathSmoothness = ribbonMetrics.pathSmoothness;
    stageBDiagnostics.maxCurvature = ribbonMetrics.maxCurvature;
    stageBDiagnostics.meanCurvature = ribbonMetrics.meanCurvature;
else
    stageBDiagnostics.baseCurvature = [];
    stageBDiagnostics.curvatureHistogram = struct('low', 0, 'medium', 0, 'high', 0, 'veryHigh', 0);
    stageBDiagnostics.cuspLocations = [];
    stageBDiagnostics.cuspCount = 0;
    stageBDiagnostics.pathSmoothness = 0;
    stageBDiagnostics.maxCurvature = 0;
    stageBDiagnostics.meanCurvature = 0;
end

% RS smoothing acceptance metrics
if isfield(plannerInfo, 'rsSmoothing')
    rsInfo = plannerInfo.rsSmoothing;
    if isfield(rsInfo, 'improvements') && isfield(rsInfo, 'iterationsExecuted')
        stageBDiagnostics.rsAcceptanceRate = double(rsInfo.improvements) / max(double(rsInfo.iterationsExecuted), 1);
        stageBDiagnostics.rsImprovements = double(rsInfo.improvements);
        stageBDiagnostics.rsIterations = double(rsInfo.iterationsExecuted);
    else
        stageBDiagnostics.rsAcceptanceRate = 0;
        stageBDiagnostics.rsImprovements = 0;
        stageBDiagnostics.rsIterations = 0;
    end
    
    % Path length improvement
    if isfield(rsInfo, 'initialLength') && isfield(rsInfo, 'finalLength')
        stageBDiagnostics.rsPathLengthImprovement = rsInfo.initialLength - rsInfo.finalLength;
    else
        stageBDiagnostics.rsPathLengthImprovement = 0;
    end
else
    stageBDiagnostics.rsAcceptanceRate = 0;
    stageBDiagnostics.rsImprovements = 0;
    stageBDiagnostics.rsIterations = 0;
    stageBDiagnostics.rsPathLengthImprovement = 0;
end

% Clothoid smoothing success
if isfield(plannerInfo, 'hcSmoothing')
    hcInfo = plannerInfo.hcSmoothing;
    stageBDiagnostics.clothoidApplied = isfield(hcInfo, 'applied') && hcInfo.applied;
    stageBDiagnostics.clothoidSegments = getStructFieldOrDefault(hcInfo, 'fittedSegments', 0);
else
    stageBDiagnostics.clothoidApplied = false;
    stageBDiagnostics.clothoidSegments = 0;
end

% Planner compute time (if available)
stageBDiagnostics.plannerComputeTime = getStructFieldOrDefault(plannerInfo, 'computeTime', NaN);

logB.diagnostics = stageBDiagnostics;
% === End Stage B Diagnostics ===

result = struct();
result.log = logB;
result.qFinal = qTraj(:, end);
result.pathStates = baseStates;
result.goalBase = goalBase;
result.achievedBase = baseStates(end, :);
result.mode = "pureHyb";
result.cmdLog = logB.cmdLog;
result.purePursuit = logB.purePursuit;
result.planner = plannerInfo;
result.diagnostics = stageBDiagnostics;
end

function logC = executeStageCPurePursuit(robot, trajStruct, qStart, baseIdx, armIdx, velLimits, chassisParams, alignmentInfo, options, stageBResult)
%EXECUTESTAGECPUREPURSUIT Track Stage C using pure pursuit driven chassis.
eeName = trajStruct.EndEffectorName;
dt = 1 / options.RateHz;

if nargin < 8 || isempty(alignmentInfo)
    alignmentInfo = struct('applied', false);
end
if nargin < 10 || isempty(stageBResult)
    stageBResult = struct();
end

% First pass: GIK reference used as pure pursuit path seed.
bundleRef = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs, ...
    'MaxIterations', options.MaxIterations);

logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

baseReferenceTail = logRef.qTraj(baseIdx, 2:end).';
baseReference = [qStart(baseIdx).'; baseReferenceTail];

[baseReference, logRef, stageCRefinement] = stageCApplyBaseRefinement(baseReference, logRef, qStart, baseIdx, options, stageBResult, bundleRef, trajStruct, velLimits);

% Pure pursuit integration honouring chassis constraints.
chassisStageC = chassisParams;
chassisStageC.vx_max = options.StageCMaxLinearSpeed;
chassisStageC.vx_min = options.StageCMinLinearSpeed;
chassisStageC.wz_max = options.StageCMaxAngularVelocity;
chassisStageC.wheel_speed_max = options.StageCMaxWheelSpeed;
chassisStageC.reverse_enabled = options.StageCReverseEnabled;
chassisStageC.interp_spacing_min = options.StageCInterpSpacing;
chassisStageC.interp_spacing_max = options.StageCWaypointSpacing;

followerOptions = struct( ...
    'SampleTime', dt, ...
    'ChassisParams', chassisStageC, ...
    'ControllerMode', "blended", ...
    'LookaheadBase', options.StageCLookaheadDistance, ...
    'LookaheadVelGain', options.StageCLookaheadVelGain, ...
    'LookaheadTimeGain', options.StageCLookaheadTimeGain, ...
    'GoalTolerance', options.StageCGoalTolerance, ...
    'ReverseEnabled', options.StageCReverseEnabled, ...
    'HeadingKp', getStructFieldOrDefault(chassisStageC, 'heading_kp', 1.2), ...
    'FeedforwardGain', getStructFieldOrDefault(chassisStageC, 'feedforward_gain', 0.9));

simRes = gik9dof.control.simulateChassisController(baseReference, ...
    'SampleTime', dt, 'FollowerOptions', followerOptions, ...
    'ControllerMode', options.StageCChassisControllerMode);

posesPP = simRes.poses;
if ~isempty(posesPP)
    posesPP(1,:) = qStart(baseIdx).';
end
refCount = size(baseReference, 1);
if size(posesPP,1) ~= refCount && ~isempty(posesPP)
    sExec = linspace(0, 1, size(posesPP,1));
    sRef = linspace(0, 1, refCount);
    yawExec = unwrap(posesPP(:,3));
    posesInterp = zeros(refCount,3);
    posesInterp(:,1) = interp1(sExec, posesPP(:,1), sRef, 'linear');
    posesInterp(:,2) = interp1(sExec, posesPP(:,2), sRef, 'linear');
    yawInterp = interp1(sExec, yawExec, sRef, 'linear');
    posesInterp(:,3) = wrapToPi(yawInterp);
    posesPP = posesInterp;
end
executedBase = posesPP(2:end, :);
if isempty(executedBase)
    executedBase = baseReference(2:end, :);
end

% Second pass: execute GIK with chassis locked to executed path.
bundle = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs, ...
    'MaxIterations', options.MaxIterations);

fixedTrajectory = struct('Indices', baseIdx, 'Values', executedBase');
logC = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits, ...
    'FixedJointTrajectory', fixedTrajectory);

% Attach pure pursuit artefacts and command logs.
logC.purePursuit.referencePath = baseReference;
logC.purePursuit.executedPath = executedBase;
logC.purePursuit.sampleTime = dt;
logC.purePursuit.commands = simRes.commands;
logC.purePursuit.wheelSpeeds = simRes.wheelSpeeds;
logC.purePursuit.status = simRes.status;
logC.purePursuit.simulation = simRes;
logC.purePursuit.chassisParams = chassisStageC;
logC.purePursuit.controllerMode = options.StageCChassisControllerMode;
logC.purePursuit.refinement = stageCRefinement;
logC.referenceInitialIk = logRef;
logC.referenceBaseStates = baseReference;
logC.execBaseStates = posesPP;
logC.simulationMode = "ppForIk";

logC.preStageCAlignment = alignmentInfo;
logC.stageCBaseRefinement = stageCRefinement;

if ~isempty(simRes.commands)
    timeCommand = (0:size(simRes.commands,1)-1)' * dt;
    logC.cmdLog = table(timeCommand, simRes.commands(:,1), simRes.commands(:,2), ...
        'VariableNames', {'time','Vx','Wz'});
else
    logC.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
        'VariableNames', {'time','Vx','Wz'});
end

% === PHASE 2: Enhanced Stage C Diagnostics ===
stageCDiagnostics = struct();

% Solver iteration statistics per waypoint
if isfield(logC, 'exitFlags') && ~isempty(logC.exitFlags) && isfield(logC, 'iterations')
    exitFlags = logC.exitFlags;
    solverIters = logC.iterations;
    
    % Iteration histogram (bins for visualization)
    stageCDiagnostics.solverIterationsPerWaypoint = solverIters;
    stageCDiagnostics.solverIterationsMean = mean(solverIters);
    stageCDiagnostics.solverIterationsMax = max(solverIters);
    stageCDiagnostics.solverIterationsStd = std(solverIters);
    
    % Exit flag distribution
    stageCDiagnostics.exitFlagHistogram = struct();
    stageCDiagnostics.exitFlagHistogram.success = sum(exitFlags == 1);
    stageCDiagnostics.exitFlagHistogram.maxIters = sum(exitFlags == 0);
    stageCDiagnostics.exitFlagHistogram.failed = sum(exitFlags < 0);
else
    stageCDiagnostics.solverIterationsPerWaypoint = [];
    stageCDiagnostics.solverIterationsMean = 0;
    stageCDiagnostics.solverIterationsMax = 0;
    stageCDiagnostics.solverIterationsStd = 0;
    stageCDiagnostics.exitFlagHistogram = struct('success', 0, 'maxIters', 0, 'failed', 0);
end

% EE error bins: excellent <0.05m, good 0.05-0.10m, acceptable 0.10-0.20m, poor >0.20m
if isfield(logC, 'eePositions') && isfield(trajStruct, 'EndEffectorPositions')
    desiredEE = trajStruct.EndEffectorPositions;
    executedEE = logC.eePositions;
    
    % Handle dimension matching
    nDesired = size(desiredEE, 2);
    nExecuted = size(executedEE, 2);
    nCommon = min(nDesired, nExecuted);
    
    if nCommon > 0
        eeErrors = vecnorm(executedEE(:, 1:nCommon) - desiredEE(:, 1:nCommon), 2, 1);
        
        stageCDiagnostics.eeErrorPerWaypoint = eeErrors;
        stageCDiagnostics.eeErrorBins = struct();
        stageCDiagnostics.eeErrorBins.excellent = sum(eeErrors < 0.05);
        stageCDiagnostics.eeErrorBins.good = sum(eeErrors >= 0.05 & eeErrors < 0.10);
        stageCDiagnostics.eeErrorBins.acceptable = sum(eeErrors >= 0.10 & eeErrors < 0.20);
        stageCDiagnostics.eeErrorBins.poor = sum(eeErrors >= 0.20);
        stageCDiagnostics.eeErrorMean = mean(eeErrors);
        stageCDiagnostics.eeErrorMax = max(eeErrors);
    else
        stageCDiagnostics.eeErrorPerWaypoint = [];
        stageCDiagnostics.eeErrorBins = struct('excellent', 0, 'good', 0, 'acceptable', 0, 'poor', 0);
        stageCDiagnostics.eeErrorMean = 0;
        stageCDiagnostics.eeErrorMax = 0;
    end
else
    stageCDiagnostics.eeErrorPerWaypoint = [];
    stageCDiagnostics.eeErrorBins = struct('excellent', 0, 'good', 0, 'acceptable', 0, 'poor', 0);
    stageCDiagnostics.eeErrorMean = 0;
    stageCDiagnostics.eeErrorMax = 0;
end

% Base yaw drift from reference
if isfield(logC, 'referenceBaseStates') && isfield(logC, 'execBaseStates')
    refBase = logC.referenceBaseStates;
    execBase = logC.execBaseStates;
    
    if size(refBase, 1) == size(execBase, 1) && ~isempty(refBase)
        yawDrift = wrapToPi(execBase(:, 3) - refBase(:, 3));
        stageCDiagnostics.baseYawDrift = yawDrift;
        stageCDiagnostics.baseYawDriftMean = mean(abs(yawDrift));
        stageCDiagnostics.baseYawDriftMax = max(abs(yawDrift));
        
        % Position deviation
        posDeviation = vecnorm(execBase(:, 1:2) - refBase(:, 1:2), 2, 2);
        stageCDiagnostics.basePosDeviationMean = mean(posDeviation);
        stageCDiagnostics.basePosDeviationMax = max(posDeviation);
    else
        stageCDiagnostics.baseYawDrift = [];
        stageCDiagnostics.baseYawDriftMean = 0;
        stageCDiagnostics.baseYawDriftMax = 0;
        stageCDiagnostics.basePosDeviationMean = 0;
        stageCDiagnostics.basePosDeviationMax = 0;
    end
else
    stageCDiagnostics.baseYawDrift = [];
    stageCDiagnostics.baseYawDriftMean = 0;
    stageCDiagnostics.baseYawDriftMax = 0;
    stageCDiagnostics.basePosDeviationMean = 0;
    stageCDiagnostics.basePosDeviationMax = 0;
end

% Refinement status and delta metrics
stageCDiagnostics.refinementApplied = stageCRefinement.applied;
if stageCRefinement.applied
    stageCDiagnostics.refinementReason = getStructFieldOrDefault(stageCRefinement, 'reason', 'applied');
    
    % Extract delta metrics if available
    if isfield(stageCRefinement, 'delta')
        stageCDiagnostics.refinementDelta = stageCRefinement.delta;
    else
        stageCDiagnostics.refinementDelta = struct('pathLength', 0, 'eeErrorMean', 0, 'eeErrorMax', 0);
    end
else
    stageCDiagnostics.refinementReason = getStructFieldOrDefault(stageCRefinement, 'reason', 'disabled');
    stageCDiagnostics.refinementDelta = struct('pathLength', 0, 'eeErrorMean', 0, 'eeErrorMax', 0);
end

logC.diagnostics = stageCDiagnostics;
% === End Stage C Diagnostics ===
end

function logC = executeStageCPureIk(robot, trajStruct, qStart, baseIdx, velLimits, options)
%EXECUTESTAGECPUREIK Track Stage C using pure IK output only.
bundle = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs, ...
    'MaxIterations', options.MaxIterations);

logC = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);

logC.simulationMode = "pureIk";
baseStates = logC.qTraj(baseIdx, :).';
logC.execBaseStates = baseStates;
logC.referenceBaseStates = baseStates;
logC.cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
    'VariableNames', {'time','Vx','Wz'});
end

function [goalBase, goalInfo] = computeStageBGoalBase(robot, qStart, baseIdx, targetPose, eeName, maxIterations)
%COMPUTESTAGEBGOALBASE Solve for the base pose with frozen arm joints.
nJoints = numel(qStart);
armIdx = setdiff(1:nJoints, baseIdx);

if nargin < 6 || isempty(maxIterations)
    maxIterations = 1500;
end

goalBundle = gik9dof.createGikSolver(robot, "EndEffector", eeName, ...
    "MaxIterations", maxIterations);
lockJointBounds(goalBundle.constraints.joint, armIdx, qStart);
[qGoal, info] = goalBundle.solve(qStart, "TargetPose", targetPose);

exitFlag = NaN;
if isstruct(info) && isfield(info, "ExitFlag")
    exitFlag = double(info.ExitFlag);
end
if ~(isfinite(exitFlag) && exitFlag > 0)
    error("gik9dof:runStagedTrajectory:StageBGoalIK", ...
        "Failed to resolve Stage B base goal configuration (ExitFlag=%g).", exitFlag);
end

goalBase = reshape(qGoal(baseIdx), 1, []);
goalInfo = info;
end

function states = interpolateBaseStates(startState, goalState, numSamples)
%INTERPOLATEBASESTATES Linear interpolation between two base poses.
if iscolumn(startState)
    startState = startState.';
end
if iscolumn(goalState)
    goalState = goalState.';
end

numSamples = max(2, round(numSamples));
alpha = linspace(0, 1, numSamples).';

yawDelta = wrapToPi(goalState(3) - startState(3));
yawInterp = wrapToPi(startState(3) + yawDelta * alpha);
xInterp = startState(1) + (goalState(1) - startState(1)) * alpha;
yInterp = startState(2) + (goalState(2) - startState(2)) * alpha;

states = [xInterp, yInterp, yawInterp];
end

function seq = assembleBaseStateSequence(startState, statesOut)
%ASSEMBLEBASESTATESEQUENCE Include start pose ahead of executed states.
if iscolumn(startState)
    startState = startState.';
end
if isempty(statesOut)
    seq = startState;
else
    seq = [startState; statesOut]; %#ok<AGROW>
end
end

function alignment = generateStageCAlignmentInfo(stageBResult, stageBStartBase, options, chassisParams, sampleTime)
alignment = struct('applied', false, 'states', [], 'commands', [], 'wheelSpeeds', [], ...
    'status', [], 'sampleTime', sampleTime, 'start', [], 'goal', [], ...
    'posErrorStart', 0, 'yawErrorStart', 0, 'fallbackUsed', false);

if ~isfield(stageBResult, 'goalBase') || isempty(stageBResult.goalBase)
    goalBase = stageBStartBase;
else
    goalBase = stageBResult.goalBase;
end

startBase = stageBStartBase;

posErr = hypot(goalBase(1) - startBase(1), goalBase(2) - startBase(2));
yawErr = abs(wrapToPi(goalBase(3) - startBase(3)));
alignment.posErrorStart = posErr;
alignment.yawErrorStart = yawErr;
alignment.start = startBase;
alignment.goal = goalBase;

posTol = max(options.StageCGoalTolerance, 0.05);
yawTol = 5 * pi / 180;

if isfield(stageBResult, 'achievedBase') && ~isempty(stageBResult.achievedBase)
    achievedBase = stageBResult.achievedBase;
    posErrAchieved = hypot(goalBase(1) - achievedBase(1), goalBase(2) - achievedBase(2));
    yawErrAchieved = abs(wrapToPi(goalBase(3) - achievedBase(3)));
else
    posErrAchieved = posErr;
    yawErrAchieved = yawErr;
end

if posErrAchieved <= posTol && yawErrAchieved <= yawTol
    return
end

if posErr <= posTol && yawErr <= yawTol
    return
end
alignment.fallbackUsed = true;
numSamples = max(ceil(posErr / 0.05) + 2, 5);
alignmentPath = interpolateBaseStates(startBase, goalBase, numSamples);

alignment.applied = true;
alignment.states = alignmentPath;
alignment.sampleTime = sampleTime;
alignment.path = alignmentPath;
alignment.positionTolerance = posTol;
alignment.yawTolerance = yawTol;

% Derive simple command approximations for logging.
if size(alignmentPath,1) >= 2
    dt = sampleTime;
    deltas = diff(alignmentPath);
    vx = hypot(deltas(:,1), deltas(:,2)) / dt;
    yawRate = deltas(:,3) / dt;
    alignment.commands = [vx, yawRate];
    alignment.wheelSpeeds = zeros(size(alignment.commands));
    alignment.status = repmat(struct('isFinished', false, 'controllerMode', "alignment"), size(alignment.commands,1), 1);
else
    alignment.commands = zeros(0,2);
    alignment.wheelSpeeds = zeros(0,2);
    alignment.status = struct([]);
end
end

function stageBResult = applyStageCAlignment(stageBResult, baseIdx, alignment)
if ~alignment.applied || isempty(alignment.states) || size(alignment.states,1) < 2
    return
end

alignmentTail = alignment.states(2:end, :);
% Update path states
stageBResult.pathStates = [stageBResult.pathStates; alignmentTail];
stageBResult.achievedBase = alignment.goal;
stageBResult.goalBase = alignment.goal;

% Update final configuration
qFinal = stageBResult.qFinal;
qFinal(baseIdx) = alignment.goal(:);
stageBResult.qFinal = qFinal;

% Update log
logB = stageBResult.log;
if isfield(logB, 'pathStates')
    logB.pathStates = stageBResult.pathStates;
end
if isfield(logB, 'execBaseStates')
    logB.execBaseStates = stageBResult.pathStates;
end

% Append command log
if ~isempty(alignment.commands)
    timeVec = (0:size(alignment.commands,1)-1)' * alignment.sampleTime;
    alignTable = array2table([timeVec, alignment.commands], 'VariableNames', {'time','Vx','Wz'});
    if isfield(logB, 'cmdLog') && ~isempty(logB.cmdLog)
        logB.cmdLog = [logB.cmdLog; alignTable];
    else
        logB.cmdLog = alignTable;
    end
end

% Record alignment info
logB.alignment = alignment;
if isfield(logB, 'purePursuit') && isstruct(logB.purePursuit)
    logB.purePursuit.alignment = alignment;
end
if isfield(logB, 'goalBase')
    logB.goalBase = alignment.goal;
end
if isfield(logB, 'achievedBase')
    logB.achievedBase = alignment.goal;
end
stageBResult.log = logB;
stageBResult.cmdLog = logB.cmdLog;
stageBResult.alignment = alignment;
stageBResult.achievedBase = alignment.goal;
stageBResult.goalBase = alignment.goal;
end

function log = buildSyntheticStageBLog(robot, qTraj, timestamps, eePoses, rateHz)
%BUILDSYNTHETICSTAGEBLOG Construct a log struct compatible with Stage B.
numSteps = numel(timestamps);
log = emptyLog(robot, qTraj(:, end), rateHz);

log.qTraj = qTraj;
log.rateHz = rateHz;
log.timestamps = timestamps(:)';
log.time = [0, log.timestamps];
log.successMask = true(1, numSteps);
log.completedWaypoints = numSteps;
log.solutionInfo = repmat({struct('ExitFlag', NaN, 'Status', 'pureHyb')}, 1, numSteps);
log.exitFlags = nan(1, numSteps);
log.iterations = nan(1, numSteps);
log.constraintViolationMax = nan(1, numSteps);

log.targetPoses = eePoses;
log.eePoses = eePoses;
if numSteps > 0
    log.targetPositions = reshape(eePoses(1:3,4,:), 3, numSteps);
    orientations = zeros(4, numSteps);
    for idx = 1:numSteps
        orientations(:, idx) = tform2quat(eePoses(:,:,idx)).';
    end
    log.targetOrientations = orientations;
    log.eePositions = log.targetPositions;
    log.eeOrientations = orientations;
else
    log.targetPositions = zeros(3,0);
    log.targetOrientations = zeros(4,0);
    log.eePositions = zeros(3,0);
    log.eeOrientations = zeros(4,0);
end
log.positionError = zeros(3, numSteps);
log.positionErrorNorm = zeros(1, numSteps);
log.orientationErrorQuat = zeros(4, numSteps);
log.orientationErrorAngle = zeros(1, numSteps);
end

function checkStageBDocking(goalBase, achievedBase, posTol, yawTol, stageBMode)
%CHECKSTAGEBDOCKING Issue warnings when docking tolerance violated.
if isempty(goalBase) || isempty(achievedBase)
    return
end

posError = hypot(goalBase(1) - achievedBase(1), goalBase(2) - achievedBase(2));
yawError = wrapToPi(goalBase(3) - achievedBase(3));

triggerPos = posTol > 0 && posError > posTol;
triggerYaw = yawTol > 0 && abs(yawError) > yawTol;

if triggerPos || triggerYaw
    warning('gik9dof:runStagedTrajectory:StageBDockingTolerance', ...
        'Stage B (%s) docking error exceeds tolerance (pos %.3fm, yaw %.2fdeg).', ...
        stageBMode, posError, rad2deg(yawError));
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

function [states, goalBase, plannerInfo] = planStageBHybridAStarPath(qStart, baseIdx, targetPose, options, robot, eeName, floorDiscs)
%PLANSTAGEBHYBRIDASTARPATH Plan a base trajectory using Hybrid A*.
startBase = reshape(qStart(baseIdx), 1, []);
[goalBase, ~] = computeStageBGoalBase(robot, qStart, baseIdx, targetPose, eeName, options.MaxIterations);

plannerInfo = struct( ...
    'statesRaw', [], ...
    'statesDensified', [], ...
    'statesRefined', [], ...
    'statesClothoid', [], ...
    'rsMetrics', struct(), ...
    'rsSmoothing', struct(), ...
    'rsParamsBase', struct(), ...
    'rsParamsApplied', struct(), ...
    'rsParamsFinal', struct(), ...
    'hcSmoothing', struct(), ...
    'hcParamsApplied', struct());

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
plannerInfo.statesRaw = statesRaw;

rateHz = options.RateHz;
if isempty(rateHz) || ~isscalar(rateHz) || rateHz <= 0
    rateHz = 100;
end
maxLinearStep = options.StageBMaxLinearSpeed / rateHz;
maxYawStep = options.StageBMaxYawRate / rateHz;

statesDensified = densifyHybridStates(statesRaw, maxLinearStep, maxYawStep);
plannerInfo.statesDensified = statesDensified;

rsInfo = struct();
statesRefined = statesDensified;

rsDefaultsCommon = gik9dof.control.defaultReedsSheppParams();
rsParamsBase = mergeStructs(rsDefaultsCommon, struct( ...
    'Rmin', options.StageBHybridMinTurningRadius, ...
    'inflationRadius', options.StageBHybridSafetyMargin));
if isfield(options, "StageBReedsSheppParams") && ~isempty(options.StageBReedsSheppParams)
    rsParamsBase = mergeStructs(rsParamsBase, options.StageBReedsSheppParams);
end

rsBaseLambdaCusp = getStructFieldOrDefault(rsParamsBase, 'lambdaCusp', 1.0);
plannerInfo.rsParamsBase = rsParamsBase;
plannerInfo.rsParamsApplied = struct();
plannerInfo.rsParamsFinal = rsParamsBase;

useReedsShepp = isfield(options, "StageBUseReedsShepp") && options.StageBUseReedsShepp;

if useReedsShepp
    rsParams = rsParamsBase;
    if rsParams.Rmin <= 0
        rsParams.Rmin = options.StageBHybridMinTurningRadius;
    end
    if rsParams.inflationRadius < 0
        rsParams.inflationRadius = options.StageBHybridSafetyMargin;
    end

    mapData = occupancyMatrix(occMap);
    [statesRS, rsInfo] = gik9dof.control.rsRefinePath(mapData, occMap.Resolution, statesDensified, rsParams);
    statesRefined = densifyHybridStates(statesRS, maxLinearStep, maxYawStep);
    plannerInfo.rsParamsApplied = rsParams;
    plannerInfo.rsParamsFinal = struct( ...
        'Rmin', rsParams.Rmin, ...
        'reverseCost', getStructFieldOrDefault(rsParams, 'reverseCost', getStructFieldOrDefault(rsParamsBase, 'reverseCost', 2.0)), ...
        'lambdaCusp', getStructFieldOrDefault(rsParams, 'lambdaCusp', rsBaseLambdaCusp));

    if isfield(options, "Verbose") && options.Verbose
        improvements = getStructFieldOrDefault(rsInfo, 'improvements', int32(0));
        executed = getStructFieldOrDefault(rsInfo, 'iterationsExecuted', int32(0));
        fprintf("[Stage B][RS] shortcuts accepted %d/%d (requested %d)\n", ...
            double(improvements), double(executed), double(getStructFieldOrDefault(rsInfo, 'iterationsPlanned', int32(0))));
    end
else
    plannerInfo.rsParamsApplied = struct();
    plannerInfo.rsParamsFinal = rsParamsBase;
end

connMetrics = reedsSheppConnection('MinTurningRadius', max(plannerInfo.rsParamsFinal.Rmin, eps));
connMetrics.ReverseCost = plannerInfo.rsParamsFinal.reverseCost;

metricsRaw = computeReedsSheppMetrics(connMetrics, statesRaw, plannerInfo.rsParamsFinal.lambdaCusp);
metricsDensified = computeReedsSheppMetrics(connMetrics, statesDensified, plannerInfo.rsParamsFinal.lambdaCusp);
metricsRefined = computeReedsSheppMetrics(connMetrics, statesRefined, plannerInfo.rsParamsFinal.lambdaCusp);

statesClothoid = statesRefined;
pathLengthRefinedEuclid = sum(vecnorm(diff(statesRefined(:,1:2).'), 2, 1));
hcInfo = struct( ...
    'applied', false, ...
    'segmentCount', int32(0), ...
    'fittedSegments', int32(0), ...
    'gearChanges', int32(0), ...
    'pathLengthOriginal', pathLengthRefinedEuclid, ...
    'pathLengthSmoothed', pathLengthRefinedEuclid, ...
    'maxCurvature', 0.0, ...
    'discretizationDistance', 0.0, ...
    'segmentBounds', zeros(0,2), ...
    'params', struct());
plannerInfo.hcParamsApplied = struct();

useClothoid = isfield(options, "StageBUseClothoid") && options.StageBUseClothoid;
if useClothoid
hcDefaults = struct( ...
    'discretizationDistance', 0.03, ...
    'maxNumWaypoints', int32(0));
    hcParamsUser = struct();
    if isfield(options, "StageBClothoidParams") && ~isempty(options.StageBClothoidParams)
        hcParamsUser = options.StageBClothoidParams;
    end
    hcParams = mergeStructs(hcDefaults, hcParamsUser);
    [statesHC, hcInfo] = gik9dof.control.rsClothoidRefine(statesRefined, hcParams);
    if ~isempty(statesHC)
        statesClothoid = densifyHybridStates(statesHC, maxLinearStep, maxYawStep);
    end
    plannerInfo.hcParamsApplied = hcParams;
end

metricsClothoid = computeReedsSheppMetrics(connMetrics, statesClothoid, plannerInfo.rsParamsFinal.lambdaCusp);

plannerInfo.rsMetrics = struct('raw', metricsRaw, 'densified', metricsDensified, 'refined', metricsRefined, 'clothoid', metricsClothoid);
plannerInfo.statesRefined = statesRefined;
plannerInfo.statesClothoid = statesClothoid;
plannerInfo.rsSmoothing = rsInfo;
plannerInfo.hcSmoothing = hcInfo;

if useReedsShepp && isfield(options, "Verbose") && options.Verbose
    deltaLength = metricsRefined.length - metricsDensified.length;
    fprintf("[Stage B][RS] length %.2fm -> %.2fm (Δ%.2fm), cusps %d -> %d\n", ...
        metricsDensified.length, metricsRefined.length, deltaLength, ...
        metricsDensified.cusps, metricsRefined.cusps);
end
if useClothoid && isfield(options, "Verbose") && options.Verbose
    deltaLengthHC = metricsClothoid.length - metricsRefined.length;
    fprintf("[Stage B][HC] length %.2fm -> %.2fm (Δ%.2fm)\n", ...
        metricsRefined.length, metricsClothoid.length, deltaLengthHC);
end

states = statesClothoid;
end

function statesOut = smoothStageBWithReedsShepp(statesIn, minTurningRadius, numSamples)
if isempty(statesIn) || size(statesIn,1) < 2
    statesOut = statesIn;
    return
end

conn = reedsSheppConnection('MinTurningRadius', max(minTurningRadius, eps));
statesOut = statesIn(1,:);
for idx = 1:size(statesIn,1)-1
    q1 = statesIn(idx,:);
    q2 = statesIn(idx+1,:);
    try
        [pathSeg, ~] = connect(conn, q1, q2);
    catch
        pathSeg = {};
    end
    if isempty(pathSeg)
        statesOut(end+1,:) = q2; %#ok<AGROW>
        continue
    end
    segmentStates = discretizeReedsSheppSegment(pathSeg{1}, numSamples);
    statesOut = [statesOut; segmentStates(2:end,:)]; %#ok<AGROW>
end

totalDistance = sum(vecnorm(diff(statesOut(:,1:2).'),2,1));
straightDist = norm(statesIn(end,1:2) - statesIn(1,1:2));
if totalDistance > max(10*straightDist, straightDist + 10)
    statesOut = statesIn;
end
end

function segStates = discretizeReedsSheppSegment(segment, numSamples)
if nargin < 2 || numSamples <= 0
    numSamples = 50;
end

totalLen = sum(segment.MotionLengths);
if totalLen <= 0
    segStates = [segment.StartPose; segment.GoalPose];
    return
end

pose = segment.StartPose;
segStates = pose;

for i = 1:numel(segment.MotionLengths)
    length_i = segment.MotionLengths(i);
    if length_i <= 0
        continue
    end
    typeChar = segment.MotionTypes(i);
    if isa(typeChar, "string") || isa(typeChar, "char")
        type = char(typeChar);
    else
        type = char(typeChar);
    end
    type = type(1);
    direction = segment.MotionDirections(i);
    numSteps = max(ceil(numSamples * (length_i / totalLen)), 1);
    ds = length_i / numSteps;
    for stepIdx = 1:numSteps
        switch type
            case 'S'
                step = direction * ds;
                pose(1) = pose(1) + step * cos(pose(3));
                pose(2) = pose(2) + step * sin(pose(3));
            case {'L','R'}
                if type == 'L'
                    curvature = direction / segment.MinTurningRadius;
                else
                    curvature = -direction / segment.MinTurningRadius;
                end
                deltaTheta = curvature * ds;
                theta0 = pose(3);
                theta1 = wrapToPi(theta0 + deltaTheta);
                if abs(curvature) < 1e-9
                    pose(1) = pose(1) + direction * ds * cos(theta0);
                    pose(2) = pose(2) + direction * ds * sin(theta0);
                    pose(3) = theta1;
                else
                    pose(1) = pose(1) + (sin(theta1) - sin(theta0)) / curvature;
                    pose(2) = pose(2) - (cos(theta1) - cos(theta0)) / curvature;
                    pose(3) = theta1;
                end
            otherwise
                step = direction * ds;
                pose(1) = pose(1) + step * cos(pose(3));
                pose(2) = pose(2) + step * sin(pose(3));
        end
        segStates(end+1,:) = pose; %#ok<AGROW>
    end
end

segStates(end,:) = segment.GoalPose;
end

function simResOut = ensureDocking(simResIn, goalBase, followerOptions, sampleTime, posTol, yawTol, controllerMode)
simResOut = simResIn;
if nargin < 5 || isempty(posTol) || ~isfinite(posTol)
    posTol = 0.05;
end
if nargin < 6 || isempty(yawTol) || ~isfinite(yawTol)
    yawTol = 5 * pi/180;
end
if nargin < 7 || isempty(controllerMode)
    controllerMode = 2;
end

if ~isfield(simResIn, 'poses') || isempty(simResIn.poses)
    return
end

finalPose = simResIn.poses(end, :);
posError = hypot(finalPose(1) - goalBase(1), finalPose(2) - goalBase(2));
yawError = abs(wrapToPi(finalPose(3) - goalBase(3)));
if posError <= max(posTol, 1e-3) && yawError <= max(yawTol, 1e-3)
    return
end

alignPath = [finalPose; goalBase];
alignRes = gik9dof.control.simulateChassisController(alignPath, ...
    'SampleTime', sampleTime, 'FollowerOptions', followerOptions, ...
    'ControllerMode', controllerMode);

if ~isfield(alignRes, 'poses') || size(alignRes.poses,1) < 2
    return
end

trimIdx = 2:size(alignRes.poses,1);
simResOut.poses = [simResIn.poses; alignRes.poses(trimIdx,:)];

if isfield(simResIn, 'commands') && isfield(alignRes, 'commands') && ~isempty(alignRes.commands)
    simResOut.commands = [simResIn.commands; alignRes.commands];
end
if isfield(simResIn, 'wheelSpeeds') && isfield(alignRes, 'wheelSpeeds') && ~isempty(alignRes.wheelSpeeds)
    simResOut.wheelSpeeds = [simResIn.wheelSpeeds; alignRes.wheelSpeeds];
end
if isfield(simResIn, 'status') && isfield(alignRes, 'status') && ~isempty(alignRes.status)
    simResOut.status = [simResIn.status; alignRes.status];
end

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

function metrics = computeReedsSheppMetrics(conn, pathStates, lambdaCusp)
%COMPUTEREEDSSHEPPMETRICS Evaluate length and cusp cost for a pose sequence.
 %#ok<INUSD> (conn retained for compatibility)
if nargin < 3 || isempty(lambdaCusp)
    lambdaCusp = 0.0;
end

metrics = struct('length', 0.0, 'cusps', 0, 'cost', 0.0, 'segments', int32(0));

if isempty(pathStates) || size(pathStates,1) < 2
    return
end

diffXY = diff(pathStates(:,1:2),1,1);
segmentLengths = sqrt(sum(diffXY.^2, 2));
lengthSum = sum(segmentLengths);

dirs = estimatePathDirectionsLocal(pathStates);
cuspSum = countDirectionChangesLocal(dirs);

metrics.length = lengthSum;
metrics.cusps = cuspSum;
metrics.cost = lengthSum + lambdaCusp * double(cuspSum);
metrics.segments = int32(numel(segmentLengths));
end

function seg = bestReedsSheppConnection(conn, poseA, poseB)
%BESTREEDSSHEPPCONNECTION Safely select the shortest RS connection.
seg = [];
try
    [segments, costs] = connect(conn, poseA, poseB);
catch
    segments = {};
    costs = [];
end
if isempty(segments)
    return
end
[~, idx] = min(costs);
seg = segments{idx};
end

function c = countReedsSheppCusps(seg)
%COUNTREEDSSHEPPCUSPS Count gear changes in an RS segment.
dirs = seg.MotionDirections(:).';
types = seg.MotionTypes;
validCount = 0;
for t = 1:numel(types)
    if types{t} ~= "N"
        validCount = validCount + 1;
    end
end
dirs = dirs(1:max(1, validCount));
if numel(dirs) < 2
    c = 0;
else
    c = sum(abs(diff(sign(dirs))) > 0);
end
end

function value = getStructFieldOrDefault(s, fieldName, defaultValue)
%GETSTRUCTFIELDORDEFAULT Return struct field value or default when missing.
if isempty(s) || ~isstruct(s) || ~isfield(s, fieldName)
    value = defaultValue;
    return
end

fieldValue = s.(fieldName);
if isempty(fieldValue)
    value = defaultValue;
else
    value = fieldValue;
end
end

function dirs = estimatePathDirectionsLocal(pathStates)
nSteps = size(pathStates,1) - 1;
dirs = zeros(nSteps,1);
for idx = 1:nSteps
    theta = pathStates(idx,3);
    delta = pathStates(idx+1,1:2) - pathStates(idx,1:2);
    projection = delta(1) * cos(theta) + delta(2) * sin(theta);
    if abs(projection) < 1e-6
        dirs(idx) = 0;
    else
        dirs(idx) = sign(projection);
    end
end

for idx = 2:nSteps
    if dirs(idx) == 0
        dirs(idx) = dirs(idx-1);
    end
end
for idx = nSteps-1:-1:1
    if dirs(idx) == 0
        dirs(idx) = dirs(idx+1);
    end
end
dirs(dirs == 0) = 1;
end

function count = countDirectionChangesLocal(directions)
if isempty(directions)
    count = 0;
    return
end
signs = sign(directions(:).');
signs(signs == 0) = 1;
count = sum(abs(diff(signs)) > 0);
end

function [pathOut, logRefOut, info] = stageCApplyBaseRefinement(basePath, logRefIn, qStart, baseIdx, options, stageBResult, bundleRef, trajStruct, velLimits)
%STAGECAPPLYBASEREFINEMENT Smooth Stage C base ribbon against obstacles.
pathOut = basePath;
logRefOut = logRefIn;

info = struct('applied', false, 'lengthBefore', computePathLength(basePath), ...
    'lengthAfter', computePathLength(basePath), 'rs', struct(), 'hc', struct(), ...
    'controllerMode', getStructFieldOrDefault(options, 'StageCChassisControllerMode', 2));

if ~isfield(options, 'StageCUseBaseRefinement') || ~options.StageCUseBaseRefinement
    return
end

if size(basePath,1) < 2
    return
end

resolution = options.StageBHybridResolution;
if ~isfinite(resolution) || resolution <= 0
    resolution = 0.05;
end
safetyMargin = options.StageBHybridSafetyMargin;
if ~isfinite(safetyMargin)
    safetyMargin = 0.1;
end

floorDiscs = [];
if isfield(options, 'FloorDiscs') && ~isempty(options.FloorDiscs)
    floorDiscs = options.FloorDiscs;
elseif isfield(stageBResult, 'log') && isfield(stageBResult.log, 'floorDiscs') && ~isempty(stageBResult.log.floorDiscs)
    floorDiscs = stageBResult.log.floorDiscs;
end

occMap = buildStageBOccupancyMap(basePath(1,:), basePath(end,:), floorDiscs, resolution, safetyMargin);
mapData = occupancyMatrix(occMap);

rateHz = options.RateHz;
if isempty(rateHz) || rateHz <= 0
    rateHz = 100;
end
maxLinearStep = options.StageCMaxLinearSpeed / rateHz;
maxYawStep = options.StageCMaxAngularVelocity / rateHz;

statesSeed = densifyHybridStates(basePath, maxLinearStep, maxYawStep);

rsParams = gik9dof.control.defaultReedsSheppParams();
if isfield(options, 'StageBReedsSheppParams') && ~isempty(options.StageBReedsSheppParams)
    rsParams = mergeStructs(rsParams, options.StageBReedsSheppParams);
end
rsParams.Rmin = options.StageBHybridMinTurningRadius;
rsParams.inflationRadius = options.StageBHybridSafetyMargin;
rsParams.allowReverse = getStructFieldOrDefault(rsParams, 'allowReverse', true);

[statesRS, rsInfo] = gik9dof.control.rsRefinePath(mapData, occMap.Resolution, statesSeed, rsParams);
statesRS = densifyHybridStates(statesRS, maxLinearStep, maxYawStep);

statesHC = statesRS;
hcInfo = struct('applied', false);
if isfield(options, 'StageBUseClothoid') && options.StageBUseClothoid
    hcParams = struct('discretizationDistance', 0.03);
    if isfield(options, 'StageBClothoidParams') && ~isempty(options.StageBClothoidParams)
        hcParams = mergeStructs(hcParams, options.StageBClothoidParams);
    end
    [statesHCtmp, hcInfoTmp] = gik9dof.control.rsClothoidRefine(statesRS, hcParams);
    if ~isempty(statesHCtmp)
        statesHC = densifyHybridStates(statesHCtmp, maxLinearStep, maxYawStep);
        hcInfo = hcInfoTmp;
    end
end

resampled = resamplePoseSequence(statesHC, size(basePath,1));
resampled(1,:) = basePath(1,:);
resampled(end,:) = basePath(end,:);

if max(abs(resampled - basePath), [], 'all') < 5e-4
    info.rs = rsInfo;
    info.hc = hcInfo;
    return
end

info.applied = true;
info.rs = rsInfo;
info.hc = hcInfo;
info.lengthBefore = computePathLength(basePath);
info.lengthAfter = computePathLength(resampled);

pathOut = resampled;

fixedRef = struct('Indices', baseIdx, 'Values', pathOut(2:end,:)');
logRefOut = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits, ...
    'FixedJointTrajectory', fixedRef);

baseTail = logRefOut.qTraj(baseIdx, 2:end).';
pathOut = [qStart(baseIdx).'; baseTail];
info.lengthAfter = computePathLength(pathOut);
info.controllerMode = getStructFieldOrDefault(options, 'StageCChassisControllerMode', 2);
end

function lengthVal = computePathLength(pathStates)
if size(pathStates,1) < 2
    lengthVal = 0.0;
    return
end
diffXY = diff(pathStates(:,1:2),1,1);
lengthVal = sum(sqrt(sum(diffXY.^2,2)));
end

function poses = resamplePoseSequence(pathStates, targetCount)
if nargin < 2 || targetCount <= 0
    poses = pathStates;
    return
end
if size(pathStates,1) <= 1
    poses = repmat(pathStates, max(1,targetCount), 1);
    return
end

diffXY = diff(pathStates(:,1:2),1,1);
segmentLengths = sqrt(sum(diffXY.^2,2));
arc = [0; cumsum(segmentLengths)];
totalLength = arc(end);
if totalLength <= eps
    poses = repmat(pathStates(1,:), targetCount, 1);
    return
end

sSamples = linspace(0, totalLength, targetCount)';
poses = zeros(targetCount, 3);
poses(:,1) = interp1(arc, pathStates(:,1), sSamples, 'linear');
poses(:,2) = interp1(arc, pathStates(:,2), sSamples, 'linear');
yaw = unwrap(pathStates(:,3));
poses(:,3) = wrapToPi(interp1(arc, yaw, sSamples, 'linear'));
end

function modeOut = resolveChassisControllerMode(modeIn, chassisParams, fieldName)
%RESOLVECHASSISCONTROLLERMODE Resolve sentinel controller mode using profile.
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

function out = mergeStructs(a, b)
%MERGESTRUCTS Simple shallow merge where fields in b override fields in a.
if nargin < 1 || isempty(a)
    a = struct();
end
if nargin < 2 || isempty(b)
    out = a;
    return
end
out = a;
fieldsB = fieldnames(b);
for idx = 1:numel(fieldsB)
    key = fieldsB{idx};
    out.(key) = b.(key);
end
end

function options = applyPipelineConfigToStaged(options, config)
%APPLYPIPELINECONFIGTOSTAGED Map unified PipelineConfig to runStagedTrajectory options.
%   Similar to trackReferenceTrajectory but for the staged execution pipeline.

% Helper to set parameter only if using default value
    function setParam(paramName, configValue)
        % Simple approach: always apply config unless explicitly checking
        % In practice, MATLAB arguments block handles defaults, so this
        % just overwrites with config values
        if ~isnan(configValue) && ~isempty(configValue)
            options.(paramName) = configValue;
        end
    end

% Map Stage B parameters
if isfield(config, 'stage_b')
    sb = config.stage_b;
    if isfield(sb, 'mode'), options.StageBMode = string(sb.mode); end
    if isfield(sb, 'lookahead_distance'), options.StageBLookaheadDistance = sb.lookahead_distance; end
    if isfield(sb, 'desired_linear_velocity'), options.StageBDesiredLinearVelocity = sb.desired_linear_velocity; end
    if isfield(sb, 'max_linear_speed'), options.StageBMaxLinearSpeed = sb.max_linear_speed; end
    if isfield(sb, 'max_angular_velocity'), options.StageBMaxAngularVelocity = sb.max_angular_velocity; end
    if isfield(sb, 'max_yaw_rate'), options.StageBMaxYawRate = sb.max_yaw_rate; end
    if isfield(sb, 'max_joint_speed'), options.StageBMaxJointSpeed = sb.max_joint_speed; end
    if isfield(sb, 'docking_position_tolerance'), options.StageBDockingPositionTolerance = sb.docking_position_tolerance; end
    if isfield(sb, 'docking_yaw_tolerance'), options.StageBDockingYawTolerance = sb.docking_yaw_tolerance; end
    if isfield(sb, 'use_hybrid_astar'), options.UseStageBHybridAStar = sb.use_hybrid_astar; end
    if isfield(sb, 'hybrid_resolution'), options.StageBHybridResolution = sb.hybrid_resolution; end
    if isfield(sb, 'hybrid_safety_margin'), options.StageBHybridSafetyMargin = sb.hybrid_safety_margin; end
    if isfield(sb, 'hybrid_min_turning_radius'), options.StageBHybridMinTurningRadius = sb.hybrid_min_turning_radius; end
    if isfield(sb, 'hybrid_motion_primitive_length'), options.StageBHybridMotionPrimitiveLength = sb.hybrid_motion_primitive_length; end
    if isfield(sb, 'use_reeds_shepp'), options.StageBUseReedsShepp = sb.use_reeds_shepp; end
    if isfield(sb, 'reeds_shepp_params'), options.StageBReedsSheppParams = sb.reeds_shepp_params; end
    if isfield(sb, 'use_clothoid'), options.StageBUseClothoid = sb.use_clothoid; end
    if isfield(sb, 'clothoid_params'), options.StageBClothoidParams = sb.clothoid_params; end
    if isfield(sb, 'chassis_controller_mode'), options.StageBChassisControllerMode = sb.chassis_controller_mode; end
end

% Map Stage C parameters
if isfield(config, 'stage_c')
    sc = config.stage_c;
    if isfield(sc, 'lookahead_distance'), options.StageCLookaheadDistance = sc.lookahead_distance; end
    if isfield(sc, 'lookahead_vel_gain'), options.StageCLookaheadVelGain = sc.lookahead_vel_gain; end
    if isfield(sc, 'lookahead_time_gain'), options.StageCLookaheadTimeGain = sc.lookahead_time_gain; end
    if isfield(sc, 'desired_linear_velocity'), options.StageCDesiredLinearVelocity = sc.desired_linear_velocity; end
    if isfield(sc, 'max_linear_speed'), options.StageCMaxLinearSpeed = sc.max_linear_speed; end
    if isfield(sc, 'min_linear_speed'), options.StageCMinLinearSpeed = sc.min_linear_speed; end
    if isfield(sc, 'max_angular_velocity'), options.StageCMaxAngularVelocity = sc.max_angular_velocity; end
    if isfield(sc, 'track_width'), options.StageCTrackWidth = sc.track_width; end
    if isfield(sc, 'wheel_base'), options.StageCWheelBase = sc.wheel_base; end
    if isfield(sc, 'max_wheel_speed'), options.StageCMaxWheelSpeed = sc.max_wheel_speed; end
    if isfield(sc, 'waypoint_spacing'), options.StageCWaypointSpacing = sc.waypoint_spacing; end
    if isfield(sc, 'path_buffer_size'), options.StageCPathBufferSize = sc.path_buffer_size; end
    if isfield(sc, 'goal_tolerance'), options.StageCGoalTolerance = sc.goal_tolerance; end
    if isfield(sc, 'interp_spacing'), options.StageCInterpSpacing = sc.interp_spacing; end
    if isfield(sc, 'reverse_enabled'), options.StageCReverseEnabled = sc.reverse_enabled; end
    if isfield(sc, 'use_base_refinement'), options.StageCUseBaseRefinement = sc.use_base_refinement; end
    if isfield(sc, 'chassis_controller_mode'), options.StageCChassisControllerMode = sc.chassis_controller_mode; end
end

% Map GIK parameters
if isfield(config, 'gik') && isfield(config.gik, 'max_iterations')
    options.MaxIterations = config.gik.max_iterations;
end

% Map Chassis parameters
if isfield(config, 'chassis')
    options.ChassisOverrides = mergeStructs(options.ChassisOverrides, config.chassis);
end

end

