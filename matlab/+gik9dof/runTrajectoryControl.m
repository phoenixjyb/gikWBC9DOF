function log = runTrajectoryControl(bundle, trajectory, options)
%RUNTRAJECTORYCONTROL Execute a velocity-level control loop using GIK.
%   log = GIK9DOF.RUNTRAJECTORYCONTROL(bundle, trajectory) streams waypoint
%   targets through the provided solver bundle and collects diagnostic data
%   for analysis and plotting. The control loop runs at a fixed rate using
%   MATLAB's rateControl utility.
%
%   Required inputs:
%       bundle     - Struct produced by gik9dof.createGikSolver.
%       trajectory - Struct with field Poses (4x4xN) describing desired
%                    end-effector poses in SE(3). Optional fields:
%                       AimTargets        (3xN) world-frame points for
%                                          constraintAiming, if enabled.
%                       DistanceBounds    (2xN) per-step lower/upper bounds.
%                       DistanceReference (1xN string/cellstr) bodies to
%                                          reference in constraintDistanceBounds.
%
%   Name-value options:
%       InitialConfiguration - Initial joint configuration (defaults to the
%                              bundle home position). Accepts row/column
%                              vector or struct matching robot.DataFormat.
%       RateHz               - Loop rate in Hz (default 100).
%       CommandFcn           - Function handle invoked each iteration as
%                              CommandFcn(q, info, k).
%       StopOnFailure        - If true (default) the loop exits when GIK
%                              reports failure. When false the previous
%                              configuration is retained and execution
%                              continues.
%       Verbose              - If true print per-step summaries (default
%                              false).
%
%   The returned log struct contains:
%       qTraj          - Joint configurations (nJoints x N+1) including the
%                        initial configuration.
%       solutionInfo   - Cell array of solver diagnostics per step.
%       timestamps     - Time stamps relative to loop start (1xN).
%       successMask    - Logical vector signalling solver convergence.
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       bundle = gik9dof.createGikSolver(robot);
%       traj.Poses = repmat(eye(4),1,1,100);
%       log = gik9dof.runTrajectoryControl(bundle, traj);
%
%   See also rateControl, generalizedInverseKinematics.

arguments
    bundle (1,1) struct
    trajectory (1,1) struct
    options.InitialConfiguration = []
    options.RateHz (1,1) double {mustBePositive} = 100
    options.CommandFcn = []
    options.StopOnFailure (1,1) logical = true
    options.Verbose (1,1) logical = false
end

poses = trajectory.Poses;
validateattributes(poses, {'double'}, {'size', [4 4 NaN], 'finite', 'real'}, mfilename, 'trajectory.Poses');
numWaypoints = size(poses, 3);

commandFcn = options.CommandFcn;
if ~isempty(commandFcn) && ~isa(commandFcn, 'function_handle')
    error("gik9dof:runTrajectoryControl:InvalidCommandFcn", ...
        "CommandFcn must be a function handle or empty.");
end

% Optional trajectory overlays.
aimTargets = [];
if isfield(trajectory, "AimTargets") && ~isempty(trajectory.AimTargets)
    aimTargets = trajectory.AimTargets;
    if size(aimTargets, 2) ~= numWaypoints
        error("gik9dof:runTrajectoryControl:AimSizeMismatch", ...
            "AimTargets must have one column per waypoint.");
    end
end

distBoundsSeq = [];
if isfield(trajectory, "DistanceBounds") && ~isempty(trajectory.DistanceBounds)
    distBoundsSeq = trajectory.DistanceBounds;
    if size(distBoundsSeq, 2) ~= numWaypoints
        error("gik9dof:runTrajectoryControl:DistanceSizeMismatch", ...
            "DistanceBounds must have one column per waypoint.");
    end
end

distRefSeq = [];
if isfield(trajectory, "DistanceReference") && ~isempty(trajectory.DistanceReference)
    distRefSeq = trajectory.DistanceReference;
    if numel(distRefSeq) ~= numWaypoints
        error("gik9dof:runTrajectoryControl:ReferenceSizeMismatch", ...
            "DistanceReference must supply one entry per waypoint.");
    end
end

% Prepare initial configuration.
if isempty(options.InitialConfiguration)
    q = bundle.tools.home();
else
    q = bundle.tools.column(options.InitialConfiguration);
end

nJoints = numel(q);
qLog = zeros(nJoints, numWaypoints + 1);
qLog(:,1) = q;

solutionInfo = cell(1, numWaypoints);
successMask = false(1, numWaypoints);
timestamps = zeros(1, numWaypoints);
exitFlags = nan(1, numWaypoints);
iterations = nan(1, numWaypoints);
constraintViolationMax = nan(1, numWaypoints);

targetPositions = reshape(poses(1:3,4,:), 3, numWaypoints);
targetOrientations = nan(4, numWaypoints);
eePoses = repmat(eye(4), 1, 1, numWaypoints);
eePositions = nan(3, numWaypoints);
eeOrientations = nan(4, numWaypoints);
orientationErrorQuat = nan(4, numWaypoints);
orientationErrorAngle = nan(1, numWaypoints);
eeName = char(bundle.constraints.pose.EndEffector);

loopClock = rateControl(options.RateHz);
startTime = tic;

for k = 1:numWaypoints
    currentPose = poses(:,:,k);
    solveArgs = {"TargetPose", currentPose};

    if ~isempty(aimTargets)
        solveArgs(end+1:end+2) = {"AimTarget", aimTargets(:,k).' }; %#ok<AGROW>
    end

    if ~isempty(distBoundsSeq)
        solveArgs(end+1:end+2) = {"DistanceBounds", distBoundsSeq(:,k).' }; %#ok<AGROW>
    end

    if ~isempty(distRefSeq)
        solveArgs(end+1:end+2) = {"DistanceReference", distRefSeq(k)}; %#ok<AGROW>
    end

    try
        [qCandidate, stepInfo] = bundle.solve(q, solveArgs{:});
    catch solverErr
        if options.StopOnFailure
            rethrow(solverErr);
        else
            warning("gik9dof:runTrajectoryControl:SolverError", ...
                "Iteration %d failed: %s", k, solverErr.message);
            qCandidate = q;
            stepInfo = struct("ExitFlag", -1, "Status", "error");
        end
    end

    success = isSuccess(stepInfo);
    exitFlag = fetchExitFlag(stepInfo);
    exitFlags(k) = exitFlag;
    if isfield(stepInfo, 'Iterations') && ~isempty(stepInfo.Iterations)
        iterations(k) = double(stepInfo.Iterations);
    end
    if isfield(stepInfo, 'ConstraintViolations') && ~isempty(stepInfo.ConstraintViolations)
        try
            violations = [stepInfo.ConstraintViolations.Violation]; %#ok<NASGU>
            constraintViolationMax(k) = max([stepInfo.ConstraintViolations.Violation]);
        catch
            % Leave NaN if structure is unexpected.
        end
    end
    if ~success && options.StopOnFailure
        warning("gik9dof:runTrajectoryControl:Infeasible", ...
            "Solver reported failure at waypoint %d (ExitFlag=%d).", k, exitFlag);
        q = qCandidate;
        qLog(:,k+1) = q;
        solutionInfo{k} = stepInfo;
        successMask(k) = success;
        timestamps(k) = toc(startTime);
        eePoses(:,:,k) = getTransform(bundle.solver.RigidBodyTree, q, eeName);
        eePositions(:,k) = eePoses(1:3,4,k);
        [targetOrientations(:,k), eeOrientations(:,k), orientationErrorQuat(:,k), orientationErrorAngle(k)] = ...
            computeOrientationError(currentPose, eePoses(:,:,k));
        break
    end

    q = qCandidate;
    qLog(:,k+1) = q;
    solutionInfo{k} = stepInfo;
    successMask(k) = success;
    timestamps(k) = toc(startTime);
    eePoses(:,:,k) = getTransform(bundle.solver.RigidBodyTree, q, eeName);
    eePositions(:,k) = eePoses(1:3,4,k);
    [targetOrientations(:,k), eeOrientations(:,k), orientationErrorQuat(:,k), orientationErrorAngle(k)] = ...
        computeOrientationError(currentPose, eePoses(:,:,k));

    if ~isempty(commandFcn)
        commandFcn(q, stepInfo, k);
    end

    if options.Verbose
        fprintf("[GIK] step %d/%d exit=%g\n", k, numWaypoints, exitFlag);
    end

    waitfor(loopClock);
end

log.qTraj = qLog;
log.solutionInfo = solutionInfo;
log.successMask = successMask;
log.timestamps = timestamps;
log.rateHz = options.RateHz;
log.completedWaypoints = find(successMask, 1, "last");
if isempty(log.completedWaypoints)
    log.completedWaypoints = 0;
end
log.exitFlags = exitFlags;
log.iterations = iterations;
log.constraintViolationMax = constraintViolationMax;
log.targetPoses = poses;
log.targetPositions = targetPositions;
log.targetOrientations = targetOrientations;
log.eePoses = eePoses;
log.eePositions = eePositions;
log.eeOrientations = eeOrientations;
log.positionError = eePositions - targetPositions;
log.positionErrorNorm = vecnorm(log.positionError, 2, 1);
log.time = [0, timestamps];
log.orientationErrorQuat = orientationErrorQuat;
log.orientationErrorAngle = orientationErrorAngle;
end

function [targetQuat, eeQuat, errQuat, errAngle] = computeOrientationError(targetPose, eePose)
targetQuat = normalizeQuat(tform2quat(targetPose).');
eeQuat = normalizeQuat(tform2quat(eePose).');
errQuat = normalizeQuat(quatMultiply(quatInverse(targetQuat), eeQuat));
w = max(min(errQuat(1), 1), -1);
errAngle = 2 * acos(w);
end

function q = normalizeQuat(q)
normVal = norm(q);
if normVal < eps
    q = [1; 0; 0; 0];
else
    q = q / normVal;
end
end

function qInv = quatInverse(q)
qInv = [q(1); -q(2:4)];
end

function q = quatMultiply(q1, q2)
w1 = q1(1); v1 = q1(2:4);
w2 = q2(1); v2 = q2(2:4);
w = w1*w2 - dot(v1, v2);
v = w1*v2 + w2*v1 + cross(v1, v2);
q = [w; v];
end

function tf = isSuccess(info)
if isempty(info)
    tf = false;
    return
end
if isfield(info, "ExitFlag")
    tf = info.ExitFlag > 0;
    return
end
if isfield(info, "Status")
    tf = any(strcmpi(info.Status, {"success", "converged"}));
    return
end
% Conservative default.
tf = true;
end

function flag = fetchExitFlag(info)
if isfield(info, "ExitFlag")
    flag = info.ExitFlag;
else
    flag = NaN;
end
end
