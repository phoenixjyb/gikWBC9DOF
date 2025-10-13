function log = runStageCPPFirst(robot, trajStruct, q_start, options)
%RUNSTAGECPPFIRST Stage C execution using PP-First method (Method 4).
%   log = GIK9DOF.RUNSTAGECPPFIRST(robot, trajStruct, q_start) executes
%   Stage C trajectory tracking using the PP-First architecture:
%     1. PREDICT: Pure Pursuit generates base motion (guaranteed diff-drive feasible)
%     2. CONSTRAIN: GIK constrained with yaw corridor around PP prediction
%     3. SOLVE: GIK solves for full-body configuration
%     4. FALLBACK: If EE error exceeds threshold, fix base and solve arm-only
%
%   log = GIK9DOF.RUNSTAGECPPFIRST(..., options) supports:
%       ChassisParams        - Chassis parameters struct (required)
%       MaxIterations        - GIK max iterations (default 1500)
%       YawTolerance         - Yaw corridor half-width (rad, default deg2rad(15))
%       PositionTolerance    - Position box half-width (m, default 0.15)
%       EEErrorTolerance     - Fallback threshold (m, default 0.01)
%       SampleTime           - Control timestep (s, default 0.1)
%       DesiredVelocity      - Target velocity (m/s, default 0.5)
%       LookaheadDistance    - PP lookahead (m, default 0.6)
%       ApplyRefinement      - Apply RS+Clothoid to seed (default false)
%       BaseIndices          - Base joint indices (default [1 2 3])
%       ArmIndices           - Arm joint indices (default [4 5 6 7 8 9])
%       EndEffector          - EE body name (default 'left_gripper_link')
%       DistanceSpecs        - Obstacle constraints (default [])
%       VerboseLevel         - 0=quiet, 1=summary, 2=detailed (default 1)
%
%   Returns:
%       log                  - Comprehensive log struct with:
%           qTraj            - [9xN] joint trajectory
%           timestamps       - [1xN] time stamps
%           eePositions      - [3xN] actual EE positions
%           targetPositions  - [3xN] desired EE positions
%           positionError    - [3xN] tracking error vectors
%           positionErrorNorm- [1xN] error magnitudes
%           ppCommands       - [Nx2] PP commands [vx, wz]
%           basePredicted    - [3xN] PP base predictions
%           baseActual       - [3xN] actual base from GIK
%           fallbackUsed     - [1xN] logical, true if fallback triggered
%           gikIterations    - [1xN] GIK iterations per waypoint
%           solveTime        - [1xN] solve time per waypoint
%           successMask      - [1xN] GIK convergence flags
%           baseSeedPath     - [Nx3] initial base path from baseSeedFromEE
%           ppFollower       - Pure Pursuit controller object
%           ppPathInfo       - Preprocessed path info
%           fallbackRate     - Scalar, fraction of waypoints using fallback
%           avgEEError       - Scalar, mean EE tracking error (m)
%           maxEEError       - Scalar, max EE tracking error (m)
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       traj = gik9dof.loadJsonTrajectory('1_pull_world_scaled.json');
%       chassisParams = gik9dof.control.loadChassisProfile('wide_track');
%       q0 = homeConfiguration(robot);
%       log = gik9dof.runStageCPPFirst(robot, traj, q0, 'ChassisParams', chassisParams);
%
%   See also runStagedTrajectory, baseSeedFromEE, initPPFromBasePath, solveArmOnlyIK.

arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    q_start (:,1) double
    options.ChassisParams (1,1) struct
    options.MaxIterations (1,1) double = 1500
    options.YawTolerance (1,1) double = deg2rad(15)
    options.PositionTolerance (1,1) double = 0.15
    options.EEErrorTolerance (1,1) double = 0.01
    options.SampleTime (1,1) double = 0.1
    options.DesiredVelocity (1,1) double = 0.5
    options.LookaheadDistance (1,1) double = 0.6
    options.ApplyRefinement (1,1) logical = false
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
    options.EndEffector (1,1) string = "left_gripper_link"
    options.DistanceSpecs = struct([])
    options.VerboseLevel (1,1) double = 1
end

verbose = options.VerboseLevel > 0;
detailedVerbose = options.VerboseLevel > 1;

if verbose
    fprintf('\n=== Stage C: PP-First Method (Method 4) ===\n');
end

%% Step 1: Generate base seed path from EE trajectory
if verbose
    fprintf('Step 1: Generating base seed path from EE waypoints...\n');
end

baseSeedPath = gik9dof.baseSeedFromEE(robot, trajStruct, q_start, ...
    'EndEffector', options.EndEffector, ...
    'BaseIndices', options.BaseIndices, ...
    'ArmIndices', options.ArmIndices, ...
    'Verbose', detailedVerbose);

if verbose
    fprintf('  Base seed path: %d waypoints, %.2f m total length\n', ...
        size(baseSeedPath, 1), sum(vecnorm(diff(baseSeedPath(:,1:2), 1, 1), 2, 2)));
end

%% Step 2: Initialize Pure Pursuit controller
if verbose
    fprintf('Step 2: Initializing Pure Pursuit controller...\n');
end

[ppFollower, ppPathInfo] = gik9dof.initPPFromBasePath(baseSeedPath, options.ChassisParams, ...
    'LookaheadBase', options.LookaheadDistance, ...
    'DesiredVelocity', options.DesiredVelocity, ...
    'SampleTime', options.SampleTime, ...
    'ApplyRefinement', options.ApplyRefinement, ...
    'Verbose', detailedVerbose);

if verbose
    fprintf('  PP initialized: lookahead %.2f m, desired velocity %.2f m/s\n', ...
        options.LookaheadDistance, options.DesiredVelocity);
end

%% Step 3: Create GIK solver
if verbose
    fprintf('Step 3: Creating GIK solver with constraints...\n');
end

gikBundle = gik9dof.createGikSolver(robot, ...
    'EndEffector', options.EndEffector, ...
    'MaxIterations', options.MaxIterations, ...
    'DistanceSpecs', options.DistanceSpecs);

%% Step 4: Main control loop
nWaypoints = size(trajStruct.Poses, 3);
dt = options.SampleTime;

% Preallocate log arrays
log = struct();
log.qTraj = zeros(9, nWaypoints);
log.timestamps = zeros(1, nWaypoints);
log.eePositions = zeros(3, nWaypoints);
log.targetPositions = zeros(3, nWaypoints);
log.positionError = zeros(3, nWaypoints);
log.positionErrorNorm = zeros(1, nWaypoints);
log.ppCommands = zeros(nWaypoints, 2); % [vx, wz]
log.basePredicted = zeros(3, nWaypoints);
log.baseActual = zeros(3, nWaypoints);
log.fallbackUsed = false(1, nWaypoints);
log.gikIterations = zeros(1, nWaypoints);
log.solveTime = zeros(1, nWaypoints);
log.successMask = false(1, nWaypoints);
log.solutionInfo = cell(1, nWaypoints);

q_current = q_start;
t_current = 0;

if verbose
    fprintf('Step 4: Executing control loop for %d waypoints...\n', nWaypoints);
    fprintf('  Yaw corridor: ±%.1f deg, Position tolerance: ±%.2f m\n', ...
        rad2deg(options.YawTolerance), options.PositionTolerance);
    fprintf('  EE error threshold for fallback: %.1f mm\n', options.EEErrorTolerance * 1000);
end

for k = 1:nWaypoints
    tic;
    
    % Get current pose
    pose_current = q_current(options.BaseIndices);
    
    %% PREDICT: Pure Pursuit step
    [vx_cmd, wz_cmd, ppStatus] = ppFollower.step(pose_current', dt);
    
    % Integrate to predict next base pose
    x_pp = pose_current(1) + vx_cmd * cos(pose_current(3)) * dt;
    y_pp = pose_current(2) + vx_cmd * sin(pose_current(3)) * dt;
    theta_pp = pose_current(3) + wz_cmd * dt;
    theta_pp = wrapToPi(theta_pp);
    
    q_base_pred = [x_pp; y_pp; theta_pp];
    
    %% CONSTRAIN: Update GIK with yaw corridor
    gik9dof.updateBaseJointBounds(gikBundle, options.BaseIndices, q_base_pred, ...
        options.YawTolerance, options.PositionTolerance);
    
    %% SOLVE: GIK with constrained base
    T_ee_target = trajStruct.Poses(:, :, k);
    
    [q_gik, solInfo] = gikBundle.solve(q_current, 'TargetPose', T_ee_target);
    
    % Compute EE error
    T_ee_actual = getTransform(robot, q_gik, char(options.EndEffector));
    p_ee_actual = T_ee_actual(1:3, 4);
    p_ee_target = T_ee_target(1:3, 4);
    ee_error_vec = p_ee_actual - p_ee_target;
    ee_error_norm = norm(ee_error_vec);
    
    %% CHECK: EE error and fallback if needed
    if ee_error_norm > options.EEErrorTolerance
        % Fallback: Fix base at predicted pose, solve arm-only
        [q_arm_fallback, solInfoFallback] = gik9dof.solveArmOnlyIK(robot, gikBundle, ...
            T_ee_target, q_base_pred, q_current(options.ArmIndices), ...
            'BaseIndices', options.BaseIndices, 'ArmIndices', options.ArmIndices);
        
        q_final = [q_base_pred; q_arm_fallback];
        log.fallbackUsed(k) = true;
        
        % Recompute EE position with fallback solution
        T_ee_actual = getTransform(robot, q_final, char(options.EndEffector));
        p_ee_actual = T_ee_actual(1:3, 4);
        ee_error_vec = p_ee_actual - p_ee_target;
        ee_error_norm = norm(ee_error_vec);
        
        solInfo = solInfoFallback;
        
        if detailedVerbose
            fprintf('  [%3d] Fallback triggered (GIK error %.3f > %.3f m)\n', ...
                k, ee_error_norm, options.EEErrorTolerance);
        end
    else
        q_final = q_gik;
    end
    
    %% UPDATE state
    q_current = q_final;
    t_current = t_current + dt;
    
    %% LOG results
    log.qTraj(:, k) = q_final;
    log.timestamps(k) = t_current;
    log.eePositions(:, k) = p_ee_actual;
    log.targetPositions(:, k) = p_ee_target;
    log.positionError(:, k) = ee_error_vec;
    log.positionErrorNorm(k) = ee_error_norm;
    log.ppCommands(k, :) = [vx_cmd, wz_cmd];
    log.basePredicted(:, k) = q_base_pred;
    log.baseActual(:, k) = q_final(options.BaseIndices);
    log.gikIterations(k) = solInfo.Iterations;
    log.solveTime(k) = toc;
    log.successMask(k) = solInfo.Status == "success";
    log.solutionInfo{k} = solInfo;
    
    % Progress indicator
    if verbose && mod(k, 20) == 0
        fprintf('  Progress: %d/%d waypoints (%.1f%%), Fallback rate: %.1f%%\n', ...
            k, nWaypoints, 100*k/nWaypoints, 100*sum(log.fallbackUsed(1:k))/k);
    end
end

%% Finalize log
log.baseSeedPath = baseSeedPath;
log.ppFollower = ppFollower;
log.ppPathInfo = ppPathInfo;
log.fallbackRate = sum(log.fallbackUsed) / nWaypoints;
log.avgEEError = mean(log.positionErrorNorm);
log.maxEEError = max(log.positionErrorNorm);
log.meanSolveTime = mean(log.solveTime);
log.totalTime = sum(log.solveTime);

% Method 4 metadata
log.mode = 'ppFirst';
log.parameters = struct( ...
    'yawTolerance', options.YawTolerance, ...
    'positionTolerance', options.PositionTolerance, ...
    'eeErrorTolerance', options.EEErrorTolerance, ...
    'lookaheadDistance', options.LookaheadDistance, ...
    'desiredVelocity', options.DesiredVelocity, ...
    'maxIterations', options.MaxIterations);

if verbose
    fprintf('\n=== Stage C Complete ===\n');
    fprintf('Waypoints processed: %d\n', nWaypoints);
    fprintf('Fallback rate: %.1f%% (%d/%d waypoints)\n', ...
        100*log.fallbackRate, sum(log.fallbackUsed), nWaypoints);
    fprintf('EE tracking error: mean %.2f mm, max %.2f mm\n', ...
        log.avgEEError * 1000, log.maxEEError * 1000);
    fprintf('Solve time: mean %.3f s/waypoint, total %.2f s\n', ...
        log.meanSolveTime, log.totalTime);
    fprintf('GIK convergence rate: %.1f%%\n', 100*sum(log.successMask)/nWaypoints);
end

end
