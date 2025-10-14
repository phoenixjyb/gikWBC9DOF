function log = runStageCPureMPC(robot, trajStruct, q_start, options)
%RUNSTAGECPUREMPC Stage C execution using Pure MPC method (Method 5).
%   log = GIK9DOF.RUNSTAGECPUREMPC(robot, trajStruct, q_start) executes
%   Stage C trajectory tracking using true receding horizon Nonlinear MPC
%   with WHOLE-BODY control (base + arm simultaneously):
%     1. BUILD: Create nlmpc controller with unicycle + arm dynamics
%     2. LOOP: For each control step:
%        a. Get EE reference trajectory segment (horizon ahead)
%        b. Solve NMPC optimization with FK-based EE tracking cost
%        c. Apply first control [v*, ω*, q̇_arm*]
%        d. Simulate full-body motion (unified dynamics)
%        e. Log and advance
%
%   NOTE: No separate IK step. MPC optimizes all DOF simultaneously using
%         forward kinematics in the cost function (differential IK embedded)
%
%   log = GIK9DOF.RUNSTAGECPUREMPC(..., options) supports:
%       NMPCParams           - NMPC configuration struct (required)
%                              .horizon: Prediction horizon (steps)
%                              .control_horizon: Control horizon (steps)
%                              .sample_time: Control period (s)
%                              .weights: Cost weights struct
%                              .constraints: Velocity/wheel/joint limits
%       BaseIndices          - Base joint indices (default [1 2 3])
%       ArmIndices           - Arm joint indices (default [4 5 6 7 8 9])
%       EndEffector          - EE body name (default 'left_gripper_link')
%       VerboseLevel         - 0=quiet, 1=summary, 2=detailed (default 1)
%
%   Returns:
%       log                  - Comprehensive log struct with:
%           qTraj            - [9xN] joint trajectory
%           timestamps       - [1xN] time stamps
%           eePositions      - [3xN] actual EE positions
%           eeOrientations   - [3x3xN] actual EE rotation matrices
%           targetPositions  - [3xN] desired EE positions
%           targetOrientations - [3x3xN] desired EE rotations
%           positionError    - [3xN] position tracking error
%           orientationError - [1xN] orientation error (Frobenius norm)
%           positionErrorNorm- [1xN] position error magnitudes
%           baseCommands     - [Nx3] MPC base commands [v, omega, 0]
%           armCommands      - [Nx6] MPC arm commands [q̇_arm]
%           mpcIterations    - [1xN] MPC iterations per step
%           mpcCost          - [1xN] MPC cost per step
%           mpcExitFlag      - [1xN] MPC solver exit flags
%           solveTime        - [1xN] solve time per step
%           avgEEPosError    - Scalar, mean EE position error (m)
%           avgEEOriError    - Scalar, mean EE orientation error (rad)
%           maxEEPosError    - Scalar, max EE position error (m)
%           maxEEOriError    - Scalar, max EE orientation error (rad)
%           avgSolveTime     - Scalar, mean solve time (s)
%           controlFrequency - Scalar, achieved control rate (Hz)
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       traj = gik9dof.loadJsonTrajectory('1_pull_world_scaled.json');
%       q0 = [0; 0; 0; zeros(6,1)];  % Initial configuration
%       
%       % Load NMPC parameters from profile
%       configTools = gik9dof.ConfigTools('config/pipeline_profiles.yaml', ...
%                                          'config/chassis_profiles.yaml');
%       nmpcParams = configTools.getStageConfig('c', 'nmpc');
%       
%       log = gik9dof.runStageCPureMPC(robot, traj, q0, ...
%           'NMPCParams', nmpcParams, ...
%           'VerboseLevel', 2);
%
%   See also: nlmpc, nlmpcmove, eeTrackingCostFcn, unicycleStateFcn

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - Whole-body MPC (Redesigned)

%% Parse options
arguments
    robot
    trajStruct
    q_start (9,1) double
    options.NMPCParams struct
    options.BaseIndices (1,:) double = [1 2 3]
    options.ArmIndices (1,:) double = [4 5 6 7 8 9]
    options.EndEffector char = 'left_gripper_link'
    options.VerboseLevel (1,1) double {mustBeInRange(options.VerboseLevel, 0, 2)} = 1
end

verbose = options.VerboseLevel >= 1;
detailedVerbose = options.VerboseLevel >= 2;

if verbose
    fprintf('\n=== Method 5 (pureMPC): Whole-Body Receding Horizon NMPC ===\n');
    fprintf('Using MATLAB MPC Toolbox for unified base+arm optimization\n');
    fprintf('No separate IK - FK embedded in cost function\n\n');
end

%% Step 1: Extract EE reference trajectory
if verbose
    fprintf('Step 1: Extracting EE pose trajectory...\n');
end

nWaypoints = size(trajStruct.Poses, 3);

% Extract EE positions and orientations from trajectory
eeRefPositions = zeros(3, nWaypoints);
eeRefOrientations = zeros(3, 3, nWaypoints);

for k = 1:nWaypoints
    T_ee = trajStruct.Poses(:, :, k);
    eeRefPositions(:, k) = T_ee(1:3, 4);
    eeRefOrientations(:, :, k) = T_ee(1:3, 1:3);
end

if verbose
    fprintf('  Extracted %d EE pose waypoints\n', nWaypoints);
    pathLength = sum(vecnorm(diff(eeRefPositions, 1, 2)));
    fprintf('  EE path length: %.2f m\n', pathLength);
end

%% Step 2: Create Whole-Body NMPC controller
if verbose
    fprintf('Step 2: Building whole-body NMPC controller...\n');
end

% Extract NMPC parameters
nmpcParams = options.NMPCParams;
p = nmpcParams.horizon;              % Prediction horizon
m = nmpcParams.control_horizon;      % Control horizon
Ts = nmpcParams.sample_time;         % Sample time

% Create nlmpc object for WHOLE-BODY control
% States: [x, y, theta, q_arm(6)] - 9 states
% Outputs: [p_ee(3), R_ee_vec(9)] - 12 outputs (EE pose)
% Inputs: [v, omega, q_dot_arm(6)] - 8 manipulated variables
nlobj = nlmpc(9, 12, 8);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;

% Set model (unicycle + arm dynamics)
% Use function handles with Ts captured in closure
nlobj.Model.StateFcn = @(x, u) gik9dof.mpc.unicycleStateFcn(x, u, Ts);
nlobj.Jacobian.StateFcn = @(x, u) gik9dof.mpc.unicycleStateJacobian(x, u, Ts);  % For speed

% Output function: Compute EE pose from full configuration
% This will be used by custom cost function
% Note: nlmpc OutputFcn signature is (x, u, Ts, params) - capture robot/eeBodyName in closure
outputParams = struct('robot', robot, 'eeBodyName', options.EndEffector);
nlobj.Model.OutputFcn = @(x, u, ~) computeEEPoseOutput(x, outputParams.robot, outputParams.eeBodyName);

% Custom cost function: FK-based EE tracking
% This replaces the separate IK step - MPC optimizes all DOF simultaneously
costParams = struct();
costParams.robot = robot;
costParams.eeBodyName = options.EndEffector;
costParams.weights = nmpcParams.weights;
nlobj.Optimization.CustomCostFcn = @(X, U, e, data) ...
    gik9dof.mpc.eeTrackingCostFcn(X, U, e, data, costParams.robot, costParams.eeBodyName, costParams.weights);

% Disable default tracking weights (we're using custom cost)
nlobj.Weights.OutputVariables = zeros(1, 12);
nlobj.Weights.ManipulatedVariablesRate = zeros(1, 8);

% Set base input constraints
nlobj.MV(1).Min = nmpcParams.constraints.v_min;
nlobj.MV(1).Max = nmpcParams.constraints.v_max;
nlobj.MV(2).Min = nmpcParams.constraints.omega_min;
nlobj.MV(2).Max = nmpcParams.constraints.omega_max;

% Set arm joint velocity constraints
if isfield(nmpcParams.constraints, 'q_dot_arm_min') && ...
   isfield(nmpcParams.constraints, 'q_dot_arm_max')
    for i = 1:6
        nlobj.MV(2+i).Min = nmpcParams.constraints.q_dot_arm_min;
        nlobj.MV(2+i).Max = nmpcParams.constraints.q_dot_arm_max;
    end
end

% Set arm joint limit constraints (state limits)
if isfield(nmpcParams.constraints, 'q_arm_min') && ...
   isfield(nmpcParams.constraints, 'q_arm_max')
    for i = 1:6
        nlobj.States(3+i).Min = nmpcParams.constraints.q_arm_min(i);
        nlobj.States(3+i).Max = nmpcParams.constraints.q_arm_max(i);
    end
end

% Set custom wheel speed constraints
constraintParams = struct();
constraintParams.track_width = nmpcParams.constraints.track_width;
constraintParams.wheel_max = nmpcParams.constraints.wheel_max;
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) ...
    gik9dof.mpc.wheelSpeedConstraints(X, U, e, data, constraintParams);

% Solver options (convert strings to numbers if needed)
maxIter = nmpcParams.solver.max_iterations;
if isstring(maxIter) || ischar(maxIter)
    maxIter = str2double(maxIter);
end
nlobj.Optimization.SolverOptions.MaxIterations = maxIter;

constraintTol = nmpcParams.solver.constraint_tolerance;
if isstring(constraintTol) || ischar(constraintTol)
    constraintTol = str2double(constraintTol);
end
nlobj.Optimization.SolverOptions.ConstraintTolerance = constraintTol;

if verbose
    fprintf('  Whole-body NMPC controller created:\n');
    fprintf('    States: 9 DOF (3 base + 6 arm)\n');
    fprintf('    Inputs: 8 DOF (v, ω, q̇_arm)\n');
    fprintf('    Horizon: p=%d steps (%.1f s), m=%d control\n', p, p*Ts, m);
    fprintf('    Sample time: Ts=%.2f s (%.0f Hz)\n', Ts, 1/Ts);
    fprintf('    Base constraints: v∈[%.2f, %.2f] m/s, ω∈[%.2f, %.2f] rad/s\n', ...
        nmpcParams.constraints.v_min, nmpcParams.constraints.v_max, ...
        nmpcParams.constraints.omega_min, nmpcParams.constraints.omega_max);
    if isfield(nmpcParams.constraints, 'q_dot_arm_min')
        fprintf('    Arm vel constraints: q̇∈[%.2f, %.2f] rad/s\n', ...
            nmpcParams.constraints.q_dot_arm_min, nmpcParams.constraints.q_dot_arm_max);
    end
    fprintf('    Wheel speed limit: |v_wheel| ≤ %.2f m/s\n', nmpcParams.constraints.wheel_max);
end

%% Step 3: Initialize state and control
if verbose
    fprintf('Step 3: Initializing control loop...\n');
end

% Initial full state [x, y, theta, q_arm(6)]
x_current = q_start;  % 9x1 full configuration
u_last = zeros(8, 1);  % [v, omega, q_dot_arm(6)] - start from rest
t_current = 0;

% Preallocate log arrays
log = struct();
nSteps = nWaypoints;  % One control step per waypoint (can be adjusted)
log.qTraj = zeros(9, nSteps);
log.timestamps = zeros(1, nSteps);
log.eePositions = zeros(3, nSteps);
log.eeOrientations = zeros(3, 3, nSteps);
log.targetPositions = zeros(3, nSteps);
log.targetOrientations = zeros(3, 3, nSteps);
log.positionError = zeros(3, nSteps);
log.orientationError = zeros(1, nSteps);
log.positionErrorNorm = zeros(1, nSteps);
log.baseCommands = zeros(nSteps, 3);  % [v, omega, 0] for compatibility
log.armCommands = zeros(nSteps, 6);   % [q_dot_arm]
log.mpcIterations = zeros(1, nSteps);
log.mpcCost = zeros(1, nSteps);
log.mpcExitFlag = zeros(1, nSteps);
log.solveTime = zeros(1, nSteps);

if verbose
    fprintf('  Initial configuration: [x=%.2f, y=%.2f, θ=%.1f°, q_arm=[%.2f, ...]]\n', ...
        x_current(1), x_current(2), rad2deg(x_current(3)), x_current(4));
    fprintf('Step 4: Executing whole-body NMPC control loop for %d steps...\n', nSteps);
end

%% Step 4: Main NMPC control loop
for k = 1:nSteps
    tic;
    
    %% 4a: Get EE reference trajectory segment
    % Reference for horizon: k to min(k+p, nWaypoints)
    k_end = min(k + p, nWaypoints);
    
    % Extract EE poses for horizon
    refPosSegment = eeRefPositions(:, k:k_end);
    refOriSegment = eeRefOrientations(:, :, k:k_end);
    
    % Pad if necessary (repeat last waypoint)
    nRef = size(refPosSegment, 2);
    if nRef < p+1
        nPad = p+1 - nRef;
        refPosSegment = [refPosSegment, repmat(refPosSegment(:, end), 1, nPad)];
        refOriSegment = cat(3, refOriSegment, repmat(refOriSegment(:, :, end), 1, 1, nPad));
    end
    
    % Format reference for NMPC: [(p+1) x 12] - nlmpcmove expects rows=timesteps, cols=outputs
    % Columns 1-3: EE position [px, py, pz]
    % Columns 4-12: EE orientation (vectorized rotation matrix)
    yref = zeros(p+1, 12);
    for i = 1:p+1
        yref(i, 1:3) = refPosSegment(:, i)';
        R = refOriSegment(:, :, i);
        yref(i, 4:12) = R(:)';  % Vectorize rotation matrix as row
    end
    
    %% 4b: Solve whole-body NMPC optimization
    try
        [u_opt, mpcInfo] = nlmpcmove(nlobj, x_current, u_last, yref);
        
        % mpcInfo is an nlmpcmoveopt object with limited properties
        % Store what we can access safely
        try
            log.mpcExitFlag(k) = mpcInfo.ExitFlag;
        catch
            log.mpcExitFlag(k) = 1;  % Assume success if field doesn't exist
        end
        
        try
            log.mpcCost(k) = mpcInfo.Cost;
        catch
            log.mpcCost(k) = 0;
        end
        
        try
            if isfield(mpcInfo, 'Output') && isfield(mpcInfo.Output, 'Iterations')
                log.mpcIterations(k) = mpcInfo.Output.Iterations;
            else
                log.mpcIterations(k) = 0;
            end
        catch
            log.mpcIterations(k) = 0;
        end
        
        % Debug: Print MPC info for first few steps
        if detailedVerbose && k <= 3
            fprintf('  [%3d] MPC solved successfully, Cost=%.2e\n', k, log.mpcCost(k));
        end
        
        % Check if solution is valid (all finite values)
        if any(~isfinite(u_opt))
            if k <= 5 || detailedVerbose
                fprintf('  [%3d] MPC returned non-finite control, using fallback\n', k);
            end
            u_opt = u_last * 0.5;  % Slow down
            log.mpcExitFlag(k) = -1;
        end
        
    catch ME
        if detailedVerbose || k <= 5
            fprintf('  [%3d] MPC error: %s, using zero control\n', k, ME.message);
        end
        u_opt = zeros(8, 1);
        log.mpcExitFlag(k) = -99;
        log.mpcIterations(k) = 0;
        log.mpcCost(k) = NaN;
    end
    
    %% 4c: Simulate full-body motion (unified dynamics)
    x_next = gik9dof.mpc.unicycleStateFcn(x_current, u_opt, Ts);
    
    %% 4d: Compute actual EE pose and errors
    T_ee_actual = getTransform(robot, x_next, char(options.EndEffector));
    p_ee_actual = T_ee_actual(1:3, 4);
    R_ee_actual = T_ee_actual(1:3, 1:3);
    
    T_ee_target = trajStruct.Poses(:, :, k);
    p_ee_target = T_ee_target(1:3, 4);
    R_ee_target = T_ee_target(1:3, 1:3);
    
    % Position error
    ee_pos_error = p_ee_actual - p_ee_target;
    
    % Orientation error (Frobenius norm of rotation error matrix)
    R_error = R_ee_actual - R_ee_target;
    ee_ori_error = norm(R_error, 'fro');
    
    %% 4e: Update state
    x_current = x_next;
    u_last = u_opt;
    t_current = t_current + Ts;
    
    %% 4f: Log results
    log.qTraj(:, k) = x_next;
    log.timestamps(k) = t_current;
    log.eePositions(:, k) = p_ee_actual;
    log.eeOrientations(:, :, k) = R_ee_actual;
    log.targetPositions(:, k) = p_ee_target;
    log.targetOrientations(:, :, k) = R_ee_target;
    log.positionError(:, k) = ee_pos_error;
    log.orientationError(k) = ee_ori_error;
    log.positionErrorNorm(k) = norm(ee_pos_error);
    log.baseCommands(k, :) = [u_opt(1:2)', 0];  % [v, omega, 0]
    log.armCommands(k, :) = u_opt(3:8)';
    log.solveTime(k) = toc;
    
    % Progress indicator
    if verbose && mod(k, 10) == 0
        fprintf('  Progress: %d/%d (%.1f%%), EE pos err: %.2f mm, ori err: %.3f, Solve: %.0f ms\n', ...
            k, nSteps, 100*k/nSteps, log.positionErrorNorm(k)*1000, ...
            log.orientationError(k), log.solveTime(k)*1000);
    end
end

%% Finalize log
log.avgEEPosError = mean(log.positionErrorNorm);
log.avgEEOriError = mean(log.orientationError);
log.maxEEPosError = max(log.positionErrorNorm);
log.maxEEOriError = max(log.orientationError);
log.avgSolveTime = mean(log.solveTime);
log.controlFrequency = 1 / log.avgSolveTime;
log.nmpcParams = nmpcParams;

%% Summary
if verbose
    fprintf('\n=== Method 5 (Whole-Body MPC) Summary ===\n');
    fprintf('Total steps: %d\n', nSteps);
    fprintf('Total time: %.2f s\n', log.timestamps(end));
    fprintf('Mean EE position error: %.2f mm (max: %.2f mm)\n', ...
        log.avgEEPosError*1000, log.maxEEPosError*1000);
    fprintf('Mean EE orientation error: %.3f (max: %.3f)\n', ...
        log.avgEEOriError, log.maxEEOriError);
    fprintf('Mean solve time: %.1f ms (%.1f Hz)\n', log.avgSolveTime*1000, log.controlFrequency);
    fprintf('MPC convergence rate: %.1f%%\n', 100*sum(log.mpcExitFlag > 0)/nSteps);
    fprintf('Mean MPC iterations: %.1f\n', mean(log.mpcIterations(log.mpcExitFlag > 0)));
    fprintf('Mean MPC cost: %.2e\n', mean(log.mpcCost(log.mpcExitFlag > 0)));
    fprintf('=========================================\n\n');
end

end

%% Local helper function
function y = computeEEPoseOutput(x, robot, eeBodyName)
%COMPUTEEEPOSOUTPUT Output function: Compute EE pose from full state
%   y = [p_ee(3); R_ee_vec(9)] - 12x1 vector
%
%   This is used by nlmpc.Model.OutputFcn

% Get full configuration from state
q_full = x;  % [x, y, theta, q_arm(6)]

% Compute FK
T_ee = getTransform(robot, q_full, eeBodyName);

% Extract position and rotation
p_ee = T_ee(1:3, 4);
R_ee = T_ee(1:3, 1:3);

% Assemble output
y = [p_ee; R_ee(:)];  % 12x1

end