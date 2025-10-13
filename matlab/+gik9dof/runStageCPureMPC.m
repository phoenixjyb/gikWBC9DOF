function log = runStageCPureMPC(robot, trajStruct, q_start, options)
%RUNSTAGECPUREMPC Stage C execution using Pure MPC method (Method 5).
%   log = GIK9DOF.RUNSTAGECPUREMPC(robot, trajStruct, q_start) executes
%   Stage C trajectory tracking using true receding horizon Nonlinear MPC:
%     1. BUILD: Create nlmpc controller with unicycle dynamics
%     2. LOOP: For each control step:
%        a. Get reference trajectory segment (horizon ahead)
%        b. Solve NMPC optimization (nlmpcmove)
%        c. Apply first control [v*, ω*]
%        d. Simulate base motion (unicycle dynamics)
%        e. Solve arm IK for fixed base
%        f. Log and advance
%
%   log = GIK9DOF.RUNSTAGECPUREMPC(..., options) supports:
%       NMPCParams           - NMPC configuration struct (required)
%                              .horizon: Prediction horizon (steps)
%                              .control_horizon: Control horizon (steps)
%                              .sample_time: Control period (s)
%                              .weights: Cost weights struct
%                              .constraints: Velocity/wheel speed limits
%       ArmIKParams          - Arm IK configuration struct
%                              .max_iterations, .tolerance, .weights
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
%           targetPositions  - [3xN] desired EE positions
%           positionError    - [3xN] tracking error vectors
%           positionErrorNorm- [1xN] error magnitudes
%           baseCommands     - [Nx2] MPC commands [v, omega]
%           basePredicted    - [3xN] MPC base predictions
%           baseActual       - [3xN] actual base states
%           mpcIterations    - [1xN] MPC iterations per step
%           mpcCost          - [1xN] MPC cost per step
%           mpcExitFlag      - [1xN] MPC solver exit flags
%           armIKSuccess     - [1xN] Arm IK convergence flags
%           solveTime        - [1xN] solve time per step
%           avgEEError       - Scalar, mean EE tracking error (m)
%           maxEEError       - Scalar, max EE tracking error (m)
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
%       armIKParams = configTools.getStageConfig('c', 'arm_ik');
%       
%       log = gik9dof.runStageCPureMPC(robot, traj, q0, ...
%           'NMPCParams', nmpcParams, ...
%           'ArmIKParams', armIKParams, ...
%           'VerboseLevel', 2);
%
%   See also: nlmpc, nlmpcmove, unicycleStateFcn

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - True receding horizon NMPC

%% Parse options
arguments
    robot
    trajStruct
    q_start (9,1) double
    options.NMPCParams struct
    options.ArmIKParams struct = struct('max_iterations', 100, 'tolerance', 1e-3, 'weights', [1 1 1 1 1 1])
    options.BaseIndices (1,:) double = [1 2 3]
    options.ArmIndices (1,:) double = [4 5 6 7 8 9]
    options.EndEffector char = 'left_gripper_link'
    options.VerboseLevel (1,1) double {mustBeInRange(options.VerboseLevel, 0, 2)} = 1
end

verbose = options.VerboseLevel >= 1;
detailedVerbose = options.VerboseLevel >= 2;

if verbose
    fprintf('\n=== Method 5 (pureMPC): Receding Horizon NMPC ===\n');
    fprintf('Using MATLAB MPC Toolbox for online optimization\n\n');
end

%% Step 1: Extract reference base path from trajectory
if verbose
    fprintf('Step 1: Extracting reference base path from EE trajectory...\n');
end

nWaypoints = size(trajStruct.Poses, 3);

% Extract EE positions from trajectory
eePositions = zeros(3, nWaypoints);
for k = 1:nWaypoints
    T_ee = trajStruct.Poses(:, :, k);
    eePositions(:, k) = T_ee(1:3, 4);
end

% Generate initial base path estimate (simple projection onto ground plane)
% This will be refined by NMPC during execution
basePathRef = zeros(nWaypoints, 3);  % [x, y, theta]
basePathRef(:, 1:2) = eePositions(1:2, :)';  % Project x, y

% Estimate heading from path tangent
for k = 1:nWaypoints-1
    dx = basePathRef(k+1, 1) - basePathRef(k, 1);
    dy = basePathRef(k+1, 2) - basePathRef(k, 2);
    basePathRef(k, 3) = atan2(dy, dx);
end
basePathRef(end, 3) = basePathRef(end-1, 3);  % Last waypoint inherits heading

if verbose
    fprintf('  Extracted %d waypoint reference path\n', nWaypoints);
    fprintf('  Path length: %.2f m\n', sum(vecnorm(diff(basePathRef(:,1:2))')));
end

%% Step 2: Create NMPC controller
if verbose
    fprintf('Step 2: Building NMPC controller...\n');
end

% Extract NMPC parameters
nmpcParams = options.NMPCParams;
p = nmpcParams.horizon;              % Prediction horizon
m = nmpcParams.control_horizon;      % Control horizon
Ts = nmpcParams.sample_time;         % Sample time

% Create nlmpc object
% States: [x, y, theta] (3 states)
% Outputs: [x, y, theta] (3 outputs, full state feedback)
% Inputs: [v, omega] (2 manipulated variables)
nlobj = nlmpc(3, 3, 2);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;

% Set model (unicycle dynamics)
nlobj.Model.StateFcn = 'gik9dof.mpc.unicycleStateFcn';
nlobj.Jacobian.StateFcn = 'gik9dof.mpc.unicycleStateJacobian';  % For speed

% Output function (full state feedback)
nlobj.Model.OutputFcn = @(x, u) x;

% Configure cost weights
weights = struct();
weights.tracking = nmpcParams.weights.tracking;
weights.input_v = nmpcParams.weights.input_v;
weights.input_omega = nmpcParams.weights.input_omega;
weights.rate_v = nmpcParams.weights.rate_v;
weights.rate_omega = nmpcParams.weights.rate_omega;
nlobj = gik9dof.mpc.configureNMPCWeights(nlobj, weights);

% Set input constraints
nlobj.MV(1).Min = nmpcParams.constraints.v_min;
nlobj.MV(1).Max = nmpcParams.constraints.v_max;
nlobj.MV(2).Min = nmpcParams.constraints.omega_min;
nlobj.MV(2).Max = nmpcParams.constraints.omega_max;

% Set custom wheel speed constraints
constraintParams = struct();
constraintParams.track_width = nmpcParams.constraints.track_width;
constraintParams.wheel_max = nmpcParams.constraints.wheel_max;
nlobj.Optimization.CustomIneqConFcn = @(X,U,data) ...
    gik9dof.mpc.wheelSpeedConstraints(X, U, data, constraintParams);

% Solver options
nlobj.Optimization.SolverOptions.MaxIterations = nmpcParams.solver.max_iterations;
nlobj.Optimization.SolverOptions.ConstraintTolerance = nmpcParams.solver.constraint_tolerance;

if verbose
    fprintf('  NMPC controller created:\n');
    fprintf('    Horizon: p=%d steps (%.1f s), m=%d control\n', p, p*Ts, m);
    fprintf('    Sample time: Ts=%.2f s (%.0f Hz)\n', Ts, 1/Ts);
    fprintf('    Constraints: v∈[%.2f, %.2f] m/s, ω∈[%.2f, %.2f] rad/s\n', ...
        nmpcParams.constraints.v_min, nmpcParams.constraints.v_max, ...
        nmpcParams.constraints.omega_min, nmpcParams.constraints.omega_max);
    fprintf('    Wheel speed limit: |v_wheel| ≤ %.2f m/s\n', nmpcParams.constraints.wheel_max);
end

%% Step 3: Initialize state and control
if verbose
    fprintf('Step 3: Initializing control loop...\n');
end

% Initial state
x_current = q_start(options.BaseIndices);  % [x, y, theta]
q_arm_current = q_start(options.ArmIndices);
u_last = [0; 0];  % [v, omega] - start from rest
t_current = 0;

% Preallocate log arrays
log = struct();
nSteps = nWaypoints;  % One control step per waypoint (can be adjusted)
log.qTraj = zeros(9, nSteps);
log.timestamps = zeros(1, nSteps);
log.eePositions = zeros(3, nSteps);
log.targetPositions = zeros(3, nSteps);
log.positionError = zeros(3, nSteps);
log.positionErrorNorm = zeros(1, nSteps);
log.baseCommands = zeros(nSteps, 2);  % [v, omega]
log.basePredicted = zeros(3, nSteps);
log.baseActual = zeros(3, nSteps);
log.mpcIterations = zeros(1, nSteps);
log.mpcCost = zeros(1, nSteps);
log.mpcExitFlag = zeros(1, nSteps);
log.armIKSuccess = false(1, nSteps);
log.solveTime = zeros(1, nSteps);

if verbose
    fprintf('  Initial state: [x=%.2f, y=%.2f, θ=%.1f°]\n', ...
        x_current(1), x_current(2), rad2deg(x_current(3)));
    fprintf('Step 4: Executing NMPC control loop for %d steps...\n', nSteps);
end

%% Step 4: Main NMPC control loop
for k = 1:nSteps
    tic;
    
    %% 4a: Get reference trajectory segment
    % Reference for horizon: k to min(k+p, nWaypoints)
    k_end = min(k + p, nWaypoints);
    refSegment = basePathRef(k:k_end, :);  % [x, y, theta]
    
    % Pad if necessary (repeat last waypoint)
    if size(refSegment, 1) < p+1
        nPad = p+1 - size(refSegment, 1);
        refSegment = [refSegment; repmat(refSegment(end, :), nPad, 1)];
    end
    
    % NMPC expects [ny x (p+1)] reference
    yref = refSegment';  % [3 x p+1]
    
    %% 4b: Solve NMPC optimization
    try
        [u_opt, mpcInfo] = nlmpcmove(nlobj, x_current, u_last, yref);
        
        log.mpcIterations(k) = mpcInfo.Iterations;
        log.mpcCost(k) = mpcInfo.Cost;
        log.mpcExitFlag(k) = mpcInfo.ExitFlag;
        
        if mpcInfo.ExitFlag < 0
            % Solver failed, use last control or zero
            if detailedVerbose
                fprintf('  [%3d] MPC solver failed (ExitFlag=%d), using fallback\n', k, mpcInfo.ExitFlag);
            end
            u_opt = u_last * 0.5;  % Slow down
        end
    catch ME
        if detailedVerbose
            fprintf('  [%3d] MPC error: %s, using zero control\n', k, ME.message);
        end
        u_opt = [0; 0];
        log.mpcExitFlag(k) = -99;
    end
    
    %% 4c: Simulate base motion (unicycle dynamics)
    x_next = gik9dof.mpc.unicycleStateFcn(x_current, u_opt, Ts);
    
    %% 4d: Solve arm IK for fixed base
    % Get EE target at current waypoint
    T_ee_target = trajStruct.Poses(:, :, k);
    eeTarget = [T_ee_target(1:3, 4); ...  % Position
                rotm2quat(T_ee_target(1:3, 1:3))'];  % Quaternion [qw qx qy qz]
    
    [q_arm_next, ikSuccess, ~] = gik9dof.mpc.solveArmIKForBase(...
        robot, eeTarget, x_next, q_arm_current, options.ArmIKParams);
    
    log.armIKSuccess(k) = ikSuccess;
    
    % Compute actual EE position
    q_full = [x_next; q_arm_next];
    T_ee_actual = getTransform(robot, q_full, char(options.EndEffector));
    p_ee_actual = T_ee_actual(1:3, 4);
    p_ee_target = T_ee_target(1:3, 4);
    ee_error = p_ee_actual - p_ee_target;
    
    %% 4e: Update state
    x_current = x_next;
    q_arm_current = q_arm_next;
    u_last = u_opt;
    t_current = t_current + Ts;
    
    %% 4f: Log results
    log.qTraj(:, k) = q_full;
    log.timestamps(k) = t_current;
    log.eePositions(:, k) = p_ee_actual;
    log.targetPositions(:, k) = p_ee_target;
    log.positionError(:, k) = ee_error;
    log.positionErrorNorm(k) = norm(ee_error);
    log.baseCommands(k, :) = u_opt';
    log.basePredicted(:, k) = refSegment(1, :)';  % First ref point
    log.baseActual(:, k) = x_next;
    log.solveTime(k) = toc;
    
    % Progress indicator
    if verbose && mod(k, 10) == 0
        fprintf('  Progress: %d/%d steps (%.1f%%), EE error: %.2f mm, Solve: %.0f ms\n', ...
            k, nSteps, 100*k/nSteps, log.positionErrorNorm(k)*1000, log.solveTime(k)*1000);
    end
end

%% Finalize log
log.avgEEError = mean(log.positionErrorNorm);
log.maxEEError = max(log.positionErrorNorm);
log.avgSolveTime = mean(log.solveTime);
log.controlFrequency = 1 / log.avgSolveTime;
log.nmpcParams = nmpcParams;
log.armIKParams = options.ArmIKParams;
log.basePathRef = basePathRef;

%% Summary
if verbose
    fprintf('\n=== Method 5 Summary ===\n');
    fprintf('Total steps: %d\n', nSteps);
    fprintf('Total time: %.2f s\n', log.timestamps(end));
    fprintf('Mean EE error: %.2f mm (max: %.2f mm)\n', log.avgEEError*1000, log.maxEEError*1000);
    fprintf('Mean solve time: %.1f ms (%.1f Hz)\n', log.avgSolveTime*1000, log.controlFrequency);
    fprintf('Arm IK success rate: %.1f%%\n', 100*sum(log.armIKSuccess)/nSteps);
    fprintf('MPC convergence rate: %.1f%%\n', 100*sum(log.mpcExitFlag > 0)/nSteps);
    fprintf('Mean MPC iterations: %.1f\n', mean(log.mpcIterations(log.mpcExitFlag > 0)));
    fprintf('Mean MPC cost: %.2e\n', mean(log.mpcCost(log.mpcExitFlag > 0)));
    fprintf('========================\n\n');
end

end
