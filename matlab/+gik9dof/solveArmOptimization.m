function [q_dot_opt, diagnostics] = solveArmOptimization(robot, state, ee_ref, options)
%SOLVEARMOPTIMIZATION Optimize arm velocities [q̇₁...q̇₆] with frozen base (Method 6)
%
% This function solves a 6-variable optimization problem to find optimal arm
% joint velocities that move the end-effector toward the reference pose while
% keeping the base configuration frozen.
%
% Inputs:
%   robot      - rigidBodyTree object
%   state      - Current robot state [x; y; theta; q_arm] (9x1)
%   ee_ref     - Target EE pose as 4x4 homogeneous transform
%   options    - Struct with fields:
%       .dt            - Time step (e.g., 0.05 for 20 Hz)
%       .q_dot_prev    - Previous arm velocities (6x1) for smoothness
%       .q_dot_max     - Maximum joint velocities (6x1 or scalar)
%       .q_min         - Minimum joint positions (6x1)
%       .q_max         - Maximum joint positions (6x1)
%       .ee_body_name  - End-effector body name
%       .arm_indices   - Indices of arm joints in full config (default 4:9)
%       .weights       - Cost weights struct:
%           .ee_pos        - EE position tracking weight
%           .ee_orient     - EE orientation tracking weight
%           .q_dot         - Joint velocity effort
%           .smooth        - Velocity change penalty
%           .posture       - Posture regularization (optional)
%       .q_nominal     - Nominal arm posture (optional, 6x1)
%
% Outputs:
%   q_dot_opt  - Optimal arm joint velocities (6x1, rad/s)
%   diagnostics - Struct with solver info (exitflag, iterations, cost, etc.)
%
% Method 6 Architecture:
%   This is the "ARM OPTIMIZATION" step that runs on ODD timesteps.
%   The base configuration is FROZEN during this solve, making it essentially
%   an inverse kinematics problem in velocity space.
%
% See also: solveBaseOptimization, runStageCAlternating

% Extract current state
x_frozen = state(1);     % BASE FROZEN!
y_frozen = state(2);     % BASE FROZEN!
theta_frozen = state(3); % BASE FROZEN!
q_arm = state(4:9);

% Set default arm indices
if ~isfield(options, 'arm_indices')
    options.arm_indices = 4:9;
end

% Set default weights if not provided
if ~isfield(options, 'weights')
    options.weights = struct();
end
weights = options.weights;
if ~isfield(weights, 'ee_pos'), weights.ee_pos = 100.0; end
if ~isfield(weights, 'ee_orient'), weights.ee_orient = 50.0; end
if ~isfield(weights, 'q_dot'), weights.q_dot = 0.01; end  % Cheap arm motion!
if ~isfield(weights, 'smooth'), weights.smooth = 5.0; end
if ~isfield(weights, 'posture'), weights.posture = 0.1; end

% Handle scalar q_dot_max
if isscalar(options.q_dot_max)
    options.q_dot_max = options.q_dot_max * ones(6, 1);
end

% Cost function: EE tracking + effort + smoothness + posture
costFcn = @(qd) armCostFunction(qd, robot, x_frozen, y_frozen, theta_frozen, ...
    q_arm, ee_ref, options, weights);

% Initial guess: continue with previous velocity
qd0 = options.q_dot_prev;

% Box constraints on joint velocities
lb = -options.q_dot_max;
ub = options.q_dot_max;

% Nonlinear constraints: joint position limits after integration
nonlcon = @(qd) jointLimitConstraints(qd, q_arm, options);

% Optimizer options: balance speed and accuracy
optimOpts = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'sqp', ...  % SQP good for constrained problems
    'MaxIterations', 100, ...
    'MaxFunctionEvaluations', 500, ...
    'OptimalityTolerance', 1e-3, ...
    'ConstraintTolerance', 1e-3, ...
    'StepTolerance', 1e-4);

% Solve the optimization
tic;
[q_dot_opt, fval, exitflag, output] = fmincon(costFcn, qd0, [], [], [], [], ...
    lb, ub, nonlcon, optimOpts);
solve_time = toc;

% Package diagnostics
diagnostics = struct();
diagnostics.exitflag = exitflag;
diagnostics.iterations = output.iterations;
diagnostics.funcCount = output.funcCount;
diagnostics.cost = fval;
diagnostics.solveTime = solve_time;
diagnostics.converged = (exitflag > 0);

% Warn if optimization failed
if exitflag < 0
    warning('gik9dof:solveArmOptimization:OptimizationFailed', ...
        'Arm optimization failed with exitflag %d. Holding arm position.', exitflag);
    q_dot_opt = zeros(6, 1);  % Fallback: freeze arm
end

end

%% Helper Functions

function J = armCostFunction(qd, robot, x_frozen, y_frozen, theta_frozen, ...
    q_arm, ee_ref, options, weights)
%ARMCOSTFUNCTION Compute cost for arm optimization
% qd = [q̇₁; q̇₂; q̇₃; q̇₄; q̇₅; q̇₆] - arm joint velocities

dt = options.dt;

% Predict next arm configuration
q_arm_next = q_arm + dt * qd;

% Build full configuration with frozen base
q_full = [x_frozen; y_frozen; theta_frozen; q_arm_next];

% Compute predicted EE pose via forward kinematics
T_ee = getTransform(robot, q_full, options.ee_body_name);

% Extract position and orientation
p_ee = T_ee(1:3, 4);
R_ee = T_ee(1:3, 1:3);

% Reference position and orientation
p_ref = ee_ref(1:3, 4);
R_ref = ee_ref(1:3, 1:3);

% Position error
J_pos = weights.ee_pos * norm(p_ee - p_ref)^2;

% Orientation error (using Frobenius norm of rotation difference)
R_err = R_ee' * R_ref;  % Relative rotation
angle_err = acos(min(1, max(-1, (trace(R_err) - 1) / 2)));  % Clamp for numerical stability
angle_err = min(angle_err, pi - angle_err);  % Wrap to [0, pi]
J_orient = weights.ee_orient * angle_err^2;

% Joint velocity effort (penalize high speeds)
J_q_dot = weights.q_dot * sum(qd.^2);

% Smoothness (penalize velocity changes)
J_smooth = weights.smooth * sum((qd - options.q_dot_prev).^2);

% Posture regularization (pull toward nominal configuration if provided)
J_posture = 0;
if isfield(options, 'q_nominal') && ~isempty(options.q_nominal)
    J_posture = weights.posture * sum((q_arm_next - options.q_nominal).^2);
end

% Total cost
J = J_pos + J_orient + J_q_dot + J_smooth + J_posture;

end

function [c, ceq] = jointLimitConstraints(qd, q_arm, options)
%JOINTLIMITCONSTRAINTS Enforce joint position limits after integration
% Ensure q_arm + dt*qd stays within [q_min, q_max]

dt = options.dt;
q_arm_next = q_arm + dt * qd;

% Inequality constraints: q_min <= q_arm_next <= q_max
% Formulated as: q_min - q_arm_next <= 0 and q_arm_next - q_max <= 0
c = [
    options.q_min - q_arm_next;  % Lower bound violations
    q_arm_next - options.q_max   % Upper bound violations
];

% No equality constraints
ceq = [];

end
