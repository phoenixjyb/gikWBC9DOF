function [v_opt, omega_opt, diagnostics] = solveBaseOptimization(robot, state, ee_ref, options)
%SOLVEBASEOPTIMIZATION Optimize base velocities [v, ω] with frozen arm (Method 6)
%
% This function solves a 2-variable optimization problem to find optimal base
% velocities that move the end-effector toward the reference pose while keeping
% the arm configuration frozen.
%
% Inputs:
%   robot      - rigidBodyTree object
%   state      - Current robot state [x; y; theta; q_arm] (9x1)
%   ee_ref     - Target EE pose as 4x4 homogeneous transform
%   options    - Struct with fields:
%       .dt            - Time step (e.g., 0.05 for 20 Hz)
%       .v_prev        - Previous linear velocity (for smoothness)
%       .omega_prev    - Previous angular velocity (for smoothness)
%       .v_max         - Maximum linear velocity (m/s)
%       .omega_max     - Maximum angular velocity (rad/s)
%       .wheel_max     - Maximum wheel speed (m/s)
%       .track_width   - Chassis track width (m)
%       .ee_body_name  - End-effector body name
%       .weights       - Cost weights struct:
%           .ee_pos      - EE position tracking weight
%           .ee_orient   - EE orientation tracking weight
%           .v           - Linear velocity effort
%           .omega       - Angular velocity effort
%           .smooth_v    - Velocity change penalty
%           .smooth_omega - Angular velocity change penalty
%
% Outputs:
%   v_opt      - Optimal linear velocity (m/s)
%   omega_opt  - Optimal angular velocity (rad/s)
%   diagnostics - Struct with solver info (exitflag, iterations, cost, etc.)
%
% Method 6 Architecture:
%   This is the "BASE OPTIMIZATION" step that runs on EVEN timesteps.
%   The arm configuration is FROZEN during this solve, making it a simple
%   2-variable optimization problem.
%
% See also: solveArmOptimization, runStageCAlternating

% Extract current state
x = state(1);
y = state(2);
theta = state(3);
q_arm_frozen = state(4:9);  % ARM FROZEN!

% Set default weights if not provided
if ~isfield(options, 'weights')
    options.weights = struct();
end
weights = options.weights;
if ~isfield(weights, 'ee_pos'), weights.ee_pos = 100.0; end
if ~isfield(weights, 'ee_orient'), weights.ee_orient = 50.0; end
if ~isfield(weights, 'v'), weights.v = 1.0; end
if ~isfield(weights, 'omega'), weights.omega = 10.0; end
if ~isfield(weights, 'smooth_v'), weights.smooth_v = 5.0; end
if ~isfield(weights, 'smooth_omega'), weights.smooth_omega = 5.0; end

% Cost function: EE tracking + effort + smoothness
costFcn = @(u) baseCostFunction(u, robot, x, y, theta, q_arm_frozen, ...
    ee_ref, options, weights);

% Initial guess: continue with previous velocity
u0 = [options.v_prev; options.omega_prev];

% Simple box constraints on velocities
lb = [-options.v_max; -options.omega_max];
ub = [options.v_max; options.omega_max];

% Nonlinear constraints: differential drive wheel speed limits
nonlcon = @(u) wheelSpeedConstraints(u, options.track_width, options.wheel_max);

% Optimizer options: keep it fast!
optimOpts = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'sqp', ...  % SQP usually fastest for small problems
    'MaxIterations', 50, ...
    'MaxFunctionEvaluations', 200, ...
    'OptimalityTolerance', 1e-3, ...
    'ConstraintTolerance', 1e-3, ...
    'StepTolerance', 1e-4);

% Solve the optimization
tic;
[u_opt, fval, exitflag, output] = fmincon(costFcn, u0, [], [], [], [], ...
    lb, ub, nonlcon, optimOpts);
solve_time = toc;

% Extract results
v_opt = u_opt(1);
omega_opt = u_opt(2);

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
    warning('gik9dof:solveBaseOptimization:OptimizationFailed', ...
        'Base optimization failed with exitflag %d. Using initial guess.', exitflag);
    v_opt = options.v_prev * 0.9;  % Fallback: slow down
    omega_opt = options.omega_prev * 0.9;
end

end

%% Helper Functions

function J = baseCostFunction(u, robot, x, y, theta, q_arm_frozen, ...
    ee_ref, options, weights)
%BASECOSTFUNCTION Compute cost for base optimization
% u = [v; omega] - base velocities

v = u(1);
omega = u(2);
dt = options.dt;

% Predict next base state using unicycle model
x_next = x + dt * v * cos(theta);
y_next = y + dt * v * sin(theta);
theta_next = theta + dt * omega;

% Build full configuration with frozen arm
q_full = [x_next; y_next; theta_next; q_arm_frozen];

% Compute current EE pose via forward kinematics
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
% Convert to axis-angle to get scalar error
angle_err = acos((trace(R_err) - 1) / 2);
angle_err = min(angle_err, pi - angle_err);  % Wrap to [0, pi]
J_orient = weights.ee_orient * angle_err^2;

% Velocity effort (penalize high speeds)
J_v = weights.v * v^2;
J_omega = weights.omega * omega^2;

% Smoothness (penalize velocity changes)
J_smooth_v = weights.smooth_v * (v - options.v_prev)^2;
J_smooth_omega = weights.smooth_omega * (omega - options.omega_prev)^2;

% Total cost
J = J_pos + J_orient + J_v + J_omega + J_smooth_v + J_smooth_omega;

end

function [c, ceq] = wheelSpeedConstraints(u, track_width, wheel_max)
%WHEELSPEEDCONSTRAINTS Enforce differential drive wheel speed limits
% For differential drive: v_left = v - omega*W/2, v_right = v + omega*W/2

v = u(1);
omega = u(2);
W = track_width;

% Wheel speeds
v_left = v - omega * W / 2;
v_right = v + omega * W / 2;

% Inequality constraints: |v_wheel| <= wheel_max
% Formulated as: v_wheel - wheel_max <= 0 and -v_wheel - wheel_max <= 0
c = [
    v_left - wheel_max;
    -v_left - wheel_max;
    v_right - wheel_max;
    -v_right - wheel_max
];

% No equality constraints
ceq = [];

end
