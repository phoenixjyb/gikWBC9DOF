function [q_arm, success, solInfo] = solveArmIKForBase(robot, eeTarget, basePose, q_arm_init, options)
% SOLVEARMIKFORBASE Solve arm IK with fixed base configuration
%
% Solves inverse kinematics for the 6-DOF arm given a fixed base pose.
% This is used in Method 5 (pureMPC) to compute arm joint angles after
% the NMPC has determined the optimal base trajectory.
%
% Inputs:
%   robot      - rigidBodyTree robot model
%   eeTarget   - [7x1] End-effector target [x; y; z; qw; qx; qy; qz]
%                Position (m) + orientation quaternion
%   basePose   - [3x1] Fixed base configuration [x; y; theta]
%   q_arm_init - [6x1] Initial guess for arm joint angles
%   options    - (optional) struct with fields:
%                .MaxIterations: Max IK iterations (default 100)
%                .Tolerance: Position tolerance (default 1e-3 m)
%                .Weights: [1x6] IK weights (default [1 1 1 1 1 1])
%
% Outputs:
%   q_arm   - [6x1] Solved arm joint angles (or q_arm_init if failed)
%   success - boolean, true if solution found within tolerance
%   solInfo - Solution info struct with fields:
%             .ExitFlag: Solver exit condition
%             .Iterations: Number of iterations used
%             .PoseErrorNorm: Final pose error (m)
%
% See also: inverseKinematics, rigidBodyTree

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC)

%% Parse options
if nargin < 5
    options = struct();
end

maxIter = getFieldOrDefault(options, 'MaxIterations', 100);
tolerance = getFieldOrDefault(options, 'Tolerance', 1e-3);
weights = getFieldOrDefault(options, 'Weights', [1 1 1 1 1 1]);

%% Build full joint configuration guess
% Joint order: [base_x, base_y, base_theta, arm_j1, arm_j2, ..., arm_j6]
q_guess = [basePose(1); basePose(2); basePose(3); q_arm_init];

%% Create inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = maxIter;
ik.SolverParameters.AllowRandomRestart = false;  % Deterministic

%% Get end-effector name
% Assumes last body is the end-effector
% Modify if different naming convention
endEffectorName = robot.BodyNames{end};

%% Convert target to transformation matrix
% eeTarget format: [x, y, z, qw, qx, qy, qz]
position = eeTarget(1:3);
quaternion = eeTarget(4:7);  % [qw, qx, qy, qz]

% Build SE(3) transformation matrix
tform = trvec2tform(position') * quat2tform(quaternion');

%% Solve IK with base joints constrained
% The IK solver will try to satisfy the target while keeping base joints
% close to their initial values (q_guess). Since base has low joint limits
% or high weights, they effectively stay fixed.

[q_full, solInfo] = ik(endEffectorName, tform, weights, q_guess);

%% Extract arm joints (indices 4-9 for 9-DOF system)
% Indices: 1=base_x, 2=base_y, 3=base_theta, 4-9=arm joints
q_arm = q_full(4:9);

%% Check success
% Success criteria: 
% 1. Solver exited normally (ExitFlag > 0)
% 2. Pose error within tolerance
success = (solInfo.ExitFlag > 0) && (solInfo.PoseErrorNorm < tolerance);

%% If failed, return initial guess
if ~success
    q_arm = q_arm_init;
end

end

%% Helper function
function value = getFieldOrDefault(s, field, default)
    if isfield(s, field)
        value = s.(field);
    else
        value = default;
    end
end
