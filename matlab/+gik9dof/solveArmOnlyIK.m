function [q_arm, solInfo] = solveArmOnlyIK(robot, gikBundle, T_ee_target, q_base_fixed, q_arm_init, options)
%SOLVEARMONYLIK Solve IK with fixed base (arm-only fallback).
%   [q_arm, solInfo] = GIK9DOF.SOLVEARMONLYIK(robot, gikBundle, T_ee_target, q_base_fixed, q_arm_init)
%   solves inverse kinematics for the arm while keeping the base locked at
%   q_base_fixed. This is used as a fallback when the PP-constrained GIK
%   solution produces excessive EE tracking error.
%
%   Inputs:
%       robot         - rigidBodyTree object
%       gikBundle     - Solver bundle from createGikSolver
%       T_ee_target   - [4x4] desired end-effector pose
%       q_base_fixed  - [3x1] fixed base configuration [x, y, theta]
%       q_arm_init    - [6x1] initial arm configuration
%       options       - Name-value pairs:
%           BaseIndices   - Base joint indices (default [1 2 3])
%           ArmIndices    - Arm joint indices (default [4 5 6 7 8 9])
%
%   Returns:
%       q_arm         - [6x1] solved arm configuration
%       solInfo       - Solution info struct from GIK solver
%
%   Example:
%       q_base = [1.5; 0.8; pi/4];
%       q_arm_guess = homeConfiguration(robot);
%       [q_arm, info] = gik9dof.solveArmOnlyIK(robot, gikBundle, T_target, q_base, q_arm_guess(4:9));
%
%   See also createGikSolver, runStageCPPFirst, updateBaseJointBounds.

arguments
    robot (1,1) rigidBodyTree
    gikBundle (1,1) struct
    T_ee_target (4,4) double
    q_base_fixed (3,1) double
    q_arm_init (6,1) double
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
end

% Lock base joints to fixed values with tight tolerance
epsilon = 1e-6;
jointConst = gikBundle.constraints.joint;

for i = 1:3
    idx = options.BaseIndices(i);
    jointConst.Bounds(idx, :) = [q_base_fixed(i) - epsilon, q_base_fixed(i) + epsilon];
end

% Build initial configuration
q_init = zeros(9, 1);
q_init(options.BaseIndices) = q_base_fixed;
q_init(options.ArmIndices) = q_arm_init;

% Solve IK using bundle.solve wrapper
[q_solution, solInfo] = gikBundle.solve(q_init, 'TargetPose', T_ee_target);

% Extract arm joints
q_arm = q_solution(options.ArmIndices);

% Note: Base joints in q_solution should be very close to q_base_fixed
% due to tight bounds, but we return only arm config for clarity

end
