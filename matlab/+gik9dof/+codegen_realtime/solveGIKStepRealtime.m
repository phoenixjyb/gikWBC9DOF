function [qNext, solverInfo] = solveGIKStepRealtime(robot, solver, qCurrent, targetPose, distanceLower, distanceWeight)
%SOLVEGIKSTEPREALTIME Single GIK step for real-time deployment (no persistent variables)
%   This version removes persistent variables and accepts pre-constructed
%   solver and robot objects for code generation compatibility.
%
%   Inputs:
%       robot         - rigidBodyTree object (9 DOF)
%       solver        - generalizedInverseKinematics object
%       qCurrent      - Current joint configuration (9x1 double)
%       targetPose    - Target end-effector pose (4x4 homogeneous transform)
%       distanceLower - Lower bound for distance constraint (scalar)
%       distanceWeight - Weight for distance constraint (scalar)
%
%   Outputs:
%       qNext      - Next joint configuration (9x1 double)
%       solverInfo - Solver diagnostics structure
%
%#codegen

% Validate inputs
coder.inline('never'); % Force function call for better debugging

% Create constraints (these are lightweight objects)
poseConstraint = constraintPoseTarget('left_gripper_link');
poseConstraint.TargetTransform = targetPose;

jointConstraint = constraintJointBounds(robot);

distanceConstraint = constraintDistanceBounds('left_gripper_link');
distanceConstraint.ReferenceBody = robot.BaseName;

% Process distance bounds
if distanceLower <= 0
    lowerBound = 0;
else
    lowerBound = distanceLower;
end

if distanceWeight < 0
    w = 0;
else
    w = distanceWeight;
end

% Use large finite upper bound instead of inf (MATLAB constraint system requires finite bounds)
% 100 meters is effectively unlimited for a mobile manipulator
distanceConstraint.Bounds = [lowerBound, 100.0];
distanceConstraint.Weights = w;

% Solve IK
[qNext, solverInfo] = solver(qCurrent, poseConstraint, jointConstraint, distanceConstraint);

end
