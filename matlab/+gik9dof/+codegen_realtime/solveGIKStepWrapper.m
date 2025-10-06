function [qNext, solverInfo] = solveGIKStepWrapper(qCurrent, targetPose, distanceLower, distanceWeight)
%SOLVEGIKSTEPWRAPPER Wrapper for GIK solver with persistent robot and solver
%   This wrapper initializes the robot and solver once and reuses them.
%   Use this for code generation - it will generate initialization code.
%
%#codegen

persistent robot solver initialized

if isempty(initialized)
    % Build robot model procedurally
    robot = gik9dof.codegen_realtime.buildRobotForCodegen();
    
    % Create GIK solver with Levenberg-Marquardt algorithm
    constraintInputs = {'pose', 'joint', 'distance'};
    solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', constraintInputs, ...
        'SolverAlgorithm', 'LevenbergMarquardt');
    
    % Note: generalizedInverseKinematics solver parameters are set during call,
    % not as properties. See solveGIKStepRealtime for usage.
    
    initialized = true;
end

% Call the realtime solver
[qNext, solverInfo] = gik9dof.codegen_realtime.solveGIKStepRealtime(...
    robot, solver, qCurrent, targetPose, distanceLower, distanceWeight);

end
