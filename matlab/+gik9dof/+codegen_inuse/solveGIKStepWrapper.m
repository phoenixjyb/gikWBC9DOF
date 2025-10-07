function [qNext, solverInfo] = solveGIKStepWrapper(qCurrent, targetPose, distanceLower, distanceWeight)
%SOLVEGIKSTEPWRAPPER Wrapper for GIK solver with persistent robot and solver
%   This wrapper initializes the robot and solver once and reuses them.
%   Use this for code generation - it will generate initialization code.
%
%#codegen

persistent robot solver initialized

if isempty(initialized)
    % Build robot model procedurally
    robot = gik9dof.codegen_inuse.buildRobotForCodegen();
    
    % Create GIK solver with Levenberg-Marquardt algorithm
    constraintInputs = {'pose', 'joint', 'distance'};
    solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', constraintInputs, ...
        'SolverAlgorithm', 'LevenbergMarquardt');
    
    % Configure solver parameters for real-time performance
    % These settings optimize for speed while maintaining solution quality
    solver.SolverParameters.MaxTime = 0.05;  % 50ms timeout for real-time control
    solver.SolverParameters.MaxIterations = 50;  % Limit iterations to prevent excessive computation
    solver.SolverParameters.AllowRandomRestart = false;  % Disable restarts for deterministic timing
    solver.SolverParameters.SolutionTolerance = 1e-6;  % Good enough for robotic control
    solver.SolverParameters.GradientTolerance = 1e-7;  % Gradient convergence criteria
    
    initialized = true;
end

% Call the realtime solver
[qNext, solverInfo] = gik9dof.codegen_inuse.solveGIKStepRealtime(...
    robot, solver, qCurrent, targetPose, distanceLower, distanceWeight);

end
