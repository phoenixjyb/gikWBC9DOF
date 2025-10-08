function [qNext, solverInfo] = solveGIKStepWrapper(qCurrent, targetPose, ...
    distBodyIndices, distRefBodyIndices, distBoundsLower, distBoundsUpper, distWeights)
%SOLVEGIKSTEPWRAPPER Wrapper for GIK solver with persistent robot and solver (20 constraints)
%   This wrapper initializes the robot and solver once and reuses them.
%   Supports up to 20 distance constraints with fixed-size arrays.
%   Use this for code generation - it will generate initialization code.
%
%   Inputs:
%       qCurrent           - Current joint configuration (9x1 double)
%       targetPose         - Target end-effector pose (4x4 homogeneous transform)
%       distBodyIndices    - Body indices for distance constraints (20x1 int32)
%       distRefBodyIndices - Reference body indices (20x1 int32)
%       distBoundsLower    - Lower bounds for each constraint (20x1 double)
%       distBoundsUpper    - Upper bounds for each constraint (20x1 double)
%       distWeights        - Weights for each constraint (20x1 double)
%
%   See solveGIKStepRealtime.m for body index mapping and usage examples.
%
%#codegen

persistent robot solver initialized

if isempty(initialized)
    % Build robot model procedurally
    robot = gik9dof.codegen_inuse.buildRobotForCodegen();
    
    % Create GIK solver with all constraint types
    % CRITICAL: Must specify 'distance' 22 times for 20 distance constraints + pose + joint
    % Solver expects: 1 pose + 1 joint + 20 distance = 22 total constraint inputs
    constraintInputs = {'pose', 'joint', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance'};
    
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

% Call the realtime solver with 20 distance constraints
[qNext, solverInfo] = gik9dof.codegen_inuse.solveGIKStepRealtime(...
    robot, solver, qCurrent, targetPose, ...
    distBodyIndices, distRefBodyIndices, distBoundsLower, distBoundsUpper, distWeights);

end
