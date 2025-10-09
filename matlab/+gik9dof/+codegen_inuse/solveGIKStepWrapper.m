function [qNext, solverInfo] = solveGIKStepWrapper(qCurrent, targetPose, ...
    distBodyIndices, distRefBodyIndices, distBoundsLower, distBoundsUpper, distWeights)
%SOLVEGIKSTEPWRAPPER GIK solver with 20 distance constraints for code generation
%   This version uses persistent variables pattern (proven to work with MATLAB Coder)
%
%#codegen

arguments
    qCurrent (9,1) double
    targetPose (4,4) double
    distBodyIndices (20,1) int32      % Not used (body pairs are fixed)
    distRefBodyIndices (20,1) int32   % Not used (body pairs are fixed)
    distBoundsLower (20,1) double
    distBoundsUpper (20,1) double
    distWeights (20,1) double
end

% Persistent solver and constraints
persistent solver robot poseConstraint jointConstraint distConstraints

if isempty(solver)
    % Build robot model procedurally
    robot = gik9dof.codegen_inuse.buildRobotForCodegen();
    
    % Create GIK solver with 22 constraint inputs (1 pose + 1 joint + 20 distance)
    constraintInputs = {'pose', 'joint', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance', ...
        'distance', 'distance', 'distance', 'distance', 'distance'};
    
    solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', constraintInputs, ...
        'SolverAlgorithm', 'LevenbergMarquardt');
    
    % Configure solver parameters for real-time Orin deployment
    solver.SolverParameters.MaxTime = 0.05;  % 50ms - real-time constraint for Jetson Orin
    solver.SolverParameters.MaxIterations = 1000;  % Increased from 50 for better convergence
    solver.SolverParameters.AllowRandomRestart = false;
    solver.SolverParameters.SolutionTolerance = 1e-6;
    solver.SolverParameters.GradientTolerance = 1e-7;
    
    % Create pose and joint constraints
    poseConstraint = constraintPoseTarget('left_gripper_link');
    jointConstraint = constraintJointBounds(robot);
    
    % Create 20 distance constraints with FIXED body pairs
    % For code generation, body names must be compile-time constants
    % All constraints start disabled (weight=0) and are enabled via weights parameter
    distConstraints = cell(1, 20);
    
    % Gripper to various bodies (most common use case)
    distConstraints{1} = constraintDistanceBounds('left_gripper_link');
    distConstraints{1}.ReferenceBody = 'abstract_chassis_link';
    
    distConstraints{2} = constraintDistanceBounds('left_gripper_link');
    distConstraints{2}.ReferenceBody = 'base';
    
    distConstraints{3} = constraintDistanceBounds('left_gripper_link');
    distConstraints{3}.ReferenceBody = 'left_arm_base_link';
    
    distConstraints{4} = constraintDistanceBounds('left_arm_link5');
    distConstraints{4}.ReferenceBody = 'abstract_chassis_link';
    
    distConstraints{5} = constraintDistanceBounds('left_arm_link4');
    distConstraints{5}.ReferenceBody = 'abstract_chassis_link';
    
    % Remaining constraints with default gripper->base pairs
    for i = 6:20
        distConstraints{i} = constraintDistanceBounds('left_gripper_link');
        distConstraints{i}.ReferenceBody = 'base';
    end
    
    % Initialize all as disabled
    for i = 1:20
        distConstraints{i}.Bounds = [0.0, 100.0];
        distConstraints{i}.Weights = 0.0;
    end
end

% Update pose constraint
poseConstraint.TargetTransform = targetPose;

% Update distance constraint bounds and weights (body pairs are FIXED at initialization)
for i = 1:20
    lowerBound = 0.0;
    upperBound = 100.0;
    weight = 0.0;
    
    % Process bounds if this constraint is enabled
    if distWeights(i) > 0
        if distBoundsLower(i) > 0
            lowerBound = distBoundsLower(i);
        end
        
        if distBoundsUpper(i) > 0 && distBoundsUpper(i) <= 100.0
            upperBound = distBoundsUpper(i);
        else
            upperBound = 100.0;
        end
        
        weight = distWeights(i);
    end
    
    distConstraints{i}.Bounds = [lowerBound, upperBound];
    distConstraints{i}.Weights = weight;
end

% Solve IK with all constraints
[qNext, solverInfo] = solver(qCurrent, poseConstraint, jointConstraint, ...
    distConstraints{1}, distConstraints{2}, distConstraints{3}, distConstraints{4}, ...
    distConstraints{5}, distConstraints{6}, distConstraints{7}, distConstraints{8}, ...
    distConstraints{9}, distConstraints{10}, distConstraints{11}, distConstraints{12}, ...
    distConstraints{13}, distConstraints{14}, distConstraints{15}, distConstraints{16}, ...
    distConstraints{17}, distConstraints{18}, distConstraints{19}, distConstraints{20});

end
