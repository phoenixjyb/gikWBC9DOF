%% test_codegen_simple.m
% Simplified codegen test - just the realtime solver without persistent variables

clear; clc;

fprintf('Testing simplified code generation...\n\n');

%% Setup
addpath(genpath(pwd));

%% Build robot first (needed for solver initialization)
fprintf('Building robot procedurally...\n');
robot = gik9dof.codegen_inuse.buildRobotForCodegen();
fprintf('  ✓ Robot built: %d bodies\n\n', robot.NumBodies);

%% Create solver
fprintf('Creating GIK solver...\n');
constraintInputs = {'pose', 'joint', ...
    'distance', 'distance', 'distance', 'distance', 'distance', ...
    'distance', 'distance', 'distance', 'distance', 'distance', ...
    'distance', 'distance', 'distance', 'distance', 'distance', ...
    'distance', 'distance', 'distance', 'distance', 'distance'};

solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', constraintInputs, ...
    'SolverAlgorithm', 'LevenbergMarquardt');
fprintf('  ✓ Solver created\n\n');

%% Prepare example inputs
fprintf('Preparing example inputs...\n');
q0 = zeros(9, 1);
targetPose = eye(4);
targetPose(1:3, 4) = [0.5; 0.2; 0.8];

distBodyIndices = int32(zeros(20, 1));
distRefBodyIndices = int32(zeros(20, 1));
distBoundsLower = zeros(20, 1);
distBoundsUpper = zeros(20, 1);
distWeights = zeros(20, 1);

% Enable one constraint
distBodyIndices(1) = int32(12);
distRefBodyIndices(1) = int32(4);
distBoundsLower(1) = 0.3;
distBoundsUpper(1) = 100.0;
distWeights(1) = 1.0;

fprintf('  ✓ Inputs ready\n\n');

%% Test function call first
fprintf('Testing function call in MATLAB...\n');
try
    [qNext, info] = gik9dof.codegen_inuse.solveGIKStepRealtime(...
        robot, solver, q0, targetPose, ...
        distBodyIndices, distRefBodyIndices, ...
        distBoundsLower, distBoundsUpper, distWeights);
    fprintf('  ✓ Function call successful\n');
    fprintf('  Status: %s, Iterations: %d\n\n', info.Status, info.Iterations);
catch ME
    fprintf('  ✗ Function call failed: %s\n\n', ME.message);
    rethrow(ME);
end

fprintf('===================================================\n');
fprintf('All preliminary checks passed!\n');
fprintf('The issue is likely in the code generation process itself.\n');
fprintf('===================================================\n');
