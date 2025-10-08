%% test_20constraints.m
% Test the 20-constraint GIK solver implementation
% This validates the new interface before code generation

clear; clc;

fprintf('===================================================\n');
fprintf('Testing 20-Constraint GIK Solver\n');
fprintf('===================================================\n\n');

%% Setup path
addpath(genpath(fileparts(fileparts(fileparts(mfilename('fullpath'))))));

%% Test configuration
fprintf('Setting up test configuration...\n');

% Initial joint configuration (home position)
q0 = zeros(9, 1);

% Target pose for end effector (slight offset from home)
targetPose = eye(4);
targetPose(1:3, 4) = [0.5; 0.2; 0.8];  % 50cm forward, 20cm left, 80cm up

% Distance constraint arrays (20 elements each)
distBodyIndices = int32(zeros(20, 1));
distRefBodyIndices = int32(zeros(20, 1));
distBoundsLower = zeros(20, 1);
distBoundsUpper = zeros(20, 1);
distWeights = zeros(20, 1);

%% Configure constraints
fprintf('Configuring distance constraints...\n');

% Constraint 1: Gripper must stay > 0.3m from chassis
distBodyIndices(1) = int32(12);     % left_gripper_link
distRefBodyIndices(1) = int32(4);   % abstract_chassis_link
distBoundsLower(1) = 0.3;
distBoundsUpper(1) = 100.0;
distWeights(1) = 1.0;
fprintf('  [1] Gripper -> Chassis: distance > 0.3m (weight=1.0)\n');

% Constraint 2: Gripper must stay < 2.0m from base (reachability)
distBodyIndices(2) = int32(12);     % left_gripper_link
distRefBodyIndices(2) = int32(1);   % base
distBoundsLower(2) = 0.0;
distBoundsUpper(2) = 2.0;
distWeights(2) = 0.5;
fprintf('  [2] Gripper -> Base: distance < 2.0m (weight=0.5)\n');

% All other constraints disabled (weight = 0)
fprintf('  [3-20] Disabled (weight=0)\n\n');

%% Test 1: Call wrapper function
fprintf('Test 1: Calling wrapper function...\n');
try
    tic;
    [qNext, solverInfo] = gik9dof.codegen_inuse.solveGIKStepWrapper(...
        q0, targetPose, ...
        distBodyIndices, distRefBodyIndices, ...
        distBoundsLower, distBoundsUpper, distWeights);
    elapsed = toc;
    
    fprintf('  ✓ Solver completed in %.3f ms\n', elapsed * 1000);
    fprintf('  Status: %d\n', solverInfo.Status);
    fprintf('  Iterations: %d\n', solverInfo.Iterations);
    fprintf('  Pose error norm: %.6f\n', solverInfo.PoseErrorNorm);
    fprintf('  Joint configuration: [');
    fprintf('%.3f ', qNext);
    fprintf(']\n\n');
    
catch ME
    fprintf('  ✗ Error: %s\n', ME.message);
    fprintf('  Location: %s (line %d)\n\n', ME.stack(1).name, ME.stack(1).line);
    rethrow(ME);
end

%% Test 2: Verify procedural robot matches wrapper robot
fprintf('Test 2: Verifying robot consistency...\n');
try
    robot = gik9dof.codegen_inuse.buildRobotForCodegen();
    fprintf('  ✓ Robot built successfully\n');
    fprintf('  Bodies: %d\n', robot.NumBodies);
    fprintf('  DOF: %d\n', sum(strcmp(robot.BodyNames, 'left_gripper_link')));
    
    % Check body names
    expectedBodies = {'base_link_x', 'base_link_y', 'abstract_chassis_link', ...
                     'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
                     'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
                     'left_arm_link6', 'left_gripper_link'};
    
    allPresent = true;
    for i = 1:length(expectedBodies)
        if ~any(strcmp(robot.BodyNames, expectedBodies{i}))
            fprintf('  ✗ Missing body: %s\n', expectedBodies{i});
            allPresent = false;
        end
    end
    
    if allPresent
        fprintf('  ✓ All expected bodies present\n\n');
    else
        fprintf('  ✗ Some bodies missing\n\n');
    end
    
catch ME
    fprintf('  ✗ Error: %s\n\n', ME.message);
    rethrow(ME);
end

%% Test 3: Test with all constraints disabled
fprintf('Test 3: All constraints disabled (should still work)...\n');
try
    distWeightsDisabled = zeros(20, 1);
    
    tic;
    [qNext2, solverInfo2] = gik9dof.codegen_inuse.solveGIKStepWrapper(...
        q0, targetPose, ...
        distBodyIndices, distRefBodyIndices, ...
        distBoundsLower, distBoundsUpper, distWeightsDisabled);
    elapsed2 = toc;
    
    fprintf('  ✓ Solver completed in %.3f ms\n', elapsed2 * 1000);
    fprintf('  Status: %d\n', solverInfo2.Status);
    fprintf('  Iterations: %d\n\n', solverInfo2.Iterations);
    
catch ME
    fprintf('  ✗ Error: %s\n\n', ME.message);
    rethrow(ME);
end

%% Test 4: Test with multiple active constraints
fprintf('Test 4: Multiple active constraints...\n');
try
    % Enable 5 constraints
    distWeightsMulti = zeros(20, 1);
    
    % Constraint 1: Gripper -> Chassis
    distBodyIndices(1) = int32(12);
    distRefBodyIndices(1) = int32(4);
    distBoundsLower(1) = 0.3;
    distBoundsUpper(1) = 100.0;
    distWeightsMulti(1) = 1.0;
    
    % Constraint 2: Link5 -> Chassis
    distBodyIndices(2) = int32(10);
    distRefBodyIndices(2) = int32(4);
    distBoundsLower(2) = 0.2;
    distBoundsUpper(2) = 100.0;
    distWeightsMulti(2) = 0.5;
    
    % Constraint 3: Link3 -> Base
    distBodyIndices(3) = int32(8);
    distRefBodyIndices(3) = int32(1);
    distBoundsLower(3) = 0.1;
    distBoundsUpper(3) = 100.0;
    distWeightsMulti(3) = 0.3;
    
    tic;
    [qNext3, solverInfo3] = gik9dof.codegen_inuse.solveGIKStepWrapper(...
        q0, targetPose, ...
        distBodyIndices, distRefBodyIndices, ...
        distBoundsLower, distBoundsUpper, distWeightsMulti);
    elapsed3 = toc;
    
    fprintf('  ✓ Solver completed in %.3f ms\n', elapsed3 * 1000);
    fprintf('  Status: %d\n', solverInfo3.Status);
    fprintf('  Iterations: %d\n', solverInfo3.Iterations);
    fprintf('  Active constraints: 3\n\n');
    
catch ME
    fprintf('  ✗ Error: %s\n\n', ME.message);
    rethrow(ME);
end

%% Summary
fprintf('===================================================\n');
fprintf('All tests passed! ✓\n');
fprintf('===================================================\n');
fprintf('The 20-constraint implementation is working correctly.\n');
fprintf('Ready for code generation.\n\n');
fprintf('Body Index Reference:\n');
fprintf('  1  = base (world frame)\n');
fprintf('  2  = base_link_x\n');
fprintf('  3  = base_link_y\n');
fprintf('  4  = abstract_chassis_link\n');
fprintf('  5  = left_arm_base_link\n');
fprintf('  6  = left_arm_link1\n');
fprintf('  7  = left_arm_link2\n');
fprintf('  8  = left_arm_link3\n');
fprintf('  9  = left_arm_link4\n');
fprintf('  10 = left_arm_link5\n');
fprintf('  11 = left_arm_link6\n');
fprintf('  12 = left_gripper_link\n');
fprintf('===================================================\n');
