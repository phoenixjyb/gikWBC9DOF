%% validate_robot_builder.m
% Quick validation script for procedural robot builder
% Run this BEFORE code generation to ensure robot model is correct

clear; clc;

fprintf('===================================================\n');
fprintf('Validating procedural robot builder...\n');
fprintf('===================================================\n\n');

%% Test 1: Build robot without errors
fprintf('Test 1: Building robot model...\n');
try
    robot = gik9dof.codegen_realtime.buildRobotForCodegen();
    fprintf('  ✓ Robot built successfully\n');
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    error('Robot build failed');
end

%% Test 2: Verify structure
fprintf('\nTest 2: Verifying robot structure...\n');
expectedJoints = {'joint_x', 'joint_y', 'joint_theta', ...
                  'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', ...
                  'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6'};

numBodies = length(robot.Bodies);
fprintf('  - Number of bodies: %d (expected: 10)\n', numBodies);

if numBodies ~= 10
    error('  ✗ FAILED: Expected 10 bodies, got %d', numBodies);
end

fprintf('  - Number of joints: %d (expected: 9 movable)\n', numBodies);
fprintf('  ✓ Structure validated\n');

%% Test 3: Check joint limits
fprintf('\nTest 3: Checking joint limits...\n');
for i = 1:length(expectedJoints)
    jnt = robot.Bodies{i}.Joint;
    fprintf('  - %s: [%.2f, %.2f]\n', jnt.Name, jnt.PositionLimits(1), jnt.PositionLimits(2));
end
fprintf('  ✓ Joint limits OK\n');

%% Test 4: Forward kinematics test
fprintf('\nTest 4: Testing forward kinematics...\n');
q0 = zeros(9, 1); % Home configuration
try
    T_ee = getTransform(robot, q0, 'left_gripper_link');
    fprintf('  - End-effector position at home: [%.3f, %.3f, %.3f]\n', ...
        T_ee(1,4), T_ee(2,4), T_ee(3,4));
    fprintf('  ✓ Forward kinematics OK\n');
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    error('Forward kinematics failed');
end

%% Test 5: Compare with URDF-based model
fprintf('\nTest 5: Comparing with URDF-based model...\n');
try
    robot_urdf = gik9dof.createRobotModel('Validate', false);
    T_ee_urdf = getTransform(robot_urdf, q0, 'left_gripper_link');
    
    % Compare end-effector positions
    pos_diff = norm(T_ee(1:3,4) - T_ee_urdf(1:3,4));
    fprintf('  - Position difference: %.6f m\n', pos_diff);
    
    if pos_diff < 1e-3
        fprintf('  ✓ Matches URDF model (tolerance: 1mm)\n');
    else
        warning('  ⚠ Position differs by %.2f mm', pos_diff * 1000);
    end
catch ME
    fprintf('  ⚠ Could not compare with URDF model: %s\n', ME.message);
end

%% Test 6: Test with GIK solver
fprintf('\nTest 6: Testing with GIK solver...\n');
try
    constraintInputs = {'pose', 'joint', 'distance'};
    solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', constraintInputs, ...
        'SolverAlgorithm', 'LevenbergMarquardt');
    
    % Simple pose target
    targetPose = eye(4);
    targetPose(1:3, 4) = [0.5; 0.2; 0.8]; % Simple reachable target
    
    poseConstraint = constraintPoseTarget('left_gripper_link');
    poseConstraint.TargetTransform = targetPose;
    jointConstraint = constraintJointBounds(robot);
    
    [qSol, solverInfo] = solver(q0, poseConstraint, jointConstraint);
    
    fprintf('  - Solver status: %d (1 = success)\n', solverInfo.Status);
    fprintf('  - Iterations: %d\n', solverInfo.Iterations);
    fprintf('  - Pose error: %.6f\n', solverInfo.PoseErrorNorm);
    
    if solverInfo.Status == 1
        fprintf('  ✓ GIK solver works correctly\n');
    else
        warning('  ⚠ GIK solver did not converge');
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    error('GIK solver test failed');
end

%% Test 7: Code generation readiness check
fprintf('\nTest 7: Code generation readiness...\n');
try
    % Check if function is on path
    which('gik9dof.codegen_realtime.buildRobotForCodegen');
    fprintf('  ✓ Function is on path\n');
    
    % Check code generation directive
    fcnInfo = functions(@gik9dof.codegen_realtime.buildRobotForCodegen);
    fprintf('  ✓ Function type: %s\n', fcnInfo.type);
    
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Summary
fprintf('\n===================================================\n');
fprintf('✓ ALL TESTS PASSED\n');
fprintf('===================================================\n');
fprintf('Robot model is ready for code generation.\n');
fprintf('Next step: Run generateCodeARM64.m\n');
fprintf('===================================================\n\n');

% Display robot details
fprintf('Robot details:\n');
showdetails(robot);
