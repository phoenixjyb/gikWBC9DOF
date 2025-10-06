%% Generate C++ Code WITHOUT Collision Checking (For Deployment)
% This script creates a lightweight version of the IK solver that doesn't
% use collision detection features. Since we're solving IK for trajectory
% tracking (not collision avoidance), this is acceptable.
%
% Run this in MATLAB R2024b on Windows

clear; clc;

%% Setup paths
PROJECT_ROOT = fileparts(mfilename('fullpath'));
CODEGEN_OUTPUT = fullfile(PROJECT_ROOT, 'codegen', 'arm64_noCollision');

% Add MATLAB code to path
addpath(fullfile(PROJECT_ROOT, 'matlab'));

fprintf('===================================================\n');
fprintf('Generating Lightweight C++ Code (No Collision)\n');
fprintf('===================================================\n');
fprintf('This version disables collision checking to avoid\n');
fprintf('external Robotics Toolbox dependencies\n');
fprintf('===================================================\n\n');

%% Build robot WITHOUT collision geometries
fprintf('Building robot model (no collision)...\n');

% Create robot with MaxNumBodies specified for codegen
robot = rigidBodyTree('MaxNumBodies', 11, 'DataFormat', 'column');

% Base (world frame - planar mobile base)
base = robot.Base;
base.Name = 'world';

% Prismatic X joint (mobile base X translation)
bodyX = rigidBody('base_x');
jntX = rigidBodyJoint('jnt_base_x', 'prismatic');
jntX.JointAxis = [1 0 0];
jntX.PositionLimits = [-100 100];  % meters
bodyX.Joint = jntX;
addBody(robot, bodyX, 'world');

% Prismatic Y joint (mobile base Y translation)
bodyY = rigidBody('base_y');
jntY = rigidBodyJoint('jnt_base_y', 'prismatic');
jntY.JointAxis = [0 1 0];
jntY.PositionLimits = [-100 100];  % meters  
bodyY.Joint = jntY;
addBody(robot, bodyY, 'base_x');

% Revolute Z joint (mobile base rotation)
bodyZ = rigidBody('base_link');
jntZ = rigidBodyJoint('jnt_base_z', 'revolute');
jntZ.JointAxis = [0 0 1];
jntZ.PositionLimits = [-pi pi];  % radians
bodyZ.Joint = jntZ;
addBody(robot, bodyZ, 'base_y');

% Arm joints (6-DOF manipulator)
armJointLimits = [
    -2.8973, 2.8973;   % Joint 1
    -1.7628, 1.7628;   % Joint 2
    -2.8973, 2.8973;   % Joint 3
    -3.0718, -0.0698;  % Joint 4
    -2.8973, 2.8973;   % Joint 5
    -0.0175, 3.7525    % Joint 6
];

armLinkLengths = [0.333, 0.316, 0.384, 0.088, 0.107, 0.05];

parentBody = 'base_link';
for i = 1:6
    bodyName = sprintf('left_arm_link%d', i);
    jointName = sprintf('left_arm_joint%d', i);
    
    body_i = rigidBody(bodyName);
    jnt_i = rigidBodyJoint(jointName, 'revolute');
    jnt_i.JointAxis = [0 0 1];
    jnt_i.PositionLimits = armJointLimits(i, :);
    
    % Simple DH-like transform
    tform = trvec2tform([0, 0, armLinkLengths(i)]);
    setFixedTransform(jnt_i, tform);
    
    body_i.Joint = jnt_i;
    
    % NO COLLISION GEOMETRY - This is the key change!
    % body_i.addVisual(...) and body_i.addCollision(...) are omitted
    
    addBody(robot, body_i, parentBody);
    parentBody = bodyName;
end

% End-effector
eeBody = rigidBody('left_gripper_link');
eeJoint = rigidBodyJoint('left_gripper_joint', 'fixed');
setFixedTransform(eeJoint, trvec2tform([0, 0, 0.1]));
eeBody.Joint = eeJoint;
addBody(robot, eeBody, 'left_arm_link6');

fprintf('✓ Robot built (9 DOF, no collision geometries)\n');
fprintf('  Bodies: %d\n', robot.NumBodies);

%% Create IK solver WITHOUT collision constraints
fprintf('Creating IK solver (no collision)...\n');

solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint', 'distance'});

% Solver settings
solver.SolverParameters.AllowRandomRestart = false;
solver.SolverParameters.MaxIterations = 100;
solver.SolverParameters.MaxTime = 0.05;  % 50ms
solver.SolverParameters.GradientTolerance = 1e-6;
solver.SolverParameters.SolutionTolerance = 1e-6;

fprintf('✓ Solver created\n');

%% Create wrapper function for code generation
fprintf('Creating wrapper function...\n');

% This function will be inlined during code generation
wrapperFcn = @(qCurrent, targetPose, distanceLower, distanceWeight) ...
    gik9dof.codegen_realtime.solveGIKStepRealtime(robot, solver, qCurrent, targetPose, distanceLower, distanceWeight);

%% Code generation config
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.GenerateExampleMain = 'DoNotGenerate';

% C++ settings
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.BuildConfiguration = 'Faster Runs';

% Enable dynamic memory (required for IK constraints)
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536;

% Enable OpenMP
cfg.EnableOpenMP = true;

%% Define example inputs
qCurrent = zeros(9, 1);
targetPose = eye(4);
distanceLower = 0.5;
distanceWeight = 1.0;

%% Generate code
fprintf('\nGenerating C++ code...\n');
fprintf('This may take 2-3 minutes...\n\n');

tic;
codegen('-config', cfg, ...
    'gik9dof.codegen_realtime.solveGIKStepWrapper', ...
    '-args', {qCurrent, targetPose, distanceLower, distanceWeight}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');
elapsed = toc;

fprintf('\n===================================================\n');
fprintf('✓ Code generation successful!\n');
fprintf('===================================================\n');
fprintf('Time elapsed: %.1f seconds\n', elapsed);
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('===================================================\n');

fprintf('\n✓ This version should build WITHOUT external dependencies!\n');
fprintf('  No collision checking = No collisioncodegen_* functions\n\n');
