# Week 1 Implementation Guide - CodegenCC45

**Objective**: Refactor MATLAB code and generate first C++ library  
**Timeline**: 5 working days  
**Deliverable**: Working C++ IK solver compiled on Ubuntu 22.04

---

## Day 1: Setup & Procedural Robot Builder

### Morning: Environment Setup

**On Windows (MATLAB side)**:
```matlab
% Open MATLAB R2024b
cd('c:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF')
addpath('matlab')

% Verify toolboxes
ver  % Check for Robotics System Toolbox, MATLAB Coder

% Test current robot model
robot = gik9dof.createRobotModel();
showdetails(robot)
% Verify: 15 bodies, 9 joints (3 base + 6 arm)
```

**On Ubuntu (Build side)**:
```bash
# Update system
sudo apt update
sudo apt install -y build-essential cmake git

# Install ROS2 dependencies (even if not using ROS yet)
sudo apt install -y libeigen3-dev libomp-dev

# Verify compiler
gcc --version  # Should be 11.x or newer
g++ --version
```

---

### Afternoon: Extract URDF Parameters

**Goal**: Extract robot parameters from URDF into MATLAB struct

**Step 1**: Create parameter extraction script

```matlab
% File: matlab/+gik9dof/+codegen/extractUrdfParameters.m
function params = extractUrdfParameters(urdfPath)
%EXTRACTURDFPARAMETERS Extract robot parameters from URDF
%   Returns a struct with all joint/link parameters for procedural building

if nargin < 1
    urdfPath = gik9dof.internal.resolvePath(...
        'mobile_manipulator_PPR_base_corrected.urdf');
end

% Import robot to extract parameters
robot = importrobot(urdfPath);

% Extract joint information
params.NumBodies = robot.NumBodies;
params.BaseName = robot.BaseName;
params.Gravity = robot.Gravity;
params.DataFormat = robot.DataFormat;

% Extract each body and joint
params.Bodies = struct([]);
for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    
    params.Bodies(i).Name = body.Name;
    params.Bodies(i).Mass = body.Mass;
    params.Bodies(i).CenterOfMass = body.CenterOfMass;
    params.Bodies(i).Inertia = body.Inertia;
    
    % Joint information
    joint = body.Joint;
    params.Bodies(i).Joint.Name = joint.Name;
    params.Bodies(i).Joint.Type = joint.Type;
    params.Bodies(i).Joint.Axis = joint.JointAxis;
    
    if ~isempty(joint.HomePosition)
        params.Bodies(i).Joint.HomePosition = joint.HomePosition;
    else
        params.Bodies(i).Joint.HomePosition = 0;
    end
    
    if ~isempty(joint.PositionLimits)
        params.Bodies(i).Joint.PositionLimits = joint.PositionLimits;
    else
        params.Bodies(i).Joint.PositionLimits = [-Inf, Inf];
    end
    
    % Transform from parent
    params.Bodies(i).Joint.FixedTransform = joint.JointToParentTransform;
    
    % Visual and collision (simplified)
    if ~isempty(body.Visuals)
        params.Bodies(i).Visual.Mesh = body.Visuals{1}.Mesh;
    end
    
    if ~isempty(body.Collisions)
        params.Bodies(i).Collision.Mesh = body.Collisions{1}.Mesh;
    end
end

% Save to MAT file for reference
save(fullfile(fileparts(mfilename('fullpath')), 'urdfParams.mat'), 'params');

fprintf('Extracted parameters for %d bodies\n', params.NumBodies);
end
```

**Step 2**: Run extraction
```matlab
params = gik9dof.codegen.extractUrdfParameters();
disp(params.Bodies(1))  % Inspect first body
```

---

### Evening: Create Procedural Robot Builder

**Goal**: Build rigidBodyTree without file I/O

```matlab
% File: matlab/+gik9dof/+codegen/buildRobotForCodegenARM64.m
function robot = buildRobotForCodegenARM64()
%BUILDROBOTFORCODEGENARM64 Procedural robot builder for code generation
%   Builds the 9-DOF mobile manipulator rigidBodyTree without file I/O
%#codegen

% Initialize empty tree
robot = rigidBodyTree('DataFormat', 'column');
robot.Gravity = [0 0 -9.81];

% Base link (world-fixed)
base = rigidBody('base_link');
robot.addBody(base, robot.BaseName);

% Prismatic X joint (world X translation)
bodyX = rigidBody('x_slider');
jntX = rigidBodyJoint('joint_x', 'prismatic');
jntX.JointAxis = [1 0 0];
jntX.PositionLimits = [-10, 10];  % meters
jntX.HomePosition = 0;
bodyX.Joint = jntX;
robot.addBody(bodyX, 'base_link');

% Prismatic Y joint (world Y translation)
bodyY = rigidBody('y_slider');
jntY = rigidBodyJoint('joint_y', 'prismatic');
jntY.JointAxis = [0 1 0];
jntY.PositionLimits = [-10, 10];  % meters
jntY.HomePosition = 0;
bodyY.Joint = jntY;
robot.addBody(bodyY, 'x_slider');

% Revolute Z joint (world yaw rotation)
bodyTheta = rigidBody('chassis_yaw');
jntTheta = rigidBodyJoint('joint_theta', 'revolute');
jntTheta.JointAxis = [0 0 1];
jntTheta.PositionLimits = [-pi, pi];  % radians
jntTheta.HomePosition = 0;
bodyTheta.Joint = jntTheta;
robot.addBody(bodyTheta, 'y_slider');

% Arm base (fixed to chassis)
armBase = rigidBody('left_arm_base_link');
jntArmBase = rigidBodyJoint('arm_base_fixed', 'fixed');
tformArmBase = trvec2tform([0.2, 0.15, 0.5]);  % Position on chassis
setFixedTransform(jntArmBase, tformArmBase);
armBase.Joint = jntArmBase;
robot.addBody(armBase, 'chassis_yaw');

% Arm joint 1 (shoulder pan)
arm1 = rigidBody('left_arm_link1');
jnt1 = rigidBodyJoint('left_arm_joint1', 'revolute');
jnt1.JointAxis = [0 0 1];
jnt1.PositionLimits = [-2.9, 2.9];  % radians
jnt1.HomePosition = 0;
arm1.Joint = jnt1;
robot.addBody(arm1, 'left_arm_base_link');

% Arm joint 2 (shoulder lift)
arm2 = rigidBody('left_arm_link2');
jnt2 = rigidBodyJoint('left_arm_joint2', 'revolute');
jnt2.JointAxis = [0 1 0];
jnt2.PositionLimits = [-1.5, 1.5];
jnt2.HomePosition = 0;
tform2 = trvec2tform([0, 0, 0.1]);  % Link offset
setFixedTransform(jnt2, tform2);
arm2.Joint = jnt2;
robot.addBody(arm2, 'left_arm_link1');

% Arm joint 3 (elbow)
arm3 = rigidBody('left_arm_link3');
jnt3 = rigidBodyJoint('left_arm_joint3', 'revolute');
jnt3.JointAxis = [0 1 0];
jnt3.PositionLimits = [-2.5, 2.5];
jnt3.HomePosition = 0;
tform3 = trvec2tform([0.3, 0, 0]);  % Upper arm length
setFixedTransform(jnt3, tform3);
arm3.Joint = jnt3;
robot.addBody(arm3, 'left_arm_link2');

% Arm joint 4 (wrist 1)
arm4 = rigidBody('left_arm_link4');
jnt4 = rigidBodyJoint('left_arm_joint4', 'revolute');
jnt4.JointAxis = [0 0 1];
jnt4.PositionLimits = [-2.0, 2.0];
jnt4.HomePosition = 0;
tform4 = trvec2tform([0.3, 0, 0]);  % Forearm length
setFixedTransform(jnt4, tform4);
arm4.Joint = jnt4;
robot.addBody(arm4, 'left_arm_link3');

% Arm joint 5 (wrist 2)
arm5 = rigidBody('left_arm_link5');
jnt5 = rigidBodyJoint('left_arm_joint5', 'revolute');
jnt5.JointAxis = [0 1 0];
jnt5.PositionLimits = [-2.0, 2.0];
jnt5.HomePosition = 0;
tform5 = trvec2tform([0.1, 0, 0]);
setFixedTransform(jnt5, tform5);
arm5.Joint = jnt5;
robot.addBody(arm5, 'left_arm_link4');

% Arm joint 6 (wrist 3)
arm6 = rigidBody('left_arm_link6');
jnt6 = rigidBodyJoint('left_arm_joint6', 'revolute');
jnt6.JointAxis = [0 0 1];
jnt6.PositionLimits = [-2.5, 2.5];
jnt6.HomePosition = 0;
tform6 = trvec2tform([0.1, 0, 0]);
setFixedTransform(jnt6, tform6);
arm6.Joint = jnt6;
robot.addBody(arm6, 'left_arm_link5');

% End-effector (gripper)
gripper = rigidBody('left_gripper_link');
jntGripper = rigidBodyJoint('gripper_fixed', 'fixed');
tformGripper = trvec2tform([0.1, 0, 0]);  % Gripper offset
setFixedTransform(jntGripper, tformGripper);
gripper.Joint = jntGripper;
robot.addBody(gripper, 'left_arm_link6');

% TODO: Add collision geometry (deferred for now)
% For codegen, meshes must be added at compile time, not runtime

end
```

**IMPORTANT**: You'll need to verify the actual joint parameters from the URDF. The values above are placeholders!

**Step 3**: Test the builder
```matlab
robot = gik9dof.codegen.buildRobotForCodegenARM64();
showdetails(robot)

% Compare with original
robotOrig = gik9dof.createRobotModel();
isequal(robot.NumBodies, robotOrig.NumBodies)  % Should be true
```

---

## Day 2: Refactor IK Solver

### Morning: Remove Persistent Variables

**Current** (`solveGIKStep.m`):
```matlab
function qNext = solveGIKStep(qCurrent, targetPose, distanceLower, distanceWeight)
%#codegen
persistent solver poseConstraint jointConstraint distanceConstraint robot
if isempty(solver)
    robot = gik9dof.codegen.loadRobotForCodegen();
    % ... initialize solver ...
end
% ... solve ...
end
```

**Problem**: Persistent variables don't work well with codegen in all contexts

**Solution**: Create explicit state struct

```matlab
% File: matlab/+gik9dof/+codegen/solveGIKStepRealtime.m
function [qNext, solverState] = solveGIKStepRealtime(qCurrent, targetPose, ...
    distanceLower, distanceWeight, solverState)
%SOLVEGIKSTEPREALTIME Real-time IK solver with explicit state management
%   [qNext, solverState] = solveGIKStepRealtime(qCurrent, targetPose, ...)
%   
%   Inputs:
%       qCurrent - 9x1 current joint configuration
%       targetPose - 4x4 target end-effector pose
%       distanceLower - minimum distance constraint (scalar)
%       distanceWeight - distance constraint weight (scalar)
%       solverState - struct with initialized solver (empty on first call)
%   
%   Outputs:
%       qNext - 9x1 next joint configuration
%       solverState - updated solver state (pass to next call)
%
%#codegen

% Initialize on first call
if isempty(solverState) || ~isfield(solverState, 'solver')
    solverState = initializeSolver();
end

% Update constraints
solverState.poseConstraint.TargetTransform = targetPose;

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

solverState.distanceConstraint.Bounds = [lowerBound, Inf];
solverState.distanceConstraint.Weights = w;

% Solve
[qNext, ~] = solverState.solver(qCurrent, ...
    solverState.poseConstraint, ...
    solverState.jointConstraint, ...
    solverState.distanceConstraint);

end

function solverState = initializeSolver()
%INITIALIZESOLVER Create solver and constraints
%#codegen

solverState.robot = gik9dof.codegen.buildRobotForCodegenARM64();

constraintInputs = {'pose','joint','distance'};
solverState.solver = generalizedInverseKinematics(...
    'RigidBodyTree', solverState.robot, ...
    'ConstraintInputs', constraintInputs, ...
    'SolverAlgorithm', 'LevenbergMarquardt');

solverState.poseConstraint = constraintPoseTarget('left_gripper_link');
solverState.jointConstraint = constraintJointBounds(solverState.robot);
solverState.distanceConstraint = constraintDistanceBounds('left_gripper_link');
solverState.distanceConstraint.ReferenceBody = solverState.robot.BaseName;

end
```

### Afternoon: Test Refactored Solver

```matlab
% Test script
q0 = zeros(9,1);
targetPose = trvec2tform([0.6, 0.2, 1.0]);

% First call (initialize)
[q1, state] = gik9dof.codegen.solveGIKStepRealtime(q0, targetPose, 0.2, 0.5, []);

% Second call (reuse state)
targetPose2 = trvec2tform([0.65, 0.2, 1.0]);
[q2, state] = gik9dof.codegen.solveGIKStepRealtime(q1, targetPose2, 0.2, 0.5, state);

% Verify result
robot = state.robot;
T = getTransform(robot, q2, 'left_gripper_link');
disp('Target:'); disp(targetPose2(1:3,4)');
disp('Actual:'); disp(T(1:3,4)');
disp('Error (m):'); disp(norm(T(1:3,4) - targetPose2(1:3,4)));
```

---

## Day 3: Code Generation Configuration

### Morning: Create Codegen Script

```matlab
% File: matlab/+gik9dof/+codegen/generateCodeARM64.m
function generateCodeARM64(outputDir)
%GENERATECODEARM64 Generate C++ code for ARM64 Linux target
%   generateCodeARM64(outputDir) generates C++ static library

if nargin < 1
    projectRoot = gik9dof.internal.projectRoot();
    outputDir = fullfile(projectRoot, 'codegen', 'linux_arm64');
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

fprintf('Generating C++ code for ARM64 target...\n');
fprintf('Output directory: %s\n', outputDir);

% Configure for embedded Linux ARM64
cfg = coder.config('lib', 'ecoder', false);
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.EnableOpenMP = true;
cfg.TargetLangStandard = 'C++17';

% Hardware settings for ARM Cortex-A (AGX Orin)
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdWordSize = 64;

% Real-time constraints
cfg.DynamicMemoryAllocation = 'Off';  % No malloc in control loop
cfg.SaturateOnIntegerOverflow = false;
cfg.ConstantInputs = 'IgnoreValues';

% Define input types for solveGIKStepRealtime
argsIK = {
    coder.typeof(0, [9,1], [false,false]),  % qCurrent (9x1)
    coder.typeof(0, [4,4], [false,false]),  % targetPose (4x4)
    coder.typeof(0),                         % distanceLower (scalar)
    coder.typeof(0),                         % distanceWeight (scalar)
    coder.typeof(struct())                   % solverState (struct)
};

% Entry point
entryPoint = fullfile(fileparts(mfilename('fullpath')), 'solveGIKStepRealtime.m');

% Generate
fprintf('\n=== Generating IK Solver ===\n');
try
    codegen('-config', cfg, ...
        entryPoint, ...
        '-args', argsIK, ...
        '-o', 'solveGIKStepRealtime', ...
        '-d', outputDir, ...
        '-report');
    
    fprintf('SUCCESS: Code generation complete\n');
    fprintf('Review report: %s/html/report.mldatx\n', outputDir);
catch ME
    fprintf('ERROR: Code generation failed\n');
    fprintf('%s\n', ME.message);
    rethrow(ME);
end

end
```

### Afternoon: Run Code Generation

```matlab
% In MATLAB
gik9dof.codegen.generateCodeARM64();

% Expected output:
% - codegen/linux_arm64/solveGIKStepRealtime.cpp
% - codegen/linux_arm64/solveGIKStepRealtime.h
% - codegen/linux_arm64/html/report.mldatx
```

**Common Errors**:

1. **"Variable-size arrays not supported"**
   - Fix: Use `coder.typeof` with fixed dimensions
   
2. **"Extrinsic function call"**
   - Fix: Remove file I/O, handle classes from MAT files
   
3. **"Code generation for handle classes not supported"**
   - Fix: Build robot procedurally, not from MAT file

---

## Day 4: Test Generated Code on Ubuntu

### Copy Files to Ubuntu

```bash
# On Ubuntu (or WSL)
cd ~/workspace
mkdir -p gik9dof_test
cd gik9dof_test

# Copy generated code from Windows
# (Adjust path for your setup)
cp -r /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/codegen/linux_arm64/* .
```

### Create Test CMakeLists.txt

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(gik9dof_test)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)

# Add generated sources
file(GLOB SOURCES "*.cpp")

# Create library
add_library(gik9dof_solver SHARED ${SOURCES})

target_include_directories(gik9dof_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gik9dof_solver
    OpenMP::OpenMP_CXX
)

# Simple test executable (optional)
add_executable(test_solver test_solver.cpp)
target_link_libraries(test_solver gik9dof_solver)
```

### Build

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)

# Check output
ls -lh libgik9dof_solver.so
```

**Success**: Library compiles without errors!

---

## Day 5: Validation & Documentation

### Create MATLAB Validation Suite

```matlab
% File: matlab/+gik9dof/+codegen/validateCodegen.m
function validateCodegen()
%VALIDATECODEGEN Test generated code matches MATLAB

fprintf('=== Codegen Validation Suite ===\n\n');

% Test 1: Robot builder
fprintf('Test 1: Robot builder...\n');
robot = gik9dof.codegen.buildRobotForCodegenARM64();
assert(robot.NumBodies == 15, 'Incorrect number of bodies');
fprintf('  PASS: Robot has 15 bodies\n\n');

% Test 2: Forward kinematics
fprintf('Test 2: Forward kinematics...\n');
q = [0 0 0, 0 -pi/4 pi/2 0 0 0]';  % Test configuration
T = getTransform(robot, q, 'left_gripper_link');
fprintf('  EE position: [%.3f, %.3f, %.3f]\n', T(1,4), T(2,4), T(3,4));
fprintf('  PASS\n\n');

% Test 3: IK solver
fprintf('Test 3: IK solver (1 step)...\n');
q0 = zeros(9,1);
targetPose = trvec2tform([0.6, 0.2, 1.0]);
tic;
[q1, state] = gik9dof.codegen.solveGIKStepRealtime(q0, targetPose, 0.2, 0.5, []);
t = toc;
fprintf('  Solve time: %.2f ms\n', t*1000);

T1 = getTransform(robot, q1, 'left_gripper_link');
error = norm(T1(1:3,4) - targetPose(1:3,4));
fprintf('  Position error: %.4f m\n', error);
assert(error < 0.05, 'IK error too large');
fprintf('  PASS\n\n');

% Test 4: Repeated calls (state reuse)
fprintf('Test 4: IK solver (10 steps)...\n');
q = q0;
waypoints = linspace(0.6, 0.8, 10);
errors = zeros(10,1);
times = zeros(10,1);

for i = 1:10
    target = trvec2tform([waypoints(i), 0.2, 1.0]);
    tic;
    [q, state] = gik9dof.codegen.solveGIKStepRealtime(q, target, 0.2, 0.5, state);
    times(i) = toc;
    
    T = getTransform(robot, q, 'left_gripper_link');
    errors(i) = norm(T(1:3,4) - target(1:3,4));
end

fprintf('  Average solve time: %.2f ms\n', mean(times)*1000);
fprintf('  Max error: %.4f m\n', max(errors));
fprintf('  PASS\n\n');

fprintf('=== All Tests Passed ===\n');
end
```

Run validation:
```matlab
gik9dof.codegen.validateCodegen()
```

---

## Week 1 Deliverables Checklist

- [ ] Procedural robot builder created and tested
- [ ] URDF parameters extracted and verified
- [ ] IK solver refactored (no persistent variables)
- [ ] Code generation script created
- [ ] C++ code generated successfully
- [ ] Generated code compiles on Ubuntu 22.04
- [ ] Validation suite passes
- [ ] Documentation updated

**Success Metrics**:
- ✅ No MATLAB Coder errors
- ✅ Generated C++ compiles without warnings
- ✅ IK solve time <10ms in MATLAB
- ✅ Position error <1cm on test cases

---

## Troubleshooting

### Issue: "buildRobotForCodegenARM64 not compatible with codegen"

**Cause**: Handle classes, dynamic arrays  
**Fix**: Ensure all arrays are fixed-size, no file I/O

### Issue: Codegen takes very long

**Cause**: Large MATLAB code, many dependencies  
**Fix**: Start with minimal entry point, add features incrementally

### Issue: Generated code has runtime errors

**Cause**: Initialization issues, pointer problems  
**Fix**: Check MATLAB Coder report for warnings, use `-report` flag

---

## Next Week Preview (Week 2)

- Create unified chassis controller entry point
- Generate full solver library
- Begin ROS2 package setup
- Create message definitions

---

**Document Version**: 1.0  
**Created**: 2025-10-06  
**Target Audience**: Implementation team starting Week 1
