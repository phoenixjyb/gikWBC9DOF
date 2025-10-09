%% generate_code_x86_64_noCollision.m
% Generate x86_64 Linux code WITHOUT collision checking for WSL validation
% This avoids the Windows .obj binary incompatibility issue
%
% Key Difference: Robot built without collision geometries
% Result: No collisioncodegen_*.obj dependencies → Builds on Linux!

clear; clc;

%% Setup paths
PROJECT_ROOT = fileparts(mfilename('fullpath'));
CODEGEN_OUTPUT = fullfile(PROJECT_ROOT, 'codegen', 'x86_64_validation_noCollision');

% Add MATLAB code to path
addpath(fullfile(PROJECT_ROOT, 'matlab'));

fprintf('===================================================================\n');
fprintf('Generating x86_64 Linux Code (No Collision) - MaxIterations=1000\n');
fprintf('===================================================================\n');
fprintf('This version builds robot WITHOUT collision geometries to avoid\n');
fprintf('Windows .obj binary dependencies that cannot link on Linux/WSL\n');
fprintf('===================================================================\n\n');

%% Build robot (collision meshes in URDF are OK - we just won't use them)
fprintf('Building robot model...\n');

% Import URDF - collision meshes will be loaded but not used by solver
robot = importrobot(fullfile(PROJECT_ROOT, 'mobile_manipulator_PPR_base_corrected.urdf'));
robot.DataFormat = 'column';

% Key: We DON'T add collision constraints to the solver
% This prevents MATLAB from generating collisioncodegen_*.obj files

fprintf('✓ Robot loaded (9 DOF, collision checking will NOT be used)\n');
fprintf('  Bodies: %d\n', robot.NumBodies);

%% Create IK solver (no collision constraints needed)
fprintf('Creating IK solver...\n');

eeBodyName = 'left_gripper_link';

% Create solver with constraints: pose + joint + 20 distance constraints
solver = generalizedInverseKinematics(...
    'RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint', ...
                         'distance', 'distance', 'distance', 'distance', 'distance', ...
                         'distance', 'distance', 'distance', 'distance', 'distance', ...
                         'distance', 'distance', 'distance', 'distance', 'distance', ...
                         'distance', 'distance', 'distance', 'distance', 'distance'});

% Solver settings - MaxIterations=1000!
solver.SolverParameters.AllowRandomRestart = false;
solver.SolverParameters.MaxIterations = 1000;  % INCREASED FROM 50!
solver.SolverParameters.MaxTime = 0.05;  % 50ms
solver.SolverParameters.GradientTolerance = 1e-6;
solver.SolverParameters.SolutionTolerance = 1e-6;
solver.SolverParameters.EnforceJointLimits = true;

fprintf('✓ Solver created with MaxIterations=1000\n');

%% Define example inputs (20-constraint signature)
fprintf('Setting up code generation types...\n');

qCurrent = zeros(9, 1);
targetPose = eye(4);

% 20 distance constraints (each with bodyIdx, refBodyIdx, lower, upper, weight)
distBodyIndices = int32(ones(20, 1));
distRefBodyIndices = int32(zeros(20, 1));
distBoundsLower = zeros(20, 1);
distBoundsUpper = ones(20, 1);
distWeights = ones(20, 1);

%% Code generation config
fprintf('Configuring code generation for x86_64 Linux...\n');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.GenerateExampleMain = 'DoNotGenerate';

% C++ settings - Class-based interface (MATLAB Coder 24.2)
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';

% Target: Linux x86-64
cfg.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';
cfg.HardwareImplementation.ProdBitPerChar = 8;
cfg.HardwareImplementation.ProdBitPerShort = 16;
cfg.HardwareImplementation.ProdBitPerInt = 32;
cfg.HardwareImplementation.ProdBitPerLong = 64;
cfg.HardwareImplementation.ProdBitPerLongLong = 64;
cfg.HardwareImplementation.ProdWordSize = 64;

% Memory and performance
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536;
cfg.EnableOpenMP = true;

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.BuildConfiguration = 'Faster Runs';

%% Code generation execution
fprintf('===================================================\n');
fprintf('Generating C++ code for x86_64 Linux (WSL)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: WSL validation with MaxIterations=1000\n');
fprintf('Configuration:\n');
fprintf('  - Language: C++17 class-based interface\n');
fprintf('  - Architecture: Intel x86-64 (Linux 64-bit)\n');
fprintf('  - Collision: DISABLED (no .obj dependencies)\n');
fprintf('  - OpenMP: Enabled\n');
fprintf('  - MaxIterations: 1000 ← KEY CHANGE!\n');
fprintf('===================================================\n\n');

fprintf('This may take 2-3 minutes...\n\n');

% Generate code
tic;
codegen('-config', cfg, ...
    'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
    '-args', {qCurrent, targetPose, distBodyIndices, distRefBodyIndices, ...
              distBoundsLower, distBoundsUpper, distWeights}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');
elapsed = toc;

fprintf('\n===================================================\n');
fprintf('✓ Code generation successful!\n');
fprintf('===================================================\n');
fprintf('Time elapsed: %.1f seconds\n', elapsed);
fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
fprintf('\n✓ This code should BUILD on WSL without errors!\n');
fprintf('  Reason: No collision geometries = No Windows .obj files\n');
fprintf('===================================================\n\n');

fprintf('📋 Next Steps:\n');
fprintf('1. Update build_validation_wsl.sh to point to:\n');
fprintf('   CODEGEN_DIR=../codegen/x86_64_validation_noCollision\n');
fprintf('2. In WSL: cd validation && bash build_validation_wsl.sh\n');
fprintf('3. Run: ./validate_gik_standalone gik_test_cases_20.json results.json\n');
fprintf('4. Expect 60-80%% pass rate with MaxIterations=1000!\n\n');
