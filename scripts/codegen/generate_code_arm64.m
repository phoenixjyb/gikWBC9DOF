%% generate_code_arm64.m
% Wrapper script for ARM64 code generation
% This is in scripts/codegen/ directory

%% Configuration
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));  % Go up two levels: scripts/codegen -> scripts -> root
addpath(genpath(fullfile(projectRoot, 'matlab')));

CODEGEN_OUTPUT = fullfile(projectRoot, 'codegen', 'arm64_realtime');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation
% qCurrent: 9x1 joint configuration
qCurrent = coder.typeof(double(0), [9 1]);

% targetPose: 4x4 homogeneous transformation matrix
targetPose = coder.typeof(double(0), [4 4]);

% distBodyIndices: 20x1 int32 array (body indices for distance constraints)
distBodyIndices = coder.typeof(int32(0), [20 1]);

% distRefBodyIndices: 20x1 int32 array (reference body indices)
distRefBodyIndices = coder.typeof(int32(0), [20 1]);

% distBoundsLower: 20x1 double array (lower bounds for distance constraints)
distBoundsLower = coder.typeof(double(0), [20 1]);

% distBoundsUpper: 20x1 double array (upper bounds for distance constraints)
distBoundsUpper = coder.typeof(double(0), [20 1]);

% distWeights: 20x1 double array (weights for distance constraints)
distWeights = coder.typeof(double(0), [20 1]);

%% Create coder configuration for C++ library (ARM64)
cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';

% ARM64-specific hardware settings
% Configure for ARM NEON SIMD (not x86 SSE)
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdLongLongMode = true;

% Memory settings
% Enable dynamic allocation for MATLAB handle classes (constraints, solver)
% This is necessary for generalizedInverseKinematics and constraint objects
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536; % 64KB threshold
cfg.EnableOpenMP = true;

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;

% Build configuration
cfg.BuildConfiguration = 'Faster Runs';

%% Code generation execution
fprintf('===================================================\n');
fprintf('Generating C++ code for ARM64 (NVIDIA AGX Orin - REAL-TIME)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: Real-time deployment on NVIDIA AGX Orin\n');
fprintf('Configuration:\n');
fprintf('  - Language: C++17\n');
fprintf('  - Architecture: ARM Cortex-A (ARM64 with NEON SIMD)\n');
fprintf('  - SIMD: ARM NEON (not x86 SSE)\n');
fprintf('  - OpenMP: Enabled\n');
fprintf('  - Dynamic Memory: Enabled (required for IK constraints)\n');
fprintf('  - Dynamic Memory Threshold: 64KB\n');
fprintf('  - Build Tool: CMake\n');
fprintf('  - Solver MaxTime: 50ms (REAL-TIME CONSTRAINT)\n');
fprintf('  - Solver MaxIterations: 1000 ← UPDATED!\n');
fprintf('===================================================\n\n');

% Generate code using full file path (not namespace syntax) for WSL compatibility
matlabRoot = fullfile(projectRoot, 'matlab');
solver_file = fullfile(matlabRoot, '+gik9dof', '+codegen_inuse', 'solveGIKStepWrapper.m');
fprintf('Target file: %s\n', solver_file);
fprintf('Generating code...\n\n');

codegen('-config', cfg, ...
    solver_file, ...
    '-args', {qCurrent, targetPose, distBodyIndices, distRefBodyIndices, ...
              distBoundsLower, distBoundsUpper, distWeights}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');

fprintf('\n===================================================\n');
fprintf('✓ Code generation successful!\n');
fprintf('===================================================\n');
fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
fprintf('===================================================\n');
