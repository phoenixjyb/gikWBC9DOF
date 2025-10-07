%% generate_code_arm64.m
% Wrapper script for ARM64 code generation
% This is at project root to avoid package namespace issues

%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
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

% distanceLower: scalar lower bound for distance constraint
distanceLower = coder.typeof(double(0));

% distanceWeight: scalar weight for distance constraint
distanceWeight = coder.typeof(double(0));

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
fprintf('  - Solver MaxIterations: 50\n');
fprintf('===================================================\n\n');

% Generate code
codegen('-config', cfg, ...
    'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
    '-args', {qCurrent, targetPose, distanceLower, distanceWeight}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');

fprintf('\n===================================================\n');
fprintf('✓ Code generation successful!\n');
fprintf('===================================================\n');
fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
fprintf('===================================================\n');
