%% generate_code_x86_64.m
% Wrapper script for x86_64 code generation (WSL validation)
% This is at project root to avoid package namespace issues

%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'matlab')));

CODEGEN_OUTPUT = fullfile(projectRoot, 'codegen', 'x86_64_validation');

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

%% Create coder configuration for C++ library (x86_64)
cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';

% x86_64-specific hardware settings
% Configure for Intel/AMD x86_64 with SSE/AVX SIMD
cfg.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';
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
fprintf('Generating C++ code for x86_64 (WSL Validation)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: Validation/testing on WSL Ubuntu x86_64\n');
fprintf('Configuration:\n');
fprintf('  - Language: C++17\n');
fprintf('  - Architecture: Intel x86-64 (Linux 64-bit)\n');
fprintf('  - SIMD: SSE/AVX (x86_64)\n');
fprintf('  - OpenMP: Enabled\n');
fprintf('  - Dynamic Memory: Enabled (required for IK constraints)\n');
fprintf('  - Dynamic Memory Threshold: 64KB\n');
fprintf('  - Build Tool: CMake\n');
fprintf('  - Solver MaxTime: 50ms (for testing)\n');
fprintf('  - Solver MaxIterations: 50\n');
fprintf('Note: This build is for VALIDATION, not real-time deployment\n');
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
