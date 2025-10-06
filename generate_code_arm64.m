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

% ARM64 specific settings
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.Hardware.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';

% Optimization settings for real-time performance
cfg.OptimizationLevel = 'O3';
cfg.EnableOpenMP = true;
cfg.InstructionSetExtensions = 'NEON';

% Memory settings for real-time safety
cfg.DynamicMemoryAllocation = 'Off';
cfg.MaxStackSize = 1048576; % 1 MB stack

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.GenCodeOnly = false;
cfg.GenerateMakefile = true;

% Build configuration
cfg.BuildConfiguration = 'Faster Runs';
cfg.Toolchain = 'CMake';
cfg.EnableDebugging = false;

%% Code generation execution
fprintf('===================================================\n');
fprintf('Generating C++ code for ARM64 (NVIDIA AGX Orin)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_realtime.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Configuration:\n');
fprintf('  - Language: C++17\n');
fprintf('  - Architecture: ARM64 (Cortex-A)\n');
fprintf('  - SIMD: NEON\n');
fprintf('  - OpenMP: Enabled\n');
fprintf('  - Dynamic Memory: Disabled\n');
fprintf('  - Optimization: O3\n');
fprintf('===================================================\n\n');

% Generate code
codegen('-config', cfg, ...
    'gik9dof.codegen_realtime.solveGIKStepWrapper', ...
    '-args', {qCurrent, targetPose, distanceLower, distanceWeight}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');

fprintf('\n===================================================\n');
fprintf('✓ Code generation successful!\n');
fprintf('===================================================\n');
fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
fprintf('===================================================\n');
