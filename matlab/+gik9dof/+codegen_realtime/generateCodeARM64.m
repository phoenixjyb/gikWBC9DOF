%% generateCodeARM64.m
% MATLAB Coder script for generating C++ code for ARM64 (NVIDIA AGX Orin)
% Target: Ubuntu 22.04, ROS2 Humble, ARM64 architecture
%
% Prerequisites:
%   - MATLAB R2024b with MATLAB Coder
%   - Robotics System Toolbox
%   - Embedded Coder (for better optimization)
%
% This script generates a standalone C++ library that can be wrapped
% by ROS2 nodes for real-time inverse kinematics on AGX Orin.

clear; clc;

%% Configuration
PROJECT_ROOT = fileparts(mfilename('fullpath'));
CODEGEN_OUTPUT = fullfile(PROJECT_ROOT, '..', '..', 'codegen', 'arm64_realtime');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation
% All inputs must be strongly typed for code generation

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
cfg.EnableOpenMP = true; % Parallel processing for matrix operations
cfg.InstructionSetExtensions = 'NEON'; % ARM NEON SIMD

% Memory settings for real-time safety
cfg.DynamicMemoryAllocation = 'Off'; % No dynamic allocation in critical path
cfg.MaxStackSize = 1048576; % 1 MB stack

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.GenCodeOnly = false; % Generate .a library file
cfg.PackageType = 'ROS2Package'; % Generate ROS2-compatible package (if toolbox supports)
cfg.GenerateMakefile = true;

% Verification settings
cfg.SaturateOnIntegerOverflow = true;
cfg.RuntimeChecks = false; % Disable for production (enable for debug)
cfg.IntegrityChecks = false;
cfg.ResponsivenessChecks = false;

% File naming
cfg.CustomSource = [];
cfg.CustomInclude = [];
cfg.CustomLibrary = [];

% Build configuration
cfg.BuildConfiguration = 'Faster Runs';
cfg.Toolchain = 'CMake';
cfg.EnableDebugging = false;
cfg.EnableMemcheck = false;
cfg.EnableAddressSanitizer = false;

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

try
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
    fprintf('\nNext steps:\n');
    fprintf('1. Copy generated code to ROS2 workspace\n');
    fprintf('2. Create ROS2 wrapper node (gik9dof_solver_node)\n');
    fprintf('3. Build on AGX Orin with colcon\n');
    fprintf('4. Test with real robot\n');
    fprintf('===================================================\n');
    
    % List generated files
    fprintf('\nGenerated files:\n');
    generatedFiles = dir(fullfile(CODEGEN_OUTPUT, '**', '*.*'));
    for i = 1:length(generatedFiles)
        if ~generatedFiles(i).isdir
            fprintf('  - %s\n', generatedFiles(i).name);
        end
    end
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('✗ Code generation failed!\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Location: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    fprintf('\nTroubleshooting:\n');
    fprintf('1. Check MATLAB version (requires R2024b)\n');
    fprintf('2. Verify Robotics System Toolbox installation\n');
    fprintf('3. Ensure all dependencies are on path\n');
    fprintf('4. Review code generation report for details\n');
    fprintf('===================================================\n');
    rethrow(ME);
end

%% Generate test harness (optional)
fprintf('\nGenerating test harness...\n');
testFile = fullfile(CODEGEN_OUTPUT, 'test_gik_solver.m');
fid = fopen(testFile, 'w');
fprintf(fid, '%% Test harness for generated C++ code\n');
fprintf(fid, 'function test_gik_solver()\n');
fprintf(fid, '    %% Test configuration\n');
fprintf(fid, '    q0 = zeros(9, 1);\n');
fprintf(fid, '    targetPose = eye(4);\n');
fprintf(fid, '    targetPose(1:3, 4) = [0.5; 0.2; 0.8];\n');
fprintf(fid, '    distanceLower = 0.1;\n');
fprintf(fid, '    distanceWeight = 1.0;\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Call solver\n');
fprintf(fid, '    [qNext, info] = gik9dof.codegen_realtime.solveGIKStepWrapper(q0, targetPose, distanceLower, distanceWeight);\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Display results\n');
fprintf(fid, '    fprintf(''Solution found: %%d\\n'', info.Status);\n');
fprintf(fid, '    fprintf(''Iterations: %%d\\n'', info.Iterations);\n');
fprintf(fid, '    fprintf(''Pose error: %%.6f\\n'', info.PoseErrorNorm);\n');
fprintf(fid, '    disp(''Joint configuration:'');\n');
fprintf(fid, '    disp(qNext);\n');
fprintf(fid, 'end\n');
fclose(fid);
fprintf('✓ Test harness saved: %s\n', testFile);

fprintf('\n===================================================\n');
fprintf('Code generation complete!\n');
fprintf('===================================================\n');
