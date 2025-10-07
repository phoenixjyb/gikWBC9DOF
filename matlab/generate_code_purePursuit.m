%% Generate C++ Code for Pure Pursuit Velocity Controller
% Generates ARM64 (Orin) and x86_64 (WSL) C++ implementations
%
% Configuration:
%   - Path buffer: 30 waypoints
%   - Update rate: 100 Hz (dt = 0.01s)
%   - Max speed: 1.5 m/s
%   - Adaptive lookahead
%   - Interpolation for smooth path

clear; clc;

fprintf('Pure Pursuit Velocity Controller - Code Generation\n');
fprintf('===================================================\n\n');

%% Change to matlab directory
originalDir = pwd;

% Check if we're already in the matlab directory
if contains(pwd, 'matlab') && exist('purePursuitVelocityController.m', 'file')
    scriptPath = pwd;
else
    [scriptPath, ~, ~] = fileparts(mfilename('fullpath'));
    cd(scriptPath);
end

fprintf('Working directory: %s\n\n', pwd);

%% Add necessary paths
% Go up one level to workspace root
addpath(fullfile(pwd, '..'));
addpath(fullfile(pwd, '..', 'matlab'));
addpath(fullfile(pwd, '+gik9dof', '+control'));

fprintf('Added paths for code generation\n\n');

%% Define input types
fprintf('Defining input types for MATLAB Coder...\n');

% Scalar inputs
refX_type = coder.typeof(0.0);
refY_type = coder.typeof(0.0);
refTheta_type = coder.typeof(0.0);
refTime_type = coder.typeof(0.0);
estX_type = coder.typeof(0.0);
estY_type = coder.typeof(0.0);
estYaw_type = coder.typeof(0.0);

% Parameters struct
params_type = struct();
params_type.lookaheadBase = coder.typeof(0.0);
params_type.lookaheadVelGain = coder.typeof(0.0);
params_type.lookaheadTimeGain = coder.typeof(0.0);
params_type.vxNominal = coder.typeof(0.0);
params_type.vxMax = coder.typeof(0.0);
params_type.vxMin = coder.typeof(0.0);  % ADDED: Bidirectional support
params_type.wzMax = coder.typeof(0.0);
params_type.track = coder.typeof(0.0);
params_type.vwheelMax = coder.typeof(0.0);
params_type.waypointSpacing = coder.typeof(0.0);
params_type.pathBufferSize = coder.typeof(0.0);
params_type.goalTolerance = coder.typeof(0.0);
params_type.interpSpacing = coder.typeof(0.0);

% State struct (fixed-size arrays for 30 waypoints)
state_type = struct();
state_type.pathX = coder.typeof(zeros(1, 30));
state_type.pathY = coder.typeof(zeros(1, 30));
state_type.pathTheta = coder.typeof(zeros(1, 30));
state_type.pathTime = coder.typeof(zeros(1, 30));
state_type.numWaypoints = coder.typeof(0);
state_type.prevVx = coder.typeof(0.0);
state_type.prevWz = coder.typeof(0.0);
state_type.prevPoseX = coder.typeof(0.0);
state_type.prevPoseY = coder.typeof(0.0);
state_type.prevPoseYaw = coder.typeof(0.0);
state_type.lastRefTime = coder.typeof(0.0);

fprintf('Input types defined\n\n');

%% ARM64 Code Generation (for Orin)
fprintf('==================================================\n');
fprintf('Generating ARM64 code for Jetson Orin...\n');
fprintf('==================================================\n');

outputDir_arm64 = fullfile(pwd, 'codegen', 'purepursuit_arm64');

cfg_arm64 = coder.config('lib');
cfg_arm64.TargetLang = 'C++';
cfg_arm64.CppNamespace = 'gik9dof_purepursuit';
cfg_arm64.GenCodeOnly = true;
cfg_arm64.GenerateReport = true;

% ARM64-specific optimizations
cfg_arm64.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg_arm64.Toolchain = 'CMake';
cfg_arm64.BuildConfiguration = 'Faster Runs';

% Memory and performance settings
cfg_arm64.EnableVariableSizing = false;
cfg_arm64.DynamicMemoryAllocation = 'Off';
cfg_arm64.EnableOpenMP = false;

fprintf('\nARM64 Config:\n');
fprintf('  Target: %s\n', cfg_arm64.TargetLang);
fprintf('  Namespace: %s\n', cfg_arm64.CppNamespace);
fprintf('  Hardware: ARM Cortex-A\n');
fprintf('  Build: %s\n', cfg_arm64.BuildConfiguration);
fprintf('  Dynamic Memory: %s\n', cfg_arm64.DynamicMemoryAllocation);
fprintf('\nStarting ARM64 codegen...\n');

try
    codegen -config cfg_arm64 purePursuitVelocityController -args {refX_type, refY_type, refTheta_type, refTime_type, estX_type, estY_type, estYaw_type, params_type, state_type} -d codegen/purepursuit_arm64 -report
    fprintf('✓ ARM64 code generation completed successfully!\n');
    fprintf('  Output: %s\n\n', outputDir_arm64);
catch ME
    fprintf('✗ ARM64 code generation FAILED:\n');
    fprintf('  Error: %s\n', ME.message);
    fprintf('  Location: %s (line %d)\n\n', ME.stack(1).file, ME.stack(1).line);
    cd(originalDir);
    rethrow(ME);
end

%% x86_64 Code Generation (for WSL)
fprintf('==================================================\n');
fprintf('Generating x86_64 code for WSL...\n');
fprintf('==================================================\n');

outputDir_x64 = fullfile(pwd, 'codegen', 'purepursuit_x64');

cfg_x64 = coder.config('lib');
cfg_x64.TargetLang = 'C++';
cfg_x64.CppNamespace = 'gik9dof_purepursuit';
cfg_x64.GenCodeOnly = true;
cfg_x64.GenerateReport = true;

% x86_64-specific settings
cfg_x64.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';
cfg_x64.Toolchain = 'CMake';
cfg_x64.BuildConfiguration = 'Faster Runs';

% Memory and performance settings
cfg_x64.EnableVariableSizing = false;
cfg_x64.DynamicMemoryAllocation = 'Off';
cfg_x64.EnableOpenMP = false;

fprintf('\nx86_64 Config:\n');
fprintf('  Target: %s\n', cfg_x64.TargetLang);
fprintf('  Namespace: %s\n', cfg_x64.CppNamespace);
fprintf('  Hardware: x86-64 Linux\n');
fprintf('  Build: %s\n', cfg_x64.BuildConfiguration);
fprintf('  Dynamic Memory: %s\n', cfg_x64.DynamicMemoryAllocation);
fprintf('\nStarting x86_64 codegen...\n');

try
    codegen -config cfg_x64 purePursuitVelocityController -args {refX_type, refY_type, refTheta_type, refTime_type, estX_type, estY_type, estYaw_type, params_type, state_type} -d codegen/purepursuit_x64 -report
    fprintf('✓ x86_64 code generation completed successfully!\n');
    fprintf('  Output: %s\n\n', outputDir_x64);
catch ME
    fprintf('✗ x86_64 code generation FAILED:\n');
    fprintf('  Error: %s\n', ME.message);
    fprintf('  Location: %s (line %d)\n\n', ME.stack(1).file, ME.stack(1).line);
    cd(originalDir);
    rethrow(ME);
end

%% Summary
fprintf('==================================================\n');
fprintf('Code Generation Summary\n');
fprintf('==================================================\n');
fprintf('✓ ARM64 (Orin):  %s\n', outputDir_arm64);
fprintf('✓ x86_64 (WSL):  %s\n', outputDir_x64);
fprintf('\nGenerated files include:\n');
fprintf('  - purePursuitVelocityController.h/cpp\n');
fprintf('  - rtwtypes.h\n');
fprintf('  - Pure Pursuit algorithm implementation\n');
fprintf('\nSpecifications:\n');
fprintf('  - Path buffer: 30 waypoints max\n');
fprintf('  - Update rate: 100 Hz (10ms period)\n');
fprintf('  - Max speed: 1.5 m/s\n');
fprintf('  - Adaptive lookahead: L = L_base + k_v*vx + k_t*dt\n');
fprintf('  - Interpolation: Linear, configurable spacing\n');
fprintf('  - Continuous reference acceptance\n');
fprintf('\nNext steps:\n');
fprintf('  1. Copy generated code to ROS2 package\n');
fprintf('  2. Update CMakeLists.txt\n');
fprintf('  3. Integrate into gik9dof_solver_node.cpp\n');
fprintf('  4. Build and test on WSL\n');
fprintf('  5. Deploy to Orin\n');
fprintf('==================================================\n');

%% Restore original directory
cd(originalDir);
fprintf('\nReturned to: %s\n', pwd);
