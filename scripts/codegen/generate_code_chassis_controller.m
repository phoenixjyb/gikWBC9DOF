%% generate_code_chassis_controller.m
% Code generation for Multi-Mode Chassis Controller targeting NVIDIA AGX Orin (ARM64)
%
% This script generates optimized C++ code for deployment on:
%   - NVIDIA AGX Orin (ARM Cortex-A78AE, ARM64 with NEON SIMD)
%   - Real-time performance target: < 10ms controller update
%   - Integration with existing ROS2 control nodes
%
% Author: WHEELTEC Mobile Manipulator Project
% Date: 2025-10-10 (MATLAB Feature Integration)

clear; clc;

try
%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
% Go up two levels: scripts/codegen -> scripts -> root
workspaceRoot = fileparts(fileparts(projectRoot));
matlabRoot = fullfile(workspaceRoot, 'matlab');
addpath(genpath(matlabRoot));

CODEGEN_OUTPUT = fullfile(workspaceRoot, 'codegen', 'chassis_controller_arm64');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation
% simulateChassisController(pathStates, options)

% pathStates: Nx3 [x y yaw] - Variable-length path
% Using fixed maximum for codegen (500 waypoints = typical 50m path @ 0.1m spacing)
MAX_PATH_WAYPOINTS = 500;
pathStates = coder.typeof(double(0), [MAX_PATH_WAYPOINTS, 3], [true, false]);

% options struct with fields:
%   SampleTime (1,1) double
%   ControllerMode (1,1) double
%   FollowerOptions struct
options = struct();
options.SampleTime = 0.1;
options.ControllerMode = 2.0;

% FollowerOptions contains ChassisParams
chassisParams = struct();
chassisParams.track = 0.573;
chassisParams.vx_max = 1.5;
chassisParams.vx_min = -0.4;
chassisParams.wz_max = 2.5;
chassisParams.wheel_speed_max = 3.3;
chassisParams.reverse_enabled = false;
chassisParams.lookahead_base = 0.6;
chassisParams.lookahead_gain = 0.3;
chassisParams.goal_tolerance = 0.1;
chassisParams.heading_kp = 1.2;
chassisParams.feedforward_gain = 0.9;

options.FollowerOptions = struct();
options.FollowerOptions.ChassisParams = chassisParams;

fprintf('===================================================\n');
fprintf('Input Type Specification\n');
fprintf('===================================================\n');
fprintf('pathStates: double[%d×3] (variable-length up to %d)\n', MAX_PATH_WAYPOINTS, MAX_PATH_WAYPOINTS);
fprintf('  [x y yaw] path waypoints\n');
fprintf('options: struct with fields:\n');
fprintf('  .SampleTime: scalar (integration step)\n');
fprintf('  .ControllerMode: scalar (0=diff, 1=heading, 2=pure pursuit)\n');
fprintf('  .FollowerOptions: struct with ChassisParams\n\n');

%% Create coder configuration for C++ library (ARM64)
cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'ChassisController';

% ARM64-specific hardware settings for NVIDIA AGX Orin
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdLongLongMode = true;

% Memory settings - CRITICAL
% Enable dynamic allocation for:
%   - Variable-length path arrays
%   - Controller state tracking
%   - Result struct arrays
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536;

% Build settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.GenCodeOnly = true;

% Optimization for real-time performance
cfg.EnableOpenMP = true;
cfg.SaturateOnIntegerOverflow = false;

% File management
cfg.FilePartitionMethod = 'SingleFile';
cfg.MaxIdLength = 63;

fprintf('===================================================\n');
fprintf('Coder Configuration\n');
fprintf('===================================================\n');
fprintf('Target Language:    %s\n', cfg.TargetLang);
fprintf('Namespace:          %s\n', cfg.CppNamespace);
fprintf('Class Name:         %s\n', cfg.CppInterfaceClassName);
fprintf('Hardware:           ARM Cortex-A (ARM64 with NEON)\n');
fprintf('OpenMP:             %s\n', mat2str(cfg.EnableOpenMP));
fprintf('Dynamic Memory:     %s (threshold: %d bytes)\n', ...
    mat2str(cfg.EnableDynamicMemoryAllocation), cfg.DynamicMemoryAllocationThreshold);
fprintf('Output Directory:   %s\n\n', CODEGEN_OUTPUT);

%% Run code generation
fprintf('===================================================\n');
fprintf('Generating C++ code for Chassis Controller...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.control.simulateChassisController\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: Multi-mode chassis control for NVIDIA AGX Orin\n');
fprintf('===================================================\n\n');

% Find target file
targetFile = fullfile(matlabRoot, '+gik9dof', '+control', 'simulateChassisController.m');
if ~isfile(targetFile)
    error('Target file not found: %s', targetFile);
end
fprintf('Target file: %s\n', targetFile);

fprintf('Generating code...\n');
tic;

codegen -config cfg ...
    gik9dof.control.simulateChassisController ...
    -args {pathStates, options} ...
    -d CODEGEN_OUTPUT ...
    -report

elapsed = toc;

fprintf('\n');
fprintf('===================================================\n');
fprintf('Code Generation Complete\n');
fprintf('===================================================\n');
fprintf('Time elapsed: %.1f seconds\n', elapsed);
fprintf('Output location: %s\n', CODEGEN_OUTPUT);

% List generated files
generatedFiles = dir(fullfile(CODEGEN_OUTPUT, '*.{cpp,h,hpp}'));
fprintf('\nGenerated files:\n');
for i = 1:length(generatedFiles)
    fprintf('  %s\n', generatedFiles(i).name);
end

fprintf('\n✓ Chassis controller code generation successful!\n\n');

catch ME
    fprintf('\n');
    fprintf('===================================================\n');
    fprintf('ERROR: Code Generation Failed\n');
    fprintf('===================================================\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    fprintf('\n');
    rethrow(ME);
end
