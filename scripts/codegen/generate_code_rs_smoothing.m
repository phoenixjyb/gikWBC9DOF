%% generate_code_rs_smoothing.m
% Code generation for RS Clothoid Path Smoothing targeting NVIDIA AGX Orin (ARM64)
%
% This script generates optimized C++ code for deployment on:
%   - NVIDIA AGX Orin (ARM Cortex-A78AE, ARM64 with NEON SIMD)
%   - Real-time or offline path smoothing
%   - Integration with path planning pipeline
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

CODEGEN_OUTPUT = fullfile(workspaceRoot, 'codegen', 'rs_smoothing_arm64');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation
% rsClothoidRefine(pathIn, params)

% pathIn: Nx3 [x y yaw] - Variable-length path
% Using fixed maximum for codegen (1000 waypoints = typical 100m path @ 0.1m spacing)
MAX_PATH_WAYPOINTS = 1000;
pathIn = coder.typeof(double(0), [MAX_PATH_WAYPOINTS, 3], [true, false]);

% params struct with fields:
%   discretizationDistance - Output spacing (default 0.05 m)
%   maxNumWaypoints - Max waypoints for referencePathFrenet (default 0 => auto)
params = struct();
params.discretizationDistance = 0.05;
params.maxNumWaypoints = int32(0);

fprintf('===================================================\n');
fprintf('Input Type Specification\n');
fprintf('===================================================\n');
fprintf('pathIn: double[%d×3] (variable-length up to %d)\n', MAX_PATH_WAYPOINTS, MAX_PATH_WAYPOINTS);
fprintf('  [x y yaw] raw path waypoints\n');
fprintf('params: struct with fields:\n');
fprintf('  .discretizationDistance: scalar (output spacing in meters)\n');
fprintf('  .maxNumWaypoints: int32 (max waypoints, 0=auto)\n\n');

%% Create coder configuration for C++ library (ARM64)
cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'RSClothoidSmoother';

% ARM64-specific hardware settings for NVIDIA AGX Orin
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdLongLongMode = true;

% Memory settings - CRITICAL
% Enable dynamic allocation for:
%   - Variable-length path arrays
%   - Clothoid spline fitting
%   - Segment processing
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536;

% Build settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.GenCodeOnly = true;

% Optimization
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
fprintf('Generating C++ code for RS Clothoid Smoothing...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.control.rsClothoidRefine\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: Clothoid-based path smoothing for NVIDIA AGX Orin\n');
fprintf('===================================================\n\n');

% Find target file
targetFile = fullfile(matlabRoot, '+gik9dof', '+control', 'rsClothoidRefine.m');
if ~isfile(targetFile)
    error('Target file not found: %s', targetFile);
end
fprintf('Target file: %s\n', targetFile);

fprintf('Generating code...\n');
tic;

codegen -config cfg ...
    fullfile(matlabRoot, '+gik9dof', '+control', 'rsClothoidRefine.m') ...
    -args {pathIn, params} ...
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

fprintf('\n✓ RS clothoid smoothing code generation successful!\n\n');

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
