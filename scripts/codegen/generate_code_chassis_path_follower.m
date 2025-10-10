%% generate_code_chassis_path_follower.m
% Code generation for chassisPathFollowerCodegen targeting NVIDIA AGX Orin (ARM64)
%
% This script generates optimized C++ code for the new function-based chassis
% path follower with 3 controller modes (0: differentiation, 1: heading, 2: pure pursuit)
%
% Target: NVIDIA AGX Orin (ARM Cortex-A78AE, ARM64 with NEON SIMD)
% Performance: < 5ms controller update (single step)
%
% Author: WHEELTEC Mobile Manipulator Project
% Date: 2025-10-10

clear; clc;

try
%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
workspaceRoot = fileparts(fileparts(projectRoot));
matlabRoot = fullfile(workspaceRoot, 'matlab');
addpath(genpath(matlabRoot));

CODEGEN_OUTPUT = fullfile(workspaceRoot, 'codegen', 'chassis_path_follower_arm64');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Chassis Path Follower - Code Generation for ARM64\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('Function: chassisPathFollowerCodegen\n');
fprintf('Modes: 0 (differentiation), 1 (heading), 2 (pure pursuit)\n');
fprintf('Output: %s\n', CODEGEN_OUTPUT);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% Define input types for code generation
% [vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params)

fprintf('Step 1: Define Input Types\n');
fprintf('───────────────────────────────────────────────────────────────\n');

% Input 1: pose - [x, y, theta] (1×3 double)
pose_type = coder.typeof(double(0), [1, 3], [false, false]);
fprintf('✓ pose: double[1×3] (fixed size)\n');

% Input 2: dt - scalar double (time step)
dt_type = coder.typeof(double(0));
fprintf('✓ dt: double (scalar)\n');

% Input 3: state - struct with fields
state_type = coder.typeof(struct(...
    'PathNumPoints', 0, ...
    'CurrentIndex', 0, ...
    'LastVelocity', 0.0, ...
    'LastAcceleration', 0.0, ...
    'LastHeadingError', 0.0, ...
    'IntegralHeadingError', 0.0, ...
    'PreviousPose', zeros(1,3), ...
    'DistanceTraveled', 0.0));
fprintf('✓ state: struct (8 fields)\n');

% Input 4: params - struct with all parameters
% Maximum path size for codegen
MAX_PATH_POINTS = 500;

% Create nested Chassis struct
Chassis = struct();
Chassis.track = 0.573;
Chassis.wheel_speed_max = 3.3;
Chassis.vx_max = 1.5;
Chassis.vx_min = -0.4;
Chassis.wz_max = 2.5;
Chassis.accel_limit = 1.2;
Chassis.decel_limit = 1.8;
Chassis.jerk_limit = 5.0;
Chassis.wheel_base = 0.36;
Chassis.reverse_enabled = false;

params_type = coder.typeof(struct(...
    'ControllerMode', 0, ...
    'ReverseEnabled', false, ...
    'LookaheadBase', 0.0, ...
    'LookaheadVelGain', 0.0, ...
    'LookaheadAccelGain', 0.0, ...
    'GoalTolerance', 0.0, ...
    'HeadingKp', 0.0, ...
    'HeadingKi', 0.0, ...
    'HeadingKd', 0.0, ...
    'FeedforwardGain', 0.0, ...
    'KappaThreshold', 0.0, ...
    'VxReduction', 0.0, ...
    'Chassis', Chassis, ...
    'PathInfo_States', zeros(MAX_PATH_POINTS, 3), ...
    'PathInfo_Curvature', zeros(MAX_PATH_POINTS, 1), ...
    'PathInfo_ArcLength', zeros(MAX_PATH_POINTS, 1), ...
    'PathInfo_DistanceRemaining', zeros(MAX_PATH_POINTS, 1)));

% Make PathInfo arrays variable-length (up to MAX_PATH_POINTS)
params_type = coder.typeof(struct(...
    'ControllerMode', 0, ...
    'ReverseEnabled', false, ...
    'LookaheadBase', 0.0, ...
    'LookaheadVelGain', 0.0, ...
    'LookaheadAccelGain', 0.0, ...
    'GoalTolerance', 0.0, ...
    'HeadingKp', 0.0, ...
    'HeadingKi', 0.0, ...
    'HeadingKd', 0.0, ...
    'FeedforwardGain', 0.0, ...
    'KappaThreshold', 0.0, ...
    'VxReduction', 0.0, ...
    'Chassis', Chassis, ...
    'PathInfo_States', coder.typeof(double(0), [MAX_PATH_POINTS, 3], [true, false]), ...
    'PathInfo_Curvature', coder.typeof(double(0), [MAX_PATH_POINTS, 1], [true, false]), ...
    'PathInfo_ArcLength', coder.typeof(double(0), [MAX_PATH_POINTS, 1], [true, false]), ...
    'PathInfo_DistanceRemaining', coder.typeof(double(0), [MAX_PATH_POINTS, 1], [true, false])));

fprintf('✓ params: struct (17 fields + Chassis struct + PathInfo arrays)\n');
fprintf('  - PathInfo arrays: variable-length up to %d points\n', MAX_PATH_POINTS);
fprintf('\n');

%% Create coder configuration for C++ library (ARM64)
fprintf('Step 2: Configure Code Generator\n');
fprintf('───────────────────────────────────────────────────────────────\n');

cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'ChassisPathFollower';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;

% Hardware optimization (ARM64 NEON)
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdLongLongMode = true;

% Optimization settings
cfg.OptimizeReductions = true;
% cfg.GlobalDataSyncMethod = 'NoSync';  % Not available in all MATLAB versions

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.ReportPotentialDifferences = false;

% Build configuration - optimize for speed
cfg.BuildConfiguration = 'Faster Runs';

% Inline options for performance
cfg.InlineThreshold = 10;
cfg.InlineThresholdMax = 200;
cfg.InlineStackLimit = 4000;

fprintf('✓ Target: C++ (ARM Cortex-A)\n');
fprintf('✓ Namespace: gik9dof\n');
fprintf('✓ Class: ChassisPathFollower\n');
fprintf('✓ SIMD: ARM NEON enabled\n');
fprintf('✓ Build: Optimized for speed\n');
fprintf('\n');

%% Run code generation
fprintf('Step 3: Generate C++ Code\n');
fprintf('───────────────────────────────────────────────────────────────\n');
fprintf('This may take 1-2 minutes...\n\n');

try
    tic;
    
    % Use full file path for WSL compatibility
    controller_file = fullfile(matlabRoot, 'chassisPathFollowerCodegen.m');
    fprintf('Target file: %s\n\n', controller_file);
    
    codegen('-config', cfg, ...
        controller_file, ...
        '-args', {pose_type, dt_type, state_type, params_type}, ...
        '-d', CODEGEN_OUTPUT, ...
        '-report');
    
    gen_time = toc;
    gen_time = toc;
    
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('  ✓ CODE GENERATION SUCCESSFUL!\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('Generation time: %.1f seconds\n', gen_time);
    fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
    fprintf('\n');
    
catch ME
    fprintf('✗ Code generation failed:\n');
    fprintf('  %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end

%% Verify output files
fprintf('Step 4: Verify Generated Files\n');
fprintf('───────────────────────────────────────────────────────────────\n');

expectedFiles = {
    'ChassisPathFollower.h';
    'ChassisPathFollower.cpp';
    'chassisPathFollowerCodegen_types.h';
    'rtwtypes.h';
};

allFound = true;
for i = 1:length(expectedFiles)
    fullPath = fullfile(CODEGEN_OUTPUT, expectedFiles{i});
    if exist(fullPath, 'file')
        fileInfo = dir(fullPath);
        fprintf('✓ %s (%.1f KB)\n', expectedFiles{i}, fileInfo.bytes/1024);
    else
        fprintf('✗ %s (MISSING)\n', expectedFiles{i});
        allFound = false;
    end
end

if ~allFound
    error('Some expected files are missing from codegen output');
end

fprintf('\n');

%% Display summary
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Code Generation Complete!\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Generated files: %d\n', length(expectedFiles));
fprintf('Controller modes: 3 (0: diff, 1: heading, 2: pure pursuit)\n');
fprintf('Max path points: %d\n', MAX_PATH_POINTS);
fprintf('\n');
fprintf('Next steps:\n');
fprintf('1. Review code generation report (HTML)\n');
fprintf('2. Build generated C++ code with your ROS2 workspace\n');
fprintf('3. Test in simulation before deployment\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

% Open code generation report
reportFile = fullfile(CODEGEN_OUTPUT, 'html', 'index.html');
if exist(reportFile, 'file')
    fprintf('\nOpening code generation report...\n');
    web(reportFile, '-browser');
end

catch ME
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('  ✗ CODE GENERATION FAILED\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('\nLocation:\n');
    if ~isempty(ME.stack)
        fprintf('  File: %s\n', ME.stack(1).file);
        fprintf('  Line: %d\n', ME.stack(1).line);
        fprintf('  Function: %s\n', ME.stack(1).name);
    end
    fprintf('═══════════════════════════════════════════════════════════════\n');
    rethrow(ME);
end
