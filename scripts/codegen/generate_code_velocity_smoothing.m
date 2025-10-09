%% MATLAB Coder Script: Generate C++ for smoothVelocityCommand
% This script generates ARM64-optimized C++ code for velocity smoothing
% Target: Jetson AGX Orin (ARM Cortex-A78), ROS2 Humble
%
% Generated code applies S-curve acceleration/jerk limits to velocity commands
% from any controller (Pure Pursuit, Heading, etc.)

clear; clc;

%% Get script directory and project root
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));

fprintf('=== Velocity Smoothing C++ Code Generation ===\n');
fprintf('Script directory: %s\n', script_dir);
fprintf('Project root: %s\n\n', project_root);

%% Add paths
addpath(fullfile(project_root, 'matlab'));

%% Configure MATLAB Coder
output_dir = fullfile(project_root, 'codegen', 'velocity_smoothing');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('Output directory: %s\n\n', output_dir);

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;

% Memory optimization
cfg.DynamicMemoryAllocation = 'Off';
cfg.EnableVariableSizing = false;

% Optimization flags
cfg.OptimizeReductions = true;
cfg.InlineThreshold = 100;
cfg.InlineStackLimit = 4000;

% ARM64 specific
cfg.EnableOpenMP = false;
cfg.SaturateOnIntegerOverflow = false;

% Code appearance
cfg.PreserveVariableNames = 'UserNames';
cfg.GenerateComments = true;
cfg.MaxIdLength = 63;

fprintf('MATLAB Coder Configuration:\n');
fprintf('  Target Language: %s\n', cfg.TargetLang);
fprintf('  Hardware: ARM Cortex-A\n');
fprintf('  Dynamic Memory: %s\n', cfg.DynamicMemoryAllocation);
fprintf('  Variable Sizing: %s\n\n', mat2str(cfg.EnableVariableSizing));

%% Define input types for codegen
% All inputs are scalar doubles
vx_target_type = coder.typeof(0.0);      % Scalar double
wz_target_type = coder.typeof(0.0);      % Scalar double
vx_prev_type = coder.typeof(0.0);        % Scalar double
wz_prev_type = coder.typeof(0.0);        % Scalar double
ax_prev_type = coder.typeof(0.0);        % Scalar double
alpha_prev_type = coder.typeof(0.0);     % Scalar double
dt_type = coder.typeof(0.0);             % Scalar double

% Parameters struct
params_struct = struct(...
    'vx_max', 0.0, ...
    'ax_max', 0.0, ...
    'jx_max', 0.0, ...
    'wz_max', 0.0, ...
    'alpha_max', 0.0, ...
    'jerk_wz_max', 0.0 ...
);
params_type = coder.typeof(params_struct);

fprintf('Input Types:\n');
fprintf('  vx_target: scalar double\n');
fprintf('  wz_target: scalar double\n');
fprintf('  vx_prev: scalar double\n');
fprintf('  wz_prev: scalar double\n');
fprintf('  ax_prev: scalar double\n');
fprintf('  alpha_prev: scalar double\n');
fprintf('  dt: scalar double\n');
fprintf('  params: struct with 6 double fields\n\n');

%% Run MATLAB Coder
try
    fprintf('Running MATLAB Coder...\n');
    tic;
    
    % Get full path to the MATLAB function
    func_path = fullfile(project_root, 'matlab', '+gik9dof', '+control', 'smoothVelocityCommand.m');
    
    codegen('-config', cfg, ...
        '-d', output_dir, ...
        func_path, ...
        '-args', {vx_target_type, wz_target_type, vx_prev_type, wz_prev_type, ...
               ax_prev_type, alpha_prev_type, dt_type, params_type})
    
    elapsed = toc;
    fprintf('\n✓ Code generation completed in %.2f seconds\n\n', elapsed);
    
    %% List generated files
    fprintf('Generated files in %s:\n', output_dir);
    generated_files = dir(fullfile(output_dir, '*'));
    for i = 1:length(generated_files)
        if ~generated_files(i).isdir
            fprintf('  - %s (%.1f KB)\n', generated_files(i).name, generated_files(i).bytes/1024);
        end
    end
    
    %% Provide deployment instructions
    fprintf('\n=== Deployment Instructions ===\n');
    fprintf('1. Copy generated files to ROS2 package:\n');
    fprintf('   Source: %s\n', output_dir);
    fprintf('   Destination: ros2/gik9dof_solver/src/velocity_smoothing/\n');
    fprintf('                ros2/gik9dof_solver/include/velocity_smoothing/\n\n');
    fprintf('2. Update CMakeLists.txt to include velocity_smoothing library\n\n');
    fprintf('3. Use in C++ code:\n');
    fprintf('   #include "velocity_smoothing/smoothVelocityCommand.h"\n');
    fprintf('   smoothVelocityCommand(vx_target, wz_target, vx_prev, wz_prev,\n');
    fprintf('                         ax_prev, alpha_prev, dt, &params,\n');
    fprintf('                         &vx_smooth, &wz_smooth, &ax_out, &alpha_out);\n\n');
    
    fprintf('✓ Code generation successful!\n');
    
catch ME
    fprintf('\n✗ Code generation failed!\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Location: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    rethrow(ME);
end
