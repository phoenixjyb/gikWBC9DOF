%% MATLAB Coder Script: Generate C++ Code for Trajectory Smoothing
% File: scripts/codegen/generate_code_trajectory_smoothing.m
% Purpose: Generate ARM Cortex-A optimized C++ code from MATLAB function
% Target: Jetson AGX Orin (ARM64), ROS2 Humble, Ubuntu 22.04
%
% Author: GitHub Copilot
% Date: 2024-01-XX
% Related: Phase 2 - C++ Code Generation for Real-Time Trajectory Smoothing
%
% Prerequisites:
%   - MATLAB Coder toolbox installed
%   - Function to compile: +gik9dof/+control/smoothTrajectoryVelocity.m
%   - Fixed-size input arrays (5 waypoints max)
%
% Usage:
%   cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
%   matlab -batch "run('scripts/codegen/generate_code_trajectory_smoothing.m')"
%
% Output:
%   codegen/trajectory_smoothing/*.cpp/h - Generated C++ files

%% Setup
clear; clc;
fprintf('═══════════════════════════════════════════════════════════════════\n');
fprintf('  MATLAB Coder: Trajectory Smoothing → C++ Code Generation\n');
fprintf('═══════════════════════════════════════════════════════════════════\n\n');

%% Configure Paths
% Ensure MATLAB path includes the smoothing function
% Get the directory of THIS script file
script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
project_root = char(java.io.File(project_root).getCanonicalPath());
matlab_root = fullfile(project_root, 'matlab');
addpath(genpath(matlab_root));

fprintf('✓ Script directory: %s\n', script_dir);
fprintf('✓ Project root: %s\n', project_root);
fprintf('✓ MATLAB functions: %s\n', matlab_root);

%% Output Directory
output_dir = fullfile(project_root, 'codegen', 'trajectory_smoothing');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
fprintf('✓ Output directory: %s\n\n', output_dir);

%% Configure MATLAB Coder for C++ Generation
fprintf('━━━ Configuration ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

cfg = coder.config('lib');  % Generate static library
cfg.TargetLang = 'C++';     % Generate C++ (not C)

% Hardware: Jetson AGX Orin (ARM Cortex-A78AE, 64-bit)
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdBitPerChar = 8;
cfg.HardwareImplementation.ProdBitPerShort = 16;
cfg.HardwareImplementation.ProdBitPerInt = 32;
cfg.HardwareImplementation.ProdBitPerLong = 64;
cfg.HardwareImplementation.ProdWordSize = 64;

% Code Generation Options
cfg.GenerateReport = true;              % Generate HTML report
cfg.ReportPotentialDifferences = true;  % Warn about MATLAB vs C++ differences
cfg.EnableVariableSizing = false;       % Fixed-size arrays only (CRITICAL!)
cfg.DynamicMemoryAllocation = 'Off';    % No malloc/free (real-time safe)
cfg.SaturateOnIntegerOverflow = false;  % For speed (assume no overflow)

% File naming and packaging
cfg.FilePartitionMethod = 'SingleFile'; % Single .cpp/.h file
cfg.Toolchain = 'CMake';                % CMake-compatible output

% Optimization
cfg.GenCodeOnly = true;                 % Generate code only (no build)
cfg.EnableAutoParallelization = false;  % Single-threaded (avoid overhead)

fprintf('✓ Target language:      C++\n');
fprintf('✓ Hardware:             ARM Cortex-A (64-bit)\n');
fprintf('✓ Dynamic memory:       OFF (real-time safe)\n');
fprintf('✓ Variable sizing:      OFF (fixed-size arrays)\n');
fprintf('✓ Code generation:      SingleFile, Uncompressed\n\n');

%% Define Input Argument Types (Fixed-Size)
fprintf('━━━ Input Types (Fixed-Size Arrays) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Rolling window buffer size
MAX_WAYPOINTS = 5;

% Define parameter struct type (matches MATLAB function signature)
% NOTE: Omit 'smoothing_method' for C++ - always use 'scurve'
params_type = struct(...
    'vx_max', coder.typeof(0), ...
    'ax_max', coder.typeof(0), ...
    'jx_max', coder.typeof(0), ...
    'wz_max', coder.typeof(0), ...
    'alpha_max', coder.typeof(0), ...
    'jerk_wz_max', coder.typeof(0), ...
    'smoothing_method', coder.typeof('', [1 10]));

% Define persistent state struct type (internal state tracking)
% NOTE: MATLAB Coder will handle persistent variables automatically,
%       but we need to ensure they are initialized correctly.
%       The generated C++ will have static variables for persistence.

% Input arguments for codegen (all fixed-size)
% NOTE: For variable-length arrays in C++, we use upper bound [MAX_SIZE] 
%       and MATLAB Coder will generate code that works with any size up to MAX
ARGS = {
    coder.typeof(0, [MAX_WAYPOINTS, 1], [0 0]);  ... % waypoints_x: [5×1] double (fixed-size)
    coder.typeof(0, [MAX_WAYPOINTS, 1], [0 0]);  ... % waypoints_y: [5×1] double (fixed-size)
    coder.typeof(0, [MAX_WAYPOINTS, 1], [0 0]);  ... % waypoints_theta: [5×1] double (fixed-size)
    coder.typeof(0, [MAX_WAYPOINTS, 1], [0 0]);  ... % t_waypoints: [5×1] double (fixed-size)
    coder.typeof(0);                              ... % t_current: scalar
    params_type                                   ... % params: struct
};

fprintf('✓ waypoints_x:      double[%d×1] (fixed-size)\n', MAX_WAYPOINTS);
fprintf('✓ waypoints_y:      double[%d×1] (fixed-size)\n', MAX_WAYPOINTS);
fprintf('✓ waypoints_theta:  double[%d×1] (fixed-size)\n', MAX_WAYPOINTS);
fprintf('✓ t_waypoints:      double[%d×1] (fixed-size)\n', MAX_WAYPOINTS);
fprintf('✓ t_current:        double (scalar)\n');
fprintf('✓ params:           struct (8 fields)\n');
fprintf('✓ Total memory:     %d bytes waypoints + %d bytes params = %d bytes\n', ...
    MAX_WAYPOINTS * 4 * 8, 7 * 8, MAX_WAYPOINTS * 4 * 8 + 7 * 8);
fprintf('\n');

%% Code Generation
fprintf('━━━ Running MATLAB Coder ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

try
    % Generate code
    fprintf('⏳ Generating C++ code...\n');
    tic;
    
    % Call codegen with function file path
    % NOTE: Must use absolute file path to the MATLAB function
    smoothing_fcn = fullfile(matlab_root, '+gik9dof', '+control', 'smoothTrajectoryVelocity.m');
    fprintf('Function path: %s\n', smoothing_fcn);
    
    if ~exist(smoothing_fcn, 'file')
        error('Function file not found: %s', smoothing_fcn);
    end
    
    codegen('-config', cfg, ...
            smoothing_fcn, ...
            '-args', ARGS, ...
            '-d', output_dir, ...
            '-o', 'smoothTrajectoryVelocity');
    
    elapsed = toc;
    
    fprintf('✅ Code generation successful! (%.2f seconds)\n\n', elapsed);
    
    %% Verify Generated Files
    fprintf('━━━ Generated Files ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    
    cpp_file = fullfile(output_dir, 'smoothTrajectoryVelocity.cpp');
    h_file = fullfile(output_dir, 'smoothTrajectoryVelocity.h');
    report_file = fullfile(output_dir, 'html', 'report.mldatx');
    
    if exist(cpp_file, 'file')
        info = dir(cpp_file);
        fprintf('✓ %s (%d KB)\n', 'smoothTrajectoryVelocity.cpp', round(info.bytes/1024));
    else
        fprintf('❌ Missing: smoothTrajectoryVelocity.cpp\n');
    end
    
    if exist(h_file, 'file')
        info = dir(h_file);
        fprintf('✓ %s (%d KB)\n', 'smoothTrajectoryVelocity.h', round(info.bytes/1024));
    else
        fprintf('❌ Missing: smoothTrajectoryVelocity.h\n');
    end
    
    if exist(report_file, 'file')
        fprintf('✓ %s (HTML report)\n', 'html/report.mldatx');
        fprintf('  View report: open(''%s'')\n', report_file);
    end
    
    fprintf('\n');
    
    %% Summary
    fprintf('━━━ Summary ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('✅ C++ code generation complete!\n\n');
    fprintf('Next steps:\n');
    fprintf('  1. Copy generated files to ROS2 package:\n');
    fprintf('     cp %s/smooth*.{cpp,h} \\\n', output_dir);
    fprintf('        ros2/gik9dof_solver/src/trajectory_smoothing/\n\n');
    fprintf('  2. Update CMakeLists.txt to include trajectory_smoothing library\n\n');
    fprintf('  3. Modify gik9dof_solver_node.cpp:\n');
    fprintf('     - Add MODE 3: Trajectory Smoothing\n');
    fprintf('     - Implement waypoint buffer (5 elements)\n');
    fprintf('     - Add smoothing state (vx_prev, wz_prev, ax_prev, alpha_prev)\n');
    fprintf('     - Increase control_rate to 50Hz\n\n');
    fprintf('  4. Build on WSL:\n');
    fprintf('     cd ros2 && colcon build --packages-select gik9dof_solver\n\n');
    fprintf('═══════════════════════════════════════════════════════════════════\n');
    
catch ME
    fprintf('❌ Code generation FAILED!\n\n');
    fprintf('Error message:\n%s\n\n', ME.message);
    fprintf('Error stack:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(k).file, ME.stack(k).line);
    end
    
    % Re-throw error
    rethrow(ME);
end

fprintf('\n🚀 Ready for ROS2 integration!\n');
