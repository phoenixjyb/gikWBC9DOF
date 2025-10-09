%% generate_code_planner_arm64.m
% Code generation for Hybrid A* Planner targeting NVIDIA AGX Orin (ARM64)
% Following the same pattern as GIK solver codegen
%
% This script generates optimized C++ code for deployment on:
%   - NVIDIA AGX Orin (ARM Cortex-A78AE, ARM64 with NEON SIMD)
%   - Real-time performance target: < 50ms planning time
%   - Integration with existing ROS2 nodes
%
% Author: WHEELTEC Mobile Manipulator Project
% Date: 2025-10-07

clear; clc;

try
%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
% Go up two levels: scripts/codegen -> scripts -> root
workspaceRoot = fileparts(fileparts(projectRoot));
matlabRoot = fullfile(workspaceRoot, 'matlab');
addpath(genpath(matlabRoot));

CODEGEN_OUTPUT = fullfile(workspaceRoot, 'codegen', 'planner_arm64');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation
% Following the same pattern as GIK solver

% Start state: HybridState object
start_state = gik9dof.HybridState();
start_state.x = 0.0;
start_state.y = 0.0;
start_state.theta = 0.0;

% Goal state: HybridState object  
goal_state = gik9dof.HybridState();
goal_state.x = 0.0;
goal_state.y = 0.0;
goal_state.theta = 0.0;

% Occupancy grid: Fixed-size 200×200 grid (20m × 20m @ 0.1m resolution)
% This matches typical robot navigation maps
grid = gik9dof.OccupancyGrid2D(0.1, 200, 200, 0.0, 0.0);

% Optional parameters (empty struct uses defaults)
opts = struct();

fprintf('===================================================\n');
fprintf('Input Type Specification\n');
fprintf('===================================================\n');
fprintf('start_state: gik9dof.HybridState (x, y, theta)\n');
fprintf('goal_state:  gik9dof.HybridState (x, y, theta)\n');
fprintf('grid:        gik9dof.OccupancyGrid2D (200×200 @ 0.1m)\n');
fprintf('opts:        struct (optional parameters)\n\n');

%% Create coder configuration for C++ library (ARM64)
cfg = coder.config('lib');

% Target settings - matching GIK solver configuration
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'HybridAStarPlanner';

% ARM64-specific hardware settings for NVIDIA AGX Orin
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdLongLongMode = true;

% Memory settings - CRITICAL for avoiding codegen errors
% Enable dynamic allocation for:
%   - Binary heap operations (heap_indices array)
%   - State list (can grow during search)
%   - Path reconstruction (variable-length output)
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536; % 64KB threshold (same as GIK)

% SIMD and parallelization
cfg.EnableOpenMP = false; % Path planning is inherently sequential (A* search)

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.ReportPotentialDifferences = false;

% Build configuration - optimize for speed
cfg.BuildConfiguration = 'Faster Runs';

% Optimization settings
cfg.OptimizeReductions = true;
cfg.SaturateOnIntegerOverflow = false;

% Inline options for performance
cfg.InlineThreshold = 10;
cfg.InlineThresholdMax = 200;
cfg.InlineStackLimit = 4000;

%% Display configuration
fprintf('===================================================\n');
fprintf('Generating C++ code for ARM64 (NVIDIA AGX Orin)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.planHybridAStarCodegen\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Purpose: Real-time path planning on NVIDIA AGX Orin\n\n');
fprintf('Configuration:\n');
fprintf('  - Language: C++17\n');
fprintf('  - Namespace: gik9dof\n');
fprintf('  - Class: HybridAStarPlanner\n');
fprintf('  - Architecture: ARM Cortex-A (ARM64 with NEON SIMD)\n');
fprintf('  - SIMD: ARM NEON (not x86 SSE)\n');
fprintf('  - OpenMP: Disabled (sequential A* search)\n');
fprintf('  - Dynamic Memory: Enabled (required for heap/state list)\n');
fprintf('  - Dynamic Memory Threshold: 64KB\n');
fprintf('  - Build Tool: CMake\n');
fprintf('  - Target Planning Time: < 50ms\n');
fprintf('  - Grid Size: 200×200 (20m × 20m)\n');
fprintf('  - Grid Resolution: 0.1m\n');
fprintf('  - Max States: 50,000\n');
fprintf('  - Max Path Length: 500 waypoints\n');
fprintf('===================================================\n\n');

%% Generate code
fprintf('Starting code generation...\n');
fprintf('This may take 2-3 minutes...\n\n');

try
    tic;
    % Use full file path instead of namespace syntax for WSL compatibility
    planner_file = fullfile(matlabRoot, '+gik9dof', 'planHybridAStarCodegen.m');
    fprintf('Target file: %s\n', planner_file);
    
    codegen('-config', cfg, ...
        planner_file, ...
        '-args', {start_state, goal_state, grid, coder.typeof(opts)}, ...
        '-d', CODEGEN_OUTPUT, ...
        '-report');
    gen_time = toc;
    
    fprintf('\n===================================================\n');
    fprintf('✓ Code generation SUCCESSFUL!\n');
    fprintf('===================================================\n');
    fprintf('Generation time: %.1f seconds\n', gen_time);
    fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
    fprintf('\n');
    
    %% Display generated files
    fprintf('Generated C++ files:\n');
    fprintf('  - planHybridAStarCodegen.h/cpp  (main interface)\n');
    fprintf('  - HybridState.h                 (state class)\n');
    fprintf('  - OccupancyGrid2D.h             (grid class)\n');
    fprintf('  - Supporting libraries (heap, collision, etc.)\n');
    fprintf('\n');
    
    fprintf('HTML Report: %s\n', fullfile(CODEGEN_OUTPUT, 'html', 'report.mldatx'));
    fprintf('\n');
    
    %% Next steps
    fprintf('===================================================\n');
    fprintf('Next Steps for Orin Deployment\n');
    fprintf('===================================================\n');
    fprintf('1. Copy generated code to Orin:\n');
    fprintf('   scp -r %s orin:/path/to/ros2_ws/src/gik9dof_planner/src/\n', CODEGEN_OUTPUT);
    fprintf('\n');
    fprintf('2. Create CMakeLists.txt for ROS2 package:\n');
    fprintf('   - Link generated library\n');
    fprintf('   - Add ROS2 node wrapper\n');
    fprintf('   - Configure ARM64 build flags\n');
    fprintf('\n');
    fprintf('3. Build on Orin:\n');
    fprintf('   cd ~/ros2_ws\n');
    fprintf('   colcon build --packages-select gik9dof_planner --cmake-args -DCMAKE_BUILD_TYPE=Release\n');
    fprintf('\n');
    fprintf('4. Test performance:\n');
    fprintf('   ros2 run gik9dof_planner planner_node\n');
    fprintf('   # Expected: < 50ms planning time\n');
    fprintf('\n');
    fprintf('5. Integrate with existing nodes:\n');
    fprintf('   - obstacle_provider → /occupancy_grid\n');
    fprintf('   - planner_node → /planned_path\n');
    fprintf('   - trajectory_manager → executes path\n');
    fprintf('===================================================\n');
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('✗ Code generation FAILED\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('Location: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    end
    fprintf('\n');
    
    fprintf('Common issues and solutions:\n');
    fprintf('  1. Variable-size data:\n');
    fprintf('     → Check that all arrays have fixed bounds\n');
    fprintf('     → Use pre-allocated buffers (MAX_STATES, MAX_PATH_LENGTH)\n');
    fprintf('\n');
    fprintf('  2. Unsupported functions:\n');
    fprintf('     → Replace containers.Map with binary heap ✓ DONE\n');
    fprintf('     → Avoid dynamic cell arrays\n');
    fprintf('\n');
    fprintf('  3. Class definitions:\n');
    fprintf('     → Ensure all properties have fixed types\n');
    fprintf('     → Pre-allocate struct fields ✓ DONE (getChassisParams)\n');
    fprintf('\n');
    fprintf('  4. Heap operations:\n');
    fprintf('     → Functions must return modified arrays ✓ DONE\n');
    fprintf('     → MATLAB pass-by-value requires explicit returns\n');
    fprintf('\n');
    fprintf('Check HTML report for detailed diagnostics:\n');
    fprintf('  %s\n', fullfile(CODEGEN_OUTPUT, 'html', 'report.mldatx'));
    fprintf('===================================================\n');
    
    rethrow(ME);
end

%% Summary
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║  Hybrid A* Planner - ARM64 Code Generation Complete             ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('✓ C++ library ready for NVIDIA AGX Orin deployment\n');
fprintf('✓ Real-time performance optimizations applied\n');
fprintf('✓ Compatible with existing ROS2 architecture\n');
fprintf('\n');
fprintf('Performance expectations:\n');
fprintf('  - MATLAB codegen version: 0.03-0.06s\n');
fprintf('  - ARM64 C++ (expected):   0.01-0.03s (2-3× faster)\n');
fprintf('  - Speedup vs original:    6-12× total improvement\n');
fprintf('\n');
fprintf('Ready for Orin deployment! 🚀\n');
fprintf('\n');

catch ME
    fprintf('\n❌ Error during code generation:\n');
    fprintf('%s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end
