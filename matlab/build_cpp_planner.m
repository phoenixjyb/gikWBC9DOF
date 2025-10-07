%% build_cpp_planner.m
% MATLAB Coder build script for Hybrid A* planner
% Generates standalone C++ library from planHybridAStarCodegen

%% Setup
clear; clc;
fprintf('========================================\n');
fprintf('MATLAB Coder C++ Generation\n');
fprintf('========================================\n\n');

% Add paths
addpath(genpath('matlab'));

%% Define example inputs for type inference
fprintf('1. Creating example inputs for type inference...\n');

% Example start/goal states
start = gik9dof.HybridState();
start.x = 2.0;
start.y = 2.0;
start.theta = 0.0;

goal = gik9dof.HybridState();
goal.x = 8.0;
goal.y = 8.0;
goal.theta = 0.0;

% Example grid (200×200, 0.1m resolution)
grid = gik9dof.OccupancyGrid2D(0.1, 200, 200, 0, 0);

% Optional parameters (use defaults if empty)
opts = struct();

fprintf('   Start state: (%.1f, %.1f, %.2f rad)\n', start.x, start.y, start.theta);
fprintf('   Goal state: (%.1f, %.1f, %.2f rad)\n', goal.x, goal.y, goal.theta);
fprintf('   Grid: %d×%d, %.2fm resolution\n\n', grid.size_x, grid.size_y, grid.resolution);

%% Configure codegen
fprintf('2. Configuring MATLAB Coder...\n');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;

% Optimization settings
cfg.EnableOpenMP = false; % Can enable for multi-threading
cfg.OptimizeReductions = true;
cfg.SaturateOnIntegerOverflow = false;

% Code generation settings
cfg.CppNamespace = 'gik9dof';

fprintf('   Target language: %s\n', cfg.TargetLang);
fprintf('   Namespace: %s\n', cfg.CppNamespace);
fprintf('   Report generation: %s\n\n', mat2str(cfg.GenerateReport));

%% Generate code
fprintf('3. Generating C++ code...\n');
fprintf('   This may take 1-2 minutes...\n\n');

try
    tic;
    codegen -config cfg gik9dof.planHybridAStarCodegen ...
        -args {start, goal, grid, coder.typeof(opts)} ...
        -d codegen/cpp_planner ...
        -report
    gen_time = toc;
    
    fprintf('✓ C++ code generation complete! (%.1f sec)\n\n', gen_time);
    
    %% Display results
    fprintf('========================================\n');
    fprintf('Output Location\n');
    fprintf('========================================\n');
    fprintf('Generated files: codegen/cpp_planner/\n');
    fprintf('  - planHybridAStarCodegen.h/cpp (main interface)\n');
    fprintf('  - HybridState.h (state class)\n');
    fprintf('  - OccupancyGrid2D.h (grid class)\n');
    fprintf('  - Supporting libraries\n\n');
    
    fprintf('HTML Report: codegen/cpp_planner/html/report.mldatx\n\n');
    
    fprintf('========================================\n');
    fprintf('Next Steps\n');
    fprintf('========================================\n');
    fprintf('1. Review HTML report for any warnings\n');
    fprintf('2. Test C++ build: cd codegen/cpp_planner && make\n');
    fprintf('3. Run performance benchmark: run benchmark_cpp_planner.m\n');
    fprintf('========================================\n');
    
catch ME
    fprintf('✗ Code generation FAILED\n\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Location: %s (line %d)\n\n', ME.stack(1).file, ME.stack(1).line);
    
    fprintf('Common issues:\n');
    fprintf('  1. Variable-size data (use fixed-size arrays)\n');
    fprintf('  2. Unsupported functions (check codegen report)\n');
    fprintf('  3. Class definitions (ensure all methods are compatible)\n');
    fprintf('  4. Nested functions (extract to separate files)\n\n');
    
    rethrow(ME);
end
