%% Debug codegen with full error output
clear; clc;

diary('codegen_debug.log');
diary on;

try
    addpath('matlab');
    fprintf('=== Step 1: Create input examples ===\n');
    
    start_state = gik9dof.HybridState();
    start_state.x = 0.0;
    start_state.y = 0.0;
    start_state.theta = 0.0;
    fprintf('  - Start state created\n');
    
    goal_state = gik9dof.HybridState();
    goal_state.x = 5.0;
    goal_state.y = 5.0;
    goal_state.theta = 0.0;
    fprintf('  - Goal state created\n');
    
    grid = gik9dof.OccupancyGrid2D(0.1, 200, 200, 0.0, 0.0);
    fprintf('  - Grid created\n');
    
    opts = struct();
    fprintf('  - Opts created\n');
    
    fprintf('\n=== Step 2: Test function execution ===\n');
    path = gik9dof.planHybridAStarCodegen(start_state, goal_state, grid, opts);
    fprintf('  - Function runs successfully!\n');
    fprintf('  - Path length: %d states\n', length(path));
    
    fprintf('\n=== Step 3: Configure codegen ===\n');
    cfg = coder.config('lib');
    fprintf('  - coder.config created\n');
    cfg.TargetLang = 'C++';
    fprintf('  - TargetLang set\n');
    cfg.EnableDynamicMemoryAllocation = true;
    fprintf('  - Dynamic memory enabled\n');
    cfg.GenerateReport = true;
    fprintf('  - Generate report enabled\n');
    cfg.LaunchReport = false;
    fprintf('  - Launch report disabled\n');
    fprintf('  - Config created successfully\n');
    
    fprintf('\n=== Step 4: Run codegen ===\n');
    codegen('-config', cfg, ...
        'gik9dof.planHybridAStarCodegen', ...
        '-args', {start_state, goal_state, grid, opts}, ...
        '-d', 'codegen/test_debug', ...
        '-report');
    
    fprintf('\n=== SUCCESS! ===\n');
    
catch ME
    fprintf('\n=== ERROR ===\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\nFull report:\n');
    fprintf('%s\n', getReport(ME));
    
    % Check if report exists
    if exist('codegen/test_debug/html/report.mldatx', 'file')
        fprintf('\nCodegen report available at: codegen/test_debug/html/report.mldatx\n');
    end
end

diary off;
fprintf('\n\nLog saved to: codegen_debug.log\n');
