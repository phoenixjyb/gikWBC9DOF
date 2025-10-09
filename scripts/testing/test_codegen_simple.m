%% Simple codegen test
clear; clc;

addpath('matlab');

% Create input examples
start_state = gik9dof.HybridState();
start_state.x = 0.0;
start_state.y = 0.0;
start_state.theta = 0.0;

goal_state = gik9dof.HybridState();
goal_state.x = 5.0;
goal_state.y = 5.0;
goal_state.theta = 0.0;

grid = gik9dof.OccupancyGrid2D(0.1, 200, 200, 0.0, 0.0);
opts = struct();

% Configure codegen
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.EnableDynamicMemoryAllocation = true;
cfg.GenerateReport = true;
cfg.LaunchReport = false;

fprintf('Starting codegen...\n');

try
    codegen('-config', cfg, ...
        'gik9dof.planHybridAStarCodegen', ...
        '-args', {start_state, goal_state, grid, opts}, ...
        '-d', 'codegen/test_simple', ...
        '-report');
    fprintf('\n✓ SUCCESS!\n');
catch ME
    fprintf('\n✗ FAILED: %s\n', ME.message);
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end
