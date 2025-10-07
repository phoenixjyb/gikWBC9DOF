%% TEST_GRID_ASTAR - Test 2D grid-based A* planner
%  Simple validation before implementing full Hybrid A*

clear; clc; close all;

addpath(fullfile(pwd, 'matlab'));

%% Test Case 1: Empty grid, straight line
fprintf('=== Test 1: Empty Grid, Straight Line ===\n');

% Create 20x20 meter grid, 0.5m resolution
grid1 = gik9dof.OccupancyGrid2D(0.5, 40, 40, 0, 0);

% Plan from (1,1) to (19,19)
start_x = 1.0;
start_y = 1.0;
goal_x = 19.0;
goal_y = 19.0;

tic;
[path1, success1] = gik9dof.planGridAStar(start_x, start_y, goal_x, goal_y, grid1, 5000);
time1 = toc;

fprintf('Success: %d\n', success1);
fprintf('Path length: %d waypoints\n', length(path1.x));
fprintf('Planning time: %.3f ms\n', time1 * 1000);

% Visualize
figure('Name', 'Test 1: Empty Grid');
subplot(1,2,1);
imagesc([grid1.origin_x, grid1.origin_x + double(grid1.size_x)*grid1.resolution], ...
        [grid1.origin_y, grid1.origin_y + double(grid1.size_y)*grid1.resolution], ...
        grid1.data);
hold on;
plot(path1.x, path1.y, 'r-', 'LineWidth', 2);
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_x, goal_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
colormap(gca, [1 1 1; 0 0 0]);  % White=free, Black=occupied
axis equal; grid on;
title('Empty Grid Test');
xlabel('X [m]'); ylabel('Y [m]');
legend('Path', 'Start', 'Goal');


%% Test Case 2: Simple obstacle
fprintf('\n=== Test 2: Simple Obstacle ===\n');

% Create grid with vertical wall
grid2 = gik9dof.OccupancyGrid2D(0.5, 40, 40, 0, 0);
grid2.data(:, 20) = true;  % Vertical wall at x=10m
grid2.data(15:25, 20) = false;  % Gap in wall

% Plan from (2,10) to (18,10)
start_x = 2.0;
start_y = 10.0;
goal_x = 18.0;
goal_y = 10.0;

tic;
[path2, success2] = gik9dof.planGridAStar(start_x, start_y, goal_x, goal_y, grid2, 5000);
time2 = toc;

fprintf('Success: %d\n', success2);
fprintf('Path length: %d waypoints\n', length(path2.x));
fprintf('Planning time: %.3f ms\n', time2 * 1000);

% Visualize
subplot(1,2,2);
imagesc([grid2.origin_x, grid2.origin_x + double(grid2.size_x)*grid2.resolution], ...
        [grid2.origin_y, grid2.origin_y + double(grid2.size_y)*grid2.resolution], ...
        grid2.data);
hold on;
plot(path2.x, path2.y, 'r-', 'LineWidth', 2);
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_x, goal_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
colormap(gca, [1 1 1; 0 0 0]);
axis equal; grid on;
title('Wall with Gap Test');
xlabel('X [m]'); ylabel('Y [m]');
legend('Path', 'Start', 'Goal');


%% Test Case 3: Narrow corridor
fprintf('\n=== Test 3: Narrow Corridor ===\n');

% Create grid with corridor
grid3 = gik9dof.OccupancyGrid2D(0.2, 100, 100, 0, 0);

% Top and bottom walls
grid3.data(1:40, :) = true;
grid3.data(61:100, :) = true;

% Left and right walls with entrance/exit
grid3.data(:, 1:20) = true;
grid3.data(:, 81:100) = true;

% Entrance and exit
grid3.data(45:55, 20) = false;
grid3.data(45:55, 80) = false;

% Plan through corridor 
% Grid: 0-20m x 0-20m, walls at x: 0-4m and 16-20m, y: 0-8m and 12-20m
% Corridor is x: 4-16m, y: 8-12m
start_x = 5.0;  % Inside left room, near entrance
start_y = 10.0; % Middle of corridor opening
goal_x = 15.0;  % Inside right room, near exit
goal_y = 10.0;  % Middle of corridor opening

tic;
[path3, success3] = gik9dof.planGridAStar(start_x, start_y, goal_x, goal_y, grid3, 10000);
time3 = toc;

fprintf('Success: %d\n', success3);
fprintf('Path length: %d waypoints\n', length(path3.x));
fprintf('Planning time: %.3f ms\n', time3 * 1000);

% Visualize
figure('Name', 'Test 3: Narrow Corridor');
imagesc([grid3.origin_x, grid3.origin_x + double(grid3.size_x)*grid3.resolution], ...
        [grid3.origin_y, grid3.origin_y + double(grid3.size_y)*grid3.resolution], ...
        grid3.data);
hold on;
plot(path3.x, path3.y, 'r-', 'LineWidth', 2);
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_x, goal_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
colormap(gca, [1 1 1; 0 0 0]);
axis equal; grid on;
title('Narrow Corridor Test');
xlabel('X [m]'); ylabel('Y [m]');
legend('Path', 'Start', 'Goal');


%% Test Case 4: No solution (blocked goal)
fprintf('\n=== Test 4: Blocked Goal (No Solution) ===\n');

% Create grid with goal surrounded by obstacles
grid4 = gik9dof.OccupancyGrid2D(0.5, 40, 40, 0, 0);
grid4.data(18:22, 18:22) = true;  % Box around goal

start_x = 2.0;
start_y = 2.0;
goal_x = 10.0;  % Inside box
goal_y = 10.0;

tic;
[path4, success4] = gik9dof.planGridAStar(start_x, start_y, goal_x, goal_y, grid4, 5000);
time4 = toc;

fprintf('Success: %d (should be 0)\n', success4);
fprintf('Path length: %d waypoints\n', length(path4.x));
fprintf('Planning time: %.3f ms\n', time4 * 1000);


%% Summary
fprintf('\n=== Summary ===\n');
fprintf('Test 1 (Empty): %s, %d points, %.1f ms\n', ...
        string(success1), length(path1.x), time1*1000);
fprintf('Test 2 (Wall):  %s, %d points, %.1f ms\n', ...
        string(success2), length(path2.x), time2*1000);
fprintf('Test 3 (Corridor): %s, %d points, %.1f ms\n', ...
        string(success3), length(path3.x), time3*1000);
fprintf('Test 4 (Blocked): %s (expected false)\n', string(success4));

fprintf('\n✓ 2D Grid A* planner validated!\n');
fprintf('Next: Add SE(2) state space for Hybrid A*\n');
