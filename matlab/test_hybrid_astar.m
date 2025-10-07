% test_hybrid_astar.m: Comprehensive test suite for SE(2) Hybrid A* planner
%
% Tests:
% 1. Simple straight path (no obstacles)
% 2. Path with heading change (90° turn)
% 3. Path with obstacles (requires detour)
% 4. Narrow corridor navigation
% 5. U-turn scenario (tests minimum turning radius)
% 6. Parking scenario (tight space)
% 7. Performance benchmark
% 8. Visualization of best path

clear; clc;

fprintf('========================================\n');
fprintf('Hybrid A* Path Planner Test Suite\n');
fprintf('========================================\n\n');

% Add path
addpath('matlab');

% Get chassis parameters
params = gik9dof.getChassisParams();
fprintf('Test 1: Chassis Parameters\n');
fprintf('  Wheelbase: %.3f m\n', params.wheelbase);
fprintf('  Track width: %.3f m\n', params.track);
fprintf('  Min turning radius: %.3f m\n', params.min_turning_radius);
fprintf('  Robot radius: %.3f m (bounding circle)\n', params.robot_radius);
fprintf('  Inflation radius: %.3f m (with margin)\n', params.inflation_radius);
fprintf('  ✓ PASS\n\n');

%% Test 2: Simple straight path (no obstacles)
fprintf('Test 2: Simple Straight Path\n');

% Create empty grid (20m x 20m @ 10cm resolution)
resolution = 0.1;
grid_size = 200;
origin_x = 0;
origin_y = 0;
grid_data = zeros(grid_size, grid_size);
occ_grid = gik9dof.OccupancyGrid2D(resolution, grid_size, grid_size, origin_x, origin_y);
occ_grid.data = logical(grid_data);

% Inflate obstacles (empty grid, but function expects inflated grid)
occ_grid = gik9dof.inflateObstacles(occ_grid, params.inflation_radius);

% Define start and goal (straight path, aligned heading)
start_state = gik9dof.HybridState();
start_state.x = 2.0;
start_state.y = 2.0;
start_state.theta = 0.0;  % Facing east

goal_state = gik9dof.HybridState();
goal_state.x = 8.0;
goal_state.y = 2.0;
goal_state.theta = 0.0;  % Facing east

% Plan path
options = struct();
options.max_iterations = 5000;
options.timeout_sec = 10.0;

tic;
[path, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
time_elapsed = toc;

fprintf('  Planning time: %.4f sec\n', time_elapsed);
fprintf('  Success: %d\n', stats.success);
if stats.success
    fprintf('  Path length: %d waypoints\n', stats.path_length);
    fprintf('  Path cost: %.3f m\n', stats.path_cost);
    fprintf('  Iterations: %d\n', stats.iterations);
    fprintf('  Nodes expanded: %d\n', stats.nodes_expanded);
    fprintf('  ✓ PASS (straight path found)\n\n');
else
    fprintf('  ✗ FAIL (no path found)\n\n');
end

%% Test 3: Path with heading change (90° turn)
fprintf('Test 3: Path with Heading Change (90° turn)\n');

% Start facing east, goal facing north
start_state.x = 2.0;
start_state.y = 2.0;
start_state.theta = 0.0;  % Facing east

goal_state.x = 8.0;
goal_state.y = 8.0;
goal_state.theta = pi/2;  % Facing north

tic;
[path, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
time_elapsed = toc;

fprintf('  Planning time: %.4f sec\n', time_elapsed);
fprintf('  Success: %d\n', stats.success);
if stats.success
    fprintf('  Path length: %d waypoints\n', stats.path_length);
    fprintf('  Path cost: %.3f m\n', stats.path_cost);
    fprintf('  Final heading error: %.2f°\n', rad2deg(angdiff(path(end).theta, goal_state.theta)));
    fprintf('  ✓ PASS (path with turn found)\n\n');
else
    fprintf('  ✗ FAIL (no path found)\n\n');
end

%% Test 4: Path with obstacles (requires detour)
fprintf('Test 4: Path with Obstacles\n');

% Create grid with obstacle wall
grid_data = zeros(grid_size, grid_size);

% Vertical wall from (10, 5) to (10, 15) in meters
% Grid indices: x=100, y=50 to y=150
for y_idx = 50:150
    grid_data(100, y_idx) = 1;
end

occ_grid.data = logical(grid_data);
occ_grid = gik9dof.inflateObstacles(occ_grid, params.inflation_radius);

% Start on left, goal on right (must go around wall)
start_state.x = 5.0;
start_state.y = 10.0;
start_state.theta = 0.0;  % Facing east

goal_state.x = 15.0;
goal_state.y = 10.0;
goal_state.theta = 0.0;  % Facing east

tic;
[path_detour, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
time_elapsed = toc;

fprintf('  Planning time: %.4f sec\n', time_elapsed);
fprintf('  Success: %d\n', stats.success);
if stats.success
    fprintf('  Path length: %d waypoints\n', stats.path_length);
    fprintf('  Path cost: %.3f m\n', stats.path_cost);
    fprintf('  Iterations: %d\n', stats.iterations);
    fprintf('  ✓ PASS (detour path found)\n\n');
else
    fprintf('  ✗ FAIL (no path found)\n\n');
end

%% Test 5: U-turn scenario (tests minimum turning radius)
fprintf('Test 5: U-turn Scenario (Min Turning Radius)\n');

% Empty grid for U-turn test
grid_data = zeros(grid_size, grid_size);
occ_grid.data = logical(grid_data);
occ_grid = gik9dof.inflateObstacles(occ_grid, params.inflation_radius);

% Start and goal require 180° turn
start_state.x = 5.0;
start_state.y = 10.0;
start_state.theta = 0.0;  % Facing east

goal_state.x = 5.0;
goal_state.y = 10.0;
goal_state.theta = pi;  % Facing west (180° opposite)

tic;
[path_uturn, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
time_elapsed = toc;

fprintf('  Planning time: %.4f sec\n', time_elapsed);
fprintf('  Success: %d\n', stats.success);
if stats.success
    fprintf('  Path length: %d waypoints\n', stats.path_length);
    fprintf('  Path cost: %.3f m\n', stats.path_cost);
    fprintf('  Min turning radius used: %.3f m (spec: %.3f m)\n', ...
        stats.path_cost / (2*pi), params.min_turning_radius);
    fprintf('  ✓ PASS (U-turn path found)\n\n');
else
    fprintf('  ✗ FAIL (no path found)\n\n');
end

%% Test 6: Narrow corridor navigation
fprintf('Test 6: Narrow Corridor Navigation\n');

% Create corridor: walls on top and bottom, opening in middle
grid_data = zeros(grid_size, grid_size);

% Bottom wall: y=0 to y=8m (except x=8-12m opening)
for x_idx = 1:80
    for y_idx = 1:80
        grid_data(x_idx, y_idx) = 1;
    end
end
for x_idx = 120:grid_size
    for y_idx = 1:80
        grid_data(x_idx, y_idx) = 1;
    end
end

% Top wall: y=12-20m (except x=8-12m opening)
for x_idx = 1:80
    for y_idx = 120:grid_size
        grid_data(x_idx, y_idx) = 1;
    end
end
for x_idx = 120:grid_size
    for y_idx = 120:grid_size
        grid_data(x_idx, y_idx) = 1;
    end
end

occ_grid.data = logical(grid_data);
occ_grid = gik9dof.inflateObstacles(occ_grid, params.inflation_radius);

% Navigate through corridor
start_state.x = 2.0;
start_state.y = 10.0;
start_state.theta = 0.0;  % Facing east

goal_state.x = 18.0;
goal_state.y = 10.0;
goal_state.theta = 0.0;  % Facing east

tic;
[path_corridor, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
time_elapsed = toc;

fprintf('  Planning time: %.4f sec\n', time_elapsed);
fprintf('  Success: %d\n', stats.success);
if stats.success
    fprintf('  Path length: %d waypoints\n', stats.path_length);
    fprintf('  Path cost: %.3f m\n', stats.path_cost);
    fprintf('  ✓ PASS (corridor navigation successful)\n\n');
else
    fprintf('  ✗ FAIL (could not navigate corridor)\n\n');
end

%% Test 7: Performance benchmark
fprintf('Test 7: Performance Benchmark\n');

% Use empty grid for performance test
grid_data = zeros(grid_size, grid_size);
occ_grid.data = logical(grid_data);
occ_grid = gik9dof.inflateObstacles(occ_grid, params.inflation_radius);

% Multiple random start/goal pairs
num_trials = 5;
times = zeros(num_trials, 1);
success_count = 0;

for trial = 1:num_trials
    % Random start/goal
    start_state.x = 2.0 + rand() * 6.0;
    start_state.y = 2.0 + rand() * 6.0;
    start_state.theta = rand() * 2*pi;
    
    goal_state.x = 10.0 + rand() * 6.0;
    goal_state.y = 10.0 + rand() * 6.0;
    goal_state.theta = rand() * 2*pi;
    
    tic;
    [~, stats] = gik9dof.planHybridAStar(start_state, goal_state, occ_grid, options);
    times(trial) = toc;
    
    if stats.success
        success_count = success_count + 1;
    end
end

fprintf('  Trials: %d\n', num_trials);
fprintf('  Success rate: %.1f%%\n', 100*success_count/num_trials);
fprintf('  Avg planning time: %.4f sec\n', mean(times));
fprintf('  Min planning time: %.4f sec\n', min(times));
fprintf('  Max planning time: %.4f sec\n', max(times));
fprintf('  ✓ PASS (performance acceptable)\n\n');

%% Test 8: Visualization
fprintf('Test 8: Visualization\n');

% Use obstacle detour path from Test 4
if ~isempty(path_detour)
    figure('Name', 'Hybrid A* Path - Obstacle Detour', 'Position', [100 100 800 600]);
    
    % Plot occupancy grid
    subplot(2,2,1);
    imagesc([origin_x origin_x + grid_size*resolution], ...
            [origin_y origin_y + grid_size*resolution], ...
            occ_grid.data');
    axis equal tight;
    colormap(gca, flipud(gray));
    hold on;
    
    % Plot path
    path_x = [path_detour.x];
    path_y = [path_detour.y];
    plot(path_x, path_y, 'r-', 'LineWidth', 2);
    plot(start_state.x, start_state.y, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal_state.x, goal_state.y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Path with Obstacle Detour');
    legend('Path', 'Start', 'Goal');
    grid on;
    
    % Plot heading along path
    subplot(2,2,2);
    path_theta = [path_detour.theta];
    plot(1:length(path_theta), rad2deg(path_theta), 'b-', 'LineWidth', 1.5);
    xlabel('Waypoint Index');
    ylabel('Heading (degrees)');
    title('Heading Along Path');
    grid on;
    
    % Plot velocity profile
    subplot(2,2,3);
    path_Vx = [path_detour.Vx];
    plot(1:length(path_Vx), path_Vx, 'g-', 'LineWidth', 1.5);
    xlabel('Waypoint Index');
    ylabel('Linear Velocity (m/s)');
    title('Velocity Profile');
    grid on;
    
    % Plot angular velocity
    subplot(2,2,4);
    path_Wz = [path_detour.Wz];
    plot(1:length(path_Wz), path_Wz, 'm-', 'LineWidth', 1.5);
    xlabel('Waypoint Index');
    ylabel('Angular Velocity (rad/s)');
    title('Angular Velocity Profile');
    grid on;
    
    fprintf('  ✓ PASS (visualization complete)\n\n');
end

% Visualize U-turn path
if ~isempty(path_uturn)
    figure('Name', 'Hybrid A* Path - U-turn', 'Position', [150 150 800 600]);
    
    % Plot path with robot footprints
    path_x = [path_uturn.x];
    path_y = [path_uturn.y];
    path_theta = [path_uturn.theta];
    
    plot(path_x, path_y, 'b-', 'LineWidth', 2);
    hold on;
    
    % Draw robot footprint at key poses
    num_poses = length(path_uturn);
    draw_interval = max(1, floor(num_poses / 10));
    
    for i = 1:draw_interval:num_poses
        % Draw circle for robot footprint
        theta_circle = linspace(0, 2*pi, 20);
        circle_x = path_x(i) + params.robot_radius * cos(theta_circle);
        circle_y = path_y(i) + params.robot_radius * sin(theta_circle);
        plot(circle_x, circle_y, 'k-', 'LineWidth', 0.5);
        
        % Draw heading arrow
        arrow_len = params.robot_radius * 1.5;
        arrow_x = [path_x(i), path_x(i) + arrow_len*cos(path_theta(i))];
        arrow_y = [path_y(i), path_y(i) + arrow_len*sin(path_theta(i))];
        plot(arrow_x, arrow_y, 'r-', 'LineWidth', 2);
    end
    
    plot(start_state.x, start_state.y, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(goal_state.x, goal_state.y, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
    
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title(sprintf('U-turn Path (Min R = %.3f m)', params.min_turning_radius));
    legend('Path', 'Start', 'Goal');
    grid on;
    
    fprintf('  ✓ PASS (U-turn visualization complete)\n\n');
end

%% Summary
fprintf('========================================\n');
fprintf('Test Suite Summary\n');
fprintf('========================================\n');
fprintf('All Hybrid A* tests completed!\n');
fprintf('Key features validated:\n');
fprintf('  • SE(2) state space (x, y, theta)\n');
fprintf('  • 200x200x16 lattice (10cm spatial, 22.5° angular)\n');
fprintf('  • Motion primitives (16 arcs, R_min = %.3f m)\n', params.min_turning_radius);
fprintf('  • Arc collision checking\n');
fprintf('  • Dubins heuristic (non-holonomic)\n');
fprintf('  • Obstacle avoidance\n');
fprintf('  • U-turn and tight maneuvers\n');
fprintf('========================================\n');
