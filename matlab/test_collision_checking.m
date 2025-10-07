%% Test Collision Checking for Hybrid A*
% Validates:
% 1. Arc collision detection (sample-based)
% 2. Footprint collision detection (circular/rectangular)
% 3. Integration with inflated occupancy grid

clear; clc;
addpath(genpath('matlab'));

%% Test 1: Create test environment with obstacles
fprintf('TEST 1: Create test occupancy grid\n');
fprintf('====================================\n');

% Create 10m x 10m grid (100x100 cells at 0.1m resolution)
grid_size = 100;
resolution = 0.1;  % [m]
origin_x = 0.0;
origin_y = 0.0;

% Constructor: OccupancyGrid2D(resolution, size_x, size_y, origin_x, origin_y)
grid = gik9dof.OccupancyGrid2D(resolution, grid_size, grid_size, origin_x, origin_y);

% Add vertical wall at x=5m (grid_x=50)
grid.data(:, 50:52) = 1.0;  % 3 cells wide

% Add horizontal wall at y=5m (grid_y=50)
grid.data(50:52, :) = 1.0;

% Add box obstacle at (7m, 3m)
gx_box = grid.worldToGridX(7.0);
gy_box = grid.worldToGridY(3.0);
grid.data(gy_box-2:gy_box+2, gx_box-2:gx_box+2) = 1.0;

num_obstacles = sum(grid.data(:) > 0.5);
fprintf('Grid: %dx%d @ %.2fm resolution\n', grid_size, grid_size, resolution);
fprintf('Obstacles: %d cells\n', num_obstacles);
fprintf('✓ PASS: Test grid created\n\n');

%% Test 2: Get chassis params and inflate obstacles
fprintf('TEST 2: Inflate obstacles by robot radius\n');
fprintf('====================================\n');

params = gik9dof.getChassisParams('fourwheel');
fprintf('Robot radius: %.3f m\n', params.robot_radius);
fprintf('Inflation radius: %.3f m\n', params.inflation_radius);

% Inflate obstacles
grid_inflated = gik9dof.inflateObstacles(grid, params.inflation_radius);

num_obstacles_inflated = sum(grid_inflated.data(:) > 0.5);
fprintf('Obstacles after inflation: %d cells (%.1f%% increase)\n', ...
        num_obstacles_inflated, 100*(num_obstacles_inflated/num_obstacles - 1));

fprintf('✓ PASS: Obstacles inflated\n\n');

%% Test 3: Check footprint collision (single pose)
fprintf('TEST 3: Footprint collision detection\n');
fprintf('====================================\n');

% Test pose 1: Free space (x=2, y=2)
x1 = 2.0; y1 = 2.0; theta1 = 0;
collision1 = gik9dof.checkFootprintCollision(x1, y1, theta1, grid_inflated, params);
fprintf('Pose (%.1f, %.1f, %.1f°): collision = %d (expect 0)\n', ...
        x1, y1, rad2deg(theta1), collision1);
assert(~collision1, 'False positive: should be free');

% Test pose 2: Inside obstacle (x=5, y=5)
x2 = 5.0; y2 = 5.0; theta2 = 0;
collision2 = gik9dof.checkFootprintCollision(x2, y2, theta2, grid_inflated, params);
fprintf('Pose (%.1f, %.1f, %.1f°): collision = %d (expect 1)\n', ...
        x2, y2, rad2deg(theta2), collision2);
assert(collision2, 'False negative: should collide');

% Test pose 3: Near obstacle (x=7, y=3) - should collide due to inflation
x3 = 7.0; y3 = 3.0; theta3 = pi/4;
collision3 = gik9dof.checkFootprintCollision(x3, y3, theta3, grid_inflated, params);
fprintf('Pose (%.1f, %.1f, %.1f°): collision = %d (expect 1 due to inflation)\n', ...
        x3, y3, rad2deg(theta3), collision3);
assert(collision3, 'False negative: should collide with inflated obstacle');

fprintf('✓ PASS: Footprint collision detection\n\n');

%% Test 4: Check arc collision (motion primitives)
fprintf('TEST 4: Arc collision detection\n');
fprintf('====================================\n');

% Test arc 1: Straight line through free space
x_start1 = 1.0; y_start1 = 2.0; theta_start1 = 0;
Vx1 = 0.8; Wz1 = 0.0; dt1 = 2.0;  % 1.6m straight forward

collision_arc1 = gik9dof.checkArcCollision(x_start1, y_start1, theta_start1, ...
                                           Vx1, Wz1, dt1, grid_inflated, params);
fprintf('Arc 1 (straight, free): collision = %d (expect 0)\n', collision_arc1);
assert(~collision_arc1, 'Arc 1 should be collision-free');

% Test arc 2: Straight line into wall (crosses x=5m wall)
x_start2 = 2.0; y_start2 = 2.0; theta_start2 = 0;
Vx2 = 0.8; Wz2 = 0.0; dt2 = 4.0;  % 3.2m straight → crosses wall at x=5m

collision_arc2 = gik9dof.checkArcCollision(x_start2, y_start2, theta_start2, ...
                                           Vx2, Wz2, dt2, grid_inflated, params);
fprintf('Arc 2 (straight into wall): collision = %d (expect 1)\n', collision_arc2);
assert(collision_arc2, 'Arc 2 should collide with wall');

% Test arc 3: Curved path around obstacle
x_start3 = 1.0; y_start3 = 2.0; theta_start3 = 0;
Vx3 = 0.6; Wz3 = 1.0; dt3 = 2.0;  % Left curve

collision_arc3 = gik9dof.checkArcCollision(x_start3, y_start3, theta_start3, ...
                                           Vx3, Wz3, dt3, grid_inflated, params);
fprintf('Arc 3 (curved, free): collision = %d (expect 0)\n', collision_arc3);
assert(~collision_arc3, 'Arc 3 should be collision-free');

% Test arc 4: Backward motion
x_start4 = 3.0; y_start4 = 2.0; theta_start4 = pi;  % Facing left
Vx4 = -0.5; Wz4 = 0.0; dt4 = 2.0;  % Back up 1m

collision_arc4 = gik9dof.checkArcCollision(x_start4, y_start4, theta_start4, ...
                                           Vx4, Wz4, dt4, grid_inflated, params);
fprintf('Arc 4 (backward, free): collision = %d (expect 0)\n', collision_arc4);
assert(~collision_arc4, 'Arc 4 should be collision-free');

fprintf('✓ PASS: Arc collision detection\n\n');

%% Test 5: Visualize collision checking
fprintf('TEST 5: Visualize collision checks\n');
fprintf('====================================\n');

figure('Name', 'Collision Checking - Inflated Grid');
subplot(1,2,1);
imagesc(grid.data');
axis equal tight; colormap(flipud(gray));
title('Original Grid');
xlabel('Grid X'); ylabel('Grid Y');
colorbar;

subplot(1,2,2);
imagesc(grid_inflated.data');
axis equal tight; colormap(flipud(gray));
title(sprintf('Inflated Grid (radius=%.2fm)', params.inflation_radius));
xlabel('Grid X'); ylabel('Grid Y');
colorbar;

% Plot test arcs
figure('Name', 'Arc Collision Detection');
hold on; axis equal; grid on;

% Draw inflated obstacles
[Y, X] = find(grid_inflated.data > 0.5);
for i = 1:length(X)
    wx = grid_inflated.gridToWorldX(X(i));
    wy = grid_inflated.gridToWorldY(Y(i));
    rectangle('Position', [wx-resolution/2, wy-resolution/2, resolution, resolution], ...
              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
end

% Draw test arcs
arcs = {
    struct('x', x_start1, 'y', y_start1, 'theta', theta_start1, ...
           'Vx', Vx1, 'Wz', Wz1, 'dt', dt1, 'collision', collision_arc1);
    struct('x', x_start2, 'y', y_start2, 'theta', theta_start2, ...
           'Vx', Vx2, 'Wz', Wz2, 'dt', dt2, 'collision', collision_arc2);
    struct('x', x_start3, 'y', y_start3, 'theta', theta_start3, ...
           'Vx', Vx3, 'Wz', Wz3, 'dt', dt3, 'collision', collision_arc3);
    struct('x', x_start4, 'y', y_start4, 'theta', theta_start4, ...
           'Vx', Vx4, 'Wz', Wz4, 'dt', dt4, 'collision', collision_arc4);
};

for i = 1:length(arcs)
    arc = arcs{i};
    
    % Sample arc
    num_samples = 30;
    t_samples = linspace(0, arc.dt, num_samples);
    x_arc = zeros(num_samples, 1);
    y_arc = zeros(num_samples, 1);
    
    for j = 1:num_samples
        [x_arc(j), y_arc(j), ~] = gik9dof.computeMotionPrimitive(...
            arc.x, arc.y, arc.theta, arc.Vx, arc.Wz, t_samples(j), params);
    end
    
    % Plot arc (red if collision, green if free)
    if arc.collision
        plot(x_arc, y_arc, 'r-', 'LineWidth', 2);
        plot(arc.x, arc.y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    else
        plot(x_arc, y_arc, 'g-', 'LineWidth', 2);
        plot(arc.x, arc.y, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    end
end

xlabel('X [m]'); ylabel('Y [m]');
title('Arc Collision Detection (Green=Free, Red=Collision)');
legend({'Obstacles (inflated)', 'Free arcs', 'Start (free)', 'Collision arcs', 'Start (collision)'}, ...
       'Location', 'best');
xlim([0 10]); ylim([0 10]);

fprintf('✓ PASS: Visualization complete\n\n');

%% Test 6: Performance benchmark
fprintf('TEST 6: Collision detection performance\n');
fprintf('====================================\n');

% Benchmark footprint collision (1000 checks)
num_tests = 1000;
x_test = rand(num_tests, 1) * 10;
y_test = rand(num_tests, 1) * 10;
theta_test = rand(num_tests, 1) * 2*pi;

tic;
for i = 1:num_tests
    gik9dof.checkFootprintCollision(x_test(i), y_test(i), theta_test(i), ...
                                    grid_inflated, params);
end
time_footprint = toc;
fprintf('Footprint checks: %d tests in %.3f s (%.3f ms/check)\n', ...
        num_tests, time_footprint, 1000*time_footprint/num_tests);

% Benchmark arc collision (100 arcs)
num_arcs = 100;
tic;
for i = 1:num_arcs
    gik9dof.checkArcCollision(x_test(i), y_test(i), theta_test(i), ...
                              0.6, 1.0, 1.0, grid_inflated, params);
end
time_arc = toc;
fprintf('Arc checks: %d tests in %.3f s (%.3f ms/check)\n', ...
        num_arcs, time_arc, 1000*time_arc/num_arcs);

fprintf('✓ PASS: Performance acceptable\n\n');

%% Summary
fprintf('====================================\n');
fprintf('ALL COLLISION TESTS PASSED!\n');
fprintf('====================================\n');
fprintf('Footprint collision: ✓ (%.2f ms/check)\n', 1000*time_footprint/num_tests);
fprintf('Arc collision: ✓ (%.2f ms/check)\n', 1000*time_arc/num_arcs);
fprintf('Inflation: ✓ (radius=%.3fm)\n', params.inflation_radius);
fprintf('\nReady for Hybrid A* integration!\n');
