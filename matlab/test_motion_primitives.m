%% Test Motion Primitives for Front-Diff + Passive-Rear Robot
% This script validates:
% 1. Motion primitive generation
% 2. Kinematic model (arc computation)
% 3. Constraint enforcement (min radius, wheel speeds)
% 4. HybridState struct

clear; clc;
addpath(genpath('matlab'));

%% Test 1: Get chassis parameters
fprintf('TEST 1: Chassis parameters\n');
fprintf('====================================\n');

params = gik9dof.getChassisParams('fourwheel');

fprintf('Platform: %s\n', params.chassis_type);
fprintf('Wheelbase: %.3f m\n', params.wheelbase);
fprintf('Track: %.3f m\n', params.track);
fprintf('Wheel radius: %.4f m\n', params.wheel_radius);
fprintf('Min turning radius: %.3f m\n', params.min_turning_radius);
fprintf('Vwheel_max: %.2f m/s\n', params.Vwheel_max);
fprintf('Vx_max: %.2f m/s\n', params.Vx_max);
fprintf('Wz_max: %.2f rad/s\n', params.Wz_max);
fprintf('\n');

assert(strcmp(params.chassis_type, 'front_diff_rear_passive'), 'Wrong chassis type');
assert(abs(params.wheelbase - 0.36) < 1e-6, 'Wrong wheelbase');
assert(abs(params.track - 0.573) < 1e-6, 'Wrong track');
assert(params.min_turning_radius > 0.3 && params.min_turning_radius < 0.4, 'Unexpected min radius');

fprintf('✓ PASS: Chassis parameters correct\n\n');

%% Test 2: Generate motion primitives
fprintf('TEST 2: Motion primitive generation\n');
fprintf('====================================\n');

primitives = gik9dof.generateMotionPrimitives(params);

fprintf('Total primitives: %d\n', length(primitives));
fprintf('\nPrimitive summary:\n');
for i = 1:min(5, length(primitives))
    p = primitives(i);
    fprintf('  %2d. Vx=%+.2f Wz=%+.2f dt=%.1f | %s\n', ...
            i, p.Vx, p.Wz, p.dt, p.description);
end
if length(primitives) > 5
    fprintf('  ... (%d more)\n', length(primitives) - 5);
end
fprintf('\n');

% Validate all primitives
num_forward = sum([primitives.Vx] > 0);
num_backward = sum([primitives.Vx] < 0);

assert(length(primitives) >= 10, 'Too few primitives');
assert(num_forward > 0, 'No forward primitives');
assert(num_backward > 0, 'No backward primitives');

% Check constraints on all primitives
for i = 1:length(primitives)
    p = primitives(i);
    
    % Wheel speeds
    v_L = p.Vx - (params.track/2) * p.Wz;
    v_R = p.Vx + (params.track/2) * p.Wz;
    assert(abs(v_L) <= params.Vwheel_max + 1e-6, ...
           sprintf('Primitive %d: Left wheel speed %.2f exceeds limit', i, v_L));
    assert(abs(v_R) <= params.Vwheel_max + 1e-6, ...
           sprintf('Primitive %d: Right wheel speed %.2f exceeds limit', i, v_R));
    
    % Min radius
    if abs(p.Wz) > 1e-4
        R = abs(p.Vx / p.Wz);
        assert(R >= params.min_turning_radius - 1e-3, ...
               sprintf('Primitive %d: Radius %.3f < min_radius %.3f', i, R, params.min_turning_radius));
    end
end

fprintf('✓ PASS: All primitives satisfy constraints\n\n');

%% Test 3: Compute motion primitive arcs
fprintf('TEST 3: Arc computation\n');
fprintf('====================================\n');

% Test straight line
x0 = 0; y0 = 0; theta0 = 0;
Vx = 0.8; Wz = 0.0; dt = 1.0;

[x1, y1, theta1] = gik9dof.computeMotionPrimitive(x0, y0, theta0, Vx, Wz, dt, params);

expected_x = 0.8;  % Straight along X axis
expected_y = 0.0;
expected_theta = 0.0;

assert(abs(x1 - expected_x) < 1e-6, 'Straight line X incorrect');
assert(abs(y1 - expected_y) < 1e-6, 'Straight line Y incorrect');
assert(abs(theta1 - expected_theta) < 1e-6, 'Straight line theta incorrect');

fprintf('Straight line: (%.3f, %.3f, %.3f°) → (%.3f, %.3f, %.3f°)\n', ...
        x0, y0, rad2deg(theta0), x1, y1, rad2deg(theta1));
fprintf('✓ PASS: Straight line motion\n\n');

% Test circular arc (left turn)
x0 = 0; y0 = 0; theta0 = 0;
Vx = 0.6; Wz = 1.5; dt = 1.0;  % Left turn

[x2, y2, theta2] = gik9dof.computeMotionPrimitive(x0, y0, theta0, Vx, Wz, dt, params);

R = Vx / Wz;  % 0.4 m radius
dtheta = Wz * dt;  % 1.5 rad heading change

fprintf('Left arc: R=%.3f m, dtheta=%.3f rad (%.1f°)\n', R, dtheta, rad2deg(dtheta));
fprintf('  Start: (%.3f, %.3f, %.3f°)\n', x0, y0, rad2deg(theta0));
fprintf('  End:   (%.3f, %.3f, %.3f°)\n', x2, y2, rad2deg(theta2));

% Verify heading change
assert(abs(theta2 - dtheta) < 1e-6, 'Heading change incorrect');

% Verify distance from center
cx = x0 - R * sin(theta0);  % Center at (0, 0.4)
cy = y0 + R * cos(theta0);
dist_start = sqrt((x0-cx)^2 + (y0-cy)^2);
dist_end = sqrt((x2-cx)^2 + (y2-cy)^2);

assert(abs(dist_start - R) < 1e-6, 'Start not on circle');
assert(abs(dist_end - R) < 1e-6, 'End not on circle');

fprintf('✓ PASS: Circular arc motion\n\n');

%% Test 4: HybridState struct
fprintf('TEST 4: HybridState struct\n');
fprintf('====================================\n');

state = gik9dof.HybridState();

assert(isfield(state, 'x'), 'Missing field x');
assert(isfield(state, 'y'), 'Missing field y');
assert(isfield(state, 'theta'), 'Missing field theta');
assert(isfield(state, 'grid_x'), 'Missing field grid_x');
assert(isfield(state, 'grid_y'), 'Missing field grid_y');
assert(isfield(state, 'theta_bin'), 'Missing field theta_bin');
assert(isfield(state, 'Vx'), 'Missing field Vx');
assert(isfield(state, 'Wz'), 'Missing field Wz');
assert(isfield(state, 'g'), 'Missing field g');
assert(isfield(state, 'h'), 'Missing field h');
assert(isfield(state, 'f'), 'Missing field f');
assert(isfield(state, 'parent_idx'), 'Missing field parent_idx');
assert(isfield(state, 'is_valid'), 'Missing field is_valid');

fprintf('State fields: %d\n', length(fieldnames(state)));
fprintf('  Continuous: x, y, theta\n');
fprintf('  Discrete: grid_x, grid_y, theta_bin\n');
fprintf('  Command: Vx, Wz, dt\n');
fprintf('  A* costs: g, h, f\n');
fprintf('  Tracking: parent_idx, is_valid\n');

fprintf('✓ PASS: HybridState struct complete\n\n');

%% Test 5: Visualize motion primitives
fprintf('TEST 5: Visualize primitives\n');
fprintf('====================================\n');

figure('Name', 'Motion Primitives - Front Diff + Passive Rear');
hold on; axis equal; grid on;

% Starting pose
x0 = 0; y0 = 0; theta0 = 0;

% Plot robot starting position
plot(x0, y0, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
quiver(x0, y0, 0.3*cos(theta0), 0.3*sin(theta0), 0, 'k', 'LineWidth', 2);

% Plot all primitives
colors = lines(length(primitives));
num_samples = 20;  % Points per arc

for i = 1:length(primitives)
    p = primitives(i);
    
    % Sample arc
    t_samples = linspace(0, p.dt, num_samples);
    x_arc = zeros(num_samples, 1);
    y_arc = zeros(num_samples, 1);
    
    for j = 1:num_samples
        [x_arc(j), y_arc(j), ~] = gik9dof.computeMotionPrimitive(...
            x0, y0, theta0, p.Vx, p.Wz, t_samples(j), params);
    end
    
    % Plot arc
    if p.Vx > 0
        plot(x_arc, y_arc, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    else
        plot(x_arc, y_arc, '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    
    % Mark endpoint
    plot(x_arc(end), y_arc(end), 'o', 'Color', colors(i,:), 'MarkerSize', 4);
end

xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('Motion Primitives (n=%d) | Wheelbase=%.2fm, R_{min}=%.2fm', ...
              length(primitives), params.wheelbase, params.min_turning_radius));
legend({'Start pose', 'Heading', 'Primitives'}, 'Location', 'best');

fprintf('✓ PASS: Primitives visualized\n\n');

%% Summary
fprintf('====================================\n');
fprintf('ALL TESTS PASSED!\n');
fprintf('====================================\n');
fprintf('Motion primitive system ready for Hybrid A*\n');
fprintf('Platform: Front differential + passive rear\n');
fprintf('Min radius: %.3f m (enforced)\n', params.min_turning_radius);
fprintf('Primitives: %d total (%d fwd, %d back)\n', ...
        length(primitives), num_forward, num_backward);
fprintf('\n');
