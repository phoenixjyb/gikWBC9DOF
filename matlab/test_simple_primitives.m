% Simple test for motion primitives
clear; clc;

% Test 1: Get params
fprintf('Getting chassis params...\n');
params = gik9dof.getChassisParams('fourwheel');
fprintf('  Wheelbase: %.3f m\n', params.wheelbase);
fprintf('  Min radius: %.3f m\n', params.min_turning_radius);

% Test 2: Generate primitives
fprintf('\nGenerating primitives...\n');
primitives = gik9dof.generateMotionPrimitives(params);
fprintf('  Generated: %d primitives\n', length(primitives));

% Test 3: Compute one arc
fprintf('\nTesting arc computation...\n');
[x, y, theta] = gik9dof.computeMotionPrimitive(0, 0, 0, 0.8, 0, 1.0, params);
fprintf('  Straight: (%.3f, %.3f, %.3f)\n', x, y, theta);

fprintf('\nDONE!\n');
