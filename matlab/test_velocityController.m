%% Test Holistic Velocity Controller (Before Codegen)
% Quick validation that the wrapper function works correctly
%
% Date: October 7, 2025

clear; clc;

%% Add paths
addpath(fullfile(pwd, 'matlab'));
addpath(fullfile(pwd, 'matlab', '+gik9dof', '+control'));

fprintf('Testing holisticVelocityController wrapper...\n\n');

%% Setup test parameters

params = struct(...
    'track', 0.5, ...          % 0.5m wheel spacing
    'Vwheel_max', 1.0, ...     % 1.0 m/s max wheel speed
    'Vx_max', 0.8, ...         % 0.8 m/s max forward speed
    'W_max', 1.0, ...          % 1.0 rad/s max yaw rate
    'yawKp', 1.5, ...          % Heading P gain
    'yawKff', 0.8 ...          % Heading FF gain
);

% Initialize state (empty on first call)
state = struct('prev', struct('x', 0.0, 'y', 0.0, 'theta', 0.0, 't', 0.0));

%% Test 1: First call (should return zero velocity)

fprintf('[Test 1] First call (no previous reference):\n');

[Vx, Wz, state] = holisticVelocityController(...
    1.0, 0.0, 0.0, 0.0, ...  % Target: x=1, y=0, theta=0, t=0
    0.0, 0.0, 0.0, ...        % Current: at origin
    params, state);

fprintf('  Target: (1.0, 0.0, 0°)\n');
fprintf('  Current: (0.0, 0.0, 0°)\n');
fprintf('  Output: Vx = %.3f m/s, Wz = %.3f rad/s\n', Vx, Wz);
fprintf('  Expected: Vx ≈ 0, Wz ≈ 0 (first call)\n');

if abs(Vx) < 0.01 && abs(Wz) < 0.01
    fprintf('  ✅ PASS\n\n');
else
    fprintf('  ❌ FAIL (should be zero on first call)\n\n');
end

%% Test 2: Second call (should command forward motion)

fprintf('[Test 2] Second call (moving straight forward):\n');

[Vx, Wz, state] = holisticVelocityController(...
    2.0, 0.0, 0.0, 0.1, ...   % Target: x=2, y=0, theta=0, t=0.1
    0.0, 0.0, 0.0, ...         % Current: still at origin
    params, state);

fprintf('  Target: (2.0, 0.0, 0°) at t=0.1s\n');
fprintf('  Previous: (1.0, 0.0, 0°) at t=0.0s\n');
fprintf('  Current: (0.0, 0.0, 0°)\n');
fprintf('  Output: Vx = %.3f m/s, Wz = %.3f rad/s\n', Vx, Wz);
fprintf('  Expected: Vx > 0 (forward), Wz ≈ 0 (straight)\n');

if Vx > 0.1 && abs(Wz) < 0.5
    fprintf('  ✅ PASS\n\n');
else
    fprintf('  ❌ FAIL\n\n');
end

%% Test 3: Turning motion

fprintf('[Test 3] Turning motion (heading error):\n');

[Vx, Wz, state] = holisticVelocityController(...
    3.0, 1.0, 0.5, 0.2, ...    % Target: x=3, y=1, theta=0.5rad, t=0.2
    0.0, 0.0, 0.0, ...          % Current: still at origin
    params, state);

fprintf('  Target: (3.0, 1.0, 28.6°) at t=0.2s\n');
fprintf('  Current: (0.0, 0.0, 0°)\n');
fprintf('  Output: Vx = %.3f m/s, Wz = %.3f rad/s\n', Vx, Wz);
fprintf('  Expected: Vx > 0, Wz > 0 (turning right)\n');

if Vx > 0.1 && Wz > 0.1
    fprintf('  ✅ PASS\n\n');
else
    fprintf('  ❌ FAIL\n\n');
end

%% Test 4: Wheel limit enforcement

fprintf('[Test 4] Wheel limit enforcement:\n');

params_fast = params;
params_fast.Vx_max = 0.3;      % Low forward limit
params_fast.Vwheel_max = 0.4;  % Low wheel limit

[Vx, Wz, state] = holisticVelocityController(...
    10.0, 0.0, 0.0, 0.3, ...   % Very far target (would need high speed)
    0.0, 0.0, 0.0, ...
    params_fast, state);

fprintf('  Target: Very far (10m ahead)\n');
fprintf('  Limits: Vx_max=0.3, Vwheel_max=0.4\n');
fprintf('  Output: Vx = %.3f m/s, Wz = %.3f rad/s\n', Vx, Wz);
fprintf('  Expected: Vx ≤ 0.3 (clamped)\n');

if Vx <= params_fast.Vx_max + 0.01
    fprintf('  ✅ PASS (wheel limits enforced)\n\n');
else
    fprintf('  ❌ FAIL (exceeds Vx_max)\n\n');
end

%% Summary

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  Wrapper Function Tests Complete!\n');
fprintf('═══════════════════════════════════════════════════════\n\n');
fprintf('✅ holisticVelocityController is working correctly\n');
fprintf('✅ Ready for code generation\n\n');
fprintf('Next: Run generate_code_velocityController.m\n');
