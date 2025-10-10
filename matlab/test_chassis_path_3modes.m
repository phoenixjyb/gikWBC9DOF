%% Test chassisPathFollowerCodegen - All 3 Controller Modes
% Tests the 3 modes from simulateChassisController:
%   Mode 0: Legacy 5-point differentiation (open-loop replay)
%   Mode 1: Heading-aware controller (simple feedback)
%   Mode 2: Pure pursuit (full feedback)

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Chassis Path Follower - 3 Controller Modes Test\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% Test 1: Mode 0 - Legacy 5-Point Differentiation
fprintf('Test 1: Mode 0 - Legacy 5-Point Differentiation\n');
fprintf('  (Open-loop replay using finite differences)\n');

% Create straight path
waypoints = [(0:0.5:5)' zeros(11,1) zeros(11,1)];
params = createDefaultChassisPathParams();

% Prepare path
chassisProfile = struct('track', 0.573, 'wheel_speed_max', 3.3, ...
    'vx_max', 1.5, 'vx_min', -0.4, 'wz_max', 2.5, ...
    'accel_limit', 1.2, 'decel_limit', 1.8, 'jerk_limit', 5.0, ...
    'wheel_base', 0.36, 'reverse_enabled', false);

PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

% Set mode 0
params.ControllerMode = 0;

% Initialize
pose = [0, 0, 0];
dt = 0.1;
state = struct();

% Step
[vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);

fprintf('  Forward velocity: %.3f m/s\n', vx);
fprintf('  Angular velocity: %.4f rad/s\n', wz);
fprintf('  Mode: %d (differentiation)\n', status.currentMode);
assert(status.currentMode == 0, 'Should be mode 0');
assert(vx >= 0, 'Should have forward velocity');
fprintf('  ✓ Mode 0 working\n');
fprintf('\n');

%% Test 2: Mode 1 - Heading-Aware Controller
fprintf('Test 2: Mode 1 - Heading-Aware Controller\n');
fprintf('  (Simple feedback with lookahead)\n');

% Create path with heading change
waypoints = [
    0 0 0;
    1 0 0;
    2 0 deg2rad(45);
    3 1 deg2rad(90);
];

PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

% Set mode 1
params.ControllerMode = 1;

% Initialize at origin
pose = [0, 0, 0];
state = struct();

% Step
[vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);

fprintf('  Forward velocity: %.3f m/s\n', vx);
fprintf('  Angular velocity: %.4f rad/s\n', wz);
fprintf('  Lookahead distance: %.3f m\n', status.lookaheadDistance);
fprintf('  Mode: %d (heading-aware)\n', status.currentMode);
assert(status.currentMode == 1, 'Should be mode 1');
assert(vx > 0, 'Should have forward velocity');
assert(status.lookaheadDistance > 0, 'Should have lookahead');
fprintf('  ✓ Mode 1 working\n');
fprintf('\n');

%% Test 3: Mode 2 - Pure Pursuit (Full Feedback)
fprintf('Test 3: Mode 2 - Pure Pursuit (Full Feedback)\n');
fprintf('  (Adaptive lookahead + curvature control)\n');

% Create circular path (high curvature)
theta = linspace(0, pi/2, 20)';
radius = 2.0;
waypoints = [radius*cos(theta) radius*sin(theta) theta];

PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

% Set mode 2 (default)
params.ControllerMode = 2;

% Initialize at start of path
pose = [waypoints(1,1), waypoints(1,2), waypoints(1,3)];
state = struct();

% Step
[vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);

fprintf('  Forward velocity: %.3f m/s\n', vx);
fprintf('  Angular velocity: %.4f rad/s\n', wz);
fprintf('  Path curvature: %.4f rad/m\n', status.curvature);
fprintf('  Cross-track error: %.4f m\n', status.crossTrackError);
fprintf('  Lookahead distance: %.3f m\n', status.lookaheadDistance);
fprintf('  Mode: %d (pure pursuit)\n', status.currentMode);
assert(status.currentMode == 2, 'Should be mode 2');
assert(vx > 0, 'Should have forward velocity');
assert(wz ~= 0, 'Should have angular velocity for curve');
assert(abs(status.curvature) > 0.1, 'Should have significant curvature');
fprintf('  ✓ Mode 2 working\n');
fprintf('\n');

%% Test 4: Mode 2 - Curvature-Based Speed Reduction
fprintf('Test 4: Mode 2 - Curvature-Based Speed Reduction\n');
fprintf('  (Verify slowdown in high-curvature regions)\n');

% Sharp turn (high curvature)
theta = linspace(0, pi, 30)';
radius = 0.5;  % Small radius = high curvature
waypoints = [radius*cos(theta) radius*sin(theta) theta];

PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

params.ControllerMode = 2;

% At midpoint of curve (highest curvature)
midIdx = round(size(waypoints,1)/2);
pose = waypoints(midIdx,:);
state = struct();
state.PathNumPoints = size(PathInfo.States,1);
state.CurrentIndex = midIdx;
state.LastVelocity = 1.0;  % Assume was going fast
state.LastAcceleration = 0.0;
state.LastHeadingError = 0.0;
state.IntegralHeadingError = 0.0;
state.PreviousPose = pose;
state.DistanceTraveled = 0.0;

% Step
[vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);

fprintf('  Path curvature: %.4f rad/m (1/r = 1/%.2f = %.2f)\n', status.curvature, radius, 1/radius);
fprintf('  Forward velocity: %.3f m/s\n', vx);
fprintf('  Max velocity: %.3f m/s\n', chassisProfile.vx_max);
fprintf('  Speed reduction: %.1f%%\n', (1 - vx/chassisProfile.vx_max)*100);

% Should be significantly slower than max speed
assert(vx < chassisProfile.vx_max * 0.8, 'Should reduce speed in high curvature');
fprintf('  ✓ Curvature-based slowdown working\n');
fprintf('\n');

%% Test 5: Mode 2 - Acceleration and Jerk Limiting
fprintf('Test 5: Mode 2 - Acceleration and Jerk Limiting\n');
fprintf('  (Verify smooth velocity profiles)\n');

% Straight path for acceleration test
waypoints = [(0:0.5:10)' zeros(21,1) zeros(21,1)];
PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

params.ControllerMode = 2;

% Start from rest
pose = [0, 0, 0];
state = struct();
state.PathNumPoints = size(PathInfo.States,1);
state.CurrentIndex = 1;
state.LastVelocity = 0.0;  % At rest
state.LastAcceleration = 0.0;
state.LastHeadingError = 0.0;
state.IntegralHeadingError = 0.0;
state.PreviousPose = pose;
state.DistanceTraveled = 0.0;

% Step several times and track acceleration
accelHistory = zeros(10,1);
for i = 1:10
    [vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);
    accelHistory(i) = state.LastAcceleration;
    
    % Move robot forward
    pose(1) = pose(1) + vx * dt;
end

fprintf('  Max acceleration: %.3f m/s²\n', max(accelHistory));
fprintf('  Accel limit: %.3f m/s²\n', chassisProfile.accel_limit);
fprintf('  Jerk limit: %.3f m/s³\n', chassisProfile.jerk_limit);

% Verify acceleration respects limits
assert(max(accelHistory) <= chassisProfile.accel_limit * 1.1, 'Acceleration should respect limit');
fprintf('  ✓ Acceleration limiting working\n');
fprintf('\n');

%% Test 6: Mode 2 - Goal Detection
fprintf('Test 6: Mode 2 - Goal Detection\n');
fprintf('  (Verify goal reached flag)\n');

% Short path
waypoints = [
    0 0 0;
    0.5 0 0;
    1.0 0 0;
];

PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

params.ControllerMode = 2;
params.GoalTolerance = 0.15;  % 15cm tolerance

% Near goal (within tolerance)
pose = [0.95, 0, 0];
state = struct();

[vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);

fprintf('  Distance to goal: %.3f m\n', status.distanceRemaining);
fprintf('  Goal tolerance: %.3f m\n', params.GoalTolerance);
fprintf('  Goal reached: %d\n', status.isFinished);

assert(status.isFinished == 1, 'Should detect goal reached');
fprintf('  ✓ Goal detection working\n');
fprintf('\n');

%% Test 7: Mode Switching
fprintf('Test 7: Mode Switching\n');
fprintf('  (Verify all 3 modes work on same path)\n');

% Simple test path
waypoints = [(0:0.5:3)' zeros(7,1) zeros(7,1)];
PathInfo = gik9dof.control.preparePathForFollower(waypoints, chassisProfile);
params.PathInfo_States = PathInfo.States;
params.PathInfo_Curvature = PathInfo.Curvature;
params.PathInfo_ArcLength = PathInfo.ArcLength;
params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

pose = [0, 0, 0];
modes = [0 1 2];
modeNames = {'Differentiation', 'Heading-Aware', 'Pure Pursuit'};

for i = 1:length(modes)
    params.ControllerMode = modes(i);
    state = struct();
    
    [vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state, params);
    
    fprintf('  Mode %d (%s): vx=%.3f m/s, wz=%.4f rad/s ✓\n', ...
        modes(i), modeNames{i}, vx, wz);
    assert(status.currentMode == modes(i), 'Mode mismatch');
end

fprintf('  ✓ Mode switching working\n');
fprintf('\n');

%% Summary
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  ✓ ALL 7 TESTS PASSED\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');
fprintf('Controller is ready for codegen and deployment!\n');
fprintf('All 3 modes (0: differentiation, 1: heading, 2: pure pursuit) working\n');
fprintf('\n');
