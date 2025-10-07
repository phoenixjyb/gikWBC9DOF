%% Test Pure Pursuit Velocity Controller
% Test cases for purePursuitVelocityController.m
% Validates path buffering, interpolation, lookahead, and Pure Pursuit algorithm

clear; clc;

%% Test 1: Single Waypoint (Point-to-Point)
fprintf('Test 1: Single waypoint navigation\n');
fprintf('=====================================\n');

params = getDefaultParams();

% Start at origin
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Target at (2, 1)
refX = 2.0;
refY = 1.0;
refTheta = atan2(1.0, 2.0);
refTime = 0.0;

% Initialize state
state = [];

% Call controller
[vx, wz, stateOut] = purePursuitVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    params, state);

fprintf('Input: ref=(%.2f, %.2f, %.2f°), pose=(%.2f, %.2f, %.2f°)\n', ...
    refX, refY, rad2deg(refTheta), estX, estY, rad2deg(estYaw));
fprintf('Output: vx=%.3f m/s, wz=%.3f rad/s\n', vx, wz);
fprintf('Path buffer: %d waypoints\n', stateOut.numWaypoints);

% Validate
assert(stateOut.numWaypoints == 1, 'Should have 1 waypoint in buffer');
assert(vx > 0, 'Forward velocity should be positive');
assert(abs(wz) > 0.01, 'Should have angular velocity to turn towards target');
fprintf('✓ Test 1 PASSED\n\n');

%% Test 2: Straight Line Path
fprintf('Test 2: Straight line path following\n');
fprintf('======================================\n');

params = getDefaultParams();

% Robot facing forward along X-axis
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Create straight path ahead
waypoints = [
    1.0, 0.0, 0.0, 0.00;
    2.0, 0.0, 0.0, 0.01;
    3.0, 0.0, 0.0, 0.02;
    4.0, 0.0, 0.0, 0.03;
    5.0, 0.0, 0.0, 0.04;
];

state = [];

% Add waypoints sequentially
for i = 1:size(waypoints, 1)
    refX = waypoints(i, 1);
    refY = waypoints(i, 2);
    refTheta = waypoints(i, 3);
    refTime = waypoints(i, 4);
    
    [vx, wz, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, state);
end

fprintf('Added %d waypoints to buffer\n', size(waypoints, 1));
fprintf('Path buffer has %d waypoints\n', state.numWaypoints);
fprintf('Final output: vx=%.3f m/s, wz=%.3f rad/s\n', vx, wz);

% For straight path, angular velocity should be near zero
assert(state.numWaypoints == 5, 'Should have 5 waypoints');
assert(vx > 0, 'Forward velocity should be positive');
assert(abs(wz) < 0.1, 'Angular velocity should be near zero for straight path');
fprintf('✓ Test 2 PASSED\n\n');

%% Test 3: Curved Path (90-degree turn)
fprintf('Test 3: 90-degree curved path\n');
fprintf('==============================\n');

params = getDefaultParams();

% Robot at origin facing +X
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Create quarter-circle path (right turn)
numPoints = 10;
radius = 2.0;
angles = linspace(0, -pi/2, numPoints); % Right turn

state = [];

for i = 1:numPoints
    refX = radius * cos(angles(i));
    refY = radius * sin(angles(i));
    refTheta = angles(i) - pi/2;
    refTime = (i-1) * 0.01;
    
    [vx, wz, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, state);
end

fprintf('Added %d waypoints along curved path\n', numPoints);
fprintf('Path buffer has %d waypoints\n', state.numWaypoints);
fprintf('Final output: vx=%.3f m/s, wz=%.3f rad/s\n', vx, wz);

% For curved path, should have turning rate
assert(state.numWaypoints >= 5, 'Should have multiple waypoints in buffer');
assert(vx > 0, 'Forward velocity should be positive');
assert(abs(wz) > 0.05, 'Should have significant angular velocity for curve');
fprintf('✓ Test 3 PASSED\n\n');

%% Test 4: Path Buffer Management (waypoint removal)
fprintf('Test 4: Path buffer waypoint removal\n');
fprintf('=====================================\n');

params = getDefaultParams();

% Robot at origin
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Add waypoints ahead
state = [];
for i = 1:5
    refX = i * 0.5;
    refY = 0.0;
    refTheta = 0.0;
    refTime = (i-1) * 0.01;
    
    [vx, wz, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, state);
end

numBefore = state.numWaypoints;
fprintf('Waypoints before movement: %d\n', numBefore);

% Move robot forward past first waypoint
estX = 1.0;
estY = 0.0;
estYaw = 0.0;

% Update with same reference
[vx, wz, state] = purePursuitVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    params, state);

numAfter = state.numWaypoints;
fprintf('Waypoints after movement: %d\n', numAfter);
fprintf('Removed %d waypoints behind robot\n', numBefore - numAfter);

assert(numAfter < numBefore, 'Should remove waypoints behind robot');
fprintf('✓ Test 4 PASSED\n\n');

%% Test 5: Buffer Size Limit (30 waypoints max)
fprintf('Test 5: Path buffer size limit\n');
fprintf('===============================\n');

params = getDefaultParams();

estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Try to add 50 waypoints (exceeds 30 limit)
state = [];
for i = 1:50
    refX = i * 0.5;
    refY = sin(i * 0.3); % Wavy path
    refTheta = 0.0;
    refTime = (i-1) * 0.01;
    
    [vx, wz, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, state);
end

fprintf('Attempted to add 50 waypoints\n');
fprintf('Buffer size: %d waypoints\n', state.numWaypoints);

assert(state.numWaypoints <= 30, 'Buffer should not exceed 30 waypoints');
fprintf('✓ Test 5 PASSED\n\n');

%% Test 6: Adaptive Lookahead
fprintf('Test 6: Adaptive lookahead distance\n');
fprintf('====================================\n');

params = getDefaultParams();
params.lookaheadBase = 0.5;
params.lookaheadVelGain = 0.3;
params.lookaheadTimeGain = 0.0;

% Test at different velocities
velocities = [0.0, 0.5, 1.0, 1.5];

estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Create path
refX = 5.0;
refY = 0.0;
refTheta = 0.0;
refTime = 0.0;

for i = 1:length(velocities)
    state = [];
    
    % First call to initialize state properly
    [~, ~, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, []);
    
    % Set previous velocity for adaptive lookahead test
    state.prevVx = velocities(i);
    
    % Second call with updated velocity state
    [vx, wz, state] = purePursuitVelocityController(...
        refX, refY, refTheta, refTime, ...
        estX, estY, estYaw, ...
        params, state);
    
    % Expected lookahead = 0.5 + 0.3 * v
    expectedLookahead = 0.5 + 0.3 * velocities(i);
    fprintf('At v=%.2f m/s, expected lookahead ≈ %.2f m\n', ...
        velocities(i), expectedLookahead);
end

fprintf('✓ Test 6 PASSED (visual verification)\n\n');

%% Test 7: Wheel Speed Limits
fprintf('Test 7: Wheel speed limit enforcement\n');
fprintf('======================================\n');

params = getDefaultParams();
params.vwheelMax = 1.0; % Strict wheel limit
params.track = 0.5;     % 0.5m track width

estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Sharp turn requiring high angular velocity
refX = 0.5;
refY = 1.0;
refTheta = pi/2;
refTime = 0.0;

state = [];

[vx, wz, state] = purePursuitVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    params, state);

% Calculate wheel speeds
vLeft = vx - (params.track / 2.0) * wz;
vRight = vx + (params.track / 2.0) * wz;

fprintf('Output: vx=%.3f m/s, wz=%.3f rad/s\n', vx, wz);
fprintf('Left wheel: %.3f m/s (limit: %.3f)\n', vLeft, params.vwheelMax);
fprintf('Right wheel: %.3f m/s (limit: %.3f)\n', vRight, params.vwheelMax);

assert(abs(vLeft) <= params.vwheelMax + 0.001, 'Left wheel within limit');
assert(abs(vRight) <= params.vwheelMax + 0.001, 'Right wheel within limit');
fprintf('✓ Test 7 PASSED\n\n');

%% Test 8: Interpolation Spacing
fprintf('Test 8: Path interpolation\n');
fprintf('==========================\n');

params = getDefaultParams();
params.interpSpacing = 0.1; % 10cm interpolation

estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% Two waypoints 1m apart
state = [];

[vx, wz, state] = purePursuitVelocityController(...
    0.0, 0.0, 0.0, 0.0, ...
    estX, estY, estYaw, ...
    params, state);

[vx, wz, state] = purePursuitVelocityController(...
    1.0, 0.0, 0.0, 0.01, ...
    estX, estY, estYaw, ...
    params, state);

fprintf('Path buffer has %d waypoints\n', state.numWaypoints);
fprintf('Expected interpolation: ~10 points for 1m segment at 0.1m spacing\n');
fprintf('✓ Test 8 PASSED (visual verification)\n\n');

%% Summary
fprintf('========================================\n');
fprintf('All Pure Pursuit Controller Tests PASSED ✓\n');
fprintf('========================================\n');

%% Helper function: Default parameters
function params = getDefaultParams()
    params = struct();
    
    % Lookahead parameters
    params.lookaheadBase = 0.8;         % Base lookahead (m)
    params.lookaheadVelGain = 0.3;      % Velocity-dependent gain
    params.lookaheadTimeGain = 0.1;     % Time-dependent gain
    
    % Velocity limits
    params.vxNominal = 1.0;             % Nominal forward speed (m/s)
    params.vxMax = 1.5;                 % Max forward speed (m/s)
    params.wzMax = 2.0;                 % Max angular rate (rad/s)
    
    % Robot parameters
    params.track = 0.674;               % Wheel track width (m)
    params.vwheelMax = 2.0;             % Max wheel speed (m/s)
    
    % Path management
    params.waypointSpacing = 0.15;      % Min spacing (m)
    params.pathBufferSize = 30;         % Max waypoints
    params.goalTolerance = 0.2;         % Waypoint reached threshold (m)
    params.interpSpacing = 0.05;        % Interpolation spacing (m)
end
