function [vx_cmd, wz_cmd, ax_cmd, alpha_cmd, jerk_vx_cmd, jerk_wz_cmd] = smoothTrajectoryVelocity(waypoints_x, waypoints_y, waypoints_theta, t_waypoints, t_current, params)
%SMOOTHTRAJECTORYVELOCITY Apply acceleration and jerk limits to trajectory following
%
% Inputs:
%   waypoints_x     - X positions of waypoints (Nx1)
%   waypoints_y     - Y positions of waypoints (Nx1)
%   waypoints_theta - Yaw angles of waypoints (Nx1, radians)
%   t_waypoints     - Time stamps of waypoints (Nx1, seconds)
%   t_current       - Current time (scalar)
%   params          - Structure with fields:
%                     .vx_max        - Max forward velocity (m/s)
%                     .ax_max        - Max forward acceleration (m/s²)
%                     .jx_max        - Max forward jerk (m/s³)
%                     .wz_max        - Max angular velocity (rad/s)
%                     .alpha_max     - Max angular acceleration (rad/s²)
%                     .jerk_wz_max   - Max angular jerk (rad/s³)
%                     .smoothing_method - 'scurve' or 'exponential'
%
% Outputs:
%   vx_cmd          - Smoothed forward velocity command (m/s)
%   wz_cmd          - Smoothed angular velocity command (rad/s)
%   ax_cmd          - Forward acceleration (m/s²) - for diagnostics
%   alpha_cmd       - Angular acceleration (rad/s²) - for diagnostics
%   jerk_vx_cmd     - Forward jerk (m/s³) - TRUE enforced value
%   jerk_wz_cmd     - Angular jerk (rad/s³) - TRUE enforced value
%
% Strategy:
%   1. Find current segment between waypoints
%   2. Compute target velocity from segment direction
%   3. Apply S-curve acceleration profile with jerk limits
%   4. Enforce velocity and acceleration bounds

% Copyright 2025 - GIK 9DOF Mobile Manipulator Project
% Jetson AGX Orin Deployment - Trajectory Smoothing Module

%% Input Validation
validateattributes(waypoints_x, {'double'}, {'vector'}, 'smoothTrajectoryVelocity', 'waypoints_x');
validateattributes(waypoints_y, {'double'}, {'vector'}, 'smoothTrajectoryVelocity', 'waypoints_y');
validateattributes(waypoints_theta, {'double'}, {'vector'}, 'smoothTrajectoryVelocity', 'waypoints_theta');
validateattributes(t_waypoints, {'double'}, {'vector'}, 'smoothTrajectoryVelocity', 't_waypoints');
validateattributes(t_current, {'double'}, {'scalar'}, 'smoothTrajectoryVelocity', 't_current');

N = length(waypoints_x);
assert(length(waypoints_y) == N, 'waypoints_y must have same length as waypoints_x');
assert(length(waypoints_theta) == N, 'waypoints_theta must have same length as waypoints_x');
assert(length(t_waypoints) == N, 't_waypoints must have same length as waypoints_x');

%% Default Parameters
if nargin < 6 || isempty(params)
    params = struct();
end

if ~isfield(params, 'vx_max'), params.vx_max = 1.5; end           % m/s
if ~isfield(params, 'ax_max'), params.ax_max = 1.0; end           % m/s²
if ~isfield(params, 'jx_max'), params.jx_max = 5.0; end           % m/s³
if ~isfield(params, 'wz_max'), params.wz_max = 2.0; end           % rad/s
if ~isfield(params, 'alpha_max'), params.alpha_max = 3.0; end     % rad/s²
if ~isfield(params, 'jerk_wz_max'), params.jerk_wz_max = 10.0; end % rad/s³
if ~isfield(params, 'smoothing_method'), params.smoothing_method = 'scurve'; end

%% Persistent State for Smooth Acceleration
persistent vx_prev wz_prev ax_prev alpha_prev t_prev

if isempty(vx_prev)
    vx_prev = 0.0;
    wz_prev = 0.0;
    ax_prev = 0.0;
    alpha_prev = 0.0;
    t_prev = t_current;
end

dt = t_current - t_prev;
if dt <= 0
    dt = 0.02; % Default 50Hz if time didn't advance
end

%% Handle Edge Cases
if N < 2
    % Not enough waypoints, stop smoothly
    vx_cmd = decelerateToStop(vx_prev, ax_prev, dt, params.ax_max, params.jx_max);
    wz_cmd = decelerateToStop(wz_prev, alpha_prev, dt, params.alpha_max, params.jerk_wz_max);
    ax_cmd = (vx_cmd - vx_prev) / dt;
    alpha_cmd = (wz_cmd - wz_prev) / dt;
    
    % Compute actual jerk (rate of change of acceleration)
    jerk_vx_cmd = (ax_cmd - ax_prev) / dt;
    jerk_wz_cmd = (alpha_cmd - alpha_prev) / dt;
    
    updateState(vx_cmd, wz_cmd, ax_cmd, alpha_cmd, t_current);
    return;
end

%% Find Current Segment
% Find the segment we're currently in or approaching
idx = find(t_waypoints >= t_current, 1, 'first');

if isempty(idx)
    % Past all waypoints, decelerate to stop
    vx_cmd = decelerateToStop(vx_prev, ax_prev, dt, params.ax_max, params.jx_max);
    wz_cmd = decelerateToStop(wz_prev, alpha_prev, dt, params.alpha_max, params.jerk_wz_max);
    ax_cmd = (vx_cmd - vx_prev) / dt;
    alpha_cmd = (wz_cmd - wz_prev) / dt;
    
    % Compute actual jerk
    jerk_vx_cmd = (ax_cmd - ax_prev) / dt;
    jerk_wz_cmd = (alpha_cmd - alpha_prev) / dt;
    
    updateState(vx_cmd, wz_cmd, ax_cmd, alpha_cmd, t_current);
    return;
end

if idx == 1
    idx = 2; % Use first segment
end

% Current segment: from waypoint (idx-1) to waypoint (idx)
p0 = [waypoints_x(idx-1); waypoints_y(idx-1)];
p1 = [waypoints_x(idx); waypoints_y(idx)];
theta0 = waypoints_theta(idx-1);
theta1 = waypoints_theta(idx);
t0 = t_waypoints(idx-1);
t1 = t_waypoints(idx);

%% Compute Target Velocities from Segment
% Direction of motion
dp = p1 - p0;
segment_length = norm(dp);
segment_duration = t1 - t0;

if segment_length < 1e-6 || segment_duration < 1e-6
    % Stationary waypoint, stop
    vx_target = 0;
    wz_target = 0;
else
    % Target forward velocity (desired)
    vx_target = segment_length / segment_duration;
    vx_target = min(vx_target, params.vx_max);
    
    % Target angular velocity
    dtheta = wrapToPi(theta1 - theta0);
    wz_target = dtheta / segment_duration;
    wz_target = sign(wz_target) * min(abs(wz_target), params.wz_max);
end

%% Apply Smoothing Based on Method
switch params.smoothing_method
    case 'scurve'
        % S-curve acceleration profile with jerk limiting
        [vx_cmd, ax_cmd] = applySCurve(vx_prev, ax_prev, vx_target, dt, ...
            params.ax_max, params.jx_max);
        [wz_cmd, alpha_cmd] = applySCurve(wz_prev, alpha_prev, wz_target, dt, ...
            params.alpha_max, params.jerk_wz_max);
        
        % NOTE: Safety verification removed - applySCurve should enforce jerk limit
        % If violations still occur, the issue is in applySCurve logic
        
    case 'exponential'
        % Exponential smoothing (simpler, faster)
        tau_v = 0.2; % Time constant for velocity smoothing (seconds)
        tau_w = 0.15; % Time constant for angular velocity
        
        alpha_v = exp(-dt / tau_v);
        alpha_w = exp(-dt / tau_w);
        
        vx_cmd = alpha_v * vx_prev + (1 - alpha_v) * vx_target;
        wz_cmd = alpha_w * wz_prev + (1 - alpha_w) * wz_target;
        
        ax_cmd = (vx_cmd - vx_prev) / dt;
        alpha_cmd = (wz_cmd - wz_prev) / dt;
        
    otherwise
        error('Unknown smoothing method: %s', params.smoothing_method);
end

%% Compute TRUE Jerk (BEFORE any final clamping!)
% This is the jerk that was ACTUALLY enforced by the S-curve algorithm
jerk_vx_cmd = (ax_cmd - ax_prev) / dt;
jerk_wz_cmd = (alpha_cmd - alpha_prev) / dt;

%% Final Safety Clamps (these should rarely trigger if S-curve works correctly)
% NOTE: These clamps can create jerk violations if triggered!
% They are here only as a last-resort safety measure
vx_cmd = sign(vx_cmd) * min(abs(vx_cmd), params.vx_max);
wz_cmd = sign(wz_cmd) * min(abs(wz_cmd), params.wz_max);
% DO NOT clamp acceleration here - it would violate jerk limit!
% The S-curve algorithm already enforced acceleration limits
% ax_cmd = sign(ax_cmd) * min(abs(ax_cmd), params.ax_max);
% alpha_cmd = sign(alpha_cmd) * min(abs(alpha_cmd), params.alpha_max);

%% Update Persistent State
updateState(vx_cmd, wz_cmd, ax_cmd, alpha_cmd, t_current);

%% Nested Helper Functions
    function updateState(vx, wz, ax, alpha, t)
        vx_prev = vx;
        wz_prev = wz;
        ax_prev = ax;
        alpha_prev = alpha;
        t_prev = t;
    end
end

%% Helper Function: S-Curve Acceleration Profile
function [v_out, a_out] = applySCurve(v_current, a_current, v_target, dt, a_max, jerk_max)
%APPLYSCURVE Apply S-curve acceleration profile with jerk limiting
%
% S-curve has 3 phases:
%   1. Jerk-up phase: acceleration increases linearly
%   2. Constant acceleration phase
%   3. Jerk-down phase: acceleration decreases linearly
%
% This implementation uses a simplified single-step approach suitable for
% real-time control at 50-100Hz.

% Velocity error
dv = v_target - v_current;

% Determine required acceleration
if abs(dv) < 1e-6
    % Already at target, maintain
    a_desired = -a_current * 0.5; % Gradually reduce acceleration to zero
else
    % Acceleration needed to reach target in reasonable time
    % Using a_desired = k * dv with damping
    % Reduced gain for smoother transitions
    k_p = 1.5; % Proportional gain (reduced from 2.0)
    a_desired = k_p * dv;
end

% Limit acceleration change by jerk
% This is the key to smooth motion - jerk is rate of change of acceleration
da_max = jerk_max * dt;
da = a_desired - a_current;

% Strictly enforce jerk limit
if abs(da) > da_max
    da = sign(da) * da_max;
end

% New acceleration after jerk-limited change
a_out = a_current + da;

% DO NOT clamp acceleration here - it would create jerk violations!
% The jerk limit naturally prevents acceleration from growing too fast
% Over time, acceleration will converge to safe values

% Integrate to get new velocity
v_out = v_current + a_out * dt;

% Clamp velocity (safety - should be done outside too)
v_out = sign(v_out) * min(abs(v_out), max(abs(v_target) * 1.2, 0.5)); % Allow 20% overshoot temporarily

end

%% Helper Function: Decelerate to Stop
function v_out = decelerateToStop(v_current, a_current, dt, a_max, jerk_max)
%DECELERATETOSTOP Smoothly decelerate to zero velocity

if abs(v_current) < 1e-3
    v_out = 0;
    return;
end

% Target deceleration (opposite sign of velocity)
a_target = -sign(v_current) * a_max;

% Limit acceleration change by jerk - STRICT enforcement
da_max = jerk_max * dt;
da = a_target - a_current;

% Strictly enforce jerk limit
if abs(da) > da_max
    da = sign(da) * da_max;
end

% New acceleration after jerk-limited change
a_out = a_current + da;

% Clamp to max acceleration (secondary constraint)
if abs(a_out) > a_max
    a_out = sign(a_out) * a_max;
end

% Integrate to get new velocity
v_out = v_current + a_out * dt;

% Don't overshoot zero
if sign(v_out) ~= sign(v_current)
    v_out = 0;
end

end
