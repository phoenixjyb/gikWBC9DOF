function [vx_smooth, wz_smooth, ax_out, alpha_out] = smoothVelocityCommand(vx_target, wz_target, vx_prev, wz_prev, ax_prev, alpha_prev, dt, params)
%SMOOTHVELOCITYCOMMAND Apply S-curve smoothing to velocity commands
%
% This function applies acceleration and jerk limits to raw velocity commands
% from any controller (Pure Pursuit, Heading Controller, etc.)
%
% Inputs:
%   vx_target    - Target forward velocity (m/s)
%   wz_target    - Target angular velocity (rad/s)
%   vx_prev      - Previous forward velocity (m/s)
%   wz_prev      - Previous angular velocity (rad/s)
%   ax_prev      - Previous forward acceleration (m/s²)
%   alpha_prev   - Previous angular acceleration (rad/s²)
%   dt           - Time step (seconds)
%   params       - Structure with fields:
%                  .vx_max        - Max forward velocity (m/s)
%                  .ax_max        - Max forward acceleration (m/s²)
%                  .jx_max        - Max forward jerk (m/s³)
%                  .wz_max        - Max angular velocity (rad/s)
%                  .alpha_max     - Max angular acceleration (rad/s²)
%                  .jerk_wz_max   - Max angular jerk (rad/s³)
%
% Outputs:
%   vx_smooth    - Smoothed forward velocity (m/s)
%   wz_smooth    - Smoothed angular velocity (rad/s)
%   ax_out       - Forward acceleration (m/s²)
%   alpha_out    - Angular acceleration (rad/s²)
%
% Strategy:
%   1. Compute desired acceleration to reach target velocity
%   2. Apply jerk limit to acceleration change
%   3. Apply acceleration limit
%   4. Integrate to get smoothed velocity
%   5. Apply velocity limit
%
% Copyright 2025 - GIK 9DOF Mobile Manipulator Project
% Jetson AGX Orin Deployment - Velocity Smoothing Module

%% Input Validation
validateattributes(vx_target, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'vx_target');
validateattributes(wz_target, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'wz_target');
validateattributes(vx_prev, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'vx_prev');
validateattributes(wz_prev, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'wz_prev');
validateattributes(ax_prev, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'ax_prev');
validateattributes(alpha_prev, {'double'}, {'scalar', 'real'}, 'smoothVelocityCommand', 'alpha_prev');
validateattributes(dt, {'double'}, {'scalar', 'positive'}, 'smoothVelocityCommand', 'dt');

%% Default Parameters
if nargin < 8 || isempty(params)
    params = struct();
end

if ~isfield(params, 'vx_max'), params.vx_max = 1.5; end           % m/s
if ~isfield(params, 'ax_max'), params.ax_max = 1.0; end           % m/s²
if ~isfield(params, 'jx_max'), params.jx_max = 5.0; end           % m/s³
if ~isfield(params, 'wz_max'), params.wz_max = 2.0; end           % rad/s
if ~isfield(params, 'alpha_max'), params.alpha_max = 3.0; end     % rad/s²
if ~isfield(params, 'jerk_wz_max'), params.jerk_wz_max = 10.0; end % rad/s³

%% Clamp targets to velocity limits
vx_target = max(-params.vx_max, min(params.vx_max, vx_target));
wz_target = max(-params.wz_max, min(params.wz_max, wz_target));

%% Forward Velocity Smoothing with S-Curve
% Compute desired acceleration
ax_desired = (vx_target - vx_prev) / dt;

% Apply jerk limit to acceleration change
jerk_vx = (ax_desired - ax_prev) / dt;
if abs(jerk_vx) > params.jx_max
    jerk_vx = sign(jerk_vx) * params.jx_max;
end

% Integrate jerk to get acceleration
ax_out = ax_prev + jerk_vx * dt;

% Apply acceleration limit
if abs(ax_out) > params.ax_max
    ax_out = sign(ax_out) * params.ax_max;
end

% Integrate acceleration to get velocity
vx_smooth = vx_prev + ax_out * dt;

% Apply velocity limit
if abs(vx_smooth) > params.vx_max
    vx_smooth = sign(vx_smooth) * params.vx_max;
    % If we hit velocity limit, set acceleration to zero
    ax_out = 0.0;
end

%% Angular Velocity Smoothing with S-Curve
% Compute desired angular acceleration
alpha_desired = (wz_target - wz_prev) / dt;

% Apply angular jerk limit
jerk_wz = (alpha_desired - alpha_prev) / dt;
if abs(jerk_wz) > params.jerk_wz_max
    jerk_wz = sign(jerk_wz) * params.jerk_wz_max;
end

% Integrate jerk to get angular acceleration
alpha_out = alpha_prev + jerk_wz * dt;

% Apply angular acceleration limit
if abs(alpha_out) > params.alpha_max
    alpha_out = sign(alpha_out) * params.alpha_max;
end

% Integrate angular acceleration to get angular velocity
wz_smooth = wz_prev + alpha_out * dt;

% Apply angular velocity limit
if abs(wz_smooth) > params.wz_max
    wz_smooth = sign(wz_smooth) * params.wz_max;
    % If we hit velocity limit, set acceleration to zero
    alpha_out = 0.0;
end

end
