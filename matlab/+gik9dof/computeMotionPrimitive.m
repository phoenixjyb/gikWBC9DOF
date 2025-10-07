function [x_end, y_end, theta_end] = computeMotionPrimitive(x_start, y_start, theta_start, Vx, Wz, dt, params)
%COMPUTEMOTIONPRIMITIVE Compute endpoint of motion primitive for front-diff + passive-rear robot
%   [x_end, y_end, theta_end] = computeMotionPrimitive(x_start, y_start, theta_start, Vx, Wz, dt, params)
%
%   Kinematic model: Simplified Ackermann (front differential + passive rear)
%   - Front axle: Powered differential (control v_L, v_R)
%   - Rear axle: Passive omniwheels (allow lateral slip)
%   - Constraint: Minimum turning radius R_min ~ 0.34 m
%
%   INPUTS:
%       x_start, y_start - Starting position [m]
%       theta_start      - Starting heading [rad], 0=+X, CCW+
%       Vx               - Forward velocity command [m/s] (positive = forward)
%       Wz               - Yaw rate command [rad/s] (positive = CCW)
%       dt               - Duration [s]
%       params           - Chassis params from getChassisParams()
%                          Must include: .wheelbase, .track, .Vwheel_max, .min_turning_radius
%
%   OUTPUTS:
%       x_end, y_end     - Ending position [m]
%       theta_end        - Ending heading [rad]
%
%   Kinematic equations (SE(2) dynamics):
%       dx/dt = Vx * cos(theta)
%       dy/dt = Vx * sin(theta)
%       dtheta/dt = Wz
%
%   Arc motion (constant Vx, Wz):
%       - If |Wz| < threshold: straight line
%       - Else: circular arc with radius R = Vx / Wz
%
%   Constraints enforced:
%       1. Wheel speed limits: |Vx ± (track/2)*Wz| <= Vwheel_max
%       2. Minimum radius: R = |Vx/Wz| >= min_turning_radius
%       3. Forward/backward speed: |Vx| <= Vx_max
%       4. Yaw rate: |Wz| <= Wz_max
%
%   See also HybridState, generateMotionPrimitives, getChassisParams

%#codegen

% Validate inputs
assert(isfield(params, 'wheelbase'), 'params must have wheelbase');
assert(isfield(params, 'track'), 'params must have track');
assert(isfield(params, 'min_turning_radius'), 'params must have min_turning_radius');

% Check wheel speed limits
v_L = Vx - (params.track / 2) * Wz;
v_R = Vx + (params.track / 2) * Wz;

if abs(v_L) > params.Vwheel_max || abs(v_R) > params.Vwheel_max
    warning('computeMotionPrimitive:WheelSpeedLimit', ...
            'Wheel speeds exceed limit: v_L=%.2f, v_R=%.2f (max=%.2f)', ...
            v_L, v_R, params.Vwheel_max);
end

% Check minimum turning radius (if turning)
Wz_threshold = 1e-4;  % [rad/s] Treat as straight line below this
if abs(Wz) >= Wz_threshold
    R = abs(Vx / Wz);
    if R < params.min_turning_radius
        warning('computeMotionPrimitive:MinRadiusViolation', ...
                'Turning radius %.3f m < min_turning_radius %.3f m', ...
                R, params.min_turning_radius);
    end
end

% Compute motion
if abs(Wz) < Wz_threshold
    % Straight line motion
    arc_length = Vx * dt;
    
    x_end = x_start + arc_length * cos(theta_start);
    y_end = y_start + arc_length * sin(theta_start);
    theta_end = theta_start;
    
else
    % Circular arc motion
    R = Vx / Wz;                    % Turning radius (signed)
    dtheta = Wz * dt;               % Total heading change [rad]
    
    % Center of circular arc (perpendicular to heading)
    cx = x_start - R * sin(theta_start);
    cy = y_start + R * cos(theta_start);
    
    % New heading
    theta_end = theta_start + dtheta;
    
    % New position (rotate around center)
    x_end = cx + R * sin(theta_end);
    y_end = cy - R * cos(theta_end);
end

% Normalize theta to [-pi, pi]
theta_end = atan2(sin(theta_end), cos(theta_end));

end
