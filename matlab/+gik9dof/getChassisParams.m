function params = getChassisParams(variant)
%GETCHASSISPARAMS Return WHEELTEC chassis parameters for path planning
%   params = getChassisParams() returns default "fourwheel" platform
%   params = getChassisParams('compact') - Top diff chassis (0.329m track)
%   params = getChassisParams('fourwheel') - Four-wheel platform (0.573m track) ← DEFAULT
%
%   YOUR PLATFORM: WHEELTEC Four-Wheel Drive with Passive Rear
%   - Front axle: Differential drive (two powered wheels)
%   - Rear axle: Passive omni-wheels (lateral rolling rollers)
%   - Wheelbase: 0.36 m (front-to-rear distance)
%   - Track width: 0.573 m (left-to-right at front wheels)
%   - Wheel radius: 0.1075 m (107.5mm, 215mm diameter drive wheels)
%   - Kinematics: Simplified Ackermann (front-wheel steer via differential)
%
%   This is a HYBRID kinematic model:
%   - Driven like differential drive (front axle, left/right speeds)
%   - Constrained like Ackermann (passive rear creates ICR)
%   - Minimum turning radius exists (NOT zero radius!)
%
%   Based on firmware analysis in docs/chassis-summary.txt
%
%   Returns struct with fields:
%       .chassis_type    - 'front_diff_rear_passive' (unique hybrid)
%       .wheelbase       - Front-to-rear distance [m] ← KEY PARAMETER
%       .track           - Front wheel track width [m]
%       .wheel_radius    - Drive wheel radius [m]
%       .robot_length    - Front-to-back dimension [m]
%       .robot_width     - Side-to-side dimension [m]
%       .robot_radius    - Conservative bounding circle [m]
%       .Vwheel_max      - Max wheel speed [m/s]
%       .Vx_max          - Max forward speed [m/s]
%       .Wz_max          - Max yaw rate [rad/s]
%       .min_turning_radius - Minimum turn radius [m] ← IMPORTANT
%       .accel_max       - Max acceleration [m/s^2]
%
%   See also gik9dof.control.defaultUnifiedParams

arguments
    variant (1,:) char {mustBeMember(variant, {'compact', 'fourwheel'})} = 'fourwheel'
end

params = struct();
params.chassis_type = 'front_diff_rear_passive';  % Hybrid: diff front + passive rear

switch variant
    case 'compact'
        % Top diff chassis (Type 0-1) - pure differential
        params.wheelbase = 0.20;        % [m] Estimated (if applicable)
        params.track = 0.329;           % [m] Wheel track width
        params.wheel_radius = 0.0625;   % [m] 125mm diameter wheels
        params.robot_length = 0.40;     % [m] Estimated base length
        params.robot_width = 0.45;      % [m] Track + bumper clearance
        
    case 'fourwheel'
        % Four-wheel platform: Front diff + Rear passive omni
        params.wheelbase = 0.36;        % [m] Front-to-rear axle distance ← YOUR SPEC
        params.track = 0.573;           % [m] Front track width
        params.wheel_radius = 0.1075;   % [m] 215mm diameter drive wheels
        params.robot_length = 0.60;     % [m] Including bumpers (wheelbase + clearance)
        params.robot_width = 0.70;      % [m] Track + side clearance
end

% Conservative bounding circle (for collision checking)
params.robot_radius = sqrt((params.robot_length/2)^2 + (params.robot_width/2)^2);

% Velocity limits (from firmware + existing controller)
params.Vwheel_max = 1.5;     % [m/s] Per-wheel limit (verified in defaultUnifiedParams)
params.Vx_max = 0.8;         % [m/s] Conservative forward speed (existing)

% KINEMATICS: Front diff + passive rear creates Instantaneous Center of Rotation (ICR)
% The passive rear wheels with lateral rollers allow the robot to pivot,
% but the ICR is constrained by geometry (NOT zero radius like pure diff-drive)

% Yaw rate limits (differential constraints at front axle)
% Front axle differential: |Wz| <= 2 * (Vwheel_max - |Vx|) / track
params.Wz_max_front_axle = 2 * params.Vwheel_max / params.track;  % [rad/s] Pure spin

% Conservative yaw limit for mixed motion (Vx = Vx_max)
params.Wz_max_at_speed = 2 * (params.Vwheel_max - params.Vx_max) / params.track;

% Use firmware limit (2.5 rad/s) as ultimate cap
params.Wz_max = min(params.Wz_max_front_axle, 2.5);  % [rad/s]

% Acceleration limit (from firmware ramp: 0.02 m/s per 10ms = 2 m/s^2)
params.accel_max = 2.0;      % [m/s^2]

% MINIMUM TURNING RADIUS (KEY for Hybrid A*)
% With passive rear, the robot pivots around a point related to wheelbase
% Approximation: R_min ≈ wheelbase / tan(max_virtual_steer_angle)
%
% For front diff + passive rear, the effective steering angle is:
%   tan(delta) = (v_R - v_L) / (Vx * track) * wheelbase
%
% At max differential (v_R - v_L = 2*Vwheel_max), assuming Vx ≈ Vwheel_max:
%   tan(delta_max) ≈ 2 * Vwheel_max / (Vwheel_max * track) * wheelbase
%                  = 2 * wheelbase / track
%
% Minimum radius: R_min = wheelbase / tan(delta_max)
%                       = wheelbase / (2 * wheelbase / track)
%                       = track / 2
%
% This is a SIMPLIFIED estimate. Actual min radius depends on tire slip,
% omni-wheel rolling resistance, and speed-dependent effects.

params.min_turning_radius_theoretical = params.track / 2;  % [m] Geometric minimum

% Conservative estimate (add 20% margin for real-world effects)
params.min_turning_radius = params.min_turning_radius_theoretical * 1.2;  % [m]

% Alternative calculation based on velocity limits
params.min_turning_radius_kinematic = params.Vx_max / params.Wz_max;  % [m]

% Use the larger (more conservative) of the two estimates
params.min_turning_radius = max(params.min_turning_radius, ...
                                 params.min_turning_radius_kinematic);

% Safety margins
params.safety_margin = 0.05;  % [m] Extra clearance (5cm)
params.inflation_radius = params.robot_radius + params.safety_margin;

% Planning parameters
params.max_speed_for_planning = params.Vx_max;  % Use conservative forward speed
params.nominal_speed = 0.5;  % [m/s] Typical cruising speed

fprintf('WHEELTEC Chassis Parameters (%s variant):\n', variant);
fprintf('  Chassis type:       %s\n', params.chassis_type);
fprintf('  Wheelbase:          %.3f m (front-to-rear)\n', params.wheelbase);
fprintf('  Track width:        %.3f m (front wheels)\n', params.track);
fprintf('  Wheel radius:       %.3f m\n', params.wheel_radius);
fprintf('  Robot radius:       %.3f m (bounding circle)\n', params.robot_radius);
fprintf('  Inflation radius:   %.3f m (with safety margin)\n', params.inflation_radius);
fprintf('  Max forward speed:  %.2f m/s\n', params.Vx_max);
fprintf('  Max yaw rate:       %.2f rad/s (%.1f deg/s)\n', ...
        params.Wz_max, rad2deg(params.Wz_max));
fprintf('  Yaw (at max speed): %.2f rad/s (%.1f deg/s)\n', ...
        params.Wz_max_at_speed, rad2deg(params.Wz_max_at_speed));
fprintf('  Min turn radius:    %.3f m (passive rear constraint!)\n', params.min_turning_radius);
fprintf('  Max acceleration:   %.2f m/s^2\n', params.accel_max);

end
