function params = getChassisParams(variant)
%GETCHASSISPARAMS Return WHEELTEC chassis parameters for path planning
%   params = getChassisParams() returns default "fourwheel" differential platform
%   params = getChassisParams('compact') - Top diff chassis (0.329m track)
%   params = getChassisParams('fourwheel') - Four-wheel diff (0.573m track) ← DEFAULT
%
%   YOUR PLATFORM: WHEELTEC Four-wheel Differential Drive
%   - Track width: 0.573 m (wider platform, Types 2-5)
%   - Wheel radius: 0.1075 m (107.5mm, 215mm diameter)
%   - Max wheel speed: 1.5 m/s
%   - Differential drive kinematics (NOT Ackermann steering)
%
%   Based on firmware analysis in docs/chassis-summary.txt
%
%   Returns struct with fields:
%       .chassis_type    - 'differential' (not Ackermann/holonomic)
%       .track          - Wheel track width [m]
%       .wheel_radius   - Drive wheel radius [m]
%       .robot_length   - Front-to-back dimension [m] (estimated)
%       .robot_width    - Side-to-side dimension [m] (track + clearance)
%       .robot_radius   - Conservative bounding circle [m]
%       .Vwheel_max     - Max wheel speed [m/s]
%       .Vx_max         - Max forward speed [m/s]
%       .Wz_max         - Max yaw rate [rad/s] (calculated from kinematics)
%       .accel_max      - Max acceleration [m/s^2] (from ramp rate)
%
%   See also gik9dof.control.defaultUnifiedParams

arguments
    variant (1,:) char {mustBeMember(variant, {'compact', 'fourwheel'})} = 'fourwheel'
end

params = struct();
params.chassis_type = 'differential';  % Diff-drive (not Ackermann)

switch variant
    case 'compact'
        % Top diff chassis (Type 0-1)
        params.track = 0.329;           % [m] Wheel track width
        params.wheel_radius = 0.0625;   % [m] 125mm diameter wheels
        params.robot_length = 0.40;     % [m] Estimated base length
        params.robot_width = 0.45;      % [m] Track + bumper clearance
        
    case 'fourwheel'
        % Four-wheel diff chassis (Type 2-5)
        params.track = 0.573;           % [m] Wider track
        params.wheel_radius = 0.1075;   % [m] 215mm diameter wheels
        params.robot_length = 0.60;     % [m] Larger platform
        params.robot_width = 0.70;      % [m] Track + clearance
end

% Conservative bounding circle (for collision checking)
params.robot_radius = sqrt((params.robot_length/2)^2 + (params.robot_width/2)^2);

% Velocity limits (from firmware + existing controller)
params.Vwheel_max = 1.5;     % [m/s] Per-wheel limit (verified in defaultUnifiedParams)
params.Vx_max = 0.8;         % [m/s] Conservative forward speed (existing)

% Differential drive yaw rate limit (from kinematics)
% |Wz| <= 2 * (Vwheel_max - |Vx|) / track
% For pure rotation (Vx=0): Wz_max = 2 * Vwheel_max / track
params.Wz_max_pure_spin = 2 * params.Vwheel_max / params.track;  % [rad/s]

% Conservative yaw limit for mixed motion (Vx = Vx_max)
params.Wz_max_at_speed = 2 * (params.Vwheel_max - params.Vx_max) / params.track;

% Use the more restrictive limit for planning
params.Wz_max = min(params.Wz_max_pure_spin, 2.5);  % Cap at firmware limit

% Acceleration limit (from firmware ramp: 0.02 m/s per 10ms = 2 m/s^2)
params.accel_max = 2.0;      % [m/s^2]

% Minimum turning radius (for Hybrid A* motion primitives)
% For differential drive, min radius occurs at max yaw rate
% R_min = Vx_max / Wz_max
params.min_turning_radius = params.Vx_max / params.Wz_max;  % [m]

% Safety margins
params.safety_margin = 0.05;  % [m] Extra clearance (5cm)
params.inflation_radius = params.robot_radius + params.safety_margin;

% Planning parameters
params.max_speed_for_planning = params.Vx_max;  % Use conservative forward speed
params.nominal_speed = 0.5;  % [m/s] Typical cruising speed

fprintf('WHEELTEC Chassis Parameters (%s variant):\n', variant);
fprintf('  Track width:        %.3f m\n', params.track);
fprintf('  Wheel radius:       %.3f m\n', params.wheel_radius);
fprintf('  Robot radius:       %.3f m (bounding circle)\n', params.robot_radius);
fprintf('  Inflation radius:   %.3f m (with safety margin)\n', params.inflation_radius);
fprintf('  Max forward speed:  %.2f m/s\n', params.Vx_max);
fprintf('  Max yaw (spin):     %.2f rad/s (%.1f deg/s)\n', ...
        params.Wz_max_pure_spin, rad2deg(params.Wz_max_pure_spin));
fprintf('  Max yaw (driving):  %.2f rad/s (%.1f deg/s)\n', ...
        params.Wz_max_at_speed, rad2deg(params.Wz_max_at_speed));
fprintf('  Min turn radius:    %.2f m\n', params.min_turning_radius);
fprintf('  Max acceleration:   %.2f m/s^2\n', params.accel_max);

end
