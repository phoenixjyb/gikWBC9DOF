function [WzOut, caps] = clampYawByWheelLimit(Vx, Wz, track, Vwheel_max, Wmax)
%CLAMPYAWBYWHEELLIMIT Apply yaw constraints derived from wheel and platform limits.
%   [WzOut, caps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, track, Vwheel_max, Wmax)
%   clamps the yaw command |Wz| so that the differential-drive wheels remain
%   within their linear speed limits. Inputs:
%       Vx          - commanded forward velocity (m/s)
%       Wz          - commanded yaw rate (rad/s)
%       track       - wheel track width (m)
%       Vwheel_max  - maximum per-wheel linear speed (m/s)
%       Wmax        - platform yaw rate clamp (rad/s)
%
%   Outputs:
%       WzOut       - clamped yaw rate command (rad/s)
%       caps        - struct with fields:
%                        .wheel   - yaw limit computed from wheel speeds
%                        .applied - final yaw limit after combining with Wmax
%
%   The wheel-derived limit assumes the standard diff-drive relation:
%       v_left  = Vx - 0.5 * track * Wz
%       v_right = Vx + 0.5 * track * Wz
%   Both |v_left| and |v_right| must remain <= Vwheel_max.

arguments
    Vx (1,1) double
    Wz (1,1) double
    track (1,1) double {mustBePositive}
    Vwheel_max (1,1) double {mustBePositive}
    Wmax (1,1) double {mustBePositive}
end

% Compute wheel-derived yaw cap
wheelMargin = max(0, Vwheel_max - abs(Vx));
if track <= 0
    Wwheel = 0;
else
    Wwheel = 2 * wheelMargin / track;
end

caps = struct();
caps.wheel = Wwheel;

Wcap = min(Wmax, Wwheel);
caps.applied = Wcap;

WzOut = max(-Wcap, min(Wcap, Wz));
end
