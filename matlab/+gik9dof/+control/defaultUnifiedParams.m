function params = defaultUnifiedParams(track)
%DEFAULTUNIFIEDPARAMS Baseline parameter set for unified chassis control.
%   params = gik9dof.control.defaultUnifiedParams() returns a struct with
%   conservative settings for the compact-track platform (0.329 m).
%
%   params = gik9dof.control.defaultUnifiedParams(track) overrides the wheel
%   track (m). The remaining parameters follow the values documented in
%   unified_chassis_controller_summary.md.
%
arguments
    track (1,1) double {mustBePositive} = 0.329
end

params = struct();
params.track = track;
params.Vwheel_max = 1.5;     % m/s per wheel
params.Vx_max = 0.8;         % m/s forward clamp
params.W_max = 2.5;          % rad/s global yaw clamp
params.yawKp = 2.0;          % heading proportional gain
params.yawKff = 0.2;         % yaw feed-forward gain
end
