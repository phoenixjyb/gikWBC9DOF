function params = defaultUnifiedParams(profile)
%DEFAULTUNIFIEDPARAMS Load baseline chassis parameters for unified control.
%   params = gik9dof.control.defaultUnifiedParams() returns the wide-track
%   preset defined in config/chassis_profiles.yaml.
%
%   params = gik9dof.control.defaultUnifiedParams(profile) loads the named
%   profile (e.g. "compact_track") from the same YAML file.
%
arguments
    profile (1,1) string = "wide_track"
end

params = gik9dof.control.loadChassisProfile(profile);
end
