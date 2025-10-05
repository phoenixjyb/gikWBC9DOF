function env = environmentConfig()
%ENVIRONMENTCONFIG Centralised environment definition for obstacles & margins.
%   env = gik9dof.environmentConfig() returns a struct containing the shared
%   environment configuration used by both holistic and staged controllers.
%   Fields include:
%       BaseHome      - 1x3 base joint positions for the home configuration.
%       FloorDiscs    - Struct array describing floor obstacles (Center, Radius,
%                       SafetyMargin, Name optional).
%       DistanceMargin- Additional radial clearance enforced by the solver.
%       DistanceWeight- Weight applied to distance constraints.

persistent cachedEnv
if ~isempty(cachedEnv)
    env = cachedEnv;
    return
end

env = struct();

env.BaseHome = [-2, -2, 0];

radius = 0.10;
height = 0.15;
safetyMargin = 0.05;
distanceMargin = 0.15;
distanceWeight = 5.0;

center1 = [-1, -1];

traj = jsondecode(fileread(fullfile(gik9dof.internal.projectRoot(), '1_pull_world_scaled.json')));
waypointIdx = min(112, numel(traj.poses));
wp = traj.poses(waypointIdx).position;
center2 = [wp(1), wp(2)];

discTemplate = @(center, name) struct( ...
    'Center', center, ...
    'Radius', radius, ...
    'SafetyMargin', safetyMargin, ...
    'Name', string(name), ...
    'Height', height, ...
    'height', height);

env.FloorDiscs = [discTemplate(center1, 'floor_disc_1'), discTemplate(center2, 'floor_disc_2')];

env.DistanceMargin = distanceMargin;
env.DistanceWeight = distanceWeight;

cachedEnv = env;
end
