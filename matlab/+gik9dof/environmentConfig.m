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

env = struct();

env.BaseHome = [-2, -2, 0];

env.FloorDiscs = struct( ...
    'Center',      {[-0.2, 0.3], [0.8, -0.4]}, ...
    'Radius',      {0.35, 0.25}, ...
    'SafetyMargin',{0.08, 0.05}, ...
    'Name',        {"floor_disc_1", "floor_disc_2"});

env.DistanceMargin = 0.10;
env.DistanceWeight = 0.5;
end
