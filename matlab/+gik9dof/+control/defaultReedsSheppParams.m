function params = defaultReedsSheppParams()
%DEFAULTREEDSSHEPPPARAMS Baseline parameters for Stage B RS shortcutting.
%   params = gik9dof.control.defaultReedsSheppParams() returns a struct with
%   the fields expected by gik9dof.control.rsRefinePath. These defaults bias
%   toward forward motion, moderate smoothing effort, and code generation
%   compatibility. Pipeline code will override Rmin and inflationRadius with
%   environment-specific values prior to execution.
%
%   Fields:
%       Rmin               - Placeholder minimum turning radius (set to 0 so
%                            the caller can inject the chassis limit).
%       reverseCost        - Cost multiplier for reverse motions (default 5.0).
%       inflationRadius    - Placeholder inflation radius (negative triggers
%                            auto fill from environment).
%       validationDistance - Collision-check spacing along motions (meters).
%       step               - Sample spacing along RS segments (meters).
%       iters              - Shortcut attempts per call (int32).
%       maxSpan            - Maximum index span replaced per shortcut (int32).
%       lambdaCusp         - Penalty (meters) per gear change.
%       seed               - RNG seed (uint32, 0 keeps deterministic seeding).
%       allowReverse       - Permit reverse segments in shortcuts (default false).
%

params = struct( ...
    'Rmin', 0, ...
    'reverseCost', 5.0, ...
    'inflationRadius', -1.0, ...
    'validationDistance', 0.035, ...
    'step', 0.05, ...
    'iters', int32(600), ...
    'maxSpan', int32(160), ...
    'lambdaCusp', 3.0, ...
    'seed', uint32(0), ...
    'allowReverse', false);
end
