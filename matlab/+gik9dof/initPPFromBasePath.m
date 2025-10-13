function [follower, pathInfo] = initPPFromBasePath(basePath, chassisParams, options)
%INITPPFROMBASEPATH Initialize Pure Pursuit controller from base path.
%   [follower, pathInfo] = GIK9DOF.INITPPFROMBASEPATH(basePath, chassisParams)
%   creates and configures a purePursuitFollower object from a candidate
%   base path. The path is preprocessed using preparePathForFollower to
%   ensure proper spacing and curvature computation.
%
%   [follower, pathInfo] = GIK9DOF.INITPPFROMBASEPATH(..., options) supports:
%       LookaheadBase     - Base lookahead distance (m, default 0.6)
%       LookaheadVelGain  - Velocity-adaptive gain (default 0.3)
%       LookaheadTimeGain - Acceleration-adaptive gain (default 0.05)
%       ControllerMode    - 'blended', 'purePursuit', 'stanley' (default 'blended')
%       SampleTime        - Control timestep (s, default 0.1)
%       DesiredVelocity   - Target forward velocity (m/s, default 0.5)
%       ReverseEnabled    - Allow reverse motion (default false)
%       ApplyRefinement   - Apply RS+Clothoid smoothing (default false)
%       RefinementParams  - Parameters for rsRefinePath (default struct())
%
%   Returns:
%       follower   - Configured purePursuitFollower object
%       pathInfo   - Preprocessed path struct from preparePathForFollower
%
%   Example:
%       chassisParams = gik9dof.control.loadChassisProfile('wide_track');
%       basePath = [...]; % Nx3 [x, y, theta]
%       [follower, pathInfo] = gik9dof.initPPFromBasePath(basePath, chassisParams);
%       [vx, wz, status] = follower.step([x, y, theta], dt);
%
%   See also purePursuitFollower, preparePathForFollower, baseSeedFromEE.

arguments
    basePath (:,3) double
    chassisParams (1,1) struct
    options.LookaheadBase (1,1) double = 0.6
    options.LookaheadVelGain (1,1) double = 0.3
    options.LookaheadTimeGain (1,1) double = 0.05
    options.ControllerMode (1,1) string = "blended"
    options.SampleTime (1,1) double = 0.1
    options.DesiredVelocity (1,1) double = 0.5
    options.ReverseEnabled (1,1) logical = false
    options.ApplyRefinement (1,1) logical = false
    options.RefinementParams (1,1) struct = struct()
    options.Verbose (1,1) logical = false
end

if options.Verbose
    fprintf('Initializing Pure Pursuit controller...\n');
    fprintf('  Input path: %d waypoints\n', size(basePath, 1));
end

% Optional: Apply path refinement (RS shortcutting + Clothoid smoothing)
if options.ApplyRefinement
    if options.Verbose
        fprintf('  Applying path refinement (RS + Clothoid)...\n');
    end
    
    % Default refinement parameters if not provided
    if isempty(fieldnames(options.RefinementParams))
        rsParams = gik9dof.control.defaultReedsSheppParams();
        rsParams.max_iterations = 100;
        rsParams.collision_check = false; % No obstacles at this stage
    else
        rsParams = options.RefinementParams;
    end
    
    % Apply Reeds-Shepp shortcutting
    try
        [refinedPath, ~] = gik9dof.control.rsRefinePath(basePath, rsParams);
        
        % Apply Clothoid smoothing
        [basePath, ~] = gik9dof.control.rsClothoidRefine(refinedPath, rsParams);
        
        if options.Verbose
            fprintf('  Refinement complete: %d waypoints after smoothing\n', size(basePath, 1));
        end
    catch ME
        warning('Path refinement failed: %s. Using original path.', ME.message);
    end
end

% Prepare path for follower (interpolation, curvature, arc length)
% Build params struct from chassis params
prepareParams = struct();
if isfield(chassisParams, 'interp_spacing_min')
    prepareParams.interp_spacing_min = chassisParams.interp_spacing_min;
else
    prepareParams.interp_spacing_min = 0.05;
end
if isfield(chassisParams, 'waypoint_spacing')
    prepareParams.waypoint_spacing = chassisParams.waypoint_spacing;
else
    prepareParams.waypoint_spacing = 0.15;
end

pathInfo = gik9dof.control.preparePathForFollower(basePath, prepareParams);

% Create Pure Pursuit follower
follower = gik9dof.control.purePursuitFollower(pathInfo.States, ...
    'ChassisParams', chassisParams, ...
    'PathInfo', pathInfo, ...
    'SampleTime', options.SampleTime, ...
    'ControllerMode', options.ControllerMode, ...
    'LookaheadBase', options.LookaheadBase, ...
    'LookaheadVelGain', options.LookaheadVelGain, ...
    'LookaheadTimeGain', options.LookaheadTimeGain, ...
    'ReverseEnabled', options.ReverseEnabled || chassisParams.reverse_enabled);

if options.Verbose
    fprintf('Pure Pursuit controller initialized.\n');
    fprintf('  Lookahead: %.2f m (base) + velocity/acceleration adaptive\n', options.LookaheadBase);
    fprintf('  Controller mode: %s\n', options.ControllerMode);
    fprintf('  Path length: %.2f m\n', pathInfo.ArcLength(end));
end

end
