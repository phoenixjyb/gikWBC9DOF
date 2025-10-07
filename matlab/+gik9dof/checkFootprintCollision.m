function has_collision = checkFootprintCollision(x, y, theta, grid, params)
%CHECKFOOTPRINTCOLLISION Check if robot footprint at pose collides with obstacles
%   has_collision = checkFootprintCollision(x, y, theta, grid, params)
%
%   Checks if the robot's circular footprint at the given pose overlaps
%   with any obstacles in the occupancy grid.
%
%   INPUTS:
%       x, y     - Robot position [m] in world frame
%       theta    - Robot heading [rad] (not used for circular model)
%       grid     - OccupancyGrid2D object
%       params   - Chassis params with .robot_radius [m]
%
%   OUTPUTS:
%       has_collision - Boolean: true if footprint collides
%
%   Method:
%       Assumes grid is already inflated by robot_radius + margin
%       So we only need to check the center point
%
%   See also checkArcCollision, OccupancyGrid2D

%#codegen

% Use circular footprint (simple: check center point since grid is inflated)
if ~isfield(params, 'robot_radius')
    error('params must have robot_radius');
end

% Convert to grid coordinates
grid_x = grid.worldToGridX(x);
grid_y = grid.worldToGridY(y);

% Bounds check
if grid_x < 1 || grid_x > grid.size_x || ...
   grid_y < 1 || grid_y > grid.size_y
    has_collision = true;
    return;
end

% Occupancy check (grid is already inflated, so check center)
has_collision = grid.data(grid_y, grid_x) > 0.5;

end
