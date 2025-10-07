function has_collision = checkArcCollision(x_start, y_start, theta_start, Vx, Wz, dt, grid, params)
%CHECKARCCOLLISION Check if motion primitive arc collides with obstacles
%   has_collision = checkArcCollision(x_start, y_start, theta_start, Vx, Wz, dt, grid, params)
%
%   Samples points along the arc and checks robot footprint against occupancy grid.
%   Uses conservative circle-based collision model for efficiency.
%
%   INPUTS:
%       x_start, y_start - Starting position [m] in world frame
%       theta_start      - Starting heading [rad]
%       Vx               - Forward velocity [m/s]
%       Wz               - Yaw rate [rad/s]
%       dt               - Duration [s]
%       grid             - OccupancyGrid2D object (from gik9dof.OccupancyGrid2D)
%       params           - Chassis params (must have .robot_radius)
%
%   OUTPUTS:
%       has_collision - Boolean: true if arc collides, false if clear
%
%   Algorithm:
%       1. Sample arc at intervals (every 0.1m or finer)
%       2. For each sample point:
%          a. Check if robot center is in bounds
%          b. Check if robot footprint (circle of radius robot_radius) overlaps obstacles
%       3. Return true at first collision, false if all samples clear
%
%   Robot model: Conservative bounding circle (radius = 0.46m)
%   Grid assumes obstacles are already inflated by robot_radius + margin
%
%   See also computeMotionPrimitive, OccupancyGrid2D, inflateObstacles

%#codegen

% Validate inputs
assert(isfield(params, 'robot_radius'), 'params must have robot_radius');

% Sampling resolution: check every 0.1m along arc
arc_length = abs(Vx) * dt;
sample_distance = 0.1;  % [m] Check every 10cm
num_samples = max(2, ceil(arc_length / sample_distance));  % At least start + end

% Time samples
t_samples = linspace(0, dt, num_samples);

% Check each sample point along arc
for i = 1:num_samples
    t = t_samples(i);
    
    % Compute position at this point along arc
    if abs(Wz) < 1e-4
        % Straight line
        dist = Vx * t;
        x = x_start + dist * cos(theta_start);
        y = y_start + dist * sin(theta_start);
    else
        % Circular arc
        R = Vx / Wz;
        dtheta = Wz * t;
        
        cx = x_start - R * sin(theta_start);
        cy = y_start + R * cos(theta_start);
        
        theta_t = theta_start + dtheta;
        x = cx + R * sin(theta_t);
        y = cy - R * cos(theta_t);
    end
    
    % Convert world position to grid indices
    grid_x = grid.worldToGridX(x);
    grid_y = grid.worldToGridY(y);
    
    % Check if position is within grid bounds
    if grid_x < 1 || grid_x > grid.size_x || ...
       grid_y < 1 || grid_y > grid.size_y
        has_collision = true;  % Out of bounds = collision
        return;
    end
    
    % Check occupancy at robot center
    % Assumes grid is already inflated by robot_radius, so just check center point
    if grid.data(grid_y, grid_x) > 0.5  % Occupied threshold
        has_collision = true;
        return;
    end
end

% All samples clear
has_collision = false;

end
