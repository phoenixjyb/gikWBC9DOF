function grid = updateOccupancyGrid(grid, lidar_points, robot_pose, params)
%UPDATEOCCUPANCYGRID Fuse lidar data into occupancy grid (code-generation compatible)
%   Bayesian update with ray casting for free space inference
%
%   Inputs:
%       grid: OccupancyGrid2D object (current map)
%       lidar_points: [N × 2] array (x, y in robot frame) [m]
%       robot_pose: [x, y, theta] current robot pose in map frame
%       params: Struct with perception parameters
%
%   Output:
%       grid: Updated OccupancyGrid2D with fused lidar data

    % Extract parameters
    MAX_RANGE = params.lidar_max_range;      % e.g., 10m
    MIN_RANGE = params.lidar_min_range;      % e.g., 0.1m
    DECAY_RATE = params.map_decay_rate;      % e.g., 0.95
    HIT_PROB = params.obstacle_hit_prob;     % e.g., 0.7
    MISS_PROB = params.free_space_prob;      % e.g., 0.3
    
    % Pre-compute rotation matrix
    cos_theta = cos(robot_pose(3));
    sin_theta = sin(robot_pose(3));
    
    % Step 1: Decay existing occupancy (temporal decay)
    % This allows map to "forget" old dynamic obstacles
    for gy = 1:grid.size_y
        for gx = 1:grid.size_x
            if grid.data(gy, gx)
                % Decay occupied cells (probabilistic approach would use continuous values)
                % For binary, we can skip this or implement as counter
                % For now, keep occupied cells as-is (static assumption)
            end
        end
    end
    
    % Step 2: Process each lidar point
    num_points = size(lidar_points, 1);
    
    for i = 1:num_points
        % Point in robot frame
        px_robot = lidar_points(i, 1);
        py_robot = lidar_points(i, 2);
        
        % Compute range
        range = sqrt(px_robot*px_robot + py_robot*py_robot);
        
        % Skip if out of valid range
        if range < MIN_RANGE || range > MAX_RANGE
            continue;
        end
        
        % Transform to map frame
        px_map = robot_pose(1) + cos_theta * px_robot - sin_theta * py_robot;
        py_map = robot_pose(2) + sin_theta * px_robot + cos_theta * py_robot;
        
        % Convert to grid coordinates
        gx_hit = grid.worldToGridX(px_map);
        gy_hit = grid.worldToGridY(py_map);
        
        % Ray casting: Mark free space along ray
        [free_cells_x, free_cells_y, num_free] = bresenhamRayCast(...
            robot_pose(1), robot_pose(2), px_map, py_map, grid);
        
        % Update free cells (decrease occupancy probability)
        for j = 1:num_free
            gx = free_cells_x(j);
            gy = free_cells_y(j);
            if grid.isInBounds(gx, gy)
                % For binary grid, just mark as free
                grid.data(gy, gx) = false;
            end
        end
        
        % Mark hit point as occupied
        if grid.isInBounds(gx_hit, gy_hit)
            grid.data(gy_hit, gx_hit) = true;
        end
    end
end


function [cells_x, cells_y, num_cells] = bresenhamRayCast(x0, y0, x1, y1, grid)
    %BRESENHAMRAYCAST Ray casting using Bresenham's line algorithm (code-gen ready)
    %   Returns grid cells along ray from (x0,y0) to (x1,y1)
    %
    %   Inputs:
    %       x0, y0: Ray start (world coordinates) [m]
    %       x1, y1: Ray end (world coordinates) [m]
    %       grid: OccupancyGrid2D object
    %
    %   Outputs:
    %       cells_x, cells_y: Grid indices along ray (excluding endpoint)
    %       num_cells: Number of cells in ray
    
    MAX_CELLS = int32(300);  % Max ray length in cells (30m @ 10cm)
    cells_x = zeros(MAX_CELLS, 1, 'int32');
    cells_y = zeros(MAX_CELLS, 1, 'int32');
    
    % Convert to grid coordinates
    gx0 = grid.worldToGridX(x0);
    gy0 = grid.worldToGridY(y0);
    gx1 = grid.worldToGridX(x1);
    gy1 = grid.worldToGridY(y1);
    
    % Bresenham's algorithm
    dx = abs(gx1 - gx0);
    dy = abs(gy1 - gy0);
    
    if gx1 >= gx0
        sx = int32(1);
    else
        sx = int32(-1);
    end
    
    if gy1 >= gy0
        sy = int32(1);
    else
        sy = int32(-1);
    end
    
    err = dx - dy;
    
    gx = gx0;
    gy = gy0;
    idx = int32(1);
    
    % Trace ray (exclude endpoint)
    while idx <= MAX_CELLS
        % Stop before hitting endpoint (endpoint is obstacle)
        if gx == gx1 && gy == gy1
            break;
        end
        
        % Store current cell
        cells_x(idx) = gx;
        cells_y(idx) = gy;
        idx = idx + 1;
        
        % Bresenham step
        e2 = int32(2) * err;
        
        if e2 > -dy
            err = err - dy;
            gx = gx + sx;
        end
        
        if e2 < dx
            err = err + dx;
            gy = gy + sy;
        end
    end
    
    num_cells = idx - 1;
    cells_x = cells_x(1:num_cells);
    cells_y = cells_y(1:num_cells);
end
