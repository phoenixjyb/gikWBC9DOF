function grid = inflateObstacles(grid, inflation_radius_m)
%INFLATEOBSTACLES Expand obstacles by robot radius for safety margin
%   Uses morphological dilation to create safety buffer
%
%   Inputs:
%       grid: OccupancyGrid2D object
%       inflation_radius_m: Inflation radius [m] (typically robot_radius + margin)
%
%   Output:
%       grid: OccupancyGrid2D with inflated obstacles

    % Convert radius to grid cells
    radius_cells = int32(ceil(inflation_radius_m / grid.resolution));
    
    % Handle zero radius (no inflation)
    if radius_cells <= 0
        return;
    end
    
    % Create circular structuring element
    diameter = 2 * radius_cells + 1;
    SE = false(diameter, diameter);
    center = radius_cells + 1;
    
    for dy = 1:diameter
        for dx = 1:diameter
            dist_sq = (dx - center)^2 + (dy - center)^2;
            if dist_sq <= double(radius_cells)^2
                SE(dy, dx) = true;
            end
        end
    end
    
    % Dilate occupancy grid (code-generation compatible version)
    % Manual implementation of morphological dilation
    inflated = false(grid.size_y, grid.size_x);
    
    for gy = 1:grid.size_y
        for gx = 1:grid.size_x
            if grid.data(gy, gx)
                % This cell is occupied, inflate it
                for dy = 1:diameter
                    for dx = 1:diameter
                        if SE(dy, dx)
                            % Offset from center
                            offset_y = dy - center;
                            offset_x = dx - center;
                            
                            new_gy = gy + offset_y;
                            new_gx = gx + offset_x;
                            
                            % Check bounds
                            if new_gy >= 1 && new_gy <= grid.size_y && ...
                               new_gx >= 1 && new_gx <= grid.size_x
                                inflated(new_gy, new_gx) = true;
                            end
                        end
                    end
                end
            end
        end
    end
    
    % Update grid
    grid.data = inflated;
end
