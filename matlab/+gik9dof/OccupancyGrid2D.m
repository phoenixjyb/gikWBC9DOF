classdef OccupancyGrid2D
    %OCCUPANCYGRID2D Binary occupancy map for path planning (code-generation compatible)
    %   Fixed-size grid with world coordinate mapping
    
    properties
        data logical   % Occupancy data [size_y × size_x]: false=free, true=occupied
        
        resolution double  % [m/cell] Grid resolution
        origin_x double    % [m] World x coordinate of grid(1,1)
        origin_y double    % [m] World y coordinate of grid(1,1)
        
        size_x int32       % Number of cells in x direction
        size_y int32       % Number of cells in y direction
    end
    
    methods
        function obj = OccupancyGrid2D(resolution, size_x, size_y, origin_x, origin_y)
            %OCCUPANCYGRID2D Constructor
            %   Inputs:
            %       resolution: [m/cell] Grid cell size
            %       size_x, size_y: Grid dimensions [cells]
            %       origin_x, origin_y: World coordinates of grid(1,1)
            
            if nargin > 0
                obj.resolution = resolution;
                obj.size_x = int32(size_x);
                obj.size_y = int32(size_y);
                obj.origin_x = origin_x;
                obj.origin_y = origin_y;
                
                % Initialize empty grid (all free)
                obj.data = false(size_y, size_x);
            end
        end
        
        function grid_x = worldToGridX(obj, x)
            %WORLDTOGRIDX Convert world x coordinate to grid index
            grid_x = int32(floor((x - obj.origin_x) / obj.resolution)) + 1;
        end
        
        function grid_y = worldToGridY(obj, y)
            %WORLDTOGRADY Convert world y coordinate to grid index
            grid_y = int32(floor((y - obj.origin_y) / obj.resolution)) + 1;
        end
        
        function x = gridToWorldX(obj, grid_x)
            %GRIDTOWORLDX Convert grid index to world x coordinate
            x = obj.origin_x + double(grid_x - 1) * obj.resolution;
        end
        
        function y = gridToWorldY(obj, grid_y)
            %GRIDTOWORLDY Convert grid index to world y coordinate
            y = obj.origin_y + double(grid_y - 1) * obj.resolution;
        end
        
        function in_bounds = isInBounds(obj, grid_x, grid_y)
            %ISINBOUNDS Check if grid coordinates are within bounds
            in_bounds = (grid_x >= 1 && grid_x <= obj.size_x && ...
                         grid_y >= 1 && grid_y <= obj.size_y);
        end
        
        function occupied = isOccupied(obj, grid_x, grid_y)
            %ISOCCUPIED Check if cell is occupied
            if ~obj.isInBounds(grid_x, grid_y)
                occupied = true;  % Out of bounds treated as occupied
            else
                occupied = obj.data(grid_y, grid_x);
            end
        end
    end
end
