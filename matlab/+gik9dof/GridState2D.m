classdef GridState2D
    %GRIDSTATE2D 2D grid-based A* state representation (code-generation compatible)
    %   Simple (x,y) state for validating search engine before adding SE(2)
    
    properties
        % Continuous position
        x double       % World x coordinate [m]
        y double       % World y coordinate [m]
        
        % Discretized grid indices
        grid_x int32   % Grid cell index in x
        grid_y int32   % Grid cell index in y
        
        % A* search metadata
        g_cost double  % Cost from start
        h_cost double  % Heuristic to goal
        f_cost double  % Total cost (g + h)
        
        % Parent tracking
        parent_x int32 % Parent grid x (-1 for start)
        parent_y int32 % Parent grid y (-1 for start)
    end
    
    methods
        function obj = GridState2D(x, y)
            %GRIDSTATE2D Constructor
            if nargin > 0
                obj.x = x;
                obj.y = y;
                obj.grid_x = int32(0);
                obj.grid_y = int32(0);
                obj.g_cost = 0.0;
                obj.h_cost = 0.0;
                obj.f_cost = 0.0;
                obj.parent_x = int32(-1);
                obj.parent_y = int32(-1);
            end
        end
    end
end
