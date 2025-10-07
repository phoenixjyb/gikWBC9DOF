function [path, success] = planGridAStar(start_x, start_y, goal_x, goal_y, grid, max_iter)
%PLANGRIDASTAR 2D grid-based A* path planner (code-generation compatible)
%   Simple A* implementation for validation before Hybrid A*
%
%   Inputs:
%       start_x, start_y: Start position [m]
%       goal_x, goal_y: Goal position [m]
%       grid: OccupancyGrid2D object
%       max_iter: Maximum search iterations
%
%   Outputs:
%       path: Struct array with fields .x, .y (world coordinates)
%       success: True if path found

    % Constants
    MAX_QUEUE_SIZE = 5000;  % Open set capacity
    SQRT2 = 1.414213562373095;
    
    % Convert start/goal to grid coordinates
    start_gx = grid.worldToGridX(start_x);
    start_gy = grid.worldToGridY(start_y);
    goal_gx = grid.worldToGridX(goal_x);
    goal_gy = grid.worldToGridY(goal_y);
    
    % Validate start/goal
    if grid.isOccupied(start_gx, start_gy) || grid.isOccupied(goal_gx, goal_gy)
        path = struct('x', [], 'y', []);
        success = false;
        return;
    end
    
    % Initialize visited set (closed list)
    visited = false(grid.size_y, grid.size_x);
    
    % Initialize g_cost map
    g_cost_map = inf(grid.size_y, grid.size_x);
    g_cost_map(start_gy, start_gx) = 0;
    
    % Initialize parent map
    parent_x = zeros(grid.size_y, grid.size_x, 'int32');
    parent_y = zeros(grid.size_y, grid.size_x, 'int32');
    
    % Initialize priority queue (open set)
    % Each entry: [f_cost, grid_x, grid_y]
    queue = zeros(MAX_QUEUE_SIZE, 3);
    queue_size = int32(0);
    
    % Add start to queue
    h_start = heuristic(start_gx, start_gy, goal_gx, goal_gy);
    f_start = 0 + h_start;
    queue_size = queue_size + 1;
    queue(queue_size, :) = [f_start, double(start_gx), double(start_gy)];
    
    % A* main loop
    iter = int32(0);
    while queue_size > 0 && iter < max_iter
        iter = iter + 1;
        
        % Pop state with lowest f_cost (min-heap extract)
        [~, min_idx] = min(queue(1:queue_size, 1));
        current_gx = int32(queue(min_idx, 2));
        current_gy = int32(queue(min_idx, 3));
        
        % Remove from queue (swap with last element)
        queue(min_idx, :) = queue(queue_size, :);
        queue_size = queue_size - 1;
        
        % Check if goal reached
        if current_gx == goal_gx && current_gy == goal_gy
            % Reconstruct path
            path = reconstructPath(goal_gx, goal_gy, parent_x, parent_y, grid);
            success = true;
            return;
        end
        
        % Skip if already visited
        if visited(current_gy, current_gx)
            continue;
        end
        
        % Mark as visited
        visited(current_gy, current_gx) = true;
        current_g = g_cost_map(current_gy, current_gx);
        
        % Expand 8-connected neighbors
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;  % Skip self
                end
                
                neighbor_gx = current_gx + int32(dx);
                neighbor_gy = current_gy + int32(dy);
                
                % Check bounds and occupancy
                if ~grid.isInBounds(neighbor_gx, neighbor_gy)
                    continue;
                end
                if grid.isOccupied(neighbor_gx, neighbor_gy)
                    continue;
                end
                if visited(neighbor_gy, neighbor_gx)
                    continue;
                end
                
                % Compute neighbor g_cost
                if dx ~= 0 && dy ~= 0
                    step_cost = SQRT2;  % Diagonal
                else
                    step_cost = 1.0;     % Cardinal
                end
                tentative_g = current_g + step_cost;
                
                % Check if this path is better
                if tentative_g < g_cost_map(neighbor_gy, neighbor_gx)
                    % Update g_cost and parent
                    g_cost_map(neighbor_gy, neighbor_gx) = tentative_g;
                    parent_x(neighbor_gy, neighbor_gx) = current_gx;
                    parent_y(neighbor_gy, neighbor_gx) = current_gy;
                    
                    % Compute f_cost and add to queue
                    h = heuristic(neighbor_gx, neighbor_gy, goal_gx, goal_gy);
                    f = tentative_g + h;
                    
                    if queue_size < MAX_QUEUE_SIZE
                        queue_size = queue_size + 1;
                        queue(queue_size, :) = [f, double(neighbor_gx), double(neighbor_gy)];
                    else
                        warning('Priority queue full, search truncated');
                    end
                end
            end
        end
    end
    
    % No path found
    path = struct('x', [], 'y', []);
    success = false;
end


%% Helper Functions

function h = heuristic(gx1, gy1, gx2, gy2)
    % Euclidean distance heuristic
    dx = double(gx2 - gx1);
    dy = double(gy2 - gy1);
    h = sqrt(dx*dx + dy*dy);
end


function path = reconstructPath(goal_gx, goal_gy, parent_x, parent_y, grid)
    % Reconstruct path from goal to start using parent map
    
    MAX_PATH_LENGTH = 1000;
    path_x = zeros(MAX_PATH_LENGTH, 1);
    path_y = zeros(MAX_PATH_LENGTH, 1);
    
    current_gx = goal_gx;
    current_gy = goal_gy;
    idx = int32(1);
    
    % Trace back from goal to start
    while idx <= MAX_PATH_LENGTH
        % Store current position (world coordinates)
        path_x(idx) = grid.gridToWorldX(current_gx);
        path_y(idx) = grid.gridToWorldY(current_gy);
        idx = idx + 1;
        
        % Get parent
        px = parent_x(current_gy, current_gx);
        py = parent_y(current_gy, current_gx);
        
        % Check if reached start (parent = 0)
        if px == 0 && py == 0
            break;
        end
        
        current_gx = px;
        current_gy = py;
    end
    
    % Reverse path (start -> goal)
    num_points = idx - 1;
    path.x = flip(path_x(1:num_points));
    path.y = flip(path_y(1:num_points));
end
