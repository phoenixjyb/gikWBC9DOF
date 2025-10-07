function [path, search_stats] = planHybridAStar(start_state, goal_state, occupancy_grid, options)
% planHybridAStar: SE(2) Hybrid A* path planning for non-holonomic robot
%
% Inputs:
%   start_state: HybridState struct (x, y, theta)
%   goal_state: HybridState struct (x, y, theta)
%   occupancy_grid: OccupancyGrid2D object (must be already inflated)
%   options: struct with fields (all optional):
%       max_iterations: Maximum search iterations (default: 10000)
%       goal_tolerance_xy: XY goal tolerance in meters (default: 0.2)
%       goal_tolerance_theta: Heading tolerance in radians (default: 0.3)
%       use_dubins_heuristic: true=Dubins, false=Euclidean (default: true)
%       timeout_sec: Maximum planning time in seconds (default: 5.0)
%
% Outputs:
%   path: Struct array with fields (x, y, theta, Vx, Wz, dt)
%       Empty if no path found
%   search_stats: Struct with fields:
%       success: true if path found
%       iterations: Number of iterations executed
%       nodes_expanded: Number of states expanded
%       planning_time_sec: Total planning time
%       path_cost: Total path cost (g-value at goal)
%       path_length: Number of waypoints in path

    % Parse options
    if nargin < 4
        options = struct();
    end
    max_iterations = getFieldOrDefault(options, 'max_iterations', 10000);
    goal_tol_xy = getFieldOrDefault(options, 'goal_tolerance_xy', 0.2);
    goal_tol_theta = getFieldOrDefault(options, 'goal_tolerance_theta', 0.3);
    use_dubins = getFieldOrDefault(options, 'use_dubins_heuristic', true);
    timeout_sec = getFieldOrDefault(options, 'timeout_sec', 5.0);
    
    % Get chassis parameters
    params = gik9dof.getChassisParams();
    R_min = params.min_turning_radius;
    
    % Generate motion primitives
    primitives = gik9dof.generateMotionPrimitives(params);
    
    % Initialize lattice discretization (matches design: 200x200x16)
    % Angular resolution: 360° / 16 = 22.5° per bin
    num_theta_bins = 16;
    theta_resolution = 2*pi / num_theta_bins;
    
    % Discretize start and goal states
    start_state.grid_x = occupancy_grid.worldToGridX(start_state.x);
    start_state.grid_y = occupancy_grid.worldToGridY(start_state.y);
    start_state.theta_bin = angleToBin(start_state.theta, num_theta_bins);
    start_state.g = 0;
    start_state.h = computeHeuristic(start_state, goal_state, R_min, use_dubins);
    start_state.f = start_state.g + start_state.h;
    
    goal_state.grid_x = occupancy_grid.worldToGridX(goal_state.x);
    goal_state.grid_y = occupancy_grid.worldToGridY(goal_state.y);
    goal_state.theta_bin = angleToBin(goal_state.theta, num_theta_bins);
    
    % Check start/goal validity
    if gik9dof.checkFootprintCollision(start_state.x, start_state.y, start_state.theta, occupancy_grid, params)
        warning('Start state in collision!');
        path = [];
        search_stats = createFailureStats(0, 0, 0);
        return;
    end
    
    % Initialize open/closed sets
    % Open list: priority queue (using containers.Map for MATLAB compatibility)
    % Key: f-score, Value: cell array of states
    open_list = containers.Map('KeyType', 'double', 'ValueType', 'any');
    open_list(start_state.f) = {start_state};
    
    % Closed set: 3D visited array (grid_x, grid_y, theta_bin)
    % Use logical array for memory efficiency
    grid_size_x = double(occupancy_grid.size_x);
    grid_size_y = double(occupancy_grid.size_y);
    visited = false(grid_size_x, grid_size_y, num_theta_bins);
    
    % Parent tracking: store parent state for path reconstruction
    % Key: state index (grid_x + grid_y*grid_size + theta_bin*grid_size^2)
    parent_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    
    % Search statistics
    tic;
    iterations = 0;
    nodes_expanded = 0;
    goal_reached = false;
    final_state = [];
    
    % Main A* loop
    while ~isempty(open_list) && iterations < max_iterations
        iterations = iterations + 1;
        
        % Check timeout
        if toc > timeout_sec
            warning('Planning timeout (%g sec)', timeout_sec);
            break;
        end
        
        % Get state with lowest f-score
        current_state = popLowestF(open_list);
        if isempty(current_state)
            break;
        end
        
        % Check if already visited (may happen due to duplicate insertions)
        if visited(current_state.grid_x, current_state.grid_y, current_state.theta_bin)
            continue;
        end
        
        % Mark as visited
        visited(current_state.grid_x, current_state.grid_y, current_state.theta_bin) = true;
        nodes_expanded = nodes_expanded + 1;
        
        % Check goal condition
        if isGoalReached(current_state, goal_state, goal_tol_xy, goal_tol_theta)
            goal_reached = true;
            final_state = current_state;
            break;
        end
        
        % Expand motion primitives
        for i = 1:length(primitives)
            prim = primitives(i);
            
            % Apply primitive to current state
            [next_x, next_y, next_theta] = gik9dof.computeMotionPrimitive(...
                current_state.x, current_state.y, current_state.theta, ...
                prim.Vx, prim.Wz, prim.dt, params);
            
            % Discretize next state
            next_grid_x = occupancy_grid.worldToGridX(next_x);
            next_grid_y = occupancy_grid.worldToGridY(next_y);
            next_theta_bin = angleToBin(next_theta, num_theta_bins);
            
            % Check bounds
            if next_grid_x < 1 || next_grid_x > grid_size_x || ...
               next_grid_y < 1 || next_grid_y > grid_size_y
                continue;
            end
            
            % Skip if already visited
            if visited(next_grid_x, next_grid_y, next_theta_bin)
                continue;
            end
            
            % Collision check along arc
            if gik9dof.checkArcCollision(current_state.x, current_state.y, current_state.theta, ...
                                         prim.Vx, prim.Wz, prim.dt, occupancy_grid, params)
                continue;
            end
            
            % Compute cost (arc length)
            arc_cost = abs(prim.Vx) * prim.dt;
            next_g = current_state.g + arc_cost;
            
            % Create next state
            next_state = gik9dof.HybridState();
            next_state.x = next_x;
            next_state.y = next_y;
            next_state.theta = next_theta;
            next_state.grid_x = next_grid_x;
            next_state.grid_y = next_grid_y;
            next_state.theta_bin = next_theta_bin;
            next_state.g = next_g;
            next_state.h = computeHeuristic(next_state, goal_state, R_min, use_dubins);
            next_state.f = next_state.g + next_state.h;
            next_state.parent_x = current_state.x;
            next_state.parent_y = current_state.y;
            next_state.parent_theta = current_state.theta;
            
            % Store primitive for path reconstruction
            next_state.Vx = prim.Vx;
            next_state.Wz = prim.Wz;
            next_state.dt = prim.dt;
            
            % Add to open list
            if ~isKey(open_list, next_state.f)
                open_list(next_state.f) = {};
            end
            state_list = open_list(next_state.f);
            state_list{end+1} = next_state;
            open_list(next_state.f) = state_list;
            
            % Store parent relationship
            next_idx = stateToIndex(next_state, grid_size_x, num_theta_bins);
            parent_map(next_idx) = current_state;
        end
    end
    
    planning_time = toc;
    
    % Reconstruct path
    if goal_reached
        path = reconstructPath(final_state, parent_map, grid_size_x, num_theta_bins);
        search_stats = createSuccessStats(iterations, nodes_expanded, ...
            planning_time, final_state.g, length(path));
    else
        path = [];
        search_stats = createFailureStats(iterations, nodes_expanded, planning_time);
    end
end

%% Helper functions

function val = getFieldOrDefault(s, field, default_val)
    if isfield(s, field)
        val = s.(field);
    else
        val = default_val;
    end
end

function bin = angleToBin(theta, num_bins)
    % Normalize theta to [0, 2*pi)
    theta_normalized = mod(theta, 2*pi);
    % Convert to bin index [1, num_bins]
    bin = floor(theta_normalized / (2*pi / num_bins)) + 1;
    bin = max(1, min(num_bins, bin));
end

function h = computeHeuristic(state, goal, R_min, use_dubins)
    if use_dubins
        h = gik9dof.computeDubinsHeuristic(state.x, state.y, state.theta, ...
                                           goal.x, goal.y, goal.theta, R_min);
    else
        h = gik9dof.computeEuclideanHeuristic(state.x, state.y, ...
                                              goal.x, goal.y);
    end
end

function reached = isGoalReached(state, goal, tol_xy, tol_theta)
    dx = state.x - goal.x;
    dy = state.y - goal.y;
    dist = sqrt(dx*dx + dy*dy);
    
    dtheta = abs(angdiff(state.theta, goal.theta));
    
    reached = (dist <= tol_xy) && (dtheta <= tol_theta);
end

function state = popLowestF(open_list)
    % Get state with lowest f-score
    if isempty(open_list)
        state = [];
        return;
    end
    
    % Find minimum f-score
    f_scores = keys(open_list);
    f_min = min([f_scores{:}]);
    
    % Get state list at min f-score
    state_list = open_list(f_min);
    
    % Pop first state
    state = state_list{1};
    
    % Update or remove entry
    if length(state_list) > 1
        open_list(f_min) = state_list(2:end);
    else
        remove(open_list, f_min);
    end
end

function idx = stateToIndex(state, grid_size, num_theta_bins)
    % Convert 3D grid indices to 1D index for parent tracking
    idx = int32(state.grid_x + ...
                (state.grid_y - 1) * grid_size + ...
                (state.theta_bin - 1) * grid_size * grid_size);
end

function path = reconstructPath(final_state, parent_map, grid_size, num_theta_bins)
    % Reconstruct path from goal to start
    path = [];
    current = final_state;
    
    while ~isempty(current)
        % Add waypoint
        waypoint = struct();
        waypoint.x = current.x;
        waypoint.y = current.y;
        waypoint.theta = current.theta;
        waypoint.Vx = current.Vx;
        waypoint.Wz = current.Wz;
        waypoint.dt = current.dt;
        
        path = [waypoint; path]; %#ok<AGROW>
        
        % Get parent
        current_idx = stateToIndex(current, grid_size, num_theta_bins);
        if isKey(parent_map, current_idx)
            current = parent_map(current_idx);
        else
            break;
        end
    end
end

function stats = createSuccessStats(iters, expanded, time, cost, length)
    stats = struct();
    stats.success = true;
    stats.iterations = iters;
    stats.nodes_expanded = expanded;
    stats.planning_time_sec = time;
    stats.path_cost = cost;
    stats.path_length = length;
end

function stats = createFailureStats(iters, expanded, time)
    stats = struct();
    stats.success = false;
    stats.iterations = iters;
    stats.nodes_expanded = expanded;
    stats.planning_time_sec = time;
    stats.path_cost = inf;
    stats.path_length = 0;
end
