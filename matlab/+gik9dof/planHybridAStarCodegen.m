function [path, search_stats] = planHybridAStarCodegen(start_state, goal_state, occupancy_grid, options)
%PLANHYBRIDASTAR_CODEGEN SE(2) Hybrid A* path planning (MATLAB Coder compatible)
%   
%   Code-generation compatible version with fixed-size arrays
%   
%   Inputs:
%       start_state: HybridState struct (x, y, theta)
%       goal_state: HybridState struct (x, y, theta)
%       occupancy_grid: OccupancyGrid2D object (must be already inflated)
%       options: struct with fields (all optional):
%           max_iterations: Maximum search iterations (default: 10000)
%           goal_tolerance_xy: XY goal tolerance in meters (default: 0.2)
%           goal_tolerance_theta: Heading tolerance in radians (default: 0.3)
%           use_dubins_heuristic: true=Dubins, false=Euclidean (default: true)
%           timeout_sec: Maximum planning time in seconds (default: 5.0)
%           max_states: Maximum states to explore (default: 50000)
%           max_path_length: Maximum path waypoints (default: 500)
%
%   Outputs:
%       path: Struct array with fields (x, y, theta, Vx, Wz, dt)
%           Fixed-size array [max_path_length × 1]
%           Actual length in search_stats.path_length
%       search_stats: Struct with fields:
%           success: true if path found
%           iterations: Number of iterations executed
%           nodes_expanded: Number of states expanded
%           planning_time_sec: Total planning time
%           path_cost: Total path cost (g-value at goal)
%           path_length: Number of waypoints in path

%#codegen

    % Parse options with defaults
    if nargin < 4
        options = struct();
    end
    
    % Fixed-size limits for code generation
    MAX_STATES = getFieldOrDefaultInt(options, 'max_states', 50000);
    MAX_PATH_LENGTH = getFieldOrDefaultInt(options, 'max_path_length', 500);
    
    max_iterations = getFieldOrDefaultInt(options, 'max_iterations', 10000);
    goal_tol_xy = getFieldOrDefaultDouble(options, 'goal_tolerance_xy', 0.2);
    goal_tol_theta = getFieldOrDefaultDouble(options, 'goal_tolerance_theta', 0.3);
    use_dubins = getFieldOrDefaultBool(options, 'use_dubins_heuristic', true);
    timeout_sec = getFieldOrDefaultDouble(options, 'timeout_sec', 5.0);
    
    % Get chassis parameters
    params = gik9dof.getChassisParams();
    R_min = params.min_turning_radius;
    
    % Generate motion primitives (returns fixed-size array)
    primitives_all = gik9dof.generateMotionPrimitives(params);
    
    % Filter valid primitives (dt > 0)
    % For codegen: Keep fixed-size array, track valid count
    num_primitives = int32(0);
    for i = 1:length(primitives_all)
        if primitives_all(i).dt > 0
            num_primitives = num_primitives + 1;
        end
    end
    primitives = primitives_all; % Use full array
    
    % Angular discretization (matches design: 16 bins)
    num_theta_bins = int32(16);
    
    % Grid size
    grid_size_x = double(occupancy_grid.size_x);
    grid_size_y = double(occupancy_grid.size_y);
    
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
    
    % Check start validity
    if gik9dof.checkFootprintCollision(start_state.x, start_state.y, start_state.theta, occupancy_grid, params)
        warning('Start state in collision!');
        path = createEmptyPath(MAX_PATH_LENGTH);
        search_stats = createFailureStats(0, 0, 0);
        return;
    end
    
    % Initialize fixed-size data structures
    % State list: Pre-allocated array of states
    state_list = repmat(gik9dof.HybridState(), MAX_STATES, 1);
    state_count = int32(0);
    
    % Add start state
    state_count = state_count + 1;
    state_list(state_count) = start_state;
    start_idx = state_count;
    
    % Open list: Binary heap (min-heap by f-score)
    % heap_indices: Indices into state_list
    % heap_size: Current heap size
    heap_indices = zeros(MAX_STATES, 1, 'int32');
    heap_size = int32(0);
    
    % Insert start state into heap
    heap_size = heap_size + 1;
    heap_indices(heap_size) = start_idx;
    
    % Closed set: 3D visited array (grid_x, grid_y, theta_bin)
    visited = false(int32(grid_size_x), int32(grid_size_y), num_theta_bins);
    
    % Parent map: Fixed-size array indexed by state index
    parent_indices = zeros(MAX_STATES, 1, 'int32');
    
    % Search loop
    tic;
    iterations = int32(0);
    nodes_expanded = int32(0);
    goal_reached = false;
    final_state_idx = int32(0);
    
    while heap_size > 0 && iterations < max_iterations && state_count < MAX_STATES
        iterations = iterations + 1;
        
        % Check timeout
        if toc > timeout_sec
            warning('Planning timeout (%g sec)', timeout_sec);
            break;
        end
        
        % Pop state with lowest f-score from heap
        [heap_indices, current_idx] = heapPop(heap_indices, heap_size, state_list);
        heap_size = heap_size - 1;
        
        if current_idx <= 0
            break;
        end
        
        current_state = state_list(current_idx);
        
        % Check if already visited
        if visited(current_state.grid_x, current_state.grid_y, current_state.theta_bin)
            continue;
        end
        
        % Mark as visited
        visited(current_state.grid_x, current_state.grid_y, current_state.theta_bin) = true;
        nodes_expanded = nodes_expanded + 1;
        
        % Check goal condition
        if isGoalReached(current_state, goal_state, goal_tol_xy, goal_tol_theta)
            goal_reached = true;
            final_state_idx = current_idx;
            break;
        end
        
        % Expand motion primitives
        for i = 1:length(primitives)
            prim = primitives(i);
            
            % Skip invalid primitives (dt=0 for unused slots)
            if prim.dt <= 0
                continue;
            end
            
            % Check state limit
            if state_count >= MAX_STATES - 1
                break;
            end
            
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
            next_state.Vx = prim.Vx;
            next_state.Wz = prim.Wz;
            next_state.dt = prim.dt;
            
            % Add to state list
            state_count = state_count + 1;
            state_list(state_count) = next_state;
            next_idx = state_count;
            
            % Store parent
            parent_indices(next_idx) = current_idx;
            
            % Add to heap
            heap_size = heap_size + 1;
            heap_indices(heap_size) = next_idx;
            heap_indices = heapBubbleUp(heap_indices, heap_size, state_list);
        end
    end
    
    planning_time = toc;
    
    % Reconstruct path
    if goal_reached
        path = reconstructPathFixed(final_state_idx, parent_indices, state_list, MAX_PATH_LENGTH);
        % Count actual path length
        actual_length = int32(0);
        for i = 1:MAX_PATH_LENGTH
            if path(i).dt > 0 || i == 1
                actual_length = actual_length + 1;
            else
                break;
            end
        end
        search_stats = createSuccessStats(iterations, nodes_expanded, ...
            planning_time, state_list(final_state_idx).g, actual_length);
    else
        path = createEmptyPath(MAX_PATH_LENGTH);
        search_stats = createFailureStats(iterations, nodes_expanded, planning_time);
    end
end

%% Helper functions

function val = getFieldOrDefaultInt(s, field, default_val)
    if isfield(s, field)
        val = int32(s.(field));
    else
        val = int32(default_val);
    end
end

function val = getFieldOrDefaultDouble(s, field, default_val)
    if isfield(s, field)
        val = double(s.(field));
    else
        val = double(default_val);
    end
end

function val = getFieldOrDefaultBool(s, field, default_val)
    if isfield(s, field)
        val = logical(s.(field));
    else
        val = logical(default_val);
    end
end

function bin = angleToBin(theta, num_bins)
    % Normalize theta to [0, 2*pi)
    theta_normalized = mod(theta, 2*pi);
    % Convert to bin index [1, num_bins]
    bin = int32(floor(theta_normalized / (2*pi / double(num_bins)))) + 1;
    bin = max(int32(1), min(num_bins, bin));
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

%% Binary Heap Operations (Min-heap by f-score)

function [heap_indices, idx] = heapPop(heap_indices, heap_size, state_list)
    %HEAPPOP Pop minimum f-score state from heap
    if heap_size <= 0
        idx = int32(0);
        return;
    end
    
    % Return root (minimum)
    idx = heap_indices(1);
    
    % Move last element to root and bubble down
    heap_indices(1) = heap_indices(heap_size);
    heap_indices = heapBubbleDown(heap_indices, heap_size - 1, state_list);
end

function heap_indices = heapBubbleUp(heap_indices, heap_size, state_list)
    %HEAPBUBBLEUP Bubble up last element to maintain heap property
    idx = heap_size;
    
    while idx > 1
        parent_idx = int32(floor(double(idx) / 2));
        
        % Compare f-scores
        if state_list(heap_indices(idx)).f < state_list(heap_indices(parent_idx)).f
            % Swap
            temp = heap_indices(idx);
            heap_indices(idx) = heap_indices(parent_idx);
            heap_indices(parent_idx) = temp;
            idx = parent_idx;
        else
            break;
        end
    end
end

function heap_indices = heapBubbleDown(heap_indices, heap_size, state_list)
    %HEAPBUBBLEDOWN Bubble down root to maintain heap property
    idx = int32(1);
    
    while true
        left_idx = 2 * idx;
        right_idx = 2 * idx + 1;
        smallest = idx;
        
        % Find smallest among parent, left, right
        if left_idx <= heap_size && ...
           state_list(heap_indices(left_idx)).f < state_list(heap_indices(smallest)).f
            smallest = left_idx;
        end
        
        if right_idx <= heap_size && ...
           state_list(heap_indices(right_idx)).f < state_list(heap_indices(smallest)).f
            smallest = right_idx;
        end
        
        if smallest ~= idx
            % Swap
            temp = heap_indices(idx);
            heap_indices(idx) = heap_indices(smallest);
            heap_indices(smallest) = temp;
            idx = smallest;
        else
            break;
        end
    end
end

%% Path Reconstruction

function path = reconstructPathFixed(final_idx, parent_indices, state_list, max_length)
    %RECONSTRUCTPATHFIXED Reconstruct path with fixed-size output
    
    % Create empty path
    path = createEmptyPath(max_length);
    
    % Trace back from goal to start
    temp_indices = zeros(max_length, 1, 'int32');
    path_count = int32(0);
    current_idx = final_idx;
    
    while current_idx > 0 && path_count < max_length
        path_count = path_count + 1;
        temp_indices(path_count) = current_idx;
        current_idx = parent_indices(current_idx);
    end
    
    % Reverse to get start-to-goal order
    for i = 1:path_count
        idx = temp_indices(path_count - i + 1);
        state = state_list(idx);
        
        path(i).x = state.x;
        path(i).y = state.y;
        path(i).theta = state.theta;
        path(i).Vx = state.Vx;
        path(i).Wz = state.Wz;
        path(i).dt = state.dt;
    end
end

function path = createEmptyPath(max_length)
    %CREATEEMPTYPATH Create empty fixed-size path array
    waypoint = struct('x', 0.0, 'y', 0.0, 'theta', 0.0, ...
                      'Vx', 0.0, 'Wz', 0.0, 'dt', 0.0);
    path = repmat(waypoint, max_length, 1);
end

function stats = createSuccessStats(iters, expanded, time, cost, length)
    stats = struct();
    stats.success = true;
    stats.iterations = double(iters);
    stats.nodes_expanded = double(expanded);
    stats.planning_time_sec = time;
    stats.path_cost = cost;
    stats.path_length = double(length);
end

function stats = createFailureStats(iters, expanded, time)
    stats = struct();
    stats.success = false;
    stats.iterations = double(iters);
    stats.nodes_expanded = double(expanded);
    stats.planning_time_sec = time;
    stats.path_cost = inf;
    stats.path_length = 0;
end
