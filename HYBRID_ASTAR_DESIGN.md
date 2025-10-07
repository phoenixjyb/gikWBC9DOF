# Hybrid A* Path Planner - C++ Code Generation Design

**Date**: 2025-01-XX  
**Status**: Design Phase  
**Context**: From-scratch implementation for MATLAB Coder C++ code generation

---

## 1. Design Objectives

### Primary Goal
Implement Hybrid A* path planner that:
- ✅ **Code-generable**: Pure value types, no handle classes
- ✅ **Real-time capable**: Bounded memory, predictable execution
- ✅ **Mobile robot optimized**: SE(2) state space with kinematic constraints
- ✅ **Integration-ready**: Outputs trajectories compatible with Pure Pursuit controller

### Constraints from Code Generation
1. **No dynamic allocation**: Fixed-size arrays only
2. **No handle classes**: Value types for all data structures
3. **No objects with pointers**: Struct-based design
4. **Bounded iterations**: Maximum search depth, time limits
5. **Extrinsic functions**: For visualization/debugging only

---

## 2. Algorithm Overview

### Core Hybrid A* Concept
```
State space: SE(2) = (x, y, theta)
- x, y: position in workspace [m]
- theta: heading angle [rad]
- steer: steering angle (optional, for car-like robots)

Search strategy:
- Discretize SE(2) space into 3D grid (x_grid, y_grid, theta_grid)
- Expand using kinematic-feasible motion primitives
- Guide search with non-holonomic heuristic
- Track continuous (x,y,theta) for precise collision checking
```

### Key Differences from Standard A*
| Aspect | Standard A* | Hybrid A* |
|--------|-------------|-----------|
| **State space** | Grid (x,y) | SE(2) (x,y,θ) |
| **Transitions** | 8-connected | Arc motions (Reed-Shepp/Dubins) |
| **Cost** | Euclidean | Arc length + penalties |
| **Heuristic** | Euclidean | Dubins/Reed-Shepp distance |
| **Collision check** | Point-based | Swept volume along arc |
| **Output** | Waypoints | Smooth trajectory |

---

## 3. Data Structures (Code-Generation Ready)

### 3.1 State Representation
```matlab
% Hybrid A* State (value type struct)
HybridState struct:
    x: double        % Continuous x position [m]
    y: double        % Continuous y position [m]
    theta: double    % Continuous heading [rad]
    steer: double    % Steering angle [rad] (optional)
    
    % Discretized indices for graph search
    grid_x: int32    % Grid index for x
    grid_y: int32    % Grid index for y
    grid_theta: int32 % Grid index for theta
    
    % Search metadata
    g_cost: double   % Cost from start
    h_cost: double   % Heuristic to goal
    f_cost: double   % Total cost (g + h)
    parent_idx: int32 % Index of parent state (-1 for start)
    
    % Motion metadata
    arc_length: double % Length of arc to reach this state
    curvature: double  % Curvature of arrival arc
end
```

**Design Rationale**:
- **Hybrid representation**: Continuous (x,y,θ) for collision checking, discrete indices for graph uniqueness
- **Metadata in struct**: Avoids separate cost maps, simplifies indexing
- **Parent tracking**: Enables path reconstruction

### 3.2 Occupancy Grid
```matlab
% Binary occupancy map (value type)
OccupancyGrid struct:
    data: logical array [MAX_GRID_Y x MAX_GRID_X]  % false=free, true=occupied
    resolution: double                             % [m/cell]
    origin_x: double                               % [m] world coord of grid(1,1)
    origin_y: double                               % [m] world coord of grid(1,1)
    size_x: int32                                  % Number of cells in x
    size_y: int32                                  % Number of cells in y
end

MAX_GRID_X = 200;  % Fixed maximum grid size
MAX_GRID_Y = 200;
```

**Design Rationale**:
- **Fixed-size array**: `logical(MAX_GRID_Y, MAX_GRID_X)` for code generation
- **Row-major indexing**: `grid(y,x)` for efficient cache access
- **Origin tracking**: Supports grids not centered at (0,0)

### 3.3 Motion Primitives
```matlab
% Arc motion primitive (constant curvature)
MotionPrimitive struct:
    steer_angle: double  % Steering angle [rad]
    arc_length: double   % Distance traveled [m]
    num_steps: int32     % Discretization steps for collision check
    
    % Pre-computed arc geometry (relative to start state)
    dx: double           % Final x displacement [m]
    dy: double           % Final y displacement [m]
    dtheta: double       % Final heading change [rad]
    cost: double         % Traversal cost (typically arc_length)
end

% Primitive set (fixed-size array)
NUM_PRIMITIVES = 15;  % e.g., 5 steer angles × 3 distances
primitives: MotionPrimitive array [NUM_PRIMITIVES]
```

**Example Primitive Set**:
```
Steer angles: [-30°, -15°, 0°, +15°, +30°]
Arc lengths:  [0.5m, 1.0m, 1.5m]
→ 15 total primitives
```

**Design Rationale**:
- **Pre-computed geometry**: Avoids runtime arc integration
- **Fixed number**: Enables fixed-size arrays
- **Symmetric set**: Supports forward planning

### 3.4 Priority Queue (Open Set)
```matlab
% Fixed-size min-heap for f_cost
PriorityQueue struct:
    states: HybridState array [MAX_QUEUE_SIZE]  % State heap
    heap_size: int32                           % Current number of states
    MAX_SIZE: int32 = 10000                    % Maximum capacity
end

% Operations:
% - push(state): Insert state, heapify up
% - pop(): Remove min f_cost state, heapify down
% - isEmpty(): Check heap_size == 0
```

**Design Rationale**:
- **Heap implementation**: O(log n) insert/remove
- **Fixed capacity**: MAX_QUEUE_SIZE for code generation
- **Overflow handling**: Return failure if queue fills (rare with good heuristic)

### 3.5 Visited Set (Closed Set)
```matlab
% 3D occupancy grid for visited states
VisitedGrid struct:
    visited: logical array [THETA_BINS x MAX_GRID_Y x MAX_GRID_X]
    state_data: HybridState array [THETA_BINS x MAX_GRID_Y x MAX_GRID_X]
end

THETA_BINS = 72;  % 5° resolution (360/5)
```

**Indexing**:
```matlab
idx = (grid_theta-1)*MAX_GRID_Y*MAX_GRID_X + (grid_y-1)*MAX_GRID_X + grid_x
visited(grid_theta, grid_y, grid_x) = true
state_data(grid_theta, grid_y, grid_x) = state
```

**Design Rationale**:
- **3D array**: Fast lookup O(1) for duplicate detection
- **Co-located data**: Store full state with visited flag
- **Fixed theta bins**: Balance between path quality and memory

---

## 4. Algorithm Pseudocode

### 4.1 Main Planning Function
```matlab
function [path, success] = planHybridAStar(start, goal, grid, params)
    % Inputs:
    %   start: HybridState (x,y,theta,steer)
    %   goal: HybridState (x,y,theta,steer)
    %   grid: OccupancyGrid
    %   params: PlannerParams (resolution, max_iter, etc.)
    
    % Initialize
    open_set = PriorityQueue();
    closed_set = VisitedGrid();
    
    % Discretize start state
    start.grid_x = worldToGrid(start.x, grid);
    start.grid_y = worldToGrid(start.y, grid);
    start.grid_theta = angleToGrid(start.theta, THETA_BINS);
    
    % Compute start costs
    start.g_cost = 0;
    start.h_cost = heuristic(start, goal, params);
    start.f_cost = start.g_cost + start.h_cost;
    start.parent_idx = -1;
    
    % Add start to open set
    push(open_set, start);
    
    iter = 0;
    while ~isEmpty(open_set) && iter < params.max_iterations
        iter = iter + 1;
        
        % Pop state with lowest f_cost
        current = pop(open_set);
        
        % Check if goal reached
        if isGoalReached(current, goal, params)
            path = reconstructPath(current, closed_set);
            success = true;
            return;
        end
        
        % Mark as visited
        closed_set.visited(current.grid_theta, current.grid_y, current.grid_x) = true;
        closed_set.state_data(current.grid_theta, current.grid_y, current.grid_x) = current;
        
        % Expand neighbors
        for i = 1:NUM_PRIMITIVES
            neighbor = applyMotionPrimitive(current, primitives(i), grid);
            
            % Skip if invalid (collision or out of bounds)
            if ~isValid(neighbor, grid)
                continue;
            end
            
            % Discretize neighbor
            neighbor.grid_x = worldToGrid(neighbor.x, grid);
            neighbor.grid_y = worldToGrid(neighbor.y, grid);
            neighbor.grid_theta = angleToGrid(neighbor.theta, THETA_BINS);
            
            % Skip if already visited
            if closed_set.visited(neighbor.grid_theta, neighbor.grid_y, neighbor.grid_x)
                continue;
            end
            
            % Compute costs
            neighbor.g_cost = current.g_cost + primitives(i).cost;
            neighbor.h_cost = heuristic(neighbor, goal, params);
            neighbor.f_cost = neighbor.g_cost + neighbor.h_cost;
            neighbor.parent_idx = current.unique_id;  % Store link to parent
            
            % Add to open set
            push(open_set, neighbor);
        end
    end
    
    % No path found
    path = [];
    success = false;
end
```

### 4.2 Motion Primitive Application
```matlab
function neighbor = applyMotionPrimitive(state, primitive, grid)
    % Bicycle model kinematics
    L = 0.5;  % Wheelbase [m]
    dt = 0.1; % Time step [s]
    
    % Initialize neighbor
    neighbor = state;  % Copy parent state
    
    % Integrate arc (constant curvature)
    for step = 1:primitive.num_steps
        % Update heading
        neighbor.theta = neighbor.theta + (primitive.steer_angle / L) * dt;
        
        % Update position
        neighbor.x = neighbor.x + cos(neighbor.theta) * dt;
        neighbor.y = neighbor.y + sin(neighbor.theta) * dt;
        
        % Check collision at each step
        if isCollision(neighbor.x, neighbor.y, grid)
            neighbor.valid = false;
            return;
        end
    end
    
    neighbor.steer = primitive.steer_angle;
    neighbor.arc_length = primitive.arc_length;
    neighbor.valid = true;
end
```

### 4.3 Heuristic Function
```matlab
function h = heuristic(state, goal, params)
    % Option 1: Euclidean distance (admissible, but weak)
    dx = goal.x - state.x;
    dy = goal.y - state.y;
    h_euclidean = sqrt(dx^2 + dy^2);
    
    % Option 2: Dubins distance (non-holonomic aware)
    min_radius = params.min_turning_radius;
    h_dubins = dubinsDistance(state, goal, min_radius);
    
    % Option 3: Hybrid (faster, still admissible)
    h = max(h_euclidean, h_dubins);
    
    % Heuristic weight (>1 for faster, sub-optimal paths)
    h = params.heuristic_weight * h;
end
```

**Heuristic Options**:
- **Euclidean**: Fast, admissible, but ignores non-holonomic constraints
- **Dubins**: Accurate, but expensive to compute
- **Hybrid**: Good balance (use Euclidean until close to goal, then Dubins)

### 4.4 Collision Checking
```matlab
function is_collision = isCollision(x, y, grid)
    % Convert world to grid coordinates
    grid_x = worldToGrid(x, grid);
    grid_y = worldToGrid(y, grid);
    
    % Check bounds
    if grid_x < 1 || grid_x > grid.size_x || grid_y < 1 || grid_y > grid.size_y
        is_collision = true;
        return;
    end
    
    % Check occupancy
    if grid.data(grid_y, grid_x)
        is_collision = true;
        return;
    end
    
    % Optional: Circle-based robot footprint
    robot_radius = 0.3;  % [m]
    radius_cells = ceil(robot_radius / grid.resolution);
    
    for dy = -radius_cells:radius_cells
        for dx = -radius_cells:radius_cells
            if sqrt(dx^2 + dy^2) <= radius_cells
                gx = grid_x + dx;
                gy = grid_y + dy;
                if gx >= 1 && gx <= grid.size_x && gy >= 1 && gy <= grid.size_y
                    if grid.data(gy, gx)
                        is_collision = true;
                        return;
                    end
                end
            end
        end
    end
    
    is_collision = false;
end
```

### 4.5 Path Reconstruction
```matlab
function path = reconstructPath(goal_state, closed_set)
    MAX_PATH_LENGTH = 1000;
    path = HybridState.empty(MAX_PATH_LENGTH, 1);  % Pre-allocate
    
    current = goal_state;
    idx = 1;
    
    while current.parent_idx ~= -1 && idx <= MAX_PATH_LENGTH
        path(idx) = current;
        idx = idx + 1;
        
        % Retrieve parent from closed set
        parent = closed_set.state_data(current.grid_theta, current.grid_y, current.grid_x);
        current = parent;
    end
    
    % Add start state
    path(idx) = current;
    
    % Reverse path (start → goal)
    path = flip(path(1:idx));
end
```

---

## 5. Parameter Configuration

### 5.1 Grid Parameters
```matlab
params.grid_resolution = 0.1;     % [m/cell] - finer = more accurate, slower
params.theta_resolution = 5.0;    % [deg] - THETA_BINS = 360/theta_resolution
params.max_grid_size_x = 200;     % [cells]
params.max_grid_size_y = 200;     % [cells]
```

### 5.2 Vehicle Parameters
```matlab
params.wheelbase = 0.5;           % [m] - for bicycle model
params.min_turning_radius = 1.0;  % [m] - kinematic constraint
params.robot_radius = 0.3;        % [m] - for collision checking
params.safety_margin = 0.1;       % [m] - extra clearance
```

### 5.3 Motion Primitives
```matlab
params.num_steer_angles = 5;      % Discretization of steering
params.max_steer_angle = 30;      % [deg]
params.arc_lengths = [0.5, 1.0, 1.5]; % [m]
```

### 5.4 Search Parameters
```matlab
params.max_iterations = 10000;    % Safety limit
params.heuristic_weight = 1.2;    % 1.0 = optimal, >1.0 = faster/sub-optimal
params.goal_tolerance_xy = 0.2;   % [m]
params.goal_tolerance_theta = 10; % [deg]
```

---

## 6. Memory Footprint Analysis

### Fixed-Size Arrays
```
OccupancyGrid:     200 × 200 × 1 byte          = 40 KB
VisitedGrid:       72 × 200 × 200 × 1 byte     = 2.88 MB (visited flags)
StateData:         72 × 200 × 200 × 80 bytes   = 230 MB (full states)
PriorityQueue:     10000 × 80 bytes            = 800 KB
MotionPrimitives:  15 × 64 bytes               = 960 bytes

TOTAL:             ~234 MB
```

**Optimization Options**:
1. **Reduce theta bins**: 72 → 36 (10° resolution) → 117 MB saved
2. **Sparse state storage**: Store states in 1D array with hash map → 90% memory reduction
3. **Smaller grid**: 200×200 → 150×150 → 44% memory reduction

**Recommended for Orin**:
- Grid: 150×150
- Theta bins: 36
- Queue size: 5000
- **Total: ~60 MB** (acceptable for real-time)

---

## 7. Code Generation Strategy

### 7.1 File Structure
```
matlab/+gik9dof/
    planHybridAStar.m           % Main planner function
    HybridState.m               % State struct definition
    OccupancyGrid.m             % Grid struct definition
    MotionPrimitive.m           % Primitive struct definition
    PriorityQueue.m             % Heap implementation
    
    private/
        applyMotionPrimitive.m  % Kinematic propagation
        computeHeuristic.m      % Heuristic function
        isCollision.m           % Collision checker
        reconstructPath.m       % Path extraction
        worldToGrid.m           % Coordinate transforms
        angleToGrid.m
```

### 7.2 Code Generation Configuration
```matlab
% In RUN_CODEGEN.m or separate script
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.Hardware = coder.hardware('NVIDIA Jetson');

% Define input types
start = coder.typeof(struct('x', 0, 'y', 0, 'theta', 0, 'steer', 0, ...
                             'grid_x', int32(0), 'grid_y', int32(0), ...
                             'grid_theta', int32(0), 'g_cost', 0, ...
                             'h_cost', 0, 'f_cost', 0, 'parent_idx', int32(0), ...
                             'arc_length', 0, 'curvature', 0));
goal = start;

grid_data = coder.typeof(false, [200, 200]);
grid = coder.typeof(struct('data', grid_data, 'resolution', 0, ...
                            'origin_x', 0, 'origin_y', 0, ...
                            'size_x', int32(0), 'size_y', int32(0)));

params = coder.typeof(struct('grid_resolution', 0, 'theta_resolution', 0, ...
                              'max_iterations', int32(0), ...
                              'heuristic_weight', 0, ...
                              'goal_tolerance_xy', 0, ...
                              'goal_tolerance_theta', 0));

% Generate code
codegen -config cfg planHybridAStar -args {start, goal, grid, params}
```

### 7.3 Testing Strategy
```matlab
% 1. Unit tests for components
test_priorityQueue();         % Heap operations
test_collisionChecking();     % Obstacle detection
test_motionPrimitives();      % Kinematic propagation
test_heuristic();             % Distance estimates

% 2. Integration tests
test_simpleGrid();            % No obstacles, straight path
test_corridorNavigation();    % Narrow passages
test_parkingScenario();       % Backing maneuvers
test_uTurn();                 % Tight turns

% 3. Comparison to Navigation Toolbox
validateAgainstMATLAB();      % Compare path quality, length, time
```

---

## 8. Integration with Existing System

### 8.1 Input from Occupancy Map
```matlab
% Current system likely has:
occupancy_map = ...;  % From sensors or static map

% Convert to OccupancyGrid struct
grid.data = (occupancy_map > 0.5);  % Threshold to binary
grid.resolution = occupancy_map.Resolution;
grid.origin_x = occupancy_map.XWorldLimits(1);
grid.origin_y = occupancy_map.YWorldLimits(1);
grid.size_x = size(grid.data, 2);
grid.size_y = size(grid.data, 1);
```

### 8.2 Output to Pure Pursuit
```matlab
% Hybrid A* output: path (array of HybridState)
% Pure Pursuit input: trajectory struct with x, y, theta, curvature

% Convert path to trajectory
trajectory.x = [path.x];
trajectory.y = [path.y];
trajectory.theta = [path.theta];
trajectory.curvature = [path.curvature];
trajectory.length = length(path);

% Densify if needed (reuse from runStagedTrajectory.m)
trajectory = densifyHybridStates(trajectory, params);

% Pass to Pure Pursuit controller
[v, omega] = purePursuitController(trajectory, robot_state, params);
```

### 8.3 ROS2 Integration
```matlab
% In ROS2 node:
% 1. Subscribe to /map (occupancy grid)
% 2. Subscribe to /goal_pose (geometry_msgs/PoseStamped)
% 3. Publish to /planned_path (nav_msgs/Path)

function planPathCallback(goal_msg)
    % Convert ROS map to OccupancyGrid struct
    grid = rosMapToGrid(map_msg);
    
    % Extract start (current pose) and goal
    start = robotPoseToHybridState(current_pose);
    goal = rosPoseToHybridState(goal_msg.pose);
    
    % Plan
    [path, success] = planHybridAStar(start, goal, grid, params);
    
    if success
        % Publish path
        path_msg = hybridPathToRosPath(path);
        publish(path_pub, path_msg);
    else
        % Log failure
        logError('Hybrid A* planning failed');
    end
end
```

---

## 9. Performance Targets

### Computational Budget
| Metric | Target | Stretch Goal |
|--------|--------|--------------|
| **Planning time** | < 500 ms | < 200 ms |
| **Memory** | < 100 MB | < 50 MB |
| **Path quality** | Within 10% of optimal | Within 5% |
| **Success rate** | > 95% (solvable scenarios) | > 99% |

### Comparison to Navigation Toolbox
- **Speed**: Expect 2-5× faster (compiled C++ vs MATLAB)
- **Memory**: Expect 50% less (fixed arrays vs dynamic objects)
- **Quality**: Should be comparable (same algorithm principles)

---

## 10. Risk Mitigation

### Risk 1: Priority Queue Overflow
**Scenario**: Open set exceeds MAX_QUEUE_SIZE  
**Mitigation**:
- Monitor queue size during search
- If approaching limit, increase heuristic weight (greedier search)
- Return "partial path" if queue fills

### Risk 2: Excessive Planning Time
**Scenario**: Complex environment, no solution  
**Mitigation**:
- Implement timeout (max_iterations)
- Use anytime planning (return best path found so far)
- Fall back to simpler planner (grid A* without theta)

### Risk 3: Poor Heuristic Performance
**Scenario**: Heuristic underestimates, explores too many states  
**Mitigation**:
- Tune heuristic weight (1.0 → 1.5)
- Profile different heuristic functions (Euclidean vs Dubins)
- Add domain knowledge (prefer paths away from obstacles)

### Risk 4: Code Generation Errors
**Scenario**: Indexing issues, type mismatches  
**Mitigation**:
- Extensive unit testing in MATLAB first
- Use `coder.typeof` for all struct fields
- Test with `codegen -report` to catch issues early

---

## 11. Development Phases

### Phase 1: Core A* (1 week)
- Implement grid-based A* (x,y only, no theta)
- Priority queue, visited set
- Simple Euclidean heuristic
- Test with 2D obstacle maps
- **Deliverable**: Working 2D planner, code-generable

### Phase 2: Hybrid A* Skeleton (1 week)
- Add theta dimension to state
- Implement motion primitives (constant steer angles)
- Extend collision checking to robot footprint
- **Deliverable**: 3D state space planner, basic kinematic feasibility

### Phase 3: Refinement (1 week)
- Implement Dubins heuristic
- Optimize memory layout
- Tune parameters (grid resolution, theta bins)
- **Deliverable**: Production-ready planner

### Phase 4: Integration (3 days)
- Connect to occupancy map input
- Interface with Pure Pursuit controller
- ROS2 node wrapper
- **Deliverable**: End-to-end pipeline

### Phase 5: Validation (3 days)
- Compare to Navigation Toolbox
- Benchmark on Orin ARM64
- Stress testing (large maps, complex environments)
- **Deliverable**: Performance report, validation test suite

**Total estimated time**: 2-3 weeks (full-time)

---

## 12. Success Criteria

### Must-Have
- ✅ Code generates to C++ without errors
- ✅ Plans kinematically-feasible paths (no skidding)
- ✅ Avoids obstacles with safety margin
- ✅ Runs on Orin ARM64 in real-time (< 500ms)
- ✅ Integrates with Pure Pursuit controller

### Nice-to-Have
- Path smoothing (post-processing)
- Reverse driving support (Reed-Shepp curves)
- Dynamic obstacle avoidance (replanning)
- Multi-resolution planning (coarse-to-fine)

### Validation Metrics
- **Unit tests**: 100% pass rate
- **Integration tests**: 95% success rate on benchmark scenarios
- **Comparison to Navigation Toolbox**: Within 10% path length, 2× faster
- **Memory**: < 100 MB on ARM64

---

## 13. References

### Academic Papers
1. **Hybrid A* Original Paper**: Dolgov et al., "Practical Search Techniques in Path Planning for Autonomous Driving" (2008)
2. **Motion Primitives**: Pivtoraiko & Kelly, "Generating Near Minimal Spanning Control Sets for Constrained Motion Planning in Discrete State Spaces" (2005)

### MATLAB Documentation
- Navigation Toolbox (for comparison): `plannerHybridAStar`
- MATLAB Coder: Fixed-size arrays, code generation best practices

### Existing Codebase
- `runStagedTrajectory.m` (lines 347-599): Parameter reference, path densification
- Pure Pursuit controller: Output interface requirements

---

## 14. Next Steps

**Immediate** (This session):
1. Create `matlab/+gik9dof/planHybridAStar_gridOnly.m` (2D A* first)
2. Define `HybridState` struct
3. Implement priority queue (min-heap)
4. Write simple test case (empty grid, straight line)

**Short-term** (Next session):
1. Extend to SE(2) state space
2. Add motion primitives
3. Implement collision checking

**Medium-term** (This week):
1. Optimize heuristic
2. Code generation testing
3. Integration with occupancy map

---

**Document Status**: DRAFT - Design phase in progress  
**Author**: GitHub Copilot + User  
**Last Updated**: 2025-01-XX
