# Hybrid A* - Perception & Odometry Integration Design

**Date**: October 7, 2025  
**Status**: Design Phase  
**Context**: Integration with real-world sensor data and localization

---

## 1. Updated Grid Parameters

### Grid Resolution: **10cm (0.1m)**
```matlab
params.grid_resolution = 0.1;  % [m/cell] - HIGH RESOLUTION
```

**Rationale**:
- **Safety**: 10cm resolution captures narrow gaps, doorways, tight spaces
- **Robot footprint**: Typical mobile manipulator ~60cm diameter → 6 cells
- **Obstacle detail**: Better representation of walls, furniture, dynamic objects
- **Planning accuracy**: Smoother paths, better clearance estimation

**Implications**:
- **Memory increase**: 200×200 grid @ 10cm = 20m×20m coverage (vs 100m×100m @ 50cm)
- **Compute increase**: More cells to search, but better heuristic guidance
- **Map size trade-off**: Need to balance coverage vs resolution

### Recommended Grid Configuration
```matlab
% Option 1: Large coverage (30m × 30m)
MAX_GRID_X = 300;
MAX_GRID_Y = 300;
resolution = 0.1;  % 10cm
% Memory: 300×300×72 (theta) × 1 byte = 6.48 MB (visited grid)

% Option 2: Medium coverage (20m × 20m) - RECOMMENDED
MAX_GRID_X = 200;
MAX_GRID_Y = 200;
resolution = 0.1;  % 10cm
% Memory: 200×200×72 × 1 byte = 2.88 MB (visited grid)

% Option 3: Tight spaces (10m × 10m)
MAX_GRID_X = 100;
MAX_GRID_Y = 100;
resolution = 0.1;  % 10cm
% Memory: 100×100×72 × 1 byte = 720 KB (visited grid)
```

**Recommended**: Option 2 (200×200 @ 10cm) for **20m×20m** coverage
- Sufficient for typical indoor/warehouse navigation
- Manageable memory footprint
- Can replan frequently with rolling window

---

## 2. Perception Integration

### 2.1 Input Sources

**Lidar** (Primary):
- Point cloud → occupancy grid conversion
- Range: ~10-30m typical
- Update rate: 10-20 Hz
- Coordinate frame: `base_link` or `lidar_link`

**Camera** (Optional):
- Semantic segmentation → obstacle detection
- Range: ~5-15m effective
- Update rate: 5-30 Hz
- Coordinate frame: `camera_link`

**Static Map** (Optional):
- Pre-built map of known environment
- Floor plan, walls, fixed obstacles
- Coordinate frame: `map` or `odom`

### 2.2 Occupancy Map Fusion Pipeline

```
┌─────────────┐
│ Lidar Data  │ → Point Cloud
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Transform   │ → Transform to map frame using TF
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Ray Casting │ → Mark free space along rays
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Hit Points  │ → Mark obstacles at endpoints
└──────┬──────┘
       │
       ▼
┌─────────────┐     ┌─────────────┐
│ Camera Data │ ──→ │ Fusion      │ → Combined occupancy grid
└─────────────┘     │ (Bayesian)  │
                    └──────┬──────┘
                           │
                           ▼
                    ┌─────────────┐
                    │ Inflation   │ → Safety margin around obstacles
                    └──────┬──────┘
                           │
                           ▼
                    ┌─────────────┐
                    │ Hybrid A*   │ → Path planning
                    └─────────────┘
```

### 2.3 Occupancy Grid Update (Code-Generation Ready)

```matlab
function grid = updateOccupancyGrid(grid, lidar_points, robot_pose, params)
    %UPDATEOCCUPANCYGRID Fuse lidar data into occupancy grid
    %   Inputs:
    %       grid: OccupancyGrid2D (current map)
    %       lidar_points: [N × 2] array (x, y in robot frame)
    %       robot_pose: [x, y, theta] current robot pose in map frame
    %       params: Struct with decay_rate, hit_prob, miss_prob
    %
    %   Output:
    %       grid: Updated OccupancyGrid2D
    
    % Parameters
    MAX_RANGE = params.lidar_max_range;  % e.g., 10m
    DECAY_RATE = params.map_decay_rate;   % e.g., 0.95 (5% decay per update)
    HIT_PROB = params.obstacle_hit_prob;  % e.g., 0.7
    MISS_PROB = params.free_space_prob;   % e.g., 0.4
    
    % Step 1: Decay existing occupancy (forget old obstacles)
    grid.data = grid.data * DECAY_RATE;
    
    % Step 2: Transform lidar points to map frame
    cos_theta = cos(robot_pose(3));
    sin_theta = sin(robot_pose(3));
    
    for i = 1:size(lidar_points, 1)
        % Point in robot frame
        px_robot = lidar_points(i, 1);
        py_robot = lidar_points(i, 2);
        
        % Skip points beyond max range
        range = sqrt(px_robot^2 + py_robot^2);
        if range > MAX_RANGE
            continue;
        end
        
        % Transform to map frame
        px_map = robot_pose(1) + cos_theta * px_robot - sin_theta * py_robot;
        py_map = robot_pose(2) + sin_theta * px_robot + cos_theta * py_robot;
        
        % Convert to grid coordinates
        gx_hit = grid.worldToGridX(px_map);
        gy_hit = grid.worldToGridY(py_map);
        
        % Ray casting: Mark free space along ray
        [free_cells_x, free_cells_y] = bresenham(robot_pose(1), robot_pose(2), px_map, py_map, grid);
        
        for j = 1:length(free_cells_x)
            gx = free_cells_x(j);
            gy = free_cells_y(j);
            if grid.isInBounds(gx, gy)
                % Decrease occupancy (mark as free)
                grid.data(gy, gx) = max(0, grid.data(gy, gx) - MISS_PROB);
            end
        end
        
        % Mark hit point as occupied
        if grid.isInBounds(gx_hit, gy_hit)
            grid.data(gy_hit, gx_hit) = min(1, grid.data(gy_hit, gx_hit) + HIT_PROB);
        end
    end
    
    % Step 3: Threshold to binary (for planning)
    grid.data = (grid.data > 0.5);  % Threshold at 50%
end


function [cells_x, cells_y] = bresenham(x0, y0, x1, y1, grid)
    %BRESENHAM Ray casting using Bresenham's line algorithm
    %   Returns grid cells along ray from (x0,y0) to (x1,y1)
    
    MAX_CELLS = 200;
    cells_x = zeros(MAX_CELLS, 1, 'int32');
    cells_y = zeros(MAX_CELLS, 1, 'int32');
    
    % Convert to grid coordinates
    gx0 = grid.worldToGridX(x0);
    gy0 = grid.worldToGridY(y0);
    gx1 = grid.worldToGridX(x1);
    gy1 = grid.worldToGridY(y1);
    
    dx = abs(gx1 - gx0);
    dy = abs(gy1 - gy0);
    sx = sign(gx1 - gx0);
    sy = sign(gy1 - gy0);
    err = dx - dy;
    
    gx = gx0;
    gy = gy0;
    idx = 1;
    
    while idx <= MAX_CELLS
        cells_x(idx) = gx;
        cells_y(idx) = gy;
        idx = idx + 1;
        
        if gx == gx1 && gy == gy1
            break;
        end
        
        e2 = 2 * err;
        if e2 > -dy
            err = err - dy;
            gx = gx + sx;
        end
        if e2 < dx
            err = err + dx;
            gy = gy + sy;
        end
    end
    
    cells_x = cells_x(1:idx-1);
    cells_y = cells_y(1:idx-1);
end
```

### 2.4 Obstacle Inflation (Safety Margin)

```matlab
function grid = inflateObstacles(grid, robot_radius_m)
    %INFLATEOBSTACLES Expand obstacles by robot radius for safety
    %   Inputs:
    %       grid: OccupancyGrid2D
    %       robot_radius_m: Robot radius [m] (e.g., 0.35m for 70cm robot)
    %
    %   Output:
    %       grid: Inflated OccupancyGrid2D
    
    % Convert radius to grid cells
    radius_cells = ceil(robot_radius_m / grid.resolution);
    
    % Create circular structuring element
    [X, Y] = meshgrid(-radius_cells:radius_cells, -radius_cells:radius_cells);
    SE = (X.^2 + Y.^2) <= radius_cells^2;
    
    % Dilate (morphological operation)
    grid.data = imdilate(grid.data, SE);
end
```

---

## 3. Odometry Integration

### 3.1 Robot Pose as Start State

```matlab
function start_state = getRobotPoseAsStartState(odom_msg, grid, params)
    %GETROBOTPOSEASSTARTSTATE Convert odometry to Hybrid A* start state
    %   Inputs:
    %       odom_msg: Odometry message (x, y, theta from localization)
    %       grid: OccupancyGrid2D (for coordinate conversion)
    %       params: Planner parameters
    %
    %   Output:
    %       start_state: HybridState struct
    
    % Extract pose from odometry
    x = odom_msg.pose.pose.position.x;
    y = odom_msg.pose.pose.position.y;
    
    % Extract orientation (quaternion → yaw)
    quat = odom_msg.pose.pose.orientation;
    theta = atan2(2*(quat.w*quat.z + quat.x*quat.y), ...
                  1 - 2*(quat.y^2 + quat.z^2));
    
    % Create start state
    start_state.x = x;
    start_state.y = y;
    start_state.theta = theta;
    start_state.steer = 0;  % Assume zero initial steering
    
    % Discretize to grid
    start_state.grid_x = grid.worldToGridX(x);
    start_state.grid_y = grid.worldToGridY(y);
    start_state.grid_theta = angleToGrid(theta, params.theta_bins);
    
    % Initialize costs
    start_state.g_cost = 0;
    start_state.h_cost = 0;
    start_state.f_cost = 0;
    start_state.parent_idx = -1;
end
```

### 3.2 Pose Uncertainty Handling

**Option 1: Conservative (Inflate obstacles more)**
```matlab
% If odometry uncertainty is ±σ, inflate by σ
robot_radius_with_uncertainty = robot_radius + 3*sigma_pose;  % 3σ rule
grid = inflateObstacles(grid, robot_radius_with_uncertainty);
```

**Option 2: Probabilistic (Weight heuristic)**
```matlab
% Add uncertainty cost to heuristic
h_cost = euclidean_distance + uncertainty_penalty;
uncertainty_penalty = params.uncertainty_weight * trace(covariance_matrix);
```

**Option 3: Replanning (Frequent updates)**
```matlab
% Replan every 0.5-1.0 seconds with updated odometry
if (current_time - last_plan_time) > params.replan_interval
    [path, success] = planHybridAStar(current_odom, goal, grid, params);
end
```

---

## 4. ROS2 Integration Architecture

### 4.1 Node Structure

```
┌─────────────────────────────────────────────────────────────┐
│                   gik9dof_planner_node                      │
├─────────────────────────────────────────────────────────────┤
│  Subscriptions:                                             │
│    /odom              → nav_msgs/Odometry                   │
│    /scan              → sensor_msgs/LaserScan               │
│    /camera/obstacles  → sensor_msgs/PointCloud2 (optional)  │
│    /goal_pose         → geometry_msgs/PoseStamped           │
│                                                              │
│  Publications:                                              │
│    /planned_path      → nav_msgs/Path                       │
│    /local_costmap     → nav_msgs/OccupancyGrid              │
│    /planner_status    → std_msgs/String                     │
│                                                              │
│  Services:                                                  │
│    /replan            → std_srvs/Trigger                    │
│    /set_goal          → geometry_msgs/PoseStamped           │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Message Flow

```
Timer (10 Hz)
  │
  ├─→ Read latest /odom
  ├─→ Read latest /scan
  │
  ├─→ updateOccupancyGrid(scan, odom)
  ├─→ inflateObstacles(grid, robot_radius)
  │
  ├─→ IF new goal received:
  │     │
  │     ├─→ planHybridAStar(odom, goal, grid)
  │     ├─→ Publish /planned_path
  │     └─→ Publish /planner_status
  │
  └─→ Publish /local_costmap (visualization)
```

### 4.3 Code Generation Wrapper

```matlab
function [path_x, path_y, path_theta, success] = hybridAStarROS2Wrapper(...
        odom_x, odom_y, odom_theta, ...  % Current robot pose
        goal_x, goal_y, goal_theta, ...   % Goal pose
        lidar_points, ...                 % [N×2] lidar scan
        grid_data_in, ...                 % Previous occupancy grid
        params)                           % Parameters struct
    
    %HYBRIDASTAR_ROS2WRAPPER Code-generable wrapper for ROS2 integration
    %   All inputs are value types (no objects)
    
    % Step 1: Create occupancy grid
    grid = createOccupancyGrid(grid_data_in, params);
    
    % Step 2: Update grid with lidar data
    robot_pose = [odom_x, odom_y, odom_theta];
    grid = updateOccupancyGrid(grid, lidar_points, robot_pose, params);
    
    % Step 3: Inflate obstacles
    grid = inflateObstacles(grid, params.robot_radius);
    
    % Step 4: Create start and goal states
    start_state = createHybridState(odom_x, odom_y, odom_theta, grid, params);
    goal_state = createHybridState(goal_x, goal_y, goal_theta, grid, params);
    
    % Step 5: Plan
    [path, success] = planHybridAStar(start_state, goal_state, grid, params);
    
    % Step 6: Extract path arrays (ROS2 compatible)
    if success
        path_x = [path.x];
        path_y = [path.y];
        path_theta = [path.theta];
    else
        path_x = [];
        path_y = [];
        path_theta = [];
    end
end
```

---

## 5. Updated Parameter Configuration

### 5.1 Grid Parameters (10cm resolution)
```matlab
params.grid_resolution = 0.1;     % [m/cell] - 10cm HIGH PRECISION
params.theta_resolution = 5.0;    % [deg] - 72 bins for theta
params.max_grid_size_x = 200;     % [cells] → 20m coverage
params.max_grid_size_y = 200;     % [cells] → 20m coverage
```

### 5.2 Perception Parameters
```matlab
% Lidar processing
params.lidar_max_range = 10.0;        % [m] - ignore beyond this
params.lidar_min_range = 0.1;         % [m] - ignore too close
params.map_decay_rate = 0.95;         % Forget 5% per update
params.obstacle_hit_prob = 0.7;       % Increase occupancy on hit
params.free_space_prob = 0.3;         % Decrease on ray trace

% Map fusion
params.occupancy_threshold = 0.5;     % Binary threshold
params.inflation_radius = 0.35;       % [m] - robot radius + margin
```

### 5.3 Odometry Parameters
```matlab
% Pose uncertainty
params.pose_sigma_xy = 0.05;          % [m] - ±5cm position error
params.pose_sigma_theta = 0.087;      % [rad] - ±5° heading error
params.uncertainty_weight = 0.1;      % Heuristic penalty weight

% Replanning
params.replan_interval = 1.0;         % [s] - replan frequency
params.replan_threshold = 0.5;        % [m] - deviate by this → replan
```

### 5.4 Vehicle Parameters (Unchanged)
```matlab
params.wheelbase = 0.5;               % [m] - for bicycle model
params.min_turning_radius = 1.0;      % [m] - kinematic constraint
params.robot_radius = 0.3;            % [m] - base footprint
params.safety_margin = 0.05;          % [m] - extra clearance (5cm)
```

---

## 6. Memory Footprint (Updated for 10cm)

### With 10cm Resolution
```
OccupancyGrid:     200 × 200 × 1 byte          = 40 KB (binary)
VisitedGrid:       72 × 200 × 200 × 1 byte     = 2.88 MB (visited flags)
StateData:         72 × 200 × 200 × 80 bytes   = 230 MB (if storing full states)
PriorityQueue:     10000 × 80 bytes            = 800 KB
MotionPrimitives:  15 × 64 bytes               = 960 bytes
Lidar buffer:      1000 points × 8 bytes       = 8 KB

TOTAL (worst case): ~234 MB
```

### Optimization: Sparse State Storage
Instead of storing full states in 3D grid, use hash map or parent index only:
```
StateData:         72 × 200 × 200 × 4 bytes (parent idx only) = 11.5 MB
TOTAL (optimized):  ~15 MB ✅
```

**Recommended for Orin**: Optimized version with parent indices only

---

## 7. Testing Strategy with Real Data

### 7.1 Simulation Testing (Gazebo)
```bash
# Launch robot in Gazebo with lidar
ros2 launch gik9dof_simulation warehouse.launch.py

# Record rosbag
ros2 bag record /odom /scan /goal_pose

# Replay for testing
ros2 bag play test_scenario_1.bag
```

### 7.2 Unit Tests
```matlab
% Test 1: Occupancy grid update
test_updateOccupancyGrid();  % Verify ray casting, fusion

% Test 2: Odometry transformation
test_odomToStartState();     % Verify quaternion → yaw conversion

% Test 3: Map inflation
test_inflateObstacles();     % Verify safety margin

% Test 4: End-to-end
test_fullPipeline();         % Odom + Scan → Path
```

### 7.3 Real Robot Testing
```
Phase 1: Static map + simulated odom
Phase 2: Real odom + static map
Phase 3: Real odom + real lidar
Phase 4: Dynamic obstacles + replanning
```

---

## 8. Performance Targets (Updated)

| Metric | Target | Notes |
|--------|--------|-------|
| **Grid update rate** | 10 Hz | Process lidar at sensor rate |
| **Planning time** | < 500 ms | First plan from scratch |
| **Replanning time** | < 100 ms | Warm start with previous path |
| **Memory** | < 20 MB | Optimized sparse storage |
| **Map coverage** | 20m × 20m | @ 10cm resolution |
| **Obstacle detection** | > 95% | Correct classification |
| **Path safety** | 0 collisions | With 5cm margin |

---

## 9. Implementation Checklist

**Perception Integration**:
- [ ] Create `updateOccupancyGrid.m` with ray casting
- [ ] Implement Bresenham line algorithm (code-generable)
- [ ] Add `inflateObstacles.m` with morphological dilation
- [ ] Test with synthetic lidar data

**Odometry Integration**:
- [ ] Create `getRobotPoseAsStartState.m`
- [ ] Implement quaternion → yaw conversion
- [ ] Add uncertainty handling (inflate or weight)
- [ ] Test with real odometry logs

**ROS2 Wrapper**:
- [ ] Create `hybridAStarROS2Wrapper.m` (value types only)
- [ ] Generate C++ code with MATLAB Coder
- [ ] Integrate into `gik9dof_planner_node.cpp`
- [ ] Test subscriber/publisher pipeline

**Validation**:
- [ ] Unit tests for each component
- [ ] Integration test with rosbag replay
- [ ] Real robot test with static map
- [ ] Real robot test with dynamic lidar

---

## 10. Next Steps

**Immediate** (Task 3):
1. Update `OccupancyGrid2D` default resolution to 0.1m
2. Create `updateOccupancyGrid.m` function
3. Create `inflateObstacles.m` function
4. Test with synthetic lidar data

**Short-term** (This week):
1. Implement SE(2) Hybrid A* with motion primitives
2. Add perception integration tests
3. Create ROS2 wrapper (value types)

**Medium-term** (Next week):
1. Generate C++ code for full pipeline
2. Deploy to Orin ARM64
3. Test with real lidar + odometry

---

**Document Status**: DESIGN - Perception integration architecture  
**Author**: GitHub Copilot + User  
**Last Updated**: October 7, 2025
