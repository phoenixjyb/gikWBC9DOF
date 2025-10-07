# Hybrid A* Planner Integration Guide

**Date**: October 7, 2025  
**Status**: In Progress  
**Next Steps**: Update CMakeLists.txt and implement Stage B

---

## ✅ Completed

1. **Generated ARM64 C++ Code**
   - Location: `codegen/planner_arm64/`
   - Generation time: 48.3 seconds
   - Output: Full C++ library with ARM optimizations

2. **Copied to ROS2 Package**
   - Destination: `ros2/gik9dof_solver/src/generated/planner/`
   - Files: ~50 C++ source/header files
   - Includes: HybridAStarPlanner class interface

3. **Architecture Decision Documented**
   - See: `docs/STAGED_CONTROL_ARCHITECTURE.md`
   - Decision: Integrate into `gik9dof_solver` node
   - Rationale: Stage B2 requires tight GIK-planner coupling

---

## 🔄 Next Steps

### Step 1: Update CMakeLists.txt ← **DO THIS NEXT**

Add planner library to build:

```cmake
# In ros2/gik9dof_solver/CMakeLists.txt

# After existing generated code sections, add:

# ========================================
# Hybrid A* Planner (MATLAB Coder ARM64)
# ========================================
set(PLANNER_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/generated/planner")

file(GLOB PLANNER_SOURCES
    "${PLANNER_GENERATED_DIR}/*.cpp"
)

# Remove main/example files if any
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*example.*\\.cpp$")

# Add planner sources to executable
target_sources(gik9dof_solver_node PRIVATE
    ${PLANNER_SOURCES}
)

# Add planner include directory
target_include_directories(gik9dof_solver_node PRIVATE
    ${PLANNER_GENERATED_DIR}
)
```

### Step 2: Create Stage B Controller

Create file: `ros2/gik9dof_solver/src/stage_b_chassis_plan.cpp`

```cpp
#include "gik9dof_solver_node.hpp"
#include "generated/planner/HybridAStarPlanner.h"
#include "generated/planner/OccupancyGrid2D.h"

void GIK9DOFSolverNode::executeStageB1_PureHybridAStar() 
{
    // 1. Plan path with Hybrid A*
    gik9dof::struct0_T start, goal;
    start.x = current_pose_.position.x;
    start.y = current_pose_.position.y;
    start.theta = current_yaw_;
    
    goal.x = goal_pose_.position.x;
    goal.y = goal_pose_.position.y;
    goal.theta = goal_yaw_;
    
    gik9dof::struct1_T path[500];
    gik9dof::struct2_T stats;
    
    hybrid_astar_planner_.b_gik9dof_planHybridAStarCodegen(
        &start, &goal, &occupancy_grid_, path, &stats
    );
    
    // 2. Send to Pure Pursuit velocity controller
    if (velocity_control_mode_ == 2) {  // Pure Pursuit
        auto [vx, wz] = computePurePursuitVelocity(path, stats.path_length);
        publishVelocity(vx, wz);
    }
    
    // 3. Check completion
    if (chassisReachedGoal()) {
        RCLCPP_INFO(this->get_logger(), "Stage B1 complete");
        transitionToStage(Stage::STAGE_C_TRACKING);
    }
}

void GIK9DOFSolverNode::executeStageB2_GIKAssisted() 
{
    // 1. Strategic planning with Hybrid A*
    gik9dof::struct1_T path[500];
    gik9dof::struct2_T stats;
    
    hybrid_astar_planner_.b_gik9dof_planHybridAStarCodegen(
        &start_state_, &goal_state_, &occupancy_grid_, path, &stats
    );
    
    // 2. For each waypoint, use GIK to refine
    int current_waypoint = getCurrentWaypointIndex();
    if (current_waypoint < stats.path_length) {
        auto waypoint = path[current_waypoint];
        
        // Call GIK solver with 3-DOF chassis virtuals
        auto chassis_config = solveGIK3DOFChassis(waypoint);
        
        // Compute velocity
        auto [vx, wz] = computeVelocityFromConfig(chassis_config);
        publishVelocity(vx, wz);
    }
    
    // 3. Check completion
    if (chassisReachedGoal()) {
        RCLCPP_INFO(this->get_logger(), "Stage B2 complete");
        transitionToStage(Stage::STAGE_C_TRACKING);
    }
}
```

### Step 3: Add Occupancy Grid Subscription

In `gik9dof_solver_node.cpp` constructor:

```cpp
// Subscribe to occupancy grid (for Stage B planning)
occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10,
    std::bind(&GIK9DOFSolverNode::occupancyGridCallback, this, std::placeholders::_1)
);

// Initialize planner
hybrid_astar_planner_ = std::make_unique<gik9dof::HybridAStarPlanner>();
```

Callback:

```cpp
void GIK9DOFSolverNode::occupancyGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Convert ROS OccupancyGrid to MATLAB Coder OccupancyGrid2D
    occupancy_grid_.setGridData(
        msg->data.data(),
        msg->info.width,
        msg->info.height,
        msg->info.resolution,
        msg->info.origin.position.x,
        msg->info.origin.position.y
    );
    
    grid_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Occupancy grid updated: %dx%d @ %.2fm",
                 msg->info.width, msg->info.height, msg->info.resolution);
}
```

### Step 4: Add ROS2 Parameters

In `config/gik9dof_solver.yaml`:

```yaml
gik9dof_solver:
  ros__parameters:
    # Control mode
    control_mode: "staged"  # "holistic" or "staged"
    
    # Staged mode configuration
    staged:
      initial_stage: "stage_b"
      
      stage_b:
        submode: "pure_hybrid_astar"  # or "gik_assisted"
        
        hybrid_astar:
          grid_resolution: 0.1
          max_planning_time: 0.05
          state_lattice_theta_bins: 72
          goal_position_tolerance: 0.2
          goal_theta_tolerance: 0.175
          
    # Velocity controller
    velocity_control_mode: 2  # Pure Pursuit
```

### Step 5: Build and Test

```bash
# On development machine (WSL or Linux)
cd ~/gikWBC9DOF/ros2
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run tests
ros2 run gik9dof_solver gik9dof_solver_node --ros-args --params-file config/gik9dof_solver.yaml

# Monitor planning
ros2 topic echo /solver_diagnostics
```

### Step 6: Deploy to Orin

```bash
# Copy workspace to Orin
scp -r ~/gikWBC9DOF/ros2 orin@orin-ip:/path/to/workspace/

# On Orin
cd /path/to/workspace/ros2
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run
ros2 run gik9dof_solver gik9dof_solver_node
```

---

## 📐 Planner API Reference

### HybridAStarPlanner Class

```cpp
namespace gik9dof {
    class HybridAStarPlanner {
    public:
        HybridAStarPlanner();
        ~HybridAStarPlanner();
        
        void b_gik9dof_planHybridAStarCodegen(
            struct0_T *start_state,      // Input: (x, y, theta)
            const struct0_T *goal_state,  // Input: (x, y, theta)
            const OccupancyGrid2D *grid,  // Input: Obstacle map
            struct1_T path[500],          // Output: Path waypoints
            struct2_T *search_stats       // Output: Planning metrics
        );
    };
}
```

### Data Structures

```cpp
// State (pose)
struct struct0_T {
    double x;      // meters
    double y;      // meters
    double theta;  // radians
};

// Path waypoint
struct struct1_T {
    double x;
    double y;
    double theta;
    double Vx;     // Forward velocity (m/s)
    double Wz;     // Yaw rate (rad/s)
    double dt;     // Time step (s)
};

// Search statistics
struct struct2_T {
    int path_length;         // Number of waypoints
    int nodes_expanded;      // A* search nodes
    double planning_time;    // seconds
    bool success;            // Planning succeeded
};
```

---

## 🐛 Troubleshooting

### CMake Issues

**Problem**: Undefined references to planner functions

**Solution**:
- Verify all `.cpp` files in `generated/planner/` are added to `PLANNER_SOURCES`
- Check `target_include_directories` includes planner directory
- Rebuild: `colcon build --packages-select gik9dof_solver --cmake-clean-first`

### Runtime Issues

**Problem**: Planning takes too long (> 50ms)

**Solution**:
- Reduce `state_lattice_theta_bins` (72 → 36)
- Decrease grid resolution (0.1m → 0.15m)
- Limit `max_iterations` in A* search

**Problem**: No path found

**Solution**:
- Check occupancy grid is published on `/occupancy_grid`
- Verify robot footprint doesn't collide at start/goal
- Increase `goal_position_tolerance` and `goal_theta_tolerance`
- Enable `enable_reverse` for backward motion

### Integration Issues

**Problem**: Stage B never completes

**Solution**:
- Check `chassisReachedGoal()` tolerance values
- Monitor actual chassis pose vs path waypoints
- Add timeout: `stage_b.max_duration`

---

## 📊 Performance Monitoring

Add diagnostics publishing:

```cpp
void GIK9DOFSolverNode::publishPlannerDiagnostics(const gik9dof::struct2_T& stats)
{
    auto msg = gik9dof_msgs::msg::PlannerDiagnostics();
    msg.stamp = this->now();
    msg.path_length = stats.path_length;
    msg.nodes_expanded = stats.nodes_expanded;
    msg.planning_time = stats.planning_time;
    msg.success = stats.success;
    
    planner_diagnostics_pub_->publish(msg);
}
```

Monitor in terminal:

```bash
ros2 topic echo /planner_diagnostics
```

---

## 📝 TODO Tracking

- [ ] Update `CMakeLists.txt` to link planner library
- [ ] Create `stage_b_chassis_plan.cpp` with B1/B2 methods
- [ ] Add occupancy grid subscription
- [ ] Implement `chassisReachedGoal()` completion check
- [ ] Add ROS2 parameters for Stage B configuration
- [ ] Create `PlannerDiagnostics.msg` message type
- [ ] Test Stage B1 (Pure Hybrid A*)
- [ ] Test Stage B2 (GIK-Assisted)
- [ ] Deploy to Orin and validate < 50ms planning
- [ ] Integration test with full staged pipeline (A → B → C)

---

**Last Updated**: October 7, 2025  
**Current Step**: Update CMakeLists.txt
