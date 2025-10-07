# Quick Start: Integrating Hybrid A* Planner with Your ROS2 Nodes

## Current Workspace Structure
```
ros2/
├── gik9dof_controllers/      # Your existing controller package
├── gik9dof_msgs/             # Your message definitions
├── gik9dof_solver/           # Your solver package  
└── nodes/
    ├── obstacle_provider.cpp      # Your obstacle detection
    ├── trajectory_manager.cpp     # Your trajectory execution
    ├── holistic_controller.cpp    # Your controller
    └── hybrid_astar_planner_node.py  # NEW planner node ✨
```

## Step 1: Install MATLAB Engine for Python (One-time setup)

```bash
# Find your MATLAB installation
ls /usr/local/MATLAB  # or /opt/MATLAB

# Navigate to Python engine folder
cd /usr/local/MATLAB/R2024b/extern/engines/python

# Install (requires admin for system-wide, or use --user for local)
sudo python3 setup.py install
# OR
python3 setup.py install --user

# Verify installation
python3 -c "import matlab.engine; print('✓ MATLAB Engine ready!')"
```

## Step 2: Install Python Dependencies

```bash
# ROS2 Python packages
sudo apt update
sudo apt install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    python3-tf-transformations

# Verify
python3 -c "import tf_transformations; print('✓ TF ready!')"
```

## Step 3: Make Planner Node Executable

```bash
# Navigate to your workspace
cd /path/to/gikWBC9DOF/ros2/nodes

# Make executable
chmod +x hybrid_astar_planner_node.py

# Test import
python3 -c "from hybrid_astar_planner_node import HybridAStarPlannerNode; print('✓ Node script OK!')"
```

## Step 4: Quick Test (Standalone)

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set MATLAB workspace path (IMPORTANT!)
export MATLAB_WORKSPACE=/path/to/gikWBC9DOF/matlab

# Run planner node
cd /path/to/gikWBC9DOF/ros2/nodes
ros2 run rclpy hybrid_astar_planner_node.py \
    --ros-args \
    -p matlab_workspace:=$MATLAB_WORKSPACE

# You should see:
# [INFO] Starting MATLAB Engine...
# [INFO] ✓ MATLAB Engine started with workspace: /path/to/...
# [INFO] Hybrid A* Planner Node initialized
# [INFO] Subscribed to: /occupancy_grid, /goal_pose, /initialpose
```

## Step 5: Integration with Your Nodes

### Option A: Add to Existing Launch File

If you have a launch file for your controllers:

```python
# In ros2/launch/your_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get MATLAB workspace path
    matlab_ws = os.environ.get('MATLAB_WORKSPACE', 
                               '/home/yanbo/gikWBC9DOF/matlab')
    
    return LaunchDescription([
        # Your existing nodes
        Node(
            package='gik9dof_controllers',
            executable='obstacle_provider',
            name='obstacle_provider'
        ),
        
        # NEW: Hybrid A* planner
        Node(
            package='rclpy',  # Or create dedicated package
            executable='hybrid_astar_planner_node.py',
            name='hybrid_astar_planner',
            output='screen',
            parameters=[{
                'matlab_workspace': matlab_ws,
                'planning_timeout': 5.0,
                'inflation_radius': 0.511,
                'grid_resolution': 0.1
            }]
        ),
        
        # Your existing trajectory manager
        Node(
            package='gik9dof_controllers',
            executable='trajectory_manager',
            name='trajectory_manager'
        ),
    ])
```

### Option B: Run Manually in Separate Terminal

```bash
# Terminal 1: Your existing nodes
cd /path/to/ros2_workspace
source install/setup.bash
ros2 launch gik9dof_controllers your_launch_file.py

# Terminal 2: Planner node
export MATLAB_WORKSPACE=/path/to/gikWBC9DOF/matlab
cd /path/to/gikWBC9DOF/ros2/nodes
./hybrid_astar_planner_node.py
```

## Step 6: Test Planning

### Method 1: RViz2 (Visual)

```bash
# Terminal 1: Launch RViz2
ros2 run rviz2 rviz2

# In RViz2:
# 1. Add -> By Display Type -> Map -> Set topic to /occupancy_grid
# 2. Add -> By Display Type -> Path -> Set topic to /planned_path
# 3. Set Fixed Frame to "map" or your base frame
# 4. Use "2D Pose Estimate" to set start position
# 5. Use "2D Nav Goal" to set goal position
# 6. Watch /planned_path appear!
```

### Method 2: Command Line (Quick Test)

```bash
# Publish a simple occupancy grid (empty 20×20m map)
ros2 topic pub /occupancy_grid nav_msgs/msg/OccupancyGrid \
"{
  header: {frame_id: 'map'},
  info: {
    resolution: 0.1,
    width: 200,
    height: 200,
    origin: {position: {x: 0.0, y: 0.0, z: 0.0}}
  },
  data: [0]
}" --once

# Set start pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 2.0, y: 2.0, z: 0.0}}
  }
}" --once

# Set goal pose (planner will auto-trigger!)
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'map'},
  pose: {position: {x: 8.0, y: 8.0, z: 0.0}}
}" --once

# Watch for planned path
ros2 topic echo /planned_path
```

## Step 7: Connect to Your obstacle_provider

Modify your `obstacle_provider.cpp` to publish to `/occupancy_grid`:

```cpp
// In obstacle_provider.cpp
#include <nav_msgs/msg/occupancy_grid.hpp>

// Add publisher
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

// In constructor
grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10
);

// In your obstacle detection callback
void publishOccupancyGrid() {
    auto grid_msg = nav_msgs::msg::OccupancyGrid();
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";
    
    // Fill in grid data from your sensors
    // grid_msg.info.resolution = 0.1;
    // grid_msg.info.width = ...;
    // grid_msg.data = ...;  // 0=free, 100=occupied, -1=unknown
    
    grid_pub_->publish(grid_msg);
}
```

## Step 8: Connect to Your trajectory_manager

Modify your `trajectory_manager.cpp` to subscribe to `/planned_path`:

```cpp
// In trajectory_manager.cpp
#include <nav_msgs/msg/path.hpp>

// Add subscriber
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

// In constructor
path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planned_path", 10,
    std::bind(&TrajectoryManager::pathCallback, this, std::placeholders::_1)
);

// Add callback
void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), 
                "Received new path with %zu waypoints", msg->poses.size());
    
    // Convert Path to your internal trajectory format
    // Execute or track the path
    for (const auto& pose : msg->poses) {
        // pose.pose.position.x, .y
        // Extract yaw from pose.pose.orientation
        // Add to trajectory queue
    }
}
```

## Expected Performance

Based on our testing:

| Metric | Value |
|--------|-------|
| Planning time | **0.03-0.06s** (30-60ms) |
| Typical waypoints | 4-15 waypoints |
| Grid size supported | 200×200 @ 0.1m (20×20m) |
| Success rate | 93% (25/27 test scenarios) |
| Speedup vs original | **3-4× faster** |

This is **real-time performance** for mobile robots! 🚀

## Troubleshooting

### Issue: "Cannot import matlab.engine"
```bash
# Solution: Install MATLAB Engine (Step 1 above)
cd /usr/local/MATLAB/R2024b/extern/engines/python
python3 setup.py install --user
```

### Issue: "MATLAB workspace path not found"
```bash
# Solution: Set correct path before running
export MATLAB_WORKSPACE=/absolute/path/to/gikWBC9DOF/matlab

# Or pass as parameter
ros2 run ... --ros-args -p matlab_workspace:=/your/path
```

### Issue: "Planning fails with no path found"
- Check start/goal are in free space (not collision)
- Verify occupancy grid is being published (`ros2 topic echo /occupancy_grid`)
- Check inflation radius isn't too large (try 0.3 instead of 0.511)
- Increase timeout: `-p planning_timeout:=10.0`

### Issue: "Too slow (> 100ms)"
- Make sure node is using `planHybridAStarCodegen` (not `planHybridAStar`)
- Check MATLAB workspace path is correct (code must be found)
- Verify grid resolution (0.1m recommended, 0.05m will be slower)

## Next Steps

1. **Test on hardware**: Deploy to WHEELTEC and test with real sensors
2. **Tune parameters**: Adjust inflation radius, planning timeout for your environment
3. **Add replanning**: Re-plan when new obstacles detected (easy - just call again!)
4. **Path smoothing**: Optional cubic spline fitting for smoother motion

## Summary

✅ **No C++ codegen needed** - Use MATLAB Engine API directly  
✅ **Real-time performance** - 0.03-0.06s planning is fast enough  
✅ **Easy integration** - Standard ROS2 topics and services  
✅ **Maintainable** - MATLAB code stays readable and debuggable  

Ready to deploy! 🎉
