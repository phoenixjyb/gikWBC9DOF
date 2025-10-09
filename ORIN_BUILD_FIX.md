# Orin Build Fix - October 9, 2025

## Problem
CMake cache contains Windows paths from development machine, and duplicate package detected.

## Solution

Run these commands on the Orin (cr@192.168.100.150):

```bash
# Navigate to ROS2 workspace
cd /home/nvidia/temp_gikrepo/ros2

# Clean old build artifacts (contains Windows paths)
rm -rf build/ install/ log/

# Remove backup package causing duplicate error
rm -rf gik9dof_solver_bak/

# Now build fresh
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Build solver
colcon build --packages-select gik9dof_solver

# Test
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

## Why This Happened
- `rsync` transferred the `build/` and `install/` directories with CMake cache containing Windows paths
- A backup package `gik9dof_solver_bak` was also transferred, causing duplicate package names

## Prevention
The deployment script should exclude these directories. Updated script will exclude:
- `build/`
- `install/`
- `log/`
- `*_bak/` packages
