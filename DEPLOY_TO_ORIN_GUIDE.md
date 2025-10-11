# Deploy to AGX Orin - Quick Guide

## Prerequisites

1. **SSH access** to AGX Orin configured (username: `cr`, remote path: `/home/nvidia/temp_gikrepo`)
2. **Generated code** ready (run codegen first if not done)
3. **Network connection** to Orin

## Quick Deploy

### Option 1: Deploy Complete Workspace (Recommended for first time)

```powershell
# Deploy everything + auto-build on Orin
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

This will:
- ✅ Transfer entire ROS2 workspace, URDF, meshes, config
- ✅ Build all packages automatically on Orin
- ✅ Verify installation

### Option 2: Deploy Chassis Controller Only (Fast, for testing)

```powershell
# Deploy only chassis path follower code
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly
```

This will:
- ✅ Transfer only MATLAB generated code + ROS2 wrapper
- ✅ Much faster (~30 seconds vs. 2-5 minutes)
- ✅ Perfect for iterative testing

### Option 3: Custom Settings

```powershell
# If you use different username or path (defaults are cr and /home/nvidia/temp_gikrepo)
.\scripts\deployment\deploy_to_orin.ps1 `
    -OrinIP "192.168.100.150" `
    -Username "cr" `
    -RemotePath "/home/nvidia/temp_gikrepo" `
    -Mode Complete
```

## What Gets Deployed

### Files Transferred (Total: ~90 KB)

1. **MATLAB Generated Code** (80 KB)
   - `ChassisPathFollower.cpp` (33.6 KB)
   - 6 other C++ files
   - 12 header files

2. **ROS2 Wrapper** (11 KB)
   - `chassis_path_follower_node.cpp`

3. **Build Files**
   - `CMakeLists.txt`
   - `package.xml`
   - `README.md`

4. **Launch File**
   - `chassis_path_follower_launch.py`

### Directory Structure on Orin

```
/home/nvidia/temp_gikrepo/
└── ros2/
    └── gik9dof_controllers/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── src/
        │   ├── chassis_path_follower_node.cpp
        │   └── matlab_generated/
        │       └── chassis_path_follower/
        │           ├── ChassisPathFollower.cpp
        │           ├── ChassisPathFollower.h
        │           └── (17 other files)
        └── launch/
            └── chassis_path_follower_launch.py
```

## Testing on Orin

### 1. Launch the Controller

```bash
# SSH to Orin
ssh cr@192.168.100.150

# Navigate to workspace
cd /home/nvidia/temp_gikrepo/ros2

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with default mode (Pure Pursuit)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py
```

### 2. Test Different Modes

```bash
# Mode 0: Differentiation (Open-Loop)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0

# Mode 1: Heading-Aware (Simple Feedback)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1

# Mode 2: Pure Pursuit (Full Feedback - Default)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2
```

### 3. Custom Parameters

```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py \
    controller_mode:=2 \
    vx_max:=1.5 \
    lookahead_base:=0.5 \
    heading_kp:=2.5
```

## Monitoring

### Check Topics

```bash
# List all topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor path following status
ros2 topic echo /path_following_active
```

### Check Parameters

```bash
# List all parameters
ros2 param list /chassis_path_follower

# Get specific parameter
ros2 param get /chassis_path_follower controller_mode
ros2 param get /chassis_path_follower vx_max

# Set parameter at runtime
ros2 param set /chassis_path_follower vx_max 1.5
```

### Check Node Status

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /chassis_path_follower
```

## Troubleshooting

### SSH Connection Failed

**Problem**: `SSH connection failed`

**Solutions**:
1. Check IP address: `ping 192.168.100.150`
2. Test SSH manually: `ssh nvidia@192.168.100.150`
3. Check username (default: `nvidia`, sometimes `cr`)
4. Verify network connection

### Generated Code Not Found

**Problem**: `Generated code not found`

**Solution**: Run codegen first:
```bash
wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh
```

### Build Fails on Orin

**Problem**: Compilation errors on Orin

**Solutions**:
1. Check ROS2 Humble installed: `source /opt/ros/humble/setup.bash`
2. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-rclcpp ros-humble-geometry-msgs \
       ros-humble-nav-msgs ros-humble-std-msgs
   ```
3. Clean and rebuild:
   ```bash
   cd /home/nvidia/temp_gikrepo/ros2
   rm -rf build/ install/ log/
   colcon build --packages-select gik9dof_controllers
   ```

### Node Doesn't Start

**Problem**: Launch file fails

**Solutions**:
1. Source workspace: `source install/setup.bash`
2. Check executable exists:
   ```bash
   ls install/gik9dof_controllers/lib/gik9dof_controllers/chassis_path_follower_node
   ```
3. Check permissions:
   ```bash
   chmod +x install/gik9dof_controllers/lib/gik9dof_controllers/chassis_path_follower_node
   ```

### Robot Doesn't Move

**Problem**: Commands sent but robot stationary

**Solutions**:
1. Check odometry: `ros2 topic echo /odom`
2. Check path received: `ros2 topic echo /path`
3. Monitor status: `ros2 topic echo /path_following_active`
4. Check velocity limits (may be too restrictive)

## Performance Validation

### Timing Test

```bash
# On Orin, monitor node performance
ros2 topic hz /cmd_vel

# Should see: 100 Hz (10ms cycle time)
```

### CPU Usage

```bash
# Monitor CPU usage
top -p $(pgrep -f chassis_path_follower_node)

# Should be < 5% on single core for typical operation
```

### Memory Usage

```bash
# Check memory usage
ps aux | grep chassis_path_follower_node

# Should be ~100 MB
```

## Quick Commands Reference

```bash
# Deploy from Windows (complete workspace)
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete

# Deploy from Windows (chassis only, faster)
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly

# On Orin: Source and launch
source /opt/ros/humble/setup.bash
cd /home/nvidia/temp_gikrepo/ros2
source install/setup.bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py

# Monitor velocity
ros2 topic echo /cmd_vel

# Check parameters
ros2 param list /chassis_path_follower

# Stop node
Ctrl+C
```

## Documentation

- **Complete Usage Guide**: `ros2/gik9dof_controllers/README.md` (on Orin after deployment)
- **Codegen Details**: `DAY5_CODEGEN_SESSION_SUMMARY.md`
- **Session Summary**: `SESSION_COMPLETE.md`
- **MATLAB Tests**: `matlab/test_chassis_path_3modes.m`

## Support

If you encounter issues:
1. Check the README on Orin: `cat ros2/gik9dof_controllers/README.md`
2. Review build logs: `cat ros2/log/latest_build/gik9dof_controllers/stdout_stderr.log`
3. Check system logs: `journalctl -u ros2-chassis-path-follower`

## Next Steps After Successful Deployment

1. ✅ Verify all 3 modes work correctly
2. ✅ Tune parameters for your robot
3. ✅ Test with real trajectory paths
4. ✅ Compare performance to MATLAB simulation
5. ✅ Profile timing and resource usage
6. ✅ Document any robot-specific tuning

---

**One Unified Script for Everything!** 🎯

```powershell
# Complete workspace (first deployment)
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete

# Chassis only (fast updates)
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly
```

That's it! 🚀
