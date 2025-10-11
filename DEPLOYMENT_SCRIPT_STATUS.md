# ⚠️ DEPLOYMENT SCRIPT STATUS

## Issue
The `deploy_to_orin.ps1` script got corrupted during refactoring and has been restored to a minimal working version.

## Current Status
✅ **Script parses correctly**  
✅ **Parameters work**  
⚠️ **Full deployment logic needs to be re-added**

## Workaround: Manual Deployment

Until the script is fully restored, use these manual steps:

### 1. Transfer Files via SCP

```powershell
# From Windows PowerShell in project root
scp -r ros2 cr@192.168.100.150:/home/nvidia/temp_gikrepo/
```

### 2. SSH to Orin and Build

```bash
ssh cr@192.168.100.150
cd /home/nvidia/temp_gikrepo/ros2

# Source ROS2
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs

# Source messages
source install/setup.bash

# Build controllers
colcon build --packages-select gik9dof_controllers \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-march=armv8-a -mtune=cortex-a78 -O3 -ffast-math"
```

### 3. Launch and Test

```bash
source install/setup.bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py
```

## Alternative: Use rsync (if available)

```powershell
# Much faster than scp
rsync -avz --progress `
    --exclude='build/' --exclude='install/' --exclude='log/' `
    ros2 cr@192.168.100.150:/home/nvidia/temp_gikrepo/
```

## TODO
- [ ] Restore full deployment script logic
- [ ] Add SSH connection testing
- [ ] Add workspace structure creation
- [ ] Add file verification
- [ ] Add optional build on Orin
- [ ] Add deployment summary

## Quick Reference

**Your Settings:**
- Username: `cr`
- Remote Path: `/home/nvidia/temp_gikrepo`
- IP: `192.168.100.150` (replace with actual)

---

**For now, use manual deployment steps above.** ✅
