# 🚀 READY TO DEPLOY NOW

Everything is ready! Here's exactly what to run:

## Step 1: First Time Deployment

```powershell
# Run this from PowerShell in the project root
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

**Replace `192.168.100.150` with your actual Orin IP address.**

### What This Does:

1. ✅ Tests SSH connection to `cr@192.168.100.150`
2. ✅ Creates directory structure at `/home/nvidia/temp_gikrepo/`
3. ✅ Transfers entire ROS2 workspace (~90 KB)
4. ✅ Transfers URDF files
5. ✅ Transfers mesh files
6. ✅ Transfers message definitions
7. ✅ Builds everything on Orin with ARM64 optimizations
8. ✅ Verifies installation

**Time**: 2-5 minutes

## Step 2: Test on Orin

After deployment completes, SSH to Orin:

```bash
ssh cr@192.168.100.150
cd /home/nvidia/temp_gikrepo/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch default mode (Pure Pursuit)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py
```

## Step 3: Test All 3 Modes

In separate terminals on Orin:

```bash
# Mode 0: Differentiation (Open-Loop)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0

# Mode 1: Heading-Aware (Simple Feedback)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1

# Mode 2: Pure Pursuit (Full Feedback - Default)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2
```

## Future Updates (Fast Mode)

After initial deployment, when you make code changes:

```powershell
# 1. Regenerate code in WSL
wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh

# 2. Fast deploy (30 seconds)
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly

# 3. Test on Orin (same commands as Step 2)
```

## One-Liner for Quick Deploy

```powershell
# Complete deployment in one command
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete -BuildOnOrin:$true
```

---

## ⚡ Your Exact Configuration

- **Username**: `cr` ✅
- **Remote Path**: `/home/nvidia/temp_gikrepo` ✅
- **No need to specify** - these are now defaults!

## 📋 Pre-Flight Checklist

Before running deployment:

- [ ] Know your Orin IP address
- [ ] Can SSH to Orin: `ssh cr@192.168.100.150`
- [ ] Generated code exists: `ls codegen/chassis_path_follower_arm64/`
- [ ] In project root directory

## 🎯 GO!

```powershell
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "YOUR_ORIN_IP" -Mode Complete
```

That's it! The script will guide you through everything. 🚀
