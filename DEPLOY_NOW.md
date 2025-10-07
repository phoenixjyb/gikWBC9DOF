# AGX Orin Deployment - Step-by-Step Guide

**Date**: October 7, 2025  
**Ready to Deploy!**

---

## 🎯 What We're Going to Do

1. Transfer the complete ROS2 workspace from Windows to AGX Orin
2. Build the MATLAB-generated solver natively on Orin (ARM64)
3. Test the solver on the target hardware

**Estimated Time**: 15-20 minutes (including build time)

---

## 📋 Before You Start

### Information You Need

1. **AGX Orin IP Address**: ________________
2. **Username** (default: `nvidia`): ________________
3. **SSH Access**: Make sure you can SSH to Orin from Windows

### Quick SSH Test

```powershell
# Test connection
ssh cr@<orin-ip-address>

# If it works, you'll see:
# cr@agx-orin:~$

# Type 'exit' to return to Windows
```

---

## 🚀 Option 1: Automated Deployment (RECOMMENDED)

We have a PowerShell script that does everything automatically!

### Step 1: Run the Deployment Script

```powershell
# From project root directory
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

# Run deployment (replace <orin-ip> with actual IP)
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.1.100"

# If your username is not 'nvidia':
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.1.100" -Username "your-username"

# To skip transferring mesh files (faster):
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.1.100" -SkipMeshes
```

**The script will:**
- ✅ Test SSH connection
- ✅ Create workspace on Orin
- ✅ Transfer ROS2 packages (gik9dof_msgs, gik9dof_solver)
- ✅ Transfer MATLAB codegen files (203 files)
- ✅ Transfer URDF and meshes
- ✅ Verify everything transferred correctly

**Expected Output:**
```
================================================
   AGX Orin Complete Deployment
================================================
Target: cr@192.168.1.100

[1/6] Testing SSH connection...
✓ SSH connection successful

[2/6] Creating workspace on AGX Orin...
✓ Workspace directories created

[3/6] Transferring ROS2 workspace...
  Using WSL rsync for faster transfer...
✓ ROS2 workspace transferred

[4/6] Transferring URDF file...
✓ URDF transferred

[5/6] Transferring mesh files...
✓ Meshes transferred

[6/6] Verifying transfer...
✓ gik9dof_msgs found
✓ gik9dof_solver found
✓ matlab_codegen found (115 files)
✓ URDF file found

================================================
   ✓ DEPLOYMENT COMPLETE!
================================================
```

### Step 2: SSH to AGX Orin

```powershell
ssh cr@<orin-ip>
```

### Step 3: Install Dependencies (First Time Only)

```bash
# Update package list
sudo apt update

# Install ROS2 and build dependencies
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  libeigen3-dev \
  libomp-dev \
  build-essential \
  cmake

# Verify installations
pkg-config --modversion eigen3  # Should show 3.4.0
ros2 --version                   # Should show ros2 doctor version
```

### Step 4: Build ROS2 Packages

```bash
cd ~/gikWBC9DOF/ros2

# Source ROS2
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expected output:
# Starting >>> gik9dof_msgs
# Finished <<< gik9dof_msgs [~10s]
# Summary: 1 package finished

# Source the built messages
source install/setup.bash

# Build solver (this will take ~45-60 seconds)
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expected output:
# Starting >>> gik9dof_solver
# [Processing 203 C++ files from MATLAB codegen...]
# Finished <<< gik9dof_solver [~50s]
# Summary: 1 package finished
```

**✅ If you see "Summary: 1 package finished" - SUCCESS!**

### Step 5: Test the Solver

```bash
# Source the workspace
source install/setup.bash

# Run the solver node
ros2 run gik9dof_solver gik9dof_solver_node
```

**Expected Output:**
```
[INFO] [1696723456.123]: Solver node starting...
[INFO] [1696723456.234]: MATLAB solver initialized successfully
[INFO] [1696723456.345]: Robot model loaded: 9 DOF
[INFO] [1696723456.456]: Subscribed to /hdas/feedback_arm_left
[INFO] [1696723456.567]: Subscribed to /odom_wheel
[INFO] [1696723456.678]: Subscribed to /gik9dof/target_trajectory
[INFO] [1696723456.789]: Publishing to /gik9dof/command_vel
[INFO] [1696723456.890]: Publishing to /gik9dof/solver_diagnostics
[INFO] [1696723456.999]: Waiting for trajectory commands...
```

**✅ If you see "Waiting for trajectory commands..." - IT WORKS!**

---

## 🧪 Quick Validation Test

Open another terminal on Orin:

```bash
# Terminal 2
ssh cr@<orin-ip>
cd ~/gikWBC9DOF/ros2
source install/setup.bash

# Check topics are created
ros2 topic list

# Should see:
# /gik9dof/command_vel
# /gik9dof/solver_diagnostics
# /gik9dof/target_trajectory
# /hdas/feedback_arm_left
# /odom_wheel

# Send a test trajectory
ros2 topic pub --once /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory \
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
positions:
- x: 0.5
  y: 0.0
  z: 0.3
orientations:
- x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
time_from_start: [0.0]"
```

**Watch Terminal 1 (solver node)** - you should see:
```
[INFO]: Received trajectory with 1 waypoints
[INFO]: Solving IK for waypoint 0...
[INFO]: Solution found! Iterations: 15, Time: 12.3 ms
[INFO]: Publishing command velocities
```

**✅ If you see this - FULL SUCCESS!** 🎉

---

## 🐛 Troubleshooting

### "colcon: command not found"

```bash
sudo apt install python3-colcon-common-extensions
```

### "Cannot find package 'Eigen3'"

```bash
sudo apt install libeigen3-dev
pkg-config --modversion eigen3
```

### "OpenMP not found"

```bash
sudo apt install libomp-dev
```

### "matlab_codegen directory not found"

The transfer didn't complete. Re-run the deployment script:
```powershell
.\deploy_to_orin_complete.ps1 -OrinIP "<your-ip>"
```

### Build succeeds but solver doesn't start

```bash
# Check for library issues
ldd install/gik9dof_solver/lib/gik9dof_solver/gik9dof_solver_node

# All libraries should show as "found"
```

---

## 📊 Performance Monitoring

Once running, check performance:

```bash
# In another terminal
source install/setup.bash

# Monitor diagnostics
ros2 topic echo /gik9dof/solver_diagnostics

# Expected output every 100ms:
# status: 1            # 1 = success
# iterations: 10-50    # depends on complexity
# solve_time_ms: 5-30  # target: <50ms
# error: 0.001         # target: <0.01
```

---

## ✅ Success Checklist

- [ ] Deployment script completed without errors
- [ ] Dependencies installed on Orin
- [ ] `gik9dof_msgs` built successfully
- [ ] `gik9dof_solver` built successfully (~50s build time)
- [ ] Solver node starts without errors
- [ ] All 5 topics created
- [ ] Test trajectory message received
- [ ] Solver computes and publishes solution
- [ ] Diagnostics show status=1 (success)

**If all checked - YOU'RE READY FOR ROBOT INTEGRATION!** 🚀

---

## 🔄 Making Changes and Rebuilding

If you need to make code changes:

```bash
# On Orin
cd ~/gikWBC9DOF/ros2

# Make your changes to src files...

# Rebuild only changed package
colcon build --packages-select gik9dof_solver

# Test immediately
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

---

## 📚 Additional Resources

- **Detailed Guide**: `docs/deployment/ORIN_DEPLOYMENT_GUIDE.md`
- **Validation**: `validation/VALIDATION_STATUS.md`
- **Architecture**: `docs/planning/CODEGENCC45_PROJECT_PLAN.md`

---

## 🆘 Still Having Issues?

1. **Check ROS2 is sourced**: `echo $ROS_DISTRO` should show `humble`
2. **Check workspace is sourced**: `ros2 pkg list | grep gik9dof`
3. **Check logs**: `cat log/latest_build/gik9dof_solver/stdout_stderr.log`

---

**Ready to Deploy?**

```powershell
.\deploy_to_orin_complete.ps1 -OrinIP "YOUR-ORIN-IP-HERE"
```

Good luck! 🚀
