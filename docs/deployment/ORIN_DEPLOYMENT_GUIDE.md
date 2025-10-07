# AGX Orin Deployment Guide

**Date**: October 7, 2025  
**Target**: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64  
**Source**: Windows 11 Development Machine

---

## 🎯 Objective

Deploy the complete MATLAB-generated C++ solver to AGX Orin and build it natively on the target platform.

---

## 📋 Prerequisites

### On Windows (Your Machine)
- ✅ MATLAB codegen complete (`ros2/gik9dof_solver/matlab_codegen/` populated)
- ✅ ROS2 packages ready (`gik9dof_msgs`, `gik9dof_solver`)
- ✅ SSH access to AGX Orin configured

### On AGX Orin
- Ubuntu 22.04 LTS
- ROS2 Humble installed
- Network connectivity to Windows machine
- User account: `cr` (or your username)

---

## 🚀 Deployment Methods

We have **two approaches**. Choose based on your situation:

### Method 1: Direct rsync (Recommended - Fast)
Transfer the entire `ros2/` workspace directly

### Method 2: ZIP Package (If rsync unavailable)
Create a deployment package and transfer via SCP

---

## 📦 Method 1: Direct rsync Deployment (RECOMMENDED)

This is the fastest and cleanest approach.

### Step 1: Prepare AGX Orin

SSH to Orin and create workspace:

```bash
# SSH to Orin
ssh cr@<orin-ip>

# Create workspace
mkdir -p ~/gikWBC9DOF/ros2
exit
```

### Step 2: Transfer ROS2 Workspace from Windows

From PowerShell on Windows:

```powershell
# Set Orin IP address
$ORIN_IP = "<your-orin-ip>"  # e.g., "192.168.1.100"
$ORIN_USER = "cr"

# Transfer entire ROS2 workspace
# Using WSL with rsync (fastest)
wsl rsync -avz --progress `
  /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2/ `
  $ORIN_USER@${ORIN_IP}:~/gikWBC9DOF/ros2/

# This will transfer:
# - gik9dof_msgs/
# - gik9dof_solver/ (including matlab_codegen/)
# - build/ (optional, can skip)
# - install/ (optional, can skip)
```

**Alternative without WSL (using scp):**

```powershell
# Compress first
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
Compress-Archive -Path ros2\gik9dof_msgs, ros2\gik9dof_solver -DestinationPath ros2_workspace.zip

# Transfer
scp ros2_workspace.zip ${ORIN_USER}@${ORIN_IP}:~/

# On Orin, extract
ssh ${ORIN_USER}@${ORIN_IP} "cd ~/gikWBC9DOF && unzip -o ~/ros2_workspace.zip"
```

### Step 3: Transfer Additional Files

```powershell
# Transfer URDF and meshes (needed for solver)
wsl rsync -avz --progress `
  /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/mobile_manipulator_PPR_base_corrected.urdf `
  $ORIN_USER@${ORIN_IP}:~/gikWBC9DOF/

wsl rsync -avz --progress `
  /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/meshes/ `
  $ORIN_USER@${ORIN_IP}:~/gikWBC9DOF/meshes/
```

---

## 🔨 Building on AGX Orin

### Step 1: SSH to Orin

```bash
ssh cr@<orin-ip>
```

### Step 2: Install Dependencies

```bash
# Update package list
sudo apt update

# Install required dependencies
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  libeigen3-dev \
  libomp-dev \
  build-essential \
  cmake

# Verify Eigen3
pkg-config --modversion eigen3
# Should show: 3.4.0 (or similar)
```

### Step 3: Source ROS2

```bash
source /opt/ros/humble/setup.bash

# Verify ROS2
ros2 --version
# Should show: ros2 doctor version 0.10.x
```

### Step 4: Build ROS2 Packages

```bash
cd ~/gikWBC9DOF/ros2

# Clean previous build (optional)
rm -rf build/ install/ log/

# Build messages first
colcon build --packages-select gik9dof_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the messages
source install/setup.bash

# Build solver
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expected output:
# Starting >>> gik9dof_solver
# Finished <<< gik9dof_solver [45-60s]
# 
# Summary: 1 package finished [~1min]
```

### Step 5: Verify Build

```bash
# Check for executable
ls -lh install/gik9dof_solver/lib/gik9dof_solver/

# Should see:
# gik9dof_solver_node (executable)

# Check library was built
find build/gik9dof_solver -name "*.so" -o -name "*.a"

# Should see MATLAB solver library
```

---

## ✅ Testing on AGX Orin

### Quick Test

```bash
# Source workspace
cd ~/gikWBC9DOF/ros2
source install/setup.bash

# Run solver node
ros2 run gik9dof_solver gik9dof_solver_node
```

**Expected output:**
```
[INFO] [gik9dof_solver]: Solver node starting...
[INFO] [gik9dof_solver]: MATLAB solver initialized successfully
[INFO] [gik9dof_solver]: Waiting for trajectory commands...
```

### Test with Mock Data

In another terminal:

```bash
cd ~/gikWBC9DOF/ros2
source install/setup.bash

# Check topics
ros2 topic list
# Should see:
# /gik9dof/command_vel
# /gik9dof/solver_diagnostics
# /gik9dof/target_trajectory
# /hdas/feedback_arm_left
# /odom_wheel

# Test publishing trajectory
ros2 topic pub --once /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory \
  "header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}
   positions: [{x: 0.5, y: 0.0, z: 0.3}]
   orientations: [{x: 0.0, y: 0.0, z: 0.0, w: 1.0}]
   time_from_start: [0.0]"
```

**Watch solver node terminal** - should show:
```
[INFO] [gik9dof_solver]: Received trajectory with 1 waypoints
[INFO] [gik9dof_solver]: Solving IK...
[INFO] [gik9dof_solver]: Solution found in X iterations, Y ms
```

---

## 🔍 Troubleshooting

### Build Error: "Eigen3 not found"

```bash
sudo apt install libeigen3-dev
pkg-config --modversion eigen3
```

### Build Error: "OpenMP not found"

```bash
sudo apt install libomp-dev
```

### Build Error: "MATLAB codegen files not found"

```bash
# Verify matlab_codegen directory exists
ls -la ~/gikWBC9DOF/ros2/gik9dof_solver/matlab_codegen/

# Should contain:
# include/ (with .h and .cpp files)
# lib/ (optional, we build from source)
```

If missing, re-transfer from Windows.

### Runtime Error: "Cannot find URDF file"

```bash
# Check URDF exists
ls -la ~/gikWBC9DOF/*.urdf

# Update path in solver node if needed
```

### Solver Not Receiving Messages

```bash
# Check QoS settings
ros2 topic info /gik9dof/target_trajectory -v

# Should show publishers and subscribers
```

---

## 📊 Performance Expectations

### Build Times (AGX Orin)
- `gik9dof_msgs`: ~10 seconds
- `gik9dof_solver`: ~45-60 seconds (203 C++ files)

### Runtime Performance
- Control loop: 10 Hz (100 ms period)
- Solver time: < 50 ms per iteration
- Expected iterations: 10-50 (depending on trajectory complexity)

### Resource Usage
- RAM: ~500 MB (solver + ROS2 overhead)
- CPU: 1-2 cores (with OpenMP parallelization)

---

## 🎯 Success Criteria

✅ **Build Success:**
- Both packages build without errors
- `gik9dof_solver_node` executable created
- No missing dependencies

✅ **Runtime Success:**
- Node starts without crashes
- MATLAB solver initializes
- All 4 topics created
- Can receive trajectory messages

✅ **Solver Success:**
- Receives state feedback
- Computes IK solutions
- Publishes command velocities
- Diagnostics show convergence

---

## 📝 Quick Command Reference

```bash
# === On Windows ===
# Transfer workspace
wsl rsync -avz ros2/ cr@<orin-ip>:~/gikWBC9DOF/ros2/

# === On AGX Orin ===
# Install deps
sudo apt install ros-humble-rclcpp libeigen3-dev libomp-dev

# Build
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_msgs gik9dof_solver

# Run
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node

# Test
ros2 topic list
ros2 topic echo /gik9dof/solver_diagnostics
```

---

## 🔄 Iterative Development Workflow

For quick iterations during development:

```bash
# 1. On Windows: Make code changes

# 2. Transfer only changed files
wsl rsync -avz --include='*.cpp' --include='*.h' --exclude='build/' `
  ros2/gik9dof_solver/ cr@<orin-ip>:~/gikWBC9DOF/ros2/gik9dof_solver/

# 3. On Orin: Rebuild just the solver
cd ~/gikWBC9DOF/ros2
colcon build --packages-select gik9dof_solver

# 4. Test immediately
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

---

## 🆘 Need Help?

1. **Build Issues**: Check [WSL Validation Guide](../deployment/wsl/WSL_VALIDATION_GUIDE.md)
2. **Runtime Issues**: Check [Validation Status](../../validation/VALIDATION_STATUS.md)
3. **Architecture**: Check [Project Plan](../planning/CODEGENCC45_PROJECT_PLAN.md)

---

**Ready to deploy?** Follow Method 1 (rsync) for fastest results!
