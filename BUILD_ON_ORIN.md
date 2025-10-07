# Building and Testing on AGX Orin - Commands to Run

**All files are on Orin - Ready to build! 🚀**

---

## 📋 Step-by-Step Commands

Copy and paste these commands into your SSH session on the Orin.

---

## Step 1: SSH to Orin

```bash
ssh cr@192.168.100.150
```

---

## Step 2: Verify Files Are There

```bash
# Check workspace structure
ls -la ~/gikWBC9DOF/

# Should see:
# ros2/
# mobile_manipulator_PPR_base_corrected.urdf

# Check ROS2 packages
ls ~/gikWBC9DOF/ros2/

# Should see:
# gik9dof_msgs/
# gik9dof_solver/

# Check MATLAB codegen
find ~/gikWBC9DOF/ros2/gik9dof_solver/matlab_codegen -type f | wc -l

# Should show: 115 files ✓
```

---

## Step 3: Install Dependencies (First Time Only)

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
pkg-config --modversion eigen3
# Should show: 3.4.0 (or similar) ✓

ros2 --version
# Should show: ros2 doctor version 0.10.x ✓
```

---

## Step 4: Build ROS2 Packages

```bash
# Navigate to workspace
cd ~/gikWBC9DOF/ros2

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Verify ROS2 is sourced
echo $ROS_DISTRO
# Should show: humble ✓

# Build messages package first
echo "Building gik9dof_msgs..."
colcon build --packages-select gik9dof_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expected output:
# Starting >>> gik9dof_msgs
# Finished <<< gik9dof_msgs [~10s]
# Summary: 1 package finished [10s]
```

**✅ If you see "Summary: 1 package finished" - SUCCESS! Continue...**

```bash
# Source the built messages
source install/setup.bash

# Build solver package (this will take ~45-60 seconds)
echo "Building gik9dof_solver (this may take ~1 minute)..."
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Expected output:
# Starting >>> gik9dof_solver
# [Processing: lots of .cpp files from MATLAB codegen]
# Finished <<< gik9dof_solver [45-60s]
# Summary: 1 package finished [~1min]
```

**✅ If you see "Summary: 1 package finished" - BUILD SUCCESSFUL! 🎉**

---

## Step 5: Test the Solver

```bash
# Source the workspace
source install/setup.bash

# Verify packages are available
ros2 pkg list | grep gik9dof
# Should show:
# gik9dof_msgs
# gik9dof_solver

# Run the solver node
ros2 run gik9dof_solver gik9dof_solver_node
```

**Expected Output:**
```
[INFO] [timestamp]: Solver node starting...
[INFO] [timestamp]: Loading robot model...
[INFO] [timestamp]: MATLAB solver initialized successfully
[INFO] [timestamp]: Robot model: 9 DOF mobile manipulator
[INFO] [timestamp]: Subscribed to /hdas/feedback_arm_left
[INFO] [timestamp]: Subscribed to /odom_wheel
[INFO] [timestamp]: Subscribed to /gik9dof/target_trajectory
[INFO] [timestamp]: Publishing to /gik9dof/command_vel
[INFO] [timestamp]: Publishing to /gik9dof/solver_diagnostics
[INFO] [timestamp]: Control loop running at 10 Hz
[INFO] [timestamp]: Waiting for trajectory commands...
```

**✅ If you see "Waiting for trajectory commands..." - IT'S WORKING! 🚀**

**Keep this terminal open** (solver is running)

---

## Step 6: Validate Topics (New Terminal)

Open a **second SSH session** to the Orin:

```bash
# Terminal 2
ssh cr@192.168.100.150
cd ~/gikWBC9DOF/ros2
source install/setup.bash

# List all topics
ros2 topic list
```

**Should see:**
```
/gik9dof/command_vel
/gik9dof/solver_diagnostics
/gik9dof/target_trajectory
/hdas/feedback_arm_left
/odom_wheel
```

**✅ All 5 topics created!**

---

## Step 7: Send Test Trajectory

Still in Terminal 2:

```bash
# Send a simple test trajectory
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
[INFO]: Trajectory callback triggered
[INFO]: Solving IK for waypoint 0...
[INFO]: Target position: [0.500, 0.000, 0.300]
[INFO]: Solution found! Iterations: XX, Time: XX.X ms
[INFO]: Publishing command velocities
```

**✅ If you see "Solution found!" - FULL SUCCESS! 🎉🎉🎉**

---

## Step 8: Monitor Diagnostics

In Terminal 2:

```bash
# Monitor solver diagnostics
ros2 topic echo /gik9dof/solver_diagnostics --once
```

**Expected output:**
```
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ''
status: 1                    # 1 = success ✓
iterations: 10-50            # Depends on complexity
solve_time_ms: 5.0-30.0     # Should be < 50ms ✓
error: 0.001                 # Should be < 0.01 ✓
joint_positions: [...]       # Solution joint angles
command_velocities: [...]    # Output commands
---
```

**✅ If status=1 and solve_time_ms < 50 - EXCELLENT PERFORMANCE!**

---

## 🎉 Success Checklist

Check all that apply:

- [ ] ✅ Dependencies installed (Eigen3, OpenMP, ROS2)
- [ ] ✅ `gik9dof_msgs` built successfully (~10s)
- [ ] ✅ `gik9dof_solver` built successfully (~50s)
- [ ] ✅ Solver node starts without errors
- [ ] ✅ All 5 topics created
- [ ] ✅ Test trajectory message received
- [ ] ✅ Solver computes solution
- [ ] ✅ Diagnostics show status=1
- [ ] ✅ Solve time < 50ms

**If all checked - MISSION ACCOMPLISHED! 🚀**

Your MATLAB-generated C++ solver is now running natively on AGX Orin!

---

## 🐛 Troubleshooting

### Build Error: "Could not find package Eigen3"
```bash
sudo apt install libeigen3-dev
pkg-config --modversion eigen3
```

### Build Error: "OpenMP not found"
```bash
sudo apt install libomp-dev
```

### Build Error: "No such file or directory: matlab_codegen"
```bash
# Check if files are there
ls -la ~/gikWBC9DOF/ros2/gik9dof_solver/matlab_codegen/

# If empty, re-transfer from Windows
```

### Runtime: "Cannot find URDF file"
```bash
# Check URDF exists
ls -la ~/gikWBC9DOF/*.urdf

# Update path in solver code if needed
```

### No messages received
```bash
# Check QoS settings
ros2 topic info /gik9dof/target_trajectory -v
```

---

## 🔧 Performance Tuning

Once running, you can monitor performance:

```bash
# CPU usage
htop

# Memory usage
free -h

# Network stats
ros2 topic hz /gik9dof/solver_diagnostics
# Should show: ~10 Hz

# Detailed timing
ros2 topic echo /gik9dof/solver_diagnostics | grep solve_time_ms
```

---

## 📚 What's Next?

1. **Integrate with real robot** - Connect to actual hardware topics
2. **Tune parameters** - Adjust solver constraints if needed
3. **Add logging** - Record data for analysis
4. **Performance testing** - Stress test with complex trajectories

---

## 🆘 Need Help?

If you encounter any issues:

1. Check build logs: `cat ~/gikWBC9DOF/ros2/log/latest_build/gik9dof_solver/stdout_stderr.log`
2. Check validation docs: See `validation/VALIDATION_STATUS.md`
3. Check deployment guide: See `docs/deployment/ORIN_DEPLOYMENT_GUIDE.md`

---

**Ready to build? Copy the commands from Step 1!** 🚀
