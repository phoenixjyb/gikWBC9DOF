# WSL Quick Reference - Copy/Paste Commands

**Purpose**: Fast reference for WSL Ubuntu 22.04 validation workflow.  
**Full Guide**: See `WSL_VALIDATION_GUIDE.md` for troubleshooting and explanations.

---

## Prerequisites Check (Run Once)

```bash
# Check environment
lsb_release -a      # Should show Ubuntu 22.04
echo $ROS_DISTRO    # Should show "humble"
which colcon        # Should find colcon

# Install missing packages (if needed)
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  libeigen3-dev \
  libomp-dev \
  libyaml-cpp-dev \
  build-essential
```

---

## Step 1: Copy Deployment Package to WSL

**From Windows PowerShell**:

```powershell
# Navigate to project
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

# Find and copy latest deployment ZIP
$latestZip = Get-ChildItem -Filter "gik9dof_deployment_*.zip" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
wsl cp "/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/$($latestZip.Name)" ~/gik9dof_deployment.zip
```

**Alternative (if above fails)** - From WSL:

```bash
cp /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/gik9dof_deployment_*.zip ~/
```

---

## Step 2: Build on WSL (ONE TIME)

```bash
# Create workspace and extract
cd ~
mkdir -p gik9dof_test
cd gik9dof_test
unzip ~/gik9dof_deployment.zip

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build messages package (dependency)
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Verify messages compiled
ros2 interface show gik9dof_msgs/msg/EndEffectorTrajectory
ros2 interface show gik9dof_msgs/msg/SolverDiagnostics

# Build solver package
colcon build --packages-select gik9dof_solver
source install/setup.bash

# Verify node compiled
ls -lh install/gik9dof_solver/lib/gik9dof_solver/gik9dof_solver_node
```

**Expected output**: `Finished <<< gik9dof_solver [XX.Xs]` with no errors.

---

## Step 3: Test with Mock Inputs (EVERY TIME)

### Terminal 1: Launch Solver Node

```bash
cd ~/gik9dof_test/gik9dof_deployment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch solver (keeps running)
ros2 launch gik9dof_solver test_solver.launch.py
```

**Expected output**:
```
[gik9dof_solver_node]: Waiting for initial robot state...
[gik9dof_solver_node]: Waiting for initial odometry...
[gik9dof_solver_node]: Control loop running at 10 Hz
```

### Terminal 2: Run Mock Input Publisher

**Open new WSL terminal**, then:

```bash
cd ~/gik9dof_test/gik9dof_deployment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Publish mock data (keeps running)
ros2 run gik9dof_solver test_mock_inputs.py
```

**Expected output** (in Terminal 1 - solver window):
```
[gik9dof_solver_node]: Received initial robot state (6 joints)
[gik9dof_solver_node]: Received initial odometry
[gik9dof_solver_node]: Received trajectory (5 waypoints)
[gik9dof_solver_node]: Solver diagnostics: status=1, iterations=12, solve_time=3.4ms
```

### Terminal 3 (Optional): Monitor Diagnostics

**Open third WSL terminal**:

```bash
cd ~/gik9dof_test/gik9dof_deployment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Watch diagnostics in real-time
ros2 topic echo /gik9dof/solver_diagnostics
```

**Expected output** (every 100ms):
```yaml
status: 1
num_iterations: 12
solve_time_ms: 3.4
distance_to_goal: 0.0023
constraint_violation: 0.0
---
```

---

## Success Criteria Checklist

- [ ] **Build succeeds**: `colcon build` exits with code 0
- [ ] **Solver starts**: `ros2 launch` runs without crash
- [ ] **Mock data flows**: Terminal 1 shows "Received initial robot state"
- [ ] **Solver converges**: `status: 1` (not 0 or 2)
- [ ] **Fast enough**: `solve_time_ms < 50.0` (10 Hz requirement = 100ms budget)
- [ ] **Iterations OK**: `num_iterations < 100` (solver converges quickly)
- [ ] **Accurate**: `distance_to_goal < 0.05` (within 5cm)
- [ ] **Joint commands publish**: `ros2 topic hz /motion_target/target_joint_state_arm_left` shows ~10 Hz

**If all checked**: ✅ WSL validation PASSED - ready for AGX Orin deployment!

**If any failed**: ❌ Check `WSL_VALIDATION_GUIDE.md` for troubleshooting. Fix on WSL first!

---

## Quick Diagnostics Commands

```bash
# Check if solver node is running
ros2 node list
# Expected: /gik9dof_solver_node

# List all topics
ros2 topic list
# Expected: /gik9dof/solver_diagnostics, /motion_target/target_joint_state_arm_left

# Check topic rates
ros2 topic hz /gik9dof/solver_diagnostics          # Should be ~10 Hz
ros2 topic hz /motion_target/target_joint_state_arm_left  # Should be ~10 Hz

# Echo single message
ros2 topic echo /motion_target/target_joint_state_arm_left --once

# Check for errors in solver node
# (Look at Terminal 1 output for stack traces or warnings)
```

---

## Common Issues - Quick Fixes

### Issue: `colcon: command not found`
```bash
sudo apt install python3-colcon-common-extensions
```

### Issue: `fatal error: Eigen/Dense: No such file`
```bash
sudo apt install libeigen3-dev
```

### Issue: `ld: cannot find -lomp`
```bash
sudo apt install libomp-dev
```

### Issue: Custom messages not found
```bash
# Build messages first, THEN source before building solver
colcon build --packages-select gik9dof_msgs
source install/setup.bash  # CRITICAL!
colcon build --packages-select gik9dof_solver
```

### Issue: Generated code missing (`src/generated/` empty)
**Root cause**: MATLAB code generation failed on Windows.
- Go back to Windows
- Check `codegen/html/report.mldatx` for errors
- Fix MATLAB code, re-run `RUN_CODEGEN.m`
- Copy new deployment package to WSL

### Issue: Solver node crashes on startup
```bash
# Check missing libraries
ldd install/gik9dof_solver/lib/gik9dof_solver/gik9dof_solver_node
# Install any missing dependencies with apt
```

### Issue: Mock publisher fails with "No module named 'rclpy'"
```bash
# Make sure to source workspace BEFORE running Python script
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gik9dof_solver test_mock_inputs.py
```

---

## Rebuild After Code Changes (Quick)

If you regenerate code on Windows and copy new deployment package:

```bash
cd ~/gik9dof_test
rm -rf gik9dof_deployment  # Clean old build
unzip ~/gik9dof_deployment.zip  # Extract new package

source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_msgs
source install/setup.bash
colcon build --packages-select gik9dof_solver
source install/setup.bash

# Test again (Terminal 1 + Terminal 2 commands above)
```

---

## Performance Expectations

| Metric | WSL (x86_64) | AGX Orin (ARM64) | Notes |
|--------|--------------|------------------|-------|
| Solve Time | 5-20 ms | 3-15 ms | Orin faster (ARM NEON, 12 cores) |
| Max Frequency | 20-30 Hz | 50+ Hz | Orin has real-time optimizations |
| Build Time | 1-2 min | 2-5 min | Orin slower clock, more cores |

**Don't panic if WSL is slower!** Focus on: ✅ Build succeeds, ✅ Solver converges, ✅ No crashes.

---

## Next Step: Deploy to AGX Orin

Once WSL validation passes, deploy to robot:

**From Windows PowerShell**:
```powershell
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
.\deploy_to_orin.ps1 <orin-ip>
```

**Example**:
```powershell
.\deploy_to_orin.ps1 192.168.1.100
```

Build on Orin will be **identical** to WSL (same OS, same ROS2, same dependencies). Should "just work"!

---

## Pro Tips

1. **Add to `.bashrc`** (skip sourcing every time):
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

2. **Check logs if node crashes**:
   ```bash
   ros2 run gik9dof_solver gik9dof_solver_node --ros-args --log-level debug
   ```

3. **Clean build if strange errors**:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```

4. **Test single package rebuild**:
   ```bash
   colcon build --packages-select gik9dof_solver --cmake-clean-cache
   ```

---

**Full Documentation**: `WSL_VALIDATION_GUIDE.md`  
**Troubleshooting**: `FAST_TRACK_2DAY.md`  
**Execution Guide**: `START_HERE.md`
