# WSL Ubuntu 22.04 Validation Guide

**Purpose**: Use WSL as an intermediate validation environment to catch build issues, dependency problems, and ROS2 integration bugs **before** deploying to AGX Orin.

---

## Why This Matters

**Risk Reduction Strategy**:
- ✅ **Same OS**: Ubuntu 22.04 on both WSL and AGX Orin → identical package versions
- ✅ **Same ROS2**: Humble on both → identical message formats, colcon behavior
- ✅ **Build Validation**: Catch CMake errors, missing dependencies, linking issues on WSL
- ✅ **ROS2 Testing**: Validate node startup, topic communication, message serialization
- ✅ **Fast Iteration**: No need to SCP to AGX Orin for every test

**What WSL Won't Catch**:
- ⚠️ ARM64-specific runtime issues (WSL is x86_64, Orin is ARM64)
- ⚠️ Performance differences (WSL has different CPU, no NEON SIMD)
- ⚠️ Hardware-specific issues (CAN bus, GPIO, etc.)

**Bottom Line**: WSL validates 80% of integration before touching the robot. Worth it!

---

## Prerequisites

Verify WSL environment (run in WSL terminal):

```bash
# Check OS version
lsb_release -a
# Expected: Ubuntu 22.04.x LTS

# Check ROS2 version
echo $ROS_DISTRO
# Expected: humble

# Check colcon
which colcon
# Expected: /usr/bin/colcon (or similar)

# Check required packages
dpkg -l | grep -E "eigen3|omp|yaml-cpp"
# Expected: libeigen3-dev, libomp-dev, libyaml-cpp-dev
```

**If packages missing**, install:
```bash
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

## Workflow: Windows → WSL → AGX Orin

### Step 1: Generate Code on Windows (MATLAB)

```matlab
% In MATLAB R2024b on Windows:
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
RUN_VALIDATION  % Must pass first!
RUN_CODEGEN     % Generates gik9dof_deployment_<timestamp>.zip
```

**Output**: `gik9dof_deployment_20251006_143022.zip` (timestamp varies)

### Step 2: Copy to WSL

**From Windows PowerShell** (easier than WSL→Windows file access):

```powershell
# Find the latest deployment ZIP
$latestZip = Get-ChildItem -Path . -Filter "gik9dof_deployment_*.zip" | Sort-Object LastWriteTime -Descending | Select-Object -First 1

# Copy to WSL home directory
wsl cp "/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/$($latestZip.Name)" ~/gik9dof_deployment.zip
```

**Or manually** (if above fails):
```powershell
# From Windows, copy to your WSL home:
wsl mkdir -p ~/gik9dof_test
Copy-Item gik9dof_deployment_*.zip \\wsl.localhost\Ubuntu-22.04\home\<your-username>\gik9dof_test\
```

### Step 3: Extract and Build on WSL

**In WSL terminal**:

```bash
# Navigate to test directory
cd ~/gik9dof_test

# Extract deployment package
unzip gik9dof_deployment.zip
# Creates: gik9dof_deployment/gik9dof_msgs/, gik9dof_deployment/gik9dof_solver/

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build messages first (dependency for solver)
cd ~/gik9dof_test/gik9dof_deployment
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Build solver node
colcon build --packages-select gik9dof_solver

# Check for build errors
echo $?  # Should be 0
```

**Expected Build Output**:
```
Starting >>> gik9dof_msgs
Finished <<< gik9dof_msgs [10.2s]
Starting >>> gik9dof_solver
Finished <<< gik9dof_solver [45.3s]

Summary: 2 packages finished [55.6s]
```

### Step 4: Inspect Generated Code Integration

**Check if MATLAB-generated C++ is properly linked**:

```bash
# Look for generated code in solver package
ls -lh ~/gik9dof_test/gik9dof_deployment/gik9dof_solver/src/generated/

# Expected files (if code generation worked):
# - buildRobotForCodegen.cpp/h
# - solveGIKStepWrapper.cpp/h
# - generalizedInverseKinematics.cpp/h
# - (50+ other generated files)

# Check if solver node compiled
ls -lh ~/gik9dof_test/gik9dof_deployment/install/gik9dof_solver/lib/gik9dof_solver/
# Expected: gik9dof_solver_node (executable)
```

**🚨 CRITICAL CHECK**: If `src/generated/` is empty, code generation failed! Go back to Windows and check `codegen/html/report.mldatx` for errors.

### Step 5: Test ROS2 Node Startup

**Source workspace and launch solver**:

```bash
cd ~/gik9dof_test/gik9dof_deployment
source install/setup.bash

# Launch solver node (should start without errors)
ros2 launch gik9dof_solver test_solver.launch.py
```

**Expected Output**:
```
[gik9dof_solver_node]: Waiting for initial robot state...
[gik9dof_solver_node]: Waiting for initial odometry...
[gik9dof_solver_node]: Waiting for trajectory...
[gik9dof_solver_node]: Control loop running at 10 Hz
```

**If node crashes**, check:
- Missing shared libraries: `ldd install/gik9dof_solver/lib/gik9dof_solver/gik9dof_solver_node`
- Missing dependencies: Install libeigen3-dev, libomp-dev
- Segmentation fault: Check generated code arrays match robot DOF (should be 9)

### Step 6: Test with Mock Inputs

**Open new WSL terminal** (keep solver running in first terminal):

```bash
cd ~/gik9dof_test/gik9dof_deployment
source install/setup.bash

# Run mock input publisher
ros2 run gik9dof_solver test_mock_inputs.py
```

**Expected Output (from solver terminal)**:
```
[gik9dof_solver_node]: Received initial robot state (6 joints)
[gik9dof_solver_node]: Received initial odometry
[gik9dof_solver_node]: Received trajectory (5 waypoints)
[gik9dof_solver_node]: Solver diagnostics: status=1, iterations=12, solve_time=3.4ms
```

**Monitor diagnostics**:

```bash
# In third terminal:
source install/setup.bash
ros2 topic echo /gik9dof/solver_diagnostics
```

**Success Criteria**:
- ✅ `status: 1` (success, not 0=failure or 2=warning)
- ✅ `iterations < 100` (solver converges quickly)
- ✅ `solve_time_ms < 50.0` (meets 10 Hz requirement: 100ms budget, 50ms for solver)
- ✅ `distance_to_goal < 0.05` (within 5cm of target)

**Failure Modes to Check**:
- ❌ `status: 0` → Solver diverged, check constraints or initial guess
- ❌ `solve_time_ms > 100` → Performance issue, check x86_64 vs ARM64 difference
- ❌ No diagnostics published → Node crashed, check `ros2 node list`

### Step 7: Verify Joint Commands Published

```bash
# Check if solver is publishing commands
ros2 topic hz /motion_target/target_joint_state_arm_left
# Expected: average rate: 10.000 Hz

ros2 topic echo /motion_target/target_joint_state_arm_left --once
# Expected: 6 joint positions (arm joints only, not base)
```

---

## Common WSL Build Issues

### Issue 1: `colcon: command not found`

**Fix**:
```bash
sudo apt install python3-colcon-common-extensions
```

### Issue 2: `fatal error: Eigen/Dense: No such file or directory`

**Fix**:
```bash
sudo apt install libeigen3-dev
# Verify: ls /usr/include/eigen3/Eigen/Dense
```

### Issue 3: `ld: cannot find -lomp`

**Fix**:
```bash
sudo apt install libomp-dev
```

### Issue 4: Custom messages not found (`gik9dof_msgs/msg/*.hpp`)

**Fix**: Build messages package first, then source:
```bash
colcon build --packages-select gik9dof_msgs
source install/setup.bash  # Critical!
colcon build --packages-select gik9dof_solver
```

### Issue 5: Generated code missing (`src/generated/` is empty)

**Root Cause**: MATLAB code generation failed on Windows.

**Fix**:
1. Go back to Windows MATLAB
2. Check `codegen/html/report.mldatx` for errors
3. Common issues:
   - Unsupported Robotics Toolbox functions → Rewrite in pure math
   - Dynamic memory allocation → Disable in codegen config
   - Variable-size arrays → Set explicit bounds

### Issue 6: Node starts but no topics published

**Debug**:
```bash
# Check if node is running
ros2 node list
# Expected: /gik9dof_solver_node

# Check declared topics
ros2 node info /gik9dof_solver_node
# Expected publishers: /motion_target/target_joint_state_arm_left, /gik9dof/solver_diagnostics

# Check for errors in node output
# Look for: "Waiting for initial robot state..." → Need to publish joint states
```

---

## Validation Checklist (WSL Sign-Off)

Before deploying to AGX Orin, confirm:

- [ ] **Build Clean**: `colcon build` exits with code 0, no warnings
- [ ] **Packages Built**: `install/gik9dof_msgs/` and `install/gik9dof_solver/` exist
- [ ] **Generated Code Exists**: `gik9dof_solver/src/generated/*.cpp` not empty
- [ ] **Node Starts**: `ros2 launch gik9dof_solver test_solver.launch.py` no crash
- [ ] **Topics Published**: `ros2 topic list` shows `/gik9dof/solver_diagnostics`
- [ ] **Mock Inputs Work**: test_mock_inputs.py publishes, solver receives data
- [ ] **Solver Converges**: `status=1`, `solve_time_ms < 50`, `iterations < 100`
- [ ] **Joint Commands**: `/motion_target/target_joint_state_arm_left` publishes at 10 Hz

**If all checkboxes passed**: ✅ Ready for AGX Orin deployment!

**If any failed**: ❌ Fix on WSL first. Do NOT deploy broken code to robot.

---

## Next Steps After WSL Validation

Once WSL tests pass:

1. **Package for AGX Orin**: Already done (gik9dof_deployment.zip)
2. **Deploy**: Run `.\deploy_to_orin.ps1 <orin-ip>` from Windows PowerShell
3. **Build on Orin**: Same `colcon build` commands (should work identically)
4. **Test on Orin**: Same test_solver.launch.py (verify ARM64 performance)
5. **Integrate**: Replace mock inputs with real robot topics

---

## Performance Expectations: WSL vs AGX Orin

| Metric | WSL (x86_64) | AGX Orin (ARM64) | Notes |
|--------|--------------|------------------|-------|
| Solve Time | 5-20 ms | 3-15 ms | Orin has 12-core ARM Cortex-A78AE, should be faster |
| Build Time | 1-2 min | 2-5 min | Orin has slower clock speed but more cores |
| Memory Usage | ~50 MB | ~50 MB | Should be identical (same code) |
| Max Frequency | 20-30 Hz | 50+ Hz | Orin optimized for real-time (NEON SIMD, OpenMP) |

**Note**: WSL solve time may be **slower** than Orin because:
- No ARM NEON SIMD vectorization (x86_64 uses SSE/AVX instead)
- Virtual machine overhead
- Different CPU architecture

**Don't panic if WSL is slower!** Focus on:
- ✅ Build succeeds
- ✅ Solver converges (`status=1`)
- ✅ No crashes or segfaults

Performance will improve on Orin with ARM64 native code.

---

## Quick Reference: WSL Commands

```bash
# Build everything
cd ~/gik9dof_test/gik9dof_deployment
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Launch solver
ros2 launch gik9dof_solver test_solver.launch.py

# (New terminal) Run mock inputs
source install/setup.bash
ros2 run gik9dof_solver test_mock_inputs.py

# (New terminal) Monitor diagnostics
source install/setup.bash
ros2 topic echo /gik9dof/solver_diagnostics

# Check topic rates
ros2 topic hz /motion_target/target_joint_state_arm_left
```

---

## Troubleshooting: WSL-Specific Issues

### WSL Can't Access Windows Files

If `wsl cp` fails:

**Option 1**: Use Windows file system from WSL
```bash
# Windows C:\ is mounted at /mnt/c/
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
cp gik9dof_deployment_*.zip ~/gik9dof_test/
```

**Option 2**: Use Windows Explorer
1. Open Windows Explorer
2. Navigate to: `\\wsl.localhost\Ubuntu-22.04\home\<username>\gik9dof_test`
3. Drag and drop the ZIP file

### ROS2 Not Sourced

Every new terminal needs:
```bash
source /opt/ros/humble/setup.bash
source ~/gik9dof_test/gik9dof_deployment/install/setup.bash
```

**Or** add to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Permissions Error on Executables

```bash
# Make node executable
chmod +x ~/gik9dof_test/gik9dof_deployment/install/gik9dof_solver/lib/gik9dof_solver/gik9dof_solver_node
```

---

## Summary: WSL as Safety Net

**What WSL Validates**:
- ✅ ROS2 package structure (CMakeLists.txt, package.xml)
- ✅ Message definitions (EndEffectorTrajectory, SolverDiagnostics)
- ✅ Node compilation (C++17, ROS2 Humble API)
- ✅ Dependencies (Eigen, OpenMP, YAML-CPP)
- ✅ Topic communication (pub/sub, serialization)
- ✅ Node lifecycle (startup, shutdown, error handling)

**What WSL Doesn't Validate**:
- ⚠️ ARM64 NEON performance
- ⚠️ Real robot sensor data (CAN bus, encoders)
- ⚠️ Real-time scheduling (PREEMPT_RT kernel on Orin)
- ⚠️ Hardware-specific issues

**Bottom Line**: WSL catches 80% of integration bugs. Use it!

---

## Success Path

1. ✅ MATLAB validation passes on Windows
2. ✅ MATLAB code generation succeeds on Windows
3. ✅ WSL build succeeds (this guide)
4. ✅ WSL solver node starts without crash
5. ✅ WSL mock tests show `status=1`, solve_time < 50ms
6. ✅ Deploy to AGX Orin with confidence
7. ✅ Orin build succeeds (should be identical to WSL)
8. ✅ Orin tests pass (better performance than WSL)
9. ✅ Integrate with real robot
10. ✅ Ship it! 🚀

You're now using a **3-tier validation strategy**:
- **Tier 1 (Windows MATLAB)**: Algorithm correctness, code generation feasibility
- **Tier 2 (WSL Ubuntu)**: ROS2 integration, build system, dependencies
- **Tier 3 (AGX Orin)**: ARM64 performance, real robot integration

This is **best practice** for robotics software deployment. Well done!
