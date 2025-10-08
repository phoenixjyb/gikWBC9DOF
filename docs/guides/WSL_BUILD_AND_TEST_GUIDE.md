# WSL Build and Test Guide - 20-Constraint GIK Solver

**Purpose**: Build and test the updated ROS2 package in WSL before Orin deployment  
**Prerequisites**: All code changes complete, 158 files copied to ROS2 package  
**Estimated Time**: 15-30 minutes

---

## Quick Start

```bash
# 1. Navigate to workspace
cd ~/gikWBC9DOF/ros2  # Adjust path as needed

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. Clean previous build (recommended after major changes)
rm -rf build/ install/ log/

# 4. Build
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Source workspace
source install/setup.bash

# 6. Launch node
ros2 run gik9dof_solver gik9dof_solver_node --ros-args \
    --params-file src/gik9dof_solver/config/gik_solver_params.yaml
```

---

## Detailed Build Process

### Step 1: Access WSL

**From Windows PowerShell**:
```powershell
wsl
```

**From Windows Terminal**:
- Open Windows Terminal
- Click dropdown → Select your Ubuntu/WSL distribution

### Step 2: Navigate to Workspace

Assuming you copied the ROS2 folder to WSL:
```bash
# If in Windows: C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\ros2
# In WSL it's: /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2

cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
```

**OR** if you copied to WSL home directory:
```bash
cd ~/gikWBC9DOF/ros2
```

### Step 3: Verify File Structure

```bash
# Check package exists
ls src/gik9dof_solver/

# Expected output:
# CMakeLists.txt  config/  include/  matlab_codegen/  package.xml  src/

# Check generated code
ls src/gik9dof_solver/matlab_codegen/include/ | wc -l
# Expected: 158 files

# Check parameter file exists
ls src/gik9dof_solver/config/gik_solver_params.yaml
# Expected: file exists
```

### Step 4: Source ROS2

```bash
source /opt/ros/humble/setup.bash

# Verify ROS2 is sourced
echo $ROS_DISTRO
# Expected: humble
```

### Step 5: Clean Previous Build (Recommended)

If this is a rebuild after major code changes:
```bash
rm -rf build/ install/ log/
```

### Step 6: Build Package

```bash
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Expected Output**:
```
Starting >>> gik9dof_solver
[Processing: gik9dof_solver]
Finished <<< gik9dof_solver [XX.Xs]

Summary: 1 package finished [XX.Xs]
```

**Build Time Estimate**: 30-60 seconds (158 C++ files)

---

## Build Troubleshooting

### Problem 1: Missing Header Files

**Error**:
```
fatal error: gik9dof_codegen_inuse_solveGIKStepWrapper.h: No such file or directory
```

**Diagnosis**:
```bash
# Check if header exists
find src/gik9dof_solver/matlab_codegen/ -name "gik9dof_codegen_inuse_solveGIKStepWrapper.h"
```

**Solution**:
```bash
# Re-run copy script from Windows
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
powershell.exe -File copy_codegen_to_ros2.ps1 -Force
```

### Problem 2: Namespace Not Found

**Error**:
```
error: 'codegen_inuse' is not a namespace-name
```

**Diagnosis**: Wrong generated code version

**Solution**: Verify ARM64 code is being used:
```bash
ls src/gik9dof_solver/matlab_codegen/include/gik9dof_codegen_inuse_solveGIKStepWrapper.h
# File should exist

grep -r "namespace codegen_inuse" src/gik9dof_solver/matlab_codegen/include/
# Should find namespace declarations
```

### Problem 3: Undefined Reference

**Error**:
```
undefined reference to `gik9dof::codegen_inuse::solveGIKStepWrapper(...)`
```

**Diagnosis**: Generated .cpp files not compiled

**Solution**: Verify CMakeLists.txt:
```bash
cat src/gik9dof_solver/CMakeLists.txt | grep "GLOB MATLAB_SOURCES"
# Should include: "${MATLAB_SOLVER_DIR}/*.cpp" AND "${MATLAB_SOLVER_DIR}/*.c"
```

### Problem 4: OpenMP Linking Error

**Error**:
```
undefined reference to `GOMP_*`
```

**Solution**: Install OpenMP:
```bash
sudo apt-get update
sudo apt-get install libomp-dev
```

---

## Testing After Build

### Step 1: Source Workspace

```bash
source install/setup.bash
```

### Step 2: Launch Node (Terminal 1)

**With parameter file**:
```bash
ros2 run gik9dof_solver gik9dof_solver_node --ros-args \
    --params-file src/gik9dof_solver/config/gik_solver_params.yaml
```

**Expected Output**:
```
[INFO] [<timestamp>] [gik9dof_solver_node]: GIK9DOF Solver Node initialized
[INFO] [<timestamp>] [gik9dof_solver_node]: MATLAB solver interface: codegen_inuse (20 constraints)
[INFO] [<timestamp>] [gik9dof_solver_node]: Control rate: 10.0 Hz
[INFO] [<timestamp>] [gik9dof_solver_node]: Max solve time: 50 ms
```

**Success Indicators**:
- ✅ Node starts without crashing
- ✅ Log shows "codegen_inuse (20 constraints)"
- ✅ No parameter loading errors

### Step 3: Check Topics (Terminal 2)

```bash
# Open new terminal
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
source install/setup.bash

# List topics
ros2 topic list
```

**Expected Topics**:
```
/gik9dof/diagnostics
/gik9dof/joint_state_command
/gik9dof/trajectory_command
/hdas/feedback_arm_left
...
```

### Step 4: Monitor Diagnostics (Terminal 2)

```bash
ros2 topic echo /gik9dof/diagnostics
```

**Expected**: Initially no messages (waiting for goal)

### Step 5: Send Test Goal (Terminal 3)

**Option A: Simple position command** (test if solver runs):
```bash
# Open new terminal, source workspace
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
source install/setup.bash

# Publish test goal
ros2 topic pub --once /gik9dof/trajectory_command geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Expected**:
- ✅ Node processes goal
- ✅ Solver executes
- ✅ Diagnostics message published
- ✅ No crashes

### Step 6: Validate Performance

Check diagnostics output:
```bash
ros2 topic echo /gik9dof/diagnostics --once
```

**Expected Fields**:
```yaml
solve_time_ms: 15.0-20.0  # Should be ~15-20ms with 5 constraints
exit_flag: 1              # Success
iterations: <number>      # Typically 5-15
pose_violation: <value>   # Should be small (<0.001)
...
```

**Success Criteria**:
- ✅ solve_time_ms < 50ms
- ✅ exit_flag = 1 (success) or 3 (search dir limit, still valid)
- ✅ No crashes or errors
- ✅ pose_violation is small

---

## Advanced Testing

### Test 1: Disable All Constraints (Baseline)

Create test parameter file:
```bash
cd src/gik9dof_solver/config
cp gik_solver_params.yaml test_no_constraints.yaml
```

Edit `test_no_constraints.yaml`:
```yaml
dist_weights: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Launch with test config:
```bash
ros2 run gik9dof_solver gik9dof_solver_node --ros-args \
    --params-file src/gik9dof_solver/config/test_no_constraints.yaml
```

Send same test goal, check diagnostics:
- **Expected**: Faster solve time (~10-15ms)

### Test 2: Enable All 20 Constraints (Maximum Load)

Create test parameter file:
```bash
cd src/gik9dof_solver/config
cp gik_solver_params.yaml test_all_constraints.yaml
```

Edit `test_all_constraints.yaml`:
```yaml
# Configure 20 meaningful constraints
dist_body_indices: [9, 9, 9, 9, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 8, 8]
dist_ref_body_indices: [0, 1, 2, 3, 0, 1, 2, 0, 1, 2, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
dist_weights: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

Launch:
```bash
ros2 run gik9dof_solver gik9dof_solver_node --ros-args \
    --params-file src/gik9dof_solver/config/test_all_constraints.yaml
```

Send test goal, check diagnostics:
- **Expected**: Slower solve time (~30-40ms, still < 50ms timeout)

### Test 3: Match MATLAB Test Configuration

Use default config (already matches MATLAB Test 1):
- 5 active constraints
- Expected ~15ms solve time

Compare with MATLAB results from `test_gik_20constraints.m`.

---

## Performance Benchmarks

Based on WSL standalone C++ tests:

| Configuration         | Expected Solve Time | Status      |
|-----------------------|---------------------|-------------|
| All disabled (0/20)   | ~10-15ms            | Baseline    |
| 1 active (1/20)       | ~10-15ms            | Minimal     |
| 5 active (5/20)       | ~15-20ms            | **DEFAULT** |
| 10 active (10/20)     | ~20-30ms            | Moderate    |
| 20 active (20/20)     | ~30-40ms            | Maximum     |

**Timeout**: 50ms (configured in MATLAB code)  
**Target**: < 50ms for real-time control (10 Hz)

---

## Validation Checklist

### Build Phase ✅
- [ ] WSL environment ready
- [ ] ROS2 Humble sourced
- [ ] Package builds without errors
- [ ] All 158 C++ files compiled
- [ ] No linker errors
- [ ] Executable created

### Launch Phase ✅
- [ ] Node launches without crashes
- [ ] Parameter file loads successfully
- [ ] Log shows "codegen_inuse (20 constraints)"
- [ ] Topics advertised correctly
- [ ] No error messages

### Execution Phase ✅
- [ ] Accepts goal pose commands
- [ ] Solver executes (exit_flag = 1 or 3)
- [ ] Solve time within expected range
- [ ] Diagnostics published correctly
- [ ] No memory errors or crashes

### Performance Phase ✅
- [ ] Default config (5 constraints): ~15-20ms
- [ ] All disabled (0 constraints): ~10-15ms
- [ ] All enabled (20 constraints): ~30-40ms
- [ ] Performance matches C++ standalone tests
- [ ] No timeouts or solver failures

---

## Next Steps After Successful WSL Testing

### 1. Create Deployment Package

```bash
# Build release version
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Package for Orin deployment
cd install/gik9dof_solver
tar -czf gik9dof_solver_arm64_$(date +%Y%m%d).tar.gz lib/ share/
```

### 2. Transfer to AGX Orin

```bash
# From WSL
scp gik9dof_solver_arm64_*.tar.gz nvidia@<ORIN_IP>:~/
```

### 3. Deploy on Orin

```bash
# On Orin
tar -xzf gik9dof_solver_arm64_*.tar.gz
# Copy to ROS2 workspace
# Test with same parameter file
```

### 4. Validate on Hardware

- Run same tests as WSL
- Verify real-time performance
- Test with actual robot motion
- Validate constraint behavior

---

## Common Issues and Solutions

### Issue: "Permission Denied" when accessing /mnt/c/

**Solution**:
```bash
sudo chmod -R 755 /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
```

### Issue: ROS2 not found

**Solution**:
```bash
# Install ROS2 Humble in WSL
sudo apt update
sudo apt install ros-humble-desktop
```

### Issue: Slow build in /mnt/c/

**Solution**: Copy to WSL filesystem for faster builds:
```bash
cp -r /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2 ~/gikWBC9DOF/
cd ~/gikWBC9DOF/ros2
colcon build
```

---

## Summary

**Current Status**: ✅ All code changes complete, ready to build

**Next Actions**:
1. Open WSL terminal
2. Navigate to ROS2 workspace
3. Run build command
4. Launch node with parameter file
5. Send test goal pose
6. Validate solve time and performance

**Success Criteria**:
- ✅ Clean build (no errors)
- ✅ Node launches successfully
- ✅ Solver executes on goals
- ✅ Performance ~15-20ms (5 constraints)
- ✅ No crashes or memory errors

**Estimated Time**: 15-30 minutes

---

**Ready to proceed?** Run the Quick Start commands above! 🚀
