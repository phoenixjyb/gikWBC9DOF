# Phase 3: Running C++ ARM64 Solver Test on AGX Orin

## Quick Start

### Prerequisites
- ✅ MATLAB reference results transferred (`matlab_reference_results.json`)
- ✅ Test script transferred (`run_cpp_test_arm64.py`)
- ⏳ C++ solver node must be running

### Step 1: SSH to Orin

```bash
ssh cr@192.168.100.150
cd ~/gikWBC9DOF
```

### Step 2: Start the C++ Solver Node

**Terminal 1** (on Orin):
```bash
source /opt/ros/humble/setup.bash
source ~/gikWBC9DOF/ros2/install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

Expected output:
```
[INFO] [gik9dof_solver]: GIK 9-DOF Solver Node initialized
[INFO] [gik9dof_solver]: Waiting for target poses...
```

### Step 3: Run the Validation Test

**Terminal 2** (on Orin, new SSH session):
```bash
cd ~/gikWBC9DOF/validation
source /opt/ros/humble/setup.bash
source ~/gikWBC9DOF/ros2/install/setup.bash
python3 run_cpp_test_arm64.py
```

Expected output:
```
============================================================
C++ ARM64 Solver Validation Test
============================================================

Step 1: Loading MATLAB reference from /home/cr/gikWBC9DOF/validation/matlab_reference_results.json
  ✅ Loaded 20 waypoints

Step 2: Initializing ROS2 node...
Step 3: Waiting for solver node...
  (Make sure gik9dof_solver_node is running)

Step 4: Testing 20 waypoints...
  Progress: 1....5....10....15....20  DONE

Step 5: Computing summary statistics...
  Success rate: 20/20 (100.0%)
  Solve time: 45.23 ms (avg)

Step 6: Saving results to /home/cr/gikWBC9DOF/validation/cpp_arm64_results.json
  ✅ Results saved (26.5 KB)

============================================================
✅ C++ ARM64 Solver Test Complete
============================================================

Next steps:
  1. Copy cpp_arm64_results.json back to Windows
  2. Run comparison analysis (Phase 4)
```

### Step 4: Copy Results Back to Windows

**On Windows PowerShell**:
```powershell
scp cr@192.168.100.150:~/gikWBC9DOF/validation/cpp_arm64_results.json C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\validation\
```

## Troubleshooting

### Issue: "Reference file not found"
**Symptom**: 
```
❌ ERROR: Reference file not found: /home/cr/gikWBC9DOF/validation/matlab_reference_results.json
```

**Solution**:
```bash
# Check if file exists
ls -lh ~/gikWBC9DOF/validation/matlab_reference_results.json

# If missing, transfer from Windows again:
# (On Windows)
scp H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\validation\matlab_reference_results.json cr@192.168.100.150:~/gikWBC9DOF/validation/
```

### Issue: "No nodes found"
**Symptom**: Test script hangs at "Waiting for solver node..."

**Solution**:
```bash
# Check if solver node is running
ros2 node list

# Should show:
# /gik9dof_solver_node
# /cpp_solver_tester

# If not, start the solver in Terminal 1 (see Step 2 above)
```

### Issue: "All waypoints timeout"
**Symptom**: Success rate: 0/20 (0.0%)

**Possible causes**:
1. **Topic name mismatch**: Check topic names
   ```bash
   ros2 topic list
   # Should include:
   # /motion_target/target_pose_arm_left
   # /motion_target/target_joint_state_arm_left
   # /gik9dof/solver_diagnostics
   ```

2. **Solver not subscribed**: Check if solver is listening
   ```bash
   ros2 topic info /motion_target/target_pose_arm_left
   # Should show at least 1 subscriber
   ```

3. **Increase timeout**: Edit `run_cpp_test_arm64.py` line 137:
   ```python
   got_response = tester.wait_for_solution(timeout_sec=5.0)  # Increase from 2.0 to 5.0
   ```

### Issue: Low success rate (< 50%)
**Possible causes**:
- Solver parameters too strict (50ms timeout)
- ARM64 SSE compatibility issues affecting solver convergence
- **This is the validation goal**: Compare with MATLAB success rate (10%)

**Expected behavior**:
- If C++ success rate ≈ MATLAB success rate (10%): ✅ Consistent behavior
- If C++ success rate ≪ MATLAB success rate: ⚠️ ARM64 port may have issues
- If C++ success rate ≫ MATLAB success rate: 🤔 Unexpected (investigate)

## Output Files

### cpp_arm64_results.json Format
```json
{
  "metadata": {
    "platform": "ARM64 Ubuntu 22.04",
    "architecture": "aarch64",
    "date": "2025-10-07 02:05:30",
    "num_waypoints": 20,
    "reference_file": "/home/cr/gikWBC9DOF/validation/matlab_reference_results.json"
  },
  "waypoints": [
    {
      "index": 1,
      "target_position": [1.65, 0.08, 0.86],
      "target_orientation": [0.5, -0.5, 0.5, -0.5],
      "joint_config": [1.234, 0.567, ...],
      "solve_time_ms": 45.23,
      "status": "success",
      "iterations": 15,
      "pose_error": 1.2e-7
    },
    ...
  ],
  "summary": {
    "total_waypoints": 20,
    "success_count": 2,
    "success_rate": 0.1,
    "avg_solve_time_ms": 45.61,
    "max_solve_time_ms": 89.46,
    "min_solve_time_ms": 27.15,
    "avg_iterations": 15.3
  }
}
```

## Alternative: Standalone C++ Test (if ROS2 test fails)

If the ROS2-based test doesn't work, we can create a standalone C++ program that calls the solver directly:

```bash
# Create and run standalone test
cd ~/gikWBC9DOF/validation
# (Standalone test script TBD based on solver capability)
```

## Next Phase

After collecting C++ results, proceed to **Phase 4: Comparison Analysis**:
- Copy `cpp_arm64_results.json` to Windows
- Run `matlab/validation/compare_results.m` (to be created)
- Generate `validation/validation_comparison.json`

## Status

- ✅ Phase 1: MATLAB reference generated
- ✅ Phase 2: Files transferred to Orin
- 🔄 Phase 3: **YOU ARE HERE** - Run C++ test on Orin
- ⏳ Phase 4: Comparison analysis (pending)

## Files on Orin

```
~/gikWBC9DOF/validation/
├── matlab_reference_results.json   (27 KB) - MATLAB reference solutions
├── run_cpp_test_arm64.py          (7.8 KB) - Test script
└── cpp_arm64_results.json         (TBD)    - C++ solver results (to be generated)
```
