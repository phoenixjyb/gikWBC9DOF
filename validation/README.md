# MATLAB vs C++ Solver Validation

This directory contains scripts to validate that the C++ code generated from MATLAB produces identical results to the original MATLAB implementation.

## Overview

**Validation Strategy:**
1. Load the same trajectory (`1_pull_world_scaled.json`)
2. Assume robot starts at first waypoint (EE aligned with first target)
3. Solve IK for remaining waypoints with both solvers
4. Compare joint configurations at each waypoint
5. Generate comparison report

## Files

- `validate_cpp_solver.m` - MATLAB script that runs MATLAB solver and compares with C++ results
- `validate_cpp_solver.py` - Python script that runs C++ solver via ROS2 node
- `README.md` - This file

## Prerequisites

**MATLAB Side (Windows):**
- MATLAB R2024b
- Robotics System Toolbox
- URDF file: `mobile_manipulator_PPR_base_corrected_sltRdcd.urdf`
- Trajectory file: `1_pull_world_scaled.json`

**C++ Side (WSL Ubuntu 22.04):**
- ROS2 Humble
- gik9dof_solver node built and running
- Python 3 with rclpy
- Trajectory file copied to WSL

## Usage

### Step 1: Run C++ Solver (WSL)

```bash
# Terminal 1: Start the solver node
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node

# Terminal 2: Run validation script
cd ~/gikWBC9DOF/validation
python3 validate_cpp_solver.py
```

This will:
- Load the trajectory
- Send each waypoint to the solver node
- Collect results from diagnostics
- Save to `validation_results_cpp.json`

### Step 2: Run MATLAB Solver and Compare (Windows)

```matlab
cd matlab
validate_cpp_solver

% This will:
% 1. Load trajectory
% 2. Solve with MATLAB GIK solver
% 3. Save MATLAB results to validation_results_matlab.json
% 4. Wait for you to copy C++ results from WSL
% 5. Compare and generate validation_comparison.json
```

### Step 3: Copy C++ Results from WSL

```powershell
# Copy results from WSL to Windows
wsl bash -c "cp ~/gikWBC9DOF/validation_results_cpp.json /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/"
```

Then press any key in MATLAB to continue comparison.

## Expected Results

**Success Criteria:**
- ✅ Success rate > 95% for both solvers
- ✅ Max joint error < 1.0 degree
- ✅ Average solve time < 50 ms
- ✅ Both solvers converge to same solutions

**Output Files:**
- `validation_results_matlab.json` - MATLAB solver results
- `validation_results_cpp.json` - C++ solver results  
- `validation_comparison.json` - Comparison metrics

## Troubleshooting

**C++ solver times out:**
- Check that solver node is running
- Verify topic names match
- Increase timeout in Python script

**Large joint errors:**
- Check URDF file matches between MATLAB and C++
- Verify joint limits are identical
- Check solver parameters (tolerance, max iterations)

**Low success rate:**
- Check trajectory is feasible for robot
- Verify collision constraints if enabled
- Adjust solver parameters

## Validation Process

1. **MATLAB Solver:**
   - Uses `generalizedInverseKinematics` with MATLAB Robotics Toolbox
   - Solves each waypoint using previous solution as initial guess
   - Records: joint config, solve time, iterations, status

2. **C++ Solver:**
   - Uses MATLAB Coder-generated C++ code
   - Same solving strategy (previous solution as initial guess)
   - Records: same metrics as MATLAB

3. **Comparison:**
   - Joint-wise absolute error at each waypoint
   - Success rate comparison
   - Solve time comparison
   - Overall validation verdict (PASS/FAIL)

## Notes

- First waypoint is treated as initial configuration (no IK solving needed)
- Robot is assumed to start with EE aligned to first target pose
- Base joints (x, y, theta) are free to move
- Arm has 6 revolute joints
- Total 9 DOF system

## Author

Generated for gikWBC9DOF project validation
Date: 2025-10-06
