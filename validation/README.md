# Validation Directory

## 🎯 GIK MATLAB vs C++ Validation Framework (Oct 8, 2025)

**NEW**: Complete standalone validation framework for comparing MATLAB and C++ GIK solvers.

### 📚 Documentation

| Document | Purpose | When to Use |
|----------|---------|-------------|
| **[GIK_VALIDATION_QUICKREF.md](GIK_VALIDATION_QUICKREF.md)** | One-page quick reference | Daily use, quick lookup |
| **[GIK_VALIDATION_FRAMEWORK.md](GIK_VALIDATION_FRAMEWORK.md)** | Complete documentation | Learning, troubleshooting |
| **[GIK_VALIDATION_SUMMARY.md](GIK_VALIDATION_SUMMARY.md)** | Implementation details | Understanding design |

### 🚀 Quick Start

**Prerequisites**: MATLAB R2024b, WSL/Linux with GCC 7+, Python 3.6+

**One-Command Validation**:
```bash
cd validation
./run_gik_validation.sh
```

This will:
1. ✅ Extract test cases from MAT files (MATLAB)
2. ✅ Build C++ validation program
3. ✅ Run C++ solver and compare with MATLAB
4. ✅ Generate detailed report

**Manual Steps** (for more control):

```bash
# 1. Extract test cases (Windows PowerShell)
matlab -batch "cd('..'); addpath(genpath('matlab')); extract_test_cases_from_mat('validation/crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat', 'validation/gik_test_cases.json', 10)"

# 2. Build validator (WSL)
cd validation
./build_validation_wsl.sh

# 3. Run validation (WSL)
./validate_gik_standalone gik_test_cases.json results.json

# 4. Analyze results (WSL/Windows)
python3 compare_gik_results.py results.json
```

### 📁 Key Files

**Scripts** (ready to use):
- `extract_test_cases_from_mat.m` - MATLAB: Extract test data from MAT files
- `validate_gik_standalone.cpp` - C++: Standalone solver validation (no ROS2)
- `build_validation_wsl.sh` - Build C++ validator
- `compare_gik_results.py` - Python: Compare and analyze results
- `run_gik_validation.sh` - Automated end-to-end workflow

**Data** (input):
- `crossCheckMatVsCpp/log_matfile/*.mat` - MATLAB trajectory logs

**Data** (generated):
- `gik_test_cases.json` - Test inputs + MATLAB reference
- `gik_validation_results.json` - C++ results + comparison
- `gik_validation_results_summary.json` - Statistical summary

### ✅ Success Criteria

Tests **PASS** if:
- Joint angle L2 difference < 0.01 rad (~0.57°)
- Joint angle max difference < 0.02 rad (~1.15°)
- C++ solver converges successfully

### � What Gets Tested

Each test case validates:
- **Input**: 9-DOF joint config + 4×4 target pose + 20 distance constraints
- **Solver**: `gik9dof::codegen_inuse::solveGIKStepWrapper()`
- **Output**: 9-DOF solution + solver status + timing
- **Comparison**: MATLAB reference vs C++ implementation

### 🎓 Example Output

```
Test 1 (waypoint 1): ✓ PASS | L2=0.00023 | max=0.00045 | 2.3 ms | 12 iters
Test 2 (waypoint 15): ✓ PASS | L2=0.00034 | max=0.00056 | 2.5 ms | 11 iters
...
✅ All tests PASSED!

Summary:
  Total tests: 10
  Passed: 10 ✓
  Mean L2 diff: 0.00034 rad (0.019°)
  Mean solve time: 2.45 ms
```

### 💡 Key Features

- ✅ **No ROS2 dependency** - Pure C++ validation
- ✅ **Automated workflow** - One command does everything
- ✅ **Flexible testing** - Extract any number of test cases
- ✅ **Statistical analysis** - Detailed comparison metrics
- ✅ **Cross-platform** - Windows (MATLAB) + WSL (C++)

---

## 🚨 Previous: ARM64 IK Validation (Oct 7, 2025)

**Purpose**: Validate ARM64-compiled C++ solver produces correct IK solutions  
**Status**: ✅ Phase 1-2 complete, 🔄 Phase 3 ready to run

### Quick Reference

| Phase | Location | Command | Output | Status |
|-------|----------|---------|--------|--------|
| 1. MATLAB Reference | Windows | `cd matlab/validation`<br>`generate_matlab_reference` | `validation/matlab_reference_results.json` | ✅ **DONE** (27KB, 20 waypoints) |
| 2. Transfer | Windows | `scp validation\matlab_reference_results.json cr@192.168.100.150:~/gikWBC9DOF/validation/` | - | ✅ **DONE** |
| 3. C++ Test | AGX Orin | See [`PHASE3_ORIN_INSTRUCTIONS.md`](PHASE3_ORIN_INSTRUCTIONS.md) | `cpp_arm64_results.json` | 🔄 **READY** |
| 4. Compare | Windows | `cd matlab/validation`<br>`compare_results` | `validation/validation_comparison.json` | ⏳ Pending Phase 3 |

**Full Details**: See [`VALIDATION_PLAN.md`](VALIDATION_PLAN.md)

---

## Original WSL x86_64 Validation

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
