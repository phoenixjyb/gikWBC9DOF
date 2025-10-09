# GIK MATLAB vs C++ Validation Framework

**Purpose**: Validate that the C++ generated GIK solver produces identical results to the MATLAB reference implementation.

**Created**: October 8, 2025  
**Status**: ✅ Ready for Use

---

## Overview

This validation framework provides a complete workflow to:

1. **Extract test cases** from MATLAB trajectory logs (MAT files)
2. **Run MATLAB solver** to generate reference solutions
3. **Run C++ solver** on the same inputs (standalone, no ROS2)
4. **Compare results** with configurable tolerances
5. **Generate reports** with detailed statistics

### Key Features

- ✅ **Standalone C++** - No ROS2 dependency for testing
- ✅ **Automated workflow** - Single script runs entire validation
- ✅ **Flexible test cases** - Extract any number of waypoints from MAT files
- ✅ **Statistical analysis** - Detailed comparison metrics
- ✅ **Cross-platform** - Works on Windows (MATLAB) + WSL (C++)

---

## Files

### Core Scripts

| File | Purpose | Platform |
|------|---------|----------|
| `extract_test_cases_from_mat.m` | Extract test data from MAT files | MATLAB (Windows) |
| `validate_gik_standalone.cpp` | Standalone C++ validation program | WSL/Linux |
| `build_validation_wsl.sh` | Build script for C++ validator | WSL/Linux |
| `compare_gik_results.py` | Results analysis and reporting | Python (any) |
| `run_gik_validation.sh` | Automated end-to-end workflow | WSL/Linux |

### Data Files

| File | Purpose | Format |
|------|---------|--------|
| `crossCheckMatVsCpp/log_matfile/*.mat` | MATLAB trajectory logs | MAT |
| `gik_test_cases.json` | Test inputs + MATLAB reference | JSON |
| `gik_validation_results.json` | C++ results + comparison | JSON |
| `gik_validation_results_summary.json` | Statistical summary | JSON |

---

## Quick Start

### Prerequisites

- **MATLAB R2024b** (or compatible) with Robotics Toolbox
- **WSL or Linux** with GCC 7.0+
- **Python 3.6+** with NumPy (for analysis)
- Generated C++ code in `../codegen/gik9dof_arm64_20constraints/`

### Method 1: Automated (Recommended)

```bash
cd validation
./run_gik_validation.sh
```

This runs the complete workflow automatically.

### Method 2: Manual Step-by-Step

#### Step 1: Extract Test Cases (MATLAB on Windows)

```matlab
cd validation
extract_test_cases_from_mat('crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat', ...
                             'gik_test_cases.json', ...
                             10)  % Extract 10 test cases
```

Output: `gik_test_cases.json`

#### Step 2: Build C++ Validator (WSL/Linux)

```bash
cd validation
./build_validation_wsl.sh
```

Output: `validate_gik_standalone` executable

#### Step 3: Run C++ Validation (WSL/Linux)

```bash
cd validation
./validate_gik_standalone gik_test_cases.json gik_validation_results.json
```

Output: `gik_validation_results.json`

#### Step 4: Compare Results (Python)

```bash
cd validation
python3 compare_gik_results.py gik_validation_results.json
```

Output: Report printed to console + `gik_validation_results_summary.json`

---

## Detailed Usage

### Extracting Test Cases

The MATLAB function `extract_test_cases_from_mat.m` reads trajectory logs and extracts:

**Inputs:**
- `qCurrent` - Current joint configuration (9×1)
- `targetPose` - Target end-effector pose (4×4)
- Distance constraints (20 constraints):
  - `distBodyIndices` - Body indices (20×1)
  - `distRefBodyIndices` - Reference body indices (20×1)
  - `distBoundsLower` - Lower bounds (20×1)
  - `distBoundsUpper` - Upper bounds (20×1)
  - `distWeights` - Constraint weights (20×1)

**Outputs:**
- `qNext` - Solved joint configuration (9×1)
- Solver metadata (iterations, solve time, status)

**Example:**
```matlab
% Extract 20 test cases from log file
extract_test_cases_from_mat(...
    'crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat', ...
    'test_20cases.json', ...
    20)
```

### Running C++ Validation

The standalone C++ program:
1. Loads test cases from JSON
2. Initializes the generated GIK solver
3. Solves each test case
4. Compares with MATLAB reference
5. Outputs results to JSON

**Usage:**
```bash
./validate_gik_standalone <input_json> [output_json]
```

**Arguments:**
- `input_json` - Test cases file (from MATLAB)
- `output_json` - Results file (default: `gik_validation_results.json`)

**Example:**
```bash
./validate_gik_standalone gik_test_cases.json my_results.json
```

### Comparing Results

The Python analysis script computes:

**Metrics:**
- **L2 norm** - RMS difference across all joints
- **Max absolute** - Largest single joint difference
- **Solve time** - C++ execution time
- **Iterations** - Solver iterations

**Tolerances:**
- L2 norm: < 0.01 rad (~0.57°)
- Max absolute: < 0.02 rad (~1.15°)

**Usage:**
```bash
python3 compare_gik_results.py gik_validation_results.json
```

**Output:**
- Detailed text report (console)
- Summary JSON file
- Exit code 0 (pass) or 1 (fail)

---

## Interpreting Results

### Success Criteria

A test case **PASSES** if:
1. ✅ C++ solver converges (status = "success")
2. ✅ Joint difference L2 norm < 0.01 rad
3. ✅ Joint difference max < 0.02 rad

### Example Report

```
========================================
GIK VALIDATION REPORT
========================================

Test Information:
  Date: 2025-10-08
  Total tests: 10
  Tolerances: L2=0.01, Max=0.02

Results Summary:
  ✅ Passed: 10/10 (100.0%)
  ❌ Failed: 0/10 (0.0%)

Joint Difference Statistics (L2 norm):
  Mean:   0.000234 rad
  Median: 0.000198 rad
  Max:    0.000567 rad

Solve Time Statistics (C++):
  Mean:   2.45 ms
  Median: 2.38 ms
  Max:    3.12 ms

========================================
✅ VALIDATION PASSED - All tests within tolerance!
========================================
```

### Common Issues

#### ❌ "Build failed"
- **Cause**: Codegen directory not found or incomplete
- **Solution**: Run MATLAB code generation first
  ```matlab
  cd matlab/+gik9dof/+codegen_inuse
  generateCodeARM64
  ```

#### ❌ "Large joint differences"
- **Cause**: Solver parameters mismatch or numerical precision
- **Solution**: 
  1. Check solver settings match in MATLAB and C++
  2. Verify same robot model
  3. Check for uninitialized variables

#### ❌ "C++ solver doesn't converge"
- **Cause**: Different initial guess or constraints
- **Solution**:
  1. Verify input data matches exactly
  2. Check constraint weights are applied correctly
  3. Review solver status messages

---

## Advanced Usage

### Custom Test Cases

Create custom test cases by modifying the extraction script:

```matlab
% Extract specific waypoint range
testIndices = 50:10:150;  % Waypoints 50, 60, 70, ..., 150

% Or extract problematic cases
testIndices = find(log.successMask == 0);  % Failed solves only
```

### Different Constraints

Test with different constraint configurations:

```matlab
% Modify constraint weights
distWeights = zeros(20, 1);
distWeights(1) = 2.0;  % Double weight on constraint 1
distWeights(3) = 0.5;  % Half weight on constraint 3
```

### Performance Profiling

Add timing to C++ validator:

```cpp
// In validate_gik_standalone.cpp
auto start = chrono::high_resolution_clock::now();
gik9dof::codegen_inuse::solveGIKStepWrapper(...);
auto end = chrono::high_resolution_clock::now();
cout << "Solve time: " << chrono::duration<double, milli>(end - start).count() << " ms\n";
```

### Batch Testing

Test multiple MAT files:

```bash
for matfile in crossCheckMatVsCpp/log_matfile/*.mat; do
    echo "Testing $matfile"
    matlab -batch "extract_test_cases_from_mat('$matfile', 'temp_test.json', 5)"
    ./validate_gik_standalone temp_test.json temp_results.json
    python3 compare_gik_results.py temp_results.json
done
```

---

## Data Format

### Test Cases JSON Structure

```json
{
  "metadata": {
    "sourceFile": "log_holistic_iter0150.mat",
    "totalWaypoints": 148,
    "numTestCases": 10,
    "extractionDate": "08-Oct-2025",
    "matlabVersion": "24.2.0.2712019 (R2024b)"
  },
  "testCases": [
    {
      "id": 1,
      "waypointIndex": 1,
      "input": {
        "qCurrent": [-2, -2, 0, 0, 0, 0, 0, 0, 0],
        "targetPose": [[0, 0, 1, 1.65], ...],
        "distBodyIndices": [12, 12, 0, ...],
        "distRefBodyIndices": [4, 1, 0, ...],
        "distBoundsLower": [0.3, 0, 0, ...],
        "distBoundsUpper": [100, 2, 0, ...],
        "distWeights": [1.0, 0.5, 0, ...]
      },
      "matlab_reference": {
        "qNext": [-1.95, -1.98, 0.05, ...],
        "success": true,
        "solveTime_ms": 3.45,
        "iterations": 12,
        "status": "success"
      }
    }
  ]
}
```

### Results JSON Structure

```json
{
  "metadata": {
    "testDate": "2025-10-08",
    "numTests": 10,
    "numPassed": 10,
    "numFailed": 0,
    "jointToleranceL2": 0.01,
    "jointToleranceMax": 0.02
  },
  "results": [
    {
      "id": 1,
      "waypointIndex": 1,
      "cpp_success": true,
      "cpp_solveTime_ms": 2.34,
      "cpp_iterations": 12,
      "cpp_status": "success",
      "qNext_cpp": [-1.95, -1.98, 0.05, ...],
      "qDiff_L2": 0.000234,
      "qDiff_max": 0.000456,
      "withinTolerance": true
    }
  ]
}
```

---

## Troubleshooting

### MATLAB Issues

**Problem**: "Function not found: gik9dof.codegen_inuse.solveGIKStepWrapper"

**Solution**:
```matlab
addpath(genpath('matlab'))
rehash toolboxcache
```

**Problem**: "MAT file not found"

**Solution**: Check path is relative to workspace root:
```matlab
pwd  % Should be: .../gikWBC9DOF
ls validation/crossCheckMatVsCpp/log_matfile/*.mat
```

### C++ Build Issues

**Problem**: "undefined reference to 'gik9dof::codegen_inuse::solveGIKStepWrapper'"

**Solution**: Ensure all .cpp files from codegen are compiled:
```bash
ls ../codegen/gik9dof_arm64_20constraints/*.cpp | wc -l  # Should be ~290
```

**Problem**: "nlohmann/json.hpp not found"

**Solution**: Copy json header to validation directory:
```bash
cp json_simple.hpp validation/
```

### WSL File Access

**Problem**: Can't access Windows files from WSL

**Solution**: Use /mnt/ paths:
```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/validation
```

---

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: GIK Validation

on: [push, pull_request]

jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup MATLAB
        uses: matlab-actions/setup-matlab@v1
      
      - name: Extract test cases
        run: |
          matlab -batch "cd validation; extract_test_cases_from_mat()"
      
      - name: Build C++ validator
        run: |
          cd validation
          ./build_validation_wsl.sh
      
      - name: Run validation
        run: |
          cd validation
          ./validate_gik_standalone gik_test_cases.json results.json
      
      - name: Check results
        run: |
          python3 validation/compare_gik_results.py validation/results.json
```

---

## Next Steps

After validation passes:

1. ✅ **Integrate with ROS2** - Use validated solver in ROS2 node
2. ✅ **Deploy to Orin** - Cross-compile for ARM64 target
3. ✅ **Real-time testing** - Validate timing on actual hardware
4. ✅ **Extended validation** - Test with full trajectories
5. ✅ **Regression tests** - Add to CI pipeline

---

## Support

For issues or questions:

1. Check this documentation first
2. Review example output in `VALIDATION_PLAN.md`
3. Inspect generated JSON files for data issues
4. Compare MATLAB and C++ solver configurations

---

## References

- [MATLAB Coder Documentation](https://www.mathworks.com/help/coder/)
- [generalizedInverseKinematics](https://www.mathworks.com/help/robotics/ref/generalizedinversekinematics-system-object.html)
- [WSL Documentation](https://docs.microsoft.com/en-us/windows/wsl/)

---

**Last Updated**: October 8, 2025
