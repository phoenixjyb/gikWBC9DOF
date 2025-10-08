# GIK 20-Constraint Solver - Standalone C++ Test Suite

This directory contains standalone C++ tests for the generated GIK 20-constraint solver code.

## Purpose

- ✅ **Validate** generated C++ code independently before ROS2 integration
- ✅ **Compare** C++ results with MATLAB baseline
- ✅ **Profile** performance and ensure ≤50ms target is met
- ✅ **Debug** issues in a simpler environment than ROS2

## Files

### Test Files
- **`test_gik_20constraints.cpp`** - Main test suite with 4 test cases
- **`gik_test_utils.h`** - Helper utilities for testing (timers, printing, etc.)

### Build Files
- **`CMakeLists.txt`** - CMake build configuration
- **`build.ps1`** - PowerShell build script for Windows

## Prerequisites

### Software Required
- **CMake** 3.15 or newer
- **Visual Studio 2022** (or MSVC compiler)
- **Generated x64 code** in `../codegen/gik9dof_x64_20constraints/`

### Generate x64 Code First
```matlab
% In MATLAB
cd('C:\path\to\gikWBC9DOF\matlab')
run('generate_gik_20constraints_x64.m')
```

## Building

### Option 1: PowerShell Script (Recommended)
```powershell
cd test_cpp
.\build.ps1
```

### Option 2: Manual CMake
```powershell
cd test_cpp
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

## Running Tests

```powershell
cd test_cpp\build
.\bin\Release\test_gik_20constraints.exe
```

## Test Cases

### Test 1: Single Distance Constraint
- **Purpose**: Basic IK with one active constraint
- **Constraint**: Gripper → Chassis (lower bound 0.3m)
- **Expected**: Solver finds valid configuration
- **MATLAB equivalent**: `test_gik_20constraints.m` Test 1

### Test 2: All Constraints Disabled
- **Purpose**: Pose-only IK (no distance constraints)
- **Expected**: Faster execution than Test 1
- **MATLAB equivalent**: `test_gik_20constraints.m` Test 3

### Test 3: Multiple Active Constraints
- **Purpose**: Test with 3 simultaneous constraints
- **Constraints**: 1 (gripper→chassis), 2 (gripper→base), 4 (link5→chassis)
- **Expected**: Solver finds valid configuration satisfying all
- **MATLAB equivalent**: `test_gik_20constraints.m` Test 4

### Test 4: Performance Benchmark
- **Purpose**: Measure average solve time over 100 iterations
- **Target**: ≤50ms per solve
- **Method**: Warm-start each iteration with previous result
- **Expected**: Pass performance target

## Expected Output

```
╔════════════════════════════════════════════════════════════╗
║  GIK 20-Constraint Solver - Standalone C++ Test Suite     ║
╚════════════════════════════════════════════════════════════╝

============================================================
TEST 1: Single Distance Constraint
============================================================
⏱️  Solver execution: 8.234 ms
qCurrent: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qNext: [0.123, 0.456, 0.789, ...]
...
✅ Test 1 PASSED

============================================================
TEST 2: All Distance Constraints Disabled
============================================================
⏱️  Solver execution (no distance constraints): 3.456 ms
...
✅ Test 2 PASSED

============================================================
TEST 3: Multiple Active Constraints (1,2,4)
============================================================
⏱️  Solver execution (3 active constraints): 12.345 ms
...
✅ Test 3 PASSED

============================================================
TEST 4: Performance Benchmark (100 iterations)
============================================================
Total time: 450.123 ms
Average per solve: 4.501 ms
✅ Performance target MET (≤50ms)

✅ Test 4 COMPLETED

============================================================
🎉 ALL TESTS PASSED! ✅
============================================================

Next steps:
  1. Compare results with MATLAB baseline
  2. Validate performance targets
  3. Proceed to ROS2 integration
```

## Validation Checklist

After running tests, validate:

- [ ] All 4 tests pass without errors
- [ ] Solver converges for each test case
- [ ] Average performance ≤50ms (Test 4)
- [ ] Joint configurations are physically valid (no NaN/Inf)
- [ ] Results are consistent across multiple runs

## Comparing with MATLAB

To validate C++ matches MATLAB:

1. **Run MATLAB tests**:
   ```matlab
   cd('matlab')
   run('test_gik_20constraints.m')
   ```

2. **Run C++ tests**:
   ```powershell
   cd test_cpp\build
   .\bin\Release\test_gik_20constraints.exe
   ```

3. **Compare**:
   - Joint configurations (`qNext` values)
   - Execution times
   - Solver convergence behavior

**Note**: Exact numerical values may differ slightly due to:
- Floating-point precision differences
- Compiler optimizations
- Random number generator state

But the solutions should be **functionally equivalent** (RMS difference < 0.001).

## Troubleshooting

### Build Errors

**"Cannot find gik9dof_codegen_inuse_solveGIKStepWrapper.h"**
- ✅ Solution: Generate x64 code first with `generate_gik_20constraints_x64.m`

**"Unresolved external symbols"**
- ✅ Solution: Check that all `.cpp` files from codegen are included
- ✅ Solution: Verify CMakeLists.txt is collecting all sources

**"'struct0_T' was not declared"**
- ✅ Solution: Include `gik9dof_codegen_inuse_solveGIKStepWrapper_types.h`

### Runtime Errors

**"Access violation" or crash**
- ✅ Check array sizes: qCurrent[9], targetPose[16], constraints[20]
- ✅ Verify column-major order for matrices
- ✅ Initialize all arrays before calling solver

**Solver doesn't converge**
- ✅ Check target pose is reachable
- ✅ Verify constraint bounds are feasible
- ✅ Try different initial configurations

**Performance target missed**
- ✅ Build in **Release** mode (not Debug)
- ✅ Enable compiler optimizations
- ✅ Profile to identify bottlenecks

## Next Steps

After all tests pass:

1. **Document results** - Record performance metrics
2. **Compare with MATLAB** - Validate numerical accuracy
3. **Generate ARM64 version** - For target platform
4. **ROS2 Integration** - Integrate validated code into ROS2 system

## Related Files

- **MATLAB tests**: `../matlab/test_gik_20constraints.m`
- **ARM64 codegen**: `../codegen/gik9dof_arm64_20constraints/`
- **x64 codegen**: `../codegen/gik9dof_x64_20constraints/`
- **Success docs**: `../GIK_20CONSTRAINTS_CODEGEN_SUCCESS.md`

---

**Status**: Ready for testing after x64 code generation completes  
**Platform**: Windows x64  
**Target**: Validate before ROS2 integration  
**Next**: Build and run tests
