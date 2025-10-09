# GIK Validation - Quick Reference

**One-page guide for validating MATLAB vs C++ GIK solver**

---

## 🚀 Quick Start (3 Commands)

### Windows PowerShell:
```powershell
# 1. Extract test cases (MATLAB)
cd validation
matlab -batch "cd('..');addpath(genpath('matlab'));extract_test_cases_from_mat('validation/crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat','validation/gik_test_cases.json',10)"
```

### WSL/Linux:
```bash
# 2. Build and run validation
cd validation
./build_validation_wsl.sh
./validate_gik_standalone gik_test_cases.json results.json

# 3. View results
python3 compare_gik_results.py results.json
```

---

## 📁 Files You Need

| File | What | Where |
|------|------|-------|
| `log_*.mat` | MATLAB trajectory log | `validation/crossCheckMatVsCpp/log_matfile/` |
| `gik_test_cases.json` | Test inputs | Generated in `validation/` |
| `results.json` | C++ outputs | Generated in `validation/` |

---

## 🎯 Success Criteria

✅ **PASS** if all test cases have:
- Joint angle L2 difference < 0.01 rad (0.57°)
- Joint angle max difference < 0.02 rad (1.15°)
- C++ solver status = "success"

---

## 🔧 Common Commands

### Extract different number of test cases:
```matlab
extract_test_cases_from_mat('path/to/file.mat', 'output.json', 20)  % 20 cases
```

### Run specific test file:
```bash
./validate_gik_standalone my_tests.json my_results.json
```

### Re-run without rebuilding:
```bash
./run_gik_validation.sh --skip-build
```

### Use existing test cases:
```bash
./run_gik_validation.sh --skip-extract
```

---

## 📊 Understanding Output

### Console output example:
```
Test 1 (waypoint 1): ✓ PASS | L2=0.00023 | max=0.00045 | 2.3 ms | 12 iters
Test 2 (waypoint 15): ✓ PASS | L2=0.00034 | max=0.00056 | 2.5 ms | 11 iters
...
✅ All tests PASSED!
```

### JSON fields:
```json
{
  "qDiff_L2": 0.00023,      // ← Lower is better
  "qDiff_max": 0.00045,     // ← Must be < 0.02
  "withinTolerance": true,  // ← PASS/FAIL
  "cpp_solveTime_ms": 2.3   // ← Performance
}
```

---

## ❌ Troubleshooting

| Problem | Solution |
|---------|----------|
| `MATLAB not found` | Add MATLAB to PATH: `export PATH="/usr/local/MATLAB/R2024b/bin:$PATH"` |
| `Codegen dir not found` | Run MATLAB codegen first: `generateCodeARM64.m` |
| `Build failed` | Check GCC version: `g++ --version` (need 7.0+) |
| `Large differences` | Check solver settings match MATLAB |
| `json.hpp not found` | File should be in `validation/` - check it exists |

---

## 🔍 What Gets Tested

Each test case validates:

**Inputs** (from MATLAB log):
- 9-DOF joint configuration
- 4×4 target pose
- 20 distance constraints

**Solver Call**:
```cpp
solveGIKStepWrapper(qCurrent, targetPose, 
                    distBodyIndices, distRefBodyIndices,
                    distBoundsLower, distBoundsUpper, distWeights,
                    qNext, solverInfo);
```

**Outputs** (compared with MATLAB):
- 9-DOF solution joints
- Solver status
- Iteration count
- Solve time

---

## 📝 File Structure

```
validation/
├── extract_test_cases_from_mat.m      ← MATLAB: Extract tests
├── validate_gik_standalone.cpp        ← C++: Run solver
├── build_validation_wsl.sh            ← Build C++ program
├── compare_gik_results.py             ← Analyze results
├── run_gik_validation.sh              ← Auto workflow
├── GIK_VALIDATION_FRAMEWORK.md        ← Full docs
├── GIK_VALIDATION_QUICKREF.md         ← This file
├── json_simple.hpp                    ← JSON library
├── gik_test_cases.json                ← Generated: inputs
├── gik_validation_results.json        ← Generated: outputs
└── crossCheckMatVsCpp/
    └── log_matfile/*.mat              ← Source: trajectory logs
```

---

## 💡 Pro Tips

1. **Start small**: Test with 5 cases first, then scale up
2. **Check MAT files**: Use `whos -file log.mat` to inspect contents
3. **Timing**: C++ should be 2-10x faster than MATLAB
4. **Failures**: Check solver status in results JSON for clues
5. **Iteration**: If C++ takes more iterations, check initialization

---

## 🎓 Next Steps After Validation

Once validation passes (all tests ✅):

1. Run with more test cases (50-100)
2. Test different MAT files (staged, holistic)
3. Deploy to ARM64 target (Orin)
4. Integrate with ROS2
5. Add to regression testing

---

## 📧 Key Metrics to Report

When sharing validation results, include:

```
Total test cases: 20
Passed: 20/20 (100%)
Mean L2 difference: 0.00034 rad (0.019°)
Max L2 difference: 0.00089 rad (0.051°)
Mean solve time (C++): 2.45 ms
Mean iterations (C++): 12.3
```

---

**Ready to run?** → `./run_gik_validation.sh`

**Need help?** → See `GIK_VALIDATION_FRAMEWORK.md`
