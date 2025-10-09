# 🚀 GIK 20-Constraint C++ Testing - WSL Edition

## ✅ **RECOMMENDED APPROACH: Use WSL**

Testing in WSL is **much better** than Windows because:
- ✅ Uses ARM64 generated code (same as AGX Orin target)
- ✅ **MaxTime=50ms real-time configuration** (production-ready)
- ✅ Linux environment (matches production)
- ✅ No MSVC/Windows header issues
- ✅ Direct path to ROS2 integration

## 📁 **Code Source After Cleanup (Oct 9, 2025)**

**Current Test Uses:** `codegen/arm64_realtime/`
- ✅ 206 files
- ✅ MaxTime = **50ms** (0.05s)
- ✅ MaxIterations = 1000
- ✅ Generated with Linux MATLAB (R2024a)
- ✅ Production-ready for Jetson Orin

**Old folders archived to:** `codegen/archive/`

---

## Quick Start (3 Commands)

### Option A: From PowerShell (Easiest)
```powershell
cd test_cpp
.\run_tests_wsl.ps1
```
This automatically builds and runs tests in WSL!

### Option B: From WSL Terminal
```bash
wsl
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/test_cpp
chmod +x build_wsl.sh
./build_wsl.sh
./build_wsl/bin/test_gik_20constraints
```

---

## Prerequisites

### Check if WSL is Installed
```powershell
wsl --status
```

### Install WSL (if needed)
```powershell
wsl --install
```

### Install Build Tools in WSL
```bash
wsl
sudo apt update
sudo apt install -y build-essential cmake
```

---

## What Gets Tested

### Test 1: Single Distance Constraint
- Gripper → chassis constraint (≥0.3m)
- Validates basic constraint handling

### Test 2: All Constraints Disabled  
- Pose-only IK
- Performance baseline

### Test 3: Multiple Constraints
- 3 simultaneous constraints
- Multi-constraint validation

### Test 4: Performance Benchmark
- 100 iterations
- Target: ≤50ms average
- Real-time capability test

---

## Expected Results

All 4 tests should **PASS** ✅ with output showing:
- Joint configurations (`qNext`)
- Execution times
- Solver convergence
- Performance metrics

---

## Files Overview

```
test_cpp/
├── test_gik_20constraints.cpp   # Main test suite
├── gik_test_utils.h             # Test utilities
├── CMakeLists.txt               # Build config (supports ARM64/x64)
├── build_wsl.sh                 # WSL build script ⭐
├── run_tests_wsl.ps1            # One-command runner ⭐
├── WSL_QUICKSTART.md            # Detailed WSL guide
└── README.md                    # Full documentation
```

---

## Why This Matters

✅ **Validates ARM64 code** before deploying to AGX Orin  
✅ **Catches issues early** in a simple environment  
✅ **Establishes performance baseline** for ROS2 integration  
✅ **Proves code correctness** before production

---

## Next After Tests Pass

1. ✅ Document test results
2. ✅ Compare with MATLAB baseline
3. 🚀 **Proceed to ROS2 Integration** (Option 1)

The WSL tests validate the **exact same code** that will run on your AGX Orin!

---

## Troubleshooting

**"wsl: command not found"**
- Install WSL: `wsl --install`

**"cmake: command not found" in WSL**
- Install tools: `sudo apt install build-essential cmake`

**Build fails with "directory not found"**
- Make sure ARM64 code was generated (should already be done)

**Permission denied on scripts**
- `chmod +x build_wsl.sh`

---

## Status

- [x] ARM64 code generated (290 files)
- [x] x64 code generated (290 files)  
- [x] Test suite created
- [x] WSL build system ready
- [ ] **→ BUILD AND RUN TESTS IN WSL** ← YOU ARE HERE
- [ ] ROS2 integration

**Ready to test!** Just run: `cd test_cpp && .\run_tests_wsl.ps1`
