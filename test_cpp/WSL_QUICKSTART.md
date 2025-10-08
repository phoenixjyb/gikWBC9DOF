# Building and Testing in WSL

## Quick Start

### 1. Open WSL Terminal
```bash
# From Windows, open WSL
wsl

# Navigate to test directory
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/test_cpp
```

### 2. Install Prerequisites (if needed)
```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y build-essential cmake

# Verify installation
gcc --version
cmake --version
```

### 3. Build the Test
```bash
# Make build script executable
chmod +x build_wsl.sh

# Run build
./build_wsl.sh
```

### 4. Run Tests
```bash
cd build_wsl
./bin/test_gik_20constraints
```

---

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
Target Pose:
  [    1.0000,     0.0000,     0.0000,     0.0000]
  [    0.0000,     1.0000,     0.0000,     0.0000]
  [    0.0000,     0.0000,     1.0000,     0.0000]
  [    0.8000,     0.0000,     0.5000,     1.0000]

✅ Test 1 PASSED

============================================================
TEST 2: All Distance Constraints Disabled
============================================================
⏱️  Solver execution (no distance constraints): 3.456 ms
qNext: [...]

✅ Test 2 PASSED

============================================================
TEST 3: Multiple Active Constraints (1,2,4)
============================================================
⏱️  Solver execution (3 active constraints): 12.345 ms
qNext: [...]

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

---

## Advantages of WSL Testing

### ✅ **Platform Match**
- ARM64 code generated for Linux
- WSL provides Linux environment
- Same libraries/toolchain as AGX Orin

### ✅ **No Missing Headers**
- No `tmwtypes.h` issues
- Clean Linux build
- Standard GCC toolchain

### ✅ **Better Development**
- Faster compilation
- Standard build tools
- Easier debugging with GDB

### ✅ **Direct Path to ROS2**
- Tests in same environment as target
- Can validate before deploying
- Easier ROS2 integration

---

## Troubleshooting

### Build Fails with "cmake: command not found"
```bash
sudo apt update
sudo apt install cmake
```

### Build Fails with "g++: command not found"
```bash
sudo apt install build-essential
```

### Permission Denied on build_wsl.sh
```bash
chmod +x build_wsl.sh
```

### Can't Find Codegen Directory
Make sure ARM64 code was generated:
```bash
# In MATLAB (Windows)
cd('C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab')
run('generate_gik_20constraints_arm64.m')  # If not already done
```

---

## Next Steps

After successful WSL tests:
1. ✅ Validate all 4 tests pass
2. ✅ Compare with MATLAB results
3. ✅ Document performance metrics
4. 🚀 **Proceed to ROS2 Integration** (Option 1)

The WSL test validates the exact code that will run on the AGX Orin!
