# Code Generation Quick Guide - October 7, 2025

## Summary of Today's Optimizations

✅ **Solver parameters optimized** for real-time control  
✅ **ARM NEON** configured for Orin (not SSE)  
✅ **x86_64 SSE/AVX** configured for WSL  
✅ **Warm-start** optimization implemented  
✅ **Timeout monitoring** added to ROS2 node  
✅ **Legacy code cleaned up** (removed `linux_arm64`)

## Code Generation Commands

### Option 1: Generate Both (Recommended)

Open MATLAB and run:

```matlab
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

% Generate ARM64 code for Orin (5-15 minutes)
generate_code_arm64

% Generate x86_64 code for WSL validation (5-15 minutes)  
generate_code_x86_64
```

### Option 2: Use RUN_CODEGEN (ARM64 only)

```matlab
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
RUN_CODEGEN  % Runs generate_code_arm64 + validation
```

## What Gets Generated

### ARM64 Output (for Orin - REAL-TIME DEPLOYMENT)
```
codegen/arm64_realtime/
├── GIKSolver.h                    # C++ class interface
├── GIKSolver.cpp                  # C++ class implementation
├── solveGIKStepWrapper.h          # Main solver function
├── solveGIKStepWrapper.cpp
├── buildRobotForCodegen.cpp       # Robot model builder
├── solveGIKStepRealtime.cpp       # Core IK solver
├── [~200 other support files]
└── html/report.mldatx             # Code generation report
```

**Key features:**
- ARM NEON SIMD instructions
- MaxTime = 50ms (REAL-TIME constraint)
- MaxIterations = 50
- Optimized for ARM Cortex-A (Orin)
- **PURPOSE: Real-time deployment**

### x86_64 Output (for WSL - VALIDATION ONLY)
```
codegen/x86_64_validation/
├── GIKSolver.h                    # C++ class interface (same API)
├── GIKSolver.cpp                  # C++ implementation (x86 SIMD)
├── [same file structure as ARM64]
└── html/report.mldatx
```

**Key features:**
- Intel/AMD SSE/AVX SIMD instructions
- Same solver parameters (50ms, 50 iterations) for testing
- Optimized for x86_64 (WSL/PC)
- **PURPOSE: Validation/testing, NOT real-time deployment**

## Verification Steps

### 1. Check MATLAB Output

Look for:
```
===================================================
✓ Code generation successful!
===================================================
Generated files location: [path]
===================================================
```

### 2. Verify Files Created

```matlab
% In MATLAB:
ls codegen/arm64_realtime/GIKSolver.*
ls codegen/x86_64_validation/GIKSolver.*
```

Should see:
- `GIKSolver.h`
- `GIKSolver.cpp`

### 3. Check Reports

Open in browser:
- `codegen/arm64_realtime/html/report.mldatx` (REAL-TIME deployment)
- `codegen/x86_64_validation/html/report.mldatx` (Validation only)

Look for:
- ✅ No errors
- ✅ Hardware settings: ARM Cortex-A (or x86-64)
- ✅ OpenMP: Enabled
- ✅ Dynamic allocation: Enabled

## Expected Timing

- **ARM64 codegen**: 5-15 minutes
- **x86_64 codegen**: 5-15 minutes
- **Total**: ~10-30 minutes for both

## If Code Generation Fails

### Common Issues:

1. **"Cannot find function"**
   - Make sure you're in project root
   - Check MATLAB path includes `matlab/` folder

2. **"Unsupported function"**
   - Check MATLAB version (need R2024b+)
   - Verify Robotics Toolbox installed

3. **"Out of memory"**
   - Close other MATLAB instances
   - Increase Java heap in MATLAB preferences

## After Code Generation

### For ARM64 (Orin Deployment):

1. **Copy to Orin** (from Windows PowerShell):
```powershell
scp -r codegen/arm64_realtime/* hdas@orin.local:~/ros2_ws/src/gik9dof_solver/matlab_codegen/
```

2. **Rebuild on Orin**:
```bash
ssh hdas@orin.local
cd ~/ros2_ws
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

3. **Run with optimizations**:
```bash
ros2 run gik9dof_solver gik9dof_solver_node
```

### For x86_64 (WSL Validation):

1. **Copy to WSL** (from Windows PowerShell):
```powershell
wsl mkdir -p ~/gikWBC9DOF/codegen/x86_64_validation
wsl cp -r codegen/x86_64_validation/* ~/gikWBC9DOF/codegen/x86_64_validation/
```

2. **Build in WSL**:
```bash
wsl
cd ~/gikWBC9DOF
# Follow validation/WSL_VALIDATION_GUIDE.md
```

**Note**: x86_64 build is for validation/testing only, NOT for real-time deployment!

## What Changed from Before

| Aspect | Before | After (Optimized) |
|--------|--------|-------------------|
| **MaxTime** | Not set (infinite) | 50ms |
| **MaxIterations** | Not set (~1000?) | 50 |
| **SIMD** | Generic | ARM NEON / x86 SSE |
| **Initial Guess** | Current state | Previous solution (warm-start) |
| **Timeout Check** | None | ROS2 node monitors |
| **Directory** | `linux_arm64` | `arm64_realtime` / `x86_64_validation` |

**Purpose Clarity:**
- `arm64_realtime`: **Real-time deployment** on NVIDIA Orin
- `x86_64_validation`: **Testing only** on WSL (not real-time)

## Key Performance Improvements Expected

- **Solve time**: 10-50ms (was unbounded)
- **Iterations**: 10-30 average (was 50+)
- **CPU usage**: 20-40% per solve (was 60-70% continuous)
- **Success rate**: >95% for smooth trajectories

## Ready to Generate?

Just open MATLAB and run:

```matlab
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
generate_code_arm64      % First
generate_code_x86_64     % Second
```

Or both at once won't work (MATLAB Coder runs one at a time), so run them sequentially.

## Documentation

For more details:
- **Performance optimizations**: `PERFORMANCE_OPTIMIZATION.md`
- **Cleanup details**: `CODEGEN_CLEANUP.md`
- **WSL validation**: `validation/WSL_VALIDATION_GUIDE.md`
- **Orin deployment**: `DEPLOY_NOW.md`

---

**Status**: Ready to generate! 🚀
