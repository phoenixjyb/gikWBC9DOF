# ARM64 Solver Hang Issue - Critical Finding

**Date**: October 7, 2025  
**Status**: 🔴 **CRITICAL ISSUE FOUND**  
**Severity**: Blocker for ARM64 deployment

## Problem Summary

The ARM64-compiled C++ IK solver **hangs indefinitely** when attempting to solve inverse kinematics, blocking the entire ROS2 node.

## Evidence

### Terminal 1 (Solver Node)
```
[INFO] [gik9dof_solver_node]: Received trajectory with 1 waypoints, seq=1
<hangs here - no further output>
```

### Terminal 2 (Test Script)
```
Step 4: Testing 20 waypoints...
  Progress: 1.
<hangs waiting for solver response>
```

### Diagnostic Check
```bash
$ ros2 topic echo /gik9dof/solver_diagnostics --once
# No output - solver never publishes diagnostics
# Confirms solver is hung inside IK computation
```

## Root Cause Analysis

### Likely Causes (in order of probability)

1. **SSE Intrinsics Stubs Causing Numerical Issues**
   - ARM64 SSE2 stubs use GCC vector extensions
   - May have different numerical behavior than x86 SSE2
   - Could cause NaN propagation or division by zero
   - Vector operations might not match MATLAB's expectations

2. **Matrix Decomposition Failure**
   - LAPACK routines (QR, SVD) sensitive to numerical precision
   - ARM64 floating-point behavior differs slightly from x86
   - Could cause infinite loops in iterative solvers

3. **Optimization Loop Not Converging**
   - Levenberg-Marquardt algorithm may not converge
   - No timeout in MATLAB-generated code
   - Solver parameters (max iterations, tolerance) may be incompatible with ARM64

### Code Flow When Hang Occurs

```
ROS2 Node Control Loop (10 Hz)
  └─> controlLoop()
       └─> solveIK(target_pose)  ⬅ HANGS HERE
            └─> matlab_solver_->gik9dof_codegen_inuse_solveGIKStepWrapper()
                 └─> generalizedInverseKinematics() [MATLAB generated]
                      └─> ErrorDampedLevenbergMarquardt() [optimization]
                           └─> xgeqp3() [QR decomposition with SSE intrinsics]
                                └─> ??? INFINITE LOOP OR NaN PROPAGATION ???
```

## What Works

✅ ARM64 build compiles successfully  
✅ ROS2 node starts and initializes  
✅ Robot state messages received correctly  
✅ Trajectory messages received correctly  
✅ No crashes - clean hang

## What Doesn't Work

❌ IK solver computation hangs indefinitely  
❌ No diagnostics published  
❌ No timeout mechanism  
❌ Entire node becomes unresponsive

## Validation Impact

### Phase 1-2: ✅ Complete
- MATLAB reference generated (10% success rate, 20 waypoints)
- Files transferred to Orin
- Test script created and deployed

### Phase 3: 🔴 **BLOCKED**
- Cannot collect C++ ARM64 results
- Solver hangs on first waypoint
- Test cannot proceed

### Phase 4: ⏸️ **BLOCKED**
- Requires Phase 3 completion
- No C++ results to compare

## Implications

### For ARM64 Deployment
🔴 **ARM64 port is NOT production-ready**
- Solver is fundamentally broken on ARM64
- Cannot be used for real-time control
- Requires significant debugging/fixes

### For SSE Compatibility Layer
⚠️ **Current approach insufficient**
- Simple stubs don't preserve numerical behavior  
- Need proper NEON implementations
- OR need to disable SSE code paths entirely

## Recommended Actions

### Immediate (Debug)

1. **Add Debug Logging**
   - Instrument MATLAB-generated code with printf statements
   - Track where execution stops
   - Monitor for NaN/Inf values

2. **Test with Simpler Target**
   - Try home configuration (all zeros)
   - Try reachable target close to current pose
   - Isolate if it's target-specific or universal

3. **Check MATLAB Solver Settings**
   - Review max iterations, tolerances
   - Check if SSE code paths can be disabled
   - Try recompiling with different optimization flags

### Short-term (Workaround)

1. **Implement Timeout**
   - Add timeout wrapper around IK solver call
   - Catch hangs after 1-2 seconds
   - Report failure gracefully

2. **Use x86_64 WSL for Validation**
   - Run Phase 3 on WSL instead of ARM64
   - Validates solver logic works on x86_64
   - Confirms ARM64-specific issue

3. **Try Different Compilation**
   - Disable SSE entirely (`-DNO_SSE`)
   - Use `-O0` (no optimization)
   - Try different GCC flags

### Long-term (Fix)

1. **Proper NEON Implementations**
   - Replace SSE stubs with actual NEON intrinsics
   - Use sse2neon library (established solution)
   - Test numerical equivalence

2. **Regenerate Code for ARM64**
   - Use MATLAB Coder with ARM64 target
   - Let MATLAB handle architecture differences
   - May require MATLAB R2024b+ with ARM support

3. **Alternative Solver**
   - Use native ARM IK library (e.g., KDL, TracIK)
   - Port algorithm to pure C++ (no MATLAB codegen)
   - Implement from scratch with ARM in mind

## Next Steps

### Option A: Debug on Orin (Recommended)
```bash
# Stop hung processes
pkill -9 gik9dof_solver_node
pkill -9 python3

# Rebuild with debug symbols and logging
cd ~/gikWBC9DOF/ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Then add instrumentation to find where it hangs.

### Option B: Pivot to WSL Validation
- Run Phase 3 on WSL x86_64 instead
- Proves MATLAB→C++ codegen works
- Defer ARM64 debugging to later

### Option C: Document and Defer
- Document this critical issue
- Mark ARM64 port as "Not Validated"
- Focus on x86_64 deployment path
- Revisit ARM64 when time permits

## Files Affected

**Working**:
- `matlab/validation/generate_matlab_reference.m` ✅
- `validation/matlab_reference_results.json` ✅
- `validation/run_cpp_test_arm64.py` ✅ (script works, solver doesn't)

**Blocked**:
- `validation/cpp_arm64_results.json` ❌ (cannot generate)
- Phase 3 execution ❌
- Phase 4 comparison ❌

**Needs Investigation**:
- `ros2/gik9dof_solver/include/emmintrin.h` (SSE stubs)
- MATLAB-generated solver code (all files in `codegen/arm64_realtime/`)
- CMake build flags for ARM64

## Workaround for Validation

Since ARM64 solver is broken, we can:

1. **Run on WSL x86_64** to validate MATLAB↔C++ equivalence
2. **Skip ARM64 validation** and document as known issue
3. **Use MATLAB-only** for ARM64 deployments (not ideal)

Would you like me to:
- A) Create debugging branch to investigate the hang?
- B) Pivot to WSL x86_64 validation?
- C) Document and defer ARM64 work?

## Conclusion

**The ARM64 port successfully compiles but has a critical runtime bug that causes the IK solver to hang indefinitely.** This is likely due to numerical issues in the SSE→ARM compatibility layer or differences in floating-point behavior between x86 and ARM architectures.

**Validation Status**: 
- Phase 1-2: ✅ Complete
- Phase 3: 🔴 Blocked (solver hangs)
- Phase 4: ⏸️ Pending

**ARM64 Deployment**: 🔴 **NOT READY**

