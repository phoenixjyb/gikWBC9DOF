# ARM64 Deployment Guide - AGX Orin

## Overview

This guide documents the successful deployment of the MATLAB-generated C++ GIK solver on NVIDIA AGX Orin (ARM64/aarch64 architecture). The solver was originally generated on Windows x86_64 and required several compatibility fixes to build and run on ARM64.

## System Information

- **Target Platform**: NVIDIA AGX Orin (Jetson)
- **Architecture**: ARM64/aarch64 (Cortex-A78AE)
- **OS**: Ubuntu 22.04 LTS (Jetpack)
- **ROS2**: Humble Hawksbill
- **Compiler**: GCC 11.4.0 (ARM64)

## Build Status

✅ **Successfully built on ARM64** (October 7, 2025)
- Build time: ~41 seconds
- All ROS2 packages compile cleanly
- Solver node runs without crashes

## Architecture Compatibility Issues Fixed

### 1. **x86 SSE Intrinsics Incompatibility**

**Problem**: MATLAB generated code with x86-specific SSE2 intrinsics (`emmintrin.h`) that don't exist on ARM64.

**Files affected**:
- `ErrorDampedLevenbergMarquardt.cpp`
- `PoseTarget.cpp`
- `RigidBodyTree.cpp`
- `GIKProblem.cpp`
- `generalizedInverseKinematics.cpp`
- `xgeqp3.cpp`

**Solution**: Created compatibility stub header at `ros2/gik9dof_solver/include/emmintrin.h`:

```cpp
// ARM64 compatibility stub for x86 SSE2 intrinsics
#ifdef __aarch64__

typedef double __m128d __attribute__((__vector_size__(16), __may_alias__));
typedef long long __m128i __attribute__((__vector_size__(16), __may_alias__));

// Double-precision intrinsics
inline __m128d _mm_loadu_pd(double const* p) { return *(__m128d*)p; }
inline void _mm_storeu_pd(double* p, __m128d a) { *(__m128d*)p = a; }
inline __m128d _mm_add_pd(__m128d a, __m128d b) { return a + b; }
inline __m128d _mm_sqrt_pd(__m128d a) { 
    return __m128d{__builtin_sqrt(a[0]), __builtin_sqrt(a[1])}; 
}
// ... (see full file for complete implementation)

// Integer intrinsics (epi32)
inline __m128i _mm_set1_epi32(int a) { /* ... */ }
inline __m128i _mm_add_epi32(__m128i a, __m128i b) { /* ... */ }
// ...
#endif
```

**CMake configuration** (`ros2/gik9dof_solver/CMakeLists.txt`):

```cmake
# ARM64 architecture detection and compatibility flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  message(STATUS "Building for ARM64 architecture")
  
  # Disable x86 SSE/AVX intrinsics
  target_compile_definitions(gik_matlab_solver PRIVATE 
    __SSE2__=0 
    __AVX__=0 
    __AVX2__=0
  )
  
  # Enable ARM NEON optimizations
  target_compile_options(gik_matlab_solver PRIVATE 
    -march=armv8-a 
    -mtune=cortex-a78
  )
  
  # Add stub header to include path
  target_include_directories(gik_matlab_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
```

### 2. **POSIX Time Functions Missing**

**Problem**: Linker errors for `coderTimeClockGettimeMonotonic` and `coderInitTimeFunctions`.

**Solution**: Created stub implementation at `ros2/gik9dof_solver/src/coder_posix_time_stubs.cpp`:

```cpp
#include "../matlab_codegen/include/coder_posix_time.h"
#include <time.h>

int coderInitTimeFunctions(double* const aFrequency) {
    if (aFrequency != nullptr) {
        *aFrequency = 1.0;  // nanosecond precision
    }
    return 0;
}

int coderTimeClockGettimeMonotonic(coderTimespec* const aCoderTimespec, 
                                   double aFrequency) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return errno;
    }
    
    aCoderTimespec->tv_sec = static_cast<double>(ts.tv_sec);
    aCoderTimespec->tv_nsec = static_cast<double>(ts.tv_nsec);
    
    return 0;
}
```

## Build Process on AGX Orin

### Prerequisites

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install \
    build-essential \
    cmake \
    libeigen3-dev \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs
```

### Build Commands

```bash
# Navigate to ROS2 workspace
cd ~/gikWBC9DOF/ros2

# Clean previous build (if any)
rm -rf build/ install/ log/

# Source ROS2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or build solver package only
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Expected output**:
```
Starting >>> gik9dof_solver
Building for ARM64 architecture
Finished <<< gik9dof_solver [41.5s]

Summary: 1 package finished [41.9s]
```

### Run the Solver

```bash
# Source workspace
source ~/gikWBC9DOF/ros2/install/setup.bash

# Run solver node
ros2 run gik9dof_solver gik9dof_solver_node
```

## Performance Considerations

### SSE Intrinsics Impact

The MATLAB-generated code originally used x86 SSE2 intrinsics for vectorized operations. On ARM64:

1. **Compiler flags disable SSE code paths** (`__SSE2__=0`)
2. **MATLAB uses scalar fallback implementations**
3. **ARM NEON optimizations enabled** (`-march=armv8-a`)

**Expected performance**: Slightly slower than native x86 SSE, but ARM NEON should compensate for most operations.

### Collision Detection

The solver was generated with `generate_code_noCollision.m`, which explicitly disables collision checking:
- ✅ Pure IK calculations only
- ✅ No collision avoidance overhead
- ✅ Stub functions never called

## Verification

### Check Topics

```bash
ros2 topic list

# Expected topics:
# /gik9dof/solver_diagnostics
# /gik9dof/target_trajectory
# /hdas/feedback_arm_left
# /motion_target/target_joint_state_arm_left
# /odom_wheel
```

### Test Solver (Optional)

```bash
cd ~/gikWBC9DOF
chmod +x test_solver_arm64.sh
./test_solver_arm64.sh
```

This script will:
1. Check if solver node is running
2. Verify topic connections
3. Send a test trajectory
4. Check for valid IK solution output

## Known Limitations

1. **No collision avoidance**: Solver computes IK without checking for self-collision or obstacles
2. **SSE stubs**: Using compatibility layer instead of native SSE intrinsics
3. **Performance not benchmarked**: Actual solve times on ARM64 vs x86 not yet measured

## Troubleshooting

### Build Fails with SSE Errors

If you see errors like `'_mm_loadu_pd' was not declared`:
1. Ensure `emmintrin.h` stub is in `ros2/gik9dof_solver/include/`
2. Check CMakeLists.txt includes ARM64 detection logic
3. Verify `-D__SSE2__=0` is set for ARM64 builds

### Linker Errors for Time Functions

If you see `undefined reference to 'coderTimeClockGettimeMonotonic'`:
1. Ensure `coder_posix_time_stubs.cpp` exists in `src/`
2. Verify it's added to `add_executable()` in CMakeLists.txt

### Solver Produces Incorrect Results

If solver computes but gives bad IK solutions:
1. Run the test script to verify basic functionality
2. Check solver diagnostics for error codes
3. May need to regenerate code in MATLAB with ARM64 target

## Files Modified for ARM64 Support

```
ros2/gik9dof_solver/
├── CMakeLists.txt                      # Added ARM64 detection and flags
├── include/
│   └── emmintrin.h                     # NEW: SSE intrinsics compatibility stub
└── src/
    ├── coder_posix_time_stubs.cpp      # NEW: POSIX time function implementations
    ├── collisioncodegen_stubs.cpp      # (existing collision stubs)
    └── gik9dof_solver_node.cpp         # (unchanged)
```

## Future Improvements

1. **Benchmark performance**: Compare ARM64 vs x86 solve times
2. **NEON intrinsics**: Replace SSE stubs with native ARM NEON implementations
3. **MATLAB regeneration**: Regenerate code with ARM64 target in MATLAB (if/when supported)
4. **Collision support**: Add optional collision checking if needed

## Credits

- MATLAB Coder: Code generation (R2024b)
- ARM compatibility fixes: October 2025
- Target platform: NVIDIA AGX Orin (Jetpack 6.0)

---

Last updated: October 7, 2025
