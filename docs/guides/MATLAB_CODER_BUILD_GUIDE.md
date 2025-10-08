# MATLAB Coder Build Guide

## Critical File Organization Issue

**Problem**: MATLAB Coder splits generated files between two directories:
- Main output: `codegen/gik9dof_arm64_20constraints/` (290 .cpp/.h files)
- Hidden dependencies: `codegen/gik9dof_arm64_20constraints/html/generatedSource/` (additional headers + C sources)

### Files That Must Be Copied

From `html/generatedSource/` to codegen root:

#### 1. Core Headers (.h)
```
tmwtypes.h                  # MATLAB type definitions (CRITICAL!)
```

#### 2. Collision Detection (.hpp + .cpp)
```
collisioncodegen_api.hpp
collisioncodegen_util.hpp
collisioncodegen_ccdExtensions.hpp
collisioncodegen_checkCollision_api.hpp
collisioncodegen_CollisionGeometry.hpp
collisioncodegen_helper.hpp

collisioncodegen_api.cpp
collisioncodegen_ccdExtensions.cpp
collisioncodegen_checkCollision.cpp
collisioncodegen_CollisionGeometry.cpp
```

#### 3. Collision Detection Library - C Sources (.c + .h)
```
ccd_ccd.c
ccd_mpr.c
ccd_polytope.c
ccd_vec3.c
coder_posix_time.c

ccd_*.h (all CCD headers)
```

#### 4. Time API (may need update)
```
CoderTimeAPI.cpp            # May need to overwrite existing version
```

## Build Requirements

### Linux/WSL (Recommended for ARM64 code)
```cmake
# CMakeLists.txt - Collect both .cpp and .c files
file(GLOB CODEGEN_SOURCES 
    "${CODEGEN_DIR}/*.cpp"
    "${CODEGEN_DIR}/*.c"      # CRITICAL: Include C files!
)

# Link libraries
target_link_libraries(test_gik_20constraints m pthread gomp)
# m       = math library
# pthread = POSIX threads
# gomp    = GNU OpenMP
```

### Windows/MSVC
- Same file copying required
- Link: `tmwtypes.h` + collision sources
- May need OpenMP library

## Automated Fix Script

### PowerShell (Windows)
```powershell
# Copy missing files from html/generatedSource to codegen root
$src = ".\codegen\gik9dof_arm64_20constraints\html\generatedSource"
$dst = ".\codegen\gik9dof_arm64_20constraints"

# Copy all missing dependencies
Copy-Item "$src\tmwtypes.h" -Destination $dst
Copy-Item "$src\collision*.hpp" -Destination $dst
Copy-Item "$src\collision*.cpp" -Destination $dst
Copy-Item "$src\ccd_*.h" -Destination $dst
Copy-Item "$src\*.c" -Destination $dst
Copy-Item "$src\CoderTimeAPI.cpp" -Destination $dst -Force
```

### Bash (Linux/WSL)
```bash
#!/bin/bash
SRC="codegen/gik9dof_arm64_20constraints/html/generatedSource"
DST="codegen/gik9dof_arm64_20constraints"

# Copy missing dependencies
cp $SRC/tmwtypes.h $DST/
cp $SRC/collision*.hpp $DST/
cp $SRC/collision*.cpp $DST/
cp $SRC/ccd_*.h $DST/
cp $SRC/*.c $DST/
cp -f $SRC/CoderTimeAPI.cpp $DST/
```

## Why This Happens

MATLAB Coder generates an HTML report in `html/` with:
- Full source code listing
- Compilation info
- Dependencies

The `html/generatedSource/` subfolder contains **complete source** including:
- External library interfaces (collision detection)
- Platform-specific implementations (time functions)
- Type definitions needed by generated code

**These files are NOT copied to the main output directory automatically!**

## Verification Steps

After copying files, verify:

```bash
# Check for tmwtypes.h
ls codegen/gik9dof_arm64_20constraints/tmwtypes.h

# Check for collision sources
ls codegen/gik9dof_arm64_20constraints/collision*.cpp | wc -l
# Expected: 4 files

# Check for C sources
ls codegen/gik9dof_arm64_20constraints/*.c | wc -l
# Expected: 5 files (ccd_*.c + coder_posix_time.c)
```

## Build Order

1. **Generate code** (MATLAB)
2. **Copy missing files** (script above)
3. **Configure CMake** (include .c files in glob)
4. **Build** (WSL/Linux recommended for ARM64)
5. **Test** (run test suite)

## Performance Expectations

**Target**: ≤50ms per solve
**Achieved**: 14.78ms average (3.4x better!)

- Single constraint: ~54ms (first run overhead)
- Subsequent solves: ~15-30ms
- 100-iteration benchmark: 14.78ms average

## Next Steps

After successful C++ build:
1. ✅ Verify MATLAB baseline comparison
2. ✅ Document build process (this file!)
3. 🔄 ROS2 integration with 20-constraint interface
4. 🔄 Deploy to AGX Orin target

## References

- Generated code: `codegen/gik9dof_arm64_20constraints/`
- Test suite: `test_cpp/test_gik_20constraints.cpp`
- Build script: `test_cpp/run_tests_wsl.ps1`
- CMake config: `test_cpp/CMakeLists.txt`

---
**Last Updated**: October 8, 2025
**Status**: Build system validated, all tests passing ✅
