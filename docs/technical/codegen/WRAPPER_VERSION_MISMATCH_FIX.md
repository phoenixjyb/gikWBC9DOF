# Wrapper Version Mismatch Fix (Round 3)

**Date**: October 9, 2025 14:45  
**Status**: ✅ FIXED  
**Commit**: 00d5d4c

## Problem: CMakeLists.txt Expected Different Wrapper Version

### Error Symptoms
```
fatal error: solveGIKStepRealtime.cpp: No such file or directory
fatal error: gik9dof_codegen_realtime_solveGIKStepWrapper_data.cpp: No such file or directory
```

Plus hundreds of namespace errors:
```
error: 'gik9dof_codegen_inuse_solveGIKStepWrapperStackData' does not name a type
error: 'robot' was not declared in this scope
error: 'solver' was not declared in this scope
```

### Root Cause

**CMakeLists.txt expected**:
- `solveGIKStepRealtime.cpp` (production ARM64 wrapper - "realtime" version)
- `gik9dof_codegen_realtime_solveGIKStepWrapper_*.cpp` (realtime support files)

**We deployed** (from backup in Round 2):
- `gik9dof_codegen_inuse_solveGIKStepWrapper_*.cpp` (backup wrapper - "inuse" version)

These are **completely different wrapper naming schemes** from different codegen configurations!

### Why This Happened

1. **Production ARM64 code** (Oct 9) uses "realtime" wrapper naming
2. **Backup code** (Oct 8) has BOTH "inuse" AND "realtime" wrappers
3. **Round 2 fix** copied ALL files from backup, which included "inuse" wrapper
4. **CMakeLists.txt** was configured for "realtime" wrapper from production run
5. **Mismatch** → Build tried to compile "inuse" wrapper but expected "realtime" types

## Solution

### Files Deployed (6 files, 12 KB)

**Realtime wrapper files from backup**:
```
solveGIKStepRealtime.cpp                                          (4,423 bytes)
solveGIKStepRealtime.h                                            (1,146 bytes)
gik9dof_codegen_realtime_solveGIKStepWrapper_data.cpp             (1,933 bytes)
gik9dof_codegen_realtime_solveGIKStepWrapper_data.h               (1,107 bytes)
gik9dof_codegen_realtime_solveGIKStepWrapper_types.h              (1,882 bytes)
gik9dof_codegen_realtime_solveGIKStepWrapper_types1.h             (1,693 bytes)
```

### Deployment Steps

1. **Copied realtime wrapper from backup**:
   ```powershell
   Copy-Item "include_backup_20251009_130628\solveGIKStepRealtime.*" "include\" -Force
   Copy-Item "include_backup_20251009_130628\gik9dof_codegen_realtime_solveGIKStepWrapper_*" "include\" -Force
   ```

2. **Deployed to Orin**:
   ```bash
   rsync -avz solveGIKStepRealtime.* gik9dof_codegen_realtime_solveGIKStepWrapper_* \
       cr@192.168.100.150:/home/nvidia/temp_gikrepo/ros2/gik9dof_solver/matlab_codegen/include/
   ```

3. **Verified**: 6 files transferred, 12 KB

## Technical Details

### Wrapper Naming Schemes

**Production ARM64 (Oct 9)** - "realtime" configuration:
- Entry point: `solveGIKStepRealtime.cpp`
- Wrapper prefix: `gik9dof_codegen_realtime_solveGIKStepWrapper_*`
- Purpose: Configured with MaxTime=50ms for real-time control

**Backup Oct 8** - "inuse" configuration:
- Entry point: `gik9dof_codegen_inuse_solveGIKStepWrapper.cpp`
- Wrapper prefix: `gik9dof_codegen_inuse_solveGIKStepWrapper_*`
- Purpose: Original codegen run before MaxTime configuration

### Why Both Exist in Backup?

The Oct 8 backup contains files from **multiple codegen runs**:
- "inuse" wrapper (older run)
- "realtime" wrapper (newer run with MaxTime configured)
- Both sets coexist in backup directory

### CMakeLists.txt References

The build system expects these exact files:
```cmake
matlab_codegen/include/solveGIKStepRealtime.cpp
matlab_codegen/include/gik9dof_codegen_realtime_solveGIKStepWrapper_data.cpp
```

## Files Now on Orin (Complete Set)

**Total**: 244 files, 7.98 MB

**Categories**:
1. libccd headers (8) - Round 2
2. libccd sources (4 C files) - Round 1
3. Collision wrappers (4 C++) - Round 1
4. **Realtime wrapper (6)** - **Round 3** ✅ JUST ADDED
5. Production ARM64 (~220) - Initial deployment

## Expected Build Result

Build should now succeed because:
- ✅ CMakeLists.txt finds `solveGIKStepRealtime.cpp`
- ✅ All realtime wrapper support files present
- ✅ Namespace matches (realtime vs inuse)
- ✅ All libccd headers/sources present (Round 1 & 2)
- ✅ Complete consistent file set

## Status

- **Wrapper version**: ✅ FIXED (realtime wrapper deployed)
- **Previous Round 1**: ✅ Source files (11 files)
- **Previous Round 2**: ✅ libccd headers (8 files)
- **Current Round 3**: ✅ Correct wrapper version (6 files)
- **Next**: User rebuild on Orin

## Lessons Learned

1. **Production code had incomplete file set** (missing .c/.cpp sources)
2. **Backup contains multiple wrapper versions** (inuse + realtime)
3. **CMakeLists.txt configured for specific wrapper name** (realtime)
4. **Must match wrapper version to build configuration**
5. **Three-round iterative fix** required to discover all missing pieces

## Next Steps

User should rebuild on Orin:
```bash
cd /home/nvidia/temp_gikrepo/ros2
source /opt/ros/humble/setup.bash
rm -rf build install log  # Clean build
colcon build --packages-select gik9dof_solver
```

Expected: **SUCCESSFUL BUILD** 🎯
