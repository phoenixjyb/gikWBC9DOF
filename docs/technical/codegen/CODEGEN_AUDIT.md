# Codegen Folder Audit & Cleanup Recommendations

**Date:** October 9, 2025  
**Analysis:** Complete asset inventory and redundancy check

## Executive Summary

**Current Status:**
- 7 codegen subfolders
- 1,458 total files
- 44.66 MB total size
- **Significant redundancy identified** ⚠️

**Recommendation:**
- **Keep:** 2 folders (arm64_realtime + planner_arm64)
- **Archive:** 3 folders (old versions)
- **Delete:** 2 folders (debug/validation artifacts)
- **Savings:** ~32 MB, 1,172 files removed

---

## Detailed Folder Analysis

### 1. `arm64_realtime/` ✅ **KEEP - PRIMARY PRODUCTION**
- **Purpose:** Current ARM64 code for Jetson Orin deployment
- **Size:** 206 files, 7.44 MB
- **Last Modified:** Oct 9, 2025 12:10 (TODAY - just regenerated)
- **Configuration:**
  - MaxTime = 50ms (real-time)
  - MaxIterations = 1000
  - SIMD: ARM NEON
  - OpenMP: Enabled
  - Collision: Stub implementations (no libccd)
- **Status:** **PRODUCTION READY** - Deploy to Orin

**Key Files:**
```
GIKSolver.h/cpp          - Main solver entry point
buildRobotForCodegen.*   - Robot model (procedural)
generalizedInverseKinematics.* - IK solver core
```

**Embedded Parameters Verified:**
- Line 302: `set_SolverParameters(weight, 0.05, ...)` ← MaxTime=50ms ✅
- Line 313: MaxIterations = 1000.0 ✅

---

### 2. `gik9dof_arm64_20constraints/` ⚠️ **ARCHIVE - DEPRECATED**
- **Purpose:** OLD ARM64 code with libccd collision detection
- **Size:** 390 files, 9.74 MB (LARGEST)
- **Last Modified:** Oct 8, 2025 14:21
- **Configuration:**
  - Full collision detection (libccd integrated)
  - Older MaxIterations settings
  - NOT updated with MaxTime=50ms
- **Status:** **DEPRECATED** - Replaced by arm64_realtime

**Why Archive (not delete):**
- Contains full libccd integration (183 extra files)
- Reference implementation if collision detection needed in future
- Large file count (390 files) due to collision library

**Collision Files Present:**
```
ccd_*.h/cpp              - libccd collision detection library
CollisionGeometry.*      - Full collision geometry
CollisionSet.*           - Collision set management
```

**Recommendation:**
- Move to `codegen/archive/gik9dof_arm64_20constraints_libccd/`
- Keep as reference for future collision work

---

### 3. `gik9dof_x64_20constraints/` ⚠️ **ARCHIVE - DEPRECATED**
- **Purpose:** OLD x86-64 code (Windows cross-compile attempt)
- **Size:** 207 files, 7.29 MB
- **Last Modified:** Oct 8, 2025 14:00
- **Binary Format:** Windows PE/COFF .obj files
- **Status:** **DEPRECATED** - Binary incompatibility with Linux

**Why Deprecated:**
- Generated on Windows MATLAB → PE/COFF format
- Cannot link with Linux WSL or Orin (needs ELF format)
- Replaced by WSL-based generation workflow

**Recommendation:**
- Move to `codegen/archive/gik9dof_x64_windows_deprecated/`
- Keep for historical reference only

---

### 4. `x86_64_validation/` ⚠️ **ARCHIVE - COMPLETED VALIDATION**
- **Purpose:** WSL validation build (OLD - MaxTime=10s)
- **Size:** 209 files, 7.50 MB
- **Last Modified:** Oct 8, 2025 18:40
- **Configuration:**
  - MaxTime = 10s (validation testing)
  - MaxIterations = 1000
  - Linux ELF binaries
- **Status:** **COMPLETED** - Validation done (30% pass rate)

**Recommendation:**
- Move to `codegen/archive/x86_64_validation_maxtime10s/`
- Validation results already documented in `VALIDATION_RESULTS_ANALYSIS.md`

---

### 5. `x86_64_validation_noCollision/` 🔧 **CURRENT VALIDATION (can archive)**
- **Purpose:** Latest WSL validation build
- **Size:** 282 files, 9.72 MB (2nd largest)
- **Last Modified:** Oct 9, 2025 10:20 (this morning)
- **Configuration:**
  - MaxTime = 10s
  - MaxIterations = 1000
  - Collision stubs
- **Status:** **VALIDATION COMPLETE**

**Why Large:**
- 282 files (more than production arm64!)
- Includes validation artifacts
- "noCollision" in name but still 75 more files than production

**Recommendation:**
- Move to `codegen/archive/x86_64_validation_final/`
- Validation complete, results documented
- Not needed for production deployment

---

### 6. `planner_arm64/` ✅ **KEEP - PRODUCTION COMPONENT**
- **Purpose:** Hybrid A* path planner for ARM64
- **Size:** 78 files, 1.47 MB
- **Last Modified:** Oct 7, 2025 19:32
- **Status:** **PRODUCTION READY** - Separate component

**Why Keep:**
- Different functionality (path planning, not IK)
- Small footprint (1.47 MB)
- Production-ready for Orin
- Independent from GIK solver

**Used By:**
- ROS2 path planning node
- Global navigation planning

---

### 7. `test_debug/` ❌ **DELETE - TEMPORARY DEBUG**
- **Purpose:** Debug/test code generation attempt
- **Size:** 86 files, 1.50 MB
- **Last Modified:** Oct 7, 2025 19:26
- **Status:** **TEMPORARY** - Can be deleted

**Why Delete:**
- Debug/test artifact
- Older than production code
- Not referenced by any deployment script
- Can be regenerated if needed

**Recommendation:**
- **DELETE** - Not worth archiving

---

## Redundancy Analysis

### Functional Overlap

| Folder | Purpose | MaxTime | MaxIter | Collision | Binary | Status |
|--------|---------|---------|---------|-----------|--------|--------|
| **arm64_realtime** | **Orin Deploy** | **50ms** | **1000** | **Stub** | **ELF** | **✅ PROD** |
| gik9dof_arm64_20constraints | Orin + Collision | ❌ Old | ❌ Old | Full libccd | ELF | ⚠️ Archive |
| gik9dof_x64_20constraints | Windows x64 | ❌ Old | ❌ Old | Full | PE/COFF | ❌ Archive |
| x86_64_validation | WSL Valid | 10s | 1000 | Stub | ELF | ⚠️ Archive |
| x86_64_validation_noCollision | WSL Valid | 10s | 1000 | Stub | ELF | ⚠️ Archive |
| **planner_arm64** | **Path Plan** | **N/A** | **N/A** | **N/A** | **ELF** | **✅ PROD** |
| test_debug | Debug | ❌ Old | ❌ Old | ❌ | ❌ | ❌ Delete |

### Key Findings

1. **3 versions of x86-64 code** (gik9dof_x64, x86_64_validation, x86_64_validation_noCollision)
   - Only one is ELF-compatible (WSL-generated)
   - Validation complete, no longer needed

2. **2 versions of ARM64 GIK code** (arm64_realtime vs gik9dof_arm64_20constraints)
   - arm64_realtime is current (Oct 9, MaxTime=50ms)
   - gik9dof_arm64_20constraints is deprecated (Oct 8, old params)

3. **1 unique component** (planner_arm64)
   - Different functionality, must keep

---

## Cleanup Plan

### Phase 1: Create Archive (SAFE)

```powershell
# Create archive directory
New-Item -ItemType Directory -Path "codegen/archive"

# Archive deprecated versions
Move-Item "codegen/gik9dof_arm64_20constraints" "codegen/archive/arm64_libccd_oct8/"
Move-Item "codegen/gik9dof_x64_20constraints" "codegen/archive/x64_windows_deprecated/"
Move-Item "codegen/x86_64_validation" "codegen/archive/x64_wsl_validation_oct8/"
Move-Item "codegen/x86_64_validation_noCollision" "codegen/archive/x64_wsl_validation_final/"
```

**Result:**
- 4 folders archived (1,048 files, 34.25 MB)
- Still accessible if needed
- Easy to delete later

### Phase 2: Delete Temporary (AGGRESSIVE)

```powershell
# Delete debug folder
Remove-Item -Recurse -Force "codegen/test_debug"
```

**Result:**
- 86 files deleted, 1.50 MB freed
- Can regenerate if needed

### Phase 3: Final Structure (CLEAN)

```
codegen/
├── arm64_realtime/           ← PRODUCTION (Orin GIK solver)
├── planner_arm64/            ← PRODUCTION (Orin planner)
└── archive/                  ← Historical reference
    ├── arm64_libccd_oct8/
    ├── x64_windows_deprecated/
    ├── x64_wsl_validation_oct8/
    └── x64_wsl_validation_final/
```

**Production folders:** 2 (284 files, 8.91 MB)  
**Archive folders:** 4 (1,048 files, 34.25 MB)  
**Total savings if archive deleted:** 79% size reduction

---

## ROS2 Folder Analysis

### Target Platform: **ARM64 Jetson Orin** ✅

**Evidence:**

1. **Source code comments:**
```cpp
// File: ros2/gik9dof_solver/src/gik9dof_solver_node.cpp
// Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64

// File: ros2/gik9dof_solver/src/stage_b_chassis_plan.cpp
// Target: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64
```

2. **CMakeLists.txt:**
```cmake
# Enable OpenMP for parallel processing (ARM NEON)
find_package(OpenMP REQUIRED)
```

3. **Stubs implementation:**
```cpp
// File: coder_posix_time_stubs.cpp
// These provide minimal implementations for ARM64 builds
// On POSIX systems (Linux/ARM64), we use clock_gettime
```

### ROS2 Folder Structure

```
ros2/
├── gik9dof_solver/          ← GIK solver ROS2 node (ARM64 Orin)
│   ├── matlab_codegen/      ← MATLAB generated code embedded
│   │   └── include/         ← GIK solver source (copied from codegen/)
│   ├── src/                 ← ROS2 C++ node implementation
│   │   ├── gik9dof_solver_node.cpp  ← Main node (Target: Orin)
│   │   ├── stage_b_chassis_plan.cpp ← Planner (Target: Orin)
│   │   └── coder_posix_time_stubs.cpp ← ARM64 stubs
│   └── CMakeLists.txt       ← Build for ARM64
│
├── gik9dof_controllers/     ← Controller nodes
├── gik9dof_msgs/            ← ROS2 message definitions
└── nodes/                   ← Additional nodes
```

### Key Insights

1. **Target Platform:** **Jetson Orin ARM64 ONLY**
   - No WSL-specific code
   - No x86-64 conditionals
   - All comments reference "ARM64" and "Orin"

2. **MATLAB Code Embedding:**
   - `ros2/gik9dof_solver/matlab_codegen/include/` contains GIK solver
   - This is a **COPY** of generated code from `codegen/arm64_realtime/`
   - Must be **manually updated** after regeneration

3. **Build System:**
   - Uses ROS2 colcon build system
   - CMake configured for ARM64 cross-compile or native Orin build
   - OpenMP enabled for NEON SIMD

### Update Workflow

**After regenerating ARM64 code:**

```powershell
# Step 1: Regenerate ARM64 code (DONE TODAY)
matlab -batch "run('scripts/codegen/generate_code_arm64.m')"

# Step 2: Copy to ROS2 workspace (MANUAL STEP NEEDED)
Remove-Item -Recurse "ros2/gik9dof_solver/matlab_codegen/include/*"
Copy-Item "codegen/arm64_realtime/*" "ros2/gik9dof_solver/matlab_codegen/include/"

# Step 3: Rebuild ROS2 package on Orin
# (On Jetson Orin via SSH)
cd ros2
colcon build --packages-select gik9dof_solver
```

⚠️ **ACTION REQUIRED:**
The ROS2 embedded code is **NOT YET UPDATED** with today's MaxTime=50ms changes!

---

## Recommendations Summary

### Immediate Actions (Today)

1. ✅ **Already Done:**
   - Regenerated arm64_realtime with MaxTime=50ms
   - Committed to git

2. ⚠️ **TODO:**
   - Update ROS2 embedded code with new MaxTime=50ms version
   - Clean up codegen folder (archive old versions)

### Cleanup Script

```powershell
# Safe cleanup (moves to archive, doesn't delete)
New-Item -ItemType Directory -Force -Path "codegen/archive"

Move-Item "codegen/gik9dof_arm64_20constraints" `
    "codegen/archive/arm64_libccd_deprecated/"

Move-Item "codegen/gik9dof_x64_20constraints" `
    "codegen/archive/x64_windows_deprecated/"

Move-Item "codegen/x86_64_validation" `
    "codegen/archive/x64_wsl_validation_v1/"

Move-Item "codegen/x86_64_validation_noCollision" `
    "codegen/archive/x64_wsl_validation_v2/"

Remove-Item -Recurse -Force "codegen/test_debug"

Write-Host "✅ Cleanup complete!"
Write-Host "Production: codegen/arm64_realtime/ (206 files)"
Write-Host "Production: codegen/planner_arm64/ (78 files)"
Write-Host "Archived: codegen/archive/ (4 folders)"
```

### ROS2 Update Script

```powershell
# Update ROS2 with latest ARM64 code
$src = "codegen/arm64_realtime"
$dst = "ros2/gik9dof_solver/matlab_codegen/include"

Write-Host "Updating ROS2 with MaxTime=50ms ARM64 code..."
Remove-Item -Recurse -Force "$dst/*"
Copy-Item -Recurse "$src/*" "$dst/"

Write-Host "✅ ROS2 updated!"
Write-Host "Next: Deploy to Orin and rebuild with colcon"
```

---

## Decision Matrix

| Action | Impact | Risk | Recommendation |
|--------|--------|------|----------------|
| Archive old ARM64 | Free 9.74 MB | Low (can restore) | ✅ DO IT |
| Archive x64 Windows | Free 7.29 MB | Low (incompatible) | ✅ DO IT |
| Archive WSL validation | Free 17.22 MB | Low (done) | ✅ DO IT |
| Delete test_debug | Free 1.50 MB | Low (regenerable) | ✅ DO IT |
| Update ROS2 code | Enable 50ms | **HIGH if skip** | ⚠️ **CRITICAL** |

---

## Next Session Checklist

- [ ] Run cleanup script (archive old versions)
- [ ] Update ROS2 embedded code with MaxTime=50ms
- [ ] Test build on Jetson Orin
- [ ] Verify MaxTime=50ms in ROS2 runtime
- [ ] Document ROS2 update procedure in README
- [ ] Delete archived folders after 1 month (optional)

**Estimated time:** 30 minutes  
**Risk level:** Low (all reversible)  
**Impact:** Clean workspace, up-to-date deployment code
