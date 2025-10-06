# MATLAB Codegen Directory Structure

## Overview
This document explains the two different codegen directories and what should be tracked in Git.

## Directory Structure

### 1. `codegen/arm64_realtime/` - MATLAB Build Output (Mostly Ignored)
**Purpose:** MATLAB Coder's native output directory for ARM64 target

**Contents:**
- `.h` files - Header files (✅ **TRACKED** for reference)
- `.cpp/.c` files - Source implementations (❌ **IGNORED** - duplicates)
- `.obj` files - Object files (❌ **IGNORED** - build artifacts)
- `.lib` files - Static libraries (❌ **IGNORED** - build artifacts)
- `.mat` files - Build metadata (✅ **TRACKED** for documentation)
- `.bat/.mk/.rsp` files - Build scripts (❌ **IGNORED** - build artifacts)
- `interface/` folder - MEX interface (❌ **IGNORED** - not needed)
- `examples/` folder - Example code (❌ **IGNORED** - not needed)
- `html/` folder - Code generation reports (❌ **IGNORED** - regeneratable)

**Git Strategy:** Track headers and metadata for reference, ignore all build artifacts and source implementations (they're duplicated in ROS2 package).

### 2. `ros2/gik9dof_solver/matlab_codegen/` - ROS2 Source Code (TRACKED)
**Purpose:** Actual source code used in ROS2 build system

**Contents:**
- `include/*.h` - All header files (✅ **TRACKED** - required for build)
- `include/*.cpp` - All source files (✅ **TRACKED** - required for build)

**Git Strategy:** Track everything - this is the canonical source used by CMake.

**Status:** ✅ **115 files committed** and tracked

## Why This Structure?

1. **MATLAB Coder generates to** `codegen/arm64_realtime/`
2. **We copy needed files to** `ros2/gik9dof_solver/matlab_codegen/include/`
3. **ROS2 CMake builds from** the `matlab_codegen/include/` directory
4. **Git tracks** the ROS2 version (single source of truth)
5. **Git ignores** build artifacts in `codegen/arm64_realtime/`

## Current .gitignore Rules

```gitignore
# MATLAB Codegen build artifacts
*.obj
*.lib
codegen/arm64_realtime/*.cpp      # Ignore duplicates
codegen/arm64_realtime/*.c        # Ignore duplicates
codegen/arm64_realtime/*.bat      # Ignore build scripts
codegen/arm64_realtime/*.mk       # Ignore makefiles
codegen/arm64_realtime/*.rsp      # Ignore response files
codegen/arm64_realtime/*.tmw      # Ignore MATLAB workspace
codegen/arm64_realtime/*.txt      # Ignore text logs
codegen/arm64_realtime/_clang-format  # Ignore formatting config
codegen/arm64_realtime/interface/*.cpp  # Ignore MEX interface
codegen/arm64_realtime/html/      # Ignore HTML reports
codegen/arm64_realtime/examples/  # Ignore examples
codegen/linux_arm64/              # Ignore other target builds
```

## What IS Tracked in codegen/arm64_realtime/

- ✅ All `.h` header files (70 files) - for reference and documentation
- ✅ `.mat` metadata files (buildInfo, codeInfo, compileInfo) - build configuration
- ✅ `.dmr` descriptor file - code generation metadata

These are kept for:
- Documentation of what MATLAB generated
- Troubleshooting code generation issues
- Reference for future regeneration

## Verification Commands

```powershell
# Check what's tracked in codegen/
git ls-files "codegen/*" | Measure-Object -Line

# Check what's tracked in ROS2 matlab_codegen/
git ls-files "ros2/gik9dof_solver/matlab_codegen/*" | Measure-Object -Line

# See what's ignored in codegen/
git status --ignored codegen/
```

## Summary

✅ **Correct Setup:**
- The `.gitignore` rules are **CORRECT**
- Build artifacts are properly ignored
- Source code is tracked in ROS2 package (115 files)
- Reference headers are tracked in codegen/ (70 files)
- No duplication of source files in Git

**Status:** Repository is properly configured! 🎉
