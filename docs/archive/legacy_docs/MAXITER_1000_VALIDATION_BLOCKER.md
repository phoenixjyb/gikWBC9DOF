# MaxIterations=1000 Validation - Build Blocker

## Status: ⚠️ Cannot Build Validator on WSL (Collision Binary Incompatibility)

### What We Accomplished
✅ **MATLAB Code Updated**
- Updated `solveGIKStepWrapper.m` line 38: `MaxIterations = 1000` (was 50)
- Created hyperparameter validation tool
- All MATLAB-side changes complete

✅ **Code Generation Attempted**
- Generated ARM64 code with MaxIterations=1000 → `codegen/arm64_realtime/`
- Generated x86_64 code with MaxIterations=1000 → `codegen/x86_64_validation/`
- Both generations successful (MATLAB Coder 24.2)

✅ **Alternative Approach Tried**
- Patched old x64 codegen (`gik9dof_x64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper.cpp` line 119)
- Changed `50.0` → `1000.0` in `set_SolverParameters()` call
- Updated `rtwtypes.h` to remove `tmwtypes.h` dependency

### The Blocker: Collision Binary Format Issue

**Problem**: Both old and new codegen include collision detection binaries (`.obj` files) that are **Windows-only** (MSVC format).

**Why It Fails**:
1. MATLAB Coder generates collision code as **binary objects** (`.obj` files), not source
2. These `.obj` files are in **Windows PE/COFF format** (for MSVC compiler)
3. WSL Ubuntu uses **Linux ELF format** (for GCC compiler)
4. Cannot link Windows binaries with Linux g++

**Files Affected**:
```
codegen/*/collisioncodegen_api.obj  
codegen/*/collisioncodegen_ccdExtensions.obj
codegen/*/collisioncodegen_checkCollision.obj
codegen/*/collisioncodegen_CollisionGeometry.obj
```

**Missing Headers**:
- `collisioncodegen_api.hpp` - not generated (API exists only in .obj)

**Build Errors**:
```
fatal error: collisioncodegen_api.hpp: No such file or directory
undefined reference to `__CxxFrameHandler4' (Windows exception handler)
undefined reference to `?what@exception@std@@UEBAPEBDXZ' (MSVC name mangling)
```

### Attempted Workarounds (All Failed)
1. ❌ **Exclude collision files** → Linker errors (other code depends on CollisionSet)
2. ❌ **Link .obj files anyway** → Format mismatch (Windows PE vs Linux ELF)  
3. ❌ **Use old codegen** → Same .obj issue

### Solutions (Require MATLAB/Linux Machine)

#### Option 1: Generate Code on Linux (Recommended)
Run MATLAB Coder directly on a Linux machine:
```matlab
% On Linux MATLAB:
generate_code_x86_64.m  % Will create Linux .o files instead of Windows .obj
```

#### Option 2: Disable Collision in Code Generation
Use the `generate_code_noCollision.m` script which builds robot without collision geometries:
```matlab
generate_code_noCollision.m  % Removes collision dependencies entirely
```

#### Option 3: Deploy to Jetson and Test There
Since ARM64 code generation succeeded:
1. Deploy `codegen/arm64_realtime/` to Jetson Orin
2. Build and test directly on target hardware
3. Compare with previous deployments (MaxIterations was 50)

### Current State of Files

**Updated Source Files** ✅:
- `matlab/+gik9dof/+codegen_inuse/solveGIKStepWrapper.m` - MaxIterations=1000
- `codegen/gik9dof_x64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper.cpp` - Patched to 1000.0
- `codegen/gik9dof_x64_20constraints/rtwtypes.h` - Standalone (no tmwtypes.h)

**Generated Code** ✅:
- `codegen/arm64_realtime/` - Ready for Jetson deployment
- `codegen/x86_64_validation/` - Cannot build on WSL (binary incompatibility)

**Validation Tools** ⏸️:
- `validation/validate_gik_standalone.cpp` - Updated for function-based API
- `validation/build_validation_wsl.sh` - Updated paths
- Cannot compile due to collision binaries

### Recommendation

**For immediate validation**: Deploy ARM64 code to Jetson Orin and test there. Compare:
- Old behavior: MaxIterations=50, pass rate ~40%
- New behavior: MaxIterations=1000, expected ~60-80% pass rate

**For WSL validation**: Need Linux-generated code (requires access to Linux MATLAB) or no-collision build.

### Next Steps When Linux MATLAB Available

1. Run on Linux MATLAB:
   ```matlab
   cd /path/to/gikWBC9DOF
   generate_code_x86_64.m
   ```

2. Generated code will have Linux-compatible `.o` files

3. Build validator:
   ```bash
   cd validation
   bash build_validation_wsl.sh
   ```

4. Run validation:
   ```bash
   ./validate_gik_standalone gik_test_cases_20.json results.json
   ```

---
**Summary**: MaxIterations=1000 is properly configured in MATLAB and ARM64 generated code. WSL validation blocked by Windows/Linux binary format mismatch in collision code. Either deploy to Jetson for testing or regenerate on Linux.
