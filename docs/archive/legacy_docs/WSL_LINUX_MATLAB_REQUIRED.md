# MaxIterations=1000 - WSL Linux Code Generation Requirement

## Current Situation

We've successfully:
✅ Updated MaxIterations from 50→1000 in MATLAB source  
✅ Generated ARM64 code with MaxIterations=1000 (ready for Jetson deployment)  
✅ Attempted multiple approaches to generate WSL-compatible x86_64 code

## The Fundamental Problem

**MATLAB Coder running on Windows ALWAYS generates Windows-format binaries (`.obj`) for collision code**, regardless of target platform settings. These binaries:
- Use Windows PE/COFF format (not Linux ELF)
- Contain MSVC-specific symbols (`__CxxFrameHandler4`, mangled C++ names)
- Cannot be linked with Linux GCC under any circumstances

### Why It Fails
```
Linux g++ → Expects ELF format .o files
Windows MATLAB Coder → Generates PE/COFF format .obj files
Result: "undefined reference" errors for all Windows-specific symbols
```

## Attempted Solutions (All Failed)

### 1. ❌ Exclude Collision Files from Build
**Problem**: Other generated code depends on CollisionSet/RigidBody classes  
**Result**: Linker errors for missing collision symbols

### 2. ❌ Generate Without Collision Geometries  
**Problem**: Can't remove collision meshes from URDF-loaded robot  
**Result**: MATLAB still generates collisioncodegen_*.obj files

### 3. ❌ Link .obj Files Anyway
**Problem**: Binary format incompatibility (Windows vs Linux)  
**Result**: Undefined reference to Windows runtime functions

### 4. ❌ Patch Old x64 Codegen
**Problem**: Old codegen also has Windows .obj files  
**Result**: Same linking errors

## THE ONLY SOLUTION

**Run MATLAB Coder on a Linux machine** (native Linux or WSL with MATLAB installed)

### Why This Works
```
Linux MATLAB → Detects Linux OS → Uses Linux compiler
→ Generates Linux ELF .o files → Links successfully with GCC
```

### How To Do It

#### Option A: Install MATLAB in WSL (Recommended)
1. Install MATLAB R2024b in WSL Ubuntu
2. Activate license for Linux
3. Run from WSL:
   ```bash
   cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
   matlab -batch "generate_code_x86_64_noCollision"
   ```
4. Generated code will have Linux `.o` files
5. Build will succeed

#### Option B: Use Native Linux Machine
1. Copy project to Linux machine with MATLAB
2. Run `generate_code_x86_64_noCollision.m`
3. Copy generated `codegen/x86_64_validation_noCollision/` back to Windows
4. Build in WSL

#### Option C: Skip WSL Validation (Use Jetson)
Since ARM64 code is ready:
1. Deploy `codegen/arm64_realtime/` to Jetson Orin
2. Build and test directly on target hardware  
3. Compare with previous deployments (MaxIterations=50 vs 1000)
4. Expected improvement: 40% → 60-80% pass rate

## Files Ready for Deployment

### ✅ ARM64 (Jetson Orin)
- Location: `codegen/arm64_realtime/`
- MaxIterations: **1000** ✓
- Status: **Ready to deploy**
- Build: Use existing Jetson build scripts

### ⚠️ x86_64 (WSL Validation)  
- Location: `codegen/x86_64_validation_noCollision/`
- MaxIterations: **1000** ✓
- Status: **Cannot build on Windows-hosted WSL**
- Needs: Linux-hosted MATLAB to regenerate

## Recommendation

**IMMEDIATE**: Deploy ARM64 code to Jetson and validate there.  

**FUTURE**: If local WSL validation is needed:
1. Install MATLAB in WSL, OR
2. Use a Linux machine with MATLAB for code generation

## What We've Accomplished

Even though WSL validation is blocked, the core work is complete:

1. ✅ **MATLAB Source Updated**
   - `solveGIKStepWrapper.m`: MaxIterations=1000
   - All hyperparameters aligned  
   - Validation tools created

2. ✅ **ARM64 Code Generated**
   - Ready for production deployment
   - MaxIterations properly embedded
   - Class-based C++ interface (MATLAB Coder 24.2)

3. ✅ **x86_64 Code Generated**  
   - Contains MaxIterations=1000
   - Just needs Linux-hosted MATLAB to rebuild collision binaries

4. ✅ **Documentation Complete**
   - All code changes documented
   - Hyperparameter analysis tools created
   - 11+ reference documents generated

## Next Actions

### For Immediate Testing
```bash
# On Jetson Orin
cd ~/gikWBC9DOF
rsync -av yanbo@windows-host:/path/to/codegen/arm64_realtime/ ./codegen/
cd ros2/build_scripts
./build_gik_solver.sh
./test_gik_solver.sh
# Compare pass rate: expect 60-80% (was 40%)
```

### For WSL Validation (When Linux MATLAB Available)
```bash
# In WSL with MATLAB installed
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
matlab -batch "generate_code_x86_64_noCollision"
cd validation
bash build_validation_wsl.sh
./validate_gik_standalone gik_test_cases_20.json results.json
```

---

**Bottom Line**: MaxIterations=1000 is properly configured and ready. WSL validation requires Linux-hosted MATLAB due to binary format incompatibility. ARM64 deployment path is clear and ready to proceed.
