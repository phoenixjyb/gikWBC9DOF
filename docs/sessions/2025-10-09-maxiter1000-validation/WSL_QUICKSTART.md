# WSL Linux Code Generation - Quick Start Guide

## ✅ New Branch Created: `wsl-linux-codegen-maxiter1000`

This branch is dedicated to generating code using MATLAB running **inside WSL** to produce Linux-native binaries.

## Prerequisites

MATLAB must be installed in WSL. You mentioned it's now installed - great!

## Find Your MATLAB Installation

Run this command in WSL to locate MATLAB:

```bash
# Try to find MATLAB
which matlab
# OR check common locations:
ls /usr/local/MATLAB/*/bin/matlab
ls /opt/MATLAB/*/bin/matlab
ls ~/MATLAB/*/bin/matlab
```

Once you find the path (e.g., `/usr/local/MATLAB/R2024b/bin/matlab`), set it:

```bash
export MATLAB_PATH=/usr/local/MATLAB/R2024b/bin/matlab
# OR add to PATH:
export PATH=$PATH:/usr/local/MATLAB/R2024b/bin
```

## Code Generation Steps

### Step 1: Navigate to Project in WSL

```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
```

### Step 2: Run Code Generation

```bash
# If MATLAB is in PATH:
matlab -batch "generate_code_x86_64_noCollision"

# OR with full path:
/usr/local/MATLAB/R2024b/bin/matlab -batch "generate_code_x86_64_noCollision"
```

This will take 5-10 minutes and generate code in:
```
codegen/x86_64_validation_noCollision/
```

### Step 3: Build the Validator

```bash
cd validation
bash build_validation_wsl.sh
```

**Expected Result**: ✅ Build SUCCESS with Linux-native binaries!

### Step 4: Run Validation

```bash
./validate_gik_standalone gik_test_cases_20.json results.json
```

**Expected Result**: 60-80% pass rate (up from 40% with MaxIterations=50)

## Why This Works

### Problem Before:
- Windows MATLAB → Windows .obj files → Cannot link in Linux WSL ❌

### Solution Now:
- Linux MATLAB (in WSL) → Linux .o files → Links successfully ✅

## What's Different on This Branch

1. **Script**: `generate_code_x86_64_noCollision.m`
   - Targets x86-64 Linux
   - Loads robot without using collision constraints
   - MaxIterations=1000 embedded

2. **Build Script**: `validation/build_validation_wsl.sh`
   - Points to `x86_64_validation_noCollision`
   - Excludes collision .cpp files (but this time they'll have Linux .o files anyway)

3. **Validator**: `validation/validate_gik_standalone.cpp`
   - Updated for class-based API (GIKSolver)
   - Ready to test MaxIterations=1000 improvement

## Quick Command Sequence

Here's the complete workflow:

```bash
# 1. Open WSL terminal
wsl

# 2. Navigate to project
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF

# 3. Set MATLAB path (adjust to your installation)
export MATLAB_PATH=/usr/local/MATLAB/R2024b/bin/matlab
# OR if already in PATH, skip this

# 4. Generate code (5-10 min)
$MATLAB_PATH -batch "generate_code_x86_64_noCollision"

# 5. Build validator (1-2 min)
cd validation
bash build_validation_wsl.sh

# 6. Run validation tests
./validate_gik_standalone gik_test_cases_20.json results.json

# 7. Check results
echo "Pass rate should be 60-80% (was 40%)"
```

## Troubleshooting

### If MATLAB not found:
```bash
find /usr/local /opt ~ -name matlab -type f 2>/dev/null | grep bin/matlab
```

### If license issues:
```bash
# Activate MATLAB license for Linux
$MATLAB_PATH -batch "disp(license)"
```

### If code generation fails:
- Check MATLAB has Robotics System Toolbox
- Check MATLAB Coder license is active
- Ensure sufficient disk space (~2GB for generated code)

## Expected Outcome

After successful code generation and build:

```
✓ Linux-native code generated with MaxIterations=1000
✓ Validator builds without linker errors
✓ Validation tests run successfully
✓ Pass rate improves from 40% to 60-80%
```

## Next Steps After Validation

1. Compare results with previous 40% pass rate
2. Analyze which test cases improved
3. If results are good, deploy ARM64 code to Jetson
4. Document the improvement

---

**Ready to start? Please run the commands above in WSL!**

Need help finding MATLAB? Let me know where it's installed and I'll adjust the instructions.
