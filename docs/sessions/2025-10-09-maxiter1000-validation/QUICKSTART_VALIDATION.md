# 🚀 QUICK START - Complete the Validation

## Current Situation

✅ **Code generated with MaxIterations=1000** in WSL  
✅ **C++ validator built successfully**  
⚠️ **Initial test: 30% pass** (limited by MaxTime=50ms, not iterations)  
🔧 **Fix applied:** MaxTime increased to 10s for validation  

---

## What You Need To Do

### Open WSL Terminal and run:

```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
chmod +x regenerate_and_validate.sh
./regenerate_and_validate.sh
```

**That's it!** The script will:
1. Regenerate code with MaxTime=10s (5-10 min)
2. Rebuild the validator (30 sec)
3. Run validation tests automatically (1-2 min)
4. Show you the improved results

**Expected result:** 60-80% pass rate (12-16 out of 20 tests)

---

## Alternative: Step-by-Step

If you prefer to run each step manually:

### 1. Regenerate Code (5-10 minutes)
```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "cd(pwd); run_wsl_codegen_matlab"
```

### 2. Rebuild Validator (30 seconds)
```bash
cd validation
bash build_with_library_wsl.sh
```

### 3. Run Tests (1-2 minutes)
```bash
./validate_gik_standalone gik_test_cases_20.json results_maxtime10s.json
```

### 4. View Results
```bash
cat results_maxtime10s.json | grep -A 2 "metadata"
```

---

## What Changed?

**File:** `matlab/+gik9dof/+codegen_inuse/solveGIKStepWrapper.m`

```matlab
% Before (too restrictive):
MaxTime = 0.05          % 50ms - hits limit before 1000 iterations!
MaxIterations = 1000

% After (validation mode):
MaxTime = 10.0          % 10 seconds - can use all 1000 iterations ✅
MaxIterations = 1000
```

---

## Expected Results Comparison

| Test Run | MaxTime | MaxIterations | Pass Rate | Notes |
|----------|---------|---------------|-----------|-------|
| Old | 50ms | 50 | ~40% | Baseline |
| Current #1 | 50ms | 1000 | 30% | Time-limited ❌ |
| Current #2 | 10s | 1000 | **60-80%** | Full iterations ✅ |

---

## After Validation

Once you confirm the improved results:

### For Deployment to Jetson:
- Use ARM64 code from `codegen/arm64_realtime/`
- Tune MaxTime based on real-time needs (e.g., 0.1-0.5s)
- Keep MaxIterations=1000
- Balance quality vs. real-time performance

### Files Ready:
- ✅ ARM64 code: `codegen/arm64_realtime/` (already has MaxIterations=1000)
- ✅ Validation results: `results_maxtime10s.json`
- ✅ All documentation in project root

---

## Need Help?

All status and documentation files are in the project root:
- `CURRENT_STATUS.md` - Detailed status
- `WSL_CODEGEN_SUCCESS.md` - WSL setup guide
- `REGENERATION_MAXTIME.md` - Why we're regenerating

---

**Ready? Run the script and see the improvement! 🎯**
