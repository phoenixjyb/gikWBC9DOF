# 🚀 Planner ARM64 Regeneration - Quick Start

## ✅ Progress Status

| Phase | Task | Status | Script |
|-------|------|--------|--------|
| 1 | Backup current code | ✅ Done | Manual (20251009_132850) |
| 2 | Verify WSL environment | ✅ Done | Manual |
| 3 | Clean planner output | ✅ Done | Manual |
| 4 | **Generate with Linux MATLAB** | ⏳ **Ready to run** | `run_planner_codegen.ps1` |
| 5 | Verify generated code | ⏳ Pending | `run_planner_verify.ps1` |
| 6 | Copy to ROS2 | ⏳ Pending | `run_planner_copy_to_ros2.ps1` |
| 7 | Final verification | ⏳ Pending | `run_planner_verify.ps1` |
| 8 | Git commit | ⏳ Pending | Manual |

## 🎯 How to Execute

### **Option 1: Automatic (Recommended)** ⭐
Run everything with one command:
```powershell
.\RUN_ALL_PLANNER_REGEN.ps1
```
This will:
- Generate planner with Linux MATLAB (5-15 min)
- Verify code quality
- Copy to ROS2
- Run final checks
- Show git commit commands

### **Option 2: Manual (Step-by-step)**
Run phases individually:

```powershell
# Phase 4: Generate planner
.\run_planner_codegen.ps1
# ⏱️ Takes 5-15 minutes

# Phase 5: Verify generated code
.\run_planner_verify.ps1

# Phase 6: Copy to ROS2
.\run_planner_copy_to_ros2.ps1

# Phase 7: Final verification
.\run_planner_verify.ps1

# Phase 8: Git commit
git add codegen/planner_arm64/ ros2/gik9dof_solver/src/generated/planner/
git commit -m "feat: Regenerate planner_arm64 with Linux MATLAB"
git push origin codegencc45-main
```

## 📋 What Gets Changed

### **Before (Current - Oct 7):**
```
codegen/planner_arm64/
├── Generated: Oct 7, 2025 19:32
├── Platform: Windows MATLAB R2024a
└── Files: 78 (.cpp, .h, .obj)

ros2/.../planner/
├── Updated: Oct 7, 2025 19:32
└── Platform: Windows MATLAB
```

### **After (Target - Oct 9+):**
```
codegen/planner_arm64/
├── Generated: Oct 9, 2025 (today)
├── Platform: Linux MATLAB R2024a ✅
└── Files: 78 (.cpp, .h, .o - ELF ARM64)

ros2/.../planner/
├── Updated: Oct 9, 2025 (today)
└── Platform: Linux MATLAB ✅
```

## ⚡ Quick Command Summary

```powershell
# Fastest way - run everything
.\RUN_ALL_PLANNER_REGEN.ps1

# Or step-by-step
.\run_planner_codegen.ps1          # Generate (5-15 min)
.\run_planner_verify.ps1           # Verify
.\run_planner_copy_to_ros2.ps1     # Copy to ROS2
git add codegen/ ros2/             # Commit
git commit -m "Regenerate planner with Linux MATLAB"
git push
```

## 🔍 What to Expect

### **Phase 4 Output (Codegen):**
```
========================================
Planner ARM64 Code Generation (Linux)
========================================
Running MATLAB codegen...
This will take 5-15 minutes.

[MATLAB compilation progress...]

✅ Generated files:
   .cpp files: ~30
   .h files:   ~30
   .o files:   ~18

Binary format check:
HybridAStarPlanner.o: ELF 64-bit LSB relocatable, ARM aarch64
OccupancyGrid2D.o:    ELF 64-bit LSB relocatable, ARM aarch64

✅ Phase 4 Complete: Planner generated with Linux MATLAB
```

### **Phase 5-7 Output (Verification):**
```
File counts:
  .cpp files: 30 ✅
  .h files:   30 ✅
  .o files:   18 ✅
  Total:      78 ✅

Key files check:
  ✅ HybridAStarPlanner.cpp
  ✅ HybridAStarPlanner.h
  ✅ OccupancyGrid2D.cpp
  ✅ OccupancyGrid2D.h

Toolchain Status:
  Planner:     Linux MATLAB ✅
  GIK Solver:  Linux MATLAB ✅
  Consistency: ✅ Both from same MATLAB
```

## ⚠️ Important Notes

1. **Backup Preserved:**
   - `codegen/planner_arm64_backup_20251009_132850/`
   - `ros2/.../planner_backup_20251009_132850/`

2. **Estimated Time:**
   - Total: ~20-30 minutes
   - Phase 4 (codegen): 5-15 min
   - Other phases: 5-10 min

3. **Requirements:**
   - WSL with Linux MATLAB R2024a ✅
   - ~2 GB free space ✅
   - MATLAB Coder license ✅

4. **Rollback:**
   If anything goes wrong:
   ```powershell
   Copy-Item codegen/planner_arm64_backup_20251009_132850 codegen/planner_arm64 -Recurse -Force
   ```

## 📖 Documentation

- **Full Plan:** `PLANNER_REGEN_PLAN.md` (8 phases, detailed)
- **Status:** `PLANNER_SYNC_STATUS.md` (problem analysis)
- **Scripts:** All `.ps1` files (automation)

## ✅ Success Criteria

After completion, verify:
- [ ] `codegen/planner_arm64/` has ~78 files dated Oct 9+
- [ ] All `.o` files are `ELF 64-bit ARM aarch64`
- [ ] No Windows `.obj` files present
- [ ] ROS2 planner folder updated
- [ ] Timestamps match GIK solver (~Oct 9, 2025)
- [ ] Changes committed to git

## 🚀 Ready to Start!

**Run this now:**
```powershell
.\RUN_ALL_PLANNER_REGEN.ps1
```

Then sit back and wait 20-30 minutes! ☕

---
**Created:** October 9, 2025
**Next Action:** Run `RUN_ALL_PLANNER_REGEN.ps1`
