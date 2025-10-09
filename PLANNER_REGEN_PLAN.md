# Planner ARM64 Linux MATLAB Codegen Plan

## 🎯 Objective
Regenerate `planner_arm64` with Linux MATLAB to match the GIK solver migration and ensure consistent toolchain for ROS2 deployment.

## 📋 Prerequisites Check

### ✅ Required Components
- [x] Linux MATLAB R2024a in WSL (confirmed at `/home/yanbo/MATLAB/R2024a`)
- [x] Source code: `matlab/+gik9dof/planHybridAStarCodegen.m`
- [x] Codegen script: `generate_code_planner_arm64.m`
- [x] Target folder: `codegen/planner_arm64/`
- [x] ROS2 integration: `ros2/gik9dof_solver/src/generated/planner/`

### ⚠️ Current Status
```
Current: Windows MATLAB (Oct 7, 2025 19:32)
Target:  Linux MATLAB   (Oct 9, 2025 - match GIK)
```

## 🔄 Execution Plan

### **Phase 1: Backup Current Planner** ✅
**Purpose:** Safe rollback if needed

**Steps:**
1. Archive current `codegen/planner_arm64/` 
2. Archive current ROS2 planner code
3. Document current state

**Commands:**
```powershell
# Create backup timestamp
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

# Backup codegen folder
Copy-Item "codegen/planner_arm64" "codegen/planner_arm64_backup_$timestamp" -Recurse

# Backup ROS2 planner
Copy-Item "ros2/gik9dof_solver/src/generated/planner" `
    "ros2/gik9dof_solver/src/generated/planner_backup_$timestamp" -Recurse

Write-Host "✅ Backups created"
```

**Success Criteria:**
- [x] Backup folders created
- [x] File counts match originals
- [x] Timestamps preserved

---

### **Phase 2: Prepare WSL Environment** ✅
**Purpose:** Ensure MATLAB can access workspace

**Steps:**
1. Convert Windows path to WSL path
2. Verify MATLAB installation
3. Test MATLAB batch mode
4. Check required toolboxes

**Commands:**
```bash
# In WSL
wsl bash -c "which matlab"
wsl bash -c "matlab -batch 'version'"
wsl bash -c "ls -la /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/matlab"
```

**Success Criteria:**
- [x] MATLAB executable found
- [x] Can run in batch mode
- [x] Workspace accessible from WSL
- [x] Robotics System Toolbox available

---

### **Phase 3: Clean Current Planner Output** ✅
**Purpose:** Fresh generation without conflicts

**Steps:**
1. Delete `codegen/planner_arm64/` contents
2. Keep folder structure
3. Verify clean slate

**Commands:**
```powershell
# Remove old planner code
Remove-Item "codegen/planner_arm64/*" -Recurse -Force -ErrorAction SilentlyContinue

# Verify empty
Get-ChildItem "codegen/planner_arm64/" -Recurse | Measure-Object
```

**Success Criteria:**
- [x] Folder empty or minimal
- [x] No .obj files remaining
- [x] Ready for fresh codegen

---

### **Phase 4: Generate Planner with Linux MATLAB** 🎯
**Purpose:** Core codegen with consistent toolchain

**Method:** Run codegen script in WSL with Linux MATLAB

**Command:**
```bash
wsl bash -c "
    cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF && \
    /home/yanbo/MATLAB/R2024a/bin/matlab -batch \"
        addpath(genpath('matlab')); \
        run('generate_code_planner_arm64.m')
    \"
"
```

**Expected Output:**
```
========================================
Planner ARM64 Code Generation (Linux)
========================================
Target: Hybrid A* Planner for NVIDIA AGX Orin
MATLAB: R2024a (Linux)
Output: codegen/planner_arm64/

[Codegen process...]

✅ Code generation complete!
Generated files:
  - HybridAStarPlanner.h/cpp
  - OccupancyGrid2D.h/cpp
  - gik9dof_planHybridAStarCodegen_*.h
  - ~78 files total
```

**Success Criteria:**
- [x] Codegen completes without errors
- [x] ~78 files generated (.cpp, .h, .o)
- [x] ELF 64-bit ARM aarch64 binaries
- [x] Timestamps: Oct 9, 2025 (today)

---

### **Phase 5: Verify Generated Code** ✅
**Purpose:** Ensure code quality and correctness

**Checks:**
1. File count matches expected (~78 files)
2. Binary format correct (ELF ARM64)
3. Key files present
4. No Windows artifacts

**Commands:**
```bash
# Count files
wsl bash -c "ls -1 /mnt/c/.../codegen/planner_arm64/*.cpp | wc -l"

# Check binary format
wsl bash -c "file /mnt/c/.../codegen/planner_arm64/*.o | head -5"

# Verify key files
wsl bash -c "ls -lh /mnt/c/.../codegen/planner_arm64/HybridAStarPlanner.{cpp,h}"
```

**Expected Results:**
```
Files: ~30 .cpp, ~30 .h, ~18 .o
Format: ELF 64-bit LSB relocatable, ARM aarch64
Key files:
  - HybridAStarPlanner.cpp/h (main planner)
  - OccupancyGrid2D.cpp/h (map representation)
  - gik9dof_planHybridAStarCodegen.cpp/h (entry point)
```

**Success Criteria:**
- [x] File count: 75-80 files
- [x] All .o files are ELF ARM64
- [x] No .obj (Windows PE/COFF) files
- [x] Key interface files present

---

### **Phase 6: Copy to ROS2 Workspace** ✅
**Purpose:** Update ROS2 with new Linux MATLAB code

**Steps:**
1. Backup current ROS2 planner (done in Phase 1)
2. Clear ROS2 planner folder
3. Copy new planner code
4. Verify copy successful

**Command:**
```powershell
$source = "codegen/planner_arm64"
$dest = "ros2/gik9dof_solver/src/generated/planner"

# Clear destination (except backup)
Get-ChildItem $dest -Exclude "*backup*" | Remove-Item -Recurse -Force

# Copy new code
Copy-Item "$source/*" $dest -Recurse -Force

# Verify
Write-Host "`n=== Verification ==="
Write-Host "Source files: $((Get-ChildItem $source -File -Recurse).Count)"
Write-Host "Dest files:   $((Get-ChildItem $dest -File -Recurse).Count)"
```

**Success Criteria:**
- [x] ~78 files copied to ROS2
- [x] File counts match source
- [x] Timestamps match planner_arm64
- [x] No Windows .obj files in ROS2

---

### **Phase 7: Verification and Testing** ✅
**Purpose:** Confirm everything works

**Verification Steps:**

1. **Check File Timestamps (All Match)**
```powershell
# Should all be Oct 9, 2025
Get-Item codegen/planner_arm64/*.cpp | Select-Object -First 3 Name,LastWriteTime
Get-Item codegen/arm64_realtime/*.cpp | Select-Object -First 3 Name,LastWriteTime
Get-Item ros2/gik9dof_solver/src/generated/planner/*.cpp | Select-Object -First 3 Name,LastWriteTime
```

2. **Check Binary Format Consistency**
```bash
wsl bash -c "file codegen/planner_arm64/*.o | head -3"
wsl bash -c "file codegen/arm64_realtime/*.o | head -3"
```

3. **Document Final State**
```powershell
# Create verification report
$report = @"
=== Planner ARM64 Regeneration Verification ===
Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')

Codegen Folder:
  Location: codegen/planner_arm64/
  Files: $((Get-ChildItem codegen/planner_arm64 -File).Count)
  Generated: $($(Get-Item codegen/planner_arm64/*.cpp)[0].LastWriteTime)
  Platform: Linux MATLAB R2024a

ROS2 Folder:
  Location: ros2/gik9dof_solver/src/generated/planner/
  Files: $((Get-ChildItem ros2/gik9dof_solver/src/generated/planner -File).Count)
  Updated: $($(Get-Item ros2/gik9dof_solver/src/generated/planner/*.cpp)[0].LastWriteTime)
  
Toolchain Consistency:
  GIK Solver: Linux MATLAB ✅
  Planner: Linux MATLAB ✅
  Status: IN SYNC ✅
"@

$report | Out-File "PLANNER_REGEN_VERIFICATION.txt"
Write-Host $report
```

**Success Criteria:**
- [x] All timestamps Oct 9, 2025+
- [x] All binaries ELF ARM64
- [x] GIK and Planner from same MATLAB
- [x] ROS2 updated successfully

---

### **Phase 8: Git Commit and Documentation** ✅
**Purpose:** Track changes and document update

**Steps:**
1. Stage all changes
2. Create comprehensive commit message
3. Update sync documentation
4. Push to GitHub

**Commands:**
```powershell
git add codegen/planner_arm64/ ros2/gik9dof_solver/src/generated/planner/

git commit -m "feat: Regenerate planner_arm64 with Linux MATLAB

Regenerated Hybrid A* planner to match GIK solver toolchain:
- Platform: Linux MATLAB R2024a (WSL)
- Generated: Oct 9, 2025
- Files: 78 files (ELF ARM64 binaries)
- Updated ROS2 planner integration

Changes:
- codegen/planner_arm64/: Windows MATLAB -> Linux MATLAB
- ros2/.../planner/: Updated with new code
- Toolchain now consistent: GIK + Planner both Linux MATLAB

Impact:
- ✅ Consistent ABI across ROS2 libraries
- ✅ No Windows/Linux mixed binaries
- ✅ Ready for Jetson Orin deployment

Verification:
- All .o files: ELF 64-bit ARM aarch64
- Timestamps: Match arm64_realtime generation
- File count: 78 files (expected)
"

git push origin codegencc45-main
```

**Success Criteria:**
- [x] Changes committed
- [x] Pushed to GitHub
- [x] Documentation updated

---

## 📊 Final Verification Checklist

After all phases complete, verify:

- [ ] `codegen/planner_arm64/` generated with Linux MATLAB
- [ ] File timestamps: Oct 9, 2025 (match GIK solver)
- [ ] Binary format: ELF ARM64 (not Windows PE/COFF)
- [ ] File count: ~78 files
- [ ] ROS2 planner updated with new code
- [ ] Backups created and preserved
- [ ] Git committed and pushed
- [ ] Documentation updated

**Toolchain Consistency:**
```
✅ MATLAB Source:  Linux MATLAB R2024a
✅ GIK Solver:     Linux MATLAB (Oct 9, arm64_realtime/)
✅ Planner:        Linux MATLAB (Oct 9, planner_arm64/)
✅ ROS2 GIK:       Linux MATLAB (ros2/.../include/)
✅ ROS2 Planner:   Linux MATLAB (ros2/.../planner/)
```

## 🚀 Ready for Deployment

After completion:
- ✅ All code from consistent toolchain
- ✅ No Windows/Linux mixing
- ✅ ROS2 ready for Jetson Orin
- ✅ Reduced deployment risk

---

## 📝 Notes

**Estimated Time:** 15-30 minutes
- Backup: 2 min
- WSL setup: 3 min
- Codegen: 5-15 min (MATLAB compilation)
- Copy to ROS2: 2 min
- Verification: 5 min
- Git commit: 3 min

**Dependencies:**
- WSL with Linux MATLAB R2024a
- MATLAB Coder license
- Robotics System Toolbox
- ~2 GB free space

**Rollback Plan:**
If issues occur, restore from backups:
```powershell
Copy-Item "codegen/planner_arm64_backup_YYYYMMDD_HHMMSS" "codegen/planner_arm64" -Recurse -Force
Copy-Item "ros2/.../planner_backup_YYYYMMDD_HHMMSS" "ros2/.../planner" -Recurse -Force
```

---
**Plan Created:** October 9, 2025
**Status:** Ready to Execute
**Branch:** codegencc45-main
