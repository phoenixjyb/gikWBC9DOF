# Complete Build & Compilation Guide
## From MATLAB to ROS2 on Jetson AGX Orin

**Last Updated:** October 10, 2025  
**Target Platform:** NVIDIA Jetson AGX Orin (ARM64)  
**Build Environment:** WSL Ubuntu 22.04 + Linux MATLAB R2024a

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Understanding the Codegen Folders](#understanding-the-codegen-folders)
3. [Prerequisites](#prerequisites)
4. [Complete Build Workflow](#complete-build-workflow)
5. [Step-by-Step Instructions](#step-by-step-instructions)
6. [Verification](#verification)
7. [Troubleshooting](#troubleshooting)
8. [Related Documentation](#related-documentation)

---

## 🎯 Overview

This project uses **MATLAB Coder** to generate ARM64 C++ code from MATLAB functions, which is then compiled into ROS2 nodes running on Jetson AGX Orin. The complete system has **4 main components** that must all be generated and deployed:

```
MATLAB Source Code
      ↓
MATLAB Coder (WSL)
      ↓
4 Generated Code Components (ARM64 C++)
      ↓
ROS2 Package (colcon build)
      ↓
Jetson AGX Orin Deployment
```

---

## 📁 Understanding the Codegen Folders

### Directory Structure

```
codegen/
├── arm64_realtime/          # ✅ REQUIRED - Main GIK solver (real-time)
├── planner_arm64/           # ✅ REQUIRED - Path planner
├── trajectory_smoothing/    # ✅ REQUIRED - Trajectory smoothing
├── velocity_smoothing/      # ✅ REQUIRED - Velocity smoothing
├── archive/                 # ⚠️  Old versions (keep for reference)
├── planner_arm64_backup_*/  # ❌ CAN DELETE - Dated backups
└── velocity_smoothing/      # (appears twice - same folder)
```

### Component Breakdown

| Component | Purpose | Source | Output | Used By |
|-----------|---------|--------|--------|---------|
| **arm64_realtime** | Main inverse kinematics solver with real-time constraints | `matlab/+gik9dof/+codegen_inuse/solveGIKStepWrapper.m` | `GIKSolver.cpp/h` + object files | `gik9dof_solver` ROS2 node |
| **planner_arm64** | Hybrid A* path planning for mobile base | `matlab/planners/*.m` | `planner*.cpp/h` | Path planning module |
| **trajectory_smoothing** | Smooths joint trajectories using quintic splines | `matlab/smoothVelocityCommand.m` | `smoothVelocityCommand.cpp/h` | Trajectory controller |
| **velocity_smoothing** | Real-time velocity command smoothing | `matlab/smoothVelocityCommand.m` | `smoothVelocityCommand.cpp/h` | Velocity controller |

### Backup Folders - Can Be Deleted

These are safe to remove:
```powershell
# Delete backup folders
Remove-Item "codegen\planner_arm64_backup_20251009_132832" -Recurse -Force
Remove-Item "codegen\planner_arm64_backup_20251009_132850" -Recurse -Force

# Keep archive/ for reference (has different versions)
```

---

## ✅ Prerequisites

### On Windows/WSL

1. **WSL2 with Ubuntu 22.04**
   ```powershell
   wsl --list --verbose
   # Should show Ubuntu-22.04 and VERSION 2
   ```

2. **Linux MATLAB R2024a in WSL**
   ```bash
   /home/yanbo/MATLAB/R2024a/bin/matlab -batch "version"
   # Should output: 24.1.0.2537033 (R2024a)
   ```

3. **Git** (for version tracking)
   ```bash
   git --version
   ```

4. **Required WSL Tools**
   ```bash
   sudo apt-get update
   sudo apt-get install file bc dos2unix
   ```

### On Jetson AGX Orin

1. **ROS2 Humble** (or your target ROS2 version)
2. **ARM64 toolchain** (for cross-compilation if needed)
3. **SSH access** for deployment

---

## 🔄 Complete Build Workflow

### The 4-Stage Process

```
┌─────────────────────────────────────────────────────────────┐
│ STAGE 1: MATLAB Codegen (4 components)                       │
│ - arm64_realtime      [5-15 min]                            │
│ - planner_arm64       [5-10 min]                            │
│ - trajectory_smoothing [2-5 min]                            │
│ - velocity_smoothing   [2-5 min]                            │
│ Total: ~20-40 minutes                                        │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 2: Copy to ROS2 Package                               │
│ - Copy generated C++ to ros2/gik9dof_solver/                │
│ - Organize into src/ and include/ directories               │
│ Total: ~1 minute                                             │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 3: Build ROS2 Package (on Orin or cross-compile)     │
│ - colcon build --packages-select gik9dof_solver             │
│ - Links all 4 components into single executable             │
│ Total: ~3-5 minutes                                          │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 4: Deploy & Test                                      │
│ - Copy to Orin via SSH/SCP                                  │
│ - Source ROS2 workspace                                      │
│ - Launch nodes                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 Step-by-Step Instructions

### Method 1: Automated Build (Recommended) 🚀

#### Step 1.1: Run Complete Codegen with Version Tracking

```powershell
# From Windows PowerShell in project root
.\scripts\codegen\run_codegen_wsl_versioned.ps1
```

This generates `arm64_realtime/` with build version info.

**Expected Output:**
```
✓ Build ID: 20251010_133906
✓ Git Commit: be54d17
✓ Files: 150+ .cpp, 200+ .h, 50+ .o (ARM64 ELF)
```

#### Step 1.2: Generate Planner Code

```powershell
.\scripts\codegen\run_planner_codegen.ps1
```

**Expected Output:**
```
✅ Codegen completed successfully!
   .cpp files: 30+
   .h files:   50+
   .o files:   30+
```

#### Step 1.3: Generate Trajectory Smoothing

```bash
# In WSL
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
./scripts/codegen/run_trajectory_smoothing_codegen.sh
```

**Expected Output:**
```
Generated: codegen/trajectory_smoothing/
  - smoothTrajectoryCommand.cpp
  - smoothTrajectoryCommand.h
  - Supporting files
```

#### Step 1.4: Generate Velocity Smoothing

```bash
# In WSL
./scripts/codegen/run_velocity_smoothing_codegen.sh
```

**Expected Output:**
```
Generated: codegen/velocity_smoothing/
  - smoothVelocityCommand.cpp
  - smoothVelocityCommand.h
  - Supporting files
```

#### Step 2: Copy All Components to ROS2

```powershell
# Automated copy script
.\scripts\deployment\run_planner_copy_to_ros2.ps1
```

This copies all 4 components to:
```
ros2/gik9dof_solver/
├── src/generated/gik/           # arm64_realtime
├── src/planner/                 # planner_arm64
├── src/trajectory_smoothing/    # trajectory_smoothing
└── src/velocity_smoothing/      # velocity_smoothing
```

#### Step 3: Build ROS2 Package

On Jetson AGX Orin:
```bash
cd ~/gikWBC9DOF/ros2
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Or cross-compile in WSL (advanced):
```bash
# Set up cross-compilation toolchain first
colcon build --packages-select gik9dof_solver \
  --cmake-args -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake
```

#### Step 4: Deploy to Orin

```powershell
# From Windows
scp -r ros2/gik9dof_solver orin_user@orin_ip:~/gikWBC9DOF/ros2/
```

#### Step 5: Test on Orin

```bash
# On Orin
source ~/gikWBC9DOF/ros2/install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

---

### Method 2: Manual Build (Detailed Control) 🔧

#### Step 1: Generate ARM64 Realtime Code

```bash
# In WSL
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "
    addpath(genpath('matlab'));
    cd('scripts/codegen');
    run('generate_code_arm64.m');
"
```

**Verify:**
```bash
ls -lh codegen/arm64_realtime/GIKSolver.cpp
file codegen/arm64_realtime/*.o | head -3
# Should show: ELF 64-bit LSB relocatable, ARM aarch64
```

#### Step 2: Generate Planner Code

```bash
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "
    addpath(genpath('matlab'));
    run('generate_code_planner_arm64.m');
"
```

**Verify:**
```bash
ls codegen/planner_arm64/*.cpp | wc -l
# Should show: 30+ files
```

#### Step 3: Generate Smoothing Components

```bash
# Trajectory smoothing
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "
    addpath(genpath('matlab'));
    run('scripts/codegen/generate_code_trajectory_smoothing.m');
"

# Velocity smoothing
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "
    addpath(genpath('matlab'));
    run('scripts/codegen/generate_code_velocity_smoothing.m');
"
```

#### Step 4: Manual Copy to ROS2

```bash
# Create directories
mkdir -p ros2/gik9dof_solver/src/generated/gik
mkdir -p ros2/gik9dof_solver/src/planner
mkdir -p ros2/gik9dof_solver/src/trajectory_smoothing
mkdir -p ros2/gik9dof_solver/src/velocity_smoothing

mkdir -p ros2/gik9dof_solver/include/generated/gik
mkdir -p ros2/gik9dof_solver/include/planner
mkdir -p ros2/gik9dof_solver/include/trajectory_smoothing
mkdir -p ros2/gik9dof_solver/include/velocity_smoothing

# Copy source files
cp codegen/arm64_realtime/*.cpp ros2/gik9dof_solver/src/generated/gik/
cp codegen/arm64_realtime/*.h ros2/gik9dof_solver/include/generated/gik/

cp codegen/planner_arm64/*.cpp ros2/gik9dof_solver/src/planner/
cp codegen/planner_arm64/*.h ros2/gik9dof_solver/include/planner/

cp codegen/trajectory_smoothing/*.cpp ros2/gik9dof_solver/src/trajectory_smoothing/
cp codegen/trajectory_smoothing/*.h ros2/gik9dof_solver/include/trajectory_smoothing/

cp codegen/velocity_smoothing/*.cpp ros2/gik9dof_solver/src/velocity_smoothing/
cp codegen/velocity_smoothing/*.h ros2/gik9dof_solver/include/velocity_smoothing/
```

#### Step 5: Build and Deploy (same as Method 1 Steps 3-5)

---

## ✅ Verification

### Check Codegen Output

```powershell
# On Windows
.\scripts\deployment\check_build_current_wsl.sh

# Expected output:
# ✓ BUILD IS CURRENT
# Build from: be54d17
# Current:    be54d17
```

### Verify All 4 Components Exist

```powershell
# PowerShell
$components = @(
    "codegen\arm64_realtime\GIKSolver.cpp",
    "codegen\planner_arm64\plannerRRTStar.cpp",
    "codegen\trajectory_smoothing\smoothTrajectoryCommand.cpp",
    "codegen\velocity_smoothing\smoothVelocityCommand.cpp"
)

foreach ($file in $components) {
    if (Test-Path $file) {
        $size = (Get-Item $file).Length / 1KB
        Write-Host "✓ $file (${size} KB)" -ForegroundColor Green
    } else {
        Write-Host "✗ $file MISSING" -ForegroundColor Red
    }
}
```

### Verify ROS2 Package Structure

```bash
# On Orin or in WSL
cd ros2/gik9dof_solver
tree -L 3 src/ include/

# Expected structure:
# src/
# ├── generated/gik/       (arm64_realtime files)
# ├── planner/             (planner_arm64 files)
# ├── trajectory_smoothing/ (smoothing files)
# └── velocity_smoothing/   (velocity files)
```

### Test Build on Orin

```bash
# On Orin
cd ~/gikWBC9DOF/ros2
colcon build --packages-select gik9dof_solver 2>&1 | tee build.log

# Check for errors
grep -i error build.log
# Should be empty or show only warnings
```

### Runtime Verification

```bash
# On Orin, run node and check version
ros2 run gik9dof_solver gik9dof_solver_node

# Should output:
# [gik9dof_solver]: GIK9DOF Solver be54d17 (2025-10-10 13:39:06)
# [gik9dof_solver]: Build: be54d17 (ARM64 WSL Linux MATLAB)
```

---

## 🔧 Troubleshooting

### Issue: "Build is outdated"

**Cause:** Source code changed since last codegen

**Solution:**
```powershell
# Rebuild all 4 components
.\scripts\codegen\run_codegen_wsl_versioned.ps1
.\scripts\codegen\run_planner_codegen.ps1
wsl bash scripts/codegen/run_trajectory_smoothing_codegen.sh
wsl bash scripts/codegen/run_velocity_smoothing_codegen.sh
```

### Issue: "Missing .cpp files in ROS2"

**Cause:** Copy script not run after codegen

**Solution:**
```powershell
.\scripts\deployment\run_planner_copy_to_ros2.ps1
```

### Issue: "Undefined reference" errors during colcon build

**Cause:** Not all 4 components copied to ROS2

**Check:**
```bash
find ros2/gik9dof_solver/src -name "*.cpp" | wc -l
# Should be 200+ files (all 4 components)
```

**Solution:** Re-run copy script for missing components

### Issue: "Wrong architecture" error on Orin

**Cause:** Built for x86_64 instead of ARM64

**Check:**
```bash
file codegen/arm64_realtime/*.o | head -1
# Should show: ARM aarch64, NOT x86-64
```

**Solution:** Regenerate using ARM64 scripts (`generate_code_arm64.m`)

### Issue: Windows line endings in bash scripts

**Cause:** Creating scripts in Windows editor

**Solution:**
```bash
# In WSL
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
find scripts -name "*.sh" -exec dos2unix {} \;
# Or manually:
sed -i 's/\r$//' scripts/codegen/*.sh
```

---

## 📚 Related Documentation

### Getting Started
- [README.md](../../README.md) - Project overview
- [docs/guides/WSL_BUILD_VERSIONING.md](WSL_BUILD_VERSIONING.md) - Build version tracking

### Build & Deployment
- [docs/guides/BUILD_VERIFICATION_GUIDE.md](BUILD_VERIFICATION_GUIDE.md) - Verify builds
- [docs/guides/BUILD_TRACKING_WITHOUT_NUMBERS.md](BUILD_TRACKING_WITHOUT_NUMBERS.md) - Version tracking concepts
- [docs/deployment/DEPLOYMENT_GUIDE.md](../deployment/DEPLOYMENT_GUIDE.md) - Deploy to Orin

### Component-Specific
- [docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md](../technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md) - Velocity smoothing
- [docs/technical/smoothing/TRAJECTORY_SMOOTHING_PLAN.md](../technical/smoothing/TRAJECTORY_SMOOTHING_PLAN.md) - Trajectory smoothing
- [docs/planning/HYBRID_ASTAR_COMPLETE.md](../planning/HYBRID_ASTAR_COMPLETE.md) - Path planning
- [docs/technical/codegen/CODEGEN_AUDIT.md](../technical/codegen/CODEGEN_AUDIT.md) - Codegen analysis

### Scripts Reference
- [scripts/README.md](../../scripts/README.md) - All available scripts
- [scripts/codegen/](../../scripts/codegen/) - Codegen scripts directory
- [scripts/deployment/](../../scripts/deployment/) - Deployment scripts

---

## 🎯 Quick Reference

### Complete Build from Scratch

```powershell
# Windows PowerShell - one-liner for all 4 components
.\scripts\codegen\run_codegen_wsl_versioned.ps1; `
.\scripts\codegen\run_planner_codegen.ps1; `
wsl bash scripts/codegen/run_trajectory_smoothing_codegen.sh; `
wsl bash scripts/codegen/run_velocity_smoothing_codegen.sh; `
.\scripts\deployment\run_planner_copy_to_ros2.ps1
```

### Verify Everything

```powershell
# Check build status
wsl bash scripts/deployment/check_build_current_wsl.sh

# Count files
Get-ChildItem codegen -Recurse -File | Group-Object Extension | Sort-Object Count -Descending
```

### Deploy Everything

```powershell
# Create deployment package with version
$buildID = Get-Content "codegen\arm64_realtime\BUILD_ID.txt"
$deployName = "gik9dof_complete_${buildID}.zip"
Compress-Archive -Path "ros2\*", "codegen\arm64_realtime\BUILD_INFO.txt" -DestinationPath $deployName
Write-Host "Created: $deployName"

# Copy to Orin
scp $deployName orin@orin_ip:~/deployments/
```

---

## 🏁 Summary

**4 Components** → **1 ROS2 Package** → **Jetson AGX Orin**

1. ✅ Generate all 4 components with MATLAB Coder (20-40 min)
2. ✅ Copy to ROS2 package structure (1 min)
3. ✅ Build with colcon (3-5 min)
4. ✅ Deploy to Orin via SSH
5. ✅ Test with version verification

**Total Time:** ~30-50 minutes for complete build from scratch

**Backup Folders:** Safe to delete `planner_arm64_backup_*` folders

---

**Need Help?** Check the [troubleshooting section](#troubleshooting) or consult:
- [docs/README.md](../README.md) - Documentation index
- [scripts/README.md](../../scripts/README.md) - Scripts guide
- [docs/guides/](.) - All guides

**Pro Tip:** Use build version tracking ([WSL_BUILD_VERSIONING.md](WSL_BUILD_VERSIONING.md)) to always know if your deployment matches your source code! 🎯
