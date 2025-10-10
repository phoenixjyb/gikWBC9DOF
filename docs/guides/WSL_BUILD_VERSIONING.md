# Build Verification for WSL MATLAB Workflow

**Context:** You build with Linux MATLAB in WSL (not Windows MATLAB)  
**Target:** ARM64 Jetson AGX Orin  
**Challenge:** Track build versions across Windows/WSL boundary

---

## 🎯 The Problem

- **Build environment:** WSL Ubuntu 22.04 + Linux MATLAB R2024a
- **Source location:** Windows filesystem (`/mnt/c/Users/yanbo/...`)
- **Git repository:** Windows side
- **Challenge:** How to track which git commit was used for each build?

---

## ✅ The Solution

**Track git commit + timestamp during WSL build process**

### Architecture
```
Windows (Git Repo)
    ↓
WSL (Linux MATLAB)
    ↓ reads git info from Windows
    ↓ saves to build output
Build Artifacts (with version info)
    ↓
Deploy to Orin
```

---

## 🚀 Quick Start

### Check Current Build Status

```powershell
# From Windows PowerShell
wsl bash -c "cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF && bash scripts/deployment/check_build_current_wsl.sh"
```

**Output:**
```
=== Build Currency Check (WSL) ===

Build Information:
  Build from: be54d17
  Build time: 2025-10-10 14:23:45

Current State:
  Current:    be54d17

✓ BUILD IS CURRENT
  Build matches current source
```

---

## 📋 Your New Workflow

### Option 1: **Automated** (Recommended)

**Single command with version tracking:**

```powershell
# From Windows PowerShell
.\scripts\codegen\run_codegen_wsl_versioned.ps1
```

**What it does:**
1. ✅ Collects git commit info
2. ✅ Creates BUILD_INFO.txt
3. ✅ Creates build_info.h (C++ header)
4. ✅ Runs MATLAB codegen in WSL
5. ✅ Verifies ARM64 binaries
6. ✅ Shows build summary

**Output files:**
```
codegen/arm64_realtime/
├── GIKSolver.cpp           # Generated code
├── GIKSolver.h
├── *.o                     # ARM64 binaries
├── BUILD_INFO.txt          # ← Human-readable version info
├── build_info.h            # ← C++ header with version
├── SOURCE_COMMIT.txt       # ← Git commit hash
├── SOURCE_COMMIT_SHORT.txt # ← Short hash (be54d17)
└── BUILD_TIME.txt          # ← Timestamp
```

---

### Option 2: **Manual Steps**

**Step 1: Save build info (before codegen)**
```bash
# In WSL
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
bash scripts/codegen/save_build_info_wsl.sh
```

**Step 2: Run MATLAB codegen**
```bash
# In WSL
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "cd('/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF'); run('scripts/codegen/generate_code_gik_arm64.m')"
```

**Step 3: Verify build**
```bash
# In WSL
bash scripts/deployment/check_build_current_wsl.sh
```

---

## 🔍 Checking Build Status

### From Windows

```powershell
# Quick check
if (Test-Path "codegen\arm64_realtime\SOURCE_COMMIT_SHORT.txt") {
    $buildCommit = Get-Content "codegen\arm64_realtime\SOURCE_COMMIT_SHORT.txt"
    $currentCommit = git rev-parse --short HEAD
    
    Write-Host "Build: $buildCommit"
    Write-Host "Current: $currentCommit"
    
    if ($buildCommit -eq $currentCommit) {
        Write-Host "✓ Build is CURRENT" -ForegroundColor Green
    } else {
        Write-Host "⚠️ Build is OUTDATED" -ForegroundColor Yellow
    }
} else {
    Write-Host "⚠️ No build version info" -ForegroundColor Yellow
}
```

### From WSL

```bash
# Comprehensive check
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
bash scripts/deployment/check_build_current_wsl.sh
```

---

## 📦 Deployment with Version Info

### Copy to ROS2 with Build Info

```powershell
# Standard copy
.\scripts\deployment\run_planner_copy_to_ros2.ps1

# Verify build info copied
if (Test-Path "ros2\gik9dof_solver\src\generated\gik\BUILD_INFO.txt") {
    Write-Host "✓ Build info included in ROS2 package"
    Get-Content "ros2\gik9dof_solver\src\generated\gik\BUILD_INFO.txt" | Select-Object -First 10
}
```

### Deploy to Orin with Build Info

Your deployment should include `BUILD_INFO.txt`:

```powershell
# Create deployment package
$buildID = Get-Content "codegen\arm64_realtime\BUILD_ID.txt"
$zipName = "gik9dof_deployment_$buildID.zip"

# Package should include BUILD_INFO.txt
Compress-Archive -Path "codegen\arm64_realtime\*", "ros2\*" -DestinationPath $zipName
```

**On Orin after deployment:**
```bash
# Check deployed version
cat ~/gikWBC9DOF/ros2/gik9dof_solver/BUILD_INFO.txt

# Compare with local version
ssh orin "cat ~/gikWBC9DOF/ros2/gik9dof_solver/BUILD_INFO.txt" > /tmp/orin_version.txt
diff codegen/arm64_realtime/BUILD_INFO.txt /tmp/orin_version.txt
```

---

## 🔧 Using Build Info in C++ Code

The generated `build_info.h` can be included in your ROS2 node:

```cpp
#include "build_info.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GIK9DOFSolverNode>();
    
    // Log build version on startup
    RCLCPP_INFO(node->get_logger(), 
                "GIK9DOF Solver %s", 
                BuildInfo::get_version_string().c_str());
    
    RCLCPP_INFO(node->get_logger(), 
                "Build: %s (%s)", 
                BuildInfo::GIT_SHORT.c_str(),
                BuildInfo::BUILD_PLATFORM.c_str());
    
    rclcpp::spin(node);
    return 0;
}
```

**Output in logs:**
```
[gik9dof_solver]: GIK9DOF Solver be54d17 (2025-10-10 14:23:45)
[gik9dof_solver]: Build: be54d17 (ARM64 WSL Linux MATLAB)
```

---

## 📊 Your Current Status

Let's check your actual build right now:

```powershell
# Run this to see your current status
wsl bash -c "cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF && bash scripts/deployment/check_build_current_wsl.sh"
```

**Expected result:** ⚠️ **OUTDATED** because `smoothVelocityCommand.m` was modified on Oct 10, 2025 but build is from Oct 9, 2025.

---

## 🎯 Recommended Actions

### Today (Immediate)

1. **Add version tracking to your next build:**
   ```powershell
   .\scripts\codegen\run_codegen_wsl_versioned.ps1
   ```

2. **Always check before deployment:**
   ```bash
   wsl bash scripts/deployment/check_build_current_wsl.sh
   ```

### Going Forward (Best Practice)

1. **Always use versioned codegen:**
   - Use `run_codegen_wsl_versioned.ps1` instead of manual MATLAB calls
   - Ensures every build has version info

2. **Check before every deployment:**
   - Script will warn if build is outdated
   - Shows exactly what changed

3. **Include BUILD_INFO.txt in deployments:**
   - Easy to verify version on target
   - Helps with debugging

---

## 🔍 Troubleshooting

### "No build version info found"

**Cause:** Build was created before version tracking was added

**Solution:**
```bash
# Regenerate version info for existing build
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
bash scripts/codegen/save_build_info_wsl.sh
```

### "Build from different commit"

**Cause:** Source code changed since build

**Solution:**
```powershell
# Rebuild with latest source
.\scripts\codegen\run_codegen_wsl_versioned.ps1
```

### "file command not found"

**Cause:** Missing utilities in WSL

**Solution:**
```bash
# Install required tools
sudo apt-get update
sudo apt-get install file bc
```

---

## 📝 File Locations

### Scripts Created

| Script | Purpose | Platform |
|--------|---------|----------|
| `scripts/codegen/save_build_info_wsl.sh` | Collect git info | WSL |
| `scripts/codegen/run_codegen_wsl_with_version.sh` | Codegen wrapper | WSL |
| `scripts/codegen/run_codegen_wsl_versioned.ps1` | Windows launcher | Windows |
| `scripts/deployment/check_build_current_wsl.sh` | Verify build | WSL |

### Generated Files

| File | Contents | Use |
|------|----------|-----|
| `BUILD_INFO.txt` | Human-readable summary | Documentation, deployment tracking |
| `build_info.h` | C++ header with version | Include in ROS2 node for runtime version |
| `SOURCE_COMMIT.txt` | Full git hash | Exact source tracking |
| `SOURCE_COMMIT_SHORT.txt` | Short hash (7 chars) | Display, logging |
| `BUILD_TIME.txt` | Timestamp | When build was created |
| `BUILD_ID.txt` | Unique ID | File naming, organization |

---

## ✅ Summary

**Problem Solved:** You now have git-based version tracking for WSL MATLAB builds

**Key Features:**
- ✅ Automatic git commit tracking
- ✅ ARM64 binary verification
- ✅ Build currency checking
- ✅ C++ header for runtime version
- ✅ Works across Windows/WSL boundary

**Next Build:**
```powershell
.\scripts\codegen\run_codegen_wsl_versioned.ps1
```

This will track everything automatically! 🎯
