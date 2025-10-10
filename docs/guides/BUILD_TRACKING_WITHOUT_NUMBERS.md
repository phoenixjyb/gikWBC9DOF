# Build Verification Without Build Numbers

**Problem:** How to know if your build is current without explicit build numbers?

**Solution:** Multiple verification strategies

---

## 🎯 Quick Answer (TL;DR)

**Use Git Commit Hash as your "build number"**

Every build should be tagged with:
- Git commit hash (e.g., `be54d17`)
- Build timestamp
- Git branch name

---

## 🚀 Immediate Solutions

### Option 1: Timestamp Comparison (Fastest)

```powershell
# Compare build vs source timestamps
$buildTime = (Get-Item "codegen/arm64_realtime/GIKSolver.cpp").LastWriteTime
$sourceTime = (Get-ChildItem "matlab" -Recurse -Include "*.m" | 
               Sort-Object LastWriteTime -Descending | 
               Select-Object -First 1).LastWriteTime

if ($buildTime -gt $sourceTime) {
    Write-Host "✓ Build is current"
} else {
    Write-Host "⚠️ Rebuild needed"
}
```

**Pros:** Instant, no dependencies  
**Cons:** Doesn't account for git history

---

### Option 2: Git Commit Tracking (Recommended)

**During codegen, save commit info:**
```powershell
# Add to your codegen script
git rev-parse HEAD > codegen/arm64_realtime/SOURCE_COMMIT.txt
git rev-parse --short HEAD > codegen/arm64_realtime/SOURCE_COMMIT_SHORT.txt
Get-Date -Format "yyyy-MM-dd HH:mm:ss" > codegen/arm64_realtime/BUILD_TIME.txt
```

**To verify build:**
```powershell
$buildCommit = Get-Content "codegen/arm64_realtime/SOURCE_COMMIT_SHORT.txt"
$currentCommit = git rev-parse --short HEAD

Write-Host "Build from: $buildCommit"
Write-Host "Current:    $currentCommit"

if ($buildCommit -eq $currentCommit) {
    Write-Host "✓ Build is current"
} else {
    Write-Host "⚠️ Source has changed since build"
    git log --oneline "$buildCommit..HEAD" -- matlab/
}
```

**Pros:** Accurate, tracks exact source version  
**Cons:** Requires git, need to modify codegen process

---

### Option 3: Embed Version in Code (Most Professional)

**Create a header file during codegen:**

```cpp
// build_info.h (auto-generated)
#ifndef BUILD_INFO_H
#define BUILD_INFO_H

#define BUILD_GIT_COMMIT    "be54d17abcd1234"
#define BUILD_GIT_SHORT     "be54d17"
#define BUILD_TIMESTAMP     "2025-10-10 14:23:45"
#define BUILD_BRANCH        "codegencc45-main"

inline void print_build_info() {
    std::cout << "Build: " << BUILD_GIT_SHORT 
              << " (" << BUILD_TIMESTAMP << ")" << std::endl;
}

#endif
```

**In your ROS2 node:**
```cpp
#include "build_info.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(node->get_logger(), 
                "GIK9DOF Solver v%s built %s", 
                BUILD_GIT_SHORT, BUILD_TIMESTAMP);
    
    // ... rest of code
}
```

**Benefits:**
- ✅ Version visible in logs
- ✅ Can query at runtime
- ✅ Survives deployment
- ✅ Professional debugging tool

---

## 📋 Pre-Built Scripts Created

I've created these scripts for you:

### 1. `scripts/deployment/get_build_info.ps1`
- Generates BUILD_INFO.txt with git commit, timestamp, user, machine
- Run before deployment to tag your build

**Usage:**
```powershell
.\scripts\deployment\get_build_info.ps1
```

### 2. `scripts/deployment/check_build_current.ps1`
- Compares source vs build timestamps
- Shows which files changed since build
- Exit code 0 = current, 1 = outdated

**Usage:**
```powershell
.\scripts\deployment\check_build_current.ps1
if ($LASTEXITCODE -ne 0) {
    Write-Host "Rebuild needed!"
}
```

### 3. `scripts/codegen/codegen_with_version.ps1`
- Wrapper for MATLAB codegen
- Embeds git commit and timestamp
- Creates build_info.h and BUILD_INFO.txt

**Usage:**
```powershell
.\scripts\codegen\codegen_with_version.ps1 -CodegenScript "generate_code_gik_arm64.m"
```

### 4. `docs/guides/BUILD_VERIFICATION_GUIDE.md`
- Complete reference with all methods
- Copy-paste ready commands
- Troubleshooting guide

---

## 🏗️ Recommended Workflow

### 1. **Before Building**
```powershell
# Record current git state
$commit = git rev-parse --short HEAD
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$buildID = "$commit-$timestamp"

Write-Host "Build ID: $buildID"
```

### 2. **During Build**
```powershell
# Save build info (add to your codegen script)
@{
    commit = git rev-parse HEAD
    short = git rev-parse --short HEAD
    branch = git rev-parse --abbrev-ref HEAD
    timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    buildID = $buildID
} | ConvertTo-Json | Out-File "codegen/arm64_realtime/build_info.json"
```

### 3. **After Building**
```powershell
# Verify build succeeded
if (Test-Path "codegen/arm64_realtime/GIKSolver.cpp") {
    Write-Host "✓ Build created: $buildID"
    
    # Tag in git (optional)
    # git tag "build-$buildID"
}
```

### 4. **Before Deployment**
```powershell
# Check if build is current
.\scripts\deployment\check_build_current.ps1

if ($LASTEXITCODE -eq 0) {
    # Create deployment package with build info
    $deployName = "gik9dof_deployment_$buildID.zip"
    # ... zip files ...
}
```

### 5. **On Target System**
```bash
# Check deployed version
cat ~/gikWBC9DOF/ros2/gik9dof_solver/BUILD_INFO.txt

# Or in ROS2 logs
ros2 run gik9dof_solver gik9dof_solver_node
# Look for: "GIK9DOF Solver vbe54d17 built 2025-10-10"
```

---

## 🎯 Your Current Build Status

Run this now to check your system:

```powershell
# Quick check
if (Test-Path "codegen/arm64_realtime/SOURCE_COMMIT.txt") {
    $buildCommit = Get-Content "codegen/arm64_realtime/SOURCE_COMMIT.txt"
    $currentCommit = git rev-parse HEAD
    
    if ($buildCommit -eq $currentCommit) {
        Write-Host "✓ Your build is CURRENT" -ForegroundColor Green
    } else {
        Write-Host "⚠️ Your build is from commit: $($buildCommit.Substring(0,7))" -ForegroundColor Yellow
        Write-Host "   Current commit: $($currentCommit.Substring(0,7))" -ForegroundColor Yellow
        Write-Host "   Rebuild recommended" -ForegroundColor Yellow
    }
} else {
    Write-Host "⚠️ No build version info found" -ForegroundColor Yellow
    Write-Host "   Build status unknown - consider rebuild" -ForegroundColor Yellow
}
```

---

## 🔧 Integration with Existing Tools

### Add to `RUN_CODEGEN.m` (MATLAB)
```matlab
% At start of codegen
gitCommit = system('git rev-parse HEAD');
gitShort = system('git rev-parse --short HEAD');
buildTime = datestr(now, 'yyyy-mm-dd HH:MM:SS');

% Save to file
fid = fopen('codegen/arm64_realtime/SOURCE_COMMIT.txt', 'w');
fprintf(fid, '%s', strtrim(gitCommit));
fclose(fid);

fprintf('Building from commit: %s\n', strtrim(gitShort));
```

### Add to `deploy_to_orin.ps1`
```powershell
# Before SCP transfer
if (-not (Test-Path "codegen/arm64_realtime/SOURCE_COMMIT.txt")) {
    Write-Host "⚠️ No build version info - deploy anyway? (y/n)"
    $response = Read-Host
    if ($response -ne 'y') { exit 1 }
}

# Show what's being deployed
$buildCommit = Get-Content "codegen/arm64_realtime/SOURCE_COMMIT.txt"
Write-Host "Deploying build: $($buildCommit.Substring(0,7))"
```

---

## 📊 Comparison: Build Tracking Methods

| Method | Speed | Accuracy | Survives Deployment | Auto-Update |
|--------|-------|----------|---------------------|-------------|
| Timestamp | ⚡ Instant | ⭐⭐ | ❌ No | ✅ Auto |
| Git Commit | ⚡ Fast | ⭐⭐⭐⭐⭐ | ✅ Yes | ❌ Manual |
| Embedded Header | ⚡ Fast | ⭐⭐⭐⭐⭐ | ✅ Yes | ❌ Manual |
| BUILD_INFO.txt | ⚡ Instant | ⭐⭐⭐⭐ | ✅ Yes | ❌ Manual |
| Git Tag | ⚡ Fast | ⭐⭐⭐⭐⭐ | ✅ Yes | ❌ Manual |

**Recommendation:** Use **Git Commit** + **Embedded Header** for best results

---

## ✅ Action Items

1. **Immediate** (today):
   - Run `.\scripts\deployment\check_build_current.ps1`
   - Add build info to your next codegen run

2. **Next codegen** (before rebuilding):
   - Add these lines to your codegen script:
     ```powershell
     git rev-parse HEAD > codegen/arm64_realtime/SOURCE_COMMIT.txt
     Get-Date > codegen/arm64_realtime/BUILD_TIME.txt
     ```

3. **Long-term** (when time permits):
   - Integrate version embedding into codegen
   - Add build info display to ROS2 node startup
   - Update deployment scripts to verify versions

---

**Bottom Line:** Git commit hash + timestamp = your "build number"

Track it, embed it, deploy it, verify it! 🎯
