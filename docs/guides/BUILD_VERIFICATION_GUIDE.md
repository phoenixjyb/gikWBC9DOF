# Quick Build Verification Commands

## Method 1: Compare timestamps (Quick!)
```powershell
# Check when codegen was last built
$buildTime = (Get-Item "codegen/arm64_realtime/GIKSolver.cpp").LastWriteTime

# Check when source was last modified
$sourceTime = (Get-ChildItem "matlab" -Recurse -Include "*.m" | 
              Sort-Object LastWriteTime -Descending | 
              Select-Object -First 1).LastWriteTime

Write-Host "Source last modified: $sourceTime"
Write-Host "Build last generated: $buildTime"

if ($buildTime -gt $sourceTime) {
    Write-Host "✓ Build is current" -ForegroundColor Green
} else {
    Write-Host "⚠️ Build is outdated - rebuild needed" -ForegroundColor Yellow
}
```

## Method 2: Check git diff
```powershell
# Get commit hash of build
$buildCommit = git log --format="%H" --before="$buildTime" -1

# Check if source changed since then
git diff --name-only $buildCommit HEAD -- matlab/

# If output is empty = build is current
# If output shows files = rebuild needed
```

## Method 3: Check file in deployment package
```powershell
# If you have a deployment ZIP
$zipFile = Get-ChildItem -Filter "gik9dof_deployment_*.zip" | 
            Sort-Object LastWriteTime -Descending | 
            Select-Object -First 1

Write-Host "Latest deployment: $($zipFile.Name)"
Write-Host "Created: $($zipFile.LastWriteTime)"

# Compare with current source
$latestSource = (Get-ChildItem "matlab" -Recurse -Include "*.m" | 
                 Sort-Object LastWriteTime -Descending | 
                 Select-Object -First 1).LastWriteTime

if ($zipFile.LastWriteTime -gt $latestSource) {
    Write-Host "✓ Deployment package is current"
} else {
    Write-Host "⚠️ Source newer than deployment - repackage needed"
}
```

## Method 4: Check ROS2 node version (on Orin/WSL)
```bash
# Add this to your ROS2 node startup
ros2 run gik9dof_solver gik9dof_solver_node --version

# Or check logs
ros2 topic echo /rosout | grep "Build"
```

## Method 5: Git-based verification
```powershell
# Show last commit that touched MATLAB source
git log -1 --format="%h %ai %s" -- matlab/

# Show last commit that touched codegen output
git log -1 --format="%h %ai %s" -- codegen/arm64_realtime/

# Compare: if codegen commit is older, rebuild needed
```

## Best Practice: Version File Workflow

1. **Before codegen:**
```powershell
# Save current state
git rev-parse HEAD > codegen/arm64_realtime/SOURCE_COMMIT.txt
Get-Date -Format "yyyy-MM-dd HH:mm:ss" > codegen/arm64_realtime/BUILD_TIME.txt
```

2. **To verify build:**
```powershell
# Check what commit was used for build
$buildCommit = Get-Content codegen/arm64_realtime/SOURCE_COMMIT.txt
$currentCommit = git rev-parse HEAD

if ($buildCommit -eq $currentCommit) {
    Write-Host "✓ Build matches current source" -ForegroundColor Green
} else {
    Write-Host "⚠️ Build from different commit:" -ForegroundColor Yellow
    Write-Host "  Build:   $buildCommit"
    Write-Host "  Current: $currentCommit"
    git log --oneline $buildCommit..HEAD
}
```

## Automated Check Script

Save this as `scripts/check_if_rebuild_needed.ps1`:
```powershell
$sourceCommit = Get-Content "codegen/arm64_realtime/SOURCE_COMMIT.txt" -ErrorAction SilentlyContinue
$currentCommit = git rev-parse HEAD

if (-not $sourceCommit) {
    Write-Host "❌ No build version info found - rebuild recommended" -ForegroundColor Red
    exit 1
}

if ($sourceCommit -ne $currentCommit) {
    Write-Host "⚠️ Source changed since build - REBUILD REQUIRED" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Changes since build:"
    git log --oneline "$sourceCommit..HEAD" -- matlab/
    exit 1
}

Write-Host "✓ Build is current" -ForegroundColor Green
exit 0
```

## Usage in Your Workflow

**Before deploying:**
```powershell
# Quick check
.\scripts\check_if_rebuild_needed.ps1

# If fails, rebuild
if ($LASTEXITCODE -ne 0) {
    Write-Host "Running codegen..."
    matlab -batch "run('scripts/codegen/generate_code_gik_arm64.m')"
}
```

## On Target System (Orin)

**Check deployed version:**
```bash
# In your deployment directory
cat ~/gikWBC9DOF/ros2/gik9dof_solver/BUILD_INFO.txt

# Compare with local
scp cr@orin:~/gikWBC9DOF/ros2/gik9dof_solver/BUILD_INFO.txt /tmp/
diff BUILD_INFO.txt /tmp/BUILD_INFO.txt
```
