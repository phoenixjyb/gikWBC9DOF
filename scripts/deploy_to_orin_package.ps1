#!/usr/bin/env pwsh
# Deploy Chassis Path Follower to NVIDIA AGX Orin
# Creates deployment package and transfers to Orin

param(
    [string]$OrinIP = "192.168.100.150",
    [string]$OrinUser = "cr",
    [switch]$PackageOnly,
    [switch]$Help
)

if ($Help) {
    Write-Host @"
Deploy Chassis Path Follower to NVIDIA AGX Orin

USAGE:
    .\scripts\deploy_to_orin_package.ps1 [OPTIONS]

OPTIONS:
    -OrinIP <ip>        Orin IP address (default: 192.168.100.150)
    -OrinUser <user>    Orin username (default: nvidia)
    -PackageOnly        Only create zip, don't transfer
    -Help               Show this help message

EXAMPLES:
    # Create package and deploy
    .\scripts\deploy_to_orin_package.ps1

    # Only create package
    .\scripts\deploy_to_orin_package.ps1 -PackageOnly

    # Custom Orin address
    .\scripts\deploy_to_orin_package.ps1 -OrinIP 192.168.1.100

WHAT IT DOES:
    1. Creates deployment package with:
       - ROS2 workspace (all packages)
       - ARM64 codegen files
       - Build/deployment scripts
    2. Transfers to Orin via scp
    3. Provides SSH commands for building on Orin
"@
    exit 0
}

$ErrorActionPreference = "Stop"
$WorkspaceRoot = $PSScriptRoot
if ($WorkspaceRoot -match 'scripts$') {
    $WorkspaceRoot = Split-Path -Parent $WorkspaceRoot
}

Write-Host "===============================================" -ForegroundColor Cyan
Write-Host "  Deploy Chassis Path Follower to Orin" -ForegroundColor Cyan
Write-Host "===============================================" -ForegroundColor Cyan
Write-Host ""

# 1. Create deployment directory structure
Write-Host "[1/5] Creating deployment package..." -ForegroundColor Yellow
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$deployDir = Join-Path $WorkspaceRoot "deployments" "orin_chassis_follower_$timestamp"
$tempDir = Join-Path $deployDir "temp"

if (Test-Path $deployDir) {
    Remove-Item $deployDir -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $tempDir | Out-Null

# 2. Copy ROS2 workspace
Write-Host "  - Copying ROS2 workspace..." -ForegroundColor Gray
$ros2Dest = Join-Path $tempDir "ros2"

# Copy ROS2 directory excluding build artifacts and symlinks
$ros2Src = Join-Path $WorkspaceRoot "ros2"
Get-ChildItem -Path $ros2Src -Directory | Where-Object { $_.Name -notin @("build", "install", "log") } | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination (Join-Path $ros2Dest $_.Name) -Recurse -Force
}

# Copy package.xml files if they exist at root
Get-ChildItem -Path $ros2Src -File | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination $ros2Dest -Force
}

Write-Host "    Copied: gik9dof_msgs, gik9dof_controllers, gik9dof_solver" -ForegroundColor DarkGray

# Note: Codegen files are already included in ros2/gik9dof_controllers/src/matlab_generated/
# No need to copy separate codegen folder

# 3. Create deployment scripts
Write-Host "  - Creating deployment scripts..." -ForegroundColor Gray

# Build script for Orin
$buildScript = @'
#!/bin/bash
# Build script for Orin
set -e

echo "==========================================="
echo "  Building ROS2 Workspace on Orin"
echo "==========================================="
echo ""

cd "$(dirname "$0")/ros2"

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[1/3] Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found!"
    exit 1
fi

# Clean previous build
echo "[2/3] Cleaning previous build..."
rm -rf build/ install/ log/

# Build workspace
echo "[3/3] Building workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "To use the workspace:"
    echo "  source install/setup.bash"
    echo ""
    echo "To launch chassis path follower:"
    echo "  ros2 launch gik9dof_controllers chassis_path_follower_launch.py"
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
'@

# Convert to Unix line endings (LF only)
$buildScript = $buildScript -replace "`r`n", "`n"
Set-Content -Path (Join-Path $tempDir "build_on_orin.sh") -Value $buildScript -NoNewline

# Test script for Orin
$testScript = @'
#!/bin/bash
# Test script for chassis path follower
set -e

cd "$(dirname "$0")/ros2"

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "ERROR: Workspace not built! Run ./build_on_orin.sh first"
    exit 1
fi

echo "==========================================="
echo "  Testing Chassis Path Follower"
echo "==========================================="
echo ""

# Test 1: Check if node is available
echo "[Test 1] Checking node availability..."
if ros2 pkg list | grep -q gik9dof_controllers; then
    echo "  ✅ gik9dof_controllers package found"
else
    echo "  ❌ gik9dof_controllers package not found"
    exit 1
fi

# Test 2: Launch in background
echo ""
echo "[Test 2] Launching node (Mode 2 - Pure Pursuit)..."
echo "  Press Ctrl+C to stop"
echo ""

ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2
'@

# Convert to Unix line endings (LF only)
$testScript = $testScript -replace "`r`n", "`n"
Set-Content -Path (Join-Path $tempDir "test_on_orin.sh") -Value $testScript -NoNewline

# README for deployment
$readmeContent = @"
# Chassis Path Follower - Orin Deployment Package

**Created**: $timestamp  
**Target**: NVIDIA AGX Orin (ARM64)  
**ROS2**: Humble Hawksbill

## Package Contents

\`\`\`
ros2/                          # ROS2 workspace
  ├── gik9dof_msgs/           # Custom message definitions
  ├── gik9dof_controllers/     # Chassis path follower node
  │   └── src/
  │       ├── chassis_path_follower_node.cpp
  │       └── matlab_generated/
  │           └── chassis_path_follower/  # MATLAB Coder generated files (ARM64)
  └── gik9dof_solver/          # GIK 9DOF solver
build_on_orin.sh              # Build script
test_on_orin.sh               # Test script
\`\`\`

**Note**: MATLAB Coder generated files are included in `ros2/gik9dof_controllers/src/matlab_generated/`

## Deployment Steps

### 1. Extract Package
\`\`\`bash
cd ~
unzip orin_chassis_follower_$timestamp.zip
cd orin_chassis_follower_$timestamp
\`\`\`

### 2. Build on Orin
\`\`\`bash
chmod +x build_on_orin.sh
./build_on_orin.sh
\`\`\`

**Expected build time**: ~5 minutes  
**Expected output**: All 3 packages build successfully

### 3. Test the Node
\`\`\`bash
chmod +x test_on_orin.sh
./test_on_orin.sh
\`\`\`

### 4. Manual Launch (Alternative)
\`\`\`bash
cd ros2
source install/setup.bash

# Mode 0: Differentiation controller
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0

# Mode 1: Heading controller
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1

# Mode 2: Pure pursuit (default)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2
\`\`\`

## Testing the Controller

### Publish a Test Path
\`\`\`bash
# Terminal 1: Launch controller
ros2 launch gik9dof_controllers chassis_path_follower_launch.py

# Terminal 2: Publish test path (straight line)
ros2 topic pub /path nav_msgs/msg/Path '{
  header: {frame_id: "map"},
  poses: [
    {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]
}' -1

# Terminal 3: Monitor velocity commands
ros2 topic echo /cmd_vel
\`\`\`

### Monitor Status
\`\`\`bash
# Path following status
ros2 topic echo /path_following_active

# Node info
ros2 node info /chassis_path_follower
\`\`\`

## Controller Modes

| Mode | Name | Description |
|------|------|-------------|
| 0 | Differentiation | Curvature-based control with differentiation |
| 1 | Heading | Heading controller with lookahead |
| 2 | Pure Pursuit | Pure pursuit with adaptive lookahead (default) |

## Parameters

Key parameters (can be set in launch file):
- \`controller_mode\`: 0, 1, or 2
- \`vx_max\`: Maximum linear velocity (default: 1.0 m/s)
- \`wz_max\`: Maximum angular velocity (default: 1.0 rad/s)
- \`lookahead_base\`: Base lookahead distance (default: 0.3 m)
- \`lookahead_gain\`: Velocity-dependent lookahead gain (default: 0.5)
- \`goal_tolerance\`: Distance threshold for goal reached (default: 0.1 m)

See \`ros2/gik9dof_controllers/launch/chassis_path_follower_launch.py\` for all parameters.

## Topics

**Subscribed**:
- \`/path\` (nav_msgs/msg/Path): Path to follow
- \`/odom\` (nav_msgs/msg/Odometry): Current robot pose

**Published**:
- \`/cmd_vel\` (geometry_msgs/msg/Twist): Velocity commands
- \`/path_following_active\` (std_msgs/msg/Bool): Following status

## Troubleshooting

### Build fails
\`\`\`bash
# Clean and rebuild
cd ros2
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
\`\`\`

### Node doesn't start
\`\`\`bash
# Check if package is installed
ros2 pkg list | grep gik9dof

# Check ROS2 environment
env | grep ROS
\`\`\`

### No velocity output
- Check if path was published: \`ros2 topic echo /path\`
- Check if odometry is available: \`ros2 topic echo /odom\`
- Check node logs: \`ros2 node info /chassis_path_follower\`

## Files Modified from Base

This deployment includes fixes for:
1. ✅ Python dependencies (empy 3.3.4)
2. ✅ C++ type mismatches (struct0_T, struct1_T, struct3_T)
3. ✅ Bounded array usage (PathInfo arrays)
4. ✅ Function call signature corrections
5. ✅ Parameter structure alignment

See \`WSL_BUILD_SUCCESS.md\` in the main repository for detailed change log.

## Support

For issues, refer to:
- Main repository: phoenixjyb/gikWBC9DOF
- Branch: codegencc45-main
- Session docs: \`WSL_BUILD_SUCCESS.md\`

---

**Package created**: $timestamp  
**Validated in**: WSL Ubuntu 22.04 + ROS2 Humble  
**Target platform**: NVIDIA AGX Orin (ARM64)
"@

Set-Content -Path (Join-Path $tempDir "README.md") -Value $readmeContent

# 5. Create zip file
Write-Host "  - Creating zip archive..." -ForegroundColor Gray
$zipName = "orin_chassis_follower_$timestamp.zip"
$zipPath = Join-Path $deployDir $zipName

# Use .NET compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::CreateFromDirectory($tempDir, $zipPath)

# Clean temp directory
Remove-Item $tempDir -Recurse -Force

$zipSize = (Get-Item $zipPath).Length / 1MB
Write-Host "  ✅ Package created: $([math]::Round($zipSize, 2)) MB" -ForegroundColor Green
Write-Host ""

# 6. Display package info
Write-Host "[2/5] Package Details:" -ForegroundColor Yellow
Write-Host "  Location: $zipPath" -ForegroundColor White
Write-Host "  Size: $([math]::Round($zipSize, 2)) MB" -ForegroundColor White
Write-Host ""

if ($PackageOnly) {
    Write-Host "[3/5] Package only mode - skipping transfer" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Manual transfer commands:" -ForegroundColor Cyan
    Write-Host "  scp `"$zipPath`" ${OrinUser}@${OrinIP}:~/" -ForegroundColor Gray
    Write-Host "  ssh ${OrinUser}@${OrinIP}" -ForegroundColor Gray
    Write-Host "  cd ~ && unzip $(Split-Path $zipPath -Leaf) && cd orin_chassis_follower_$timestamp" -ForegroundColor Gray
    Write-Host "  ./build_on_orin.sh" -ForegroundColor Gray
    Write-Host ""
    exit 0
}

# 7. Transfer to Orin
Write-Host "[3/5] Transferring to Orin..." -ForegroundColor Yellow
Write-Host "  Target: ${OrinUser}@${OrinIP}" -ForegroundColor White

# Check if scp is available
$scpCmd = Get-Command scp -ErrorAction SilentlyContinue
if (-not $scpCmd) {
    Write-Host "  ⚠️  scp not found - using manual transfer instructions" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Please transfer manually:" -ForegroundColor Cyan
    Write-Host "  1. Copy $zipPath to Orin" -ForegroundColor Gray
    Write-Host "  2. SSH to Orin: ssh ${OrinUser}@${OrinIP}" -ForegroundColor Gray
    Write-Host "  3. Extract: unzip $(Split-Path $zipPath -Leaf)" -ForegroundColor Gray
    Write-Host "  4. Build: cd orin_chassis_follower_$timestamp && ./build_on_orin.sh" -ForegroundColor Gray
    Write-Host ""
    exit 0
}

# Use WSL for scp if on Windows
Write-Host "  Using WSL for transfer..." -ForegroundColor Gray
$wslZipPath = $zipPath -replace '\\', '/' -replace 'C:', '/mnt/c'
$transferCmd = "scp `"$wslZipPath`" ${OrinUser}@${OrinIP}:~/"

try {
    wsl bash -c $transferCmd
    Write-Host "  ✅ Transfer complete" -ForegroundColor Green
} catch {
    Write-Host "  ❌ Transfer failed: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Manual transfer command:" -ForegroundColor Cyan
    Write-Host "  scp `"$zipPath`" ${OrinUser}@${OrinIP}:~/" -ForegroundColor Gray
    Write-Host ""
    exit 1
}

Write-Host ""

# 8. Provide next steps
Write-Host "[4/5] Next Steps on Orin:" -ForegroundColor Yellow
Write-Host ""
Write-Host "  SSH to Orin:" -ForegroundColor Cyan
Write-Host "    ssh ${OrinUser}@${OrinIP}" -ForegroundColor Gray
Write-Host ""
Write-Host "  Extract and build:" -ForegroundColor Cyan
Write-Host "    cd ~ && unzip $(Split-Path $zipPath -Leaf)" -ForegroundColor Gray
Write-Host "    cd orin_chassis_follower_$timestamp" -ForegroundColor Gray
Write-Host "    chmod +x build_on_orin.sh" -ForegroundColor Gray
Write-Host "    ./build_on_orin.sh" -ForegroundColor Gray
Write-Host ""
Write-Host "  Test the node:" -ForegroundColor Cyan
Write-Host "    chmod +x test_on_orin.sh" -ForegroundColor Gray
Write-Host "    ./test_on_orin.sh" -ForegroundColor Gray
Write-Host ""

# 9. Summary
Write-Host "[5/5] Deployment Summary:" -ForegroundColor Yellow
Write-Host "  ✅ Package created and transferred" -ForegroundColor Green
Write-Host "  📦 Size: $([math]::Round($zipSize, 2)) MB" -ForegroundColor White
Write-Host "  📍 Location on Orin: ~/$(Split-Path $zipPath -Leaf)" -ForegroundColor White
Write-Host "  🎯 Ready for build on Orin" -ForegroundColor White
Write-Host ""
Write-Host "===============================================" -ForegroundColor Cyan
Write-Host "  Deployment package ready!" -ForegroundColor Cyan
Write-Host "===============================================" -ForegroundColor Cyan
