# Deploy Chassis Path Follower to AGX Orin
# Usage: .\deploy_chassis_to_orin.ps1 -OrinIP "192.168.100.150" [-Username "nvidia"]

param(
    [Parameter(Mandatory=$true)]
    [string]$OrinIP,
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "nvidia",
    
    [Parameter(Mandatory=$false)]
    [string]$RemotePath = "/home/nvidia/gikWBC9DOF",
    
    [Parameter(Mandatory=$false)]
    [switch]$BuildOnOrin = $true,
    
    [Parameter(Mandatory=$false)]
    [switch]$TestAfterBuild = $false
)

$ErrorActionPreference = "Stop"

Write-Host "`n" -NoNewline
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "   🚀 Chassis Path Follower - Deploy to AGX Orin" -ForegroundColor Cyan
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Target: $Username@$OrinIP" -ForegroundColor White
Write-Host "Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Gray
Write-Host "═══════════════════════════════════════════════════════════════`n" -ForegroundColor Cyan

# Get project root
$PSScriptRoot = Split-Path $MyInvocation.MyCommand.Path
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

# Verify directories exist
if (-not (Test-Path (Join-Path $projectRoot "ros2\gik9dof_controllers"))) {
    Write-Host "✗ Error: gik9dof_controllers package not found" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path (Join-Path $projectRoot "codegen\chassis_path_follower_arm64"))) {
    Write-Host "✗ Error: Generated code not found" -ForegroundColor Red
    Write-Host "  Run codegen first: wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh" -ForegroundColor Yellow
    exit 1
}

# Test SSH connection
Write-Host "[1/7] Testing SSH connection..." -ForegroundColor Yellow
Write-Host "  You may be prompted for password" -ForegroundColor Gray
try {
    $sshTest = ssh -o ConnectTimeout=10 "$Username@$OrinIP" "echo 'OK'" 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "✗ SSH connection failed" -ForegroundColor Red
        Write-Host "  Check IP address and username" -ForegroundColor Yellow
        Write-Host "  Try: ssh $Username@$OrinIP" -ForegroundColor Gray
        exit 1
    }
    Write-Host "✓ SSH connection successful`n" -ForegroundColor Green
} catch {
    Write-Host "✗ Cannot connect to $OrinIP" -ForegroundColor Red
    exit 1
}

# Create workspace structure on Orin
Write-Host "[2/7] Creating workspace structure..." -ForegroundColor Yellow
$createDirs = @"
mkdir -p $RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower
mkdir -p $RemotePath/ros2/gik9dof_controllers/launch
mkdir -p $RemotePath/ros2/gik9dof_controllers/include
mkdir -p $RemotePath/ros2/gik9dof_msgs
echo "Directories created"
"@

ssh "$Username@$OrinIP" $createDirs
Write-Host "✓ Workspace structure created`n" -ForegroundColor Green

# Transfer MATLAB generated C++ code
Write-Host "[3/7] Transferring MATLAB generated code..." -ForegroundColor Yellow
Write-Host "  Source: codegen/chassis_path_follower_arm64/" -ForegroundColor Gray
Write-Host "  Transferring 19 files (80 KB)..." -ForegroundColor Gray

$codegenSource = Join-Path $projectRoot "codegen\chassis_path_follower_arm64"

# Use rsync if available (faster)
$useRsync = $false
try {
    wsl which rsync > $null 2>&1
    if ($LASTEXITCODE -eq 0) {
        $useRsync = $true
        Write-Host "  Using rsync for transfer..." -ForegroundColor Cyan
    }
} catch {}

if ($useRsync) {
    $wslPath = $codegenSource -replace '\\', '/' -replace 'C:', '/mnt/c'
    wsl rsync -avz --progress `
        --include='*.cpp' --include='*.h' `
        --exclude='examples/' --exclude='interface/' --exclude='html/' `
        "$wslPath/" "$Username@${OrinIP}:$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower/"
} else {
    # Use scp
    $cppFiles = Get-ChildItem "$codegenSource\*.cpp", "$codegenSource\*.h" -File
    foreach ($file in $cppFiles) {
        scp $file.FullName "$Username@${OrinIP}:$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower/" 2>$null
    }
}
Write-Host "✓ Generated code transferred`n" -ForegroundColor Green

# Transfer ROS2 wrapper node
Write-Host "[4/7] Transferring ROS2 wrapper node..." -ForegroundColor Yellow
$wrapperNode = Join-Path $projectRoot "ros2\gik9dof_controllers\src\chassis_path_follower_node.cpp"
scp $wrapperNode "$Username@${OrinIP}:$RemotePath/ros2/gik9dof_controllers/src/"
Write-Host "✓ Wrapper node transferred`n" -ForegroundColor Green

# Transfer build files
Write-Host "[5/7] Transferring build configuration..." -ForegroundColor Yellow
$buildFiles = @(
    "ros2\gik9dof_controllers\CMakeLists.txt",
    "ros2\gik9dof_controllers\package.xml",
    "ros2\gik9dof_controllers\README.md"
)

foreach ($file in $buildFiles) {
    $srcFile = Join-Path $projectRoot $file
    if (Test-Path $srcFile) {
        scp $srcFile "$Username@${OrinIP}:$RemotePath/ros2/gik9dof_controllers/"
    }
}
Write-Host "✓ Build files transferred`n" -ForegroundColor Green

# Transfer launch file
Write-Host "[6/7] Transferring launch file..." -ForegroundColor Yellow
$launchFile = Join-Path $projectRoot "ros2\gik9dof_controllers\launch\chassis_path_follower_launch.py"
if (Test-Path $launchFile) {
    scp $launchFile "$Username@${OrinIP}:$RemotePath/ros2/gik9dof_controllers/launch/"
    Write-Host "✓ Launch file transferred`n" -ForegroundColor Green
} else {
    Write-Host "⚠ Launch file not found`n" -ForegroundColor Yellow
}

# Verify transfer
Write-Host "[7/7] Verifying transfer..." -ForegroundColor Yellow
$verifyScript = @"
echo "Checking transferred files..."
echo ""

# Check ROS2 package structure
if [ -f "$RemotePath/ros2/gik9dof_controllers/CMakeLists.txt" ]; then
    echo "✓ CMakeLists.txt"
else
    echo "✗ CMakeLists.txt MISSING"
fi

if [ -f "$RemotePath/ros2/gik9dof_controllers/package.xml" ]; then
    echo "✓ package.xml"
else
    echo "✗ package.xml MISSING"
fi

# Check source files
if [ -f "$RemotePath/ros2/gik9dof_controllers/src/chassis_path_follower_node.cpp" ]; then
    echo "✓ chassis_path_follower_node.cpp"
else
    echo "✗ chassis_path_follower_node.cpp MISSING"
fi

# Check MATLAB generated code
if [ -d "$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower" ]; then
    cpp_count=`$(find "$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower" -name "*.cpp" | wc -l)
    h_count=`$(find "$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower" -name "*.h" | wc -l)
    echo "✓ MATLAB generated: `$cpp_count C++ files, `$h_count headers"
    
    # Check key files
    if [ -f "$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower/ChassisPathFollower.cpp" ]; then
        echo "  ✓ ChassisPathFollower.cpp"
    else
        echo "  ✗ ChassisPathFollower.cpp MISSING"
    fi
else
    echo "✗ matlab_generated directory MISSING"
fi

# Check launch file
if [ -f "$RemotePath/ros2/gik9dof_controllers/launch/chassis_path_follower_launch.py" ]; then
    echo "✓ Launch file"
else
    echo "⚠ Launch file not found"
fi

echo ""
echo "Disk usage:"
du -sh "$RemotePath/ros2/gik9dof_controllers"
"@

ssh "$Username@$OrinIP" $verifyScript

Write-Host ""
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "   ✓ TRANSFER COMPLETE!" -ForegroundColor Green
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan

# Build on Orin if requested
if ($BuildOnOrin) {
    Write-Host ""
    Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
    Write-Host "   🔨 Building on AGX Orin..." -ForegroundColor Yellow
    Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
    Write-Host ""
    
    $buildScript = @"
#!/bin/bash
set -e

cd $RemotePath/ros2

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 1: Source ROS2 environment"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source /opt/ros/humble/setup.bash
echo "✓ ROS2 Humble sourced"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 2: Build gik9dof_msgs (dependencies)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -d "gik9dof_msgs" ]; then
    colcon build --packages-select gik9dof_msgs
    source install/setup.bash
    echo "✓ gik9dof_msgs built"
else
    echo "⚠ gik9dof_msgs not found (may not be needed)"
fi
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 3: Build gik9dof_controllers"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "This will compile the chassis path follower with ARM64 optimizations..."
echo ""

colcon build --packages-select gik9dof_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ \$? -eq 0 ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ✓ BUILD SUCCESSFUL!"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Executable location:"
    ls -lh install/gik9dof_controllers/lib/gik9dof_controllers/chassis_path_follower_node
    echo ""
else
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ✗ BUILD FAILED"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    exit 1
fi
"@
    
    # Save build script to temp file
    $tempScript = [System.IO.Path]::GetTempFileName() + ".sh"
    $buildScript | Out-File -FilePath $tempScript -Encoding ASCII -NoNewline
    
    # Transfer and execute
    scp $tempScript "$Username@${OrinIP}:/tmp/build_chassis.sh"
    ssh "$Username@$OrinIP" "chmod +x /tmp/build_chassis.sh && /tmp/build_chassis.sh"
    
    Remove-Item $tempScript -Force
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host ""
        Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
        Write-Host "   ✓ BUILD COMPLETE!" -ForegroundColor Green
        Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
    }
}

# Print next steps
Write-Host ""
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Cyan
Write-Host "   📋 Next Steps on AGX Orin" -ForegroundColor Yellow
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Cyan
Write-Host ""

if (-not $BuildOnOrin) {
    Write-Host "1. SSH to AGX Orin:" -ForegroundColor White
    Write-Host "   ssh $Username@$OrinIP" -ForegroundColor Gray
    Write-Host ""
    Write-Host "2. Build the package:" -ForegroundColor White
    Write-Host "   cd $RemotePath/ros2" -ForegroundColor Gray
    Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor Gray
    Write-Host "   colcon build --packages-select gik9dof_controllers" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "$(if ($BuildOnOrin) { '1' } else { '3' }). Test the controller:" -ForegroundColor White
Write-Host "   cd $RemotePath/ros2" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py" -ForegroundColor Gray
Write-Host ""

Write-Host "$(if ($BuildOnOrin) { '2' } else { '4' }). Test with different modes:" -ForegroundColor White
Write-Host "   # Mode 0 (Differentiation)" -ForegroundColor Gray
Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0" -ForegroundColor Gray
Write-Host ""
Write-Host "   # Mode 1 (Heading-Aware)" -ForegroundColor Gray
Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1" -ForegroundColor Gray
Write-Host ""
Write-Host "   # Mode 2 (Pure Pursuit - Default)" -ForegroundColor Gray
Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2" -ForegroundColor Gray
Write-Host ""

Write-Host "$(if ($BuildOnOrin) { '3' } else { '5' }). Monitor topics:" -ForegroundColor White
Write-Host "   ros2 topic echo /cmd_vel" -ForegroundColor Gray
Write-Host "   ros2 topic echo /path_following_active" -ForegroundColor Gray
Write-Host ""

Write-Host "$(if ($BuildOnOrin) { '4' } else { '6' }). Check parameters:" -ForegroundColor White
Write-Host "   ros2 param list /chassis_path_follower" -ForegroundColor Gray
Write-Host "   ros2 param get /chassis_path_follower controller_mode" -ForegroundColor Gray
Write-Host ""

Write-Host "📚 Documentation:" -ForegroundColor Cyan
Write-Host "   - Complete guide: ros2/gik9dof_controllers/README.md" -ForegroundColor Gray
Write-Host "   - Codegen details: DAY5_CODEGEN_SESSION_SUMMARY.md" -ForegroundColor Gray
Write-Host "   - Session summary: SESSION_COMPLETE.md" -ForegroundColor Gray
Write-Host ""

Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "   🚀 Deployment Complete! Ready for Testing" -ForegroundColor Green
Write-Host "═══════════════════════════════════════════════════════════════`n" -ForegroundColor Cyan
