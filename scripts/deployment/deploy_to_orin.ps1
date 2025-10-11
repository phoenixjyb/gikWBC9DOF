# Deploy to AGX Orin - Unified Deployment Script# deploy_to_orin.ps1

# Supports both complete workspace and chassis-only deployment# PowerShell script to transfer generated code to AGX Orin

## Usage: .\deploy_to_orin.ps1 <orin-ip-address>

# Usage Examples:

#   # Deploy complete workspaceparam(

#   .\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete    [Parameter(Mandatory=$true)]

#    [string]$OrinIP,

#   # Deploy only chassis path follower (faster, for testing)    

#   .\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly    [Parameter(Mandatory=$false)]

#    [string]$Username = "cr"

#   # Custom settings (use your actual username and path))

#   .\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Username "cr" -RemotePath "/home/nvidia/temp_gikrepo"

$ErrorActionPreference = "Stop"

param(

    [Parameter(Mandatory=$true, HelpMessage="IP address of the AGX Orin")]Write-Host "========================================" -ForegroundColor Cyan

    [string]$OrinIP,Write-Host "Deploying to AGX Orin: $OrinIP" -ForegroundColor Cyan

    Write-Host "========================================" -ForegroundColor Cyan

    [Parameter(Mandatory=$false, HelpMessage="SSH username")]Write-Host ""

    [string]$Username = "cr",

    # Check if deployment package exists

    [Parameter(Mandatory=$false, HelpMessage="Remote deployment path")]$projectRoot = $PSScriptRoot

    [string]$RemotePath = "/home/nvidia/temp_gikrepo",$deployDir = Join-Path $projectRoot "deployment_package"

    

    [Parameter(Mandatory=$false, HelpMessage="Deployment mode: Complete or ChassisOnly")]if (-not (Test-Path $deployDir)) {

    [ValidateSet("Complete", "ChassisOnly")]    Write-Host "✗ Deployment package not found" -ForegroundColor Red

    [string]$Mode = "Complete",    Write-Host "Run RUN_CODEGEN.m in MATLAB first" -ForegroundColor Yellow

        exit 1

    [Parameter(Mandatory=$false, HelpMessage="Skip mesh files to save time")]}

    [switch]$SkipMeshes = $false,

    # Find latest deployment package

    [Parameter(Mandatory=$false, HelpMessage="Build on Orin after deployment")]$packages = Get-ChildItem -Path $deployDir -Filter "gik_codegen_*.zip" | Sort-Object LastWriteTime -Descending

    [switch]$BuildOnOrin = $true,

    if ($packages.Count -eq 0) {

    [Parameter(Mandatory=$false, HelpMessage="Run tests after build")]    Write-Host "✗ No deployment packages found in $deployDir" -ForegroundColor Red

    [switch]$TestAfterBuild = $false    Write-Host "Run RUN_CODEGEN.m in MATLAB first" -ForegroundColor Yellow

)    exit 1

}

$ErrorActionPreference = "Stop"

$latestPackage = $packages[0]

# BannerWrite-Host "Found deployment package:" -ForegroundColor Green

Write-Host "`n" -NoNewlineWrite-Host "  $($latestPackage.Name)" -ForegroundColor White

Write-Host "========================================================" -ForegroundColor CyanWrite-Host "  Size: $([math]::Round($latestPackage.Length / 1MB, 2)) MB" -ForegroundColor White

Write-Host "   AGX Orin Deployment - Unified Script" -ForegroundColor CyanWrite-Host ""

Write-Host "========================================================" -ForegroundColor Cyan

Write-Host "Mode: $Mode" -ForegroundColor White# Test SSH connection

Write-Host "Target: $Username@$OrinIP" -ForegroundColor WhiteWrite-Host "Testing SSH connection to $Username@$OrinIP..." -ForegroundColor Yellow

Write-Host "Remote Path: $RemotePath" -ForegroundColor Whitetry {

Write-Host "Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Gray    ssh -o ConnectTimeout=5 "$Username@$OrinIP" "echo 'Connection successful'"

Write-Host "========================================================`n" -ForegroundColor Cyan    Write-Host "✓ SSH connection OK" -ForegroundColor Green

    Write-Host ""

# Get project root} catch {

$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent    Write-Host "✗ Cannot connect to $OrinIP" -ForegroundColor Red

if (-not (Test-Path (Join-Path $projectRoot "ros2"))) {    Write-Host "Check network connection and SSH access" -ForegroundColor Yellow

    Write-Host "✗ Error: ros2/ directory not found" -ForegroundColor Red    exit 1

    Write-Host "  Project root: $projectRoot" -ForegroundColor Yellow}

    exit 1

}# Transfer deployment package

Write-Host "Transferring deployment package..." -ForegroundColor Yellow

#==============================================================================try {

# STEP 1: Test SSH Connection    scp $latestPackage.FullName "$Username@$OrinIP`:~/"

#==============================================================================    Write-Host "✓ Transfer complete" -ForegroundColor Green

Write-Host "[1/8] Testing SSH connection..." -ForegroundColor Yellow    Write-Host ""

Write-Host "  Connecting to $Username@$OrinIP" -ForegroundColor Gray} catch {

try {    Write-Host "✗ Transfer failed" -ForegroundColor Red

    $sshOutput = ssh -o ConnectTimeout=10 "$Username@$OrinIP" "echo 'OK'" 2>&1    exit 1

    if ($LASTEXITCODE -ne 0) {}

        Write-Host "✗ SSH connection failed" -ForegroundColor Red

        Write-Host "  Check IP address and username" -ForegroundColor Yellow# Extract on AGX Orin

        Write-Host "  Try: ssh $Username@$OrinIP" -ForegroundColor GrayWrite-Host "Extracting on AGX Orin..." -ForegroundColor Yellow

        exit 1$remoteCommands = @"

    }cd ~/gikWBC9DOF/ros2/gik9dof_solver

    Write-Host "✓ SSH connection successful`n" -ForegroundColor Greenmkdir -p matlab_codegen

} catch {cd matlab_codegen

    Write-Host "✗ Cannot connect to $OrinIP" -ForegroundColor Redunzip -o ~/$($ latestPackage.Name) -d .

    Write-Host "  Error: $_" -ForegroundColor Yellowecho 'Extracted files:'

    exit 1find . -type f | wc -l

}"@



#==============================================================================try {

# STEP 2: Create Workspace Structure on Orin    ssh "$Username@$OrinIP" $remoteCommands

#==============================================================================    Write-Host "✓ Extraction complete" -ForegroundColor Green

Write-Host "[2/8] Creating workspace structure on Orin..." -ForegroundColor Yellow    Write-Host ""

} catch {

$remoteSetupScript = @"    Write-Host "⚠ Extraction may have issues" -ForegroundColor Yellow

#!/bin/bash    Write-Host "Check manually on AGX Orin" -ForegroundColor Yellow

set -e}



echo 'Creating directory structure...'# Summary

mkdir -p $RemotePath/ros2/gik9dof_controllers/src/matlab_generatedWrite-Host "========================================" -ForegroundColor Cyan

mkdir -p $RemotePath/ros2/gik9dof_controllers/launchWrite-Host "✓ DEPLOYMENT COMPLETE" -ForegroundColor Green

mkdir -p $RemotePath/ros2/gik9dof_msgsWrite-Host "========================================" -ForegroundColor Cyan

mkdir -p $RemotePath/urdfWrite-Host ""

mkdir -p $RemotePath/meshesWrite-Host "Next steps on AGX Orin:" -ForegroundColor Yellow

mkdir -p $RemotePath/configWrite-Host "1. SSH to AGX Orin:" -ForegroundColor White

mkdir -p $RemotePath/scriptsWrite-Host "   ssh $Username@$OrinIP" -ForegroundColor Gray

Write-Host ""

echo 'Directory structure created successfully'Write-Host "2. Build ROS2 packages:" -ForegroundColor White

ls -la $RemotePath/Write-Host "   cd ~/gikWBC9DOF/ros2" -ForegroundColor Gray

"@Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor Gray

Write-Host "   colcon build --packages-select gik9dof_msgs" -ForegroundColor Gray

$remoteSetupScript | ssh "$Username@$OrinIP" "cat > /tmp/setup_workspace.sh && chmod +x /tmp/setup_workspace.sh && bash /tmp/setup_workspace.sh"Write-Host "   source install/setup.bash" -ForegroundColor Gray

Write-Host "   colcon build --packages-select gik9dof_solver" -ForegroundColor Gray

if ($LASTEXITCODE -ne 0) {Write-Host ""

    Write-Host "✗ Failed to create workspace structure" -ForegroundColor RedWrite-Host "3. Test the solver:" -ForegroundColor White

    exit 1Write-Host "   ros2 launch gik9dof_solver test_solver.launch.py" -ForegroundColor Gray

}Write-Host ""

Write-Host "✓ Workspace structure created`n" -ForegroundColor GreenWrite-Host "See FAST_TRACK_2DAY.md for detailed instructions" -ForegroundColor Yellow


#==============================================================================
# STEP 3: Transfer Files Based on Mode
#==============================================================================
if ($Mode -eq "Complete") {
    Write-Host "[3/8] Transferring complete workspace..." -ForegroundColor Yellow
    
    # Check if rsync is available
    $hasRsync = $false
    try {
        $rsyncTest = rsync --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            $hasRsync = $true
            Write-Host "  Using rsync for fast transfer" -ForegroundColor Gray
        }
    } catch {
        Write-Host "  Rsync not available, using ZIP+SCP" -ForegroundColor Gray
    }
    
    if ($hasRsync) {
        # Use rsync for efficient transfer
        Write-Host "  Transferring ROS2 packages..." -ForegroundColor Gray
        rsync -avz --progress `
            --exclude='build/' --exclude='install/' --exclude='log/' `
            --exclude='*.pyc' --exclude='__pycache__/' `
            (Join-Path $projectRoot "ros2") `
            "$Username@${OrinIP}:$RemotePath/"
        
        Write-Host "  Transferring URDF files..." -ForegroundColor Gray
        $urdfFiles = Get-ChildItem -Path $projectRoot -Filter "*.urdf"
        foreach ($urdf in $urdfFiles) {
            rsync -avz --progress $urdf.FullName "$Username@${OrinIP}:$RemotePath/urdf/"
        }
        
        if (-not $SkipMeshes) {
            Write-Host "  Transferring mesh files..." -ForegroundColor Gray
            if (Test-Path (Join-Path $projectRoot "meshes")) {
                rsync -avz --progress `
                    (Join-Path $projectRoot "meshes") `
                    "$Username@${OrinIP}:$RemotePath/"
            }
        }
        
        Write-Host "  Transferring config files..." -ForegroundColor Gray
        if (Test-Path (Join-Path $projectRoot "config")) {
            rsync -avz --progress `
                (Join-Path $projectRoot "config") `
                "$Username@${OrinIP}:$RemotePath/"
        }
        
    } else {
        # Fallback to ZIP + SCP
        Write-Host "  Creating deployment archive..." -ForegroundColor Gray
        $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
        $zipPath = Join-Path $env:TEMP "orin_deploy_$timestamp.zip"
        
        # Create ZIP archive with ROS2 workspace
        $ros2Path = Join-Path $projectRoot "ros2"
        Compress-Archive -Path $ros2Path -DestinationPath $zipPath -Force
        
        Write-Host "  Archive created: $(Split-Path $zipPath -Leaf)" -ForegroundColor Gray
        Write-Host "  Transferring to Orin (this may take a few minutes)..." -ForegroundColor Gray
        
        scp $zipPath "$Username@${OrinIP}:/tmp/"
        
        Write-Host "  Extracting on Orin..." -ForegroundColor Gray
        ssh "$Username@$OrinIP" "cd $RemotePath && unzip -o /tmp/$(Split-Path $zipPath -Leaf) && rm /tmp/$(Split-Path $zipPath -Leaf)"
        
        # Clean up local ZIP
        Remove-Item $zipPath -Force
    }
    
    Write-Host "✓ Complete workspace transferred`n" -ForegroundColor Green
    
} else {
    # ChassisOnly mode
    Write-Host "[3/8] Transferring chassis path follower only..." -ForegroundColor Yellow
    
    # Verify generated code exists
    $chassisCodegenPath = Join-Path $projectRoot "codegen\chassis_path_follower_arm64"
    if (-not (Test-Path $chassisCodegenPath)) {
        Write-Host "✗ Generated code not found: $chassisCodegenPath" -ForegroundColor Red
        Write-Host "  Run codegen first: wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh" -ForegroundColor Yellow
        exit 1
    }
    
    # Transfer MATLAB generated code
    Write-Host "  [3a] Transferring MATLAB generated code..." -ForegroundColor Gray
    $generatedFiles = Get-ChildItem -Path $chassisCodegenPath -Include "*.cpp","*.h" -Recurse
    Write-Host "    Found $($generatedFiles.Count) files to transfer" -ForegroundColor Gray
    
    foreach ($file in $generatedFiles) {
        $relativePath = $file.FullName.Substring($chassisCodegenPath.Length + 1).Replace('\', '/')
        $remoteDir = "$RemotePath/ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower"
        
        # Create directory and copy file
        $cmd = "mkdir -p `"$remoteDir`" && cat > `"$remoteDir/$relativePath`""
        Get-Content $file.FullName -Raw | ssh "$Username@$OrinIP" $cmd
    }
    Write-Host "    ✓ MATLAB code transferred ($($generatedFiles.Count) files)" -ForegroundColor Green
    
    # Transfer ROS2 wrapper node
    Write-Host "  [3b] Transferring ROS2 wrapper node..." -ForegroundColor Gray
    $wrapperPath = Join-Path $projectRoot "ros2\gik9dof_controllers\src\chassis_path_follower_node.cpp"
    if (Test-Path $wrapperPath) {
        Get-Content $wrapperPath -Raw | ssh "$Username@$OrinIP" "cat > $RemotePath/ros2/gik9dof_controllers/src/chassis_path_follower_node.cpp"
        Write-Host "    ✓ Wrapper node transferred" -ForegroundColor Green
    } else {
        Write-Host "    ✗ Warning: Wrapper node not found" -ForegroundColor Yellow
    }
    
    # Transfer build files
    Write-Host "  [3c] Transferring build configuration..." -ForegroundColor Gray
    $buildFiles = @("CMakeLists.txt", "package.xml", "README.md")
    foreach ($file in $buildFiles) {
        $filePath = Join-Path $projectRoot "ros2\gik9dof_controllers\$file"
        if (Test-Path $filePath) {
            Get-Content $filePath -Raw | ssh "$Username@$OrinIP" "cat > $RemotePath/ros2/gik9dof_controllers/$file"
        }
    }
    Write-Host "    ✓ Build files transferred" -ForegroundColor Green
    
    # Transfer launch file
    Write-Host "  [3d] Transferring launch file..." -ForegroundColor Gray
    $launchPath = Join-Path $projectRoot "ros2\gik9dof_controllers\launch\chassis_path_follower_launch.py"
    if (Test-Path $launchPath) {
        Get-Content $launchPath -Raw | ssh "$Username@$OrinIP" "cat > $RemotePath/ros2/gik9dof_controllers/launch/chassis_path_follower_launch.py"
        Write-Host "    ✓ Launch file transferred" -ForegroundColor Green
    }
    
    Write-Host "✓ Chassis path follower transferred`n" -ForegroundColor Green
}

#==============================================================================
# STEP 4: Transfer Dependencies (gik9dof_msgs)
#==============================================================================
Write-Host "[4/8] Transferring message definitions..." -ForegroundColor Yellow
$msgsPath = Join-Path $projectRoot "ros2\gik9dof_msgs"
if (Test-Path $msgsPath) {
    $msgFiles = Get-ChildItem -Path $msgsPath -Recurse -File
    Write-Host "  Found $($msgFiles.Count) message files" -ForegroundColor Gray
    
    foreach ($file in $msgFiles) {
        $relativePath = $file.FullName.Substring($msgsPath.Length + 1).Replace('\', '/')
        $remoteDir = Split-Path "$RemotePath/ros2/gik9dof_msgs/$relativePath" -Parent
        ssh "$Username@$OrinIP" "mkdir -p `"$remoteDir`""
        Get-Content $file.FullName -Raw | ssh "$Username@$OrinIP" "cat > `"$RemotePath/ros2/gik9dof_msgs/$relativePath`""
    }
    Write-Host "✓ Message definitions transferred`n" -ForegroundColor Green
} else {
    Write-Host "  (No message definitions found - skipping)`n" -ForegroundColor Gray
}

#==============================================================================
# STEP 5: Verify Files on Orin
#==============================================================================
Write-Host "[5/8] Verifying files on Orin..." -ForegroundColor Yellow

$verifyScript = @"
#!/bin/bash
cd $RemotePath

echo '=== Verification Report ==='
echo ''
echo 'Directory structure:'
ls -lh ros2/gik9dof_controllers/ 2>/dev/null | head -20

echo ''
echo 'Generated code files:'
find ros2/gik9dof_controllers/src/matlab_generated -name '*.cpp' -o -name '*.h' 2>/dev/null | wc -l

echo ''
echo 'Key files:'
ls -lh ros2/gik9dof_controllers/CMakeLists.txt 2>/dev/null && echo '✓ CMakeLists.txt' || echo '✗ CMakeLists.txt missing'
ls -lh ros2/gik9dof_controllers/package.xml 2>/dev/null && echo '✓ package.xml' || echo '✗ package.xml missing'
ls -lh ros2/gik9dof_controllers/src/chassis_path_follower_node.cpp 2>/dev/null && echo '✓ Wrapper node' || echo '✗ Wrapper node missing'
ls -lh ros2/gik9dof_controllers/launch/*.py 2>/dev/null && echo '✓ Launch file' || echo '✗ Launch file missing'

echo ''
echo 'ChassisPathFollower.cpp size:'
ls -lh ros2/gik9dof_controllers/src/matlab_generated/chassis_path_follower/ChassisPathFollower.cpp 2>/dev/null || echo '✗ Not found'

echo ''
echo '=== Verification Complete ==='
"@

$verifyScript | ssh "$Username@$OrinIP" "cat > /tmp/verify_deployment.sh && chmod +x /tmp/verify_deployment.sh && bash /tmp/verify_deployment.sh"

Write-Host "✓ Verification complete`n" -ForegroundColor Green

#==============================================================================
# STEP 6: Build on Orin (Optional)
#==============================================================================
if ($BuildOnOrin) {
    Write-Host "[6/8] Building on Orin..." -ForegroundColor Yellow
    Write-Host "  This may take 2-5 minutes..." -ForegroundColor Gray
    
    $buildScript = @"
#!/bin/bash
set -e

cd $RemotePath/ros2

echo '=== ROS2 Build Process ==='
echo ''

# Source ROS2
source /opt/ros/humble/setup.bash

echo 'Building gik9dof_msgs (dependencies)...'
colcon build --packages-select gik9dof_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ''
echo 'Sourcing message definitions...'
source install/setup.bash

echo ''
echo 'Building gik9dof_controllers with ARM64 optimizations...'
colcon build --packages-select gik9dof_controllers \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-march=armv8-a -mtune=cortex-a78 -O3 -ffast-math" \
    --event-handlers console_direct+

echo ''
echo '=== Build Complete ==='
echo ''
echo 'Verifying executable:'
ls -lh install/gik9dof_controllers/lib/gik9dof_controllers/chassis_path_follower_node 2>/dev/null || echo 'Executable not found'
"@

    $buildScript | ssh "$Username@$OrinIP" "cat > /tmp/build_on_orin.sh && chmod +x /tmp/build_on_orin.sh && bash /tmp/build_on_orin.sh"
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "✗ Build failed on Orin" -ForegroundColor Red
        Write-Host "  SSH to Orin and check build logs" -ForegroundColor Yellow
        exit 1
    }
    
    Write-Host "✓ Build successful on Orin`n" -ForegroundColor Green
} else {
    Write-Host "[6/8] Skipping build (use -BuildOnOrin to build automatically)`n" -ForegroundColor Gray
}

#==============================================================================
# STEP 7: Run Tests (Optional)
#==============================================================================
if ($TestAfterBuild -and $BuildOnOrin) {
    Write-Host "[7/8] Running tests on Orin..." -ForegroundColor Yellow
    
    $testScript = @"
#!/bin/bash
cd $RemotePath/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo 'Running ROS2 tests...'
colcon test --packages-select gik9dof_controllers
colcon test-result --verbose
"@

    $testScript | ssh "$Username@$OrinIP" "cat > /tmp/run_tests.sh && chmod +x /tmp/run_tests.sh && bash /tmp/run_tests.sh"
    Write-Host "✓ Tests complete`n" -ForegroundColor Green
} else {
    Write-Host "[7/8] Skipping tests (use -TestAfterBuild to run automatically)`n" -ForegroundColor Gray
}

#==============================================================================
# STEP 8: Deployment Summary
#==============================================================================
Write-Host "[8/8] Deployment Summary" -ForegroundColor Yellow
Write-Host ""
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   Deployment Complete!" -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "Mode: $Mode" -ForegroundColor White
Write-Host "Target: $Username@$OrinIP" -ForegroundColor White
Write-Host "Path: $RemotePath" -ForegroundColor White
Write-Host "Build: $(if ($BuildOnOrin) { 'Yes' } else { 'No' })" -ForegroundColor White
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ""

#==============================================================================
# Next Steps
#==============================================================================
Write-Host "NEXT STEPS:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. SSH to Orin:" -ForegroundColor White
Write-Host "   ssh $Username@$OrinIP" -ForegroundColor Gray
Write-Host ""
Write-Host "2. Navigate to workspace:" -ForegroundColor White
Write-Host "   cd $RemotePath/ros2" -ForegroundColor Gray
Write-Host ""
Write-Host "3. Source ROS2:" -ForegroundColor White
Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host ""

if ($Mode -eq "ChassisOnly" -or $Mode -eq "Complete") {
    Write-Host "4. Launch chassis path follower:" -ForegroundColor White
    Write-Host "   # Default mode (Pure Pursuit)" -ForegroundColor Gray
    Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py" -ForegroundColor Gray
    Write-Host ""
    Write-Host "   # Test all 3 modes:" -ForegroundColor Gray
    Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0  # Differentiation" -ForegroundColor Gray
    Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1  # Heading-Aware" -ForegroundColor Gray
    Write-Host "   ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2  # Pure Pursuit" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "5. Monitor topics:" -ForegroundColor White
Write-Host "   ros2 topic list" -ForegroundColor Gray
Write-Host "   ros2 topic echo /cmd_vel" -ForegroundColor Gray
Write-Host "   ros2 param list /chassis_path_follower" -ForegroundColor Gray
Write-Host ""

if (-not $BuildOnOrin) {
    Write-Host "Note: Build was skipped. To build manually:" -ForegroundColor Yellow
    Write-Host "  cd $RemotePath/ros2" -ForegroundColor Gray
    Write-Host "  colcon build --packages-select gik9dof_msgs gik9dof_controllers" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "For detailed documentation, see:" -ForegroundColor White
Write-Host "  - DEPLOY_TO_ORIN_GUIDE.md" -ForegroundColor Gray
Write-Host "  - ros2/gik9dof_controllers/README.md (on Orin)" -ForegroundColor Gray
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "✓ Deployment script completed successfully!" -ForegroundColor Green
Write-Host ""
