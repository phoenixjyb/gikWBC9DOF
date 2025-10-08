# deploy_ros2_to_orin.ps1
# PowerShell script to transfer updated ROS2 workspace to AGX Orin
# For 20-constraint GIK solver integration
# Usage: .\deploy_ros2_to_orin.ps1

param(
    [Parameter(Mandatory=$false)]
    [string]$OrinIP = "192.168.100.150",
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "cr",
    
    [Parameter(Mandatory=$false)]
    [string]$RemotePath = "/home/nvidia/temp_gikrepo"
)

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Deploying ROS2 Workspace to AGX Orin" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $Username@$OrinIP`:$RemotePath" -ForegroundColor White
Write-Host ""

# Project paths
$projectRoot = $PSScriptRoot
$ros2Dir = Join-Path $projectRoot "ros2"

# Verify ROS2 workspace exists
if (-not (Test-Path $ros2Dir)) {
    Write-Host "✗ ROS2 workspace not found: $ros2Dir" -ForegroundColor Red
    exit 1
}

Write-Host "✓ ROS2 workspace found" -ForegroundColor Green
Write-Host "  Path: $ros2Dir" -ForegroundColor Gray
Write-Host ""

# Test SSH connection
Write-Host "Testing SSH connection to $Username@$OrinIP..." -ForegroundColor Yellow
try {
    $result = ssh -o ConnectTimeout=5 "$Username@$OrinIP" "echo 'OK'" 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ SSH connection successful" -ForegroundColor Green
        Write-Host ""
    } else {
        throw "SSH connection failed"
    }
} catch {
    Write-Host "✗ Cannot connect to $Username@$OrinIP" -ForegroundColor Red
    Write-Host "Error: $_" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Troubleshooting:" -ForegroundColor Yellow
    Write-Host "1. Check network connection: ping $OrinIP" -ForegroundColor Gray
    Write-Host "2. Check SSH access: ssh $Username@$OrinIP" -ForegroundColor Gray
    Write-Host "3. Verify credentials and IP address" -ForegroundColor Gray
    exit 1
}

# Create remote directory
Write-Host "Creating remote directory: $RemotePath..." -ForegroundColor Yellow
try {
    ssh "$Username@$OrinIP" "mkdir -p $RemotePath"
    Write-Host "✓ Remote directory ready" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "✗ Failed to create remote directory" -ForegroundColor Red
    exit 1
}

# Count files to transfer
Write-Host "Analyzing ROS2 workspace..." -ForegroundColor Yellow
$fileCount = (Get-ChildItem -Path $ros2Dir -Recurse -File | Measure-Object).Count
$totalSize = [math]::Round((Get-ChildItem -Path $ros2Dir -Recurse -File | Measure-Object -Property Length -Sum).Sum / 1MB, 2)

Write-Host "  Files: $fileCount" -ForegroundColor Gray
Write-Host "  Total Size: $totalSize MB" -ForegroundColor Gray
Write-Host ""

# Create temporary zip file
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$zipFileName = "ros2_workspace_$timestamp.zip"
$zipPath = Join-Path $env:TEMP $zipFileName

Write-Host "Creating deployment package..." -ForegroundColor Yellow
Write-Host "  Package: $zipFileName" -ForegroundColor Gray

try {
    # Create zip file - use robocopy-style approach to handle symlinks
    # Copy ros2 to temp, excluding problematic directories
    $tempRos2 = Join-Path $env:TEMP "ros2_for_zip_$timestamp"
    
    Write-Host "  Preparing files..." -ForegroundColor Gray
    
    # Create temp directory
    if (Test-Path $tempRos2) {
        Remove-Item $tempRos2 -Recurse -Force
    }
    New-Item -ItemType Directory -Path $tempRos2 | Out-Null
    
    # Copy with exclusions (avoid symlinks and build artifacts)
    robocopy $ros2Dir $tempRos2 /E /XD build install log /XF *.pyc /NFL /NDL /NJH /NJS /NC /NS /NP
    
    # Compress the clean copy
    Compress-Archive -Path "$tempRos2\*" -DestinationPath $zipPath -Force
    
    # Cleanup temp directory
    Remove-Item $tempRos2 -Recurse -Force
    
    $zipSize = [math]::Round((Get-Item $zipPath).Length / 1MB, 2)
    Write-Host "✓ Package created: $zipSize MB" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "✗ Failed to create zip package" -ForegroundColor Red
    Write-Host "Error: $_" -ForegroundColor Yellow
    
    # Cleanup temp directory if it exists
    if (Test-Path $tempRos2) {
        Remove-Item $tempRos2 -Recurse -Force -ErrorAction SilentlyContinue
    }
    
    exit 1
}

# Transfer zip file
Write-Host "Transferring package to Orin..." -ForegroundColor Yellow
Write-Host "This may take a minute..." -ForegroundColor Gray

try {
    scp $zipPath "$Username@$OrinIP`:~/$zipFileName"
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Transfer complete" -ForegroundColor Green
        Write-Host ""
    } else {
        throw "scp transfer failed"
    }
} catch {
    Write-Host "✗ Transfer failed" -ForegroundColor Red
    Write-Host "Error: $_" -ForegroundColor Yellow
    
    # Cleanup local zip
    if (Test-Path $zipPath) {
        Remove-Item $zipPath -Force
    }
    exit 1
}

# Extract on Orin and cleanup
Write-Host "Extracting on Orin..." -ForegroundColor Yellow

$remoteCommands = @"
cd $RemotePath
echo 'Extracting ros2 workspace...'
unzip -q -o ~/$zipFileName -d .
echo 'Cleaning up zip file...'
rm ~/$zipFileName
echo 'Verifying extraction...'
find ros2 -type f | wc -l
"@

try {
    $extractResult = ssh "$Username@$OrinIP" $remoteCommands
    Write-Host $extractResult
    Write-Host "✓ Extraction complete" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "⚠ Extraction may have issues" -ForegroundColor Yellow
    Write-Host "Check manually on AGX Orin" -ForegroundColor Yellow
}

# Cleanup local zip file
Write-Host "Cleaning up local package..." -ForegroundColor Yellow
try {
    if (Test-Path $zipPath) {
        Remove-Item $zipPath -Force
        Write-Host "✓ Local cleanup complete" -ForegroundColor Green
    }
} catch {
    Write-Host "⚠ Could not delete: $zipPath" -ForegroundColor Yellow
}
Write-Host ""

# Display deployment summary
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "✓ DEPLOYMENT COMPLETE" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Deployment Details:" -ForegroundColor White
Write-Host "  Local:  $ros2Dir" -ForegroundColor Gray
Write-Host "  Remote: $Username@$OrinIP`:$RemotePath/ros2" -ForegroundColor Gray
Write-Host "  Files:  $fileCount (~$totalSize MB)" -ForegroundColor Gray
Write-Host ""

# Build instructions
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Next Steps on AGX Orin" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "1. Connect to AGX Orin:" -ForegroundColor Yellow
Write-Host "   ssh $Username@$OrinIP" -ForegroundColor White
Write-Host ""

Write-Host "2. Navigate to workspace:" -ForegroundColor Yellow
Write-Host "   cd $RemotePath/ros2" -ForegroundColor White
Write-Host ""

Write-Host "3. Source ROS2:" -ForegroundColor Yellow
Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor White
Write-Host ""

Write-Host "4. Clean previous build (recommended after major changes):" -ForegroundColor Yellow
Write-Host "   rm -rf build/ install/ log/" -ForegroundColor White
Write-Host ""

Write-Host "5. Build packages (in order):" -ForegroundColor Yellow
Write-Host "   # Build message package first" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release" -ForegroundColor White
Write-Host ""
Write-Host "   # Source the messages" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor White
Write-Host ""
Write-Host "   # Build solver package" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release" -ForegroundColor White
Write-Host ""

Write-Host "6. Test the solver:" -ForegroundColor Yellow
Write-Host "   source install/setup.bash" -ForegroundColor White
Write-Host "   ros2 run gik9dof_solver gik9dof_solver_node --ros-args \\" -ForegroundColor White
Write-Host "       --params-file src/gik9dof_solver/config/gik_solver_params.yaml" -ForegroundColor White
Write-Host ""

Write-Host "7. Monitor diagnostics (in another terminal):" -ForegroundColor Yellow
Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor White
Write-Host "   cd $RemotePath/ros2" -ForegroundColor White
Write-Host "   source install/setup.bash" -ForegroundColor White
Write-Host "   ros2 topic echo /gik9dof/diagnostics" -ForegroundColor White
Write-Host ""

# Important notes
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Important Notes" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "✓ Updated Interface:" -ForegroundColor Green
Write-Host "  - NEW: 20-constraint solver (7-parameter interface)" -ForegroundColor Gray
Write-Host "  - Default: 5 active constraints (gripper/link collision avoidance)" -ForegroundColor Gray
Write-Host "  - Enhanced diagnostics with per-constraint reporting" -ForegroundColor Gray
Write-Host ""
Write-Host "✓ Expected Performance:" -ForegroundColor Green
Write-Host "  - Solve time: ~15-20ms (default 5 constraints)" -ForegroundColor Gray
Write-Host "  - Build time: ~2-3 minutes (2 packages, 158 C++ files)" -ForegroundColor Gray
Write-Host ""
Write-Host "✓ Files Deployed:" -ForegroundColor Green
Write-Host "  - Updated solver node (7-parameter interface)" -ForegroundColor Gray
Write-Host "  - Enhanced diagnostics message definition" -ForegroundColor Gray
Write-Host "  - Default parameter configuration (YAML)" -ForegroundColor Gray
Write-Host "  - All 158 generated solver files (ARM64)" -ForegroundColor Gray
Write-Host ""

# Optional: Auto-connect to Orin
Write-Host "========================================" -ForegroundColor Cyan
$response = Read-Host "Connect to AGX Orin now? (y/N)"
if ($response -eq "y" -or $response -eq "Y") {
    Write-Host ""
    Write-Host "Connecting to $Username@$OrinIP..." -ForegroundColor Yellow
    Write-Host "You'll be placed in $RemotePath/ros2" -ForegroundColor Gray
    Write-Host ""
    ssh -t "$Username@$OrinIP" "cd $RemotePath/ros2 && exec bash"
} else {
    Write-Host ""
    Write-Host "Deployment complete. Run the steps above when ready." -ForegroundColor White
}
