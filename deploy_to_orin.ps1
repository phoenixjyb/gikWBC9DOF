# deploy_to_orin.ps1
# PowerShell script to transfer generated code to AGX Orin
# Usage: .\deploy_to_orin.ps1 <orin-ip-address>

param(
    [Parameter(Mandatory=$true)]
    [string]$OrinIP,
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "cr"
)

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Deploying to AGX Orin: $OrinIP" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Check if deployment package exists
$projectRoot = $PSScriptRoot
$deployDir = Join-Path $projectRoot "deployment_package"

if (-not (Test-Path $deployDir)) {
    Write-Host "✗ Deployment package not found" -ForegroundColor Red
    Write-Host "Run RUN_CODEGEN.m in MATLAB first" -ForegroundColor Yellow
    exit 1
}

# Find latest deployment package
$packages = Get-ChildItem -Path $deployDir -Filter "gik_codegen_*.zip" | Sort-Object LastWriteTime -Descending

if ($packages.Count -eq 0) {
    Write-Host "✗ No deployment packages found in $deployDir" -ForegroundColor Red
    Write-Host "Run RUN_CODEGEN.m in MATLAB first" -ForegroundColor Yellow
    exit 1
}

$latestPackage = $packages[0]
Write-Host "Found deployment package:" -ForegroundColor Green
Write-Host "  $($latestPackage.Name)" -ForegroundColor White
Write-Host "  Size: $([math]::Round($latestPackage.Length / 1MB, 2)) MB" -ForegroundColor White
Write-Host ""

# Test SSH connection
Write-Host "Testing SSH connection to $Username@$OrinIP..." -ForegroundColor Yellow
try {
    ssh -o ConnectTimeout=5 "$Username@$OrinIP" "echo 'Connection successful'"
    Write-Host "✓ SSH connection OK" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "✗ Cannot connect to $OrinIP" -ForegroundColor Red
    Write-Host "Check network connection and SSH access" -ForegroundColor Yellow
    exit 1
}

# Transfer deployment package
Write-Host "Transferring deployment package..." -ForegroundColor Yellow
try {
    scp $latestPackage.FullName "$Username@$OrinIP`:~/"
    Write-Host "✓ Transfer complete" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "✗ Transfer failed" -ForegroundColor Red
    exit 1
}

# Extract on AGX Orin
Write-Host "Extracting on AGX Orin..." -ForegroundColor Yellow
$remoteCommands = @"
cd ~/gikWBC9DOF/ros2/gik9dof_solver
mkdir -p matlab_codegen
cd matlab_codegen
unzip -o ~/$($ latestPackage.Name) -d .
echo 'Extracted files:'
find . -type f | wc -l
"@

try {
    ssh "$Username@$OrinIP" $remoteCommands
    Write-Host "✓ Extraction complete" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "⚠ Extraction may have issues" -ForegroundColor Yellow
    Write-Host "Check manually on AGX Orin" -ForegroundColor Yellow
}

# Summary
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "✓ DEPLOYMENT COMPLETE" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps on AGX Orin:" -ForegroundColor Yellow
Write-Host "1. SSH to AGX Orin:" -ForegroundColor White
Write-Host "   ssh $Username@$OrinIP" -ForegroundColor Gray
Write-Host ""
Write-Host "2. Build ROS2 packages:" -ForegroundColor White
Write-Host "   cd ~/gikWBC9DOF/ros2" -ForegroundColor Gray
Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_msgs" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_solver" -ForegroundColor Gray
Write-Host ""
Write-Host "3. Test the solver:" -ForegroundColor White
Write-Host "   ros2 launch gik9dof_solver test_solver.launch.py" -ForegroundColor Gray
Write-Host ""
Write-Host "See FAST_TRACK_2DAY.md for detailed instructions" -ForegroundColor Yellow
