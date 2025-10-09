# Deploy to AGX Orin - Complete Workspace Transfer
# Usage: .\deploy_to_orin_complete.ps1 -OrinIP "192.168.100.150" [-Username "cr"]

param(
    [Parameter(Mandatory=$true)]
    [string]$OrinIP,
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "cr",  # Updated default username for Orin
    # user name is cr; but we drop stuff to /home/nvidia/ . 
    [Parameter(Mandatory=$false)]
    # [string]$RemotePath = "/home/nvidia/camo_9dof/gikWBC9DOF",  # Updated deployment path
    [string]$RemotePath = "/home/nvidia/temp_gikrepo",  # Updated deployment path

    [Parameter(Mandatory=$false)]
    [switch]$SkipMeshes = $false
)

$ErrorActionPreference = "Stop"

Write-Host "`n" -NoNewline
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "   AGX Orin Complete Deployment" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "Target: $Username@$OrinIP" -ForegroundColor White
Write-Host "Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Gray
Write-Host "================================================`n" -ForegroundColor Cyan

# Verify we're in the right directory
# $PSScriptRoot is scripts/deployment/, so go up two levels
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
if (-not (Test-Path (Join-Path $projectRoot "ros2"))) {
    Write-Host "✗ Error: ros2/ directory not found" -ForegroundColor Red
    Write-Host "  Project root: $projectRoot" -ForegroundColor Yellow
    exit 1
}

# Test SSH connection
Write-Host "[1/6] Testing SSH connection..." -ForegroundColor Yellow
Write-Host "  You may be prompted for password multiple times" -ForegroundColor Gray
try {
    $sshTest = ssh -o ConnectTimeout=10 "$Username@$OrinIP" "echo 'OK'" 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "✗ SSH connection failed" -ForegroundColor Red
        Write-Host "  Check IP address and username" -ForegroundColor Yellow
        exit 1
    }
    Write-Host "✓ SSH connection successful`n" -ForegroundColor Green
} catch {
    Write-Host "✗ Cannot connect to $OrinIP" -ForegroundColor Red
    Write-Host "  Error: $_" -ForegroundColor Yellow
    exit 1
}

# Create workspace on Orin
Write-Host "[2/6] Creating workspace on AGX Orin..." -ForegroundColor Yellow
Write-Host "  Target path: $RemotePath" -ForegroundColor Gray
try {
    ssh "$Username@$OrinIP" "mkdir -p $RemotePath/ros2 $RemotePath/meshes"
    Write-Host "✓ Workspace directories created`n" -ForegroundColor Green
} catch {
    Write-Host "⚠ Warning: Could not create directories (may already exist)`n" -ForegroundColor Yellow
}

# Transfer ROS2 workspace
Write-Host "[3/6] Transferring ROS2 workspace..." -ForegroundColor Yellow
Write-Host "  This may take 2-5 minutes depending on network speed" -ForegroundColor Gray

$ros2Source = Join-Path $projectRoot "ros2"

# Check if WSL rsync is available (faster)
$useRsync = $false
try {
    wsl which rsync > $null 2>&1
    if ($LASTEXITCODE -eq 0) {
        $useRsync = $true
        Write-Host "  Using WSL rsync for faster transfer..." -ForegroundColor Cyan
    }
} catch {
    Write-Host "  Using scp for transfer..." -ForegroundColor Cyan
}

if ($useRsync) {
    # Convert Windows path to WSL path
    $wslPath = $ros2Source -replace '\\', '/' -replace 'C:', '/mnt/c'
    
    # Use rsync (much faster, shows progress)
    try {
        wsl rsync -avz --progress `
            --exclude='build' --exclude='install' --exclude='log' `
            --exclude='*_bak' --exclude='*_backup_*' `
            "$wslPath/" "$Username@${OrinIP}:$RemotePath/ros2/"
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✓ ROS2 workspace transferred`n" -ForegroundColor Green
        } else {
            throw "rsync failed"
        }
    } catch {
        Write-Host "⚠ rsync failed, falling back to scp..." -ForegroundColor Yellow
        $useRsync = $false
    }
}

if (-not $useRsync) {
    # Fallback: Create ZIP and transfer via scp
    Write-Host "  Creating temporary archive..." -ForegroundColor Gray
    $tempZip = Join-Path $env:TEMP "gikwbc9dof_ros2_$(Get-Date -Format 'yyyyMMdd_HHmmss').zip"
    
    $packagesDir = Join-Path $ros2Source "*"
    Compress-Archive -Path $packagesDir -DestinationPath $tempZip -Force `
        -CompressionLevel Optimal
    
    $zipSize = [math]::Round((Get-Item $tempZip).Length / 1MB, 2)
    Write-Host "  Archive size: $zipSize MB" -ForegroundColor Gray
    Write-Host "  Transferring..." -ForegroundColor Gray
    
    scp $tempZip "$Username@${OrinIP}:/tmp/gikwbc9dof_ros2.zip"
    
    Write-Host "  Extracting on Orin..." -ForegroundColor Gray
    ssh "$Username@$OrinIP" "cd $RemotePath && unzip -o /tmp/gikwbc9dof_ros2.zip -d ros2/ && rm /tmp/gikwbc9dof_ros2.zip"
    
    Remove-Item $tempZip -Force
    Write-Host "✓ ROS2 workspace transferred`n" -ForegroundColor Green
}

# Transfer URDF
Write-Host "[4/6] Transferring URDF file..." -ForegroundColor Yellow
$urdfFile = Join-Path $projectRoot "mobile_manipulator_PPR_base_corrected.urdf"
if (Test-Path $urdfFile) {
    scp $urdfFile "$Username@${OrinIP}:$RemotePath/"
    Write-Host "✓ URDF transferred`n" -ForegroundColor Green
} else {
    Write-Host "⚠ URDF file not found (may not be needed)`n" -ForegroundColor Yellow
}

# Transfer meshes (optional)
if (-not $SkipMeshes) {
    Write-Host "[5/6] Transferring mesh files..." -ForegroundColor Yellow
    $meshesDir = Join-Path $projectRoot "meshes"
    if (Test-Path $meshesDir) {
        Write-Host "  This may take a minute..." -ForegroundColor Gray
        
        if ($useRsync) {
            $wslMeshPath = $meshesDir -replace '\\', '/' -replace 'C:', '/mnt/c'
            wsl rsync -avz --progress "$wslMeshPath/" "$Username@${OrinIP}:$RemotePath/meshes/"
        } else {
            $meshZip = Join-Path $env:TEMP "meshes.zip"
            Compress-Archive -Path (Join-Path $meshesDir "*") -DestinationPath $meshZip -Force
            scp $meshZip "$Username@${OrinIP}:/tmp/meshes.zip"
            ssh "$Username@$OrinIP" "cd $RemotePath/meshes && unzip -o /tmp/meshes.zip && rm /tmp/meshes.zip"
            Remove-Item $meshZip -Force
        }
        Write-Host "✓ Meshes transferred`n" -ForegroundColor Green
    } else {
        Write-Host "⚠ Meshes directory not found (skipping)`n" -ForegroundColor Yellow
    }
} else {
    Write-Host "[5/6] Skipping mesh files (use -SkipMeshes:$false to include)`n" -ForegroundColor Gray
}

# Verify transfer
Write-Host "[6/6] Verifying transfer..." -ForegroundColor Yellow
$verifyScript = @"
echo "Checking ROS2 workspace..."
if [ -d $RemotePath/ros2/gik9dof_msgs ]; then
    echo "✓ gik9dof_msgs found"
else
    echo "✗ gik9dof_msgs NOT found"
fi

if [ -d $RemotePath/ros2/gik9dof_solver ]; then
    echo "✓ gik9dof_solver found"
else
    echo "✗ gik9dof_solver NOT found"
fi

if [ -d $RemotePath/ros2/gik9dof_solver/matlab_codegen ]; then
    file_count=`$(find $RemotePath/ros2/gik9dof_solver/matlab_codegen -type f | wc -l)
    echo "✓ matlab_codegen found (`$file_count files)"
else
    echo "✗ matlab_codegen NOT found"
fi

if [ -d $RemotePath/ros2/gik9dof_solver/src/velocity_controller ]; then
    echo "✓ velocity_controller found"
else
    echo "✗ velocity_controller NOT found"
fi

if [ -f $RemotePath/mobile_manipulator_PPR_base_corrected.urdf ]; then
    echo "✓ URDF file found"
else
    echo "⚠ URDF file not found"
fi

echo ""
echo "Disk usage:"
du -sh $RemotePath
"@

ssh "$Username@$OrinIP" $verifyScript

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "   ✓ DEPLOYMENT COMPLETE!" -ForegroundColor Green
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# Print next steps
Write-Host "📋 Next Steps on AGX Orin:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. SSH to AGX Orin:" -ForegroundColor White
Write-Host "   ssh $Username@$OrinIP" -ForegroundColor Gray
Write-Host ""
Write-Host "2. Install dependencies (first time only):" -ForegroundColor White
Write-Host "   sudo apt update" -ForegroundColor Gray
Write-Host "   sudo apt install -y ros-humble-rclcpp libeigen3-dev libomp-dev" -ForegroundColor Gray
Write-Host ""
Write-Host "3. Build ROS2 packages:" -ForegroundColor White
Write-Host "   cd $RemotePath/ros2" -ForegroundColor Gray
Write-Host "   source /opt/ros/humble/setup.bash" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_msgs" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host "   colcon build --packages-select gik9dof_solver" -ForegroundColor Gray
Write-Host ""
Write-Host "4. Test the solver:" -ForegroundColor White
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host "   ros2 run gik9dof_solver gik9dof_solver_node\" -ForegroundColor Gray
Write-Host "       --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml" -ForegroundColor Gray
Write-Host ""
Write-Host "📚 For detailed instructions, see:" -ForegroundColor Cyan
Write-Host "   docs/deployment/ORIN_DEPLOYMENT_GUIDE.md" -ForegroundColor Gray
Write-Host ""
