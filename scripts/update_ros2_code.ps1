# Update ROS2 with Latest ARM64 Code (MaxTime=50ms)
# Copies regenerated arm64_realtime code to ROS2 workspace
# Run from project root: .\scripts\update_ros2_code.ps1

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  ROS2 Code Update Script" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Verify we're in the right directory
if (!(Test-Path "codegen/arm64_realtime") -or !(Test-Path "ros2/gik9dof_solver")) {
    Write-Host "❌ ERROR: Must run from project root!" -ForegroundColor Red
    Write-Host "Current directory: $(Get-Location)" -ForegroundColor Yellow
    exit 1
}

$src = "codegen/arm64_realtime"
$dst = "ros2/gik9dof_solver/matlab_codegen/include"

# Check source exists and is recent
if (!(Test-Path $src)) {
    Write-Host "❌ ERROR: Source not found: $src" -ForegroundColor Red
    exit 1
}

$srcLastModified = (Get-Item $src).LastWriteTime
Write-Host "📂 Source: $src" -ForegroundColor White
Write-Host "   Last modified: $srcLastModified" -ForegroundColor Gray

# Check for MaxTime=50ms in source
Write-Host ""
Write-Host "🔍 Verifying MaxTime=50ms in source code..." -ForegroundColor Yellow
$gikSolverCpp = "$src/GIKSolver.cpp"
if (Test-Path $gikSolverCpp) {
    $maxTimeFound = Select-String -Path $gikSolverCpp -Pattern "0\.05" -Quiet
    if ($maxTimeFound) {
        Write-Host "   ✅ MaxTime=0.05 (50ms) verified in GIKSolver.cpp" -ForegroundColor Green
    } else {
        Write-Host "   ⚠️  WARNING: MaxTime=0.05 not found! Code may not be updated." -ForegroundColor Red
        Write-Host "   Run: matlab -batch `"run('scripts/codegen/generate_code_arm64.m')`"" -ForegroundColor Yellow
        $continue = Read-Host "Continue anyway? (y/N)"
        if ($continue -ne 'y') {
            exit 1
        }
    }
} else {
    Write-Host "   ⚠️  GIKSolver.cpp not found, skipping verification" -ForegroundColor Yellow
}

# Backup existing ROS2 code
if (Test-Path $dst) {
    $backupDir = "ros2/gik9dof_solver/matlab_codegen/include_backup_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
    Write-Host ""
    Write-Host "💾 Creating backup of existing ROS2 code..." -ForegroundColor Yellow
    Copy-Item -Recurse $dst $backupDir
    Write-Host "   ✅ Backup created: $backupDir" -ForegroundColor Green
}

# Clear destination
Write-Host ""
Write-Host "🗑️  Clearing destination: $dst" -ForegroundColor Yellow
if (Test-Path $dst) {
    Remove-Item -Recurse -Force "$dst/*"
} else {
    New-Item -ItemType Directory -Force -Path $dst | Out-Null
}

# Copy new code
Write-Host "📦 Copying ARM64 code to ROS2 workspace..." -ForegroundColor Yellow
Copy-Item -Recurse "$src/*" "$dst/"

# Verify copy
$copiedFiles = (Get-ChildItem $dst -File -Recurse).Count
Write-Host "   ✅ Copied $copiedFiles files" -ForegroundColor Green

# Verify key files
Write-Host ""
Write-Host "🔍 Verifying key files..." -ForegroundColor Yellow
$keyFiles = @(
    "GIKSolver.h",
    "GIKSolver.cpp",
    "buildRobotForCodegen.h",
    "generalizedInverseKinematics.h"
)

foreach ($file in $keyFiles) {
    if (Test-Path "$dst/$file") {
        Write-Host "   ✅ $file" -ForegroundColor Green
    } else {
        Write-Host "   ❌ MISSING: $file" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  ROS2 Update Complete!" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "📊 Summary:" -ForegroundColor White
Write-Host "   Source: $src" -ForegroundColor Gray
Write-Host "   Destination: $dst" -ForegroundColor Gray
Write-Host "   Files copied: $copiedFiles" -ForegroundColor Gray
Write-Host "   MaxTime: 50ms (0.05s)" -ForegroundColor Green
Write-Host "   MaxIterations: 1000" -ForegroundColor Green
Write-Host ""

Write-Host "🚀 Next Steps (on Jetson Orin):" -ForegroundColor Cyan
Write-Host "   1. Deploy updated ros2/ folder to Orin" -ForegroundColor White
Write-Host "   2. cd ros2/" -ForegroundColor White
Write-Host "   3. colcon build --packages-select gik9dof_solver" -ForegroundColor White
Write-Host "   4. source install/setup.bash" -ForegroundColor White
Write-Host "   5. ros2 run gik9dof_solver gik9dof_solver_node" -ForegroundColor White
Write-Host ""

Write-Host "💡 Deployment Command (from Windows/WSL):" -ForegroundColor Cyan
Write-Host "   scp -r ros2/ orin@jetson-orin:/home/orin/ros2_ws/src/gikWBC9DOF/" -ForegroundColor White
Write-Host ""
