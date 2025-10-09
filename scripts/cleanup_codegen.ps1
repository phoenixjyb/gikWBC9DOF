# Codegen Cleanup Script - Safe Archive Mode
# Archives old codegen versions, deletes debug folder
# Run from project root: .\scripts\cleanup_codegen.ps1

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Codegen Folder Cleanup" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Verify we're in the right directory
if (!(Test-Path "codegen") -or !(Test-Path "ros2")) {
    Write-Host "❌ ERROR: Must run from project root!" -ForegroundColor Red
    Write-Host "Current directory: $(Get-Location)" -ForegroundColor Yellow
    exit 1
}

# Create archive directory
Write-Host "📁 Creating archive directory..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path "codegen/archive" | Out-Null

# Archive deprecated ARM64 with libccd
if (Test-Path "codegen/gik9dof_arm64_20constraints") {
    Write-Host "📦 Archiving: gik9dof_arm64_20constraints (libccd version)..." -ForegroundColor Yellow
    Move-Item "codegen/gik9dof_arm64_20constraints" `
        "codegen/archive/arm64_libccd_deprecated/" -Force
    Write-Host "   ✅ Moved to codegen/archive/arm64_libccd_deprecated/" -ForegroundColor Green
} else {
    Write-Host "   ⏭️  Already archived: gik9dof_arm64_20constraints" -ForegroundColor Gray
}

# Archive deprecated x64 Windows
if (Test-Path "codegen/gik9dof_x64_20constraints") {
    Write-Host "📦 Archiving: gik9dof_x64_20constraints (Windows PE/COFF)..." -ForegroundColor Yellow
    Move-Item "codegen/gik9dof_x64_20constraints" `
        "codegen/archive/x64_windows_deprecated/" -Force
    Write-Host "   ✅ Moved to codegen/archive/x64_windows_deprecated/" -ForegroundColor Green
} else {
    Write-Host "   ⏭️  Already archived: gik9dof_x64_20constraints" -ForegroundColor Gray
}

# Archive old WSL validation (MaxTime=10s v1)
if (Test-Path "codegen/x86_64_validation") {
    Write-Host "📦 Archiving: x86_64_validation (old validation)..." -ForegroundColor Yellow
    Move-Item "codegen/x86_64_validation" `
        "codegen/archive/x64_wsl_validation_v1/" -Force
    Write-Host "   ✅ Moved to codegen/archive/x64_wsl_validation_v1/" -ForegroundColor Green
} else {
    Write-Host "   ⏭️  Already archived: x86_64_validation" -ForegroundColor Gray
}

# Archive latest WSL validation (MaxTime=10s v2)
if (Test-Path "codegen/x86_64_validation_noCollision") {
    Write-Host "📦 Archiving: x86_64_validation_noCollision (final validation)..." -ForegroundColor Yellow
    Move-Item "codegen/x86_64_validation_noCollision" `
        "codegen/archive/x64_wsl_validation_final/" -Force
    Write-Host "   ✅ Moved to codegen/archive/x64_wsl_validation_final/" -ForegroundColor Green
} else {
    Write-Host "   ⏭️  Already archived: x86_64_validation_noCollision" -ForegroundColor Gray
}

# Delete debug folder
if (Test-Path "codegen/test_debug") {
    Write-Host "🗑️  Deleting: test_debug (temporary debug build)..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force "codegen/test_debug"
    Write-Host "   ✅ Deleted test_debug/" -ForegroundColor Green
} else {
    Write-Host "   ⏭️  Already deleted: test_debug" -ForegroundColor Gray
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Cleanup Complete!" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Show final structure
Write-Host "📂 Production Folders:" -ForegroundColor Green
if (Test-Path "codegen/arm64_realtime") {
    $files = (Get-ChildItem "codegen/arm64_realtime" -File -Recurse).Count
    $size = [math]::Round(((Get-ChildItem "codegen/arm64_realtime" -Recurse | Measure-Object -Property Length -Sum).Sum / 1MB), 2)
    Write-Host "   ✅ arm64_realtime/ ($files files, $size MB)" -ForegroundColor White
}
if (Test-Path "codegen/planner_arm64") {
    $files = (Get-ChildItem "codegen/planner_arm64" -File -Recurse).Count
    $size = [math]::Round(((Get-ChildItem "codegen/planner_arm64" -Recurse | Measure-Object -Property Length -Sum).Sum / 1MB), 2)
    Write-Host "   ✅ planner_arm64/ ($files files, $size MB)" -ForegroundColor White
}

Write-Host ""
Write-Host "📦 Archived Folders:" -ForegroundColor Yellow
if (Test-Path "codegen/archive") {
    Get-ChildItem "codegen/archive" -Directory | ForEach-Object {
        $files = (Get-ChildItem $_.FullName -File -Recurse).Count
        $size = [math]::Round(((Get-ChildItem $_.FullName -Recurse | Measure-Object -Property Length -Sum).Sum / 1MB), 2)
        Write-Host "   📁 $($_.Name)/ ($files files, $size MB)" -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "💡 Next Steps:" -ForegroundColor Cyan
Write-Host "   1. Review archived folders in codegen/archive/" -ForegroundColor White
Write-Host "   2. Run update_ros2_code.ps1 to update ROS2 with MaxTime=50ms" -ForegroundColor White
Write-Host "   3. Delete archive/ after 1 month if not needed" -ForegroundColor White
Write-Host ""
