# run_planner_copy_to_ros2.ps1
# Phase 6: Copy regenerated planner to ROS2
# Part of PLANNER_REGEN_PLAN.md

$ErrorActionPreference = "Stop"

Write-Host "========================================"  -ForegroundColor Cyan
Write-Host "Phase 6: Copy Planner to ROS2" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$source = "codegen/planner_arm64"
$dest = "ros2/gik9dof_solver/src/generated/planner"

# Verify source exists
if (-not (Test-Path $source)) {
    Write-Host "❌ Error: Source not found: $source" -ForegroundColor Red
    Write-Host "Run Phase 4 first: .\run_planner_codegen.ps1" -ForegroundColor Yellow
    exit 1
}

# Count source files
$sourceFiles = (Get-ChildItem $source -File -Recurse).Count
Write-Host "Source: $source" -ForegroundColor White
Write-Host "  Files: $sourceFiles" -ForegroundColor Gray

# Clear destination (except backups)
Write-Host "`nClearing destination..." -ForegroundColor Yellow
Get-ChildItem $dest -Exclude "*backup*" -ErrorAction SilentlyContinue | Remove-Item -Recurse -Force

# Copy new code
Write-Host "Copying files..." -ForegroundColor Yellow
Copy-Item "$source\*" $dest -Recurse -Force

# Verify copy
$destFiles = (Get-ChildItem $dest -File -Recurse).Count
Write-Host "`nDestination: $dest" -ForegroundColor White
Write-Host "  Files: $destFiles" -ForegroundColor Gray

if ($sourceFiles -eq $destFiles) {
    Write-Host "`n✅ Phase 6 Complete: Planner copied to ROS2" -ForegroundColor Green
    Write-Host "  $destFiles files copied successfully" -ForegroundColor White
    
    # Show sample files
    Write-Host "`nSample files:" -ForegroundColor Cyan
    Get-ChildItem "$dest\*.cpp" | Select-Object -First 5 Name, LastWriteTime | Format-Table -AutoSize
    
    Write-Host "`nNext: Run Phase 7 verification" -ForegroundColor Yellow
    Write-Host "  .\run_planner_verify.ps1" -ForegroundColor Gray
} else {
    Write-Host "`n⚠️ Warning: File count mismatch" -ForegroundColor Yellow
    Write-Host "  Source: $sourceFiles files" -ForegroundColor White
    Write-Host "  Dest:   $destFiles files" -ForegroundColor White
    Write-Host "Check for errors" -ForegroundColor Gray
}
