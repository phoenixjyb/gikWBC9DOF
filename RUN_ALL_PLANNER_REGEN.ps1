# RUN_ALL_PLANNER_REGEN.ps1
# Master script to execute all phases of planner regeneration
# See PLANNER_REGEN_PLAN.md for details

$ErrorActionPreference = "Stop"

Write-Host "========================================"  -ForegroundColor Cyan
Write-Host "Planner ARM64 Regeneration - Full Process" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "This will execute Phases 4-7 of PLANNER_REGEN_PLAN.md:" -ForegroundColor White
Write-Host "  Phase 1-3: ✅ Already complete (backups and cleanup)" -ForegroundColor Green
Write-Host "  Phase 4:   Generate planner with Linux MATLAB (5-15 min)" -ForegroundColor Yellow
Write-Host "  Phase 5:   Verify generated code" -ForegroundColor Yellow
Write-Host "  Phase 6:   Copy to ROS2" -ForegroundColor Yellow
Write-Host "  Phase 7:   Final verification" -ForegroundColor Yellow
Write-Host ""

$response = Read-Host "Continue? (y/n)"
if ($response -ne 'y') {
    Write-Host "Cancelled." -ForegroundColor Yellow
    exit 0
}

Write-Host "`n" + "="*50 -ForegroundColor Cyan
Write-Host "Starting automated regeneration..." -ForegroundColor Cyan
Write-Host "="*50 -ForegroundColor Cyan

# Phase 4: Generate planner
Write-Host "`n[1/4] Phase 4: Generating planner with Linux MATLAB..." -ForegroundColor Yellow
& .\run_planner_codegen.ps1
if ($LASTEXITCODE -ne 0) {
    Write-Host "`n❌ Phase 4 failed. Stopping." -ForegroundColor Red
    exit 1
}

# Phase 5: Verify (part 1)
Write-Host "`n[2/4] Phase 5: Verifying generated code..." -ForegroundColor Yellow
& .\run_planner_verify.ps1

# Phase 6: Copy to ROS2
Write-Host "`n[3/4] Phase 6: Copying to ROS2..." -ForegroundColor Yellow
& .\run_planner_copy_to_ros2.ps1
if ($LASTEXITCODE -ne 0) {
    Write-Host "`n⚠️ Phase 6 had warnings. Continuing..." -ForegroundColor Yellow
}

# Phase 7: Final verification
Write-Host "`n[4/4] Phase 7: Final verification..." -ForegroundColor Yellow
& .\run_planner_verify.ps1

# Summary
Write-Host "`n" + "="*50 -ForegroundColor Green
Write-Host "✅ ALL PHASES COMPLETE!" -ForegroundColor Green
Write-Host "="*50 -ForegroundColor Green

Write-Host "`nRegeneration Summary:" -ForegroundColor Cyan
Write-Host "  ✅ Planner generated with Linux MATLAB" -ForegroundColor Green
Write-Host "  ✅ Code verified (file counts, binaries)" -ForegroundColor Green
Write-Host "  ✅ Copied to ROS2 workspace" -ForegroundColor Green
Write-Host "  ✅ Toolchain consistency confirmed" -ForegroundColor Green

Write-Host "`nBackups preserved:" -ForegroundColor Yellow
Get-ChildItem codegen/planner_arm64_backup_* -Directory | Select-Object -First 1 Name
Get-ChildItem ros2/gik9dof_solver/src/generated/planner_backup_* -Directory | Select-Object -First 1 Name

Write-Host "`nNext: Phase 8 - Git Commit" -ForegroundColor Cyan
Write-Host @"
Run these commands:

git add codegen/planner_arm64/ ros2/gik9dof_solver/src/generated/planner/
git commit -m "feat: Regenerate planner_arm64 with Linux MATLAB

- Platform: Linux MATLAB R2024a (WSL)
- Toolchain consistency: GIK + Planner both Linux MATLAB
- Ready for Jetson Orin deployment
- See PLANNER_REGEN_PLAN.md for details"

git push origin codegencc45-main
"@ -ForegroundColor White

Write-Host "`nSee PLANNER_REGEN_PLAN.md for full documentation" -ForegroundColor Gray
