# run_planner_codegen.ps1
# PowerShell wrapper to run planner ARM64 codegen in WSL
# Executes Phase 4 of PLANNER_REGEN_PLAN.md

$ErrorActionPreference = "Stop"

Write-Host "========================================"  -ForegroundColor Cyan
Write-Host "Planner ARM64 Codegen - Linux MATLAB" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Phase 4 of PLANNER_REGEN_PLAN.md" -ForegroundColor Gray
Write-Host ""

# Make script executable
Write-Host "Making script executable..." -ForegroundColor Yellow
wsl bash -c "chmod +x /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/run_planner_codegen_wsl.sh"

Write-Host "Starting Linux MATLAB codegen..." -ForegroundColor Yellow
Write-Host "⏱️  Expected time: 5-15 minutes" -ForegroundColor Gray
Write-Host ""

# Run the script
wsl bash -c "/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/run_planner_codegen_wsl.sh"

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "✅ Codegen completed successfully!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Cyan
    Write-Host "  1. Review generated code in codegen/planner_arm64/" -ForegroundColor White
    Write-Host "  2. Run Phase 5: Verify generated code" -ForegroundColor White
    Write-Host "  3. Run Phase 6: Copy to ROS2" -ForegroundColor White
    Write-Host ""
    Write-Host "See PLANNER_REGEN_PLAN.md for details" -ForegroundColor Gray
} else {
    Write-Host ""
    Write-Host "❌ Codegen failed (exit code: $LASTEXITCODE)" -ForegroundColor Red
    Write-Host "Check error messages above" -ForegroundColor Yellow
    exit 1
}
