# WSL Codegen with Version Tracking (Windows Launcher)
# Launches WSL MATLAB codegen with automatic build versioning

param(
    [string]$CodegenScript = "generate_code_arm64.m"
)

Write-Host "========================================"  -ForegroundColor Cyan
Write-Host "WSL MATLAB Codegen (with versioning)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Make scripts executable
Write-Host "Setting up WSL scripts..." -ForegroundColor Yellow
wsl bash -c "cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF && chmod +x scripts/codegen/*.sh scripts/deployment/*.sh"

# Run codegen with version tracking
Write-Host "Starting codegen in WSL..." -ForegroundColor Yellow
Write-Host ""

wsl bash -c "cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF && bash scripts/codegen/run_codegen_wsl_with_version.sh $CodegenScript"

$exitCode = $LASTEXITCODE

Write-Host ""
if ($exitCode -eq 0) {
    Write-Host "✓ Codegen completed successfully!" -ForegroundColor Green
    
    # Show build info
    if (Test-Path "codegen\arm64_realtime\BUILD_INFO.txt") {
        Write-Host ""
        Write-Host "Build Information:" -ForegroundColor Cyan
        Get-Content "codegen\arm64_realtime\BUILD_INFO.txt" | Select-Object -First 15
    }
    
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Yellow
    Write-Host "  1. Copy to ROS2: scripts\deployment\run_planner_copy_to_ros2.ps1" -ForegroundColor White
    Write-Host "  2. Check build: wsl bash scripts/deployment/check_build_current_wsl.sh" -ForegroundColor White
    
} else {
    Write-Host "❌ Codegen failed (exit code: $exitCode)" -ForegroundColor Red
    Write-Host "Check error messages above" -ForegroundColor Yellow
}

exit $exitCode
