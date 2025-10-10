# run_new_codegen.ps1
# PowerShell wrapper to run new feature code generation in WSL
#
# Usage: .\scripts\codegen\run_new_codegen.ps1

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "New Feature Code Generation" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$scriptPath = $PSScriptRoot
$workspaceRoot = Split-Path -Parent (Split-Path -Parent $scriptPath)

# Fix line endings for WSL
Write-Host "Fixing line endings for WSL..." -ForegroundColor Yellow
wsl bash -c "cd '$($workspaceRoot -replace '\\', '/')' && dos2unix scripts/codegen/run_new_codegen_wsl.sh 2>/dev/null || sed -i 's/\r$//' scripts/codegen/run_new_codegen_wsl.sh"

# Make executable
wsl bash -c "cd '$($workspaceRoot -replace '\\', '/')' && chmod +x scripts/codegen/run_new_codegen_wsl.sh"

Write-Host "Running code generation in WSL..." -ForegroundColor Green
Write-Host ""

# Run the WSL script
wsl bash scripts/codegen/run_new_codegen_wsl.sh

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "✓ New feature code generation complete!" -ForegroundColor Green
    Write-Host ""
    Write-Host "New components:" -ForegroundColor Cyan
    Write-Host "  1. chassis_controller_arm64/ - Multi-mode chassis control" -ForegroundColor White
    Write-Host "  2. rs_smoothing_arm64/ - Clothoid path smoothing" -ForegroundColor White
    Write-Host ""
    Write-Host "Total: 4 existing + 2 new = 6 codegen components" -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "❌ Code generation failed!" -ForegroundColor Red
    Write-Host "Check the output above for errors." -ForegroundColor Yellow
    exit 1
}
