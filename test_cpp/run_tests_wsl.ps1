# Quick WSL Test Runner
# Run from Windows PowerShell to automatically build and test in WSL

Write-Host "=============================================="
Write-Host "GIK 20-Constraint C++ Test - WSL Runner"
Write-Host "=============================================="

# Convert Windows path to WSL path
$windowsPath = "C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\test_cpp"
$wslPath = "/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/test_cpp"

Write-Host "Launching WSL and running tests..."
Write-Host ""

# Run build and test in WSL
wsl bash -c "cd '$wslPath' && chmod +x build_wsl.sh && ./build_wsl.sh && echo '' && echo 'Running tests...' && echo '=============================================='  && ./build_wsl/bin/test_gik_20constraints"

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "=============================================="
    Write-Host "✅ Tests completed successfully!"
    Write-Host "=============================================="
} else {
    Write-Host ""
    Write-Host "=============================================="
    Write-Host "❌ Tests failed or build error occurred"
    Write-Host "=============================================="
    exit 1
}
