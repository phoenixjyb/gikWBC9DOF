# Check if Build is Current
# Compares source file timestamps with build artifacts

param(
    [string]$SourceDir = "matlab",
    [string]$BuildDir = "codegen/arm64_realtime",
    [switch]$Verbose
)

Write-Host "=== Build Currency Check ===" -ForegroundColor Cyan
Write-Host ""

# Check if directories exist
if (-not (Test-Path $SourceDir)) {
    Write-Host "❌ Source directory not found: $SourceDir" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $BuildDir)) {
    Write-Host "❌ Build directory not found: $BuildDir" -ForegroundColor Red
    Write-Host "Have you built the project yet?" -ForegroundColor Yellow
    exit 1
}

# Get latest source file timestamp
$sourceFiles = Get-ChildItem -Path $SourceDir -Recurse -File -Include "*.m"
if ($sourceFiles.Count -eq 0) {
    Write-Host "❌ No MATLAB source files found in $SourceDir" -ForegroundColor Red
    exit 1
}

$latestSourceFile = $sourceFiles | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$latestSourceTime = $latestSourceFile.LastWriteTime

Write-Host "Latest source file:" -ForegroundColor White
Write-Host "  File: $($latestSourceFile.FullName)" -ForegroundColor Gray
Write-Host "  Time: $latestSourceTime" -ForegroundColor Gray
Write-Host ""

# Get oldest build artifact timestamp
$buildFiles = Get-ChildItem -Path $BuildDir -Recurse -File -Include "*.cpp","*.h","*.o"
if ($buildFiles.Count -eq 0) {
    Write-Host "❌ No build artifacts found in $BuildDir" -ForegroundColor Red
    Write-Host "Build may have failed or not started" -ForegroundColor Yellow
    exit 1
}

$oldestBuildFile = $buildFiles | Sort-Object LastWriteTime | Select-Object -First 1
$oldestBuildTime = $oldestBuildFile.LastWriteTime

Write-Host "Oldest build artifact:" -ForegroundColor White
Write-Host "  File: $($oldestBuildFile.FullName)" -ForegroundColor Gray
Write-Host "  Time: $oldestBuildTime" -ForegroundColor Gray
Write-Host ""

# Compare timestamps
$timeDiff = ($oldestBuildTime - $latestSourceTime).TotalSeconds

Write-Host "=== Analysis ===" -ForegroundColor Cyan
Write-Host ""

if ($timeDiff -gt 0) {
    Write-Host "✓ Build is CURRENT" -ForegroundColor Green
    Write-Host "  Build is $([Math]::Abs($timeDiff)) seconds newer than latest source" -ForegroundColor Gray
    Write-Host ""
    Write-Host "Status: UP TO DATE ✓" -ForegroundColor Green
    exit 0
} else {
    Write-Host "⚠️ Build is OUTDATED" -ForegroundColor Yellow
    Write-Host "  Source is $([Math]::Abs($timeDiff)) seconds newer than build" -ForegroundColor Gray
    Write-Host ""
    
    # Show which files changed
    Write-Host "Files modified since build:" -ForegroundColor Yellow
    $modifiedFiles = $sourceFiles | Where-Object { $_.LastWriteTime -gt $oldestBuildTime } | Select-Object -First 10
    foreach ($file in $modifiedFiles) {
        Write-Host "  - $($file.Name) ($($file.LastWriteTime))" -ForegroundColor Gray
    }
    
    if ($sourceFiles.Count -gt 10) {
        Write-Host "  ... and $(($sourceFiles | Where-Object { $_.LastWriteTime -gt $oldestBuildTime }).Count - 10) more" -ForegroundColor Gray
    }
    
    Write-Host ""
    Write-Host "Status: REBUILD REQUIRED ⚠️" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Action: Run code generation again" -ForegroundColor White
    exit 1
}
