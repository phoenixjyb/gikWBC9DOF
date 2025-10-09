#!/usr/bin/env pwsh
# Copy MATLAB Coder ARM64 generated code to ROS2 package
# Includes all necessary files from html/generatedSource subdirectory

param(
    [switch]$Force = $false
)

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Copy ARM64 GIK Code to ROS2 Package" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# Paths
$codegenSrc = ".\codegen\gik9dof_arm64_20constraints"
$htmlSrc = "$codegenSrc\html\generatedSource"
$ros2Dst = ".\ros2\gik9dof_solver\matlab_codegen\include"

# Verify source exists
if (-not (Test-Path $codegenSrc)) {
    Write-Host "❌ ERROR: Source directory not found: $codegenSrc" -ForegroundColor Red
    Write-Host "   Please generate ARM64 code first using:" -ForegroundColor Yellow
    Write-Host "   matlab -batch `"cd('matlab'); run('generate_gik_20constraints_arm64.m')`"" -ForegroundColor Yellow
    exit 1
}

if (-not (Test-Path $htmlSrc)) {
    Write-Host "❌ ERROR: html/generatedSource not found: $htmlSrc" -ForegroundColor Red
    exit 1
}

# Create destination if needed
if (-not (Test-Path $ros2Dst)) {
    Write-Host "📁 Creating destination directory: $ros2Dst" -ForegroundColor Yellow
    New-Item -ItemType Directory -Path $ros2Dst -Force | Out-Null
}

# Check if destination already has files
$existingFiles = Get-ChildItem -Path $ros2Dst -File -ErrorAction SilentlyContinue
if ($existingFiles -and -not $Force) {
    Write-Host "⚠️  WARNING: Destination already contains files" -ForegroundColor Yellow
    $response = Read-Host "Overwrite existing files? (y/N)"
    if ($response -ne 'y' -and $response -ne 'Y') {
        Write-Host "❌ Aborted by user" -ForegroundColor Red
        exit 1
    }
}

Write-Host ""
Write-Host "📦 Copying generated code..." -ForegroundColor Green

# Step 1: Copy all .cpp and .h files from main codegen directory
Write-Host "  ✓ Copying main codegen files (.cpp, .h)..." -ForegroundColor Gray
$cppFiles = Get-ChildItem -Path $codegenSrc -Filter "*.cpp" -File
$hFiles = Get-ChildItem -Path $codegenSrc -Filter "*.h" -File
$copiedCount = 0
foreach ($file in $cppFiles) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $copiedCount++
}
foreach ($file in $hFiles) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $copiedCount++
}
Write-Host "    Copied $copiedCount files" -ForegroundColor Gray

# Step 2: Copy critical header from html/generatedSource
Write-Host "  ✓ Copying tmwtypes.h (MATLAB type definitions)..." -ForegroundColor Gray
Copy-Item -Path "$htmlSrc\tmwtypes.h" -Destination $ros2Dst -Force

# Step 3: Copy collision detection headers (.hpp)
Write-Host "  ✓ Copying collision headers (.hpp)..." -ForegroundColor Gray
$collisionHeaders = Get-ChildItem -Path $htmlSrc -Filter "collision*.hpp"
$collisionHeaderCount = 0
foreach ($file in $collisionHeaders) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $collisionHeaderCount++
}
Write-Host "    Copied $collisionHeaderCount collision headers" -ForegroundColor Gray

# Step 4: Copy collision detection sources (.cpp)
Write-Host "  ✓ Copying collision sources (.cpp)..." -ForegroundColor Gray
$collisionSources = Get-ChildItem -Path $htmlSrc -Filter "collision*.cpp"
$collisionSourceCount = 0
foreach ($file in $collisionSources) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $collisionSourceCount++
}
Write-Host "    Copied $collisionSourceCount collision sources" -ForegroundColor Gray

# Step 5: Copy CCD library headers (.h)
Write-Host "  ✓ Copying CCD headers (ccd_*.h)..." -ForegroundColor Gray
$ccdHeaders = Get-ChildItem -Path $htmlSrc -Filter "ccd_*.h"
$ccdHeaderCount = 0
foreach ($file in $ccdHeaders) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $ccdHeaderCount++
}
Write-Host "    Copied $ccdHeaderCount CCD headers" -ForegroundColor Gray

# Step 6: Copy all C sources (.c)
Write-Host "  ✓ Copying C sources (ccd_*.c, coder_posix_time.c)..." -ForegroundColor Gray
$cSources = Get-ChildItem -Path $htmlSrc -Filter "*.c"
$cSourceCount = 0
foreach ($file in $cSources) {
    Copy-Item -Path $file.FullName -Destination $ros2Dst -Force
    $cSourceCount++
}
Write-Host "    Copied $cSourceCount C sources" -ForegroundColor Gray

# Step 7: Overwrite CoderTimeAPI.cpp from html (may have platform-specific code)
Write-Host "  ✓ Copying CoderTimeAPI.cpp (time functions)..." -ForegroundColor Gray
if (Test-Path "$htmlSrc\CoderTimeAPI.cpp") {
    Copy-Item -Path "$htmlSrc\CoderTimeAPI.cpp" -Destination $ros2Dst -Force
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "✅ Copy Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan

# Summary
$totalFiles = Get-ChildItem -Path $ros2Dst -File | Measure-Object
Write-Host ""
Write-Host "📊 Summary:" -ForegroundColor Cyan
Write-Host "  Source:      $codegenSrc" -ForegroundColor Gray
Write-Host "  Destination: $ros2Dst" -ForegroundColor Gray
Write-Host "  Total files: $($totalFiles.Count)" -ForegroundColor Gray
Write-Host ""
Write-Host "Expected file count: ~65 files (60 .cpp + 5 .c)" -ForegroundColor Yellow
Write-Host "  - Main sources:      ~56 .cpp files" -ForegroundColor Gray
Write-Host "  - Collision:         ~4 .cpp files" -ForegroundColor Gray
Write-Host "  - CCD library:       ~5 .c files" -ForegroundColor Gray
Write-Host "  - Headers:           All .h/.hpp files" -ForegroundColor Gray

if ($totalFiles.Count -lt 60) {
    Write-Host ""
    Write-Host "⚠️  WARNING: File count seems low. Expected ~65 files." -ForegroundColor Yellow
    Write-Host "   Please verify code generation completed successfully." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Next steps:" -ForegroundColor Cyan
Write-Host "  1. Update ros2/gik9dof_solver/CMakeLists.txt to include .c files" -ForegroundColor Gray
Write-Host "  2. Update gik9dof_solver_node.cpp for 7-parameter interface" -ForegroundColor Gray
Write-Host "  3. Build: cd ros2 && colcon build --packages-select gik9dof_solver" -ForegroundColor Gray
Write-Host ""
