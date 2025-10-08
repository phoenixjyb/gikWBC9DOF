# Build script for GIK 20-Constraint C++ Test
# Requires: CMake, Visual Studio (or MSVC compiler)

Write-Host "========================================"
Write-Host "Building GIK 20-Constraint C++ Test"
Write-Host "========================================"

# Check for x64 codegen directory
$codegenDir = "..\codegen\gik9dof_x64_20constraints"
if (-not (Test-Path $codegenDir)) {
    Write-Host "❌ Error: x64 codegen directory not found: $codegenDir"
    Write-Host "Please run generate_gik_20constraints_x64.m first!"
    exit 1
}

Write-Host "✅ Found x64 codegen directory"

# Create build directory
$buildDir = "build"
if (Test-Path $buildDir) {
    Write-Host "Cleaning existing build directory..."
    Remove-Item -Recurse -Force $buildDir
}

Write-Host "Creating build directory..."
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

# Run CMake
Write-Host "`nConfiguring with CMake..."
Push-Location $buildDir

try {
    # Configure
    cmake .. -G "Visual Studio 17 2022" -A x64
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ CMake configuration failed!"
        exit 1
    }
    
    Write-Host "✅ CMake configuration successful"
    
    # Build
    Write-Host "`nBuilding..."
    cmake --build . --config Release
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ Build failed!"
        exit 1
    }
    
    Write-Host "✅ Build successful!"
    
    # Check for executable
    $exePath = "bin\Release\test_gik_20constraints.exe"
    if (Test-Path $exePath) {
        Write-Host "`n✅ Executable created: $exePath"
        Write-Host "`nTo run tests:"
        Write-Host "  cd test_cpp\build"
        Write-Host "  .\bin\Release\test_gik_20constraints.exe"
    } else {
        Write-Host "⚠️  Warning: Executable not found at expected location"
    }
    
} finally {
    Pop-Location
}

Write-Host "`n========================================"
Write-Host "Build process completed!"
Write-Host "========================================"
