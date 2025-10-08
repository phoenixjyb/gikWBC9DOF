#!/bin/bash
# Build script for GIK 20-Constraint C++ Test in WSL
# Uses ARM64 generated code (Linux-compatible)

set -e  # Exit on error

echo "========================================"
echo "Building GIK 20-Constraint C++ Test (WSL)"
echo "========================================"

# Convert Windows path to WSL path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEGEN_DIR="${SCRIPT_DIR}/../codegen/gik9dof_arm64_20constraints"

# Check for ARM64 codegen directory
if [ ! -d "$CODEGEN_DIR" ]; then
    echo "❌ Error: ARM64 codegen directory not found: $CODEGEN_DIR"
    echo "Please run generate_gik_20constraints_arm64.m first!"
    exit 1
fi

echo "✅ Found ARM64 codegen directory"

# Create build directory
BUILD_DIR="${SCRIPT_DIR}/build_wsl"
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning existing build directory..."
    rm -rf "$BUILD_DIR"
fi

echo "Creating build directory..."
mkdir -p "$BUILD_DIR"

# Run CMake
echo ""
echo "Configuring with CMake..."
cd "$BUILD_DIR"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_ARM64_CODE=ON

if [ $? -ne 0 ]; then
    echo "❌ CMake configuration failed!"
    exit 1
fi

echo "✅ CMake configuration successful"

# Build
echo ""
echo "Building..."
cmake --build . -- -j$(nproc)

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"

# Check for executable
EXEC_PATH="bin/test_gik_20constraints"
if [ -f "$EXEC_PATH" ]; then
    echo ""
    echo "✅ Executable created: $EXEC_PATH"
    echo ""
    echo "To run tests:"
    echo "  cd test_cpp/build_wsl"
    echo "  ./bin/test_gik_20constraints"
else
    echo "⚠️  Warning: Executable not found at expected location"
fi

echo ""
echo "========================================"
echo "Build process completed!"
echo "========================================"
