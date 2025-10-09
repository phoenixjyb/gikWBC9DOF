#!/bin/bash
# Build script using pre-compiled library (WSL/Linux)

set -e

echo "=========================================="
echo "Building GIK Standalone Validation (using .a library)"
echo "=========================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CODEGEN_DIR="${SCRIPT_DIR}/../codegen/x86_64_validation_noCollision"
SRC_FILE="${SCRIPT_DIR}/validate_gik_standalone.cpp"
OUT_FILE="${SCRIPT_DIR}/validate_gik_standalone"

# MATLAB include directory for WSL
MATLAB_INCLUDE="/home/yanbo/MATLAB/R2024a/extern/include"

# Check if library exists
LIB_FILE="${CODEGEN_DIR}/solveGIKStepWrapper.a"
if [ ! -f "$LIB_FILE" ]; then
    echo "❌ Error: Pre-compiled library not found: $LIB_FILE"
    exit 1
fi

echo "Configuration:"
echo "  Source: $SRC_FILE"
echo "  Library: $LIB_FILE"
echo "  MATLAB headers: $MATLAB_INCLUDE"
echo "  Output: $OUT_FILE"
echo ""

# Compiler flags
CXX="g++"
CXXFLAGS="-std=c++17 -O2 -Wall"
INCLUDES="-I${CODEGEN_DIR} -I${MATLAB_INCLUDE}"
LIBS="-lm -lstdc++ -lpthread -lgomp -lccd -lrt"

# Build command - link against pre-compiled library
echo "Compiling and linking..."
CMD="$CXX $CXXFLAGS $INCLUDES -o $OUT_FILE $SRC_FILE $LIB_FILE $LIBS"

echo "  $CXX $CXXFLAGS -o $OUT_FILE ..."
echo ""

# Execute build
if $CMD; then
    echo ""
    echo "=========================================="
    echo "✅ Build successful!"
    echo "=========================================="
    echo "Executable: $OUT_FILE"
    echo ""
    echo "Run with:"
    echo "  cd validation"
    echo "  ./validate_gik_standalone gik_test_cases_20.json results.json"
    echo ""
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
