#!/bin/bash
# Build script for standalone GIK validation program (WSL/Linux)

set -e

echo "=========================================="
echo "Building GIK Standalone Validation"
echo "=========================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CODEGEN_DIR="${SCRIPT_DIR}/../codegen/x86_64_validation_noCollision"  # New codegen with MaxIterations=1000
SRC_FILE="${SCRIPT_DIR}/validate_gik_standalone.cpp"
OUT_FILE="${SCRIPT_DIR}/validate_gik_standalone"

# Check if codegen directory exists
if [ ! -d "$CODEGEN_DIR" ]; then
    echo "❌ Error: Codegen directory not found: $CODEGEN_DIR"
    echo "   Please run generate_code_x86_64_noCollision.m in MATLAB first"
    exit 1
fi

# Check if header file exists
HEADER_FILE="${CODEGEN_DIR}/GIKSolver.h"  # New class-based interface
if [ ! -f "$HEADER_FILE" ]; then
    echo "❌ Error: GIK header file not found: $HEADER_FILE"
    exit 1
fi

echo "Configuration:"
echo "  Source: $SRC_FILE"
echo "  Codegen: $CODEGEN_DIR"
echo "  Output: $OUT_FILE"
echo ""

# MATLAB include directory for WSL
MATLAB_INCLUDE="/home/yanbo/MATLAB/R2024a/extern/include"

# Compiler flags
CXX="g++"
CXXFLAGS="-std=c++17 -O2 -Wall"
INCLUDES="-I${CODEGEN_DIR} -I${MATLAB_INCLUDE}"
LIBS="-lm -lstdc++ -lpthread -lgomp -lccd -lrt"

# Find all .cpp and .c files in codegen directory (exclude MEX interface, HTML, examples, AND collision files)
# Exclude collision .cpp files and rigidBodyTree files that have collision dependencies
echo "Finding codegen source files..."
CODEGEN_SOURCES_CPP=$(find "$CODEGEN_DIR" -name "*.cpp" -type f ! -path "*/interface/*" ! -path "*/html/*" ! -path "*/examples/*" ! -name "CollisionSet.cpp" ! -name "RigidBody.cpp" ! -name "generalizedInverseKinematics.cpp" ! -name "rigidBodyTree1.cpp" ! -name "rigidBody1.cpp" ! -name "RigidBodyTree.cpp")
CODEGEN_SOURCES_C=$(find "$CODEGEN_DIR" -name "*.c" -type f ! -path "*/interface/*" ! -path "*/html/*" ! -path "*/examples/*")
# DO NOT include .obj files - they are Windows binaries and won't link on Linux!
CODEGEN_SOURCES="$CODEGEN_SOURCES_CPP $CODEGEN_SOURCES_C"
NUM_SOURCES=$(echo "$CODEGEN_SOURCES" | wc -w)
echo "  Found $NUM_SOURCES source files (excluding MEX/HTML/examples/collision-dependent)"

# Build command
echo ""
echo "Compiling..."
CMD="$CXX $CXXFLAGS $INCLUDES -o $OUT_FILE $SRC_FILE $CODEGEN_SOURCES $LIBS"

echo "  $CXX $CXXFLAGS ..."
echo ""

# Execute build
if $CMD; then
    echo ""
    echo "✅ Build successful!"
    echo "   Output: $OUT_FILE"
    
    # Show file size
    FILE_SIZE=$(du -h "$OUT_FILE" | cut -f1)
    echo "   Size: $FILE_SIZE"
    
    echo ""
    echo "Usage:"
    echo "  $OUT_FILE <test_cases.json> [output_results.json]"
    echo ""
    echo "Example:"
    echo "  $OUT_FILE gik_test_cases.json gik_validation_results.json"
    
    exit 0
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
