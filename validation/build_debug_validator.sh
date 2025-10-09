#!/bin/bash
# Build script for GIK debug validator (WSL/Linux)

set -e

echo "=========================================="
echo "Building GIK Debug Validator"
echo "=========================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CODEGEN_DIR="${SCRIPT_DIR}/../codegen/gik9dof_arm64_20constraints"
SRC_FILE="${SCRIPT_DIR}/validate_gik_debug.cpp"
OUT_FILE="${SCRIPT_DIR}/validate_gik_debug"

# Check if codegen directory exists
if [ ! -d "$CODEGEN_DIR" ]; then
    echo "❌ Error: Codegen directory not found: $CODEGEN_DIR"
    echo "   Please generate C++ code first using MATLAB Coder"
    exit 1
fi

# Check if header file exists
HEADER_FILE="${CODEGEN_DIR}/gik9dof_codegen_inuse_solveGIKStepWrapper.h"
if [ ! -f "$HEADER_FILE" ]; then
    echo "❌ Error: GIK header file not found: $HEADER_FILE"
    exit 1
fi

echo "Finding source files..."

# Find all .cpp files (exclude interface/, html/, examples/)
CPP_FILES=$(find "$CODEGEN_DIR" -name "*.cpp" \
    -not -path "*/interface/*" \
    -not -path "*/html/*" \
    -not -path "*/examples/*" \
    2>/dev/null)

# Find all .c files (for CCD collision detection library)
C_FILES=$(find "$CODEGEN_DIR" -name "*.c" \
    -not -path "*/interface/*" \
    -not -path "*/html/*" \
    -not -path "*/examples/*" \
    2>/dev/null)

CPP_COUNT=$(echo "$CPP_FILES" | wc -l)
C_COUNT=$(echo "$C_FILES" | wc -l)

echo "  Found $CPP_COUNT .cpp files"
echo "  Found $C_COUNT .c files"
echo ""

# Compile
echo "Compiling debug validator..."
echo "  Source: $SRC_FILE"
echo "  Output: $OUT_FILE"
echo ""

g++ -o "$OUT_FILE" \
    "$SRC_FILE" \
    $CPP_FILES \
    $C_FILES \
    -I"$CODEGEN_DIR" \
    -std=c++14 \
    -O2 \
    -Wall \
    -Wno-unused-variable \
    -Wno-unused-but-set-variable \
    -lm \
    -lstdc++ \
    -lpthread \
    -lgomp \
    -lccd \
    -lrt

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    ls -lh "$OUT_FILE"
    echo ""
    echo "Run with:"
    echo "  $OUT_FILE <test_cases.json> <output.json> [--reset-per-test] [--sequential] [--verbose]"
    echo ""
    echo "Options:"
    echo "  --reset-per-test  : Delete and recreate solver between each test"
    echo "  --sequential      : Use previous solution as initial guess for next test"
    echo "  --verbose         : Print detailed per-test information"
else
    echo "❌ Build failed!"
    exit 1
fi
