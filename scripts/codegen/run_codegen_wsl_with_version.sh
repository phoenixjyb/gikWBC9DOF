#!/bin/bash
# WSL MATLAB Codegen Wrapper with Version Tracking
# Wraps your MATLAB codegen process to add build versioning

set -e

WORKSPACE="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"
MATLAB_BIN="/home/yanbo/MATLAB/R2024a/bin/matlab"
CODEGEN_SCRIPT="${1:-generate_code_arm64.m}"

cd "$WORKSPACE"

echo "========================================"
echo "WSL MATLAB Codegen with Version Tracking"
echo "========================================"
echo ""
echo "Script: $CODEGEN_SCRIPT"
echo "MATLAB: $MATLAB_BIN"
echo ""

# Step 1: Save build info
echo "[1/3] Collecting build information..."
bash scripts/codegen/save_build_info_wsl.sh

BUILD_ID=$(cat codegen/arm64_realtime/BUILD_ID.txt)
GIT_SHORT=$(cat codegen/arm64_realtime/SOURCE_COMMIT_SHORT.txt)

echo ""
echo "[2/3] Running MATLAB codegen..."
echo "  This may take 5-15 minutes..."
echo ""

# Change to workspace directory first
cd "$WORKSPACE"

# Run MATLAB codegen
"$MATLAB_BIN" -batch "addpath(genpath('matlab')); cd('scripts/codegen'); run('$CODEGEN_SCRIPT');"

MATLAB_EXIT=$?

if [ $MATLAB_EXIT -ne 0 ]; then
    echo ""
    echo "❌ MATLAB codegen failed (exit code: $MATLAB_EXIT)"
    exit $MATLAB_EXIT
fi

echo ""
echo "[3/3] Verifying generated code..."

# Check if files were generated
if [ ! -f "codegen/arm64_realtime/GIKSolver.cpp" ]; then
    echo "❌ GIKSolver.cpp not found - codegen may have failed"
    exit 1
fi

# Count generated files
CPP_COUNT=$(find codegen/arm64_realtime -name "*.cpp" | wc -l)
H_COUNT=$(find codegen/arm64_realtime -name "*.h" | wc -l)
O_COUNT=$(find codegen/arm64_realtime -name "*.o" | wc -l)

echo "✓ Generated files:"
echo "  .cpp: $CPP_COUNT"
echo "  .h:   $H_COUNT"
echo "  .o:   $O_COUNT"

# Verify binary format (should be ARM64)
echo ""
echo "Binary format check:"
file codegen/arm64_realtime/*.o 2>/dev/null | head -3

echo ""
echo "========================================"
echo "✓ Codegen Complete!"
echo "========================================"
echo ""
echo "Build ID:    $BUILD_ID"
echo "Git Commit:  $GIT_SHORT"
echo "Output:      codegen/arm64_realtime/"
echo ""
echo "Build info:"
echo "  - BUILD_INFO.txt (human-readable)"
echo "  - build_info.h (C++ header)"
echo "  - SOURCE_COMMIT.txt (git tracking)"
echo ""
echo "Next: Deploy to target or copy to ROS2"
