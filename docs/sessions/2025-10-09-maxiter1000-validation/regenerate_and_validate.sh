#!/bin/bash
# Complete Regeneration and Validation Workflow
# Run this in WSL terminal

set -e

echo "=========================================="
echo "GIK Code Regeneration with MaxTime=10s"
echo "=========================================="
echo ""

PROJECT_DIR="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"
MATLAB_BIN="/home/yanbo/MATLAB/R2024a/bin/matlab"

cd "$PROJECT_DIR"

echo "Step 1: Regenerating C++ code..."
echo "  MaxIterations: 1000"
echo "  MaxTime: 10.0s (was 0.05s)"
echo "  This will take 5-10 minutes..."
echo ""

$MATLAB_BIN -batch "cd('$PROJECT_DIR'); run_wsl_codegen_matlab"

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Code generation failed!"
    exit 1
fi

echo ""
echo "=========================================="
echo "Step 2: Rebuilding validator..."
echo "=========================================="
echo ""

cd "$PROJECT_DIR/validation"
bash build_with_library_wsl.sh

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Build failed!"
    exit 1
fi

echo ""
echo "=========================================="
echo "Step 3: Running validation tests..."
echo "=========================================="
echo ""

./validate_gik_standalone gik_test_cases_20.json results_maxtime10s.json

echo ""
echo "=========================================="
echo "✅ Complete!"
echo "=========================================="
echo ""
echo "Results saved to: results_maxtime10s.json"
echo ""
echo "Compare with previous results:"
echo "  Old (MaxTime=50ms): results_maxiter1000.json - 30% pass (6/20)"
echo "  New (MaxTime=10s):  results_maxtime10s.json - Expected 60-80% pass"
echo ""
