#!/bin/bash
# Simple WSL Code Generation Wrapper

MATLAB_PATH="/home/yanbo/MATLAB/R2024a/bin/matlab"
PROJECT_DIR="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"

echo "=========================================="
echo "WSL Linux Code Generation"
echo "=========================================="
echo "MATLAB: $MATLAB_PATH"
echo "Project: $PROJECT_DIR"
echo ""

cd "$PROJECT_DIR"

echo "Running MATLAB code generation script..."
echo "This will take 5-10 minutes..."
echo ""

$MATLAB_PATH -batch "run_wsl_codegen_matlab"

exitcode=$?
echo ""
if [ $exitcode -eq 0 ]; then
    echo "✓ Code generation completed successfully!"
else
    echo "❌ Code generation failed with exit code: $exitcode"
fi

exit $exitcode
