#!/bin/bash
# WSL Linux Code Generation Script
# Generates x86_64 Linux binaries using MATLAB in WSL

set -e  # Exit on error

MATLAB_PATH="/home/yanbo/MATLAB/R2024a/bin/matlab"
PROJECT_DIR="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"

echo "=========================================="
echo "WSL Linux Code Generation"
echo "=========================================="
echo "MATLAB: $MATLAB_PATH"
echo "Project: $PROJECT_DIR"
echo ""

# Navigate to project
cd "$PROJECT_DIR"

# Run code generation
echo "Starting code generation (this will take 5-10 minutes)..."
echo ""

$MATLAB_PATH -batch "fprintf('Starting Linux x86_64 code generation with MaxIterations=1000...\n'); generate_code_x86_64_noCollision"

echo ""
echo "=========================================="
echo "Code Generation Complete!"
echo "=========================================="
echo "Output: codegen/x86_64_validation_noCollision/"
echo ""
echo "Next: Run build_validation_wsl.sh to build the validator"
