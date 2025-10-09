#!/bin/bash
# Run planner ARM64 codegen with Linux MATLAB in WSL
# Phase 4 of PLANNER_REGEN_PLAN.md

set -e  # Exit on error

echo "========================================"
echo "Planner ARM64 Codegen - Linux MATLAB"
echo "========================================"
echo "Date: $(date)"
echo "MATLAB: /home/yanbo/MATLAB/R2024a"
echo ""

# Convert to WSL path
WORKSPACE="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"

cd "$WORKSPACE"

echo "Running MATLAB codegen..."
echo "This will take 5-15 minutes."
echo ""

/home/yanbo/MATLAB/R2024a/bin/matlab -batch "
    addpath(genpath('matlab'));
    run('generate_code_planner_arm64.m');
"

echo ""
echo "========================================"
echo "Checking generated files..."
echo "========================================"

if [ -d "codegen/planner_arm64" ]; then
    CPP_COUNT=$(ls -1 codegen/planner_arm64/*.cpp 2>/dev/null | wc -l)
    H_COUNT=$(ls -1 codegen/planner_arm64/*.h 2>/dev/null | wc -l)
    O_COUNT=$(ls -1 codegen/planner_arm64/*.o 2>/dev/null | wc -l)
    
    echo "✅ Generated files:"
    echo "   .cpp files: $CPP_COUNT"
    echo "   .h files:   $H_COUNT"
    echo "   .o files:   $O_COUNT"
    echo ""
    
    echo "Binary format check:"
    file codegen/planner_arm64/*.o 2>/dev/null | head -3
    
    echo ""
    echo "✅ Phase 4 Complete: Planner generated with Linux MATLAB"
else
    echo "❌ Error: codegen/planner_arm64/ not found"
    exit 1
fi
