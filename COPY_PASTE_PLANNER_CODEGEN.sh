#!/bin/bash
# Direct WSL script for planner ARM64 codegen
# Copy and paste this entire script into WSL terminal

set -e

echo "========================================"
echo "Planner ARM64 Codegen - Linux MATLAB"
echo "========================================"
echo "Date: $(date)"
echo ""

# Navigate to workspace
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF

# Run MATLAB codegen
echo "Running MATLAB codegen..."
echo "This will take 5-15 minutes..."
echo ""

/home/yanbo/MATLAB/R2024a/bin/matlab -nodisplay -nosplash -nodesktop -r "addpath(genpath('matlab')); run('generate_code_planner_arm64.m'); exit;"

echo ""
echo "========================================"
echo "Checking results..."
echo "========================================"

if [ -f "codegen/planner_arm64/HybridAStarPlanner.cpp" ]; then
    echo "✅ Codegen successful!"
    echo ""
    echo "Generated files:"
    ls -1 codegen/planner_arm64/*.cpp | wc -l | xargs echo "  .cpp files:"
    ls -1 codegen/planner_arm64/*.h | wc -l | xargs echo "  .h files:"
    ls -1 codegen/planner_arm64/*.o 2>/dev/null | wc -l | xargs echo "  .o files:"
    echo ""
    echo "Binary format:"
    file codegen/planner_arm64/*.o 2>/dev/null | head -3
    echo ""
    echo "✅ Phase 4 Complete!"
else
    echo "❌ Error: Codegen failed"
    echo "HybridAStarPlanner.cpp not found"
    exit 1
fi
