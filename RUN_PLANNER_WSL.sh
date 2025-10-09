#!/bin/bash
# WSL Planner ARM64 Code Generation
# Copy-paste this into your WSL terminal

cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF

/home/yanbo/MATLAB/R2024a/bin/matlab -batch "cd('/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF'); run_planner_codegen_wsl"

# Verify results
echo ""
echo "Verifying generated code..."
ls -lh codegen/planner_arm64/*.cpp 2>/dev/null | wc -l
file codegen/planner_arm64/*.o 2>/dev/null | head -3
