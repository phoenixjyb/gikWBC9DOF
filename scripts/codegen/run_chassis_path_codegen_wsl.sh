#!/bin/bash
# Run chassis path follower codegen in WSL

cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF

echo "=========================================="
echo "Chassis Path Follower - ARM64 Codegen"
echo "=========================================="

/home/yanbo/MATLAB/R2024a/bin/matlab -batch "addpath(genpath('matlab')); run('scripts/codegen/generate_code_chassis_path_follower.m')"

exit $?
