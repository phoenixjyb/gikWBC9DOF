#!/bin/bash
# Generate C++ code from MATLAB smoothVelocityCommand function
# Usage: ./run_velocity_smoothing_codegen.sh

cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
/home/yanbo/MATLAB/R2024a/bin/matlab -batch "run('scripts/codegen/generate_code_velocity_smoothing.m')"
