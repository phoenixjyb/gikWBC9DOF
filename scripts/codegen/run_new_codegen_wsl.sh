#!/bin/bash
#
# run_new_codegen_wsl.sh
# Run code generation for new MATLAB features from origin/main merge
#
# Components generated:
#   1. Chassis Controller (simulateChassisController)
#   2. RS Clothoid Smoothing (rsClothoidRefine)
#
# Usage: wsl bash scripts/codegen/run_new_codegen_wsl.sh
#

set -e  # Exit on error

echo "========================================"
echo "New Feature Code Generation (WSL)"
echo "========================================"
echo ""

# Get workspace root (script is in scripts/codegen/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# MATLAB paths
MATLAB_BIN="/home/yanbo/MATLAB/R2024a/bin/matlab"
MATLAB_ROOT="$WORKSPACE_ROOT/matlab"

if [ ! -f "$MATLAB_BIN" ]; then
    echo "❌ MATLAB not found: $MATLAB_BIN"
    exit 1
fi

echo "MATLAB: $MATLAB_BIN"
echo "Workspace: $WORKSPACE_ROOT"
echo ""

# Component 1: Chassis Controller
echo "========================================"
echo "[1/2] Chassis Controller"
echo "========================================"
echo "Source: gik9dof.control.simulateChassisController"
echo "Output: codegen/chassis_controller_arm64/"
echo "Purpose: Multi-mode chassis control"
echo ""

cd "$WORKSPACE_ROOT"

echo "Running MATLAB code generation..."
"$MATLAB_BIN" -batch "addpath(genpath('$MATLAB_ROOT')); run('scripts/codegen/generate_code_chassis_controller.m')"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Chassis controller code generation complete"
    echo ""
else
    echo ""
    echo "❌ Chassis controller code generation failed"
    exit 1
fi

# Component 2: RS Clothoid Smoothing
echo "========================================"
echo "[2/2] RS Clothoid Smoothing"
echo "========================================"
echo "Source: gik9dof.control.rsClothoidRefine"
echo "Output: codegen/rs_smoothing_arm64/"
echo "Purpose: Clothoid-based path smoothing"
echo ""

echo "Running MATLAB code generation..."
"$MATLAB_BIN" -batch "addpath(genpath('$MATLAB_ROOT')); run('scripts/codegen/generate_code_rs_smoothing.m')"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ RS smoothing code generation complete"
    echo ""
else
    echo ""
    echo "❌ RS smoothing code generation failed"
    exit 1
fi

# Summary
echo "========================================"
echo "Summary"
echo "========================================"
echo "✓ Chassis Controller: codegen/chassis_controller_arm64/"
echo "✓ RS Smoothing: codegen/rs_smoothing_arm64/"
echo ""
echo "🎉 All new feature codegen complete!"
echo ""
echo "Total components: 2"
echo "Total: 4 existing + 2 new = 6 codegen components"
