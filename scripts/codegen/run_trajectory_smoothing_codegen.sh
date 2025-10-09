#!/bin/bash
# Run MATLAB Coder for Trajectory Smoothing
# File: run_trajectory_smoothing_codegen.sh
# Purpose: Generate C++ code from MATLAB function for ARM Cortex-A deployment

set -e  # Exit on error

PROJECT_ROOT="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"
MATLAB_BIN="/home/yanbo/MATLAB/R2024a/bin/matlab"
CODEGEN_SCRIPT="scripts/codegen/generate_code_trajectory_smoothing.m"

echo "═══════════════════════════════════════════════════════════════════"
echo "  MATLAB Coder: Trajectory Smoothing C++ Code Generation"
echo "═══════════════════════════════════════════════════════════════════"
echo ""
echo "Project root: $PROJECT_ROOT"
echo "MATLAB:       $MATLAB_BIN"
echo "Script:       $CODEGEN_SCRIPT"
echo ""

# Check if script exists
if [ ! -f "$PROJECT_ROOT/$CODEGEN_SCRIPT" ]; then
    echo "❌ ERROR: Codegen script not found!"
    echo "   Expected: $PROJECT_ROOT/$CODEGEN_SCRIPT"
    exit 1
fi

# Change to project root
cd "$PROJECT_ROOT"

echo "━━━ Running MATLAB Coder ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Run MATLAB in batch mode
"$MATLAB_BIN" -batch "run('$CODEGEN_SCRIPT')"

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "━━━ Code Generation Complete ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "✅ SUCCESS! Generated files:"
    ls -lh codegen/trajectory_smoothing/*.cpp codegen/trajectory_smoothing/*.h 2>/dev/null || echo "   (files pending...)"
    echo ""
    echo "Next steps:"
    echo "  1. Check generated code: codegen/trajectory_smoothing/"
    echo "  2. Review HTML report: codegen/trajectory_smoothing/html/report.mldatx"
    echo "  3. Copy to ROS2 package (see PHASE_2_IMPLEMENTATION_PLAN.md)"
    echo ""
    exit 0
else
    echo "━━━ Code Generation Failed ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "❌ FAILED with exit code: $EXIT_CODE"
    echo ""
    echo "Check output above for error messages."
    echo ""
    exit $EXIT_CODE
fi
