#!/bin/bash
# WSL Linux Code Generation Script for MaxIterations=1000
# This script runs MATLAB code generation IN WSL to produce Linux-native binaries

set -e

echo "=========================================="
echo "WSL Linux Code Generation Setup"
echo "=========================================="
echo ""

# Find MATLAB installation in WSL
echo "Looking for MATLAB installation..."

# Common MATLAB installation paths in Linux/WSL
MATLAB_PATHS=(
    "/usr/local/MATLAB/R2024b/bin/matlab"
    "/usr/local/MATLAB/R2024a/bin/matlab"
    "/opt/MATLAB/R2024b/bin/matlab"
    "/opt/MATLAB/R2024a/bin/matlab"
    "$HOME/MATLAB/R2024b/bin/matlab"
    "$(which matlab 2>/dev/null)"
)

MATLAB_CMD=""
for path in "${MATLAB_PATHS[@]}"; do
    if [ -x "$path" ]; then
        MATLAB_CMD="$path"
        echo "✓ Found MATLAB: $MATLAB_CMD"
        break
    fi
done

if [ -z "$MATLAB_CMD" ]; then
    echo "❌ MATLAB not found in standard locations."
    echo ""
    echo "Please specify MATLAB path:"
    echo "  export MATLAB_PATH=/path/to/matlab/bin/matlab"
    echo "  ./generate_code_wsl_linux.sh"
    echo ""
    echo "Or add MATLAB to PATH:"
    echo "  export PATH=\$PATH:/usr/local/MATLAB/R2024b/bin"
    echo ""
    exit 1
fi

# Check if MATLAB_PATH environment variable is set
if [ -n "$MATLAB_PATH" ]; then
    MATLAB_CMD="$MATLAB_PATH"
    echo "Using MATLAB from environment: $MATLAB_CMD"
fi

# Verify MATLAB works
echo "Testing MATLAB..."
$MATLAB_CMD -batch "disp('MATLAB OK'); disp(version);" || {
    echo "❌ MATLAB test failed"
    exit 1
}

echo ""
echo "=========================================="
echo "Starting Code Generation"
echo "=========================================="
echo ""

# Navigate to project directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

echo "Project directory: $PROJECT_ROOT"
echo "MATLAB command: $MATLAB_CMD"
echo ""

# Run code generation script
echo "Generating x86_64 Linux code with MaxIterations=1000..."
echo "This will take 5-10 minutes..."
echo ""

$MATLAB_CMD -batch "generate_code_x86_64_noCollision" || {
    echo ""
    echo "❌ Code generation failed!"
    echo "Check the error messages above."
    exit 1
}

echo ""
echo "=========================================="
echo "✓ Code Generation Complete!"
echo "=========================================="
echo ""
echo "Generated code location:"
echo "  $PROJECT_ROOT/codegen/x86_64_validation_noCollision/"
echo ""
echo "This code has Linux-native binaries (.o files, not Windows .obj)"
echo ""
echo "Next steps:"
echo "  cd validation"
echo "  bash build_validation_wsl.sh"
echo "  ./validate_gik_standalone gik_test_cases_20.json results.json"
echo ""
