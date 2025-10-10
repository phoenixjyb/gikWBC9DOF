#!/bin/bash
# Check Build Currency (WSL Version)
# Verifies if codegen output is up-to-date with source

set -e

WORKSPACE="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"
BUILD_DIR="$WORKSPACE/codegen/arm64_realtime"
SOURCE_DIR="$WORKSPACE/matlab"

cd "$WORKSPACE"

echo "=== Build Currency Check (WSL) ==="
echo ""

# Check if build exists
if [ ! -f "$BUILD_DIR/GIKSolver.cpp" ]; then
    echo "❌ No build found in codegen/arm64_realtime/"
    echo "Run codegen first!"
    exit 1
fi

# Check if we have build version info
if [ ! -f "$BUILD_DIR/SOURCE_COMMIT.txt" ]; then
    echo "⚠️  No build version info found"
    echo "Build was created without version tracking"
    echo ""
    echo "Falling back to timestamp comparison..."
    
    # Get timestamps
    BUILD_TIME=$(stat -c %Y "$BUILD_DIR/GIKSolver.cpp")
    LATEST_SOURCE=$(find "$SOURCE_DIR" -name "*.m" -type f -printf '%T@\n' | sort -n | tail -1)
    
    if (( $(echo "$BUILD_TIME > $LATEST_SOURCE" | bc -l) )); then
        echo "✓ Build appears current (by timestamp)"
        exit 0
    else
        echo "⚠️  Build appears outdated (by timestamp)"
        echo "Consider rebuilding"
        exit 1
    fi
fi

# Read build commit
BUILD_COMMIT=$(cat "$BUILD_DIR/SOURCE_COMMIT_SHORT.txt" 2>/dev/null || cat "$BUILD_DIR/SOURCE_COMMIT.txt" | head -c 7)
CURRENT_COMMIT=$(git rev-parse --short HEAD)

BUILD_TIME=$(cat "$BUILD_DIR/BUILD_TIME.txt" 2>/dev/null || echo "unknown")

echo "Build Information:"
echo "  Build from: $BUILD_COMMIT"
echo "  Build time: $BUILD_TIME"
echo ""
echo "Current State:"
echo "  Current:    $CURRENT_COMMIT"
echo ""

# Compare commits
if [ "$BUILD_COMMIT" = "$CURRENT_COMMIT" ]; then
    echo "✓ BUILD IS CURRENT" 
    echo "  Build matches current source"
    exit 0
else
    echo "⚠️  BUILD IS OUTDATED"
    echo "  Source has changed since build"
    echo ""
    echo "Changes since build:"
    git log --oneline "$BUILD_COMMIT..HEAD" -- matlab/ | head -10
    echo ""
    echo "Modified MATLAB files:"
    git diff --name-only "$BUILD_COMMIT..HEAD" -- matlab/*.m matlab/+*/*.m | head -10
    echo ""
    echo "ACTION: Rebuild recommended"
    exit 1
fi
