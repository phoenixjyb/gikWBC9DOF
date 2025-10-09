#!/bin/bash
# Orin ROS2 Workspace Cleanup Script
# Run this on the Jetson AGX Orin to fix build issues

set -e  # Exit on error

WORKSPACE="/home/nvidia/temp_gikrepo/ros2"

echo "================================================"
echo "   Orin ROS2 Workspace Cleanup"
echo "================================================"
echo ""

cd "$WORKSPACE"

echo "[1/4] Removing old build artifacts..."
rm -rf build/ install/ log/
echo "✓ Cleaned build/, install/, log/"
echo ""

echo "[2/4] Removing backup folders..."
# Remove backup packages at root level
find . -maxdepth 1 -type d -name "*_bak" -exec rm -rf {} + 2>/dev/null || true
find . -maxdepth 1 -type d -name "*_backup_*" -exec rm -rf {} + 2>/dev/null || true

# Remove backup folders in generated code
find . -type d -name "*_backup_*" -exec rm -rf {} + 2>/dev/null || true
echo "✓ Removed backup folders"
echo ""

echo "[3/4] Listing ROS2 packages..."
ls -1d */
echo ""

echo "[4/4] Ready to build!"
echo ""
echo "Run these commands:"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select gik9dof_msgs"
echo "  source install/setup.bash"
echo "  colcon build --packages-select gik9dof_solver"
echo ""
echo "================================================"
echo "   ✓ Cleanup Complete!"
echo "================================================"
