# Quick Orin Workspace Cleanup
# Executes cleanup script remotely on Jetson AGX Orin

param(
    [Parameter(Mandatory=$false)]
    [string]$OrinIP = "192.168.100.150",
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "cr"
)

$scriptContent = @'
#!/bin/bash
cd /home/nvidia/temp_gikrepo/ros2
echo "Cleaning workspace..."
rm -rf build/ install/ log/
find . -maxdepth 1 -type d -name "*_bak" -exec rm -rf {} + 2>/dev/null || true
find . -type d -name "*_backup_*" -exec rm -rf {} + 2>/dev/null || true
echo "✓ Cleanup complete!"
echo ""
echo "ROS2 packages:"
ls -1d */
'@

Write-Host "Cleaning Orin workspace: $Username@$OrinIP" -ForegroundColor Cyan
Write-Host ""

ssh "$Username@$OrinIP" $scriptContent

Write-Host ""
Write-Host "✓ Cleanup done! Now build:" -ForegroundColor Green
Write-Host "  ssh $Username@$OrinIP" -ForegroundColor Gray
Write-Host "  cd /home/nvidia/temp_gikrepo/ros2" -ForegroundColor Gray
Write-Host "  source /opt/ros/humble/setup.bash" -ForegroundColor Gray
Write-Host "  colcon build --packages-select gik9dof_msgs" -ForegroundColor Gray
Write-Host "  source install/setup.bash" -ForegroundColor Gray
Write-Host "  colcon build --packages-select gik9dof_solver" -ForegroundColor Gray
