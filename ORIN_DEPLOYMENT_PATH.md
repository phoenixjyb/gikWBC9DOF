# Orin Deployment Configuration

**Target Path on Orin:** `/home/nvidia/camo_9dof/gikWBC9DOF`  
**Username:** `nvidia`  
**Last Updated:** October 7, 2025

---

## Deployment Script Usage

The `deploy_to_orin_complete.ps1` script has been updated to use the new deployment path.

### Quick Deploy

```powershell
# From Windows PowerShell in project root
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x"
```

**Default settings:**
- Username: `nvidia`
- Remote path: `/home/nvidia/camo_9dof/gikWBC9DOF`

### Custom Settings

```powershell
# Override username
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x" -Username "custom_user"

# Override remote path
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x" -RemotePath "/custom/path"

# Skip mesh files (faster)
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x" -SkipMeshes
```

---

## Orin Workspace Structure

After deployment, the Orin workspace will look like:

```
/home/nvidia/camo_9dof/gikWBC9DOF/
├── ros2/
│   ├── gik9dof_msgs/
│   ├── gik9dof_solver/
│   │   ├── include/
│   │   │   └── velocity_controller/     # NEW: Velocity controller headers
│   │   ├── src/
│   │   │   ├── velocity_controller/     # NEW: Velocity controller sources
│   │   │   └── gik9dof_solver_node.cpp
│   │   ├── matlab_codegen/              # IK solver generated code
│   │   ├── config/
│   │   │   └── gik9dof_solver_params.yaml  # NEW: Configuration file
│   │   └── CMakeLists.txt
│   ├── build/                           # Created by colcon build
│   ├── install/                         # Created by colcon build
│   └── log/                             # Created by colcon build
├── meshes/
│   └── *.STL
└── mobile_manipulator_PPR_base_corrected.urdf
```

---

## Building on Orin

```bash
# SSH to Orin
ssh nvidia@<orin-ip>

# Navigate to ROS2 workspace
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs

# Source the workspace
source install/setup.bash

# Build solver (with velocity controller integration)
colcon build --packages-select gik9dof_solver

# Source again
source install/setup.bash
```

**Expected build time:**
- Messages: ~30 seconds
- Solver: ~2-3 minutes (ARM64 compilation)

---

## Running the Node

### With Configuration File (Recommended)

```bash
# Make sure you're in the ROS2 workspace
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2

# Source the workspace
source install/setup.bash

# Run with config
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

### With Default Parameters

```bash
ros2 run gik9dof_solver gik9dof_solver_node
```

**Default behavior:**
- Uses new holistic velocity controller
- Control rate: 10 Hz
- Publishes to `/cmd_vel` and `/motion_target/target_joint_state_arm_left`

---

## Testing Velocity Controller

### Check Current Mode

```bash
ros2 param get /gik9dof_solver_node use_velocity_controller
# Expected: true (new controller)
```

### Switch to Legacy Mode

```bash
ros2 param set /gik9dof_solver_node use_velocity_controller false
# Note: Currently requires node restart
```

### Monitor Output

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check topics
ros2 topic list

# Monitor diagnostics
ros2 topic echo /gik9dof/solver_diagnostics
```

---

## Path Reference

All documentation has been updated to reference:

**Orin workspace:** `/home/nvidia/camo_9dof/gikWBC9DOF`

Updated files:
- ✅ `deploy_to_orin_complete.ps1` - Deployment script
- ✅ `QUICKSTART_TESTING.md` - Quick start guide
- ✅ This file - Deployment configuration

---

## Troubleshooting

### Issue: Permission denied when creating directories

```bash
# Ensure the parent directory exists and is writable
ssh nvidia@<orin-ip>
mkdir -p /home/nvidia/camo_9dof
chmod 755 /home/nvidia/camo_9dof
```

### Issue: Deployment script can't connect

**Check:**
1. Orin IP address is correct
2. SSH is enabled on Orin
3. Username is `nvidia` (or override with `-Username`)

**Test connection manually:**
```powershell
ssh nvidia@<orin-ip> "echo 'Connection OK'"
```

### Issue: Files deployed to wrong location

**Verify deployment path:**
```bash
ssh nvidia@<orin-ip>
ls -la /home/nvidia/camo_9dof/gikWBC9DOF/
```

**Redeploy to custom path:**
```powershell
.\deploy_to_orin_complete.ps1 -OrinIP "x.x.x.x" -RemotePath "/desired/path"
```

---

## Notes for Future Deployments

1. **Always use the deployment script** for consistency
2. **Deployment path is configurable** via `-RemotePath` parameter
3. **Meshes are optional** - use `-SkipMeshes` for faster deployment during testing
4. **Clean builds** may be needed after major changes:
   ```bash
   cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
   rm -rf build/ install/ log/
   colcon build
   ```

---

**Quick Command Reference:**

```powershell
# Deploy from Windows
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x"
```

```bash
# Build on Orin
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Run
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```
