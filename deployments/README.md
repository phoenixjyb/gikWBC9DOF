# Deployment Packages

This directory contains pre-built deployment packages for the GIK 9-DOF controller.

## Available Packages

### `gikWBC9DOF_arm64_deployment_20251007_084546.zip`
**Platform**: ARM64 (NVIDIA AGX Orin)  
**Date**: October 7, 2025  
**Contents**:
- Compiled ARM64 binaries
- MATLAB Coder generated libraries
- ROS2 packages for Humble
- Configuration files

**Deployment**:
```bash
# Extract on target (AGX Orin)
unzip gikWBC9DOF_arm64_deployment_20251007_084546.zip
cd gikWBC9DOF_arm64

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

### `ros2_workspace_20251007_003017.zip`
**Platform**: x86_64/ARM64  
**Date**: October 7, 2025  
**Contents**:
- Complete ROS2 workspace source
- All package dependencies
- Configuration files

**Usage**:
```bash
# Extract
unzip ros2_workspace_20251007_003017.zip
cd ros2

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

---

## Deployment Guide

For detailed deployment instructions, see:
- [BUILD_ON_ORIN.md](../BUILD_ON_ORIN.md) - AGX Orin specific build
- [ORIN_DEPLOYMENT_PATH.md](../ORIN_DEPLOYMENT_PATH.md) - Complete deployment workflow
- [DEPLOY_NOW.md](../DEPLOY_NOW.md) - Quick deployment steps

---

## Creating New Packages

Use the deployment scripts in the root directory:

```powershell
# Windows PowerShell
.\deploy_to_orin_complete.ps1
```

This will create a timestamped .zip package ready for deployment.

---

## Package Contents

Each deployment package typically includes:

```
deployment_package/
├── ros2/                    # ROS2 workspace
│   ├── src/                 # Source packages
│   ├── install/            # Built binaries (if pre-built)
│   └── build/              # Build artifacts (if included)
├── config/                  # Configuration files
├── launch/                  # Launch files
└── README.md               # Package-specific instructions
```

---

**Last Updated**: October 7, 2025
