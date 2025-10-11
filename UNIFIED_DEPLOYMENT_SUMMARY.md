# Unified Orin Deployment - Summary

## What Changed

**Before**: Multiple separate deployment scripts
- `deploy_chassis_to_orin.ps1` - Chassis only
- `deploy_to_orin_complete.ps1` - Complete workspace
- Different parameters, confusing to maintain

**After**: One unified script with mode selection
- `deploy_to_orin.ps1` - Handles everything
- `-Mode Complete` for full workspace
- `-Mode ChassisOnly` for fast updates

## Your Settings (Now Built In)

✅ **Username**: `cr` (default)  
✅ **Remote Path**: `/home/nvidia/temp_gikrepo` (default)  
✅ **No more configuration needed!**

## Quick Usage

### First Time Deployment (Complete)

```powershell
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

**What it does**:
- ✅ Transfers entire ROS2 workspace
- ✅ Transfers URDF files
- ✅ Transfers mesh files (use `-SkipMeshes` to skip)
- ✅ Transfers config files
- ✅ Builds everything on Orin
- ✅ Verifies installation
- ⏱️ **Takes**: 2-5 minutes

### Fast Updates (Chassis Only)

```powershell
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly
```

**What it does**:
- ✅ Transfers only MATLAB generated code (19 files)
- ✅ Transfers ROS2 wrapper node
- ✅ Transfers build files + launch file
- ✅ Builds gik9dof_controllers package
- ✅ Perfect for iterative testing
- ⏱️ **Takes**: 30-60 seconds

## All Options

```powershell
.\scripts\deployment\deploy_to_orin.ps1 `
    -OrinIP "192.168.100.150" `      # Required: IP address
    -Username "cr" `                  # Optional: Default is "cr"
    -RemotePath "/home/nvidia/temp_gikrepo" `  # Optional: Default path
    -Mode Complete `                  # Complete or ChassisOnly
    -SkipMeshes `                    # Optional: Skip mesh files
    -BuildOnOrin:$true `             # Optional: Auto-build (default: true)
    -TestAfterBuild:$false           # Optional: Run tests (default: false)
```

## Transfer Methods

**Preferred**: `rsync` (fast, incremental)
- Automatically used if available
- Only transfers changed files
- Shows progress

**Fallback**: `ZIP + SCP`
- Used if rsync not installed
- Creates temporary archive
- Still reliable

## After Deployment

### SSH to Orin

```bash
ssh cr@192.168.100.150
cd /home/nvidia/temp_gikrepo/ros2
```

### Source and Launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch default mode (Pure Pursuit)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py

# Or test specific mode
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0  # Differentiation
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1  # Heading-Aware
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2  # Pure Pursuit
```

## Typical Workflow

### Day 1: Initial Setup

```powershell
# Deploy everything first time
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

### Day 2+: Testing and Tuning

```powershell
# Make changes to MATLAB code
# ... edit and test in MATLAB ...

# Regenerate code
wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh

# Fast deploy updated code
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly

# Test on Orin
ssh cr@192.168.100.150
# ... launch and test ...
```

### When You Update Other Files

```powershell
# If you change URDFs, meshes, or other packages
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

## Verification

The script automatically verifies:
- ✅ SSH connection works
- ✅ Directory structure created
- ✅ All files transferred
- ✅ Generated code present (19 files)
- ✅ Wrapper node present
- ✅ Build files present
- ✅ Launch file present
- ✅ ChassisPathFollower.cpp size correct (~33 KB)
- ✅ Build successful (if -BuildOnOrin)
- ✅ Executable created

## Troubleshooting

### SSH Connection Failed

```powershell
# Test manually
ssh cr@192.168.100.150

# Check IP
ping 192.168.100.150
```

### Generated Code Not Found

```bash
# Run codegen first
wsl bash scripts/codegen/run_chassis_path_codegen_wsl.sh

# Verify files exist
ls codegen/chassis_path_follower_arm64/
```

### Build Failed on Orin

```bash
# SSH to Orin
ssh cr@192.168.100.150
cd /home/nvidia/temp_gikrepo/ros2

# Check logs
cat log/latest_build/gik9dof_controllers/stdout_stderr.log

# Clean and rebuild
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_msgs gik9dof_controllers
```

## Benefits of Unified Script

1. **Single Source of Truth**: One script to maintain
2. **Consistent Settings**: Username and path built-in
3. **Mode Selection**: Complete vs ChassisOnly via parameter
4. **Better Documentation**: All options in one place
5. **Easier Updates**: Change once, works everywhere
6. **Clearer Usage**: Less confusion about which script to use

## Documentation

- **This File**: Quick summary
- **DEPLOY_TO_ORIN_GUIDE.md**: Complete guide with all options
- **ros2/gik9dof_controllers/README.md**: Package documentation (on Orin after deployment)
- **DAY5_CODEGEN_SESSION_SUMMARY.md**: Codegen details

## Quick Reference Card

```
┌─────────────────────────────────────────────────────┐
│           Orin Deployment Quick Reference           │
├─────────────────────────────────────────────────────┤
│                                                     │
│  Script: .\scripts\deployment\deploy_to_orin.ps1   │
│                                                     │
│  First Time:                                        │
│  -OrinIP "192.168.100.150" -Mode Complete          │
│                                                     │
│  Fast Updates:                                      │
│  -OrinIP "192.168.100.150" -Mode ChassisOnly       │
│                                                     │
│  On Orin:                                           │
│  ssh cr@192.168.100.150                            │
│  cd /home/nvidia/temp_gikrepo/ros2                 │
│  source /opt/ros/humble/setup.bash                 │
│  source install/setup.bash                         │
│  ros2 launch gik9dof_controllers \                 │
│       chassis_path_follower_launch.py              │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

**Ready to deploy?** 🚀

```powershell
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```
