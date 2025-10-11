# Deployment Scripts - Clean Organization

## 🎯 Main Deployment Script

### `deploy_to_orin.ps1` (23 KB)
**The ONE script you need for Orin deployment!**

Two modes supported:
- **Complete**: Full workspace deployment (first time)
- **ChassisOnly**: Fast updates (iterative testing)

```powershell
# First time deployment
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete

# Fast updates
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode ChassisOnly
```

Built-in defaults:
- Username: `cr`
- Remote path: `/home/nvidia/temp_gikrepo`

---

## 🛠️ Utility Scripts

### `check_build_current.ps1` (3.3 KB)
Verifies ROS2 workspace build status in WSL or Windows.

```powershell
.\scripts\deployment\check_build_current.ps1
```

### `cleanup_orin_remote.ps1` (1.2 KB)
Cleans up remote Orin workspace (build/, install/, log/ folders).

```powershell
.\scripts\deployment\cleanup_orin_remote.ps1 -OrinIP "192.168.100.150"
```

### `copy_codegen_to_ros2.ps1` (6.2 KB)
Copies MATLAB generated code to ROS2 workspace locally.

```powershell
.\scripts\deployment\copy_codegen_to_ros2.ps1
```

### `get_build_info.ps1` (1.9 KB)
Shows current build information and git status.

```powershell
.\scripts\deployment\get_build_info.ps1
```

### `run_planner_copy_to_ros2.ps1` (2.2 KB)
Copies planner-specific generated code to ROS2 workspace.

```powershell
.\scripts\deployment\run_planner_copy_to_ros2.ps1
```

---

## 📋 Script Purposes Summary

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `deploy_to_orin.ps1` | Deploy to Orin hardware | Production deployment |
| `check_build_current.ps1` | Verify builds | Before deployment |
| `cleanup_orin_remote.ps1` | Clean Orin workspace | Troubleshooting builds |
| `copy_codegen_to_ros2.ps1` | Local code organization | After codegen |
| `get_build_info.ps1` | Check project status | Anytime |
| `run_planner_copy_to_ros2.ps1` | Planner code copy | After planner codegen |

---

## ✅ Recent Cleanup (October 11, 2025)

**Removed:**
- ❌ `deploy_ros2_to_orin.ps1` - Superseded by unified `deploy_to_orin.ps1`
- ❌ `deploy_chassis_to_orin.ps1` - Merged into unified script
- ❌ `deploy_to_orin_complete.ps1` - Merged into unified script

**Result:**
- One clear deployment path
- No confusion about which script to use
- Better maintainability

---

## 🚀 Quick Start

**For Orin deployment, you only need ONE command:**

```powershell
.\scripts\deployment\deploy_to_orin.ps1 -OrinIP "192.168.100.150" -Mode Complete
```

Everything else is handled automatically!

---

## 📚 Documentation

- **READY_TO_DEPLOY_NOW.md** - Quick start guide
- **UNIFIED_DEPLOYMENT_SUMMARY.md** - Complete explanation
- **DEPLOY_TO_ORIN_GUIDE.md** - Detailed usage guide

---

**Last Updated**: October 11, 2025  
**Commit**: 7976d33 - Cleaned up old deployment scripts
