# Scripts Index

This directory contains all automation scripts for code generation, testing, and deployment.

---

## 📂 Directory Structure

### [codegen/](codegen/)
**Code generation scripts for MATLAB Coder**

#### MATLAB Scripts
- `generate_code_velocity_smoothing.m` - Generate velocity smoothing C++ code
- `run_planner_codegen_wsl.m` - Run planner codegen in WSL/MATLAB

#### PowerShell Scripts
- `run_planner_codegen.ps1` - Run planner code generation (Windows)
- `RUN_ALL_PLANNER_REGEN.ps1` - Regenerate all planner code

#### Bash Scripts
- `run_planner_codegen_wsl.sh` - Run planner codegen wrapper (WSL)
- `RUN_PLANNER_WSL.sh` - Run planner in WSL environment
- `run_trajectory_smoothing_codegen.sh` - Generate trajectory smoothing code
- `run_velocity_smoothing_codegen.sh` - Generate velocity smoothing code
- `COPY_PASTE_PLANNER_CODEGEN.sh` - Copy/paste planner codegen helper

### [testing/](testing/)
**Testing and validation scripts**

- `run_planner_verify.ps1` - Verify planner code generation (PowerShell)
- `test_matlab_batch.sh` - Batch MATLAB testing (Bash)
- `run_real_trajectory_test.bat` - Test with real trajectory data (Windows)

### [deployment/](deployment/)
**Deployment automation**

- `run_planner_copy_to_ros2.ps1` - Copy planner code to ROS2 workspace

---

## 🚀 Quick Start

### Generate Velocity Smoothing Code
```bash
# In WSL:
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF
./scripts/codegen/run_velocity_smoothing_codegen.sh
```

### Generate Trajectory Smoothing Code
```bash
# In WSL:
./scripts/codegen/run_trajectory_smoothing_codegen.sh
```

### Run Planner Code Generation
```powershell
# In PowerShell:
.\scripts\codegen\run_planner_codegen.ps1
```

### Verify Code Generation
```powershell
# In PowerShell:
.\scripts\testing\run_planner_verify.ps1
```

### Deploy to ROS2
```powershell
# In PowerShell:
.\scripts\deployment\run_planner_copy_to_ros2.ps1
```

---

## 📝 Script Details

### Code Generation Scripts

#### `generate_code_velocity_smoothing.m`
- **Purpose**: Generate ARM-optimized C++ code for velocity smoothing
- **Input**: MATLAB function `+gik9dof/+control/smoothVelocityCommand.m`
- **Output**: `codegen/velocity_smoothing/*.cpp/h`
- **Duration**: ~40 seconds
- **Target**: ARM Cortex-A78 (Jetson AGX Orin)

#### `run_velocity_smoothing_codegen.sh`
- **Purpose**: Wrapper script to run velocity smoothing codegen from bash
- **Platform**: WSL/Linux
- **Usage**: `./run_velocity_smoothing_codegen.sh`

#### `run_trajectory_smoothing_codegen.sh`
- **Purpose**: Generate C++ code for trajectory smoothing
- **Input**: MATLAB function `smoothTrajectoryVelocity.m`
- **Output**: `codegen/trajectory_smoothing/*.cpp/h`
- **Platform**: WSL/Linux

#### `run_planner_codegen.ps1`
- **Purpose**: Generate planner code from PowerShell
- **Platform**: Windows PowerShell
- **Features**: Progress output, error handling

#### `RUN_ALL_PLANNER_REGEN.ps1`
- **Purpose**: Regenerate all planner-related code
- **Platform**: Windows PowerShell
- **Includes**: Multiple planner variants

### Testing Scripts

#### `run_planner_verify.ps1`
- **Purpose**: Verify planner code generation succeeded
- **Checks**: File existence, compilation status
- **Platform**: Windows PowerShell

#### `test_matlab_batch.sh`
- **Purpose**: Run batch MATLAB tests
- **Platform**: WSL/Linux
- **Usage**: `./test_matlab_batch.sh`

#### `run_real_trajectory_test.bat`
- **Purpose**: Test with real trajectory data
- **Platform**: Windows batch
- **Data Source**: `data/` directory

### Deployment Scripts

#### `run_planner_copy_to_ros2.ps1`
- **Purpose**: Copy generated planner code to ROS2 workspace
- **Source**: `codegen/planner_arm64/`
- **Destination**: `ros2/gik9dof_solver/`
- **Platform**: Windows PowerShell

---

## 🛠️ Development Workflow

### Typical Code Generation Workflow
```bash
# 1. Generate velocity smoothing code
./scripts/codegen/run_velocity_smoothing_codegen.sh

# 2. Copy to ROS2 (if needed)
# (Usually done automatically during generation)

# 3. Build ROS2 package
cd ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_solver

# 4. Verify build
./scripts/testing/run_planner_verify.ps1
```

### Complete Planner Regeneration
```powershell
# Regenerate all planner code
.\scripts\codegen\RUN_ALL_PLANNER_REGEN.ps1

# Deploy to ROS2
.\scripts\deployment\run_planner_copy_to_ros2.ps1

# Verify
.\scripts\testing\run_planner_verify.ps1
```

---

## 🔧 Script Configuration

### Environment Variables
Most scripts use these environment variables:
- `MATLAB_ROOT`: MATLAB installation path (usually auto-detected)
- `ROS_DISTRO`: ROS2 distribution (humble)
- `PROJECT_ROOT`: Project root directory

### Paths
Scripts assume this directory structure:
```
gikWBC9DOF/
├── scripts/          # Scripts directory (here)
├── matlab/           # MATLAB source code
├── codegen/          # Generated code output
└── ros2/             # ROS2 workspace
```

---

## 📋 Script Maintenance

### Adding New Scripts
1. Place in appropriate subdirectory (`codegen/`, `testing/`, `deployment/`)
2. Make executable (bash scripts): `chmod +x script_name.sh`
3. Add documentation to this README
4. Test on target platforms (Windows/WSL)

### Script Naming Convention
- PowerShell: `run_[purpose].ps1` or `RUN_[PURPOSE].ps1`
- Bash: `run_[purpose].sh` or `run_[purpose]_[platform].sh`
- Batch: `run_[purpose].bat`
- MATLAB: `[purpose]_codegen.m` or `run_[purpose].m`

---

## 🐛 Troubleshooting

### "MATLAB not found"
- Verify MATLAB installation path
- Check PATH environment variable
- For WSL: Ensure MATLAB accessible at `/home/yanbo/MATLAB/R2024a/bin/matlab`

### "Permission denied" (bash scripts)
```bash
chmod +x scripts/codegen/*.sh
chmod +x scripts/testing/*.sh
```

### Code generation fails
- Check MATLAB Coder license
- Verify source files exist in `matlab/` directory
- Review codegen logs in `logs/codegen_output.log`

### ROS2 copy fails
- Verify ROS2 workspace exists: `ros2/gik9dof_solver/`
- Check file permissions
- Ensure codegen completed successfully first

---

## 📊 Script Statistics

- **Total Scripts**: 12
- **Codegen Scripts**: 9 (MATLAB, PowerShell, Bash)
- **Testing Scripts**: 3 (PowerShell, Bash, Batch)
- **Deployment Scripts**: 1 (PowerShell)
- **Platforms**: Windows, WSL/Linux, MATLAB

---

**Last Updated:** October 10, 2025  
**Maintained By:** Development Team

