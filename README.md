# GIK 9-DOF Whole-Body Controller

**9-DOF Mobile Manipulator Control System**  
**Platform**: NVIDIA Jetson AGX Orin | Ubuntu 22.04 | ROS2 Humble | ARM64

**Status**: ✅ Production Ready - Velocity Smoothing Integrated  
**Last Updated**: October 10, 2025  
**Branch**: `codegencc45-main`

---

## 🎯 Quick Start

### 🚀 For First-Time Users
👉 **[docs/getting-started/START_HERE.md](docs/getting-started/START_HERE.md)** - Complete setup guide

### 🏗️ For Building & Deploying
📘 **[docs/guides/COMPLETE_BUILD_GUIDE.md](docs/guides/COMPLETE_BUILD_GUIDE.md)** - ⭐ **Complete build tutorial** (MATLAB → ROS2 → Orin)

### 👨‍💻 For Developers
📖 **[docs/sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md](docs/sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md)** - Current session status

### 🎓 For Understanding the System
🏗️ **[docs/technical/architecture/STAGED_CONTROL_ARCHITECTURE.md](docs/technical/architecture/STAGED_CONTROL_ARCHITECTURE.md)** - Control architecture  
⚡ **[docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md](docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md)** - Latest feature

---

## 📁 Project Structure

```
gikWBC9DOF/
├── README.md                    # This file
├── WORKSPACE_ORGANIZATION_PLAN.md
│
├── docs/                        # 📚 All documentation
│   ├── README.md               # Documentation index
│   ├── getting-started/        # New user guides
│   ├── technical/              # Technical docs
│   │   ├── architecture/       # System architecture
│   │   ├── smoothing/          # Trajectory & velocity smoothing
│   │   ├── planning/           # Path planning (Hybrid A*)
│   │   └── codegen/            # MATLAB Coder
│   ├── fixes/                  # Bug fixes & solutions
│   ├── sessions/               # Development session notes
│   ├── testing/                # Test procedures
│   ├── deployment/             # Deployment guides
│   └── organization/           # Project organization
│
├── scripts/                     # 🔧 Automation scripts
│   ├── README.md               # Scripts index
│   ├── codegen/                # Code generation scripts
│   ├── testing/                # Test scripts
│   └── deployment/             # Deployment scripts
│
├── matlab/                      # 🧮 MATLAB source code
│   ├── +gik9dof/               # Main package
│   │   ├── +control/           # Control functions
│   │   ├── +planning/          # Path planning
│   │   └── +utils/             # Utilities
│   └── tests/                  # MATLAB tests
│
├── ros2/                        # 🤖 ROS2 workspace
│   └── gik9dof_solver/         # Main ROS2 package
│       ├── src/                # C++ source
│       ├── config/             # Configuration files
│       ├── matlab_codegen/     # Generated GIK solver
│       ├── trajectory_smoothing/ # Generated smoothing
│       ├── velocity_smoothing/  # Generated velocity smoothing (NEW)
│       └── CMakeLists.txt
│
├── codegen/                     # 🏗️ Generated code output
│   ├── velocity_smoothing/     # Velocity smoothing (NEW)
│   ├── trajectory_smoothing/   # Trajectory smoothing
│   ├── planner_arm64/          # Path planner (ARM64)
│   └── ...
│
├── models/                      # 🤖 Robot models
│   ├── mobile_manipulator_PPR_base_corrected.urdf
│   └── mobile_manipulator_PPR_base_corrected_sltRdcd.urdf
│
├── data/                        # 📊 Test data
│   └── trajectory files (.json)
│
├── meshes/                      # 🎨 Robot meshes
│   └── STL files
│
├── validation/                  # ✅ Validation data
│   └── Test cases
│
├── test_cpp/                    # 🧪 C++ tests
│   └── Unit tests
│
├── logs/                        # 📋 Log files
│   └── Codegen & build logs
│
└── deployments/                 # 📦 Deployment packages
    └── Archived deployments
```

---

## 🏗️ System Overview

### Core Components

#### 1. GIK 9-DOF Solver
- **Whole-body inverse kinematics** for mobile manipulator
- **3-DOF base** (x, y, θ) + **6-DOF arm** (6 joints)
- **MATLAB-generated C++** code for ARM64
- **Real-time capable**: <5ms solve time

#### 2. Path Planning (Hybrid A*)
- **3-DOF chassis navigation** in cluttered environments
- **Obstacle avoidance** with collision checking
- **MATLAB-generated C++** implementation

#### 3. Velocity Smoothing (NEW ✨)
- **Real-time velocity command smoothing**
- **Jerk limiting** for smoother motion
- **Applied in Stage B** (navigation phase)
- **10µs execution time**, <0.1% CPU overhead

#### 4. Trajectory Smoothing
- **Waypoint-based trajectory smoothing**
- **Pre-computation** for planned motions
- **Used in Mode 3** and Stage C

### Control Modes

The system supports **two control modes**:

#### Holistic Mode
- **9-DOF whole-body control** runs continuously
- Direct GIK solving at control rate
- Suitable for teleoperation and continuous tracking

#### Staged Mode (A → B → C)
Three time-based stages:

**Stage A (0-5s):** Arm Ramp-Up
- 6-DOF arm motion only
- Chassis parked (no motion)

**Stage B (5-35s):** Navigation ⭐
- 3-DOF chassis navigation
- Hybrid A* path planning
- Pure Pursuit velocity control
- **✅ Velocity smoothing applied**

**Stage C (35-65s):** Manipulation
- 9-DOF whole-body tracking
- Trajectory smoothing (Mode 3)

---

## 🚀 Latest Features

### ✅ Velocity Smoothing Integration (Oct 10, 2025)
- Created velocity-based real-time smoothing function
- Generated ARM-optimized C++ code (38.6s codegen)
- Integrated into ROS2 gik9dof_solver package
- Applied to Stage B navigation
- Build successful (1min 19s)
- **Status**: Ready for deployment

📖 **See**: [docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md](docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md)

### ✅ Hybrid A* Path Planning
- Complete 3-DOF path planner implementation
- Collision detection with LibCCD
- ARM64-optimized code generation
- **Status**: Production ready

### ✅ ROS2 Integration
- Full ROS2 Humble integration
- Parameter-based configuration
- Topic-based communication
- **Status**: Production ready

---

## 📚 Documentation

### 🎓 Start Here
- **[Getting Started Guide](docs/getting-started/START_HERE.md)** - Setup and first run
- **[Planner Quick Start](docs/getting-started/PLANNER_QUICK_START.md)** - Path planning guide
- **[Real Data Test Guide](docs/getting-started/REAL_DATA_TEST_GUIDE.md)** - Testing with real data

### 🏗️ Architecture
- **[Staged Control Architecture](docs/technical/architecture/STAGED_CONTROL_ARCHITECTURE.md)** - Three-stage control
- **[Integration Strategy](docs/technical/architecture/INTEGRATION_STRATEGY.md)** - System integration
- **[Downstream Control Analysis](docs/technical/architecture/DOWNSTREAM_CONTROL_ANALYSIS.md)** - Control flow

### ⚡ Velocity Smoothing (Latest)
- **[Integration Complete](docs/technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md)** - Full summary
- **[Quick Reference](docs/technical/smoothing/VELOCITY_SMOOTHING_QUICK_REF.md)** - Quick guide
- **[Architecture](docs/technical/smoothing/VELOCITY_SMOOTHING_ARCHITECTURE.md)** - Technical details
- **[Comparison](docs/technical/smoothing/SMOOTHING_COMPARISON.md)** - Waypoint vs velocity

### 🔧 Other Topics
- **[All Documentation](docs/README.md)** - Complete documentation index
- **[Scripts Index](scripts/README.md)** - Available automation scripts
- **[Fixes](docs/fixes/)** - Bug fixes and solutions
- **[Testing](docs/testing/)** - Test procedures

---

## 🔧 Building & Running

### Prerequisites
- **MATLAB R2024a** with Coder toolbox
- **ROS2 Humble** (Ubuntu 22.04)
- **CMake** 3.22+
- **GCC** 11+ (or appropriate ARM compiler for Jetson)

### Build ROS2 Package
```bash
cd ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_solver

# Source the workspace
source install/setup.bash
```

### Run Node
```bash
# Run GIK solver node
ros2 run gik9dof_solver gik9dof_solver_node

# In another terminal, monitor output
ros2 topic echo /cmd_vel
```

### Configuration
Edit parameters in:
```
ros2/gik9dof_solver/config/gik9dof_solver_params.yaml
```

Key parameters:
- `control_mode`: "holistic" or "staged"
- `velocity_control_mode`: 0 (Pure Pursuit), 1 (Heading), 2 (Combined), 3 (Waypoint)
- `velocity_smoothing.enable`: true/false
- Velocity/acceleration/jerk limits

---

## 🧪 Testing

### Generate Code
```bash
# Generate velocity smoothing
./scripts/codegen/run_velocity_smoothing_codegen.sh

# Generate trajectory smoothing
./scripts/codegen/run_trajectory_smoothing_codegen.sh

# Generate planner
./scripts/codegen/run_planner_codegen.ps1
```

### Verify Build
```powershell
# Windows PowerShell
.\scripts\testing\run_planner_verify.ps1
```

### Run Tests
```bash
# MATLAB batch tests
./scripts/testing/test_matlab_batch.sh

# Real trajectory test
./scripts/testing/run_real_trajectory_test.bat
```

---

## 📦 Deployment

### Deploy to Jetson Orin
```bash
# Copy built package
cd ros2
scp -r install/gik9dof_solver cr@192.168.100.150:~/ros2_ws/install/

# On Jetson
source ~/ros2_ws/install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

### Create Deployment Package
Built packages are archived in `deployments/` with timestamp.

---

## 🐛 Troubleshooting

### Common Issues

**"Waiting for robot state"**
- Node needs joint states (`/hdas/feedback_arm_left`) and odometry (`/odom_wheel`)
- Run robot/simulator or create test publishers

**Build Errors**
- Check [docs/fixes/ORIN_BUILD_FIX.md](docs/fixes/ORIN_BUILD_FIX.md)
- Verify libccd headers: [docs/fixes/LIBCCD_HEADERS_FIX.md](docs/fixes/LIBCCD_HEADERS_FIX.md)

**Code Generation Fails**
- Check MATLAB Coder license
- Review logs in `logs/codegen_output.log`
- See [docs/technical/codegen/CODEGEN_AUDIT.md](docs/technical/codegen/CODEGEN_AUDIT.md)

**More Help**
- Browse [docs/fixes/](docs/fixes/) for all known issues
- Check session notes in [docs/sessions/](docs/sessions/)

---

## 📊 Project Status

### ✅ Completed Features
- [x] 9-DOF GIK solver (MATLAB → C++)
- [x] Hybrid A* path planner (3-DOF)
- [x] Trajectory smoothing (waypoint-based)
- [x] Velocity smoothing (real-time) ⭐ NEW
- [x] ROS2 Humble integration
- [x] ARM64 optimization (Jetson Orin)
- [x] Staged control architecture (A→B→C)
- [x] Multiple velocity control modes (0/1/2/3)

### 🔄 In Progress
- Testing velocity smoothing with real robot
- Performance validation on Jetson

### 📋 Planned Features
- Advanced collision avoidance
- Dynamic obstacle handling
- Multi-robot coordination

---

## 📞 Support

### For Issues
1. Check [docs/fixes/](docs/fixes/) for known solutions
2. Review session notes in [docs/sessions/](docs/sessions/)
3. Consult architecture docs in [docs/technical/architecture/](docs/technical/architecture/)

### For Development
- Current work: [docs/sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md](docs/sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md)
- Organization: [WORKSPACE_ORGANIZATION_PLAN.md](WORKSPACE_ORGANIZATION_PLAN.md)

---

## 📄 License

[Add your license information here]

---

## 🙏 Acknowledgments

- MATLAB Coder for C++ code generation
- ROS2 Humble for robotics middleware
- LibCCD for collision detection
- NVIDIA Jetson platform

---

**Last Updated**: October 10, 2025  
**Repository**: phoenixjyb/gikWBC9DOF  
**Branch**: codegencc45-main

