# GIK 9-DOF Whole-Body Controller# GIK 9-DOF Whole-Body Controller# CODEGENCC45 Project - Documentation Index



**9-DOF Mobile Manipulator Control System**  

**Platform**: NVIDIA AGX Orin | Ubuntu 22.04 | ROS2 Humble | ARM64  

**Status**: ✅ Production Ready - Namespace Conflicts Resolved  **9-DOF Mobile Manipulator Control System**  **Project**: 9-DOF Whole-Body IK Solver for AGX Orin  

**Last Updated**: October 7, 2025

**Platform**: NVIDIA AGX Orin | Ubuntu 22.04 | ROS2 Humble | ARM64  **Branch**: `codegencc45`  

---

**Status**: ✅ Production Ready - Namespace Conflicts Resolved  **Target**: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64  

## 🎯 Quick Start

**Last Updated**: October 7, 2025**Deadline**: 2-day implementation  

### For First-Time Users

👉 **[START_HERE.md](START_HERE.md)** - Complete setup and execution guide



### For Developers------

- 📖 **[QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md)** - Development quick reference  

- 🏗️ **[ROS2 Integration Guide](docs/guides/ROS2_INTEGRATION_COMPLETE.md)** - ROS2 integration details  

- ✅ **[Test Results](NAMESPACE_FIX_TEST_RESULTS.md)** - Latest test results  

## 🎯 Quick Start# 9-DOF Whole-Body IK Solver for Mobile Manipulator

---



## 📁 Project Structure

### For First-Time Users**Project**: MATLAB-Generated C++ IK Solver for ROS2  

```

gikWBC9DOF/👉 **[START_HERE.md](START_HERE.md)** - Complete setup and execution guide**Branch**: `codegencc45`  

├── README.md                    # This file

├── START_HERE.md               # Quick start guide**Target**: NVIDIA AGX Orin, Ubuntu 22.04, ROS2 Humble, ARM64  

├── QUICK_START_NEXT_SESSION.md # Developer quick ref

│### For Developers**Status**: ✅ MATLAB-to-ROS2 Integration Complete  

├── 📂 matlab/                  # MATLAB source code

│   ├── gik9DOF_solver/        # 9-DOF IK solver- 📖 **[QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md)** - Development quick reference**Platform Support**: ✅ x86_64 (Windows/WSL) | ✅ ARM64 (AGX Orin)

│   ├── holistic_velocity_controller/

│   ├── pure_pursuit_velocity_ctrl/- 🏗️ **[ROS2_INTEGRATION_COMPLETE.md](ROS2_INTEGRATION_COMPLETE.md)** - ROS2 integration details

│   └── hybrid_astar_planner/

│- ✅ **[NAMESPACE_FIX_TEST_RESULTS.md](NAMESPACE_FIX_TEST_RESULTS.md)** - Latest test results---

├── 📂 ros2/                    # ROS2 workspace

│   ├── gik9dof_solver/        # Main ROS2 node

│   ├── gik_matlab_solver/     # GIK solver library

│   ├── hybrid_astar_planner/  # Planner library---## Platform Support

│   ├── holistic_velocity_controller/

│   └── pure_pursuit_velocity_ctrl/

│

├── 📂 docs/                    # Documentation## 📁 Project Structure| Platform | Architecture | Status | Notes |

│   ├── technical/             # Technical documentation

│   │   ├── hybrid-astar/     # Hybrid A* planner docs|----------|-------------|--------|-------|

│   │   ├── pure-pursuit/     # Pure Pursuit docs

│   │   └── gik-solver/       # GIK solver docs```| **Windows** | x86_64 | ✅ Validated | MATLAB codegen + validation |

│   ├── deployment/            # Deployment guides

│   ├── guides/                # Development guidesgikWBC9DOF/| **WSL2 Ubuntu 22.04** | x86_64 | ✅ Validated | ROS2 build and testing |

│   └── archive/               # Historical documents

│       ├── sessions/          # Old session notes├── README.md                              # This file| **AGX Orin** | ARM64/aarch64 | ✅ **NEW: Working** | Built with compatibility layer |

│       └── namespace-conflict/ # Namespace fix history

│├── START_HERE.md                          # Quick start guide

├── 📂 deployments/             # Deployment packages

│   └── *.zip                  # Built packages│**Latest**: Successfully deployed to ARM64 with SSE intrinsics compatibility fixes (Oct 2025).  

│

├── 📂 validation/              # Test data & results├── 📂 matlab/                             # MATLAB source codeSee [ARM64 Deployment Guide](docs/deployment/ARM64_DEPLOYMENT_GUIDE.md) for details.

└── 📂 meshes/                  # Robot URDF files

```│   ├── gik9DOF_solver/                   # 9-DOF IK solver



---│   ├── holistic_velocity_controller/     # Simple heading controller---



## 🏗️ System Architecture│   ├── pure_pursuit_velocity_ctrl/       # Pure Pursuit controller



### Three-Stage Control System│   └── hybrid_astar_planner/             # Chassis path planner## 🚀 Quick Start



**Stage A: Arm Ramp-Up**│

- Move arm to home configuration

- Prepare for chassis motion├── 📂 ros2/                               # ROS2 workspace### New to This Project?



**Stage B: Chassis Planning** ⭐ *Namespace conflict resolved*│   ├── gik9dof_solver/                   # Main ROS2 node👉 **Start here:** [`START_HERE.md`](START_HERE.md) - Complete execution guide

- Hybrid A* path planning

- Pure Pursuit path following│   │   ├── src/                          # C++ source

- Arm held static at home

│   │   │   ├── gik9dof_solver_node.cpp  # Main node### Need Quick Reference?

**Stage C: Whole-Body Tracking**

- Full 9-DOF control (3 base + 6 arm)│   │   │   ├── stage_b_factory.hpp      # Factory pattern (namespace fix)- 📖 [Quick Start Guide](docs/guides/QUICK_START.md) - 15-minute overview

- GIK solver for coordinated motion

- End-effector tracking│   │   │   └── stage_b_chassis_plan.cpp # Stage B controller- 🔄 [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) - Testing strategy



---│   │   ├── include/                      # Headers- 🚀 [Orin Deployment](docs/deployment/ORIN_NEXT_STEPS.md) - Deploy to AGX Orin



## ✅ Recent Achievements│   │   └── config/                       # ROS2 parameters



### Namespace Conflict Resolution (Oct 7, 2025)│   │---

- ✅ Resolved namespace collision between GIK solver and Hybrid A* planner

- ✅ Implemented Pimpl pattern with wrapper functions│   ├── gik_matlab_solver/                # GIK solver library

- ✅ Clean build with no conflicts

- ✅ Runtime tested and validated│   ├── hybrid_astar_planner/             # Planner library## 📚 Documentation Structure



**Details**: [NAMESPACE_CONFLICT_RESOLVED.md](NAMESPACE_CONFLICT_RESOLVED.md)│   ├── holistic_velocity_controller/     # Velocity controller



### ROS2 Integration│   └── pure_pursuit_velocity_ctrl/       # Pure Pursuit libraryAll documentation has been organized into logical categories:

- ✅ Complete ROS2 Humble integration

- ✅ Parameter server configuration│

- ✅ Topic-based control interface

├── 📂 codegen/                            # MATLAB Coder output### 📘 Guides (User-Facing)

**Details**: [ROS2 Integration Guide](docs/guides/ROS2_INTEGRATION_COMPLETE.md)

│   ├── lib/                              # Generated libraries- [ROS2 Integration Guide](docs/guides/ROS2_INTEGRATION_GUIDE.md) - How to integrate with ROS2

---

│   └── dll/                              # Generated DLLs- [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) - Complete testing strategy

## 🚀 Building and Running

│- [Quick Start](docs/guides/QUICK_START.md) - Fast project overview

### Prerequisites

- Ubuntu 22.04├── 📂 docs/                               # Documentation- [Development Guidelines](docs/guides/guideline.md) - Coding standards

- ROS2 Humble

- MATLAB R2024a+ (for code generation)│   ├── guides/                           # User guides

- C++17 compiler

- Eigen3│   ├── design/                           # Design documents### 🚀 Deployment



### Build│   ├── deployment/                       # Deployment guides- [Orin Next Steps](docs/deployment/ORIN_NEXT_STEPS.md) - **Deploy to AGX Orin**

```bash

cd ros2│   └── archive/                          # Archived documents- [ARM64 Deployment Guide](docs/deployment/ARM64_DEPLOYMENT_GUIDE.md) - **✨ NEW: ARM64/Orin deployment**

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash│       ├── sessions/                     # Old session notes- [CodegenCC45 README](docs/deployment/README_CODEGENCC45.md) - Deployment overview

```

│       └── namespace-conflict/           # Namespace fix history

### Run

```bash│#### WSL Workflow

ros2 run gik9dof_solver gik9dof_solver_node \

  --ros-args --params-file gik9dof_solver/config/gik9dof_solver.yaml├── 📂 deployments/                        # Deployment packages- [WSL Integration Summary](docs/deployment/wsl/WSL_INTEGRATION_SUMMARY.md) - Why use WSL

```

│   ├── *.zip                             # Built packages for Orin- [WSL Validation Guide](docs/deployment/wsl/WSL_VALIDATION_GUIDE.md) - Complete WSL testing

---

│   └── README.md                         # Deployment instructions- [WSL Quick Reference](docs/deployment/wsl/WSL_QUICK_REFERENCE.md) - Command cheatsheet

## 📚 Documentation

│

### 📘 Essential Reading

| Document | Description |├── 📂 validation/                         # Validation & test data### 🔧 Technical Reference

|----------|-------------|

| [START_HERE.md](START_HERE.md) | First-time setup guide |│   ├── test_cases/                       # Test scenarios- [CODEGEN Structure](docs/technical/CODEGEN_STRUCTURE.md) - **Understanding codegen directories**

| [QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md) | Development quick reference |

| [NAMESPACE_FIX_TEST_RESULTS.md](NAMESPACE_FIX_TEST_RESULTS.md) | Latest test results |│   └── results/                          # Validation results- [CODEGEN Analysis](docs/technical/CODEGEN.md) - MATLAB code generation details

| [NAMESPACE_CONFLICT_RESOLVED.md](NAMESPACE_CONFLICT_RESOLVED.md) | Namespace fix solution |

│- [MATLAB Codegen Analysis](docs/technical/MATLAB_CODEGEN_ANALYSIS.md) - What to generate

### 🔧 Technical Documentation

└── 📂 meshes/                             # Robot URDF and meshes- [Unified Chassis Summary](docs/technical/unified_chassis_controller_summary.md) - Controller details

**Hybrid A* Planner** (`docs/technical/hybrid-astar/`)

- [README](docs/technical/hybrid-astar/HYBRID_ASTAR_README.md) - Algorithm overview    └── mobile_manipulator*.urdf          # Robot models

- [Design](docs/technical/hybrid-astar/HYBRID_ASTAR_DESIGN.md) - Architecture details

- [Comparison](docs/technical/hybrid-astar/HYBRID_ASTAR_COMPARISON.md) - vs other planners```### 📋 Project Planning

- [Perception Integration](docs/technical/hybrid-astar/HYBRID_ASTAR_PERCEPTION_INTEGRATION.md)

- [Project Plan](docs/planning/CODEGENCC45_PROJECT_PLAN.md) - Master architecture

**Pure Pursuit Controller** (`docs/technical/pure-pursuit/`)

- [Quickstart](docs/technical/pure-pursuit/PUREPURSUIT_QUICKSTART.md)---- [Implementation Roadmap](docs/planning/IMPLEMENTATION_ROADMAP.md) - Original 6-8 week plan

- [Design](docs/technical/pure-pursuit/PUREPURSUIT_DESIGN.md)

- [Bidirectional Support](docs/technical/pure-pursuit/PUREPURSUIT_BIDIRECTIONAL_COMPLETE.md)- [Implementation Summary](docs/planning/IMPLEMENTATION_SUMMARY.md) - What was built

- [Integration](docs/technical/pure-pursuit/PUREPURSUIT_INTEGRATION_COMPLETE.md)

- [Dependencies](docs/technical/pure-pursuit/PUREPURSUIT_DEPENDENCY_ANALYSIS.md)## 🏗️ System Architecture- [Week 1 Guide](docs/planning/WEEK1_IMPLEMENTATION_GUIDE.md) - Week 1 details

- [Reverse Analysis](docs/technical/pure-pursuit/PUREPURSUIT_REVERSE_ANALYSIS.md)

- [Fast Track 2-Day](docs/planning/FAST_TRACK_2DAY.md) - Hour-by-hour plan

**GIK Solver** (`docs/technical/gik-solver/`)

- [Enhancements Quick Ref](docs/technical/gik-solver/GIK_ENHANCEMENTS_QUICKREF.md)
- [Stage B Fixes](docs/technical/gik-solver/STAGE_B_FIXES.md)
- [Planner Integration](docs/technical/gik-solver/PLANNER_PROJECT_SUMMARY.md)

**⭐ Control System Architecture** (`docs/technical/`)

- **[State Machine Architecture](docs/technical/STATE_MACHINE_ARCHITECTURE.md)** - **How to switch between control modes, stages, and velocity controllers**

- [Chassis Integration](docs/technical/gik-solver/CHASSIS_INTEGRATION_HYBRID_ASTAR.md)

**Stage A: Arm Ramp-Up**### 📦 Archive (Historical)

### 🚀 Deployment Guides (`docs/deployment/`)

- [Build on AGX Orin](docs/deployment/BUILD_ON_ORIN.md)- Move arm to home configuration- [Context Handoff](docs/archive/CONTEXT_HANDOFF.md) - Project context for conversations

- [Deployment Path](docs/deployment/ORIN_DEPLOYMENT_PATH.md)

- [Testing Guide](docs/deployment/ORIN_TESTING_GUIDE.md)- Prepare for chassis motion- [How to Use Context Handoff](docs/archive/HOW_TO_USE_CONTEXT_HANDOFF.md)

- [Deploy Now](docs/deployment/DEPLOY_NOW.md)

- [Package Info](docs/deployment/DEPLOYMENT_PACKAGE_INFO.md)- [Project Overview](docs/archive/PROJECT_OVERVIEW.md) - Historical overview



### 📖 Development Guides (`docs/guides/`)**Stage B: Chassis Planning** ⭐ *New: Namespace conflict resolved*- [Development Diary](docs/archive/diary.md) - Development log

- [ROS2 Integration](docs/guides/ROS2_INTEGRATION_COMPLETE.md)

- [Code Generation](docs/guides/CODEGEN_QUICK_GUIDE.md)- Hybrid A* path planning- [Origin Main Merge Analysis](docs/archive/ORIGIN_MAIN_MERGE_ANALYSIS.md)

- [Code Generation Success Patterns](docs/guides/CODEGEN_SUCCESS_SUMMARY.md)

- [Quick Testing](docs/guides/QUICKSTART_TESTING.md)- Pure Pursuit path following

- [Naming Conventions](docs/guides/NAMING_CONVENTION.md)

- [State Machine Integration](docs/guides/STATE_MACHINE_INTEGRATION_STATUS.md)- Arm held static at home position### 📄 Additional Documentation



---- [Reorganization Plan](docs/REORGANIZATION_PLAN.md) - How docs were organized



## 🔧 Development**Stage C: Whole-Body Tracking**- [Orin MATLAB Integration](docs/ORIN_MATLAB_INTEGRATION.md) - MATLAB solver integration



### Code Generation (MATLAB)- Full 9-DOF control (3 base + 6 arm)- [Validation Workflow](docs/VALIDATION_WORKFLOW.md) - Testing documentation

```matlab

% Generate all components- GIK solver for coordinated motion

run('RUN_CODEGEN.m')

- End-effector tracking---

% Or generate individually:

run('generate_code_arm64.m')        % For ARM64

run('generate_code_planner_arm64.m') % Hybrid A* for ARM64

```### Key Components## 🎯 Quick Access by Task



### Validation

```matlab

% Run validation suite```## 🎯 Quick Access by Task

run('RUN_VALIDATION.m')

```┌─────────────────────────────────────────────────────┐



### Debugging│  ROS2 Node (gik9dof_solver_node)                   │### "I Want to Start Right Now"

- Check `codegen_debug.log` for build issues

- Use `test_codegen_debug.m` for isolated testing│  - Manages control stages                          │1. Read [`START_HERE.md`](START_HERE.md) (5 minutes)



---│  - Publishes /cmd_vel and /joint_commands          │2. Open MATLAB and run `RUN_VALIDATION`



## 📊 Project Status└─────────────────────────────────────────────────────┘3. Follow the 4-step workflow



| Component | Status | Notes |                     │

|-----------|--------|-------|

| GIK Solver (9-DOF) | ✅ Complete | MATLAB Coder generated |        ├────────────┼────────────┐### "I Need to Build on WSL"

| Hybrid A* Planner | ✅ Complete | Namespace conflict resolved |

| Pure Pursuit Controller | ✅ Complete | Bidirectional support |        ▼            ▼            ▼1. Read [WSL Quick Reference](docs/deployment/wsl/WSL_QUICK_REFERENCE.md) (copy/paste commands)

| ROS2 Integration | ✅ Complete | Humble, parameter server |

| Stage A (Arm Ramp) | ✅ Complete | Tested |┌──────────┐  ┌──────────┐  ┌──────────┐2. Troubleshooting: [WSL Validation Guide](docs/deployment/wsl/WSL_VALIDATION_GUIDE.md)

| Stage B (Chassis Plan) | ✅ Complete | Wrapper functions tested |

| Stage C (Whole-Body) | ✅ Complete | Full 9-DOF control |│ Stage A  │  │ Stage B  │  │ Stage C  │

| ARM64 Build | ✅ Complete | AGX Orin validated |

| Runtime Testing | ✅ Complete | Node runs successfully |│ Arm Ramp │  │ Chassis  │  │ Whole-   │### "I'm Deploying to AGX Orin"



---│          │  │ Planning │  │ Body IK  │1. **NEW**: Read [ARM64 Deployment Guide](docs/deployment/ARM64_DEPLOYMENT_GUIDE.md) - **Complete ARM64 build instructions**



## 📝 Archive└──────────┘  └──────────┘  └──────────┘2. Read [Orin Next Steps](docs/deployment/ORIN_NEXT_STEPS.md)



Old session notes and intermediate documentation: `docs/archive/`                    │              │3. Transfer files to Orin using deployment script

- `sessions/` - Development session summaries

- `namespace-conflict/` - Namespace fix work-in-progress docs                    ▼              ▼4. Build on target following ARM64 guide



---            ┌──────────────┐  ┌──────────────┐



## 📞 Quick Reference            │ Hybrid A*    │  │ GIK Solver   │### "I'm Stuck / Troubleshooting"



**Branch**: `codegencc45`              │ Planner      │  │ (9-DOF)      │1. Check [Fast Track 2-Day](docs/planning/FAST_TRACK_2DAY.md) → Troubleshooting section

**ROS2**: Humble  

**Platform**: NVIDIA AGX Orin (ARM64)              └──────────────┘  └──────────────┘2. Check [WSL Validation Guide](docs/deployment/wsl/WSL_VALIDATION_GUIDE.md) → Common Issues

**Build**: colcon (ROS2)  

**MATLAB**: R2024a+                    │              3. Check [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) → Failure Recovery Paths



**Need Help?**                    ▼              

1. [START_HERE.md](START_HERE.md) - Setup guide

2. [QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md) - Quick reference            ┌──────────────┐### "I Need Architecture Details"

3. `docs/archive/` - Historical context

            │ Pure Pursuit │1. [Project Plan](docs/planning/CODEGENCC45_PROJECT_PLAN.md) - Overall architecture

---

            │ Controller   │2. [Requirements Confirmed](docs/planning/REQUIREMENTS_CONFIRMED.md) - All design decisions

**Last Updated**: October 7, 2025  

**Status**: Production Ready ✅            └──────────────┘3. [ROS2 Integration Guide](docs/guides/ROS2_INTEGRATION_GUIDE.md) - ROS2 specifics


```4. [CODEGEN Structure](docs/technical/CODEGEN_STRUCTURE.md) - Directory organization



---### "I Need to Explain This to Someone"

1. [Quick Start](docs/guides/QUICK_START.md) - 15-minute overview

## ✅ Recent Achievements2. [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) - Visual workflow diagram

3. [WSL Integration Summary](docs/deployment/wsl/WSL_INTEGRATION_SUMMARY.md) - Why WSL matters

### Namespace Conflict Resolution (Oct 7, 2025)

- ✅ Resolved namespace collision between GIK solver and Hybrid A* planner---

- ✅ Implemented Pimpl pattern with wrapper functions

- ✅ Clean build with no conflicts## 📋 Execution Checklist (Track Your Progress)

- ✅ Runtime tested and validated

Copy this to a text file and check off as you go:

See: [NAMESPACE_CONFLICT_RESOLVED.md](NAMESPACE_CONFLICT_RESOLVED.md)

```

### ROS2 Integration[ ] Phase 1: MATLAB Validation (Windows)

- ✅ Complete ROS2 Humble integration    [ ] Read START_HERE.md

- ✅ Parameter server configuration    [ ] Run RUN_VALIDATION.m

- ✅ Topic-based control interface    [ ] All 7 tests pass

- ✅ Service-based goal setting

[ ] Phase 2: Code Generation (Windows)

See: [ROS2_INTEGRATION_COMPLETE.md](ROS2_INTEGRATION_COMPLETE.md)    [ ] Run RUN_CODEGEN.m

    [ ] Wait 10-15 minutes

---    [ ] gik9dof_deployment_<timestamp>.zip created



## 🚀 Building and Running[ ] Phase 3: WSL Validation (Intermediate)

    [ ] Read WSL_QUICK_REFERENCE.md

### Prerequisites    [ ] Copy ZIP to WSL

- Ubuntu 22.04    [ ] Build: colcon build --packages-select gik9dof_msgs

- ROS2 Humble    [ ] Build: colcon build --packages-select gik9dof_solver

- MATLAB R2024a or later (for code generation)    [ ] Test: ros2 launch gik9dof_solver test_solver.launch.py

- C++17 compiler    [ ] Test: ros2 run gik9dof_solver test_mock_inputs.py

- Eigen3    [ ] Verify: status=1, solve_time<50ms, iterations<100

    

### Build[ ] Phase 4: AGX Orin Deployment

```bash    [ ] Run deploy_to_orin.ps1 <orin-ip>

cd ros2    [ ] SSH to AGX Orin

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    [ ] Build on Orin (same commands as WSL)

source install/setup.bash    [ ] Test on Orin (expect faster performance!)

```    

[ ] Phase 5: Robot Integration

### Run    [ ] Connect to real /hdas/feedback_arm_left

```bash    [ ] Connect to real /odom_wheel

ros2 run gik9dof_solver gik9dof_solver_node \    [ ] Connect to real /gik9dof/target_trajectory

  --ros-args --params-file gik9dof_solver/config/gik9dof_solver.yaml    [ ] Verify 10 Hz control loop

```    [ ] Tune parameters

    [ ] Ship it! 🚀

### Test```

See [ORIN_TESTING_GUIDE.md](ORIN_TESTING_GUIDE.md) for comprehensive testing procedures.

---

---

## 🗂️ File Structure Reference

## 📚 Key Documentation

```

### Essential ReadinggikWBC9DOF/

- **[START_HERE.md](START_HERE.md)** - First-time setup guide│

- **[QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md)** - Development quick ref├── README.md                          ← YOU ARE HERE (Navigation Hub)

- **[NAMESPACE_FIX_TEST_RESULTS.md](NAMESPACE_FIX_TEST_RESULTS.md)** - Latest test results│

├── 🚀 EXECUTION FILES (Run These)

### Technical Details│   ├── RUN_VALIDATION.m               ← Step 1: Run in MATLAB

- **[HYBRID_ASTAR_README.md](HYBRID_ASTAR_README.md)** - Path planning algorithm│   ├── RUN_CODEGEN.m                  ← Step 2: Run in MATLAB

- **[PUREPURSUIT_QUICKSTART.md](PUREPURSUIT_QUICKSTART.md)** - Velocity controller│   └── deploy_to_orin.ps1             ← Step 4: Run in PowerShell

- **[GIK_ENHANCEMENTS_QUICKREF.md](GIK_ENHANCEMENTS_QUICKREF.md)** - Solver features│

- **[STAGE_B_FIXES.md](STAGE_B_FIXES.md)** - Stage B implementation├── 📖 PRIMARY DOCUMENTATION (Must Read)

│   ├── START_HERE.md                  ← Main execution guide (READ FIRST!)

### Deployment│   ├── QUICK_START.md                 ← 15-minute overview

- **[BUILD_ON_ORIN.md](BUILD_ON_ORIN.md)** - AGX Orin build instructions│   ├── VALIDATION_WORKFLOW.md         ← 3-tier workflow diagram

- **[ORIN_DEPLOYMENT_PATH.md](ORIN_DEPLOYMENT_PATH.md)** - Deployment workflow│   └── CODEGENCC45_PROJECT_PLAN.md    ← Master architecture

- **[DEPLOY_NOW.md](DEPLOY_NOW.md)** - Quick deployment steps│

├── 🖥️ WSL VALIDATION DOCS (WSL Ubuntu 22.04)

### Design Documents│   ├── WSL_INTEGRATION_SUMMARY.md     ← Why WSL matters

- **[HYBRID_ASTAR_DESIGN.md](HYBRID_ASTAR_DESIGN.md)** - Planner architecture│   ├── WSL_VALIDATION_GUIDE.md        ← Complete WSL guide

- **[PUREPURSUIT_DESIGN.md](PUREPURSUIT_DESIGN.md)** - Controller design│   └── WSL_QUICK_REFERENCE.md         ← Copy/paste commands

- **[STATE_MACHINE_INTEGRATION_STATUS.md](STATE_MACHINE_INTEGRATION_STATUS.md)** - Control flow│

├── 📚 TECHNICAL REFERENCE

---│   ├── REQUIREMENTS_CONFIRMED.md      ← All design decisions

│   ├── MATLAB_CODEGEN_ANALYSIS.md     ← Codegen scope

## 🔧 Development│   ├── ROS2_INTEGRATION_GUIDE.md      ← ROS2 details

│   ├── ORIGIN_MAIN_MERGE_ANALYSIS.md  ← Merge impact analysis

### Code Generation (MATLAB)│   └── FAST_TRACK_2DAY.md             ← Hour-by-hour plan

```matlab│

% Generate all components├── 📜 HISTORICAL (Optional)

run('RUN_CODEGEN.m')│   ├── IMPLEMENTATION_ROADMAP.md      ← Original 6-8 week plan

│   ├── WEEK1_IMPLEMENTATION_GUIDE.md  ← Week 1 plan

% Or generate individually:│   └── README_CODEGENCC45.md          ← Alternative intro

run('generate_code_arm64.m')        % For ARM64│

run('generate_code_planner_arm64.m') % Hybrid A* for ARM64├── 🔧 MATLAB CODE

```│   └── matlab/

│       └── +gik9dof/+codegen_inuse/

### Validation│           ├── buildRobotForCodegen.m        (169 lines)

```matlab│           ├── solveGIKStepRealtime.m        (52 lines)

% Run validation suite│           ├── solveGIKStepWrapper.m         (38 lines)

run('RUN_VALIDATION.m')│           ├── generateCodeARM64.m           (197 lines)

```│           └── validate_robot_builder.m      (150 lines)

│

### Debugging├── 🤖 ROS2 PACKAGES

- Check `codegen_debug.log` for build issues│   └── ros2/

- Use `test_codegen_debug.m` for isolated testing│       ├── gik9dof_msgs/

│       │   └── msg/

---│       │       ├── EndEffectorTrajectory.msg

│       │       └── SolverDiagnostics.msg

## 🐛 Known Issues & Solutions│       └── gik9dof_solver/

│           ├── src/gik9dof_solver_node.cpp   (300+ lines)

### Namespace Conflicts (RESOLVED ✅)│           ├── launch/test_solver.launch.py

**Problem**: GIK solver and Hybrid A* planner both define `gik9dof::struct0_T`  │           └── scripts/test_mock_inputs.py    (150 lines)

**Solution**: Pimpl pattern with factory functions and wrapper functions  │

**Status**: ✅ Fixed and tested└── 📦 ASSETS

    ├── mobile_manipulator_PPR_base_corrected.urdf

See: [NAMESPACE_CONFLICT_RESOLVED.md](NAMESPACE_CONFLICT_RESOLVED.md)    └── meshes/

```

### ARM64 SSE Intrinsics (RESOLVED ✅)

**Problem**: x86 SSE intrinsics not available on ARM64  ---

**Solution**: Compatibility layer with sse2neon  

**Status**: ✅ Working on AGX Orin## 💡 Pro Tips



See: [BUILD_ON_ORIN.md](BUILD_ON_ORIN.md)### **Bookmark These Files**

1. **[`README.md`](README.md)** - Always start here when lost

---2. **[`START_HERE.md`](START_HERE.md)** - Your execution playbook

3. **[WSL Quick Reference](docs/deployment/wsl/WSL_QUICK_REFERENCE.md)** - Keep open during testing

## 📊 Project Status4. **[CODEGEN Structure](docs/technical/CODEGEN_STRUCTURE.md)** - Understanding the codebase



| Component | Status | Notes |### **Open These Side-by-Side**

|-----------|--------|-------|- Left monitor: VSCode with MATLAB/C++ code

| GIK Solver (9-DOF) | ✅ Complete | MATLAB Coder generated |- Right monitor: Relevant guide in browser (for copy/paste)

| Hybrid A* Planner | ✅ Complete | Namespace conflict resolved |

| Pure Pursuit Controller | ✅ Complete | Bidirectional support |---

| ROS2 Integration | ✅ Complete | Humble, parameter server |

| Stage A (Arm Ramp) | ✅ Complete | Tested |## 🆘 When Things Go Wrong

| Stage B (Chassis Plan) | ✅ Complete | Wrapper functions tested |

| Stage C (Whole-Body) | ✅ Complete | Full 9-DOF control |### **Error During MATLAB Validation**

| ARM64 Build | ✅ Complete | AGX Orin validated |→ Check [`START_HERE.md`](START_HERE.md) → Troubleshooting section

| Runtime Testing | ✅ Complete | Node runs successfully |

### **Error During Code Generation**

---→ Check `codegen/html/report.mldatx` in MATLAB  

→ See [Fast Track 2-Day](docs/planning/FAST_TRACK_2DAY.md) → Day 1 Hour 2

## 🤝 Contributing

### **Error During WSL Build**

When adding new features:→ Check [WSL Validation Guide](docs/deployment/wsl/WSL_VALIDATION_GUIDE.md) → Common Issues  

1. Document in appropriate `docs/` subdirectory→ Most common: Missing dependencies (libeigen3-dev, libomp-dev)

2. Update this README if structure changes

3. Add tests to `validation/`### **Error During AGX Orin Build**

4. Follow existing naming conventions→ Should be same as WSL! Check same issues as WSL  

→ If Orin-specific: Check [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) → Gate 4

---

### **Solver Not Converging**

## 📝 Archive→ Check [Fast Track 2-Day](docs/planning/FAST_TRACK_2DAY.md) → Day 2 Hour 4  

→ Tune constraints, increase max iterations

Old session notes and intermediate documentation have been moved to:

- `docs/archive/sessions/` - Development session summaries---

- `docs/archive/namespace-conflict/` - Namespace fix work-in-progress docs

## 📊 Project Status

These are kept for historical reference but are not part of active development.

| Component | Status | Files | Details |

---|-----------|--------|-------|---------|

| **MATLAB Solver** | ✅ Complete | 203 C++ files | ARM64 optimized, validated |

## 📞 Quick Reference| **ROS2 Integration** | ✅ Complete | 4 topics | Builds in 54.8s, 0 errors |

| **Validation Framework** | ✅ Complete | 12 test files | CLI-based testing works |

**Current Branch**: `codegencc45`  | **Documentation** | ✅ Organized | 26 MD files | Reorganized into logical folders |

**ROS2 Version**: Humble  | **WSL Support** | ✅ Ready | 3 guides | Ubuntu 22.04, tested |

**Target Platform**: NVIDIA AGX Orin (ARM64)  | **AGX Orin Deployment** | ⏳ Ready | Scripts ready | Awaiting hardware |

**Build System**: colcon (ROS2)  

**MATLAB Version**: R2024a+**Current Phase**: Post-Integration Validation  

**Next Step**: Deploy to AGX Orin or create C++ test node

**Need Help?**

- Check [START_HERE.md](START_HERE.md) first---

- Review [QUICK_START_NEXT_SESSION.md](QUICK_START_NEXT_SESSION.md) for common tasks

- See `docs/archive/` for historical context## 📚 Documentation Organization



---All documentation has been reorganized (October 6, 2025):



**Last Updated**: October 7, 2025  - **Root**: Only `README.md` and `START_HERE.md`

**Status**: Production Ready ✅- **docs/guides/**: User-facing guides (4 files)

- **docs/deployment/**: Deployment guides + WSL subfolder (5 files)
- **docs/technical/**: Technical reference (4 files)
- **docs/planning/**: Project planning (6 files)
- **docs/archive/**: Historical documents (5 files)

See [Reorganization Plan](docs/REORGANIZATION_PLAN.md) for details.

---

## 🎓 Learning Path

**If you're new to this project**, read in this order:

1. **5 minutes**: [`README.md`](README.md) (you are here) - Navigation
2. **15 minutes**: [Quick Start](docs/guides/QUICK_START.md) - Overview
3. **10 minutes**: [Validation Workflow](docs/guides/VALIDATION_WORKFLOW.md) - Testing strategy
4. **20 minutes**: [`START_HERE.md`](START_HERE.md) - Execution guide
5. **Ready to start!** → Follow deployment guides

**Total onboarding time**: ~50 minutes

---

## 🔄 Recent Updates

**Last Updated**: October 6, 2025  
**Branch**: `codegencc45`  

**Latest Changes**:
- ✅ **Complete MATLAB-to-ROS2 integration** (218 files committed)
- ✅ **Documentation reorganization** (26 MD files → organized structure)
- ✅ **Validation framework** (automated + manual testing)
- ✅ **Clean .gitignore** (build artifacts properly ignored)
- ✅ **CODEGEN structure documented** (understand directory layout)

**Recent Commits**:
```
130eb83 chore: Ignore clang-format file in codegen
5b7ea91 chore: Add comprehensive .gitignore for build artifacts
5574edf feat: Complete MATLAB-to-ROS2 integration with validation framework
```

---

## 📞 Need Help?

1. **Documentation Navigation**: Always come back to this [`README.md`](README.md)
2. **Quick Questions**: Check [Quick Start](docs/guides/QUICK_START.md)
3. **Execution Issues**: Check [`START_HERE.md`](START_HERE.md) → Troubleshooting
4. **WSL Issues**: Check [WSL Validation Guide](docs/deployment/wsl/WSL_VALIDATION_GUIDE.md)
5. **Architecture Questions**: Check [Project Plan](docs/planning/CODEGENCC45_PROJECT_PLAN.md)
6. **Code Organization**: Check [CODEGEN Structure](docs/technical/CODEGEN_STRUCTURE.md)

---

## 🏁 Ready to Start?

**Your next step:**

1. Read [`START_HERE.md`](START_HERE.md) (5 minutes)
2. Check [Orin Next Steps](docs/deployment/ORIN_NEXT_STEPS.md) for deployment
3. Or continue development with the organized documentation

Good luck! 🚀

---

**Navigation**: [TOP](#9-dof-whole-body-ik-solver-for-mobile-manipulator) | [START_HERE.md](START_HERE.md) | [Quick Start](docs/guides/QUICK_START.md)
