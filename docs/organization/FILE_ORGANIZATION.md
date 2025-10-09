# Project File Organization

**Last Updated**: October 9, 2025  
**Branch**: `wsl-linux-codegen-maxiter1000`

---

## Directory Structure

```
gikWBC9DOF/
├── README.md                    ← Main project overview
├── START_HERE.md                ← Quick start guide
├── FILE_ORGANIZATION.md         ← This file
│
├── scripts/                     ← All executable scripts
│   ├── codegen/                 ← Code generation scripts
│   │   ├── generate_code_arm64.m
│   │   ├── generate_code_x86_64.m
│   │   ├── generate_code_planner_arm64.m
│   │   ├── RUN_CODEGEN.m
│   │   └── RUN_VALIDATION.m
│   ├── deployment/              ← Deployment automation
│   │   ├── deploy_ros2_to_orin.ps1
│   │   ├── copy_codegen_to_ros2.ps1
│   │   └── deploy_to_orin_complete.ps1
│   ├── testing/                 ← Test scripts
│   │   ├── test_codegen_*.m
│   │   ├── test_solver_arm64.sh
│   │   └── run_pp_codegen.m
│   └── wsl/                     ← WSL-specific scripts
│       ├── run_wsl_codegen_matlab.m
│       └── run_wsl_codegen.sh
│
├── matlab/                      ← MATLAB source code
│   ├── +gik9dof/                ← Main namespace
│   │   ├── +codegen_inuse/      ← Active code generation
│   │   │   ├── solveGIKStepWrapper.m (MaxIterations=1000)
│   │   │   ├── buildRobotForCodegen.m
│   │   │   └── solveGIKStepRealtime.m
│   │   ├── +codegen_obsolete/   ← Legacy (not used)
│   │   └── *.m                  ← Utilities
│   ├── holisticVelocityController.m
│   ├── purePursuitVelocityController.m
│   └── codegen/                 ← Generated code cache
│
├── codegen/                     ← Generated C++ code
│   ├── arm64_realtime/          ← ARM64 for Jetson Orin ✅
│   ├── x86_64_validation_noCollision/ ← WSL validation ✅
│   ├── planner_arm64/           ← Hybrid A* planner
│   └── gik9dof_*_20constraints/ ← Legacy builds
│
├── ros2/                        ← ROS2 deployment code
│   └── gik9dof_solver/
│       ├── src/                 ← C++ node implementation
│       │   ├── gik9dof_solver_node.cpp (1238 lines)
│       │   ├── stage_b_chassis_plan.cpp
│       │   ├── collisioncodegen_stubs.cpp
│       │   ├── generated/       ← MATLAB generated code
│       │   ├── purepursuit/
│       │   └── velocity_controller/
│       ├── include/
│       ├── launch/
│       └── CMakeLists.txt
│
├── validation/                  ← C++ validation tools
│   ├── validate_gik_standalone.cpp
│   ├── build_with_library_wsl.sh
│   ├── gik_test_cases_20.json
│   ├── results_maxtime10s.json
│   └── crossCheckMatVsCpp/      ← MATLAB reference data
│
├── docs/                        ← Documentation
│   ├── CODEGEN_ARCHITECTURE_ANALYSIS.md ← Strategic analysis
│   ├── CODEGEN_STRATEGY_SUMMARY.md      ← Quick reference
│   ├── VALIDATION_RESULTS_ANALYSIS.md   ← Test results
│   ├── CURRENT_STATUS.md
│   ├── QUICKSTART_VALIDATION.md
│   ├── SESSION_SUMMARY.md
│   ├── WSL_CODEGEN_SUCCESS.md
│   ├── REGENERATION_MAXTIME.md
│   └── archive/                 ← Old documentation
│       ├── legacy_docs/         ← Pre-Oct 2025 docs
│       ├── namespace-conflict/
│       └── sessions/
│
├── data/                        ← Test data and configs
│   ├── 1_pull_world_scaled.json
│   └── *.json
│
├── logs/                        ← Build and execution logs
│   ├── codegen_output.log
│   ├── purepursuit_codegen.log
│   └── *.txt
│
├── meshes/                      ← 3D models (URDF)
├── deployments/                 ← Deployment archives
└── test_cpp/                    ← C++ unit tests

```

---

## Key Files by Purpose

### 🚀 Production Code Generation
- **ARM64 (Jetson Orin)**: `scripts/codegen/generate_code_arm64.m`
- **x86-64 (WSL Validation)**: `scripts/wsl/run_wsl_codegen_matlab.m`
- **Planner**: `scripts/codegen/generate_code_planner_arm64.m`

### 🔧 Development & Testing
- **Main Validation**: `scripts/codegen/RUN_VALIDATION.m`
- **C++ Validator**: `validation/validate_gik_standalone.cpp`
- **Test Cases**: `validation/gik_test_cases_20.json`

### 📦 Deployment
- **ROS2 Deploy**: `scripts/deployment/deploy_ros2_to_orin.ps1`
- **Code Copy**: `scripts/deployment/copy_codegen_to_ros2.ps1`

### 📚 Documentation (Read These First!)
1. **`README.md`** - Project overview
2. **`START_HERE.md`** - Quick start
3. **`docs/CODEGEN_STRATEGY_SUMMARY.md`** - Strategic decisions
4. **`docs/VALIDATION_RESULTS_ANALYSIS.md`** - Test results

---

## What's Where?

### I need to...

| Task | Location | File |
|------|----------|------|
| **Generate code for Orin** | `scripts/codegen/` | `generate_code_arm64.m` |
| **Validate in WSL** | `scripts/wsl/` | `run_wsl_codegen_matlab.m` |
| **Deploy to Orin** | `scripts/deployment/` | `deploy_ros2_to_orin.ps1` |
| **Run tests** | `validation/` | `./validate_gik_standalone` |
| **Understand architecture** | `docs/` | `CODEGEN_ARCHITECTURE_ANALYSIS.md` |
| **Check status** | `docs/` | `CURRENT_STATUS.md` |
| **See test results** | `docs/` | `VALIDATION_RESULTS_ANALYSIS.md` |
| **Modify IK solver** | `matlab/+gik9dof/+codegen_inuse/` | `solveGIKStepWrapper.m` |
| **Change ROS2 node** | `ros2/gik9dof_solver/src/` | `gik9dof_solver_node.cpp` |

---

## Archive Policy

### Active Documentation (docs/)
- Current session findings
- Strategic analysis
- Test results
- How-to guides

### Archived Documentation (docs/archive/)
- **`legacy_docs/`** - Pre-October 2025 documentation
- **`namespace-conflict/`** - Namespace resolution history
- **`sessions/`** - Historical session notes

---

## Code Generation Workflow

```
1. Modify MATLAB Source
   matlab/+gik9dof/+codegen_inuse/solveGIKStepWrapper.m

2. Generate C++ Code
   Windows: matlab -batch "cd(pwd); scripts/codegen/generate_code_arm64"
   WSL:     /home/yanbo/MATLAB/R2024a/bin/matlab -batch "scripts/wsl/run_wsl_codegen_matlab"

3. Validate (WSL)
   cd validation
   bash build_with_library_wsl.sh
   ./validate_gik_standalone gik_test_cases_20.json results.json

4. Deploy to Orin
   scripts/deployment/deploy_ros2_to_orin.ps1
```

---

## Recent Changes (October 9, 2025)

### File Organization Cleanup
- ✅ Moved all `.m` scripts to `scripts/codegen/`, `scripts/testing/`, `scripts/wsl/`
- ✅ Moved deployment scripts to `scripts/deployment/`
- ✅ Moved logs to `logs/`
- ✅ Moved data files to `data/`
- ✅ Archived legacy docs to `docs/archive/legacy_docs/`
- ✅ Created strategic analysis documents in `docs/`

### Key Updates
- **MaxIterations**: 50 → 1000 (20x increase)
- **MaxTime**: 0.05s → 10s (validation only)
- **Code Generation**: WSL Linux MATLAB approach established
- **Validation**: 30% pass rate (6/20 tests)
- **Status**: Production-ready for deployment

---

## Git Repository

**Remote**: `https://github.com/phoenixjyb/gikWBC9DOF`  
**Branch**: `wsl-linux-codegen-maxiter1000`  
**Status**: All changes committed and pushed

---

**Maintained by**: GitHub Copilot & Team  
**Last Sync**: October 9, 2025
