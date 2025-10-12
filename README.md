# gikWBC9DOF - Whole-Body Mobile Manipulation

Generalized Inverse Kinematics (GIK) Whole-Body Controller for a 9-DOF mobile manipulator (3-DOF differential drive base + 6-DOF arm).

## 📁 Project Structure

```
gikWBC9DOF/
├── README.md                          # This file
├── projectDiagnosis.md               # ⭐ MAIN REFERENCE - Comprehensive project analysis
├── HANDOVER.md                       # Project handover documentation
├── guideline.md                      # Development guidelines
├── diary.md                          # Development diary/notes
│
├── matlab/                           # Main MATLAB source code
│   └── +gik9dof/                    # GIK package namespace
│       ├── +control/                # Controllers (pure pursuit, heading)
│       ├── +planning/               # Path planning (Hybrid A*, RRT)
│       ├── +animation/              # Animation and visualization
│       ├── +config/                 # Configuration management
│       └── *.m                      # Core functions
│
├── config/                           # Configuration files
│   ├── pipeline_profiles.yaml       # ⭐ Pipeline configuration profiles
│   └── chassis_profiles.yaml        # Chassis controller parameters
│
├── scripts/                          # Utility scripts
│   ├── run_*.m                      # Execution scripts
│   ├── generation/                  # Animation generation scripts
│   │   ├── generate_*.m
│   │   └── regenerate_*.m
│   └── analysis/                    # Analysis and debugging tools
│       ├── analyze_*.m
│       ├── compare_*.m
│       ├── verify_*.m
│       └── view_*.m
│
├── tests/                            # Test scripts
│   └── test_*.m                     # All test scripts
│
├── docs/                             # Documentation
│   ├── PROJECT_OVERVIEW.md          # High-level project overview
│   ├── GIK_SETUP_OVERVIEW.md        # GIK solver setup guide
│   ├── ANIMATION_GENERATION_GUIDE.md # Animation workflow
│   ├── SIMULATION_WORKFLOW_GUIDE.md # Simulation guide
│   ├── UNIFIED_CONFIG_QUICK_REF.md  # Config system reference
│   └── staged_path_findings.md      # Staged path analysis
│
├── archive/                          # Archived files
│   ├── docs/                        # Old documentation (integrated into projectDiagnosis.md)
│   ├── scripts/                     # Debug and temporary scripts
│   └── temp/                        # Log files and temporary data
│
├── refEETrajs/                       # Reference end-effector trajectories (JSON)
├── results/                          # Simulation results and logs
├── meshes/                           # 3D mesh files for visualization
├── *.urdf                           # Robot URDF models
└── *.json                           # Task/trajectory specifications
```

## 🚀 Quick Start

### Prerequisites
- MATLAB R2023a or later
- Robotics System Toolbox
- Optimization Toolbox

### Running Simulations

```matlab
% 1. Fresh simulation with animation
cd /path/to/gikWBC9DOF
scripts/run_fresh_sim_with_animation

% 2. Run with specific pipeline profile
profile = gik9dof.config.loadPipelineProfile('ppForIk_tuned');
log = gik9dof.trackReferenceTrajectory('refEETrajs/1_pull_world_scaled.json', ...
                                        'PipelineConfig', profile);

% 3. Generate animation from existing log
gik9dof.animateStagedWithHelper(log);
```

### Configuration Profiles

Located in `config/pipeline_profiles.yaml`:
- **`ppForIk_default`** - Standard 3-pass IK pipeline
- **`ppForIk_tuned`** - Optimized parameters (recommended)
- **`pureIk_default`** - Holistic full-body IK
- **`pureHyb_default`** - Pure pursuit with Hybrid A*

## 📚 Key Documentation

| Document | Purpose |
|----------|---------|
| **projectDiagnosis.md** | ⭐ Main reference - comprehensive analysis, architecture, function relationships |
| **HANDOVER.md** | Project handover, context, and key decisions |
| **docs/UNIFIED_CONFIG_QUICK_REF.md** | Configuration system quick reference |
| **docs/SIMULATION_WORKFLOW_GUIDE.md** | Step-by-step simulation workflow |
| **docs/ANIMATION_GENERATION_GUIDE.md** | Animation generation guide |

## 🔍 Key Features

### Three-Pass Architecture (ppForIk Mode)
1. **Pass 1**: Reference IK - Full-body IK without chassis constraints
2. **Pass 2**: Chassis Simulation - Pure pursuit controller with dynamics
3. **Pass 3**: Final IK - Arm tracking with locked base trajectory

### Staged Execution
- **Stage A**: Arm ramp-up (base locked)
- **Stage B**: Base navigation (arm locked or GIK in loop)
- **Stage C**: Coordinated tracking (full-body or 3-pass)

### Controller Modes
- **gikInLoop**: GIK solver in control loop (Stage B)
- **pureHyb**: Pure pursuit + Hybrid A* planning
- **ppForIk**: 3-pass pure pursuit + IK
- **pureIk**: Holistic full-body IK

## 🧪 Testing

```matlab
% Run all tests
cd tests
test_pipeline_profiles         % Test unified config system
test_animation_fix            % Test animation data source fix
test_stagec_path_fix          % Test Stage C path synchronization

% Analysis tools
cd ../scripts/analysis
verify_fix                     % Verify animation fix
analyze_all_tests             % Comprehensive test analysis
compare_test_configs          % Compare configuration profiles
```

## 📊 Recent Improvements

### October 12, 2025
- ✅ **Animation Data Source Fix**: Red dot now shows Pass 3 actual trajectory
- ✅ **Function Relationship Analysis**: Documented runTrajectoryControl vs simulateChassisExecution
- ✅ **Project Organization**: Cleaned up root folder structure

### October 11, 2025
- ✅ **Unified Configuration System**: Migrated to pipeline_profiles.yaml
- ✅ **Chassis Controller Integration**: Unified chassis parameter management
- ✅ **Documentation Consolidation**: Integrated all analysis into projectDiagnosis.md

## 🐛 Known Issues

See `projectDiagnosis.md` Section 11 (Recent Bug Fixes) for detailed issue tracking.

## 📝 Development Notes

- **Main reference**: `projectDiagnosis.md` contains comprehensive analysis
- **Development diary**: `diary.md` tracks daily progress
- **Guidelines**: `guideline.md` provides coding standards
- **Archive**: Old documentation and debug scripts moved to `archive/`

## 📧 Contact

For questions or issues, refer to `HANDOVER.md` for project context and key decisions.

---

**Last Updated**: October 12, 2025
