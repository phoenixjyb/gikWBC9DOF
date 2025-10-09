# Workspace Organization Plan

**Date:** October 10, 2025  
**Goal:** Clean up root directory and organize files logically

---

## 📊 Current Issues

### Root Directory Clutter
- **40+ markdown files** scattered in root
- **10+ script files** (.sh, .ps1, .bat, .m) in root
- **2 URDF files** in root
- Mix of documentation, fixes, guides, and session notes

### Consequences
- Hard to find relevant documentation
- Unclear which files are current vs historical
- No clear navigation structure

---

## 🎯 Proposed Structure

```
gikWBC9DOF/
├── README.md (updated with navigation)
├── .gitignore
│
├── docs/
│   ├── README.md (index of all documentation)
│   │
│   ├── getting-started/
│   │   ├── START_HERE.md
│   │   ├── PLANNER_QUICK_START.md
│   │   └── REAL_DATA_TEST_GUIDE.md
│   │
│   ├── technical/
│   │   ├── architecture/
│   │   │   ├── INTEGRATION_STRATEGY.md
│   │   │   ├── CLASS_BASED_ARCHITECTURE_FIX.md
│   │   │   ├── STAGED_CONTROL_ARCHITECTURE.md
│   │   │   └── DOWNSTREAM_CONTROL_ANALYSIS.md
│   │   │
│   │   ├── smoothing/
│   │   │   ├── SMOOTHING_COMPARISON.md
│   │   │   ├── SMOOTHING_STRATEGY_EXPLAINED.md
│   │   │   ├── SMOOTHING_POINT_COUNT.md
│   │   │   ├── TRAJECTORY_SMOOTHING_PLAN.md
│   │   │   ├── VELOCITY_SMOOTHING_ARCHITECTURE.md
│   │   │   ├── VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md
│   │   │   ├── VELOCITY_SMOOTHING_QUICK_REF.md
│   │   │   ├── JERK_LIMIT_ANALYSIS.md
│   │   │   └── ROLLING_WINDOW_STRATEGY.md
│   │   │
│   │   ├── planning/
│   │   │   ├── HYBRID_ASTAR_COMPLETE.md
│   │   │   ├── HYBRID_ASTAR_SESSION_SUMMARY.md
│   │   │   └── PHASE_2_IMPLEMENTATION_PLAN.md
│   │   │
│   │   └── codegen/
│   │       ├── CODEGEN_AUDIT.md
│   │       ├── CONFIG_FILES_SUMMARY.md
│   │       └── WRAPPER_VERSION_MISMATCH_FIX.md
│   │
│   ├── fixes/
│   │   ├── ORIN_BUILD_FIX.md
│   │   ├── LIBCCD_HEADERS_FIX.md
│   │   ├── MISSING_SOURCE_FILES_FIX.md
│   │   ├── BACKUP_FILES_INCOMPATIBILITY.md
│   │   ├── PLANNER_NAMESPACE_FIX.md
│   │   ├── WRAPPER_CONFUSION_RESOLVED.md
│   │   ├── GIK_CODE_UPDATE.md
│   │   └── REALTIME_MAXTIME_UPDATE.md
│   │
│   ├── sessions/
│   │   ├── NEXT_SESSION_VELOCITY_SMOOTHING.md (current)
│   │   ├── NEXT_SESSION_START_HERE.md
│   │   ├── SESSION_COMPLETE.md
│   │   ├── TRAJECTORY_SMOOTHING_SESSION_SUMMARY.md
│   │   ├── SESSION_SUMMARY_OCT07_2025.md
│   │   ├── RUN_TEST_NOW.md
│   │   └── archive/
│   │
│   ├── testing/
│   │   ├── REAL_TRAJECTORY_TEST.md
│   │   ├── WSL_BUILD_VERIFICATION.md
│   │   └── VALIDATION_WORKFLOW.md
│   │
│   ├── deployment/
│   │   ├── ORIN_MATLAB_INTEGRATION.md
│   │   └── README.md
│   │
│   └── organization/
│       ├── FILE_ORGANIZATION.md
│       ├── SYNC_STATUS_AFTER_CLEANUP.md
│       ├── PLANNER_SYNC_STATUS.md
│       ├── PLANNER_REGEN_PLAN.md
│       └── REORGANIZATION_PLAN.md
│
├── scripts/
│   ├── README.md (index of scripts)
│   │
│   ├── codegen/
│   │   ├── generate_code_velocity_smoothing.m
│   │   ├── run_planner_codegen.ps1
│   │   ├── run_planner_codegen_wsl.m
│   │   ├── run_planner_codegen_wsl.sh
│   │   ├── RUN_PLANNER_WSL.sh
│   │   ├── run_trajectory_smoothing_codegen.sh
│   │   ├── run_velocity_smoothing_codegen.sh
│   │   ├── RUN_ALL_PLANNER_REGEN.ps1
│   │   └── COPY_PASTE_PLANNER_CODEGEN.sh
│   │
│   ├── testing/
│   │   ├── run_planner_verify.ps1
│   │   ├── test_matlab_batch.sh
│   │   └── run_real_trajectory_test.bat
│   │
│   └── deployment/
│       └── run_planner_copy_to_ros2.ps1
│
├── models/
│   ├── mobile_manipulator_PPR_base_corrected.urdf
│   └── mobile_manipulator_PPR_base_corrected_sltRdcd.urdf
│
├── matlab/
│   └── (existing structure)
│
├── ros2/
│   └── (existing structure)
│
├── codegen/
│   └── (existing structure)
│
├── data/
│   └── (existing structure)
│
├── meshes/
│   └── (existing structure)
│
├── validation/
│   └── (existing structure)
│
├── test_cpp/
│   └── (existing structure)
│
├── logs/
│   ├── .gitkeep
│   └── WSL_COMMANDS.txt
│
└── deployments/
    └── (existing structure)
```

---

## 📋 File Categorization

### Documentation Files (35 total)

#### Getting Started (3)
- START_HERE.md
- PLANNER_QUICK_START.md
- REAL_DATA_TEST_GUIDE.md

#### Technical - Architecture (4)
- INTEGRATION_STRATEGY.md
- CLASS_BASED_ARCHITECTURE_FIX.md
- DOWNSTREAM_CONTROL_ANALYSIS.md
- docs/technical/STAGED_CONTROL_ARCHITECTURE.md (already in place)

#### Technical - Smoothing (9)
- SMOOTHING_COMPARISON.md
- SMOOTHING_STRATEGY_EXPLAINED.md
- SMOOTHING_POINT_COUNT.md
- TRAJECTORY_SMOOTHING_PLAN.md
- VELOCITY_SMOOTHING_ARCHITECTURE.md
- VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md
- VELOCITY_SMOOTHING_QUICK_REF.md
- JERK_LIMIT_ANALYSIS.md
- ROLLING_WINDOW_STRATEGY.md

#### Technical - Planning (3)
- docs/planning/HYBRID_ASTAR_COMPLETE.md (already in place)
- docs/planning/HYBRID_ASTAR_SESSION_SUMMARY.md (already in place)
- PHASE_2_IMPLEMENTATION_PLAN.md

#### Technical - Codegen (3)
- CODEGEN_AUDIT.md
- CONFIG_FILES_SUMMARY.md
- WRAPPER_VERSION_MISMATCH_FIX.md

#### Fixes (8)
- ORIN_BUILD_FIX.md
- LIBCCD_HEADERS_FIX.md
- MISSING_SOURCE_FILES_FIX.md
- BACKUP_FILES_INCOMPATIBILITY.md
- PLANNER_NAMESPACE_FIX.md
- WRAPPER_CONFUSION_RESOLVED.md
- GIK_CODE_UPDATE.md
- REALTIME_MAXTIME_UPDATE.md

#### Sessions (5)
- NEXT_SESSION_VELOCITY_SMOOTHING.md (CURRENT)
- NEXT_SESSION_START_HERE.md
- SESSION_COMPLETE.md
- TRAJECTORY_SMOOTHING_SESSION_SUMMARY.md
- RUN_TEST_NOW.md
- docs/sessions/SESSION_SUMMARY_OCT07_2025.md (already in place)

#### Testing (3)
- REAL_TRAJECTORY_TEST.md
- WSL_BUILD_VERIFICATION.md
- docs/technical/VALIDATION_WORKFLOW.md (already in place)

#### Deployment (1)
- docs/deployment/ORIN_MATLAB_INTEGRATION.md (already in place)

#### Organization (4)
- FILE_ORGANIZATION.md
- SYNC_STATUS_AFTER_CLEANUP.md
- PLANNER_SYNC_STATUS.md
- PLANNER_REGEN_PLAN.md
- docs/REORGANIZATION_PLAN.md (already in place)

### Script Files (12 total)

#### Codegen Scripts (9)
- run_planner_codegen.ps1
- run_planner_codegen_wsl.m
- run_planner_codegen_wsl.sh
- RUN_PLANNER_WSL.sh
- run_trajectory_smoothing_codegen.sh
- run_velocity_smoothing_codegen.sh
- RUN_ALL_PLANNER_REGEN.ps1
- COPY_PASTE_PLANNER_CODEGEN.sh
- scripts/codegen/generate_code_velocity_smoothing.m (already in place)

#### Testing Scripts (3)
- run_planner_verify.ps1
- test_matlab_batch.sh
- run_real_trajectory_test.bat

#### Deployment Scripts (1)
- run_planner_copy_to_ros2.ps1

### Model Files (2)
- mobile_manipulator_PPR_base_corrected.urdf
- mobile_manipulator_PPR_base_corrected_sltRdcd.urdf

### Other Files (1)
- WSL_COMMANDS.txt → logs/

---

## 🚀 Organization Steps

### Step 1: Create New Directory Structure
```powershell
# Create documentation directories
New-Item -ItemType Directory -Force -Path docs/getting-started
New-Item -ItemType Directory -Force -Path docs/technical/architecture
New-Item -ItemType Directory -Force -Path docs/technical/smoothing
New-Item -ItemType Directory -Force -Path docs/technical/planning
New-Item -ItemType Directory -Force -Path docs/technical/codegen
New-Item -ItemType Directory -Force -Path docs/fixes
New-Item -ItemType Directory -Force -Path docs/sessions/archive
New-Item -ItemType Directory -Force -Path docs/testing
New-Item -ItemType Directory -Force -Path docs/organization

# Create script directories
New-Item -ItemType Directory -Force -Path scripts/codegen
New-Item -ItemType Directory -Force -Path scripts/testing
New-Item -ItemType Directory -Force -Path scripts/deployment

# Create models directory
New-Item -ItemType Directory -Force -Path models
```

### Step 2: Move Documentation Files
Git will track the moves automatically.

### Step 3: Move Script Files
Preserve executable permissions where needed.

### Step 4: Move Model Files
Move URDF files to models/ directory.

### Step 5: Update README
Create navigation guide for new structure.

### Step 6: Git Commit
```bash
git add -A
git commit -m "docs: reorganize workspace structure

- Move 35 MD files to docs/ subdirectories
- Move 12 script files to scripts/ subdirectories
- Move 2 URDF files to models/
- Create README files for navigation
- Update main README with new structure"
```

### Step 7: Git Push
```bash
git push origin codegencc45-main
```

---

## ✅ Benefits

1. **Clear Navigation**: Easy to find documentation by category
2. **Better Git History**: Related files grouped together
3. **Reduced Root Clutter**: Only essential files in root
4. **Logical Grouping**: Technical docs, fixes, sessions separated
5. **Future-Proof**: Easy to add new docs in right place

---

## 📝 Notes

- Keep README.md in root (essential entry point)
- Keep .gitignore in root (essential)
- Preserve existing docs/, matlab/, ros2/, etc. subdirectories
- Use git mv to preserve file history
- Create README.md in each docs/ subdirectory for navigation

