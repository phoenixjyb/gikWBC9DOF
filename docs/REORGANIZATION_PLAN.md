# Documentation Reorganization Plan

## 🎯 Objective
Clean up 26 scattered markdown files in root directory and remove unused directories.

## 📁 Proposed Structure

```
Root/
├── README.md                          # Keep - main entry point
├── START_HERE.md                      # Keep - quick start
├── docs/
│   ├── guides/                        # User-facing guides
│   │   ├── ROS2_INTEGRATION_GUIDE.md
│   │   ├── VALIDATION_WORKFLOW.md
│   │   ├── QUICK_START.md (merge or delete if duplicate)
│   │   └── guideline.md
│   ├── deployment/                    # Deployment-specific
│   │   ├── ORIN_NEXT_STEPS.md
│   │   ├── README_CODEGENCC45.md
│   │   └── wsl/
│   │       ├── WSL_INTEGRATION_SUMMARY.md
│   │       ├── WSL_QUICK_REFERENCE.md
│   │       └── WSL_VALIDATION_GUIDE.md
│   ├── technical/                     # Technical reference
│   │   ├── CODEGEN_STRUCTURE.md
│   │   ├── CODEGEN.md
│   │   ├── MATLAB_CODEGEN_ANALYSIS.md
│   │   └── unified_chassis_controller_summary.md
│   ├── planning/                      # Project planning (archive)
│   │   ├── CODEGENCC45_PROJECT_PLAN.md
│   │   ├── IMPLEMENTATION_ROADMAP.md
│   │   ├── IMPLEMENTATION_SUMMARY.md
│   │   ├── WEEK1_IMPLEMENTATION_GUIDE.md
│   │   ├── FAST_TRACK_2DAY.md
│   │   └── REQUIREMENTS_CONFIRMED.md
│   └── archive/                       # Historical/obsolete
│       ├── CONTEXT_HANDOFF.md
│       ├── HOW_TO_USE_CONTEXT_HANDOFF.md
│       ├── PROJECT_OVERVIEW.md
│       ├── diary.md
│       └── ORIGIN_MAIN_MERGE_ANALYSIS.md
```

## 🗑️ Directories to Remove

### 1. `deployment_package/` ❌ REMOVE
- Contains: 1 old zip file (gik_codegen_20251006_170613.zip)
- Size: 0.15 MB
- Reason: Old deployment artifact, not tracked in Git
- Action: Delete entirely, already in .gitignore

### 2. `codegen/html/` ❌ REMOVE (already ignored)
- Contains: MATLAB code generation reports
- Reason: Regeneratable, already ignored
- Action: Confirm in .gitignore

### 3. `.vscode/` ⚠️ KEEP (but gitignored)
- Contains: User-specific IDE settings
- Reason: Useful for development, already ignored
- Action: Keep ignored

## 📊 Current vs Proposed

### Root Directory Files
**Before:** 26 MD files scattered in root  
**After:** 2 MD files in root (README.md, START_HERE.md)

### Organization Benefits
✅ Clear separation: guides / deployment / technical / planning / archive  
✅ Easy to find current documentation  
✅ Historical docs preserved but not cluttering  
✅ WSL docs grouped together  
✅ Deployment docs grouped together  

## 🔄 Migration Actions

1. **Create directory structure**
   ```bash
   mkdir -p docs/guides docs/deployment/wsl docs/technical docs/planning docs/archive
   ```

2. **Move files** (Git mv to preserve history)
   - Guides → docs/guides/
   - Deployment → docs/deployment/
   - WSL → docs/deployment/wsl/
   - Technical → docs/technical/
   - Planning → docs/planning/
   - Archive → docs/archive/

3. **Remove obsolete**
   - Delete deployment_package/
   - Verify .gitignore covers build artifacts

4. **Update cross-references**
   - Update README.md with new paths
   - Update START_HERE.md links

## ✅ Validation

After reorganization:
- [ ] README.md links work
- [ ] START_HERE.md links work
- [ ] Git history preserved (used `git mv`)
- [ ] No broken references
- [ ] Clean `git status`
- [ ] All docs accessible from README

## 🎯 Final Structure Stats

**Directories:**
- docs/ (organized)
- matlab/ (keep)
- meshes/ (keep)
- ros2/ (keep)
- validation/ (keep)
- codegen/ (keep - partially tracked)

**Root MD Files:** 2 (down from 26)
**Organized Docs:** 24 (in logical folders)
**Deleted:** deployment_package/, MD_ANALYSIS.txt
