# Documentation Index

Welcome to the gikWBC9DOF documentation! This directory contains all project documentation organized by category.

---

## 📂 Directory Structure

### [getting-started/](getting-started/)
**Start here if you're new to the project**
- `START_HERE.md` - Project overview and setup guide
- `PLANNER_QUICK_START.md` - Quick start guide for path planning
- `REAL_DATA_TEST_GUIDE.md` - Guide for testing with real trajectory data

### [technical/](technical/)
**Technical documentation and architecture**

#### [architecture/](technical/architecture/)
System architecture and design decisions
- `INTEGRATION_STRATEGY.md` - Velocity smoothing integration strategy
- `CLASS_BASED_ARCHITECTURE_FIX.md` - Class-based architecture refactoring
- `DOWNSTREAM_CONTROL_ANALYSIS.md` - Control flow analysis
- `STAGED_CONTROL_ARCHITECTURE.md` - Three-stage control architecture (A→B→C)

#### [smoothing/](technical/smoothing/)
Trajectory and velocity smoothing documentation
- `VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md` - ⭐ **Complete velocity smoothing integration**
- `VELOCITY_SMOOTHING_QUICK_REF.md` - Quick reference for velocity smoothing
- `VELOCITY_SMOOTHING_ARCHITECTURE.md` - Velocity smoothing architecture details
- `SMOOTHING_COMPARISON.md` - Waypoint vs velocity-based smoothing comparison
- `SMOOTHING_STRATEGY_EXPLAINED.md` - Smoothing algorithm explanation
- `SMOOTHING_POINT_COUNT.md` - Point count analysis for smoothing
- `TRAJECTORY_SMOOTHING_PLAN.md` - Trajectory smoothing implementation plan
- `JERK_LIMIT_ANALYSIS.md` - Jerk limiting analysis and parameters
- `ROLLING_WINDOW_STRATEGY.md` - Rolling window smoothing strategy

#### [planning/](technical/planning/)
Path planning and navigation
- `HYBRID_ASTAR_COMPLETE.md` - Hybrid A* implementation complete
- `HYBRID_ASTAR_SESSION_SUMMARY.md` - Hybrid A* development session summary
- `PHASE_2_IMPLEMENTATION_PLAN.md` - Phase 2 features implementation plan

#### [codegen/](technical/codegen/)
MATLAB Coder and code generation
- `CODEGEN_AUDIT.md` - Code generation audit and review
- `CONFIG_FILES_SUMMARY.md` - Configuration files summary
- `WRAPPER_VERSION_MISMATCH_FIX.md` - Wrapper version compatibility fix

### [fixes/](fixes/)
**Bug fixes and problem resolutions**
- `ORIN_BUILD_FIX.md` - Jetson Orin build issues and solutions
- `LIBCCD_HEADERS_FIX.md` - LibCCD header compatibility fix
- `MISSING_SOURCE_FILES_FIX.md` - Missing source files resolution
- `BACKUP_FILES_INCOMPATIBILITY.md` - Backup file compatibility issues
- `PLANNER_NAMESPACE_FIX.md` - Planner namespace conflict resolution
- `WRAPPER_CONFUSION_RESOLVED.md` - Wrapper function confusion resolution
- `GIK_CODE_UPDATE.md` - GIK code updates and changes
- `REALTIME_MAXTIME_UPDATE.md` - Real-time maximum time parameter update

### [sessions/](sessions/)
**Development session notes and progress**
- `NEXT_SESSION_VELOCITY_SMOOTHING.md` - ⭐ **Current session - Velocity smoothing complete**
- `NEXT_SESSION_START_HERE.md` - Previous session start point
- `SESSION_COMPLETE.md` - Completed session summary
- `TRAJECTORY_SMOOTHING_SESSION_SUMMARY.md` - Trajectory smoothing session
- `SESSION_SUMMARY_OCT07_2025.md` - October 7 session summary
- `RUN_TEST_NOW.md` - Testing instructions

### [testing/](testing/)
**Testing guides and validation**
- `REAL_TRAJECTORY_TEST.md` - Real trajectory testing procedures
- `WSL_BUILD_VERIFICATION.md` - WSL build verification steps
- `VALIDATION_WORKFLOW.md` - Validation workflow and procedures

### [deployment/](deployment/)
**Deployment guides for target platforms**
- `ORIN_MATLAB_INTEGRATION.md` - Jetson Orin MATLAB integration guide

### [organization/](organization/)
**Project organization and maintenance**
- `FILE_ORGANIZATION.md` - File organization guidelines
- `SYNC_STATUS_AFTER_CLEANUP.md` - Git sync status after cleanup
- `PLANNER_SYNC_STATUS.md` - Planner sync status
- `PLANNER_REGEN_PLAN.md` - Planner regeneration plan
- `REORGANIZATION_PLAN.md` - Project reorganization plan

---

## 🚀 Quick Links

### For New Users
1. Start with [`getting-started/START_HERE.md`](getting-started/START_HERE.md)
2. Follow [`getting-started/PLANNER_QUICK_START.md`](getting-started/PLANNER_QUICK_START.md)
3. Test with [`getting-started/REAL_DATA_TEST_GUIDE.md`](getting-started/REAL_DATA_TEST_GUIDE.md)

### For Developers
- **Current Work**: [`sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md`](sessions/NEXT_SESSION_VELOCITY_SMOOTHING.md)
- **Architecture**: [`technical/architecture/STAGED_CONTROL_ARCHITECTURE.md`](technical/architecture/STAGED_CONTROL_ARCHITECTURE.md)
- **Smoothing**: [`technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md`](technical/smoothing/VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md)

### For Troubleshooting
- Check [`fixes/`](fixes/) for known issues and solutions
- Build issues? See [`fixes/ORIN_BUILD_FIX.md`](fixes/ORIN_BUILD_FIX.md)
- Header issues? See [`fixes/LIBCCD_HEADERS_FIX.md`](fixes/LIBCCD_HEADERS_FIX.md)

---

## 📊 Documentation Statistics

- **Total Documents**: ~50 markdown files
- **Getting Started Guides**: 3
- **Technical Docs**: 20 (architecture, smoothing, planning, codegen)
- **Fix Documentation**: 8
- **Session Notes**: 6
- **Testing Guides**: 3
- **Deployment Guides**: 1
- **Organization Docs**: 5

---

## 📝 Documentation Standards

### File Naming
- Use `SCREAMING_SNAKE_CASE` for consistency with existing files
- Descriptive names (e.g., `VELOCITY_SMOOTHING_INTEGRATION_COMPLETE.md`)
- Date-based for sessions (e.g., `SESSION_SUMMARY_OCT07_2025.md`)

### Structure
- Clear title and date at top
- Status/summary section
- Table of contents for long docs
- Code examples with syntax highlighting
- Links to related documentation

### Location Guide
- **How-to guides** → `getting-started/`
- **Architecture/design** → `technical/architecture/`
- **Algorithm details** → Relevant `technical/` subdirectory
- **Bug fixes** → `fixes/`
- **Session notes** → `sessions/`
- **Test procedures** → `testing/`
- **Deployment** → `deployment/`

---

**Last Updated:** October 10, 2025  
**Maintained By:** Development Team

