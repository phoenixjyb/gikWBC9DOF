# Technical Documentation - GIK9DOF Control System

**Last Updated**: 2025-01-XX

---

## 📖 Documentation Index

### 🎯 **Start Here**

| Document | Description | Audience |
|----------|-------------|----------|
| **[State Machine Quick Reference](STATE_MACHINE_QUICKREF.md)** ⭐ | Quick lookup: How to switch modes and stages | All users |
| **[State Machine Architecture](STATE_MACHINE_ARCHITECTURE.md)** | Complete control system documentation | Developers |
| **[State Machine Diagrams](STATE_MACHINE_DIAGRAMS.md)** | Visual flow diagrams | Visual learners |
| **[Trajectory Completion Quick Ref](TRAJECTORY_COMPLETION_QUICKREF.md)** ⚠️ | **What happens at trajectory end** | All users |
| **[Trajectory Completion Behavior](TRAJECTORY_COMPLETION_BEHAVIOR.md)** | Deep dive: End-of-trajectory behavior analysis | Developers |

---

## 🏗️ System Components

### Hybrid A* Planner (`hybrid-astar/`)

**Purpose**: Grid-based path planning for chassis motion in Stage B

| Document | Description |
|----------|-------------|
| [HYBRID_ASTAR_README.md](hybrid-astar/HYBRID_ASTAR_README.md) | Algorithm overview and usage |
| [HYBRID_ASTAR_DESIGN.md](hybrid-astar/HYBRID_ASTAR_DESIGN.md) | Architecture and implementation |
| [HYBRID_ASTAR_COMPARISON.md](hybrid-astar/HYBRID_ASTAR_COMPARISON.md) | Comparison with other planners |
| [HYBRID_ASTAR_PERCEPTION_INTEGRATION.md](hybrid-astar/HYBRID_ASTAR_PERCEPTION_INTEGRATION.md) | Integrating occupancy grids |

**When to Read**: 
- Configuring Stage B (Submode B1: Pure Hybrid A*)
- Tuning planner parameters
- Debugging path planning issues

---

### Pure Pursuit Controller (`pure-pursuit/`)

**Purpose**: Path-following velocity controller (Mode 2, RECOMMENDED)

| Document | Description |
|----------|-------------|
| [PUREPURSUIT_QUICKSTART.md](pure-pursuit/PUREPURSUIT_QUICKSTART.md) | Quick setup guide |
| [PUREPURSUIT_DESIGN.md](pure-pursuit/PUREPURSUIT_DESIGN.md) | Algorithm and architecture |
| [PUREPURSUIT_BIDIRECTIONAL_COMPLETE.md](pure-pursuit/PUREPURSUIT_BIDIRECTIONAL_COMPLETE.md) | Bidirectional support details |
| [PUREPURSUIT_INTEGRATION_COMPLETE.md](pure-pursuit/PUREPURSUIT_INTEGRATION_COMPLETE.md) | Integration with control system |
| [PUREPURSUIT_DEPENDENCY_ANALYSIS.md](pure-pursuit/PUREPURSUIT_DEPENDENCY_ANALYSIS.md) | Dependencies and build |
| [PUREPURSUIT_REVERSE_ANALYSIS.md](pure-pursuit/PUREPURSUIT_REVERSE_ANALYSIS.md) | Reverse motion analysis |

**When to Read**:
- Setting `velocity_control_mode: 2`
- Tuning lookahead parameters
- Implementing bidirectional navigation
- Debugging path tracking

---

### GIK Solver (`gik-solver/`)

**Purpose**: 9-DOF inverse kinematics solver (MATLAB codegen)

| Document | Description |
|----------|-------------|
| [GIK_ENHANCEMENTS_QUICKREF.md](gik-solver/GIK_ENHANCEMENTS_QUICKREF.md) | Quick reference for enhancements |
| [STAGE_B_FIXES.md](gik-solver/STAGE_B_FIXES.md) | Stage B controller fixes |
| [PLANNER_PROJECT_SUMMARY.md](gik-solver/PLANNER_PROJECT_SUMMARY.md) | Integration with planners |

**When to Read**:
- Using Holistic mode
- Stage C (whole-body tracking)
- Submode B2 (GIK-assisted planning)
- Tuning solver parameters

---

## 🔧 Development Documentation

### Code Generation

| Document | Description |
|----------|-------------|
| [CODEGEN_STRUCTURE.md](CODEGEN_STRUCTURE.md) | Understanding codegen directories |
| [CODEGEN.md](CODEGEN.md) | MATLAB code generation details |
| [MATLAB_CODEGEN_ANALYSIS.md](MATLAB_CODEGEN_ANALYSIS.md) | What to generate, what to skip |

**When to Read**:
- Regenerating MATLAB code
- ARM64 platform build
- Understanding `matlab_solver/` structure

---

### Controllers

| Document | Description |
|----------|-------------|
| [unified_chassis_controller_summary.md](unified_chassis_controller_summary.md) | Chassis controller overview |

**When to Read**:
- Understanding velocity control implementation
- Debugging wheel velocity commands

---

## 📋 Quick Reference Tables

### Control Mode Selection

| Mode | When to Use | Configuration |
|------|-------------|---------------|
| **Holistic** | Simple trajectory, no obstacles | `control_mode: "holistic"` |
| **Staged** | Complex navigation, obstacles | `control_mode: "staged"` |

### Velocity Control Modes

| Mode | Algorithm | When to Use |
|------|-----------|-------------|
| **0** | Legacy (5-point diff) | Debugging only |
| **1** | Heading controller | Simple testing |
| **2** | Pure Pursuit | **Production (RECOMMENDED)** |

### Stage B Submodes

| Submode | Pipeline | When to Use |
|---------|----------|-------------|
| **B1** | Hybrid A* → Velocity | Fast planning, occupancy grid available |
| **B2** | Hybrid A* → GIK 3-DOF → Velocity | Complex constraints, high accuracy |

---

## 🎯 Common Tasks

### Task 1: Change Control Mode
1. **Edit**: `ros2/gik9dof_solver/config/gik9dof_solver.yaml`
2. **Change**: `control_mode: "holistic"` or `"staged"`
3. **Restart**: `ros2 launch gik9dof_solver gik9dof_solver_launch.py`
4. **Verify**: Check logs for mode initialization

**Reference**: [State Machine Architecture](STATE_MACHINE_ARCHITECTURE.md#control-mode-selection)

---

### Task 2: Tune Pure Pursuit Controller
1. **Edit**: `ros2/gik9dof_solver/config/gik9dof_solver.yaml`
2. **Adjust**:
   ```yaml
   purepursuit:
     lookahead_base: 0.8       # Increase for smoother, wider turns
     lookahead_vel_gain: 0.3   # Velocity-dependent lookahead
     vx_max: 1.5               # Max forward speed
   ```
3. **Restart**: Launch node
4. **Test**: Monitor path tracking performance

**Reference**: [Pure Pursuit Quickstart](pure-pursuit/PUREPURSUIT_QUICKSTART.md)

---

### Task 3: Configure Hybrid A* Planner
1. **Edit**: `ros2/gik9dof_solver/config/gik9dof_solver.yaml`
2. **Adjust**:
   ```yaml
   staged:
     stage_b:
       grid_resolution: 0.1       # Smaller = more precise, slower
       max_planning_time: 0.05    # Max time per planning cycle
       robot_radius: 0.5          # Safety margin
   ```
3. **Restart**: Launch node
4. **Test**: Navigate around obstacles

**Reference**: [Hybrid A* Design](hybrid-astar/HYBRID_ASTAR_DESIGN.md)

---

### Task 4: Debug Stage Transitions
1. **Check logs** for transition messages:
   ```
   [STAGE A] Arm at home. Transitioning to STAGE_B.
   [STAGE B] Chassis reached goal. Transitioning to STAGE_C.
   ```
2. **If stuck in Stage A**: Check arm joint positions
3. **If stuck in Stage B**: Check goal tolerance, obstacles
4. **Check timeouts** in config file

**Reference**: [State Machine Diagrams](STATE_MACHINE_DIAGRAMS.md)

---

## 🔍 Troubleshooting Guide

### Issue: Stage A never completes
**Symptoms**: Stays in Stage A indefinitely

**Diagnosis**:
1. Check arm joint positions in logs
2. Compare with home config `[0, 0, 0, 0, 0, 0]`
3. Check `position_tolerance` parameter (default 0.1 rad)

**Solutions**:
- Increase tolerance: `stage_a.position_tolerance: 0.2`
- Check for joint limits or collisions
- Verify arm controller is working

**Reference**: [State Machine Architecture - Stage A](STATE_MACHINE_ARCHITECTURE.md#stage-a-arm-ramp-up-6-dof-chassis-frozen)

---

### Issue: Stage B never reaches goal
**Symptoms**: Chassis moves but never transitions to Stage C

**Diagnosis**:
1. Check current chassis pose vs goal pose
2. Check `xy_tolerance` and `theta_tolerance` parameters
3. Check for obstacles blocking path

**Solutions**:
- Increase tolerances:
  ```yaml
  staged:
    stage_b:
      xy_tolerance: 0.20       # Was 0.15
      theta_tolerance: 0.262   # 15 degrees (was 10)
  ```
- Check planner is finding valid paths
- Verify occupancy grid if using Hybrid A*

**Reference**: [State Machine Architecture - Stage B](STATE_MACHINE_ARCHITECTURE.md#stage-b-chassis-planning)

---

### Issue: Jerky velocity control
**Symptoms**: Robot motion is not smooth

**Diagnosis**:
1. Check `velocity_control_mode` (should be 2)
2. Monitor lookahead distance
3. Check path waypoint spacing

**Solutions**:
- Switch to Pure Pursuit: `velocity_control_mode: 2`
- Increase lookahead:
  ```yaml
  purepursuit:
    lookahead_base: 1.0       # Was 0.8
    lookahead_vel_gain: 0.4   # Was 0.3
  ```
- Reduce `vx_max` for smoother motion

**Reference**: [Pure Pursuit Design](pure-pursuit/PUREPURSUIT_DESIGN.md)

---

## 📚 Related Documentation

### Main Project Docs
- [START_HERE.md](../../START_HERE.md) - First-time setup
- [QUICK_START_NEXT_SESSION.md](../../QUICK_START_NEXT_SESSION.md) - Developer quick ref
- [README.md](../../README.md) - Project overview

### Deployment Guides
- [docs/deployment/](../deployment/) - Deployment to AGX Orin
- [docs/guides/](../guides/) - Development guides

### Historical Context
- [docs/archive/sessions/](../archive/sessions/) - Session summaries
- [docs/archive/namespace-conflict/](../archive/namespace-conflict/) - Namespace fix history

---

## 📝 Contributing to Documentation

### Adding New Technical Docs

1. **Determine category**:
   - Component-specific? → `hybrid-astar/`, `pure-pursuit/`, or `gik-solver/`
   - General technical? → Root `technical/` directory

2. **Follow naming convention**:
   - UPPERCASE with underscores: `MY_NEW_DOC.md`
   - Descriptive names: `FEATURE_IMPLEMENTATION_GUIDE.md`

3. **Include standard header**:
   ```markdown
   # Document Title
   
   **Status**: ✅ COMPLETE / 🔄 WIP / ⏳ PLANNED
   **Last Updated**: YYYY-MM-DD
   **Related Files**: List relevant source files
   ```

4. **Update this README**: Add to appropriate section

5. **Cross-reference**: Link to related docs

---

## 📊 Documentation Status

| Category | Docs | Status |
|----------|------|--------|
| **State Machine** | 3 | ✅ Complete |
| **Hybrid A*** | 4 | ✅ Complete |
| **Pure Pursuit** | 6 | ✅ Complete |
| **GIK Solver** | 3 | ✅ Complete |
| **Codegen** | 3 | ✅ Complete |
| **Controllers** | 1 | ✅ Complete |

**Last Audit**: 2025-01-XX

---

**Need Help?**
- 🚀 Start with [State Machine Quick Reference](STATE_MACHINE_QUICKREF.md)
- 🔍 Search for keywords in relevant component directory
- 📖 Check main [README.md](../../README.md) for overview
