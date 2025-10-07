# State Machine Quick Reference

**Complete Documentation**: [`STATE_MACHINE_ARCHITECTURE.md`](STATE_MACHINE_ARCHITECTURE.md)

---

## Control Flow at a Glance

```
┌─────────────────────────────────────────────────────┐
│          control_mode: "holistic" | "staged"        │
└───────────────┬─────────────────────────────────────┘
                │
       ┌────────┴─────────┐
       │                  │
   HOLISTIC            STAGED
   (Direct)         (A → B → C)
       │                  │
       │     ┌────────────┼────────────┐
       │     │            │            │
       │   STAGE A      STAGE B     STAGE C
       │   (Arm)      (Chassis)   (Whole-body)
       │     │            │            │
       └─────┴────────────┴────────────┘
                          │
              ┌───────────▼───────────┐
              │ velocity_control_mode │
              │   0 | 1 | 2           │
              └───────────────────────┘
```

---

## Quick Decision Tree

### Q1: What control mode?
```yaml
# Simple trajectory, no obstacles
control_mode: "holistic"

# Complex navigation with obstacles  
control_mode: "staged"
```

### Q2: (If Staged) Stage B submode?
```yaml
# Fast planning with occupancy grid
staged:
  stage_b:
    submode: "pure_hybrid_astar"

# Complex constraints, need refinement
staged:
  stage_b:
    submode: "gik_assisted"
```

### Q3: Which velocity controller?
```yaml
# PRODUCTION (RECOMMENDED)
velocity_control_mode: 2  # Pure Pursuit

# Simple testing
velocity_control_mode: 1  # Heading controller

# Legacy/debugging
velocity_control_mode: 0  # 5-point diff
```

---

## Stage Transitions (Staged Mode Only)

### Stage A → B: `checkArmAtHome()`
**Trigger**: All arm joints within 0.1 rad of home position `[0, 0, 0, 0, 0, 0]`

```cpp
// gik9dof_solver_node.cpp (line 503-520)
bool arm_at_home = checkArmAtHome();
if (arm_at_home) {
    gik9dof::stageBActivate(...);
    current_stage_ = ControlStage::STAGE_B;
}
```

### Stage B → C: `stageBChassisReachedGoal()`
**Trigger**: Chassis within tolerance of goal pose

```cpp
// Checks position and heading tolerance
if (gik9dof::stageBChassisReachedGoal(chassis_pose, goal_pose, 
                                       xy_tolerance, theta_tolerance)) {
    gik9dof::stageBDeactivate(...);
    current_stage_ = ControlStage::STAGE_C;
}
```

**Default tolerances**:
- `xy_tolerance: 0.15 m`
- `theta_tolerance: 0.175 rad` (10 degrees)

---

## Configuration File

**Location**: `ros2/gik9dof_solver/config/gik9dof_solver.yaml`

**Key parameters**:
```yaml
# Main mode selection
control_mode: "staged"              # "holistic" | "staged"
velocity_control_mode: 2            # 0 | 1 | 2

# Staged control
staged:
  stage_b:
    submode: "pure_hybrid_astar"    # "pure_hybrid_astar" | "gik_assisted"
    xy_tolerance: 0.15              # meters (for chassisReachedGoal)
    theta_tolerance: 0.175          # radians (10 degrees)
```

---

## Recommended Configuration

**For production navigation with obstacle avoidance**:
```yaml
control_mode: "staged"
velocity_control_mode: 2  # Pure Pursuit

staged:
  stage_b:
    submode: "pure_hybrid_astar"
    xy_tolerance: 0.15
    theta_tolerance: 0.175

purepursuit:
  lookahead_base: 0.8
  vx_max: 1.5
  vx_min: -1.0  # Bidirectional support
```

---

## Code References

| Component | File | Line |
|-----------|------|------|
| Control mode enum | `gik9dof_solver_node.h` | 156 |
| Mode initialization | `gik9dof_solver_node.cpp` | 83-87 |
| Mode switch | `gik9dof_solver_node.cpp` | 350-362 |
| Stage A execution | `gik9dof_solver_node.cpp` | 384-432 |
| Stage B execution | `gik9dof_solver_node.cpp` | 434-474 |
| Stage C execution | `gik9dof_solver_node.cpp` | 476-480 |
| `checkArmAtHome()` | `gik9dof_solver_node.cpp` | 503-520 |
| `stageBChassisReachedGoal()` | `stage_b_chassis_plan.cpp` | 451-467 |

---

## Related Documentation

- 📖 **[Full State Machine Documentation](STATE_MACHINE_ARCHITECTURE.md)** - Complete details
- 🚀 **[Hybrid A* Planner](hybrid-astar/HYBRID_ASTAR_README.md)** - Stage B planner
- 🎯 **[Pure Pursuit Controller](pure-pursuit/PUREPURSUIT_BIDIRECTIONAL_COMPLETE.md)** - Velocity control mode 2
- ⚙️ **[GIK Solver](gik-solver/GIK_ENHANCEMENTS_QUICKREF.md)** - Inverse kinematics solver
