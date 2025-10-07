# State Machine Documentation - Complete

**Created**: 2025-01-XX  
**Status**: ✅ **COMPLETE**

---

## What Was Created

Three comprehensive documents explaining the GIK9DOF control system state machine:

### 1. **STATE_MACHINE_ARCHITECTURE.md** (Main Documentation)
**File**: `docs/technical/STATE_MACHINE_ARCHITECTURE.md`

**Purpose**: Complete technical reference for the control system

**Contents**:
- ✅ Overview of dual-mode architecture (Holistic vs Staged)
- ✅ Control mode selection (`control_mode` parameter)
- ✅ Holistic mode detailed flow
- ✅ Staged mode (A → B → C) detailed flow for each stage
- ✅ Stage transition flags (`checkArmAtHome()`, `stageBChassisReachedGoal()`)
- ✅ Velocity control selection (modes 0/1/2)
- ✅ Stage B submodes (Pure Hybrid A* vs GIK-Assisted)
- ✅ Configuration guide with YAML examples
- ✅ State transition diagram (ASCII art)
- ✅ Decision tree for selecting modes
- ✅ Code references table (file + line numbers)

**Word Count**: ~4,800 words

---

### 2. **STATE_MACHINE_QUICKREF.md** (Quick Reference)
**File**: `docs/technical/STATE_MACHINE_QUICKREF.md`

**Purpose**: Quick lookup guide for developers

**Contents**:
- ✅ Control flow diagram (single page)
- ✅ Quick decision tree (Q1: mode? Q2: submode? Q3: velocity?)
- ✅ Stage transition triggers with code snippets
- ✅ Configuration file location and key parameters
- ✅ Recommended production configuration
- ✅ Code reference table
- ✅ Links to related documentation

**Word Count**: ~800 words

---

### 3. **STATE_MACHINE_DIAGRAMS.md** (Visual Reference)
**File**: `docs/technical/STATE_MACHINE_DIAGRAMS.md`

**Purpose**: Comprehensive visual diagrams for all state transitions

**Contents**:
- ✅ Complete system state machine (all modes + transitions)
- ✅ Stage A detailed flow with timeout handling
- ✅ Stage B detailed flow with goal checking
- ✅ Stage B submode decision tree (B1 vs B2)
- ✅ Velocity controller selection flow
- ✅ Parameter configuration flow (YAML → Runtime)
- ✅ Box-and-arrow diagrams (ASCII art)
- ✅ Legend for diagram notation

**Diagrams**: 6 detailed ASCII flow diagrams

---

## Questions Answered

All questions from the user's request:

### Q1: "How to switch between holistic and staged?"
**Answer**: Edit `control_mode` parameter in YAML

```yaml
# ros2/gik9dof_solver/config/gik9dof_solver.yaml
control_mode: "holistic"  # or "staged"
```

**Documentation**: 
- Architecture doc (Section: Control Mode Selection)
- Quick ref (Section: Q1: What control mode?)
- Diagrams (Complete system state machine)

---

### Q2: "What flags control stage transitions (A → B → C)?"
**Answer**: Two flag functions

**Stage A → B**: `checkArmAtHome()` returns `true`
- Checks all arm joints within 0.1 rad of `[0, 0, 0, 0, 0, 0]`
- Code: `gik9dof_solver_node.cpp` lines 503-520

**Stage B → C**: `stageBChassisReachedGoal()` returns `true`
- Checks chassis position within `xy_tolerance` (default 0.15 m)
- Checks heading within `theta_tolerance` (default 0.175 rad)
- Code: `stage_b_chassis_plan.cpp` lines 451-467

**Documentation**:
- Architecture doc (Sections: Stage A, Stage B)
- Quick ref (Section: Stage Transitions)
- Diagrams (Stage A/B detailed flows)

---

### Q3: "How to switch velocity control modes?"
**Answer**: Edit `velocity_control_mode` parameter in YAML

```yaml
# ros2/gik9dof_solver/config/gik9dof_solver.yaml
velocity_control_mode: 2  # Options: 0 | 1 | 2
```

**Modes**:
- 0 = Legacy (5-point differentiation)
- 1 = Simple heading controller
- 2 = Pure Pursuit (RECOMMENDED)

**Documentation**:
- Architecture doc (Section: Velocity Control Selection)
- Quick ref (Section: Q3: Which velocity controller?)
- Diagrams (Velocity controller selection flow)

---

### Q4: "How to switch Stage B submodes?"
**Answer**: Edit `staged.stage_b.submode` parameter in YAML

```yaml
# ros2/gik9dof_solver/config/gik9dof_solver.yaml
staged:
  stage_b:
    submode: "pure_hybrid_astar"  # or "gik_assisted"
```

**Submodes**:
- B1: `"pure_hybrid_astar"` = Hybrid A* → Velocity (faster)
- B2: `"gik_assisted"` = Hybrid A* → GIK 3-DOF → Velocity (refined)

**Documentation**:
- Architecture doc (Section: Stage B Submodes)
- Quick ref (Section: Q2: Stage B submode?)
- Diagrams (Stage B submode decision)

---

## Code References Documented

All critical code locations documented:

| Component | File | Lines | Documented In |
|-----------|------|-------|---------------|
| Enum definitions | `gik9dof_solver_node.h` | 156-162 | Architecture, Quick Ref |
| Parameter parsing | `gik9dof_solver_node.cpp` | 83-87 | Architecture |
| Control mode switch | `gik9dof_solver_node.cpp` | 350-362 | Architecture, Diagrams |
| Stage A execution | `gik9dof_solver_node.cpp` | 384-432 | Architecture |
| Stage B execution | `gik9dof_solver_node.cpp` | 434-474 | Architecture |
| Stage C execution | `gik9dof_solver_node.cpp` | 476-480 | Architecture |
| `checkArmAtHome()` | `gik9dof_solver_node.cpp` | 503-520 | Architecture, Quick Ref |
| `stageBChassisReachedGoal()` | `stage_b_chassis_plan.cpp` | 451-467 | Architecture, Quick Ref |
| Wrapper functions | `stage_b_chassis_plan.cpp` | 421-467 | Architecture |
| Factory header | `stage_b_factory.hpp` | Full file | Architecture |
| Configuration file | `gik9dof_solver.yaml` | Full file | All 3 docs |

---

## Integration with Existing Docs

### Updated Files

**README.md**:
- Added reference to state machine documentation in Technical Documentation section
- Highlighted as "⭐ Control System Architecture"

### Cross-References

All three new documents reference existing documentation:

**Hybrid A* Planner**:
- `docs/technical/hybrid-astar/HYBRID_ASTAR_README.md`
- `docs/technical/hybrid-astar/HYBRID_ASTAR_DESIGN.md`

**Pure Pursuit Controller**:
- `docs/technical/pure-pursuit/PUREPURSUIT_BIDIRECTIONAL_COMPLETE.md`
- `docs/technical/pure-pursuit/PUREPURSUIT_QUICKSTART.md`

**GIK Solver**:
- `docs/technical/gik-solver/GIK_ENHANCEMENTS_QUICKREF.md`

---

## Usage Recommendations

### For First-Time Users
👉 **Start with**: `STATE_MACHINE_QUICKREF.md`
- Quick decision tree
- Recommended configuration
- 5-minute read

### For Developers
👉 **Start with**: `STATE_MACHINE_ARCHITECTURE.md`
- Complete technical details
- Code references
- Configuration guide
- 15-minute read

### For Visual Learners
👉 **Start with**: `STATE_MACHINE_DIAGRAMS.md`
- Flow diagrams for all states
- Visual transition logic
- 10-minute read

---

## Example Workflows Documented

### Workflow 1: Simple Navigation (No Obstacles)
**Use Case**: Direct point-to-point motion

**Configuration**:
```yaml
control_mode: "holistic"
velocity_control_mode: 2  # Pure Pursuit
```

**Reference**: Architecture doc, Section "Holistic Mode"

---

### Workflow 2: Warehouse Navigation (With Obstacles)
**Use Case**: Navigate with obstacle avoidance

**Configuration**:
```yaml
control_mode: "staged"
velocity_control_mode: 2  # Pure Pursuit

staged:
  stage_b:
    submode: "pure_hybrid_astar"
    xy_tolerance: 0.15
    theta_tolerance: 0.175
```

**Reference**: Architecture doc, Section "Summary: Decision Tree"

---

### Workflow 3: Complex Constrained Motion
**Use Case**: Tight spaces, high accuracy required

**Configuration**:
```yaml
control_mode: "staged"
velocity_control_mode: 2  # Pure Pursuit

staged:
  stage_b:
    submode: "gik_assisted"
    xy_tolerance: 0.10  # Tighter tolerance
    theta_tolerance: 0.087  # 5 degrees
```

**Reference**: Architecture doc, Sections "Stage B Submodes" + "Configuration Guide"

---

## Testing Guidance

All three documents include references to:

1. **Parameter location**: `ros2/gik9dof_solver/config/gik9dof_solver.yaml`
2. **How to restart**: `ros2 launch gik9dof_solver gik9dof_solver_launch.py`
3. **Validation**: Check logs for stage transitions

**Example Log Messages** (documented in Architecture doc):
```
[STAGE A] Arm at home. Transitioning to STAGE_B.
[STAGE B] Chassis reached goal. Transitioning to STAGE_C.
```

---

## Summary Statistics

| Metric | Value |
|--------|-------|
| Documents Created | 3 |
| Total Word Count | ~6,400 words |
| Diagrams Created | 6 ASCII flow diagrams |
| Code References | 10 file+line mappings |
| Configuration Examples | 12+ YAML snippets |
| Questions Answered | 4 (all from user) |
| Cross-References | 8 to existing docs |

---

## Next Steps for Users

1. ✅ **Read Quick Reference** - Understand basic modes
2. ✅ **Choose Configuration** - Use decision tree
3. ✅ **Edit YAML** - Update parameters
4. ✅ **Test** - Launch node and validate
5. ✅ **Refer to Architecture** - Deep dive when needed

---

## Maintenance Notes

**When to Update These Docs**:
- ❗ Adding new control modes
- ❗ Changing stage transition logic
- ❗ Adding velocity control modes
- ❗ Modifying tolerance defaults
- ❗ Changing parameter names

**Files to Update**:
1. Architecture doc (main reference)
2. Quick ref (if affects decision tree)
3. Diagrams (if affects flow)
4. README.md (if new major section)

---

## Related Session Documents

- `docs/archive/namespace-conflict/NAMESPACE_CONFLICT_RESOLUTION.md` - How we got here
- `NAMESPACE_FIX_TEST_RESULTS.md` - Runtime validation
- `WORKSPACE_CLEANUP_SUMMARY.md` - File organization

---

**Status**: ✅ **COMPLETE** - All user questions answered, fully documented, integrated with existing docs.
