# State Machine Integration - Work in Progress

**Date**: 2025-10-07  
**Status**: 🚧 **IN PROGRESS** - Namespace conflict blocking build

---

## Summary

Implementing staged control state machine (A → B → C) in `gik9dof_solver_node.cpp` to enable efficient mobile manipulator control with chassis planning separation.

---

## Completed Work ✅

### 1. **State Machine Framework Added**
- Added `ControlMode` enum: `HOLISTIC` | `STAGED`
- Added `ControlStage` enum: `STAGE_A` | `STAGE_B` | `STAGE_C`
- Added `stage_b_controller_` member (`std::unique_ptr<gik9dof::StageBController>`)
- Added `control_mode_` and `current_stage_` state tracking

### 2. **Parameter Infrastructure**
- Declared `control_mode` parameter ("holistic" | "staged")
- Declared `staged.*` parameters:
  - `staged.stage_b_mode` (1=Pure Hybrid A*, 2=GIK-Assisted)
  - `staged.planner.max_iterations`, `timeout_ms`, tolerances
  - `staged.stage_a_timeout`, `stage_b_timeout`, `stage_c_timeout`

### 3. **Control Flow Refactoring**
- Refactored `controlLoop()` to dispatch to `executeStagedControl()` or `executeHolisticControl()`
- Implemented `executeStagedControl()` with switch on `current_stage_`
- Created stage execution functions:
  - `executeStageA()` - Arm ramp-up to home configuration
  - `executeStageB()` - Chassis planning and path following
  - `executeStageC()` - Whole-body tracking (delegates to holistic mode)
  - `executeHolisticControl()` - Original GIK solve logic

### 4. **Stage Transitions**
- **A → B**: When `checkArmAtHome()` returns true
  - Activates Stage B controller with goal pose
  - Logs transition with current/goal poses
- **B → C**: When `chassisReachedGoal()` or Stage B completes
  - Deactivates Stage B controller
  - Logs final chassis pose
- **Helper Functions**:
  - `checkArmAtHome()` - Checks if arm within tolerance of home config
  - `extractYaw()` - Extracts yaw angle from quaternion

### 5. **Stage B Controller Integration**
- Created `StageBParams` struct initialization from ROS2 parameters
- Set planner parameters (grid resolution, robot radius, timeouts)
- Set goal tolerances (xy, theta)
- Pass GIK 3-DOF parameters for Stage B2 mode
- Pass velocity controller mode selection

---

## Current Blocker 🚧

### **Namespace Conflict: `struct0_T` and `struct1_T`**

**Problem**: Both MATLAB Coder modules define the same struct names in `gik9dof` namespace:
- **GIK Solver**: `gik9dof_codegen_realtime_solveGIKStepWrapper_types.h`
  - `struct0_T` = Solver info (iterations, time, success)
  - `struct1_T` = Constraint violations (pose, joint limits, distance)
- **Hybrid A* Planner**: `gik9dof_planHybridAStarCodegen_types.h`
  - `struct0_T` = Node state (x, y, theta, grid coordinates)
  - `struct1_T` = Waypoint (x, y, theta)

**Error**:
```
error: redefinition of 'struct gik9dof::struct0_T'
error: redefinition of 'struct gik9dof::struct1_T'
```

**Why it happens**:
- `GIKSolver.h` included early in node (line 27)
- `stage_b_chassis_plan.hpp` includes `HybridAStarPlanner.h` which includes planner types
- Both headers pulled into same translation unit → conflict

**Attempted Solutions**:
1. ❌ **Forward declarations** - Doesn't work for inline constructor logic using types
2. ❌ **Include after class definition** - Still conflicts when included in same .cpp file
3. ❌ **Remove GIK from Stage B** - Already done, Stage B is chassis-only (3-DOF)

---

## Proposed Solutions

### **Option 1: PIMPL Idiom (Recommended)** ⭐
Move Stage B controller to pointer-to-implementation:
```cpp
// In class:
class StageBControllerImpl;  // Forward declaration
std::unique_ptr<StageBControllerImpl> stage_b_impl_;

// In .cpp (separate file):
#include "stage_b_chassis_plan.hpp"
class StageBControllerImpl {
    std::unique_ptr<gik9dof::StageBController> controller_;
    // ... wrapper methods ...
};
```

**Pros**: Clean separation, no namespace pollution  
**Cons**: Extra wrapper class, slight performance overhead

### **Option 2: Rename Planner Namespace**
Regenerate planner code with different namespace (e.g., `gik9dof_planner`):
```matlab
% In MATLAB Coder settings:
cfg.CustomHeaderCode = 'namespace gik9dof_planner {';
cfg.CustomSourceCode = '}  // namespace gik9dof_planner';
```

**Pros**: Cleanest long-term solution  
**Cons**: Requires MATLAB regeneration, affects existing code

### **Option 3: Anonymous Namespace Wrapper**
Wrap one of the includes in anonymous namespace:
```cpp
namespace {
#include "stage_b_chassis_plan.hpp"
}  // anonymous
using ::StageBController;  // Export needed types
```

**Pros**: Minimal code changes  
**Cons**: Hacky, may cause linker issues

### **Option 4: Split Node Into Header + Implementation**
Create `gik9dof_solver_node.h` with class definition, move constructor/methods to .cpp:
- `gik9dof_solver_node.h` - Class definition, forward declarations
- `gik9dof_solver_node.cpp` - Include both headers, implement methods

**Pros**: Standard C++ practice, clean architecture  
**Cons**: Large refactor, breaks single-file simplicity

---

## Recommended Next Steps

### **Immediate: Option 4 (Split Node)**
1. Create `gik9dof_solver_node.h`:
   - Move class definition from .cpp to header
   - Forward-declare `gik9dof::StageBController`
   - Keep inline methods minimal
   
2. Update `gik9dof_solver_node.cpp`:
   - Include both `GIKSolver.h` AND `stage_b_chassis_plan.hpp`
   - Move all constructor logic and stage execution methods to .cpp
   - No conflicts since types are only used in implementation

3. Update `CMakeLists.txt`:
   - No changes needed (still compiles same .cpp file)

### **Long-Term: Option 2 (Rename Planner Namespace)**
- Regenerate planner code in MATLAB with unique namespace
- Update all planner references
- Prevents future conflicts with other MATLAB Coder modules

---

## Code Changes Summary

### Files Modified
| File | Lines Changed | Status |
|------|---------------|--------|
| `gik9dof_solver_node.cpp` | ~300 | ✅ Logic done, ❌ Won't compile |
| `stage_b_chassis_plan.hpp` | 0 | ✅ Already complete |
| `stage_b_chassis_plan.cpp` | 0 | ✅ Already complete |

### Key Additions
- **State Machine**: 150 lines (enum defs, control flow, transitions)
- **Stage A Logic**: 40 lines (arm home check, B activation)
- **Stage B Logic**: 35 lines (execute step, chassis goal check)
- **Helper Functions**: 25 lines (extractYaw, checkArmAtHome)
- **Parameters**: 15 lines (control_mode, staged.*)

**Total**: ~265 lines added

---

## Testing Plan (Once Built)

### **Unit Testing**
1. **Stage A**: Mock arm state, verify A→B transition when at home
2. **Stage B**: Mock occupancy grid, verify path planning and following
3. **Stage C**: Verify transition to whole-body tracking
4. **Holistic Mode**: Verify unchanged behavior when `control_mode: "holistic"`

### **Integration Testing**
1. **A→B→C Pipeline**: End-to-end staged control with mock trajectory
2. **Performance**: Verify < 50ms planning, smooth transitions
3. **Diagnostics**: Check planner diagnostics published correctly

### **Hardware Testing (Orin)**
1. **Real Occupancy Grid**: Test with actual sensor data
2. **GIK Performance**: Validate 3-5× speedup with iteration limit
3. **Smooth Execution**: No jerks during stage transitions

---

## Architecture Decisions

### **Why Staged Control?**
- **Performance**: Chassis planning (2D) much faster than 9-DOF GIK
- **Modularity**: Separate arm initialization, chassis motion, tracking phases
- **Flexibility**: Can switch execution modes (B1 vs B2) via parameter

### **Why Stage B in Main Node?**
- **Tight Coupling**: Control loop needs direct access to Stage B state
- **State Machine**: Natural fit in main control loop
- **Performance**: Avoid extra ROS2 messaging overhead

### **Why Three Stages?**
- **Stage A**: Arm safety (ramp to known good configuration)
- **Stage B**: Chassis efficiency (fast 2D planning without arm)
- **Stage C**: Whole-body precision (full 9-DOF tracking)

---

## Next Actions

1. **Resolve Namespace Conflict** (Option 4: Split node into .h/.cpp)
2. **Build and Test** in WSL
3. **Deploy to Orin** and validate on ARM64
4. **Performance Testing** (measure planning time, GIK solve time)
5. **Integration with Pure Pursuit Bidirectional** (already implemented)

---

**Status**: Ready for namespace conflict resolution. All logic implemented, just needs compilation fix.
