# Namespace Conflict Resolution - Stage B Integration

## Date: October 7, 2025
## Status: IN PROGRESS - Token limit reached, continuation needed

---

## PROBLEM SUMMARY

### Root Cause
Both MATLAB Coder generated modules use **identical namespace and struct names**:

1. **GIK Solver** (`gik_matlab_solver` library):
   - Namespace: `gik9dof::`
   - Types: `struct0_T` (solver info), `struct1_T` (constraint violations)

2. **Hybrid A* Planner** (`hybrid_astar_planner` library):
   - Namespace: `gik9dof::`
   - Types: `struct0_T` (node state), `struct1_T` (waypoint)

**Compiler Error**: When both headers included in same translation unit:
```
error: redefinition of 'struct gik9dof::struct0_T'
error: redefinition of 'struct gik9dof::struct1_T'
```

### Why This Happens
- Stage B controller (`StageBController`) needs **planner** headers
- Main node needs **GIK solver** headers
- Both need to be in the same executable → namespace collision

---

## ATTEMPTED SOLUTIONS (All Failed)

### ❌ Attempt 1: Forward Declarations in Header
- **Approach**: Forward declare types, don't include headers
- **Failure**: Inline constructor still needs complete types for initialization

### ❌ Attempt 2: File Split (Header + Implementation)
- **Approach**: Split node into .h (declarations) + .cpp (implementations)
- **Failure**: Both conflicting headers still end up in same .cpp file

### ❌ Attempt 3: Pimpl with Wrapper Class
- **Approach**: Create `StageBControllerWrapper` to hide planner types
- **Failure**: Wrapper itself needs to call real methods, still needs planner headers

### ❌ Attempt 4: Separate Library + unique_ptr
- **Approach**: Compile Stage B in separate library, use unique_ptr
- **Failure**: `unique_ptr<T>` destructor requires complete type `T` at point of use

### ❌ Attempt 5: Separate Library + Raw Pointer
- **Approach**: Use raw pointer instead of unique_ptr
- **Current Status**: **CLOSEST TO SUCCESS** but still blocked
- **Failure**: Cannot call methods on incomplete type (forward declaration)

---

## CURRENT STATE (Where We Left Off)

### Files Created/Modified

#### ✅ `stage_b_factory.hpp` (NEW)
- Forward declares `StageBController`
- Declares `StageBParams` and `StageBMode` enums
- Factory functions:
  ```cpp
  gik9dof::StageBController* createStageBController(rclcpp::Node*, const StageBParams&);
  void destroyStageBController(gik9dof::StageBController*);
  ```

#### ✅ `stage_b_chassis_plan.cpp` 
- Implements factory functions (returns `new StageBController`, `delete controller`)

#### ✅ `CMakeLists.txt`
- Created `stage_b_controller` library (links against `hybrid_astar_planner`)
- Node links against `gik_matlab_solver` + `stage_b_controller` (NOT planner directly)

#### ✅ `gik9dof_solver_node.h`
- Changed member: `gik9dof::StageBController* stage_b_controller_;` (raw pointer)
- Forward declares `StageBController`

#### ✅ `gik9dof_solver_node.cpp`
- Includes `stage_b_factory.hpp` (NOT stage_b_chassis_plan.hpp)
- Constructor: `stage_b_controller_ = gik9dof::createStageBController(this, params);`
- Destructor: `gik9dof::destroyStageBController(stage_b_controller_);`

### Current Build Error
```
error: invalid use of incomplete type 'class gik9dof::StageBController'
```

**Lines failing**:
- Line 410: `stage_b_controller_->activate(...)`
- Line 448: `stage_b_controller_->executeStep(...)`
- Line 454: `stage_b_controller_->chassisReachedGoal()`
- Line 455: `stage_b_controller_->deactivate()`

**Why**: Cannot call methods on forward-declared (incomplete) class, even through pointer!

---

## NEXT STEPS (Solution Path)

### Option A: Complete Pimpl Pattern (Recommended)
Add **wrapper functions** for ALL Stage B methods in `stage_b_factory.hpp/cpp`:

```cpp
// In stage_b_factory.hpp
namespace gik9dof {
    // Factory
    StageBController* createStageBController(rclcpp::Node*, const StageBParams&);
    void destroyStageBController(StageBController*);
    
    // Method wrappers (NEW - needed!)
    void stageBActivate(StageBController*, 
                       const Eigen::Vector3d& current,
                       const Eigen::Vector3d& goal,
                       const std::vector<double>& arm_config);
    
    bool stageBExecuteStep(StageBController*,
                          const Eigen::Vector3d& current,
                          const std::vector<double>& arm_config,
                          geometry_msgs::msg::Twist& base_cmd,
                          sensor_msgs::msg::JointState& arm_cmd);
    
    bool stageBChassisReachedGoal(StageBController*, const Eigen::Vector3d& current);
    void stageBDeactivate(StageBController*);
}
```

```cpp
// In stage_b_chassis_plan.cpp (implementation)
void stageBActivate(StageBController* ctrl, 
                   const Eigen::Vector3d& current,
                   const Eigen::Vector3d& goal,
                   const std::vector<double>& arm_config) {
    ctrl->activate(current, goal, arm_config);
}
// ... etc for all methods
```

**Then in node .cpp**:
```cpp
// executeStageA:
gik9dof::stageBActivate(stage_b_controller_, current, goal, arm_config);

// executeStageB:
bool active = gik9dof::stageBExecuteStep(stage_b_controller_, current, arm_config, base_cmd, arm_cmd);
if (gik9dof::stageBChassisReachedGoal(stage_b_controller_, current)) {
    gik9dof::stageBDeactivate(stage_b_controller_);
    current_stage_ = ControlStage::STAGE_C;
}
```

### Option B: Change MATLAB Coder Namespace (Ultimate Fix)
**Regenerate** one MATLAB Coder module with different namespace:
- Edit MATLAB Coder settings to use `gik9dof_planner::` instead of `gik9dof::`
- Regenerate planner code
- No more conflicts!

**Why not done**: Requires MATLAB + regeneration workflow

---

## TECHNICAL NOTES

### Why Separate Libraries Don't Solve It
- Both libraries are **linked** into final executable
- Linker merges all symbols from both libraries
- **Duplicate symbol error** at link time (or runtime symbol collision)
- Even with separate compilation, final binary has both conflicting types

### Why Forward Declaration Doesn't Work
- Forward declaration allows **pointer declaration** ✅
- But **method calls** require complete class definition ❌
- C++ compiler needs to know:
  - Method signatures (parameter types, return type)
  - Virtual table layout (for polymorphism)
  - Member offsets (for `this` pointer adjustment)

### Pimpl Pattern Requirements
For Pimpl to work with incomplete types:
1. ✅ Use pointer (raw or smart) to forward-declared class
2. ✅ Define destructor in .cpp (where complete type available)
3. ❌ **Cannot call methods directly** - need wrapper functions!
4. ✅ All actual operations go through wrapper functions in separate library

---

## FILES TO MODIFY (Next Session)

### 1. `stage_b_factory.hpp`
Add wrapper function declarations for:
- `stageBActivate()`
- `stageBExecuteStep()`
- `stageBChassisReachedGoal()`
- `stageBDeactivate()`

### 2. `stage_b_chassis_plan.cpp`
Implement wrapper functions (simple delegation to actual methods)

### 3. `gik9dof_solver_node.cpp`
Update `executeStageA()` and `executeStageB()` to call wrapper functions instead of direct methods

### 4. Build & Test
- Should compile cleanly
- No namespace conflicts (wrappers are in stage_b library)
- Test state machine transitions

---

## ESTIMATED WORK REMAINING

- **Time**: 15-20 minutes
- **Lines of code**: ~50 lines (wrappers + updates)
- **Build time**: 2-3 minutes
- **Risk**: Low (pattern is proven, just tedious)

---

## ALTERNATIVE: Quick Hack (If desperate)

**Rename conflicting types manually** in generated code:
```bash
cd ros2/gik9dof_solver/src/generated/planner
sed -i 's/struct0_T/planner_struct0_T/g' *.h *.cpp
sed -i 's/struct1_T/planner_struct1_T/g' *.h *.cpp
```

**Pros**: Immediate fix
**Cons**: Lost on next codegen, fragile, hack-ish

---

## KEY INSIGHT

**The fundamental issue**: C++ does not allow method calls on incomplete types.

**The only clean solution**: Pimpl pattern requires **wrapper functions** for every method you want to call from outside the library.

**Why this wasn't obvious**: Most Pimpl examples show data access, not complex method calls. When the hidden class has rich API, you need rich wrapper API.

---

## COMMIT MESSAGE (When done)

```
fix: resolve namespace conflicts between GIK solver and planner

- Created stage_b_factory with wrapper functions for all Stage B methods
- Implemented complete Pimpl pattern to isolate planner namespace
- Stage B controller now in separate library with C-style API
- Main node uses factory + wrappers, no direct header inclusion
- Resolves gik9dof::struct0_T / struct1_T redefinition errors

Refs: NAMESPACE_CONFLICT_RESOLUTION.md
```
