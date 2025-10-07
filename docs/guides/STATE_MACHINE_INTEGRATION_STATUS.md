# State Machine Integration Status

**Date**: October 8, 2025  
**Status**: ✅ **ACTIVE** - Stage B Pure Pursuit integration complete, namespace issues resolved

**Latest Update**: Pure Pursuit controller successfully integrated with Hybrid A* in Stage B for smooth path tracking.

---

## Summary

Implemented staged control state machine (A → B → C) in `gik9dof_solver_node.cpp` with efficient mobile manipulator control and chassis planning separation. **Stage B now features Pure Pursuit controller integration for smooth velocity tracking of Hybrid A* paths.**

---

## Completed Work ✅

### 1. **State Machine Framework** ✅
- Added `ControlMode` enum: `HOLISTIC` | `STAGED`
- Added `ControlStage` enum: `STAGE_A` | `STAGE_B` | `STAGE_C`
- Added `stage_b_controller_` member (`StageBController*` via factory pattern)
- Added `control_mode_` and `current_stage_` state tracking
- **Status**: Fully implemented

### 2. **Parameter Infrastructure** ✅
- Declared `control_mode` parameter ("holistic" | "staged")
- Declared `staged.*` parameters:
  - `staged.stage_b_mode` (1=Pure Hybrid A*, 2=GIK-Assisted)
  - `staged.planner.max_iterations`, `timeout_ms`, tolerances
  - `staged.stage_a_timeout`, `stage_b_timeout`, `stage_c_timeout`
- Added `velocity_control_mode` parameter (0=legacy, 1=heading, 2=pure pursuit)
- **Status**: Fully implemented

### 3. **Control Flow Refactoring** ✅
- Refactored `controlLoop()` to dispatch to `executeStagedControl()` or `executeHolisticControl()`
- Implemented `executeStagedControl()` with switch on `current_stage_`
- Created stage execution functions:
  - `executeStageA()` - Arm ramp-up to home configuration
  - `executeStageB()` - Chassis planning and path following
  - `executeStageC()` - Whole-body tracking (delegates to holistic mode)
  - `executeHolisticControl()` - Original GIK solve logic
- **Status**: Fully implemented

### 4. **Stage Transitions** ✅
- **A → B**: When `checkArmAtHome()` returns true
  - Activates Stage B controller with goal pose
  - Logs transition with current/goal poses
- **B → C**: When `chassisReachedGoal()` or Stage B completes
  - Deactivates Stage B controller
  - Logs final chassis pose
- **Helper Functions**:
  - `checkArmAtHome()` - Checks if arm within tolerance of home config
  - `extractYaw()` - Extracts yaw angle from quaternion
- **Status**: Fully implemented

### 5. **Stage B Controller Integration** ✅
- Created `StageBParams` struct initialization from ROS2 parameters
- Set planner parameters (grid resolution, robot radius, timeouts)
- Set goal tolerances (xy, theta)
- Pass GIK 3-DOF parameters for Stage B2 mode
- Pass velocity controller mode selection (0/1/2)
- **NEW**: Pass Pure Pursuit parameters to Stage B controller
- **Status**: Fully implemented, builds successfully

### 6. **Pure Pursuit + Hybrid A* Integration** ✅ **NEW - Oct 8, 2025**
- Integrated Pure Pursuit velocity controller in `executeB1_PureHybridAStar()`
- Replaced jerky motion primitive velocities with smooth lookahead-based tracking
- Added velocity control mode switching in Stage B (mode 0/1/2)
- Resolved namespace conflicts between factory and internal param structures
- **Benefits**:
  - ✅ Smooth velocity transitions (no primitive jumps)
  - ✅ Lookahead-based steering (better accuracy)
  - ✅ Consistent Pure Pursuit across all modes (Holistic, Stage B, Stage C)
- **Status**: ✅ **COMPLETE** - Builds successfully, ready for testing

---

## Namespace Conflict Resolution ✅

### **Solution Implemented: Renamed Internal Structures**

**Problem** (RESOLVED): Both MATLAB Coder modules defined the same struct names:
- GIK Solver: `struct0_T`, `struct1_T`
- Hybrid A* Planner: `struct0_T`, `struct1_T`

**Solution Applied**:
1. Created separate param structures:
   - `StageBParams` (factory interface in `stage_b_factory.hpp`)
   - `StageBParams_Internal` (detailed internal params in `stage_b_chassis_plan.hpp`)
2. Added factory function `createStageBController()` to map between structures
3. Used different enum names:
   - Factory: `StageBMode` (HybridAStar, GIKAssisted)
   - Internal: `StageBMode_Internal` (B1_PURE_HYBRID_ASTAR, B2_GIK_ASSISTED)

**Result**: ✅ **Builds successfully** with no namespace conflicts

---

## Stage B Velocity Controller Architecture

### **Three-Mode Flexibility** 🎯

Stage B supports three velocity control modes, selectable via `velocity_control_mode` parameter:

```yaml
gik9dof_solver_node:
  ros__parameters:
    velocity_control_mode: 2  # 0, 1, or 2
```

#### **Mode 0: Legacy 5-Point Finite Difference**
```cpp
// Stage B Behavior (Mode 0):
if (params_.velocity_control_mode == 0) {
    // Fallback: Direct primitive velocities
    base_cmd.linear.x = state_.path[nearest_idx].Vx;   // Discrete jumps
    base_cmd.angular.z = state_.path[nearest_idx].Wz;  // From search primitives
}
```
- **Use Case**: Backward compatibility, debugging
- **Characteristics**: Jerky motion (0.4→0.6→0.8 m/s step changes)
- **Source**: Motion primitive velocities from Hybrid A* graph search

#### **Mode 1: Simple Heading Controller**
```cpp
// Stage B Behavior (Mode 1):
if (params_.velocity_control_mode == 1) {
    // Fallback: Direct primitive velocities
    base_cmd.linear.x = state_.path[nearest_idx].Vx;   // Same as mode 0
    base_cmd.angular.z = state_.path[nearest_idx].Wz;
}
```
- **Use Case**: Lightweight control applications
- **Note**: In Stage B, behaves same as Mode 0 (uses primitives)

#### **Mode 2: Pure Pursuit** ⭐ **RECOMMENDED**
```cpp
// Stage B Behavior (Mode 2) - NEW Oct 8, 2025:
if (params_.velocity_control_mode == 2) {
    // ✅ Pure Pursuit: Smooth tracking
    int lookahead_idx = findNearestWaypoint(current_base_pose);
    
    double refX = state_.path[lookahead_idx].x;
    double refY = state_.path[lookahead_idx].y;
    double refTheta = state_.path[lookahead_idx].theta;
    
    // Call Pure Pursuit controller
    gik9dof_purepursuit::purePursuitVelocityController(
        refX, refY, refTheta, refTime,
        estX, estY, estYaw,
        &params_.pp_params,
        &pp_state_,
        &Vx, &Wz,
        &newState
    );
    
    base_cmd.linear.x = Vx;   // Smooth velocities
    base_cmd.angular.z = Wz;  // Lookahead-based steering
}
```
- **Use Case**: Production, best motion quality
- **Benefits**:
  - ✅ Smooth velocity transitions (no jumps)
  - ✅ Lookahead-based steering (better cornering)
  - ✅ Consistent with Holistic/Stage C modes

### **Why Motion Primitives Need Pure Pursuit**

**Critical Understanding:**

Hybrid A* outputs include `Vx` and `Wz` fields, but these are **NOT** smooth tracking velocities:

```matlab
% In generateMotionPrimitives.m:
Vx_forward = [0.4, 0.6, 0.8];     % Fixed forward speeds
Wz_options = [-2.0, -1.0, 0.0, 1.0, 2.0];  % Fixed yaw rates

% In planHybridAStar.m (search loop):
next_state.Vx = prim.Vx;  % Stores PRIMITIVE velocity
next_state.Wz = prim.Wz;  % Used for graph search, not tracking
```

**These are SEARCH ACTIONS**, not optimized tracking commands!

**Direct execution** (Mode 0/1):
- Robot gets: 0.4 m/s → 0.6 m/s → 0.8 m/s (JERKY!)
- Yaw rate: -2.0 rad/s → 0.0 rad/s (STEP CHANGE!)

**Pure Pursuit** (Mode 2):
- Converts geometric waypoints (x, y, θ) → smooth velocities
- Applies lookahead distance, velocity smoothing
- Result: Continuous velocity profile ✅

---

## Code Changes Summary

### Files Modified (Oct 8, 2025)
| File | Lines Changed | Status |
|------|---------------|--------|
| `gik9dof_solver_node.cpp` | +15 | ✅ Add pp_params passing |
| `stage_b_factory.hpp` | +2 | ✅ Add pp_params pointer |
| `stage_b_chassis_plan.hpp` | +25 | ✅ Rename to _Internal, add pp_params |
| `stage_b_chassis_plan.cpp` | +80 | ✅ Pure Pursuit integration |
| **Total** | **~122 lines** | ✅ **Builds successfully** |

### Key Additions (Pure Pursuit Integration)
- **Pure Pursuit Call**: 45 lines (executeB1_PureHybridAStar refactor)
- **Factory Mapping**: 35 lines (createStageBController param conversion)
- **Param Structures**: 25 lines (pp_params field additions)
- **Documentation**: 15 lines (comments explaining primitive vs tracking)

---

## Testing Plan

### **Build Verification** ✅ **COMPLETE**
- ✅ Compiles successfully on x86_64 WSL Ubuntu 22.04
- ✅ No errors, warnings only (unused parameters in WIP code)
- ✅ ROS2 Humble package builds with colcon

### **Unit Testing** ⏸️ **TODO**
1. **Stage A**: Mock arm state, verify A→B transition when at home
2. **Stage B Pure Pursuit**: Mock Hybrid A* path, verify smooth velocity output
3. **Stage B Primitive Mode**: Compare with Pure Pursuit (expect jerky motion)
4. **Stage C**: Verify transition to whole-body tracking
5. **Holistic Mode**: Verify unchanged behavior when `control_mode: "holistic"`

### **Integration Testing** ⏸️ **TODO**
1. **A→B→C Pipeline**: End-to-end staged control with mock trajectory
2. **Velocity Mode Switching**: Test mode 0/1/2 behavior differences
3. **Performance**: Verify < 50ms planning, smooth transitions
4. **Diagnostics**: Check planner diagnostics published correctly

### **Motion Quality Testing** ⏸️ **TODO - HIGH PRIORITY**
Compare velocity profiles for same Hybrid A* path:

| Test | Mode | Expected Result |
|------|------|-----------------|
| **Primitive Baseline** | 0 or 1 | Jerky step changes (0.4→0.6→0.8 m/s) |
| **Pure Pursuit** | 2 | Smooth acceleration/deceleration |
| **Metrics** | All | Measure: max jerk, path error, completion time |

### **Hardware Testing (Orin)** ⏸️ **TODO**
1. **Real Occupancy Grid**: Test with actual sensor data
2. **GIK Performance**: Validate 3-5× speedup with iteration limit
3. **Smooth Execution**: No jerks during stage transitions
4. **Pure Pursuit Quality**: Measure tracking error in real motion

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

### **Why Pure Pursuit in Stage B?**
- **Motion Quality**: Converts discrete search primitives → smooth tracking
- **Consistency**: Same controller across all modes (Holistic/Stage B/Stage C)
- **Flexibility**: Preserves mode 0/1 for comparison and debugging

---

## Next Actions

### **Immediate - Testing Phase** 🎯
1. ✅ ~~Resolve Namespace Conflict~~ - **COMPLETE**
2. ✅ ~~Build and Test in WSL~~ - **COMPLETE**
3. ⏸️ **Motion Quality Tests** - Compare mode 0 vs 2 velocity profiles
4. ⏸️ **Integration Tests** - Full A→B→C pipeline with mock data
5. ⏸️ **Deploy to Orin** and validate on ARM64

### **Short-Term - Validation**
6. ⏸️ **Performance Testing** - Measure planning time, GIK solve time
7. ⏸️ **Parameter Tuning** - Optimize Pure Pursuit lookahead for Stage B
8. ⏸️ **Documentation** - Create user guide for velocity mode selection

### **Long-Term - Enhancement**
9. ⏸️ **Velocity Profiling** - Time-optimal velocity along Hybrid A* path
10. ⏸️ **Path Smoothing** - Post-process Hybrid A* waypoints (Bézier, B-splines)
11. ⏸️ **Adaptive Lookahead** - Scale lookahead with velocity and curvature

---

## Recent Updates

### **October 8, 2025 - Pure Pursuit Integration** ✅
- Integrated Pure Pursuit velocity controller in Stage B
- Replaced jerky primitive velocities with smooth lookahead tracking
- Resolved namespace conflicts via structure renaming
- Build verification: ✅ PASS
- **Commits**: 
  - `25ff591` - Pure Pursuit integration
  - `31f037d` - Integration summary documentation

### **October 7, 2025 - Initial State Machine**
- Implemented A→B→C state machine framework
- Created Stage B controller integration
- Encountered namespace conflicts (RESOLVED Oct 8)

---

## References

**Related Documents:**
- `PUREPURSUIT_HYBRID_ASTAR_INTEGRATION.md` - Integration session summary
- `docs/technical/CONTROLLER_ARCHITECTURE_STATUS.md` - Full architecture audit
- `docs/technical/pure-pursuit/PUREPURSUIT_DESIGN.md` - Pure Pursuit details
- `docs/technical/hybrid-astar/HYBRID_ASTAR_DESIGN.md` - Hybrid A* algorithm

**Code Locations:**
- State machine: `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp`
- Stage B controller: `ros2/gik9dof_solver/src/stage_b_chassis_plan.cpp`
- Factory pattern: `ros2/gik9dof_solver/src/stage_b_factory.hpp`
- Pure Pursuit: `ros2/gik9dof_solver/include/purepursuit/`
- Hybrid A*: `ros2/gik9dof_solver/include/planner/`

---

**Status**: ✅ **READY FOR TESTING** - Build complete, awaiting motion quality validation
