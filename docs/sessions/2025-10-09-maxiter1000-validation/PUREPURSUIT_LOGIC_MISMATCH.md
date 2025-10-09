# CRITICAL: Pure Pursuit Logic Mismatch Between MATLAB and ROS2

**Date**: October 8, 2025  
**Severity**: 🔴 **HIGH** - Manual C++ modifications found  
**Status**: ⚠️ **REQUIRES DECISION**

---

## Executive Summary

**CRITICAL FINDING**: The ROS2 C++ code has **manually added bidirectional support** that does NOT exist in the MATLAB wrapper function. If we copy the freshly generated C++ code to ROS2, we will **LOSE** the reverse/bidirectional capability.

---

## The Mismatch

### MATLAB Source (`purePursuitVelocityController.m`)
```matlab
%% Calculate velocities
% Forward velocity: nominal speed
vx = vxNominal;

% Reduce speed in sharp turns
curvatureMag = abs(curvature);
if curvatureMag > 0.5
    vx = vxNominal * 0.5;
elseif curvatureMag > 0.2
    vx = vxNominal * 0.7;
end

% Clamp forward velocity
vx = max(0.0, min(vxMax, vx));  % ❌ ALWAYS POSITIVE (forward only)
```

### Fresh Codegen Output (`matlab/codegen/purepursuit_arm64/`)
```cpp
// Calculate velocities
// Forward velocity: nominal speed
idx = params->vxNominal;
// Reduce speed in sharp turns
newNum = std::abs(dxSeg);
if (newNum > 0.5) {
  idx = params->vxNominal * 0.5;
} else if (newNum > 0.2) {
  idx = params->vxNominal * 0.7;
}
// Clamp forward velocity
*vx = std::fmax(0.0, std::fmin(params->vxMax, idx));  // ❌ ALWAYS POSITIVE
```

### Current ROS2 Code (MANUALLY MODIFIED)
```cpp
// BIDIRECTIONAL SUPPORT: Determine if we should move forward or reverse
// Check if lookahead point is primarily behind the robot
double vx_direction = 1.0;  // Default: forward
if (dxSeg < -0.3) {
  //  Lookahead point is significantly behind robot
  //  Use reverse motion
  vx_direction = -1.0;  // ✅ ALLOWS REVERSE
  //  When reversing, invert the steering
  dxSeg = -dxSeg;
}

// Calculate velocities
idx = params->vxNominal * vx_direction;  // ✅ Can be negative
// ... speed reduction logic ...

// Clamp velocity (BIDIRECTIONAL: allow negative for reverse)
if (vx_direction > 0.0) {
  //  Forward motion: clamp to [0, vxMax]
  *vx = std::fmax(0.0, std::fmin(params->vxMax, idx));
} else {
  //  Reverse motion: clamp to [vxMin, 0]  ✅ Uses vxMin parameter
  *vx = std::fmax(params->vxMin, std::fmin(0.0, idx));
}
```

---

## Impact Analysis

### If We Update ROS2 with Fresh Codegen

**LOST CAPABILITY**:
- ❌ Bidirectional motion (reverse driving)
- ❌ Smart detection of backward waypoints
- ❌ Uses `vxMin` parameter (negative velocities)

**RETAINED CAPABILITY**:
- ✅ Path buffering (30 waypoints)
- ✅ Adaptive lookahead
- ✅ Path interpolation
- ✅ Wheel speed limits

### Current ROS2 Capabilities

**ROS2 parameters declared**:
```cpp
// In gik9dof_solver_node.cpp
this->declare_parameter("purepursuit.vx_min", -1.0);  // ✅ Declared
```

**But MATLAB doesn't use it**:
```matlab
% In purePursuitVelocityController.m
vxMin = params.vxMin;  % ✅ Extracted but NEVER USED
```

---

## Code Comparison Summary

| Feature | MATLAB Wrapper | Fresh Codegen | ROS2 Deployed | Who Added? |
|---------|---------------|---------------|---------------|------------|
| Path buffer | ✅ Yes | ✅ Yes | ✅ Yes | MATLAB |
| Adaptive lookahead | ✅ Yes | ✅ Yes | ✅ Yes | MATLAB |
| Path interpolation | ✅ Yes | ✅ Yes | ✅ Yes | MATLAB |
| Wheel speed limits | ✅ Yes | ✅ Yes | ✅ Yes | MATLAB |
| Speed reduction in turns | ✅ Yes | ✅ Yes | ✅ Yes | MATLAB |
| **Bidirectional support** | ❌ **No** | ❌ **No** | ✅ **Yes** | **MANUAL** |
| Uses `vxMin` parameter | ❌ No | ❌ No | ✅ Yes | **MANUAL** |
| Reverse motion detection | ❌ No | ❌ No | ✅ Yes | **MANUAL** |

---

## Verification Commands

```powershell
# Check MATLAB wrapper
Get-Content matlab/purePursuitVelocityController.m | Select-String "vx_direction|BIDIRECTIONAL"
# Result: NO MATCHES

# Check fresh codegen
Get-Content matlab/codegen/purepursuit_arm64/purePursuitVelocityController.cpp | Select-String "vx_direction|BIDIRECTIONAL"
# Result: NO MATCHES

# Check deployed ROS2 code
Get-Content ros2/gik9dof_solver/src/purepursuit/purePursuitVelocityController.cpp | Select-String "vx_direction|BIDIRECTIONAL"
# Result: MATCHES FOUND (lines ~275-290)
```

---

## Decision Required

### Option 1: Keep Manual Modifications (Recommended)
**Action**: Do NOT update ROS2 C++ code with fresh codegen  
**Pros**:
- Retain bidirectional capability
- Keep working reverse motion
- No functionality loss

**Cons**:
- MATLAB wrapper and C++ diverge
- Future MATLAB improvements won't auto-transfer
- Need to manually port MATLAB changes

**Recommendation**: ⭐ **Use this if bidirectional motion is required**

---

### Option 2: Add Bidirectional Support to MATLAB
**Action**: Update `purePursuitVelocityController.m` with bidirectional logic, then regenerate

**Code to add** (around line 280 in MATLAB):
```matlab
%% BIDIRECTIONAL SUPPORT: Determine direction
% Check if lookahead point is behind robot
vx_direction = 1.0;  % Default: forward
if xLookahead < -0.3
    % Lookahead point significantly behind robot
    vx_direction = -1.0;  % Reverse
    curvature = -curvature;  % Invert steering
end

%% Calculate velocities
vx = vxNominal * vx_direction;  % Can be negative

% Reduce speed in sharp turns
curvatureMag = abs(curvature);
if curvatureMag > 0.5
    vx = vxNominal * 0.5 * vx_direction;
elseif curvatureMag > 0.2
    vx = vxNominal * 0.7 * vx_direction;
end

% Clamp velocity (bidirectional)
if vx_direction > 0.0
    vx = max(0.0, min(vxMax, vx));  % Forward: [0, vxMax]
else
    vx = max(vxMin, min(0.0, vx));  % Reverse: [vxMin, 0]
end
```

**Pros**:
- MATLAB and C++ stay in sync
- Future improvements transfer automatically
- Proper software engineering

**Cons**:
- Requires MATLAB code modification
- Need to regenerate C++ code
- Need to test MATLAB version
- Estimated effort: 2-3 hours

**Recommendation**: ⭐ **Use this for long-term maintainability**

---

### Option 3: Update C++ and Accept Feature Loss
**Action**: Copy fresh codegen to ROS2, lose bidirectional

**Pros**:
- MATLAB and C++ in sync
- Simpler maintenance

**Cons**:
- ❌ **LOSE bidirectional motion**
- ❌ **LOSE reverse capability**
- May break existing functionality

**Recommendation**: ❌ **NOT RECOMMENDED** (unless you don't need reverse)

---

## Questions for User

1. **Do you need bidirectional (forward/reverse) motion?**
   - If YES → Option 1 (keep manual) or Option 2 (add to MATLAB)
   - If NO → Option 3 (accept fresh codegen)

2. **When was bidirectional support added to C++ and why?**
   - Check commit history
   - Was it tested?
   - Is it production-critical?

3. **Do you use reverse motion in Stage B?**
   - Check if any trajectories require backing up
   - Check if Hybrid A* can generate reverse segments

---

## Recommendation

**If bidirectional motion is required** (which seems likely given someone manually added it):

1. **Short-term**: Keep manual modifications in ROS2 (Option 1)
2. **Long-term**: Add bidirectional support to MATLAB wrapper (Option 2)

**If bidirectional NOT needed**:
- Use fresh codegen (Option 3)

---

## Files Involved

### MATLAB Source
- `matlab/purePursuitVelocityController.m` (wrapper, codegen target)

### Fresh Codegen Output
- `matlab/codegen/purepursuit_x64/purePursuitVelocityController.cpp`
- `matlab/codegen/purepursuit_arm64/purePursuitVelocityController.cpp`

### Deployed ROS2 Code (MANUALLY MODIFIED)
- `ros2/gik9dof_solver/src/purepursuit/purePursuitVelocityController.cpp`
- `ros2/gik9dof_solver/include/purepursuit/purePursuitVelocityController.h`

### Integration
- `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp` (declares `vxMin` parameter)
- `ros2/gik9dof_solver/src/stage_b_chassis_plan.cpp` (calls controller)

---

## Next Steps

**REQUIRED**: User must decide on approach before proceeding.

**If Option 1 (keep manual)**:
- Document the manual modifications
- Create process for porting future MATLAB changes

**If Option 2 (add to MATLAB)**:
- Modify `purePursuitVelocityController.m`
- Test in MATLAB
- Regenerate C++ code
- Deploy and test in ROS2

**If Option 3 (accept loss)**:
- Copy fresh codegen to ROS2
- Remove `vxMin` parameter declarations
- Test forward-only motion

---

## Status

⚠️ **BLOCKED**: Awaiting user decision on bidirectional support

**See also**:
- `PUREPURSUIT_CODEGEN_STATUS.md` (original analysis - now INCOMPLETE)
- `MERGE_COMPLETE_SUMMARY.md` (merge summary - needs update)
