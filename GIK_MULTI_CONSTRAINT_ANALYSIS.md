# GIK Multi-Constraint Support Analysis

**Date:** 2025-01-08  
**Question:** Does our GIK support multiple distance constraints?

---

## Answer: YES in MATLAB, NO in Generated C++

### MATLAB Simulation: ✅ Supports Multiple Distance Constraints

**File:** `matlab/+gik9dof/createGikSolver.m`

**Implementation:**
```matlab
% Lines 58: accepts struct array
options.DistanceSpecs = struct([])

% Lines 87-98: Creates multiple distance constraint objects
distanceSpecs = buildDistanceSpecs(robot, options);
distanceConst = cell(1, numel(distanceSpecs));  % Cell array for multiple constraints
for i = 1:numel(distanceSpecs)
    spec = distanceSpecs(i);
    dc = constraintDistanceBounds(char(spec.Body));
    dc.ReferenceBody = char(spec.ReferenceBody);
    dc.Bounds = spec.Bounds;
    dc.Weights = spec.Weight;
    distanceConst{i} = dc;  % Store in cell array
end
```

**Features:**
- ✅ Accepts `DistanceSpecs` as **struct array**
- ✅ Creates **multiple** `constraintDistanceBounds` objects
- ✅ Each constraint can have different:
  - Body name
  - Reference body
  - Distance bounds [lower, upper]
  - Weight
- ✅ All constraints passed to GIK solver together

**Example Usage:**
```matlab
% Define multiple distance constraints
distSpecs = [];
distSpecs(1).Body = 'left_gripper_link';
distSpecs(1).ReferenceBody = 'base_link';
distSpecs(1).Bounds = [0.2, Inf];
distSpecs(1).Weight = 0.5;

distSpecs(2).Body = 'left_upper_arm_link';
distSpecs(2).ReferenceBody = 'base_link';
distSpecs(2).Bounds = [0.3, Inf];
distSpecs(2).Weight = 0.3;

bundle = gik9dof.createGikSolver(robot, 'DistanceSpecs', distSpecs);
```

---

### Generated C++ Code: ❌ Only ONE Distance Constraint

**File:** `ros2/gik9dof_solver/src/generated/solveGIKStepWithLock.cpp`

**Implementation:**
```cpp
// Lines 32-34: Single scalar parameters
void solveGIKStepWithLock(const double qCurrent[9], const double targetPose[16],
                          double distanceLower, double distanceWeight,  // Single values!
                          const bool lockMask[9], double qNext[9])

// Lines 44-50: Single constraint object
distanceConstraint.init();  // ONE constraint
distanceConstraint.ReferenceBody.reserve(200);
b_robot->get_BaseName((char *)distanceConstraint.ReferenceBody.data(), b_iv);

// Lines 62-68: Updates single constraint
distanceConstraint.Bounds[0] = distanceLower;
distanceConstraint.Bounds[1] = rtInf;
distanceConstraint.Weights = distanceWeight;

// Line 85: Passes single constraint
solver.step(qCurrent, poseConstraint, jointConstraint, distanceConstraint, qNext);
```

**Limitations:**
- ❌ Only **ONE** distance constraint object
- ❌ Fixed body: `left_gripper_link` (hardcoded)
- ❌ Fixed reference: `robot.BaseName` (base_link)
- ❌ Only lower bound is configurable (upper always Inf)
- ❌ Single weight value

---

## Why the Difference?

### MATLAB Coder Limitations

MATLAB Coder has difficulty generating C++ code for:
1. **Cell arrays of objects** (`distanceConst{i}`)
2. **Variable-length constraint lists**
3. **Dynamic object creation in loops**

The generated C++ code simplified to a **single, hardcoded constraint** to make code generation feasible.

### Manual C++ Enhancement Needed

To support multiple distance constraints in C++, you would need to:

1. **Modify function signature** to accept arrays:
```cpp
void solveGIKStepWithLock(
    const double qCurrent[9], 
    const double targetPose[16],
    int numDistanceConstraints,
    const char** distanceBodies,
    const char** distanceRefBodies,
    const double* distanceLowers,
    const double* distanceWeights,
    const bool lockMask[9], 
    double qNext[9])
```

2. **Create constraint array**:
```cpp
std::vector<robotics::manip::manipcore::DistanceBoundsConstraint> distanceConstraints;
for (int i = 0; i < numDistanceConstraints; i++) {
    robotics::manip::manipcore::DistanceBoundsConstraint dc;
    dc.init();
    dc.ReferenceBody = distanceRefBodies[i];
    dc.Bounds[0] = distanceLowers[i];
    dc.Bounds[1] = rtInf;
    dc.Weights = distanceWeights[i];
    distanceConstraints.push_back(dc);
}
```

3. **Update solver call** to pass multiple constraints

This is **manual work** that cannot be auto-generated from current MATLAB code.

---

## Current Production Status

### What's Deployed in ROS2

**Current C++ implementation:** ONE distance constraint only
- Body: `left_gripper_link`
- Reference: `base_link`
- Configurable: Lower bound and weight only

**ROS2 Interface:**
```cpp
// In gik9dof_solver_node.cpp
double distance_lower = distance_constraint_active_ ? min_ground_clearance_ : 0.0;
double distance_weight = distance_constraint_active_ ? distance_constraint_weight_ : 0.0;

codegen::solveGIKStepWithLock(q_current, target_pose, 
                               distance_lower, distance_weight,  // Single values
                               lock_mask, q_next);
```

**Parameters available:**
- `min_ground_clearance` (double) - Lower bound
- `distance_constraint_weight` (double) - Weight
- `distance_constraint_active` (bool) - On/Off

---

## Use Cases

### What Current System Can Do ✅

1. **Keep gripper above ground**
   - Single constraint: `left_gripper_link` to `base_link`
   - Minimum distance: `min_ground_clearance` (e.g., 0.2m)

### What Current System CANNOT Do ❌

1. **Keep multiple bodies clear**
   - ❌ Can't constrain `left_upper_arm_link` simultaneously
   - ❌ Can't constrain `left_forearm_link` simultaneously
   
2. **Different distance requirements**
   - ❌ Can't set different bounds for different bodies
   - ❌ Can't set different weights for different constraints

3. **Obstacle avoidance for multiple points**
   - ❌ Can't avoid multiple obstacles with separate constraints

---

## Recommendation

### For Production Use

**Current system is sufficient for:**
- Typical mobile manipulator tasks
- Ground clearance for end-effector
- Single primary safety constraint

**If you need multiple distance constraints:**

### Option 1: Manual C++ Implementation (Recommended if needed)
1. Write custom C++ code (not from MATLAB Coder)
2. Support multiple constraint specification
3. More flexible than MATLAB Coder output
4. Requires C++ development and testing

### Option 2: Use MATLAB for Planning (Workaround)
1. Use MATLAB simulation with multiple constraints for **offline planning**
2. Generate waypoint trajectories
3. Execute trajectories in ROS2 with single constraint active
4. Less real-time flexibility but no C++ changes needed

### Option 3: Sequential Constraints (Workaround)
1. Use different constraints at different stages
2. Example: Stage A (arm motion) uses one constraint, Stage B (base motion) uses different settings
3. Limited but works within current framework

---

## Summary

| Feature | MATLAB Simulation | Generated C++ | Manual C++ |
|---------|------------------|---------------|------------|
| Multiple distance constraints | ✅ Yes | ❌ No | ✅ Possible |
| Configurable bodies | ✅ Yes | ❌ No | ✅ Possible |
| Configurable references | ✅ Yes | ❌ No | ✅ Possible |
| Different bounds per constraint | ✅ Yes | ❌ No | ✅ Possible |
| Different weights per constraint | ✅ Yes | ❌ No | ✅ Possible |
| Auto-generated from MATLAB | ✅ Yes | ❌ Limited | ❌ No |

**Bottom Line:**
- MATLAB simulation has full multi-constraint support
- Generated C++ is simplified to single constraint (MATLAB Coder limitation)
- If you need multi-constraint in production, it requires manual C++ development

---

## Files Referenced

- `matlab/+gik9dof/createGikSolver.m` - MATLAB multi-constraint implementation
- `ros2/gik9dof_solver/src/generated/solveGIKStepWithLock.cpp` - Generated C++ (single constraint)
- `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp` - ROS2 wrapper (single constraint interface)

---

## Related Documentation

- `GIK_CODEGEN_ANALYSIS.md` - Analysis of what can/cannot be regenerated
- `PUREPURSUIT_CODEGEN_STATUS_NOTE.md` - Comparison with Pure Pursuit codegen
