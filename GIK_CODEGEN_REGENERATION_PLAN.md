# GIK Solver C++ Code Generation Plan

**Date:** 2025-01-08  
**Objective:** Regenerate GIK solver C++ code using MATLAB Coder with proper constraint and obstacle handling based on official MATLAB documentation

---

## Current State Analysis

### What Works in MATLAB Simulation ✅

**File:** `matlab/+gik9dof/createGikSolver.m`

**Features:**
1. **Multiple distance constraints** via `DistanceSpecs` struct array
2. **Collision mesh handling** via `collisionTools.m`
3. **Flexible constraint configuration**:
   - Pose target (position + orientation)
   - Joint bounds (soft limits)
   - Distance bounds (obstacle avoidance)
   - Optional aiming constraint
4. **Dynamic constraint management** with tunable weights

### What's in Generated C++ Currently ❌

**File:** `ros2/gik9dof_solver/src/generated/solveGIKStepWithLock.cpp`

**Limitations:**
1. **Single distance constraint** only (hardcoded)
2. **No collision mesh support**
3. **Fixed body configuration** (`left_gripper_link`)
4. **Simplified** from full MATLAB capability

---

## MATLAB Official Documentation References

### Key Concepts from MATLAB Documentation

#### 1. generalizedInverseKinematics (GIK)

**Official MATLAB Documentation:**
> The `generalizedInverseKinematics` object solves inverse kinematics problems by minimizing a cost function derived from the violation of all active constraints.

**Source:** MATLAB Robotics System Toolbox Documentation

**Key Properties:**
- `RigidBodyTree` - Robot model
- `SolverAlgorithm` - Optimization algorithm ('BFGSGradientProjection', 'LevenbergMarquardt')
- `ConstraintInputs` - Cell array defining constraint order
- `SolverParameters` - Tuning parameters (MaxIterations, etc.)

#### 2. Constraint Types

**constraintPoseTarget**
```matlab
poseTgt = constraintPoseTarget('end_effector_name');
poseTgt.TargetTransform = [4x4 homogeneous transform];
poseTgt.Weights = [translation_weight, orientation_weight];
```

**constraintJointBounds**
```matlab
jointConst = constraintJointBounds(robot);
jointConst.Bounds = [lower_limits; upper_limits];  % 2 x NumJoints
jointConst.Weights = weights_vector;  % 1 x NumJoints
```

**constraintDistanceBounds**
```matlab
distConst = constraintDistanceBounds('body_name');
distConst.ReferenceBody = 'reference_body_name';
distConst.Bounds = [lower_bound, upper_bound];
distConst.Weights = weight_scalar;
```

#### 3. MATLAB Coder Requirements

**From MATLAB Documentation:**
> For code generation, the solver object must be declared as `persistent` so it is initialized only once.

**Code Generation Pattern:**
```matlab
function q_next = solve_gik_step(q_current, target_pose, ...)
    %#codegen
    persistent gik poseTgt jointConst distConst robot
    
    if isempty(gik)
        % Initialize once
        robot = loadRobotForCodegen();  % Must be procedural
        gik = generalizedInverseKinematics('RigidBodyTree', robot);
        gik.ConstraintInputs = {'pose', 'joint', 'distance'};
        
        poseTgt = constraintPoseTarget('left_gripper_link');
        jointConst = constraintJointBounds(robot);
        distConst = constraintDistanceBounds('left_gripper_link');
        distConst.ReferenceBody = robot.BaseName;
    end
    
    % Update constraints
    poseTgt.TargetTransform = target_pose;
    % ... update other constraints ...
    
    % Solve
    [q_next, info] = gik(q_current, poseTgt, jointConst, distConst);
end
```

---

## Challenges for Code Generation

### What MATLAB Coder CANNOT Handle

Based on official documentation and your current code:

#### ❌ Cell Arrays of Constraint Objects
```matlab
% This does NOT work with codegen
distanceConst = cell(1, numel(distanceSpecs));
for i = 1:numel(distanceSpecs)
    distanceConst{i} = constraintDistanceBounds(body);
end
```

**Why:** MATLAB Coder cannot generate code for variable-length cell arrays of objects

**Workaround:** Fixed number of constraints or single constraint

#### ❌ Dynamic Constraint Addition
```matlab
% This does NOT work with codegen
constraints = {poseTgt, jointConst};
if ~isempty(distanceConst)
    constraints = [constraints, distanceConst];  % Dynamic concatenation
end
[qNext, info] = gik(q0, constraints{:});
```

**Why:** Variable number of input arguments not supported

**Workaround:** Always pass all constraints, use zero weights to disable

#### ❌ Runtime File Loading
```matlab
% This does NOT work with codegen
robot = loadRobot('mobile_manipulator.urdf');
```

**Why:** File I/O is limited, rigidBodyTree construction from file not supported

**Workaround:** Procedural robot building (see below)

#### ❌ Collision Mesh Attachment
```matlab
% This does NOT work with codegen
addCollision(body, collisionMesh('meshes/link.stl'));
```

**Why:** Runtime mesh loading not supported

**Workaround:** Use collision primitives (boxes, spheres, cylinders) defined procedurally

---

## Proposed Solution Architecture

### Option A: Fixed Multi-Constraint (Recommended)

**Concept:** Support up to N fixed distance constraints (e.g., 3-5), use weights to enable/disable

**Implementation:**
```matlab
function [qNext, solverInfo] = solveGIKMultiConstraint(robot, solver, qCurrent, targetPose, constraintParams)
%SOLVEGIK_MULTICONSTRAINT GIK with up to 5 distance constraints
%   constraintParams structure:
%     .numDistanceConstraints (1-5)
%     .distanceBodies (1x5 cell array of strings)
%     .distanceRefBodies (1x5 cell array of strings)
%     .distanceLowerBounds (1x5 double array)
%     .distanceWeights (1x5 double array)
%
%#codegen

persistent poseConst jointConst distConst1 distConst2 distConst3 distConst4 distConst5

if isempty(poseConst)
    % Initialize once
    poseConst = constraintPoseTarget('left_gripper_link');
    jointConst = constraintJointBounds(robot);
    
    % Create 5 fixed distance constraints
    distConst1 = constraintDistanceBounds('left_gripper_link');
    distConst1.ReferenceBody = robot.BaseName;
    
    distConst2 = constraintDistanceBounds('left_gripper_link');
    distConst2.ReferenceBody = robot.BaseName;
    
    distConst3 = constraintDistanceBounds('left_upper_arm_link');
    distConst3.ReferenceBody = robot.BaseName;
    
    distConst4 = constraintDistanceBounds('left_forearm_link');
    distConst4.ReferenceBody = robot.BaseName;
    
    distConst5 = constraintDistanceBounds('left_arm_base_link');
    distConst5.ReferenceBody = robot.BaseName;
end

% Update pose target
poseConst.TargetTransform = targetPose;

% Update distance constraints with weights (0 = disabled)
numActive = constraintParams.numDistanceConstraints;

% Constraint 1
if numActive >= 1
    distConst1.Bounds = [constraintParams.distanceLowerBounds(1), 100.0];
    distConst1.Weights = constraintParams.distanceWeights(1);
else
    distConst1.Weights = 0.0;
end

% Constraint 2
if numActive >= 2
    distConst2.Bounds = [constraintParams.distanceLowerBounds(2), 100.0];
    distConst2.Weights = constraintParams.distanceWeights(2);
else
    distConst2.Weights = 0.0;
end

% ... repeat for constraints 3-5 ...

% Solve (always pass all 5 constraints)
[qNext, solverInfo] = solver(qCurrent, poseConst, jointConst, ...
                              distConst1, distConst2, distConst3, distConst4, distConst5);
end
```

**Advantages:**
- ✅ Supports multiple distance constraints
- ✅ Compatible with MATLAB Coder
- ✅ Flexible via weight control
- ✅ No dynamic memory allocation

**Disadvantages:**
- ⚠️ Fixed maximum number of constraints
- ⚠️ Bodies are hardcoded (cannot change at runtime)

### Option B: Current Single Constraint (Simple)

**Keep current implementation** with improvements:

```matlab
function [qNext, solverInfo] = solveGIKStepRealtime(robot, solver, qCurrent, targetPose, ...
                                                     distanceLower, distanceWeight)
%#codegen
% (Current implementation - already works)
```

**Advantages:**
- ✅ Already working
- ✅ Simple and proven
- ✅ Sufficient for primary use case

**Disadvantages:**
- ❌ Only one distance constraint

---

## Procedural Robot Builder for Codegen

**Problem:** Cannot load URDF at codegen time

**Solution:** Build robot procedurally

**File:** `matlab/+gik9dof/+codegen_inuse/loadRobotForCodegen.m`

```matlab
function robot = loadRobotForCodegen()
%LOADROBOTFORCODEGEN Procedurally construct the 9-DOF robot for code generation
%   This function builds the robot model programmatically without file I/O,
%   making it compatible with MATLAB Coder.
%
%#codegen

% Create base robot tree
robot = rigidBodyTree('DataFormat', 'column');

% Base joints (3 DOF: x, y, theta)
baseX = rigidBody('base_x_link');
jntX = rigidBodyJoint('base_x_joint', 'prismatic');
setFixedTransform(jntX, trvec2tform([0 0 0]));
jntX.JointAxis = [1 0 0];
jntX.PositionLimits = [-5 5];
baseX.Joint = jntX;
addBody(robot, baseX, robot.BaseName);

baseY = rigidBody('base_y_link');
jntY = rigidBodyJoint('base_y_joint', 'prismatic');
setFixedTransform(jntY, trvec2tform([0 0 0]));
jntY.JointAxis = [0 1 0];
jntY.PositionLimits = [-5 5];
baseY.Joint = jntY;
addBody(robot, baseY, 'base_x_link');

baseTheta = rigidBody('abstract_chassis_link');
jntTheta = rigidBodyJoint('base_theta_joint', 'revolute');
setFixedTransform(jntTheta, trvec2tform([0 0 0]));
jntTheta.JointAxis = [0 0 1];
jntTheta.PositionLimits = [-pi pi];
baseTheta.Joint = jntTheta;
addBody(robot, baseTheta, 'base_y_link');

% Arm base (fixed)
armBase = rigidBody('left_arm_base_link');
armBase.Joint = rigidBodyJoint('arm_base_fixed', 'fixed');
setFixedTransform(armBase.Joint, trvec2tform([0.184 0 0.251]));
addBody(robot, armBase, 'abstract_chassis_link');

% Arm joints (6 DOF)
% Joint 1
link1 = rigidBody('left_arm_link1');
jnt1 = rigidBodyJoint('left_arm_joint1', 'revolute');
setFixedTransform(jnt1, trvec2tform([0 0 0.061]));
jnt1.JointAxis = [0 0 1];
jnt1.PositionLimits = deg2rad([-175 175]);
link1.Joint = jnt1;
addBody(robot, link1, 'left_arm_base_link');

% Joint 2
link2 = rigidBody('left_arm_link2');
jnt2 = rigidBodyJoint('left_arm_joint2', 'revolute');
setFixedTransform(jnt2, trvec2tform([0 0 0]));
jnt2.JointAxis = [0 1 0];
jnt2.PositionLimits = deg2rad([-85 155]);
link2.Joint = jnt2;
addBody(robot, link2, 'left_arm_link1');

% Joint 3
link3 = rigidBody('left_arm_link3');
jnt3 = rigidBodyJoint('left_arm_joint3', 'revolute');
setFixedTransform(jnt3, trvec2tform([0.203 0 0]));
jnt3.JointAxis = [0 1 0];
jnt3.PositionLimits = deg2rad([-120 155]);
link3.Joint = jnt3;
addBody(robot, link3, 'left_arm_link2');

% Joint 4
link4 = rigidBody('left_arm_link4');
jnt4 = rigidBodyJoint('left_arm_joint4', 'revolute');
setFixedTransform(jnt4, trvec2tform([0 0 0]));
jnt4.JointAxis = [0 0 1];
jnt4.PositionLimits = deg2rad([-175 175]);
link4.Joint = jnt4;
addBody(robot, link4, 'left_arm_link3');

% Joint 5
link5 = rigidBody('left_arm_link5');
jnt5 = rigidBodyJoint('left_arm_joint5', 'revolute');
setFixedTransform(jnt5, trvec2tform([0.1515 0 0]));
jnt5.JointAxis = [0 1 0];
jnt5.PositionLimits = deg2rad([-100 90]);
link5.Joint = jnt5;
addBody(robot, link5, 'left_arm_link4');

% Joint 6
link6 = rigidBody('left_arm_link6');
jnt6 = rigidBodyJoint('left_arm_joint6', 'revolute');
setFixedTransform(jnt6, trvec2tform([0 0 0]));
jnt6.JointAxis = [0 0 1];
jnt6.PositionLimits = deg2rad([-175 175]);
link6.Joint = jnt6;
addBody(robot, link6, 'left_arm_link5');

% End-effector (gripper)
gripper = rigidBody('left_gripper_link');
gripper.Joint = rigidBodyJoint('gripper_fixed', 'fixed');
setFixedTransform(gripper.Joint, trvec2tform([0.0855 0 0]));
addBody(robot, gripper, 'left_arm_link6');

% Optional: Add collision primitives (boxes/spheres/cylinders)
% These must be added procedurally, not from mesh files
% Example for arm base:
% collision = collisionBox(0.12, 0.12, 0.10);
% collision.Pose = trvec2tform([0 0 0.05]);
% addCollision(armBase, collision);

end
```

---

## Implementation Steps

### Phase 1: Prepare MATLAB Code (Week 1)

#### Step 1.1: Create Procedural Robot Builder ✅
- [x] File: `matlab/+gik9dof/+codegen_inuse/loadRobotForCodegen.m`
- [x] Already exists but verify all dimensions match URDF

#### Step 1.2: Decide on Constraint Strategy
- [ ] **Option A:** Implement fixed multi-constraint (5 constraints)
- [ ] **Option B:** Keep single constraint, improve interface

#### Step 1.3: Create New Codegen Entry Point
```matlab
% File: matlab/+gik9dof/+codegen_inuse/solveGIKStepMultiConstraint.m
% (based on chosen option)
```

#### Step 1.4: Add Collision Primitives (Optional)
```matlab
% Add procedural collision boxes/spheres/cylinders to critical links
% Cannot use mesh files - must be geometric primitives
```

### Phase 2: Test in MATLAB (Week 1)

#### Step 2.1: Validate Procedural Robot
```matlab
robot = gik9dof.codegen_inuse.loadRobotForCodegen();
% Verify:
% - 9 DOF
% - Joint limits match URDF
% - Forward kinematics matches
% - Base name correct
```

#### Step 2.2: Test New Solver Function
```matlab
% Test with coder.screener
coder.screener('gik9dof.codegen_inuse.solveGIKStepMultiConstraint');

% Test functionally
[qNext, info] = gik9dof.codegen_inuse.solveGIKStepMultiConstraint(...);
```

### Phase 3: Generate C++ Code (Week 2)

#### Step 3.1: Create Codegen Script
```matlab
% File: matlab/generate_code_gik_multiconstraint.m

% Define input types
robot_type = coder.typeof(loadRobotForCodegen());
solver_type = coder.typeof(generalizedInverseKinematics('RigidBodyTree', robot_type));
qCurrent_type = coder.typeof(zeros(9,1));
targetPose_type = coder.typeof(eye(4));
constraintParams_type = struct();
constraintParams_type.numDistanceConstraints = coder.typeof(uint8(0));
constraintParams_type.distanceLowerBounds = coder.typeof(zeros(1,5));
constraintParams_type.distanceWeights = coder.typeof(zeros(1,5));

% Configure for ARM64
cfg_arm64 = coder.config('lib');
cfg_arm64.TargetLang = 'C++';
cfg_arm64.CppNamespace = 'gik9dof';
cfg_arm64.GenCodeOnly = true;
cfg_arm64.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';

% Generate
codegen -config cfg_arm64 gik9dof.codegen_inuse.solveGIKStepMultiConstraint ...
    -args {robot_type, solver_type, qCurrent_type, targetPose_type, constraintParams_type} ...
    -d codegen/gik_multiconstraint_arm64 -report
```

#### Step 3.2: Handle Codegen Errors
- Fix type inconsistencies (like Pure Pursuit uint32 issues)
- Simplify any non-codegen-compatible constructs
- May need to create wrapper functions for complex MATLAB types

### Phase 4: Integrate into ROS2 (Week 2-3)

#### Step 4.1: Update ROS2 Wrapper
```cpp
// In gik9dof_solver_node.cpp

// Define constraint parameters structure
struct ConstraintParams {
    uint8_t numDistanceConstraints;
    double distanceLowerBounds[5];
    double distanceWeights[5];
};

// Update solver call
ConstraintParams params;
params.numDistanceConstraints = active_constraint_count_;
params.distanceLowerBounds[0] = min_ground_clearance_;
params.distanceWeights[0] = distance_constraint_weight_;
// ... configure other constraints ...

codegen::solveGIKStepMultiConstraint(robot, solver, q_current, target_pose, 
                                      params, q_next, &solver_info);
```

#### Step 4.2: Add ROS2 Parameters
```yaml
# config/solver_params.yaml
distance_constraints:
  count: 2  # Number of active constraints
  
  constraint_0:
    body: "left_gripper_link"
    reference: "base_link"
    lower_bound: 0.2
    weight: 0.5
    
  constraint_1:
    body: "left_upper_arm_link"
    reference: "base_link"
    lower_bound: 0.3
    weight: 0.3
```

#### Step 4.3: Test and Validate
- [ ] Build ROS2 package
- [ ] Run existing test cases
- [ ] Verify multi-constraint behavior
- [ ] Performance benchmarking

---

## Expected Outcomes

### With Fixed Multi-Constraint (Option A)

**Capabilities:**
- ✅ Up to 5 simultaneous distance constraints
- ✅ Per-constraint weight control
- ✅ Different bodies can have different bounds
- ✅ Enable/disable via weights
- ✅ Full MATLAB Coder compatibility

**Limitations:**
- ⚠️ Maximum 5 constraints (hardcoded)
- ⚠️ Body names are fixed in code (cannot change at runtime)
- ⚠️ No collision meshes (only if using primitives)

### Performance Expectations

Based on similar systems:
- **Solve time:** 1-5 ms per iteration (ARM64)
- **Real-time capable:** Yes, at 100 Hz
- **Memory:** Static allocation only (codegen requirement)

---

## Documentation to Reference

### Official MATLAB Documentation

1. **generalizedInverseKinematics**
   - https://www.mathworks.com/help/robotics/ref/generalizedinversekinematics.html

2. **MATLAB Coder for Robotics**
   - https://www.mathworks.com/help/robotics/ug/code-generation-for-path-planning-and-vehicle-control.html

3. **Constraint Objects:**
   - constraintPoseTarget: https://www.mathworks.com/help/robotics/ref/constraintposetarget.html
   - constraintJointBounds: https://www.mathworks.com/help/robotics/ref/constraintjointbounds.html
   - constraintDistanceBounds: https://www.mathworks.com/help/robotics/ref/constraintdistancebounds.html

4. **rigidBodyTree Code Generation**
   - https://www.mathworks.com/help/robotics/ref/rigidbodytree.html
   - Note: Must be built procedurally for codegen

### Project Documentation

1. **Current Implementation:** `guideline.md`
2. **Multi-Constraint Analysis:** `GIK_MULTI_CONSTRAINT_ANALYSIS.md`
3. **Pure Pursuit Success:** `PUREPURSUIT_CODEGEN_SUCCESS.md` (lessons learned)

---

## Risk Mitigation

### Known Challenges

**Challenge 1: MATLAB Coder Compatibility**
- **Risk:** Complex MATLAB code may not be codegen-compatible
- **Mitigation:** Start with simple case, incrementally add features
- **Fallback:** Keep manual C++ option available

**Challenge 2: Type Consistency**
- **Risk:** Type mismatches like Pure Pursuit (uint32 vs double)
- **Mitigation:** Use lessons from Pure Pursuit codegen success
- **Mitigation:** Extensive type checking with `coder.screener`

**Challenge 3: Performance**
- **Risk:** Generated code may be slower than manual C++
- **Mitigation:** Benchmark early, optimize if needed
- **Fallback:** Manual C++ for critical sections

---

## Success Criteria

### Minimum Viable Product (MVP)

- ✅ Generate C++ code successfully (ARM64 + x64)
- ✅ Support at least 2-3 distance constraints
- ✅ Pass all existing ROS2 test cases
- ✅ Performance ≤ 5ms per solve @ 100Hz

### Full Success

- ✅ Support 5 distance constraints
- ✅ Configurable via ROS2 parameters
- ✅ Documentation complete
- ✅ Performance ≤ 3ms per solve
- ✅ Validated on hardware

---

## Next Steps

1. **Review this plan** with team
2. **Choose Option A or B** for constraint handling
3. **Verify procedural robot builder** matches URDF exactly
4. **Create codegen entry point** based on chosen option
5. **Test with coder.screener** before full codegen
6. **Follow Pure Pursuit debugging process** if errors occur

---

## References

- MATLAB Robotics System Toolbox Documentation
- MATLAB Coder Documentation
- `GIK_MULTI_CONSTRAINT_ANALYSIS.md`
- `PUREPURSUIT_CODEGEN_SUCCESS.md`
- Your project's `guideline.md`

---

**Status:** 📋 PLAN READY - Awaiting decision on Option A vs Option B
