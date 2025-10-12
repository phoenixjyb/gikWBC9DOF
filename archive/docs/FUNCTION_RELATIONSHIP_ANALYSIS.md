# Function Relationship Analysis: runTrajectoryControl vs simulateChassisExecution

**Date:** October 12, 2025  
**Question:** Are `runTrajectoryControl` and `simulateChassisExecution` duplicated? Can we use a single one?  
**Answer:** ❌ **NO - They serve COMPLETELY DIFFERENT purposes and are NOT duplicated**

---

## Executive Summary

These two functions operate at **different levels of abstraction** and solve **fundamentally different problems**:

| Function | Purpose | Input | Output | Controller Type |
|----------|---------|-------|--------|-----------------|
| **runTrajectoryControl** | **IK control loop** | EE poses (SE(3)) | Joint trajectory (qTraj) | **GIK solver** (whole-body IK) |
| **simulateChassisExecution** | **Chassis simulation** | Base waypoints (x,y,θ) | Base commands (Vx, Wz) | **Pure pursuit / heading** (chassis only) |

**They are complementary, not duplicated** - they are used together in the three-pass architecture (ppForIk mode).

---

## Detailed Comparison

### 1. runTrajectoryControl - Whole-Body IK Control Loop

**File:** `matlab/+gik9dof/runTrajectoryControl.m` (518 lines)

#### What It Does
- **Streams end-effector poses** through a GIK (Generalized Inverse Kinematics) solver
- Solves **whole-body inverse kinematics** to find joint angles that achieve desired EE poses
- Controls **ALL 9 DOF** (3 base + 6 arm joints)
- Runs at a fixed control rate (typically 100 Hz or 10 Hz)
- Applies **velocity limits** after IK solution
- Supports **fixed joint trajectories** (locking base or arm)

#### Inputs
```matlab
bundle       % GIK solver bundle (from createGikSolver)
trajectory   % Struct with:
             %   - Poses [4×4×N] SE(3) end-effector targets
             %   - AimTargets (optional)
             %   - DistanceBounds (optional)
options      % RateHz, VelocityLimits, FixedJointTrajectory, etc.
```

#### Outputs
```matlab
log = struct(
    'qTraj',         % [9×N] Joint angles (PRIMARY OUTPUT)
    'eePositions',   % [3×N] Actual EE positions (FK)
    'targetPositions', % [3×N] Desired EE positions
    'positionError', % [3×N] Tracking errors
    'solutionInfo',  % {1×N} Solver diagnostics
    'timestamps',    % [1×N] Time stamps
    'successMask',   % [1×N] Solver convergence flags
    'iterations',    % [1×N] Iterations per step
    'solveTime'      % [1×N] Solve time per step
);
```

#### Key Features
1. **IK Solver Loop**: Calls `bundle.solve(q, 'TargetPose', pose)` for each waypoint
2. **Velocity Clamping**: Applies max velocity limits to joint velocities after IK
3. **Fixed Joints**: Can lock specific joints (base or arm) to prescribed trajectories
4. **Rate Control**: Uses `rateControl(Hz)` to maintain loop timing
5. **Comprehensive Logging**: Captures solver diagnostics, iterations, solve times

#### Use Cases
- **Stage A**: Arm ramp-up (base locked)
- **Stage B (gikInLoop)**: Base navigation (arm locked)
- **Stage C Pass 1**: Reference IK (full-body, no constraints)
- **Stage C Pass 3**: Final IK with locked base (arm tracking with fixed base)
- **Holistic pureIk**: Full-body IK tracking

---

### 2. simulateChassisExecution - Chassis Simulation

**File:** `matlab/+gik9dof/+control/simulateChassisExecution.m` (328 lines)

#### What It Does
- **Simulates chassis controller execution** on a given base path
- Applies **chassis dynamics constraints** (velocity, acceleration, wheel limits)
- Generates **velocity commands** (Vx, Wz) via pure pursuit or heading controller
- **Integrates** chassis kinematics to produce executed base trajectory
- Controls **ONLY base (3 DOF)** - arm is not involved

#### Inputs
```matlab
pathStates   % [N×3] Base waypoints (x, y, yaw)
options      % SampleTime, ControllerMode (0/1/2), FollowerOptions
```

#### Outputs
```matlab
result = struct(
    'poses',        % [N×3] Executed base trajectory (x, y, θ)
    'commands',     % [N×2] Velocity commands (Vx, Wz)
    'wheelSpeeds',  % [N×2] Left/right wheel speeds
    'status',       % [N×1] Controller status per step
    'follower',     % Pure pursuit follower object (if mode 2)
    'controllerMode' % Which mode was used (0/1/2)
);
```

#### Controller Modes

| Mode | Name | Description | Output |
|------|------|-------------|--------|
| **0** | Legacy differentiation | Feedforward replay using numerical differentiation | Commands from path derivatives |
| **1** | Simple heading controller | P control + feedforward for heading | Commands via PID |
| **2** | **Pure pursuit follower** | Lookahead-based path following (RECOMMENDED) | Commands via pure pursuit |

#### Key Features
1. **Chassis Dynamics**: Applies vx_max, wz_max, accel_limit, wheel_speed_max
2. **Pure Pursuit**: Adaptive lookahead, Stanley/blended control options
3. **Kinematic Integration**: Integrates (Vx, Wz) → (x, y, θ) using differential drive model
4. **Realistic Simulation**: Produces achievable trajectory respecting constraints
5. **No IK**: Does NOT solve inverse kinematics - only simulates controller execution

#### Use Cases
- **Stage B (pureHyb)**: Simulate pure pursuit on Hybrid A* planned path
- **Stage C Pass 2 (ppForIk)**: Simulate chassis controller on reference base path from Pass 1
- **Holistic ppForIk Pass 2**: Simulate chassis controller on reference base path

---

## How They Work Together (Three-Pass Architecture)

In **ppForIk mode** (both Staged Stage C and Holistic), these functions work together:

```
Pass 1: runTrajectoryControl (Reference IK)
  Input:  EE poses [4×4×N]
  Solver: GIK (full-body, no chassis constraints)
  Output: log.qTraj [9×N] → extract base path [N×3]
  Result: IDEAL trajectory (may not be feasible)
  ↓
  Extract base: baseReference = log.qTraj(1:3,:)'

Pass 2: simulateChassisExecution (Chassis Simulation)
  Input:  baseReference [N×3]
  Controller: Pure pursuit + chassis dynamics
  Output: result.poses [N×3] executed base trajectory
  Result: REALISTIC trajectory (chassis-feasible)
  ↓
  executedBase = result.poses

Pass 3: runTrajectoryControl (Final IK with Fixed Base)
  Input:  EE poses [4×4×N]
  Solver: GIK with FixedJointTrajectory (base locked to executedBase)
  Output: log.qTraj [9×N] final trajectory
  Result: ACTUAL trajectory (arm tracks EE with realistic base)
```

**Key Point:** Pass 1 and Pass 3 use `runTrajectoryControl` (IK), Pass 2 uses `simulateChassisExecution` (chassis simulation). They are NOT interchangeable!

---

## Why They Cannot Be Merged

### Fundamental Differences

1. **Problem Domain**
   - `runTrajectoryControl`: **Inverse kinematics** - Given EE pose, find joint angles
   - `simulateChassisExecution`: **Controller simulation** - Given path, generate commands

2. **Solver Type**
   - `runTrajectoryControl`: Uses **generalizedInverseKinematics** (MATLAB toolbox)
   - `simulateChassisExecution`: Uses **pure pursuit controller** (custom implementation)

3. **Input Space**
   - `runTrajectoryControl`: **Task space** (SE(3) end-effector poses)
   - `simulateChassisExecution`: **Configuration space** (base positions x,y,θ)

4. **Output Type**
   - `runTrajectoryControl`: **Joint angles** q [9×1] (what to command robot)
   - `simulateChassisExecution`: **Velocities** (Vx, Wz) (how robot moves)

5. **DOF Controlled**
   - `runTrajectoryControl`: **All 9 DOF** (or subset via fixed joints)
   - `simulateChassisExecution`: **Base only (3 DOF)** - no arm involvement

6. **Constraint Application**
   - `runTrajectoryControl`: Constraints in **IK solver** (pose, distance, joint bounds)
   - `simulateChassisExecution`: Constraints in **controller** (vx_max, accel_limit, wheel limits)

7. **Time Integration**
   - `runTrajectoryControl`: **Step-by-step** IK solve per waypoint (no integration)
   - `simulateChassisExecution`: **Integrates** chassis kinematics (Vx, Wz → x, y, θ)

---

## Usage Patterns in Codebase

### runTrajectoryControl Called In:
1. **Stage A** (runStagedTrajectory.m:141): Arm ramp-up IK
2. **Stage B gikInLoop** (runStagedTrajectory.m:332): Base navigation IK
3. **Stage C Pass 1** (runStagedTrajectory.m:620): Reference IK
4. **Stage C Pass 3** (runStagedTrajectory.m:686): Final IK with fixed base
5. **Stage C pureIk** (runStagedTrajectory.m:839): Direct full-body IK
6. **Holistic Pass 1** (trackReferenceTrajectory.m:323): Reference IK
7. **Holistic Pass 3** (trackReferenceTrajectory.m:373): Final IK with fixed base
8. **Holistic pureIk** (trackReferenceTrajectory.m:407): Direct full-body IK

**Total: 8 call sites** - all for **IK control loops**

### simulateChassisExecution Called In:
1. **Stage B pureHyb** (runStagedTrajectory.m:282): Simulate pure pursuit on planned path
2. **Stage B alignment** (runStagedTrajectory.m:438): Align to docking pose
3. **Stage C Pass 2** (runStagedTrajectory.m:655): Simulate chassis on reference path
4. **Stage B rotation** (runStagedTrajectory.m:1561): Rotation alignment
5. **Holistic Pass 2** (trackReferenceTrajectory.m:355): Simulate chassis on reference path

**Total: 5 call sites** - all for **chassis controller simulation**

**Observation:** They are used in **different stages** of the pipeline and serve **different purposes**.

---

## Example Workflow (Stage C ppForIk)

```matlab
% Pass 1: Generate reference base path via IK
bundleRef = gik9dof.createGikSolver(robot, ...);
logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...
    'RateHz', 10, 'MaxIterations', 1500);
% Output: logRef.qTraj [9×N] (full-body trajectory)

% Extract base trajectory (x, y, θ)
baseReference = logRef.qTraj(1:3, :)';  % [N×3]

% Pass 2: Simulate chassis controller on reference path
simRes = gik9dof.control.simulateChassisExecution(baseReference, ...
    'ControllerMode', 2, ...  % Pure pursuit
    'FollowerOptions', struct('ChassisParams', chassisParams));
% Output: simRes.poses [N×3] (executed base trajectory)
%         simRes.commands [N×2] (Vx, Wz)

% Resample executed base to match trajectory length
executedBase = resamplePath(simRes.poses, numWaypoints);

% Pass 3: Final IK with base locked to executed trajectory
bundleFinal = gik9dof.createGikSolver(robot, ...);
logC = gik9dof.runTrajectoryControl(bundleFinal, trajStruct, ...
    'RateHz', 10, ...
    'FixedJointTrajectory', struct(...
        'Indices', [1 2 3], ...  % Base joints
        'Values', executedBase'));  % [3×N] locked base
% Output: logC.qTraj [9×N] (final trajectory with realistic base)
```

**This workflow REQUIRES both functions** - they cannot be replaced by a single function!

---

## Conceptual Analogy

Think of them as:

**runTrajectoryControl** = **Planner**
- "Given where I want the end-effector to be, what joint angles should I use?"
- Solves **inverse kinematics** (task space → joint space)
- Like an **architect** designing a path

**simulateChassisExecution** = **Simulator**
- "Given a desired base path, how will the robot actually execute it?"
- Simulates **controller execution** (path → commands → motion)
- Like a **physics engine** validating the path

You need BOTH: the architect AND the physics engine!

---

## Conclusion

### Are They Duplicated?
**NO** ❌ - They solve completely different problems:
- `runTrajectoryControl`: **Inverse kinematics** (EE poses → joint angles)
- `simulateChassisExecution`: **Controller simulation** (base path → velocity commands)

### Can We Use a Single Function?
**NO** ❌ - They are complementary and work together:
- Pass 1 (IK) generates reference base path
- Pass 2 (simulation) validates/corrects base path
- Pass 3 (IK) tracks EE with corrected base

### Should Anything Change?
**NO** ✅ - The current architecture is correct:
- Clear separation of concerns
- Well-defined responsibilities
- Necessary for three-pass ppForIk architecture
- Both functions are actively used throughout the codebase

---

## Recommendations

### ✅ Keep Both Functions
- They serve fundamentally different purposes
- Both are essential for the three-pass architecture
- No duplication or redundancy exists

### 📝 Clarify Documentation
- Update function headers to emphasize the difference
- Add cross-references explaining how they work together
- Document the three-pass architecture more clearly in function comments

### 🎯 Future Enhancement (Optional)
Consider adding a **higher-level orchestrator** that encapsulates the three-pass workflow:
```matlab
function log = runPpForIk(robot, trajStruct, chassisParams, options)
    % Pass 1: Reference IK
    logRef = gik9dof.runTrajectoryControl(...);
    
    % Pass 2: Chassis simulation
    simRes = gik9dof.control.simulateChassisExecution(...);
    
    % Pass 3: Final IK
    log = gik9dof.runTrajectoryControl(..., 'FixedJointTrajectory', ...);
end
```

But this would be **in addition to**, not **instead of**, the existing functions.

---

**Author:** GitHub Copilot  
**Reviewer:** [To be added]  
**Status:** Analysis Complete ✅
