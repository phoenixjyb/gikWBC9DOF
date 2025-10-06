# MATLAB Functions Analysis for Code Generation

## Executive Summary

This document analyzes the MATLAB codebase to identify which functions are essential for C++ code generation and which should remain MATLAB-only for simulation and design purposes.

---

## Core Functions Requiring Code Generation

### Category 1: Inverse Kinematics Solver (Priority: CRITICAL)

#### Files in `+codegen/` package

| File | Status | Purpose | Codegen Ready? |
|------|--------|---------|----------------|
| `solveGIKStep.m` | ✅ Keep | Single IK iteration with pose, joint, and distance constraints | ✅ Yes |
| `followTrajectory.m` | ✅ Keep | Iterates IK solver over waypoint sequence | ✅ Yes |
| `loadRobotForCodegen.m` | 🔄 Modify | Loads robot model for codegen (currently uses MAT file) | ⚠️ Needs refactor |
| `solveGIKStepWithLock.m` | ✅ Keep | IK step with locked base (for Stage A) | ✅ Likely ready |
| `stagedFollowTrajectory.m` | ⚠️ Evaluate | Staged control trajectory following | ❓ TBD |
| `stageBPlanPath.m` | ❌ Remove | Hybrid A* path planning - replace with ROS Nav2 | ❌ No |
| `generateRobotModelData.m` | ⚠️ Modify | MAT file generation - replace with procedural builder | ⚠️ Refactor needed |

**Key Actions**:
1. ✅ `solveGIKStep.m` - Ready to use, minimal changes needed
2. ✅ `followTrajectory.m` - Ready to use
3. 🔧 Create `buildRobotForCodegenARM64.m` - Procedural robot builder without MAT file dependency
4. ❌ Remove `stageBPlanPath.m` - Planning handled by ROS2 Nav2

---

### Category 2: Unified Chassis Controller (Priority: HIGH)

#### Files in `+control/` package

| File | Status | Purpose | Codegen Ready? |
|------|--------|---------|----------------|
| `unifiedChassisCtrl.m` | ✅ Keep | Main controller converting pose refs to (Vx, Wz) | ⚠️ Needs minor fixes |
| `clampYawByWheelLimit.m` | ✅ Keep | Velocity clamping based on differential drive limits | ✅ Yes |
| `defaultUnifiedParams.m` | ⚠️ Modify | Returns default parameters - needs to be data-only | ⚠️ Convert to config |
| `purePursuitFollower.m` | ⚠️ Simulation Only | Pure pursuit for Stage B - MATLAB simulation/validation | ❌ No (see note) |

**Key Actions**:
1. 🔧 `unifiedChassisCtrl.m` - Remove persistent state, use explicit state struct
2. ✅ `clampYawByWheelLimit.m` - Should be ready
3. 🔧 `defaultUnifiedParams.m` - Convert to YAML config file, load in ROS2
4. 📝 `purePursuitFollower.m` - **Keep for MATLAB simulation**, don't generate C++ (real robot uses ROS2 Nav2)

**Note (Updated Oct 6, 2025)**: `purePursuitFollower.m` merged from `origin/main` is used for desktop simulation and validation only. Real AGX Orin deployment uses existing ROS2 navigation stack.

---

### Category 3: Robot Model & Configuration (Priority: CRITICAL)

#### Core model files

| File | Status | Purpose | Codegen Ready? |
|------|--------|---------|----------------|
| `createRobotModel.m` | 🔄 Adapt | Loads URDF and creates rigidBodyTree | ⚠️ Needs procedural version |
| `configurationTools.m` | ✅ Keep | Configuration conversions and utilities | ⚠️ May need simplification |
| `collisionTools.m` | 🔄 Adapt | Attaches collision meshes to robot | ⚠️ Needs codegen-safe version |

**Key Actions**:
1. 🔧 Create `createRobotModelProcedural.m` - Build robot without file I/O at runtime
2. ✅ Keep core parts of `configurationTools.m` - Joint array conversions
3. 🔧 Adapt `collisionTools.m` - Pre-attach meshes during build, not at runtime

---

### Category 4: Trajectory Control (Priority: HIGH)

| File | Status | Purpose | Codegen Ready? |
|------|--------|---------|----------------|
| `runTrajectoryControl.m` | 🔄 Simplify | Main control loop for trajectory following | ⚠️ Complex, needs refactor |
| `trackReferenceTrajectory.m` | ❌ Remove | High-level orchestrator with plotting | ❌ MATLAB-only |
| `runStagedTrajectory.m` | ❌ Remove | Staged A/B/C control - too complex for first release | ❌ MATLAB-only |

**Key Actions**:
1. 🔧 Extract core loop from `runTrajectoryControl.m` → create `trajectoryControlStep.m`
2. ❌ Don't generate `trackReferenceTrajectory.m` - orchestration in ROS2
3. ❌ Don't generate `runStagedTrajectory.m` - focus on holistic control

---

## Functions to EXCLUDE from Code Generation

### Visualization & Animation (ALL excluded)

```
+viz/animate_whole_body.m
animateTrajectory.m
animateHolisticWithHelper.m
animateStagedWithHelper.m
animateStagedLegacy.m
compareAnimations.m
compareAnimationsLegacy.m
```

**Reason**: Graphics rendering not needed in embedded deployment

---

### Plotting & Analysis (ALL excluded)

```
generateLogPlots.m
plotTrajectoryLog.m
evaluateLog.m
generateExternalPlots.m
```

**Reason**: Offline analysis tools, run in MATLAB

---

### Environment Setup & Simulation (ALL excluded)

```
environmentConfig.m
addFloorDiscs.m
demoFloorDiscs.m
generateHolisticRamp.m
simulatePurePursuit()  ← NEW (Oct 6, 2025): in runStagedTrajectory.m
export_all_commands.m  ← NEW (Oct 6, 2025): root directory utility
```

**Reason**: 
- Static obstacles replaced by live perception
- Trajectory generation handled externally (task planner)
- Demo functions not needed in deployment
- **Pure pursuit simulation** - Desktop validation only (real robot uses ROS2 Nav2)
- **Command export** - Post-processing utility for log analysis

---

### Artifact Management (ALL excluded)

```
saveRunArtifacts.m
+internal/createResultsFolder.m
```

**Reason**: File I/O for results - MATLAB simulation only

---

### Utilities (Selective Keep)

| File | Status | Purpose |
|------|--------|---------|
| `+internal/resolvePath.m` | ❌ Exclude | Path resolution for MATLAB environment |
| `+internal/projectRoot.m` | ❌ Exclude | Project structure utility |

---

## New Files to Create for Code Generation

### 1. Real-Time Entry Points

**`+codegen/solveGIKStepRealtime.m`**
```matlab
function [qNext, convergence] = solveGIKStepRealtime(qCurrent, targetPose, ...
    distanceLower, distanceWeight)
%SOLVEGIKSTEPREALTIME Real-time safe IK solver with fixed-size arrays
%#codegen
% - No persistent variables (pass robot model explicitly)
% - Fixed-size arrays (bounded)
% - Deterministic execution time
```

**`+codegen/trajectoryControlStepRealtime.m`**
```matlab
function [qNext, baseCmd, diagnostics] = trajectoryControlStepRealtime(...
    qCurrent, targetPose, basePose, controlParams)
%TRAJECTORYCONTROLSTEPREALTIME Single control cycle combining IK and chassis control
%#codegen
% - Returns joint commands AND base velocity commands
% - Includes diagnostics for monitoring
```

**`+codegen/buildRobotForCodegenARM64.m`**
```matlab
function robot = buildRobotForCodegenARM64()
%BUILDROBOTFORCODEGENARM64 Procedural robot builder for ARM64 targets
%#codegen
% - Builds rigidBodyTree from scratch (no file I/O)
% - Hard-coded joint parameters from URDF
% - Pre-attached collision meshes
```

---

### 2. Wrapper Functions for ROS Integration

**`+codegen/unifiedChassisControlRealtime.m`**
```matlab
function [vx, wz, state] = unifiedChassisControlRealtime(...
    refPose, estPose, state, params)
%UNIFIEDCHASSISCONTROLREALTIME Stateless chassis controller
%#codegen
% - Explicit state management (no persistent)
% - Returns only (Vx, Wz) for differential drive
```

---

## Code Generation Configuration Matrix

| Function | Entry Point | Max Array Sizes | Dynamic Memory | Dependencies |
|----------|-------------|-----------------|----------------|--------------|
| `solveGIKStepRealtime` | ✅ Yes | q: 9×1, pose: 4×4 | OFF | Robot model |
| `followTrajectory` | ✅ Yes | waypoints: 50×(4×4) | OFF | solveGIKStep |
| `trajectoryControlStep` | ✅ Yes | q: 9×1, pose: 4×4 | OFF | GIK + chassis |
| `unifiedChassisControlRealtime` | ✅ Yes | Fixed scalars | OFF | None |
| `buildRobotForCodegenARM64` | 🔧 Init | Fixed structure | ON (init only) | None |

---

## Refactoring Checklist

### Phase 1: Core Solver (Week 1)

- [ ] Create `buildRobotForCodegenARM64.m` from URDF
- [ ] Test procedural builder in MATLAB
- [ ] Refactor `solveGIKStep.m` to accept robot as input (remove persistent)
- [ ] Create `solveGIKStepRealtime.m` entry point
- [ ] Test code generation on Windows
- [ ] Verify compilation on Ubuntu 22.04

### Phase 2: Chassis Controller (Week 1)

- [ ] Extract stateless core from `unifiedChassisCtrl.m`
- [ ] Create `unifiedChassisControlRealtime.m`
- [ ] Convert `defaultUnifiedParams.m` to YAML config
- [ ] Test code generation
- [ ] Verify differential drive math

### Phase 3: Trajectory Control (Week 2)

- [ ] Simplify `runTrajectoryControl.m` → `trajectoryControlStep.m`
- [ ] Create combined entry point with IK + chassis
- [ ] Add diagnostic outputs (convergence, errors)
- [ ] Test code generation
- [ ] Benchmark execution time

### Phase 4: Integration Testing (Week 2)

- [ ] Generate all C++ code for ARM64
- [ ] Create minimal ROS2 wrapper node
- [ ] Test with mock data in simulation
- [ ] Verify message interface compliance
- [ ] Profile real-time performance

---

## Dependencies Analysis

### External Dependencies (MATLAB Toolboxes)

1. **Robotics System Toolbox** - Required
   - `rigidBodyTree`
   - `generalizedInverseKinematics`
   - `constraintPoseTarget`, `constraintJointBounds`, `constraintDistanceBounds`

2. **Navigation Toolbox** - Optional (if keeping pure pursuit)
   - `controllerPurePursuit` - ❌ Not needed (using Nav2)

### Generated Code Dependencies

1. **OpenMP** - For parallel computation
2. **Eigen** (possibly) - Linear algebra (check MATLAB Coder output)
3. **Standard C++17** - No external libraries preferred

---

## MATLAB Coder Constraints to Address

### Current Issues

1. **Persistent Variables**
   - `solveGIKStep.m` uses persistent solver
   - **Fix**: Pass robot model explicitly or initialize once in ROS node

2. **Dynamic Arrays**
   - Trajectory waypoints are variable-size
   - **Fix**: Bound to max 50 waypoints using `coder.typeof`

3. **File I/O**
   - `createRobotModel.m` loads URDF at runtime
   - **Fix**: Procedural builder with hard-coded parameters

4. **Handle Classes**
   - `rigidBodyTree` is a handle class
   - **Fix**: Build at initialization, pass as value or global

5. **String Operations**
   - Some functions use string arrays
   - **Fix**: Use char arrays or enums for codegen

---

## Recommended Code Generation Strategy

### Approach A: Minimal Entry Points (RECOMMENDED)

Generate only 3 entry points:

1. **`initializeRobot()`** - One-time setup
2. **`solveIKStep(...)`** - IK computation
3. **`computeChassisCmd(...)`** - Chassis control

**Pros**: Minimal code, clear interfaces, easy to test  
**Cons**: ROS wrapper does more work

### Approach B: Comprehensive Library

Generate complete control library with many entry points

**Pros**: More functionality in generated code  
**Cons**: Complex, harder to debug, more codegen constraints

### Decision: **Use Approach A** for initial deployment

---

## Testing Strategy

### Unit Tests (MATLAB)

```matlab
% Test procedural robot builder
robot = gik9dof.codegen.buildRobotForCodegenARM64();
assert(robot.NumBodies == 15);

% Test IK step
q0 = zeros(9,1);
targetPose = trvec2tform([0.6 0.2 1.0]);
qNext = gik9dof.codegen.solveGIKStepRealtime(q0, targetPose, 0.2, 0.5);
assert(all(isfinite(qNext)));

% Test chassis controller
refPose = [1.0, 0.5, 0.3]; % x, y, theta
estPose = [0.0, 0.0, 0.0];
state = struct();
params = gik9dof.control.defaultUnifiedParams();
[vx, wz, state] = gik9dof.codegen.unifiedChassisControlRealtime(...
    refPose, estPose, state, params);
assert(isscalar(vx) && isscalar(wz));
```

### Integration Tests (ROS2)

1. **Mock data test**: Publish fake joint states, verify commands
2. **Trajectory test**: Follow simple straight-line trajectory
3. **Performance test**: Measure latency and jitter
4. **Stress test**: Rapid trajectory changes

---

## Code Generation Command Template

```matlab
% Configuration for ARM64 Linux (AGX Orin)
cfg = coder.config('lib', 'ecoder', false);
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.GenerateReport = true;
cfg.EnableOpenMP = true;
cfg.TargetLangStandard = 'C++17';
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg.HardwareImplementation.ProdWordSize = 64;
cfg.DynamicMemoryAllocation = 'Off';

% Define argument types
argsIK = {
    coder.typeof(0, [9,1], [false,false]),  % qCurrent
    coder.typeof(0, [4,4], [false,false]),  % targetPose
    coder.typeof(0),                         % distanceLower
    coder.typeof(0)                          % distanceWeight
};

% Generate
codegen('-config', cfg, ...
    'gik9dof.codegen.solveGIKStepRealtime', ...
    '-args', argsIK, ...
    '-o', 'solveGIKStepRealtime', ...
    '-d', 'codegen/linux_arm64');
```

---

## Summary

### Functions to Generate (13 total)

**Core Solver** (4):
- `buildRobotForCodegenARM64.m` (new)
- `solveGIKStepRealtime.m` (adapted)
- `followTrajectory.m` (existing)
- `solveGIKStepWithLock.m` (existing)

**Chassis Control** (2):
- `unifiedChassisControlRealtime.m` (adapted)
- `clampYawByWheelLimit.m` (existing)

**Configuration Tools** (3):
- `configurationTools.m` - partial (conversions only)
- `collisionTools.m` - adapted (pre-attach meshes)

**Trajectory Control** (1):
- `trajectoryControlStepRealtime.m` (new - simplified from runTrajectoryControl)

**Utilities** (3):
- Helper math functions as needed

### Functions to Exclude (25+)

All visualization, plotting, simulation, and environment setup functions

---

## Next Actions

1. ✅ Review this analysis
2. 🔧 Start with `buildRobotForCodegenARM64.m` creation
3. 🔧 Refactor `solveGIKStep.m` for explicit robot input
4. ✅ Test code generation incrementally
5. 🔧 Create minimal ROS2 wrapper
6. ✅ Validate on target hardware

---

**Document Version**: 1.0  
**Date**: 2025-10-06  
**Status**: Draft for Review
