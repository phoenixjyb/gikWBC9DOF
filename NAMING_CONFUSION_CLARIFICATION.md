# Naming Confusion: simulateChassisController vs unifiedChassisCtrl
**Date:** October 10, 2025  
**Issue:** "simulate" prefix is misleading - suggests it's NOT for deployment

---

## TL;DR - You're Right!

**YES**, the naming is confusing:
- ❌ `simulateChassisController` - Name suggests "simulation only"
- ✅ `unifiedChassisCtrl` - Name suggests "production controller"

**BUT** they serve **COMPLETELY DIFFERENT purposes**:
- `simulateChassisController`: **Path follower** (tracks geometric paths)
- `unifiedChassisCtrl`: **Command converter** (differentiates GIK trajectories)

They are **NOT interchangeable** - you need BOTH for different scenarios!

---

## 1. The Two Controllers Compared

### Controller A: `simulateChassisController`

**File:** `matlab/+gik9dof/+control/simulateChassisController.m`

**What it does:**
```matlab
% INPUT: Geometric path (waypoints)
pathStates = [x, y, yaw]  % Nx3 matrix

% OUTPUT: Robot follows the path using lookahead geometry
[vx, wz] = pathFollower(currentPose, pathStates)
```

**Purpose:** **Geometric path following**
- Takes waypoints as input
- Uses lookahead point for steering
- No reference trajectory needed
- Works with sparse waypoints

**Use cases:**
- ✅ Following Hybrid A* planner output
- ✅ Following waypoint paths from navigation
- ✅ Staged control "Stage B" (base independent)

**Modes:**
- Mode 0: Legacy differentiation (validation)
- Mode 1: Simple heading controller (basic)
- Mode 2: Pure pursuit follower (MAIN)

**Deployment status:** ❌ Mode 2 NOT deployed (class-based)

---

### Controller B: `unifiedChassisCtrl`

**File:** `matlab/+gik9dof/+control/unifiedChassisCtrl.m`

**What it does:**
```matlab
% INPUT: Timestamped reference trajectory from GIK
ref = struct('x', x, 'y', y, 'theta', theta, 't', t, 'arm_qdot', qdot)

% OUTPUT: Differentiates trajectory + adds feedback correction
[vx, wz] = differentiate(ref) + feedbackCorrection(pose)
```

**Purpose:** **Trajectory-to-command converter**
- Takes dense timestamped trajectory as input
- Differentiates to get velocities
- Adds proportional heading correction
- Integrates arm joint velocities

**Use cases:**
- ✅ Following GIK full-body trajectories (holistic control)
- ✅ Following GIK arm-only trajectories (Stage C)
- ✅ Following velocity commands directly (Stage B)

**Modes:**
- `"holistic"`: Full-body GIK → base + arm commands
- `"staged-C"`: Arm-only GIK → base + arm commands
- `"staged-B"`: Velocity input → base commands

**Deployment status:** ✅ DEPLOYED in C++ (`holisticVelocityController.cpp`)

---

## 2. Key Differences

| Aspect | simulateChassisController | unifiedChassisCtrl |
|--------|---------------------------|-------------------|
| **Input type** | Geometric waypoints | Timestamped trajectory |
| **Input density** | Sparse (meters apart) | Dense (dt = 0.1s) |
| **Algorithm** | Lookahead geometry | Differentiation + feedback |
| **Reference needed?** | No (geometric) | Yes (timed trajectory) |
| **Arm integration?** | No | Yes (holistic/staged-C) |
| **Deployed?** | ❌ No (class-based) | ✅ Yes (function-based) |
| **Name suggests** | Simulation only 😕 | Production ready 😊 |

---

## 3. Why the Confusing Names?

### Historical Context

**`simulateChassisController`:**
- Created as simulation/testing harness
- Wrapper to test different control modes
- Mode dispatch for research comparison
- Name reflects: "simulate different chassis controllers"
- **But**: Mode 2 is actually a full controller!

**`unifiedChassisCtrl`:**
- Created as production command converter
- Unifies heterogeneous reference formats
- Name reflects: "unified chassis control interface"
- **And**: Actually is production code!

### The Naming Problem

```
simulateChassisController  ← "simulate" sounds like MATLAB-only testing
    └─ Mode 2: purePursuitFollower  ← Actually a REAL controller!

unifiedChassisCtrl  ← "unified" sounds production-ready
    └─ Converts GIK trajectories  ← Actually is deployed!
```

**User's concern:** If we deploy Mode 2, calling it "simulate" is misleading!

---

## 4. Are They Similar? NO!

### Completely Different Algorithms

**`simulateChassisController` (Mode 2):**
```matlab
% Pure pursuit geometric tracking
lookaheadPoint = findLookaheadPoint(pose, path, lookaheadDistance);
curvature = computeCurvature(pose, lookaheadPoint, path);
wz = vx * curvature;  % Ackermann-like steering

% Features:
% - Lookahead point selection
% - Geometric curvature computation
% - Curvature-based speed control
% - Accel/jerk limiting
% - No differentiation needed!
```

**`unifiedChassisCtrl`:**
```matlab
% Trajectory differentiation + heading correction
if mode == "holistic"
    % Differentiate reference trajectory
    vxRef = (ref.x - prevRef.x) / dt;
    vyRef = (ref.y - prevRef.y) / dt;
    wzRef = (ref.theta - prevRef.theta) / dt;
    
    % Add proportional heading correction
    headingError = angdiff(ref.theta, estPose(3));
    wz = wzRef * yawKff + headingError * yawKp;
    
    % Transform to robot frame
    vx = vxRef * cos(estPose(3)) + vyRef * sin(estPose(3));
end

% Features:
% - Finite differencing
% - Heading feedback
% - Arm joint velocity passthrough
% - GIK integration
```

**They solve DIFFERENT problems!**

---

## 5. Can We Use One Instead of the Other?

### Short Answer: **NO!**

They complement each other for different scenarios.

### Scenario Matrix

| Scenario | Use This | Why |
|----------|----------|-----|
| **Hybrid A* path** | `simulateChassisController` | Sparse waypoints, no timestamps |
| **Full-body GIK** | `unifiedChassisCtrl` | Dense trajectory with arm motion |
| **Staged control (B)** | `simulateChassisController` | Base independent, geometric |
| **Staged control (C)** | `unifiedChassisCtrl` | Arm-only GIK with base following |
| **Navigation waypoints** | `simulateChassisController` | Geometric path following |
| **Pre-planned maneuvers** | `unifiedChassisCtrl` | Timed trajectory replay |

### When to Use Each

**Use `simulateChassisController` (or its refactored version) when:**
- ✅ You have waypoints but no timestamps
- ✅ Path is sparse (meters between points)
- ✅ You want adaptive speed control
- ✅ You want geometric tracking (lookahead)
- ✅ Base operates independently from arm

**Use `unifiedChassisCtrl` when:**
- ✅ You have timestamped trajectory from GIK
- ✅ Trajectory is dense (e.g., dt = 0.1s)
- ✅ You need arm joint velocities
- ✅ You want feedforward + feedback control
- ✅ Full-body coordination required

---

## 6. Naming Recommendations

### Option 1: Rename to Reflect Purpose

**Current:**
```
simulateChassisController  ← Misleading "simulate"
```

**Better names:**
```
chassisPathFollower        ← Emphasizes geometric path following
geometricPathController    ← Emphasizes geometric (vs trajectory) control
waypointFollowerController ← Emphasizes waypoint input
```

**Keep:**
```
unifiedChassisCtrl         ← Good name, reflects unification
```

### Option 2: Clarify with Comments

If renaming is too disruptive, add prominent comments:

```matlab
function result = simulateChassisController(pathStates, options)
%SIMULATECHASSISCONTROLLER Geometric path following controller
%   NOTE: Despite the "simulate" prefix, this is a PRODUCTION controller
%   for geometric path following. The name reflects its origin as a
%   simulation harness for testing multiple control modes.
%
%   For trajectory-based control (GIK), use unifiedChassisCtrl instead.
```

### Option 3: Create New Deployment Name

**In MATLAB:** Keep `simulateChassisController` for simulation
**For Codegen:** Create `chassisPathFollowerCodegen` (new name)

```matlab
% New deployment function (codegen-compatible)
function [vx, wz, state, status] = chassisPathFollowerCodegen(...)
%CHASSISPATHFOLLOWERCODEGEN Production geometric path follower
%   Implements pure pursuit / Stanley / blended control modes
%   for geometric waypoint following.
%
%   Replaces: purePursuitVelocityController (old simple version)
%   Different from: unifiedChassisCtrl (trajectory-based control)
```

---

## 7. Current Deployment Architecture

### What's Actually in C++ Right Now

```
ros2/gik9dof_solver/src/
├── holisticVelocityController.cpp  ← Wraps unifiedChassisCtrl
│   └─ Uses: unifiedChassisCtrl (modes: holistic/staged-C/staged-B)
│
├── purePursuitVelocityController.cpp  ← OLD simple path follower
│   └─ Uses: Basic pure pursuit (single mode, missing features)
│
└── (Other GIK/planner components)
```

### After Refactoring (Proposed)

```
ros2/gik9dof_solver/src/
├── holisticVelocityController.cpp  ← Wraps unifiedChassisCtrl (NO CHANGE)
│   └─ Uses: unifiedChassisCtrl (modes: holistic/staged-C/staged-B)
│
├── chassisPathFollowerCodegen.cpp  ← NEW advanced path follower
│   └─ Implements: Mode 2 from simulateChassisController
│   └─ Modes: blended/purePursuit/stanley
│   └─ Features: All 30+ parameters, limiting, etc.
│
├── purePursuitVelocityController.cpp  ← DEPRECATED (keep for reference)
│
└── (Other GIK/planner components)
```

### Usage Pattern

```cpp
// Scenario 1: Full-body GIK control
auto [cmd, state] = holisticVelocityController(
    refPose, estPose, state, params);  // Uses unifiedChassisCtrl

// Scenario 2: Geometric path following
auto [vx, wz, state, status] = chassisPathFollowerCodegen(
    pose, dt, state, params);  // New! From simulateChassisController Mode 2
```

---

## 8. Recommendation

### For the Refactoring Project

**Option A: Keep "simulate" name for now, document clearly**
```matlab
% In refactored file header:
%   NOTE: This is a PRODUCTION controller for geometric path following.
%   The "simulate" prefix is historical (from simulation harness origin).
%   Use this for waypoint-based navigation.
%   For GIK trajectory control, use unifiedChassisCtrl instead.
```

**Option B: Use new name for codegen version (RECOMMENDED)**
```matlab
% Create new file with deployment-ready name:
function [vx, wz, state, status] = chassisPathFollowerCodegen(...)
%CHASSISPATHFOLLOWERCODEGEN Geometric path follower (codegen)
%   Production controller for waypoint-based navigation.
%   Implements pure pursuit, Stanley, and blended control modes.
%
%   This is the codegen-compatible version of Mode 2 from
%   simulateChassisController. Despite the different name, it provides
%   the same advanced path following functionality.
%
%   Different from:
%   - unifiedChassisCtrl: For GIK trajectory-based control
%   - purePursuitVelocityController: Old simple version (deprecated)
```

**Option C: Rename everything (MOST DISRUPTIVE)**
- Rename `simulateChassisController` → `chassisPathFollower`
- Update all call sites
- Update documentation
- High risk of breaking things

### My Recommendation: **Option B**

**Why:**
- ✅ Clear deployment-ready name
- ✅ No risk to existing MATLAB simulation code
- ✅ Distinct from both old and unifiedChassisCtrl
- ✅ Easy to document and explain
- ✅ Follows pattern: `holisticVelocityController` wraps `unifiedChassisCtrl`

**Pattern:**
```
MATLAB (simulation):
├─ simulateChassisController  ← Keep for testing
└─ unifiedChassisCtrl          ← Keep for holistic control

Codegen (deployment):
├─ chassisPathFollowerCodegen  ← NEW name (clear purpose)
└─ holisticVelocityController  ← Existing (wraps unifiedChassisCtrl)
```

---

## 9. Summary

### The Confusion

1. **Name misleading**: "simulate" suggests MATLAB-only
2. **Different purposes**: Path follower vs trajectory converter
3. **Not interchangeable**: Serve different use cases
4. **Both needed**: Complement each other

### The Reality

- `simulateChassisController` (Mode 2) = **Real path follower**
- `unifiedChassisCtrl` = **Real trajectory converter**
- They solve **different problems**
- Current C++ only has partial implementation

### The Solution

**Create new codegen version with clear name:**
- File: `chassisPathFollowerCodegen.m`
- Purpose: Geometric waypoint following
- Modes: blended/purePursuit/stanley
- Distinct from: `unifiedChassisCtrl` (trajectory-based)

### Action Items

1. ✅ Document naming confusion (this file!)
2. 🔄 Update refactoring plan with new name
3. ⏳ Create `chassisPathFollowerCodegen.m` (Day 1)
4. ⏳ Update MODE_ARCHITECTURE_ANALYSIS.md
5. ⏳ Add clear comments distinguishing the two controllers

---

**END OF NAMING CLARIFICATION**
