# Control Mode Architecture & RS/Clothoid Integration Analysis
**Date:** October 10, 2025  
**Purpose:** Document mode selector architecture and path preprocessing integration

---

## Executive Summary

**Key Findings:**
1. ✅ **TWO SEPARATE MODE SYSTEMS** exist (they're different!)
2. ✅ **RS/Clothoid preprocessing** is OPTIONAL (not required for runtime)
3. ⚠️ **Mode numbering confusion** - need clarification

---

## 1. Two Different "Mode" Systems

### System A: `simulateChassisController` Modes (0/1/2)

**Location:** `matlab/+gik9dof/+control/simulateChassisController.m`

**Purpose:** **Simulation wrapper** for testing different control strategies

**Modes:**
```matlab
ControllerMode: 0, 1, or 2 (integer)

Mode 0: "Legacy 5-point differentiation" (feedforward replay)
- Differentiates reference trajectory (5-point stencil)
- Pure feedforward, no feedback
- Used for: Validation against original trajectories

Mode 1: "Simple heading controller" (P + feedforward yaw)
- Proportional heading error correction
- Feedforward yaw rate from path
- Used for: Basic path following without lookahead

Mode 2: "Pure Pursuit follower" (delegates to purePursuitFollower class)
- Calls simulatePurePursuitExecution()
- Creates purePursuitFollower class instance
- Uses lookahead point for geometric tracking
- **THIS IS THE MAIN ONE FOR DEPLOYMENT**
```

**Usage Context:**
```matlab
% MATLAB simulation/testing ONLY
result = simulateChassisController(pathStates, ...
    'ControllerMode', 2, ...  % Select mode 0, 1, or 2
    'FollowerOptions', chassisParams);
```

**Deployment Status:** ❌ NOT in C++ (simulation tool only)

---

### System B: `purePursuitFollower` Modes (string)

**Location:** `matlab/+gik9dof/+control/purePursuitFollower.m` (class)

**Purpose:** **Advanced path follower** with multiple control laws

**Modes:**
```matlab
ControllerMode: 'blended', 'purePursuit', 'stanley' (string)

Mode 'purePursuit': Geometric path tracking
- Classic pure pursuit algorithm
- Lookahead point selection
- Curvature-based steering
- Best at: High-speed smooth paths

Mode 'stanley': Cross-track error correction
- Stanley controller (Stanford's autonomous vehicle)
- Proportional to cross-track error
- Heading error correction
- Best at: Low-speed precise tracking

Mode 'blended': Speed-adaptive blend
- Blends Stanley (low speed) + Pure Pursuit (high speed)
- Transition threshold: ~0.3 m/s
- Best at: Full speed range, smooth transitions
```

**Usage Context:**
```matlab
% Create follower object (class-based, MATLAB only)
follower = purePursuitFollower(pathStates, ...
    'ControllerMode', 'blended', ...  % or 'purePursuit', 'stanley'
    'ChassisParams', chassisProfile);

% Step through path
[vx, wz, status] = follower.step(pose, dt);
```

**Deployment Status:** ❌ NOT in C++ yet (class-based, can't codegen)

---

### System C: `unifiedChassisCtrl` Modes (string) - DIFFERENT!

**Location:** `matlab/+gik9dof/+control/unifiedChassisCtrl.m`

**Purpose:** **Command converter** for whole-body control (ALREADY DEPLOYED)

**Modes:**
```matlab
Mode: 'holistic', 'staged-C', 'staged-B' (string)

Mode 'holistic': Full-body GIK control
- Converts GIK poses → chassis velocities
- Differentiates reference trajectory
- Includes arm joint velocities

Mode 'staged-C': Stage C (arm control only)
- Similar to holistic but arm-only focus
- Base follows pre-planned path

Mode 'staged-B': Stage B (base follower)
- Takes velocity commands directly
- No differentiation needed
- Base independent control
```

**Usage Context:**
```matlab
% Real-time control (DEPLOYED in C++)
[cmd, state] = unifiedChassisCtrl('holistic', ref, estPose, state, params);
```

**Deployment Status:** ✅ DEPLOYED in C++ (`holisticVelocityController` calls this)

---

## 2. Mode System Comparison

| Aspect | System A (simulateChassisController) | System B (purePursuitFollower) | System C (unifiedChassisCtrl) |
|--------|-------------------------------------|-------------------------------|-------------------------------|
| **Mode Type** | Integer (0/1/2) | String ('blended'/etc) | String ('holistic'/etc) |
| **Purpose** | Simulation testing | Advanced path following | Whole-body command converter |
| **Deployed?** | ❌ MATLAB only | ❌ MATLAB only (class) | ✅ C++ deployed |
| **Codegen?** | ⚠️ Partial (mode 2 blocked) | ❌ No (class + inputParser) | ✅ Yes (function-based) |
| **Usage** | Testing wrapper | Path tracking | Real-time control |

---

## 3. Current C++ Deployment (What You Actually Have)

### Deployed Components

**In `ros2/gik9dof_solver/`:**

1. **`purePursuitVelocityController.cpp`**
   - ✅ Generated from `matlab/purePursuitVelocityController.m`
   - ✅ Function-based, codegen-compatible
   - ❌ **SINGLE MODE** (pure pursuit only)
   - ❌ Missing: curvature control, accel limits, mode selection

2. **`holisticVelocityController.cpp`** (wraps `unifiedChassisCtrl`)
   - ✅ Generated from `matlab/holisticVelocityController.m`
   - ✅ Mode dispatch: 'holistic'/'staged-C'/'staged-B'
   - ✅ Full-body control integration

### What's Missing

❌ System A modes 0/1 (not needed - simulation only)  
❌ System B modes ('blended'/'stanley') - **THIS IS WHAT WE WANT!**  
❌ Advanced features (curvature, accel limits, etc.)

---

## 4. RS/Clothoid Preprocessing Integration

### Overview

**RS/Clothoid smoothing is an OPTIONAL offline preprocessing step**

It's NOT required at runtime, but provides significant benefits.

### Architecture

```
OFFLINE (Python/MATLAB):
┌────────────────────────────────────────────────────┐
│ Raw Path (from planner)                            │
│   ↓                                                 │
│ preparePathForFollower()                           │
│   ├─ Resample with adaptive spacing               │
│   ├─ Compute curvature                            │
│   ├─ (OPTIONAL) Call rsClothoidRefine()           │
│   │   └─ Fit clothoid splines                     │
│   └─ Return PathInfo struct                        │
│                                                     │
│ Save: PathInfo.mat                                 │
│   .States (Nx3)                                    │
│   .Curvature (Nx1)                                 │
│   .ArcLength (Nx1)                                 │
│   .DistanceRemaining (Nx1)                         │
└────────────────────────────────────────────────────┘

RUNTIME (C++):
┌────────────────────────────────────────────────────┐
│ Load: PathInfo.mat                                 │
│   ↓                                                 │
│ purePursuitFollowerCodegen()                       │
│   ├─ Use precomputed curvature                    │
│   ├─ Apply curvature-based speed control          │
│   ├─ Apply acceleration/jerk limiting             │
│   └─ Output: vx, wz commands                       │
└────────────────────────────────────────────────────┘
```

### Current Implementation

**Where RS/Clothoid is used:**

1. **`preparePathForFollower.m`** (lines 159-190)
   ```matlab
   % Resamples path with adaptive spacing
   % Computes curvature
   % Does NOT call rsClothoidRefine by default
   ```

2. **`runStagedTrajectory.m`** (lines 1198, 1690)
   ```matlab
   % Explicitly calls rsClothoidRefine for full-body trajectories
   [statesHC, hcInfo] = gik9dof.control.rsClothoidRefine(statesRefined, hcParams);
   ```

3. **`purePursuitFollower` class** (line 119)
   ```matlab
   % Calls preparePathForFollower in constructor
   obj.PathInfo = preparePathForFollower(pathStates, obj.Chassis);
   ```

### Key Insight

**`preparePathForFollower` does NOT call `rsClothoidRefine` by default!**

It:
- ✅ Resamples path with adaptive spacing
- ✅ Computes curvature from discrete differences
- ✅ Adds dense samples around high-curvature regions
- ❌ Does NOT fit clothoid splines

To use RS/Clothoid smoothing:
```matlab
% Option A: Call explicitly (full-body planning)
[smoothPath, info] = rsClothoidRefine(rawPath, params);
PathInfo = preparePathForFollower(smoothPath, chassisParams);

% Option B: Call from higher-level function (runStagedTrajectory does this)
```

---

## 5. Integration Strategy for Refactored Code

### Option 1: Preprocessing Pipeline (RECOMMENDED)

**Offline/Initialization:**
```matlab
% 1. Plan path (Hybrid A*, etc.)
rawPath = planHybridAstar(start, goal, grid);

% 2. (OPTIONAL) Apply RS/Clothoid smoothing
if useClothoidSmoothing
    smoothParams = struct('discretizationDistance', 0.05);
    [rawPath, ~] = rsClothoidRefine(rawPath, smoothParams);
end

% 3. Prepare for follower (compute curvature, resample)
chassisParams = loadChassisProfile();
PathInfo = preparePathForFollower(rawPath, chassisParams);

% 4. Save for deployment
save('mission_path.mat', 'PathInfo');
```

**Runtime (C++):**
```cpp
// Load preprocessed path
PathInfo pathInfo = loadPathInfo("mission_path.mat");

// Initialize controller
PurePursuitParams params;
params.PathInfo = pathInfo;
params.ControllerMode = "blended";  // or "purePursuit", "stanley"
params.Chassis = chassisProfile;

PurePursuitState state = initializeState(params);

// Control loop
while (!done) {
    Pose pose = getCurrentPose();
    
    // Call controller (C++ codegen version)
    auto [vx, wz, state, status] = purePursuitFollowerCodegen(
        pose, dt, state, params);
    
    publishCommand(vx, wz);
    
    if (status.isFinished) break;
}
```

### Option 2: Real-Time Preprocessing (ADVANCED)

**If paths are generated online:**
```cpp
// Receive new path from planner
PathStates rawPath = plannerOutput;

// Preprocess in C++ (if we codegen preparePathForFollower)
PathInfo pathInfo = preparePathForFollowerCodegen(rawPath, chassisParams);

// Update controller
updateControllerPath(pathInfo);
```

**Requires:**
- [ ] Codegen `preparePathForFollower.m`
- [ ] Codegen curvature computation helpers
- [ ] More complex integration

---

## 6. Proposed Architecture for Refactored Code

### New Mode System (Unified)

**We'll implement System B modes in the new codegen function:**

```matlab
function [vx, wz, state, status] = purePursuitFollowerCodegen(...
    pose, dt, state, params)
%PUREPURSUITFOLLOWERCODEGEN Advanced path follower with multiple modes
%
% Modes (params.ControllerMode):
%   'purePursuit' - Classic geometric tracking
%   'stanley'     - Cross-track error correction  
%   'blended'     - Speed-adaptive blend (DEFAULT)

switch lower(params.ControllerMode)
    case 'purepursuit'
        [vx, wz, status] = computePurePursuit(pose, state, params);
        
    case 'stanley'
        [vx, wz, status] = computeStanley(pose, state, params);
        
    case 'blended'
        [vx, wz, status] = computeBlended(pose, state, params);
end

% Apply limiting pipeline
vx = applyCurvatureLimiting(vx, status.curvature, params.Chassis);
[vx, accel] = applyAccelerationLimiting(vx, state.LastVelocity, ...
    state.LastAcceleration, dt, params.Chassis);
[vx, wz] = applyWheelSpeedLimiting(vx, wz, params.Chassis);

% Update state
state.LastVelocity = vx;
state.LastAcceleration = accel;
```

### ROS2 Parameter Mapping

**New parameters to add:**
```yaml
purepursuit:
  # Existing parameters (keep these)
  lookahead_base: 0.6
  lookahead_vel_gain: 0.3
  vx_max: 1.5
  wz_max: 2.0
  track: 0.674
  # ... (keep all 13 existing)
  
  # NEW: Controller mode selection
  controller_mode: "blended"  # or "purePursuit", "stanley"
  
  # NEW: Chassis model
  accel_limit: 1.2            # m/s²
  decel_limit: 1.8            # m/s²
  jerk_limit: 5.0             # m/s³
  
  # NEW: Curvature-based speed control
  curvature_slowdown:
    kappa_threshold: 0.9      # rad/m
    vx_reduction: 0.6         # fraction (60% speed in curves)
  
  # NEW: Heading PID (for Stanley/blended modes)
  heading_kp: 1.2
  heading_ki: 0.0
  heading_kd: 0.1
  feedforward_gain: 0.9
```

---

## 7. Relationship to Your Field Issues

### Why Current C++ Doesn't Work Well

**Your current `purePursuitVelocityController.cpp`:**
- ✅ Has basic pure pursuit
- ❌ NO mode selection (stuck in one mode)
- ❌ NO curvature-based speed control
- ❌ NO acceleration/jerk limiting
- ❌ NO preprocessing integration

**Result:**
- Goes too fast in curves → overshooting
- Jerky motion → wheel slip
- Poor at low speeds → instability

### How New System Fixes This

**New `purePursuitFollowerCodegen.cpp`:**
- ✅ 3 modes (blended handles full speed range)
- ✅ Curvature-based speed reduction
- ✅ Acceleration/jerk limiting
- ✅ Can use preprocessed paths with curvature

**Result:**
- Slows down in curves → smooth tracking
- Smooth acceleration → no wheel slip
- Blended mode → stable at all speeds

---

## 8. RS/Clothoid Decision Tree

**Do you need RS/Clothoid smoothing?**

```
Is your path from a geometric planner (Hybrid A*, RRT, etc.)?
├─ YES → Are there sharp corners or discontinuities?
│   ├─ YES → Use RS/Clothoid smoothing (offline)
│   └─ NO  → preparePathForFollower is enough
│
└─ NO  → Is path from full-body IK (runStagedTrajectory)?
    ├─ YES → Use RS/Clothoid smoothing (already integrated)
    └─ NO  → preparePathForFollower is enough
```

**Summary:**
- **Hybrid A* paths** → May benefit from smoothing
- **Full-body IK paths** → Already uses smoothing
- **Simple waypoints** → Just use preparePathForFollower

---

## 9. Implementation Impact

### What Changes

**Files to create:**
```
purePursuitFollowerCodegen.m       ← NEW: Main function with 3 modes
├─ Mode dispatch (blended/pure pursuit/stanley)
├─ Limiting functions (curvature/accel/jerk/wheel)
└─ Status reporting

Helper functions (optional, can be inline):
├─ computePurePursuit.m
├─ computeStanley.m
├─ computeBlended.m
└─ Various limiting functions
```

**Files to keep:**
```
purePursuitFollower.m              ← KEEP: Original class (MATLAB testing)
preparePathForFollower.m           ← KEEP: Path preprocessing
rsClothoidRefine.m                 ← KEEP: Clothoid smoothing (optional)
unifiedChassisCtrl.m               ← KEEP: Whole-body control (deployed)
```

**Files to deprecate:**
```
purePursuitVelocityController.m    ← REPLACE: Old simple version
simulateChassisController.m        ← KEEP: Simulation tool (not deployed)
```

### What Stays the Same

**Existing C++ deployment:**
- ✅ `holisticVelocityController` (no changes)
- ✅ `unifiedChassisCtrl` (no changes)
- ✅ GIK solver (no changes)
- ✅ Hybrid A* planner (no changes)

**Only replace:**
- `purePursuitVelocityController.cpp` → `purePursuitFollowerCodegen.cpp`

---

## 10. Questions & Answers

### Q1: Do I need to implement all 3 modes?

**A:** YES - They serve different purposes:
- `'purePursuit'` - High-speed smooth tracking
- `'stanley'` - Low-speed precision
- `'blended'` - Automatic transition (RECOMMENDED DEFAULT)

You can implement incrementally:
1. Start with `'purePursuit'` (port from old version)
2. Add limiting functions (curvature/accel/jerk)
3. Add `'stanley'` mode
4. Add `'blended'` mode

### Q2: Is RS/Clothoid smoothing required?

**A:** NO - It's optional:
- **Without it:** Use `preparePathForFollower` alone
  - ✅ Computes curvature from discrete differences
  - ✅ Adaptive resampling
  - ⚠️ May have noisy curvature on sharp paths
  
- **With it:** Use `rsClothoidRefine` first
  - ✅ Smoother curvature profiles
  - ✅ C² continuous (curvature continuous)
  - ⚠️ More computation (offline only)

**Recommendation:** Start without it, add later if needed.

### Q3: What about simulateChassisController modes 0/1?

**A:** Those are SIMULATION ONLY:
- Mode 0 (differentiation): For validation testing
- Mode 1 (simple heading): Basic research version
- Mode 2 (pure pursuit): Calls the class we're replacing

You don't need modes 0/1 in deployed C++ code.

### Q4: How does this relate to unifiedChassisCtrl?

**A:** DIFFERENT LAYERS:
```
Application Layer:
├─ Full-body control → holisticVelocityController
│   └─ Calls unifiedChassisCtrl (converts poses → velocities)
│
└─ Path following → purePursuitFollowerCodegen (NEW)
    └─ Takes path + pose → velocities directly
```

They serve different use cases:
- `unifiedChassisCtrl`: Whole-body IK-based control
- `purePursuitFollowerCodegen`: Geometric path following

Can use both! Depends on your control mode.

---

## 11. Summary

### Key Takeaways

1. **Two separate "mode" systems** exist:
   - System A: `simulateChassisController` (0/1/2) - Simulation only
   - System B: `purePursuitFollower` (blended/purePursuit/stanley) - What we want!
   - System C: `unifiedChassisCtrl` (holistic/staged) - Already deployed

2. **RS/Clothoid is OPTIONAL**:
   - Used in full-body planning (`runStagedTrajectory`)
   - NOT required for basic path following
   - Can add later if needed

3. **Current C++ is System "None"**:
   - Single pure pursuit mode
   - No advanced features
   - Explains field issues

4. **New refactored code implements System B**:
   - 3 modes (blended recommended)
   - All advanced features
   - Fixes field issues

### Next Steps

**Implement in this order:**
1. ✅ Create structs (params/state/status)
2. ✅ Port pure pursuit mode (from old version)
3. ✅ Add limiting functions (curvature/accel/jerk)
4. ✅ Add Stanley mode
5. ✅ Add blended mode
6. ✅ Test and codegen
7. ⚠️ (Optional) Integrate RS/Clothoid preprocessing

---

**END OF MODE ARCHITECTURE ANALYSIS**
