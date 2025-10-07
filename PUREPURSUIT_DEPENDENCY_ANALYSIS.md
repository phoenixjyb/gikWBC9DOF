# Pure Pursuit Code Generation - Dependency Analysis

**Date**: October 7, 2025  
**Status**: ✅ Analysis Complete

---

## 📋 Files Required for Code Generation

### ✅ Primary Function
**File**: `matlab/+gik9dof/+control/unifiedChassisCtrl.m`
- **Purpose**: Main controller that converts position references → velocity commands
- **Algorithm**: Simple heading controller (NOT the Pure Pursuit class)
- **Inputs**: Reference pose, current pose, controller params
- **Outputs**: Velocity commands (Vx, Wz)

### ✅ Helper Function 1
**File**: `matlab/+gik9dof/+control/clampYawByWheelLimit.m`
- **Purpose**: Enforce differential drive wheel speed limits
- **Algorithm**: Computes max yaw rate from wheel constraints
- **Dependencies**: None (pure math)

### ✅ MATLAB Built-in
**Function**: `wrapToPi(angle)`
- **Purpose**: Wrap angles to [-π, π]
- **Replacement**: Easy to inline (already done in our 5-point code!)
- **Code**: `while(angle > pi) angle -= 2*pi; while(angle < -pi) angle += 2*pi;`

---

## ❌ NOT Needed for Code Generation

### Pure Pursuit Class NOT Used
**File**: `matlab/+gik9dof/+control/purePursuitFollower.m`
- **Status**: ❌ NOT called by `unifiedChassisCtrl`
- **Note**: This is a separate, advanced path-following class
- **Usage**: Used in Stage B for complex trajectory tracking
- **Decision**: Skip for now (we're doing simpler holistic mode)

---

## 🎯 What `unifiedChassisCtrl` Actually Does

Looking at the code (lines 60-96), it's **NOT Pure Pursuit**, it's a **simple heading controller**:

```matlab
% For "holistic" or "staged-C" mode:
1. Differentiate reference positions → world velocities
   vxWorld = (x[n] - x[n-1]) / dt
   
2. Transform to robot frame
   vBody = R(yaw) * [vxWorld; vyWorld]
   
3. Compute desired heading from velocity direction
   phiDesired = atan2(vyWorld, vxWorld)
   
4. Heading error feedback
   Vx = |vBody| * cos(headingError)
   Wz = Kp * headingError + Kff * wRef
   
5. Apply wheel limits
   [Wz_clamped] = clampYawByWheelLimit(...)
```

**This is actually simpler than Pure Pursuit!** It's a **heading-based velocity controller** with feed-forward and feedback terms.

---

## 🔍 Key Insight: Two Controller Modes

### Mode 1: "holistic" or "staged-C" (What we want)
- **Input**: Target positions (x, y, θ, t)
- **Algorithm**: 
  - Differentiate positions → velocities
  - Heading control with proportional + feed-forward
- **Use Case**: Follow IK solver output (our current scenario)

### Mode 2: "staged-B" (Advanced, not needed yet)
- **Input**: Velocity commands (v, w, t) from path planner
- **Algorithm**: Pass-through with wheel limit enforcement
- **Use Case**: Follow pre-planned Hybrid A* path
- **Requires**: Separate path planning step

**For Phase 2A, we only need Mode 1!**

---

## 📦 Codegen Dependencies Summary

| File | Type | Status | Notes |
|------|------|--------|-------|
| `unifiedChassisCtrl.m` | Main function | ✅ Ready | No problematic dependencies |
| `clampYawByWheelLimit.m` | Helper function | ✅ Ready | Pure math, codegen-friendly |
| `wrapToPi()` | MATLAB built-in | ✅ Inline | Easy replacement |
| `purePursuitFollower.m` | Class | ❌ Not needed | Unused by unifiedChassisCtrl |

**Verdict**: ✅ **All dependencies are codegen-friendly!** No toolbox issues, no classes, pure numerical code.

---

## 🚀 Code Generation Strategy

### Step 1: Create Wrapper Function (Simplify Interface)

Since `unifiedChassisCtrl` uses complex struct I/O and modes, create a simpler wrapper:

```matlab
function [Vx, Wz, stateOut] = holisticVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    stateIn, ...
    params)
%HOLISTICVELOCITYCONTROLLER Simplified interface for codegen
%   Wraps unifiedChassisCtrl in holistic mode with scalar I/O

    % Build reference struct
    ref.x = refX;
    ref.y = refY;
    ref.theta = refTheta;
    ref.t = refTime;
    
    % Call main controller
    estPose = [estX, estY, estYaw];
    [cmd, stateOut] = gik9dof.control.unifiedChassisCtrl(...
        "holistic", ref, estPose, stateIn, params);
    
    % Extract outputs
    Vx = cmd.base.Vx;
    Wz = cmd.base.Wz;
end
```

**Benefits**:
- Scalar inputs/outputs (easier for C++)
- No string mode selection (fixed to "holistic")
- Cleaner ROS2 integration

---

### Step 2: Define Codegen Types

```matlab
% Reference: target from IK solver
refX = 0.0;
refY = 0.0;
refTheta = 0.0;
refTime = 0.0;

% Estimate: current robot pose
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% State: controller memory (previous reference)
stateIn = struct('prev', struct('x', 0.0, 'y', 0.0, 'theta', 0.0, 't', 0.0));

% Parameters: robot and controller config
params = struct(...
    'track', 0.5, ...          % wheel spacing (m)
    'Vwheel_max', 1.0, ...     % max wheel speed (m/s)
    'Vx_max', 0.8, ...         % max forward speed (m/s)
    'W_max', 1.0, ...          % max yaw rate (rad/s)
    'yawKp', 1.5, ...          % heading P gain
    'yawKff', 0.8 ...          % heading FF gain
);
```

---

### Step 3: Codegen Command

```matlab
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenerateReport = true;
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';

codegen -config cfg holisticVelocityController ...
    -args {refX, refY, refTheta, refTime, ...
           estX, estY, estYaw, ...
           stateIn, params} ...
    -d codegen/velocity_controller
```

---

## ✅ Readiness Checklist

- [x] Identified all required files (2 functions)
- [x] Verified no toolbox dependencies
- [x] Confirmed no class-based code
- [x] Checked for MATLAB built-ins (only `wrapToPi`, easy to replace)
- [x] Designed simplified wrapper for C++ integration
- [x] Defined codegen type specifications
- [ ] Create wrapper function
- [ ] Create codegen script
- [ ] Generate and test C++ code

**Status**: Ready to proceed with implementation! 🎉

---

## 🎓 Technical Notes

### Why Not "Pure Pursuit"?

The name "unified chassis controller" is somewhat misleading. The actual algorithm is:

1. **Differentiation** (positions → velocities)
2. **Coordinate transformation** (world → robot frame)
3. **Heading control** (proportional + feedforward)
4. **Wheel limit enforcement** (differential drive kinematics)

**NOT** the classic Pure Pursuit algorithm (lookahead point on path), but rather a **heading-based velocity controller**.

**However**, this is actually BETTER for our use case:
- ✅ Simpler (no path, just target position)
- ✅ More responsive (lower latency)
- ✅ Direct integration with IK solver output
- ✅ Codegen-friendly (no path data structures)

The true Pure Pursuit (`purePursuitFollower` class) would be used in Stage B when following a Hybrid A* planned path. That's Phase 2B!

---

## 🔄 Comparison: Current vs. New

| Aspect | Current (5-pt diff) | New (Velocity Controller) |
|--------|---------------------|---------------------------|
| **Algorithm** | Pure differentiation | Differentiation + heading control |
| **Accuracy** | O(h⁴) smoothing | O(h) diff + feedback |
| **Control Law** | None (open-loop) | Heading P + FF (closed-loop) |
| **Frame Handling** | Manual transform | Integrated |
| **Wheel Limits** | None | Enforced via kinematics |
| **Benefits** | Smooth numerical | True trajectory tracking |

**Upgrade Value**: Moving from open-loop differentiation to closed-loop controller with proper differential drive kinematics!

---

**Next Step**: Create the wrapper function and codegen script.
