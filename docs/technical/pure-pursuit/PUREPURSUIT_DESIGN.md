# Pure Pursuit Controller - Design Specification

**Date:** October 7, 2025  
**Goal:** Implement true Pure Pursuit path following that works with ANY position reference source  
**Status:** 🚀 Design Phase

---

## Requirements

### 1. Flexible Input Sources
The controller must accept position references from:
- ✅ **GIK solver output** - Single position target per cycle
- ✅ **Hybrid A* planner** - Multi-waypoint path
- ✅ **Manual waypoints** - User-defined trajectory
- ✅ **Any other planner** - Generic `(x, y, θ, t)` format

### 2. Output
- **Velocity commands:** `(vx, vy=0, wz)` for differential drive
- **Update rate:** 10-100 Hz (configurable)
- **Constraints:** Respect wheel speed limits, max velocities

### 3. Algorithm Features
- ✅ **Lookahead distance** - Tunable parameter
- ✅ **Path following** - Smooth trajectory tracking
- ✅ **Curvature calculation** - Geometric steering
- ✅ **Goal handling** - Behavior when reaching final waypoint
- ✅ **Path buffer** - Manage sequence of waypoints

---

## Pure Pursuit Algorithm

### Classic Pure Pursuit (for reference)

```
Given:
- Current pose: (x_robot, y_robot, θ_robot)
- Path: List of waypoints [(x₁,y₁), (x₂,y₂), ..., (xₙ,yₙ)]
- Lookahead distance: L

Steps:
1. Find lookahead point on path at distance L ahead
2. Transform lookahead point to robot frame
3. Calculate curvature to reach lookahead point
4. Convert curvature to angular velocity
5. Set forward velocity (constant or adaptive)
6. Output (vx, wz)
```

### Our Adaptation for GIK Integration

**Challenge:** GIK provides single position reference, not a full path

**Solution:** Build internal path buffer and apply Pure Pursuit

```matlab
function [vx, wz, state] = purePursuitVelocityController(
    refX, refY, refTheta, refTime,  % Current reference from GIK
    estX, estY, estYaw,              % Current robot pose
    params,                          % Lookahead, velocities, etc.
    state)                           % Path buffer + controller state

% 1. Update path buffer with new reference
%    - Add new waypoint if significantly different
%    - Remove passed waypoints
%    - Maintain sliding window of recent references

% 2. Find lookahead point
%    - Search along buffered path
%    - Distance L ahead of current position

% 3. Pure Pursuit calculation
%    - Transform lookahead to robot frame
%    - Calculate curvature: κ = 2 * y_lookahead / L²
%    - Angular velocity: wz = vx * κ

% 4. Goal region handling
%    - If close to final waypoint, reduce speed
%    - Stop when within tolerance

% 5. Apply constraints
%    - Enforce wheel speed limits
%    - Respect max velocity/angular rate

end
```

---

## Controller Modes

### Mode 1: Single-Point Tracking (GIK compatibility)
**Input:** One position reference per cycle  
**Behavior:** 
- Build path from recent references
- Apply Pure Pursuit to accumulated path
- Gracefully handle sparse waypoints

**Use case:** Direct integration with current GIK solver

### Mode 2: Multi-Waypoint Path Following
**Input:** Array of waypoints  
**Behavior:**
- Load full path into buffer
- Classic Pure Pursuit along path
- Clear when goal reached

**Use case:** Integration with Hybrid A* or other planners

### Mode 3: Continuous Path Update
**Input:** Streaming waypoints  
**Behavior:**
- Continuously update path buffer
- Remove old waypoints
- Follow latest trajectory

**Use case:** Dynamic replanning, moving goals

---

## Interface Design

### MATLAB Wrapper Function

```matlab
function [vx, wz, stateOut] = purePursuitVelocityController(...
    refX, refY, refTheta, refTime, ...  % Reference (single point or latest)
    estX, estY, estYaw, ...             % Current pose
    params, ...                         % Configuration
    stateIn)                            % Controller state

% params struct:
%   .lookahead         - Lookahead distance (m)
%   .minLookahead      - Minimum lookahead (m)
%   .maxLookahead      - Maximum lookahead (m)
%   .vxNominal         - Nominal forward speed (m/s)
%   .vxMax             - Max forward speed (m/s)
%   .wzMax             - Max angular rate (rad/s)
%   .track             - Wheel track width (m)
%   .vwheelMax         - Max wheel speed (m/s)
%   .goalTolerance     - Goal reached threshold (m)
%   .pathBufferSize    - Max waypoints to keep
%   .waypointSpacing   - Min distance between waypoints (m)
%   .adaptiveLookahead - Enable speed-dependent lookahead (bool)
%   .goalRegionRadius  - Slow down when within this distance (m)

% stateIn struct:
%   .path              - Struct array of waypoints
%     .x, .y, .theta, .t
%   .goalReached       - Flag
%   .currentSegment    - Index of current path segment
%   .prevPose          - Previous robot pose

% Outputs:
%   vx    - Forward velocity (m/s)
%   wz    - Angular velocity (rad/s)
%   stateOut - Updated state
end
```

### Alternative: Batch Waypoint Input

```matlab
function [vx, wz, stateOut] = purePursuitVelocityController(...
    waypointArray, ...   % N×3 array: [x, y, theta] or N×4: [x, y, theta, t]
    estX, estY, estYaw, ...
    params, ...
    stateIn)
    
% For full path input (e.g., from Hybrid A*)
end
```

---

## Implementation Strategy

### Phase 1: Core Pure Pursuit
1. MATLAB implementation of classic Pure Pursuit
2. Test with simple paths (straight, circle, S-curve)
3. Validate against Navigation Toolbox (if available)

### Phase 2: Path Buffer Management
1. Add waypoint buffer logic
2. Handle sparse waypoint updates (GIK mode)
3. Implement waypoint culling (remove passed points)

### Phase 3: Adaptive Features
1. Speed-dependent lookahead: `L = L_base + k * vx`
2. Goal region behavior (slow down near target)
3. Path prediction for smooth transitions

### Phase 4: Code Generation
1. MATLAB Coder compatibility fixes
2. Generate ARM64 + x86_64 code
3. Build standalone C++ library

### Phase 5: ROS2 Integration
1. Add Pure Pursuit to runtime switch
2. Three modes: Legacy 5-pt | Simple Heading | Pure Pursuit
3. Config file with Pure Pursuit parameters

---

## Parameter Tuning Guide

### Lookahead Distance
- **Small L (0.3-0.5m):** Tight tracking, may oscillate
- **Medium L (0.8-1.2m):** Balanced, good for most cases
- **Large L (1.5-2.5m):** Smooth, wide turns, slower response

**Adaptive:** `L = 0.5 + 0.3 * vx` (grows with speed)

### Forward Velocity
- **Constant:** Simple, predictable
- **Adaptive:** Slow in turns, fast on straights
- **Goal-aware:** Decelerate near target

### Goal Tolerance
- **Tight (0.05m):** Precise positioning
- **Loose (0.2m):** Faster, good enough for most tasks

---

## Comparison: Simple Heading vs Pure Pursuit

| Feature | Simple Heading | Pure Pursuit |
|---------|---------------|--------------|
| Input | Single point | Path or buffered points |
| Method | Heading error feedback | Geometric curvature |
| Lookahead | No | Yes (configurable) |
| Cornering | Poor (reactive) | Good (anticipatory) |
| Smoothness | Depends on reference | Inherently smooth |
| Complexity | Low | Medium |
| Tuning | 2 params (Kp, Kff) | 5+ params (L, vx, tolerances) |

---

## Next Steps

1. ✅ Design complete (this document)
2. ⏭️ Implement MATLAB Pure Pursuit core algorithm
3. ⏭️ Add path buffer management
4. ⏭️ Test with synthetic paths
5. ⏭️ MATLAB Coder compatibility
6. ⏭️ Generate C++ code
7. ⏭️ ROS2 integration with 3-way switch
8. ⏭️ Deploy and test on Orin

---

## Configuration Confirmed ✓

1. **Path buffer size:** 30 waypoints max
2. **Interpolation strategy:** 
   - Chassis max speed: 1.5 m/s
   - Update rate: 100 Hz (dt = 0.01s)
   - Interpolation spacing: configurable (default 0.05m)
3. **Goal behavior:** Continue accepting new references (continuous operation)
4. **Lookahead:** Adaptive - function of velocity AND time
   - `L = L_base + k_v * vx + k_t * dt_since_ref`
   - Allows temporal adaptation as well as speed-dependent

---

## Implementation Status

- ✅ **purePursuitVelocityController.m** - Core algorithm implemented
- ✅ **test_purePursuitController.m** - 8 comprehensive test cases
- ✅ **generate_code_purePursuit.m** - Code generation script
- ⏭️ Run tests in MATLAB
- ⏭️ Generate ARM64 and x86_64 C++ code
- ⏭️ ROS2 integration
- ⏭️ Deployment to Orin
