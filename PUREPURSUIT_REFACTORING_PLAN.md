# Pure Pursuit Follower Refactoring Plan
**Date:** October 10, 2025  
**Goal:** Make advanced purePursuitFollower codegen-compatible  
**Strategy:** Refactor class → function while preserving ALL features

---

## Executive Summary

**Decision:** Refactor new `purePursuitFollower` class into codegen-compatible function  
**Rationale:** Gets all advanced features (curvature, accel limits, modes) in deployable C++  
**Effort:** 3-4 days (refactoring + testing)  
**Value:** Long-term solution with full feature set

---

## Current Architecture Analysis

### What Makes It Non-Codegen-Compatible

1. **Class-based OOP** - MATLAB Coder supports classes poorly
2. **`inputParser`** (line 57) - Not codegen-compatible
3. **`arguments` block** - Must be converted to manual validation
4. **Handle semantics** - State management needs restructuring
5. **Properties with validation** - Must become struct fields

### Features to Preserve

#### ✅ Core Features (MUST HAVE)
- [x] 3 controller modes (blended/purePursuit/stanley)
- [x] Advanced chassis model (accel/jerk/curvature limits)
- [x] Path preprocessing integration
- [x] Curvature-based speed control
- [x] Acceleration/jerk limiting
- [x] Adaptive lookahead (base + vel + accel)
- [x] Bidirectional support (forward/reverse)
- [x] Comprehensive status reporting

#### ✅ Enhanced Parameters (MUST HAVE)
- [x] 30+ tuning parameters
- [x] Chassis profile struct
- [x] Controller mode selection
- [x] PID heading control
- [x] Curvature slowdown configuration

#### 🟡 Nice-to-Have Features
- [ ] `setPath()` dynamic method (→ reinitialize state)
- [ ] `reset()` method (→ reset function)
- [ ] PathInfo caching (→ preprocessing workflow)

---

## Refactoring Strategy

### Phase 1: Create Codegen-Compatible Structures ✅

**1.1 Parameters Struct**
```matlab
% Input: params struct with ALL fields
params = struct(
    % Timing
    'SampleTime', 0.1,
    
    % Controller selection
    'ControllerMode', 'blended',  % or 'purePursuit', 'stanley'
    
    % Lookahead tuning
    'LookaheadBase', 0.6,
    'LookaheadVelGain', 0.30,
    'LookaheadAccelGain', 0.05,  % renamed from LookaheadTimeGain
    
    % Goal detection
    'GoalTolerance', 0.10,
    
    % Heading PID
    'HeadingKp', 1.2,
    'HeadingKi', 0.0,
    'HeadingKd', 0.1,
    'FeedforwardGain', 0.9,
    
    % Chassis model (advanced)
    'Chassis', struct(
        'track', 0.573,              % m
        'wheel_speed_max', 3.3,      % m/s
        'vx_max', 1.5,               % m/s
        'vx_min', -0.4,              % m/s
        'wz_max', 2.5,               % rad/s
        'accel_limit', 1.2,          % m/s²
        'decel_limit', 1.8,          % m/s²
        'jerk_limit', 5.0,           % m/s³
        'wheel_base', 0.36,          % m
        'curvature_slowdown', struct(
            'kappa_threshold', 0.9,  % rad/m
            'vx_reduction', 0.6),    % fraction
        'reverse_enabled', false
    ),
    
    % Path preprocessing (PathInfo struct)
    'PathInfo', struct(
        'States', [],                % Nx3 [x y theta]
        'Curvature', [],             % Nx1 curvature [rad/m]
        'ArcLength', [],             % Nx1 cumulative arc length [m]
        'DistanceRemaining', [],     % Nx1 distance to goal [m]
        'SegmentIndex', [],          % Nx1 segment index
        'Diagnostics', struct('warnings', strings(1,0))
    )
);
```

**1.2 State Struct**
```matlab
% Persistent state between calls
state = struct(
    % Path tracking
    'CurrentIndex', 1,               % Current waypoint index
    'PathNumPoints', 0,              % Number of points in path
    
    % Velocity history (for accel/jerk limiting)
    'LastVelocity', 0.0,             % Previous vx command
    'LastAcceleration', 0.0,         % Previous acceleration
    
    % Heading controller state (PID)
    'LastHeadingError', 0.0,         % For D-term
    'HeadingIntegral', 0.0,          % For I-term
    
    % Path data (copied from PathInfo for codegen)
    'PathStates', zeros(0, 3),       % Nx3 [x y theta]
    'PathCurvature', zeros(0, 1),    % Nx1 curvature
    'PathArcLength', zeros(0, 1),    % Nx1 arc length
    'PathDistRemaining', zeros(0, 1) % Nx1 distance remaining
);
```

**1.3 Status Struct (Output)**
```matlab
status = struct(
    'isFinished', false,
    'nearestIndex', 1,
    'targetIndex', 1,
    'distanceToGoal', 0.0,
    'lookaheadDistance', 0.6,
    'curvature', 0.0,
    'crossTrackError', 0.0,
    'headingError', 0.0,
    'vxCommand', 0.0,
    'wzCommand', 0.0,
    'acceleration', 0.0,
    'wheelSpeeds', [0.0, 0.0],
    'controllerMode', 'blended',
    'warnings', strings(1, 0)
);
```

### Phase 2: Refactor Core Algorithm ✅

**2.1 Main Function Signature**
```matlab
function [vx, wz, state, status] = purePursuitFollowerCodegen(...
    pose, dt, state, params)
%PUREPURSUITFOLLOWERCODEGEN Codegen-compatible advanced path follower
%
% Inputs:
%   pose   - [x y theta] current robot pose
%   dt     - time step (s)
%   state  - persistent state struct
%   params - parameters struct with Chassis and PathInfo
%
% Outputs:
%   vx     - forward velocity command (m/s)
%   wz     - angular velocity command (rad/s)
%   state  - updated state struct
%   status - status struct with diagnostics
%
% Features:
%   - 3 controller modes: blended/purePursuit/stanley
%   - Curvature-based speed control
%   - Acceleration/jerk limiting
%   - Advanced chassis model
%   - Comprehensive status reporting

%#codegen
```

**2.2 Initialization Logic**
```matlab
% Initialize state on first call or empty state
if isempty(fieldnames(state)) || state.PathNumPoints == 0
    state = initializeState(params.PathInfo);
end

% Handle empty path
if state.PathNumPoints < 2
    vx = 0; wz = 0;
    status = createEmptyStatus(params);
    return
end
```

**2.3 Controller Mode Dispatch**
```matlab
% Select controller based on mode and velocity
mode = lower(params.ControllerMode);

switch mode
    case 'purepursuit'
        [vx_desired, wz_desired, status] = computePurePursuit(...
            pose, state, params);
        
    case 'stanley'
        [vx_desired, wz_desired, status] = computeStanley(...
            pose, state, params);
        
    case 'blended'
        [vx_desired, wz_desired, status] = computeBlended(...
            pose, state, params);
        
    otherwise
        error('Unknown controller mode: %s', mode);
end
```

**2.4 Velocity Smoothing Pipeline**
```matlab
% Apply curvature-based speed reduction
vx_curve_limited = applyCurvatureLimiting(vx_desired, ...
    status.curvature, params.Chassis);

% Apply acceleration limiting
[vx_accel_limited, accel] = applyAccelerationLimiting(...
    vx_curve_limited, state.LastVelocity, ...
    state.LastAcceleration, dt, params.Chassis);

% Apply jerk limiting
[vx_smooth, accel_smooth] = applyJerkLimiting(...
    vx_accel_limited, accel, state.LastAcceleration, dt, params.Chassis);

% Apply wheel speed limiting
[vx, wz] = applyWheelSpeedLimiting(vx_smooth, wz_desired, params.Chassis);
```

### Phase 3: Implement Controller Algorithms ✅

**3.1 Pure Pursuit Controller**
```matlab
function [vx, wz, status] = computePurePursuit(pose, state, params)
    % Find lookahead point
    [target_idx, target_pose, lookahead_dist] = findLookaheadPoint(...
        pose, state, params);
    
    % Compute curvature to target
    kappa = computeCurvatureToTarget(pose, target_pose);
    
    % Compute commands
    vx = params.Chassis.vx_max;  % Will be limited later
    wz = kappa * vx;
    
    % Update status
    status = updateStatus(state, target_idx, kappa, 0, 0, params);
end
```

**3.2 Stanley Controller**
```matlab
function [vx, wz, status] = computeStanley(pose, state, params)
    % Find nearest point on path
    [nearest_idx, cross_track_error, heading_error] = ...
        findNearestPathPoint(pose, state);
    
    % Stanley law: wz = k_cte * cte / vx + heading_correction
    k = params.HeadingKp;
    vx = params.Chassis.vx_max;
    
    % Avoid division by zero at low speeds
    vx_safe = max(abs(vx), 0.1);
    wz = k * cross_track_error / vx_safe + heading_error;
    
    % Update status
    kappa = state.PathCurvature(nearest_idx);
    status = updateStatus(state, nearest_idx, kappa, ...
        cross_track_error, heading_error, params);
end
```

**3.3 Blended Controller**
```matlab
function [vx, wz, status] = computeBlended(pose, state, params)
    % Compute both controllers
    [vx_pp, wz_pp, status_pp] = computePurePursuit(pose, state, params);
    [vx_st, wz_st, status_st] = computeStanley(pose, state, params);
    
    % Blend based on speed (Stanley at low speed, PP at high speed)
    vx_current = abs(state.LastVelocity);
    vx_threshold = 0.3;  % m/s
    
    if vx_current < vx_threshold
        % Low speed: Use Stanley (better tracking)
        alpha = vx_current / vx_threshold;  % 0 to 1
    else
        % High speed: Use Pure Pursuit (smoother)
        alpha = 1.0;
    end
    
    % Blend outputs
    vx = alpha * vx_pp + (1 - alpha) * vx_st;
    wz = alpha * wz_pp + (1 - alpha) * wz_st;
    
    % Merge status
    status = status_pp;
    status.crossTrackError = status_st.crossTrackError;
end
```

### Phase 4: Implement Limiting Functions ✅

**4.1 Curvature-Based Speed Limiting**
```matlab
function vx_limited = applyCurvatureLimiting(vx_desired, curvature, chassis)
    %APPLYCURVATURELIMITING Reduce speed in high-curvature sections
    %#codegen
    
    kappa = abs(curvature);
    vx_limited = vx_desired;
    
    % Apply configured slowdown threshold
    if kappa > chassis.curvature_slowdown.kappa_threshold
        vx_limited = vx_desired * chassis.curvature_slowdown.vx_reduction;
    end
    
    % Enforce centripetal acceleration limit
    if kappa > 1e-6
        vx_max_centripetal = sqrt(chassis.accel_limit / kappa);
        vx_limited = min(vx_limited, vx_max_centripetal);
    end
end
```

**4.2 Acceleration Limiting**
```matlab
function [vx_limited, accel] = applyAccelerationLimiting(...
    vx_desired, vx_prev, accel_prev, dt, chassis)
    %APPLYACCELERATIONLIMITING Limit acceleration rate
    %#codegen
    
    % Compute desired acceleration
    accel_desired = (vx_desired - vx_prev) / dt;
    
    % Determine limits based on sign
    if accel_desired > 0
        accel_limit = chassis.accel_limit;
    else
        accel_limit = -chassis.decel_limit;
    end
    
    % Clamp acceleration
    accel = clamp(accel_desired, -chassis.decel_limit, chassis.accel_limit);
    
    % Apply acceleration
    vx_limited = vx_prev + accel * dt;
end
```

**4.3 Jerk Limiting**
```matlab
function [vx_smooth, accel_smooth] = applyJerkLimiting(...
    vx_desired, accel, accel_prev, dt, chassis)
    %APPLYJERKLIMITING Limit jerk (rate of acceleration change)
    %#codegen
    
    % Compute desired jerk
    jerk_desired = (accel - accel_prev) / dt;
    
    % Clamp jerk
    jerk = clamp(jerk_desired, -chassis.jerk_limit, chassis.jerk_limit);
    
    % Apply jerk
    accel_smooth = accel_prev + jerk * dt;
    
    % Reconstruct velocity
    vx_smooth = vx_desired;  % Already has accel applied
end
```

### Phase 5: Path Preprocessing Integration ✅

**5.1 Preprocessing Workflow**
```matlab
% OFFLINE: In MATLAB or Python preprocessing tool
rawPath = loadRawPath();  % Nx3 [x y theta]

% Run preprocessing
PathInfo = gik9dof.control.preparePathForFollower(rawPath, chassisParams);

% Save for deployment
save('pathinfo.mat', 'PathInfo');
```

**5.2 Runtime Integration**
```matlab
% In C++/ROS2: Load preprocessed path
params.PathInfo = loadPathInfo('pathinfo.mat');

% Initialize state
state = struct();

% Control loop
while ~done
    [vx, wz, state, status] = purePursuitFollowerCodegen(...
        pose, dt, state, params);
    
    publishCommand(vx, wz);
    
    if status.isFinished
        break;
    end
end
```

---

## Implementation Checklist

### Week 1: Core Refactoring (Days 1-3)

- [ ] **Day 1: Structure Setup**
  - [ ] Create `purePursuitFollowerCodegen.m` skeleton
  - [ ] Define params struct with all fields
  - [ ] Define state struct with all fields
  - [ ] Define status struct with all fields
  - [ ] Add `%#codegen` directive
  - [ ] Create initialization function
  
- [ ] **Day 2: Core Algorithm**
  - [ ] Implement main function logic
  - [ ] Implement mode dispatch (pure pursuit/stanley/blended)
  - [ ] Port findLookaheadPoint from class
  - [ ] Port findNearestPathPoint from class
  - [ ] Port computeCurvatureToTarget from class
  - [ ] Implement updateStatus helper
  
- [ ] **Day 3: Limiting Functions**
  - [ ] Implement applyCurvatureLimiting
  - [ ] Implement applyAccelerationLimiting
  - [ ] Implement applyJerkLimiting
  - [ ] Implement applyWheelSpeedLimiting
  - [ ] Add parameter validation

### Week 1: Testing (Days 4-5)

- [ ] **Day 4: MATLAB Testing**
  - [ ] Create test script with sample paths
  - [ ] Test pure pursuit mode
  - [ ] Test stanley mode
  - [ ] Test blended mode
  - [ ] Verify acceleration limiting
  - [ ] Verify jerk limiting
  - [ ] Compare against class version
  
- [ ] **Day 5: Codegen Verification**
  - [ ] Run MATLAB Coder on function
  - [ ] Fix any codegen errors
  - [ ] Verify generated C++ compiles
  - [ ] Create code generation script
  - [ ] Document codegen settings

### Week 2: Integration (Days 6-7)

- [ ] **Day 6: C++ Integration**
  - [ ] Generate C++ code for ARM64
  - [ ] Update ROS2 CMakeLists.txt
  - [ ] Replace old purePursuit calls
  - [ ] Add new parameters to config
  - [ ] Build and test in simulation
  
- [ ] **Day 7: Field Testing**
  - [ ] Test straight line tracking
  - [ ] Test curve following
  - [ ] Test mode switching
  - [ ] Tune parameters
  - [ ] Document results

---

## File Structure

```
matlab/
├── +gik9dof/
│   └── +control/
│       ├── purePursuitFollowerCodegen.m          ← NEW: Main function
│       ├── purePursuitFollower.m                 ← KEEP: Original class for MATLAB
│       ├── preparePathForFollower.m              ← KEEP: Preprocessing
│       ├── rsClothoidRefine.m                    ← KEEP: Smoothing
│       └── helpers/
│           ├── applyCurvatureLimiting.m          ← NEW: Helper
│           ├── applyAccelerationLimiting.m       ← NEW: Helper
│           ├── applyJerkLimiting.m               ← NEW: Helper
│           └── applyWheelSpeedLimiting.m         ← NEW: Helper
│
scripts/
└── codegen/
    ├── generate_code_purepursuit_enhanced.m      ← NEW: Codegen script
    └── test_purepursuit_enhanced.m               ← NEW: Test script
│
codegen/
└── purepursuit_enhanced_arm64/                   ← NEW: Generated C++
```

---

## Risk Mitigation

### Risk 1: Codegen Compatibility Issues
- **Mitigation:** Incremental development with frequent codegen tests
- **Fallback:** Simplify features if needed

### Risk 2: Performance Degradation
- **Mitigation:** Profile generated C++ code
- **Fallback:** Optimize critical paths

### Risk 3: Behavioral Differences
- **Mitigation:** Extensive comparison testing against class version
- **Fallback:** Keep class version as reference

### Risk 4: Field Testing Delays
- **Mitigation:** Thorough simulation testing first
- **Fallback:** Deploy alongside old version with runtime switch

---

## Success Criteria

- [ ] All 3 controller modes working in C++
- [ ] Acceleration/jerk limiting functional
- [ ] Curvature-based speed reduction functional
- [ ] Generated C++ code compiles for ARM64
- [ ] Passes all MATLAB unit tests
- [ ] Performance within 10% of class version
- [ ] Field testing shows improved tracking
- [ ] No overshooting in curves
- [ ] Smooth motion (no jerks)

---

## Next Steps

**DECISION POINT:** Ready to start implementation?

1. ✅ Create skeleton function with structs
2. ✅ Port core algorithm from class
3. ✅ Implement limiting functions
4. ✅ Test in MATLAB
5. ✅ Generate C++ and integrate

---

**END OF REFACTORING PLAN**
