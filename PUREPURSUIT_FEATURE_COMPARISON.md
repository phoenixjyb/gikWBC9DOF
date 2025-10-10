# Pure Pursuit Feature Comparison & Integration Analysis
**Date:** October 10, 2025  
**Purpose:** Compare old (C++) vs new (MATLAB class) implementations and plan integration

---

## Executive Summary

🔍 **Field Testing Issue:** Your chassis doesn't move as intended with the current C++ pure pursuit  
🎯 **Goal:** Identify improvements in new MATLAB code that should be backported to C++  
📊 **Finding:** **NEW CLASS HAS SIGNIFICANT ENHANCEMENTS** that explain field issues

---

## 1. Implementation Comparison Matrix

| Feature | Old (C++) `purePursuitVelocityController.m` | New Class `purePursuitFollower.m` | Impact |
|---------|---------------------------------------------|-----------------------------------|---------|
| **Architecture** | Function-based, stateful | Class-based, OOP | ⚠️ Harder to codegen |
| **Controller Modes** | ❌ Pure pursuit only | ✅ 3 modes: blended/purePursuit/stanley | 🔴 **CRITICAL MISSING** |
| **Chassis Model** | ✅ Basic (track, limits) | ✅ **Advanced** (accel, jerk, curvature) | 🔴 **CRITICAL MISSING** |
| **Path Preprocessing** | ❌ None | ✅ `preparePathForFollower` (curvature, arc length) | 🔴 **MAJOR MISSING** |
| **Curvature Handling** | ❌ None | ✅ Curvature-based speed reduction | 🔴 **MAJOR MISSING** |
| **Acceleration Limits** | ❌ None | ✅ Acceleration + jerk limiting | 🔴 **MAJOR MISSING** |
| **Wheel Speed Limiting** | ✅ Basic | ✅ **Advanced** (per-wheel saturation) | 🟡 Enhancement |
| **Cross-Track Error** | ❌ Not computed | ✅ Computed and logged | 🟡 Diagnostic |
| **Status Reporting** | ❌ Minimal | ✅ **Comprehensive** (14 fields) | 🟡 Diagnostic |
| **Reverse Support** | ✅ Bidirectional | ✅ Bidirectional | ✅ Equal |
| **Lookahead Adaptation** | ✅ Base + vel + time | ✅ Base + vel + **accel** | 🟡 Enhancement |
| **Goal Detection** | ✅ Tolerance-based | ✅ Tolerance-based | ✅ Equal |
| **Heading Controller** | ❌ None | ✅ PID with I-term clamping | 🔴 **MAJOR MISSING** |
| **Stanley Controller** | ❌ None | ✅ Cross-track + heading | 🔴 **MAJOR MISSING** |
| **Blended Mode** | ❌ None | ✅ **Speed-dependent blend** | 🔴 **MAJOR MISSING** |

### Legend:
- 🔴 **CRITICAL MISSING**: Explains field performance issues
- 🟡 Enhancement: Nice to have
- ✅ Equal: Feature parity

---

## 2. Detailed Feature Analysis

### A. **CRITICAL: Chassis Model** 🔴

**Old (C++):**
```matlab
% Only basic parameters:
- track (wheel separation)
- vxMax, vxMin (speed limits)
- wzMax (angular rate limit)
- vwheelMax (wheel speed limit)
```

**New (Class):**
```matlab
% Advanced chassis model:
Chassis = struct(
    'track', 0.573,              % Wheel separation (m)
    'wheel_speed_max', 3.3,      % Max wheel speed (m/s)
    'vx_max', 1.5,               % Max forward speed
    'vx_min', -0.4,              % Max reverse speed
    'wz_max', 2.5,               % Max yaw rate
    'accel_limit', 1.2,          % ← NEW: Acceleration limit (m/s²)
    'decel_limit', 1.8,          % ← NEW: Deceleration limit (m/s²)
    'jerk_limit', 5.0,           % ← NEW: Jerk limit (m/s³)
    'wheel_base', 0.36,          % ← NEW: For stability calculations
    'curvature_slowdown', struct(...  % ← NEW: Speed reduction in curves
        'kappa_threshold', 0.9, ...
        'vx_reduction', 0.6), ...
    'reverse_enabled', false
)
```

**Impact:** Without acceleration/jerk limiting, your robot may:
- ❌ Accelerate too aggressively (wheel slip)
- ❌ Overshoot waypoints
- ❌ Struggle in tight curves
- ❌ Have jerky motion

---

### B. **CRITICAL: Controller Modes** 🔴

**Old (C++):**
- Only pure pursuit algorithm
- No fallback strategies
- Single control law

**New (Class):**
```matlab
ControllerMode options:
1. "purePursuit" - Geometric path tracking
2. "stanley"     - Cross-track error correction
3. "blended"     - Speed-dependent blend of both (DEFAULT)
```

**Blended Mode Logic:**
```matlab
% At low speeds: Use Stanley (better at stopping)
% At high speeds: Use Pure Pursuit (smoother)
% Blend based on velocity
```

**Impact:** Single mode may cause:
- ❌ Poor low-speed tracking (pure pursuit struggles near zero velocity)
- ❌ No correction for cross-track errors
- ❌ Instability at start/stop

---

### C. **CRITICAL: Path Preprocessing** 🔴

**Old (C++):**
```matlab
% Path is just waypoints [x y theta]
% No preprocessing
% No curvature information
% No arc length tracking
```

**New (Class):**
```matlab
% Calls preparePathForFollower() which computes:
PathInfo = struct(
    'States', pathStates,         % Original waypoints
    'Curvature', kappa,           % ← Curvature at each point
    'ArcLength', s,               % ← Cumulative arc length
    'DistanceRemaining', d_rem,   % ← Distance to goal
    'SegmentIndex', seg_idx,      % ← Segment boundaries
    'Diagnostics', diag           % ← Warnings/issues
)
```

**Functions involved:**
- `preparePathForFollower.m` - Main preprocessor
- `rsRefinePath.m` - Path refinement
- `rsClothoidRefine.m` - Clothoid spline smoothing

**Impact:** Without preprocessing:
- ❌ No speed reduction in tight curves → **overshooting**
- ❌ No smooth velocity profiles
- ❌ Jerky motion in complex paths
- ❌ This likely explains your field issues!

---

### D. **CRITICAL: Curvature-Based Speed Control** 🔴

**Old (C++):**
```matlab
% Speed is constant (vxNominal)
% No awareness of path geometry
```

**New (Class):**
```matlab
% Speed reduces based on path curvature:
if abs(kappa) > kappa_threshold
    vx_desired = vx_desired * vx_reduction;  % e.g., 0.6× slower
end

% Also respects centripetal acceleration limit:
vx_max_curve = sqrt(a_max / abs(kappa))
```

**Impact:** Without curvature awareness:
- ❌ **Robot goes too fast in curves** → overshooting
- ❌ Loss of traction
- ❌ Path deviation
- ❌ **THIS IS LIKELY YOUR MAIN FIELD ISSUE**

---

### E. **MAJOR: Acceleration/Jerk Limiting** 🔴

**Old (C++):**
```matlab
% Commands change instantly
% No smoothing
```

**New (Class):**
```matlab
% Limits rate of change:
accel = (vx_desired - last_vx) / dt;
accel = clamp(accel, -decel_limit, accel_limit);
vx_cmd = last_vx + accel * dt;

% Also limits jerk (rate of acceleration change)
jerk = (accel - last_accel) / dt;
jerk = clamp(jerk, -jerk_limit, jerk_limit);
```

**Impact:** Without smoothing:
- ❌ Jerky motion
- ❌ Mechanical stress
- ❌ Wheel slip
- ❌ Poor ride quality

---

## 3. Comparison: unifiedChassisCtrl vs simulateChassisController

### Purpose Alignment

| Aspect | `unifiedChassisCtrl` | `simulateChassisController` |
|--------|---------------------|---------------------------|
| **Purpose** | Convert references to commands | Simulate path execution |
| **Modes** | 3: holistic/staged-C/staged-B | 3: 0=diff, 1=heading, 2=pursuit |
| **Architecture** | Function, codegen-ready | Function → delegates to class |
| **Usage** | Real-time control interface | Simulation/testing |
| **Codegen** | ✅ Compatible (plain struct) | ❌ Uses class for mode 2 |

### Mode Comparison

**`unifiedChassisCtrl` modes:**
```matlab
1. "holistic"  - GIK full-body control (pose → velocity)
2. "staged-C"  - Stage C: arm control only
3. "staged-B"  - Stage B: base follower
```

**`simulateChassisController` modes:**
```matlab
0. Legacy: 5-point differentiation (feedforward replay)
1. Simple: Heading controller (P + feedforward yaw)
2. Pure Pursuit: Delegates to purePursuitFollower class
```

### Functionality Overlap

**NO OVERLAP** - They serve different purposes:
- `unifiedChassisCtrl`: **Real-time command converter** (deployed in C++)
- `simulateChassisController`: **Simulation wrapper** (MATLAB testing only)

**Current C++ Deployment:**
```cpp
// ros2/gik9dof_solver uses:
1. holisticVelocityController → unifiedChassisCtrl (mode dispatch)
2. purePursuitVelocityController (standalone, mode 2 equivalent)
```

---

## 4. RS Smoothing & Clothoid Integration

### New Smoothing Tools

**From origin/main merge:**

1. **`rsClothoidRefine.m`** (204 lines)
   - Fits clothoid splines to SE(2) paths
   - Splits at gear changes (forward/reverse)
   - Uses MATLAB's `referencePathFrenet`
   - Produces smooth curvature profiles

2. **`rsRefinePath.m`** (291 lines)
   - Wrapper around rsClothoidRefine
   - Handles edge cases
   - Validates output

3. **`preparePathForFollower.m`** (254 lines)
   - **Main integration point**
   - Calls rsRefinePath → rsClothoidRefine
   - Computes curvature, arc length
   - Returns PathInfo struct

### Integration with Pure Pursuit

**Current Flow (NEW):**
```
Raw Path → preparePathForFollower()
              ↓
         rsRefinePath()
              ↓
         rsClothoidRefine()
              ↓
         PathInfo (with curvature)
              ↓
         purePursuitFollower.step()
              ↓
         Speed-adjusted commands
```

**Old Flow (C++):**
```
Raw Path → purePursuitVelocityController()
              ↓
         Commands (no preprocessing)
```

### Why This Matters

**Your field issue likely stems from:**
1. ❌ No path smoothing → jerky waypoints
2. ❌ No curvature computation → no speed adjustment
3. ❌ No acceleration limiting → aggressive motion
4. ❌ Single controller mode → poor low-speed performance

---

## 5. Parameter Tunability Comparison

### Old (C++) Parameters (13 total)

**ROS2 configuration:**
```yaml
purepursuit:
  lookahead_base: 0.8          # Base lookahead (m)
  lookahead_vel_gain: 0.3      # Velocity-dependent gain
  lookahead_time_gain: 0.1     # Time-dependent gain
  vx_nominal: 1.0              # Nominal speed (m/s)
  vx_max: 1.5                  # Max forward speed
  vx_min: -1.0                 # Max reverse speed
  wz_max: 2.0                  # Max yaw rate
  track: 0.674                 # Wheel separation (m)
  vwheel_max: 2.0              # Max wheel speed
  waypoint_spacing: 0.15       # Min spacing (m)
  path_buffer_size: 30.0       # Buffer size
  goal_tolerance: 0.2          # Goal threshold (m)
  interp_spacing: 0.05         # Interpolation spacing
```

### New (Class) Parameters (30+ total)

**Additional tuning knobs:**
```matlab
ChassisParams:
  accel_limit: 1.2             % ← NEW: Acceleration limit
  decel_limit: 1.8             % ← NEW: Deceleration limit
  jerk_limit: 5.0              % ← NEW: Jerk limit
  wheel_base: 0.36             % ← NEW: Wheelbase
  curvature_slowdown:          % ← NEW: Curve speed reduction
    kappa_threshold: 0.9
    vx_reduction: 0.6

ControllerMode: "blended"      % ← NEW: Mode selection

Heading PID:                   % ← NEW: PID tuning
  HeadingKp: 1.2
  HeadingKi: 0.0
  HeadingKd: 0.1
  FeedforwardGain: 0.9

Path Preprocessing:            % ← NEW: Smoothing options
  discretizationDistance: 0.05
  maxNumWaypoints: 0

Status/Logging:                % ← NEW: Diagnostics
  crossTrackError
  curvature
  acceleration
  wheelSpeeds
  warnings[]
```

**Tunability Ratio:** New has **~2.3× more parameters** for fine-tuning

---

## 6. Root Cause Analysis: Field Issues

### Likely Problems with Current C++ Implementation

Based on comparison, your field issues probably stem from:

#### **Problem 1: Speed Too High in Curves** 🔴
- **Symptom:** Robot overshoots waypoints in turns
- **Cause:** No curvature-based speed reduction
- **Fix:** Add curvature computation + speed limiting

#### **Problem 2: Jerky Motion** 🔴
- **Symptom:** Abrupt accelerations, wheel slip
- **Cause:** No acceleration/jerk limiting
- **Fix:** Add velocity smoothing with accel limits

#### **Problem 3: Poor Low-Speed Tracking** 🔴
- **Symptom:** Instability near goal, oscillation at start
- **Cause:** Pure pursuit struggles at low velocities
- **Fix:** Add blended mode (Stanley at low speed)

#### **Problem 4: Aggressive Commands** 🔴
- **Symptom:** Commands change too quickly
- **Cause:** No rate limiting on outputs
- **Fix:** Add jerk limiting

#### **Problem 5: No Path Smoothing** 🔴
- **Symptom:** Follows waypoints too literally, zigzag motion
- **Cause:** No clothoid smoothing
- **Fix:** Preprocess paths with rsClothoidRefine

---

## 7. Integration Strategy Options

### Option A: **Backport Key Features to Old C++** ✅ RECOMMENDED

**Approach:** Enhance existing `purePursuitVelocityController.m`, regenerate C++

**Changes needed:**
1. Add chassis parameters (accel_limit, jerk_limit, curvature_slowdown)
2. Add velocity smoothing function
3. Add curvature computation (simplified, no full preprocessing)
4. Add mode selection (keep pure pursuit, add simple Stanley fallback)
5. Keep codegen-compatible (no classes, no inputParser)

**Effort:** Medium (2-3 days implementation + testing)  
**Risk:** Low (incremental improvements to proven base)  
**Value:** High (fixes field issues while keeping C++ deployment)

---

### Option B: **Create Hybrid System** 🟡 MEDIUM EFFORT

**Approach:** Use new tools for preprocessing, old function for control

**Architecture:**
```matlab
% Offline/initialization:
PathInfo = preparePathForFollower(rawPath, chassis);
% Extract: curvature[], arcLength[], smoothedStates[]

% Real-time loop (C++):
[vx, wz] = purePursuitVelocityController_enhanced(...
    refX, refY, refTheta, ...
    curvature,    % ← NEW INPUT
    params_enhanced)  % ← Add accel/jerk limits
```

**Effort:** Medium (modify both preprocessing and controller)  
**Risk:** Medium (new workflow, more complex)  
**Value:** High (gets best of both worlds)

---

### Option C: **Full Rewrite for Codegen** ❌ NOT RECOMMENDED

**Approach:** Rewrite purePursuitFollower class as codegen-compatible function

**Effort:** High (4-5 days, complex refactoring)  
**Risk:** High (many edge cases, hard to debug)  
**Value:** Medium (gets all features but takes longest)

---

## 8. Recommended Action Plan

### Phase 1: **Quick Wins** (1-2 days)

1. ✅ **Add acceleration limiting** to existing C++ controller
   ```matlab
   % In purePursuitVelocityController.m:
   accel = (vx_desired - stateOut.prevVx) / dt;
   accel = clamp(accel, -params.decelLimit, params.accelLimit);
   vx_cmd = stateOut.prevVx + accel * dt;
   ```

2. ✅ **Add jerk limiting**
   ```matlab
   jerk = (accel - stateOut.prevAccel) / dt;
   jerk = clamp(jerk, -params.jerkLimit, params.jerkLimit);
   accel_smooth = stateOut.prevAccel + jerk * dt;
   ```

3. ✅ **Add basic curvature speed reduction**
   ```matlab
   % Estimate curvature from path curvature
   if abs(wz / vx) > params.kappaThreshold
       vx_desired = vx_desired * params.vxReduction;
   end
   ```

4. ✅ **Regenerate C++ code**

5. ✅ **Test in simulation, then field**

### Phase 2: **Path Preprocessing** (2-3 days)

1. ✅ Create offline tool using `preparePathForFollower`
2. ✅ Generate PathInfo files for planned trajectories
3. ✅ Modify controller to accept curvature array
4. ✅ Implement proper curvature-based speed profiles

### Phase 3: **Controller Modes** (3-4 days)

1. ✅ Add simple Stanley controller (codegen-compatible)
2. ✅ Add blended mode (speed-dependent switch)
3. ✅ Add mode parameter to ROS2 config
4. ✅ Field test all modes

---

## 9. Specific Code Recommendations

### A. Enhanced Parameters Struct

```matlab
% Add to params in purePursuitVelocityController.m:
params.accelLimit = 1.2;           % m/s²
params.decelLimit = 1.8;           % m/s²
params.jerkLimit = 5.0;            % m/s³
params.kappaThreshold = 0.9;       % rad/m
params.vxReduction = 0.6;          % 60% speed in curves
params.controllerMode = 0;         % 0=pursuit, 1=stanley, 2=blended
params.stanleyGain = 0.5;          % Cross-track gain
```

### B. Enhanced State Struct

```matlab
% Add to state in purePursuitVelocityController.m:
stateOut.prevVx = vx_cmd;          % For accel limiting
stateOut.prevAccel = accel;        % For jerk limiting
stateOut.crossTrackError = cte;    % For Stanley mode
```

### C. Acceleration Limiting Function

```matlab
function vx_smooth = limitAcceleration(vx_desired, vx_prev, accel_prev, dt, params)
    %LIMITACCELERATION Apply acceleration and jerk limits
    
    % Compute desired acceleration
    accel_desired = (vx_desired - vx_prev) / dt;
    
    % Limit acceleration
    accel = clamp(accel_desired, -params.decelLimit, params.accelLimit);
    
    % Limit jerk
    jerk = (accel - accel_prev) / dt;
    jerk = clamp(jerk, -params.jerkLimit, params.jerkLimit);
    accel_smooth = accel_prev + jerk * dt;
    
    % Apply smoothed acceleration
    vx_smooth = vx_prev + accel_smooth * dt;
end
```

### D. Curvature Speed Reduction

```matlab
function vx_limited = applyCurvatureLimit(vx_desired, curvature, params)
    %APPLYCURVATURELIMIT Reduce speed in high-curvature sections
    
    kappa = abs(curvature);
    
    if kappa > params.kappaThreshold
        % Reduce speed proportionally
        vx_limited = vx_desired * params.vxReduction;
    else
        vx_limited = vx_desired;
    end
    
    % Also enforce centripetal acceleration limit
    if kappa > 1e-6
        vx_max_centripetal = sqrt(params.accelLimit / kappa);
        vx_limited = min(vx_limited, vx_max_centripetal);
    end
end
```

---

## 10. Testing & Validation Plan

### Simulation Tests

1. **Step Response** - Test acceleration/jerk limiting
2. **Curve Following** - Test curvature speed reduction
3. **Low-Speed Maneuvers** - Test mode switching (if implemented)
4. **Reverse Paths** - Test bidirectional behavior

### Field Tests

1. **Straight Line** - Verify basic tracking
2. **90° Turn** - Verify curve slowdown
3. **S-Curve** - Verify smooth transitions
4. **Figure-8** - Verify continuous operation
5. **Reverse Path** - Verify bidirectional

### Success Criteria

- ✅ No overshooting in curves
- ✅ Smooth acceleration (no jerks)
- ✅ Stable at low speeds
- ✅ Consistent tracking error < 10cm
- ✅ No wheel slip

---

## 11. Documentation Updates Needed

After implementation:

1. Update `PUREPURSUIT_INVENTORY.md` with new features
2. Create `PUREPURSUIT_ENHANCEMENT_SUMMARY.md`
3. Update ROS2 parameter documentation
4. Add field test results
5. Update tunability guide with new parameters

---

## 12. Summary & Next Steps

### Key Findings

🔴 **CRITICAL GAPS** in current C++ implementation:
1. No acceleration/jerk limiting → jerky motion
2. No curvature awareness → overshooting in curves
3. No path preprocessing → no smooth velocity profiles
4. Single controller mode → poor low-speed performance

### Immediate Actions

**DECISION POINT:** Which option do you prefer?

**Option A** (Recommended): Backport key features to existing C++
- ✅ Fastest to production (2-3 days)
- ✅ Keeps proven architecture
- ✅ Incremental improvement
- ✅ Low risk

**Option B**: Hybrid with preprocessing
- ⚠️ More complex
- ⚠️ New workflow
- ✅ Gets full feature set

### Questions for You

1. **What specific field issues are you seeing?**
   - Overshooting? Jerky motion? Instability?
   
2. **What's your timeline?**
   - Quick fix needed? Or time for full implementation?

3. **Do you have simulation environment?**
   - Can we test before field deployment?

4. **Are paths known in advance?**
   - If yes, offline preprocessing is viable

---

**END OF COMPARISON ANALYSIS**
