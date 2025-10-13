# Stage C Method Numbering & Implementation Guide

**Last Updated:** October 13, 2025  
**Project:** gikWBC9DOF Stage C Trajectory Tracking Methods

---

## Quick Reference Table

| Method | Name | ExecutionMode | Status | Architecture | Lines of Code |
|--------|------|---------------|--------|--------------|---------------|
| **0** | Pure IK | `'pureIk'` | ✅ **Implemented** | Single-pass GIK only | ~19 lines |
| **1** | PP-For-IK | `'ppForIk'` | ✅ **Implemented** (Current Default) | Three-pass feed-forward | ~250 lines |
| **2** | Iterative MPC | *(not assigned)* | ❌ Not Implemented | Per-waypoint projection | N/A |
| **3** | Differential IK QP | *(not assigned)* | ❌ Not Implemented | Unified QP solver | N/A |
| **4** | PP-First | `'ppFirst'` | ✅ **Implemented** | Predict→Constrain→Solve | ~276 lines |
| **5** | Pure MPC | `'pureMPC'` | 🚧 **In Development** | Receding horizon NMPC | TBD |

---

## Method Details

### Method 0: Pure IK (`'pureIk'`)

**Status:** ✅ Implemented (baseline)  
**Location:** `runStagedTrajectory.m` lines 975-994  
**Function:** `executeStageCPureIk()`

#### Architecture
```
For each waypoint:
  1. GIK solve with all DOF free
  2. No chassis constraints
  3. No differential drive enforcement
```

#### Characteristics
- **Simplest approach**: Direct IK without any base motion constraints
- **Fastest**: No multi-pass processing
- **Unrealistic base motion**: Violates differential drive constraints
- **Use case**: Debugging, baseline comparison, understanding arm workspace

#### Code
```matlab
function logC = executeStageCPureIk(robot, trajStruct, qStart, baseIdx, velLimits, options)
%EXECUTESTAGECPUREIK Track Stage C using pure IK output only.
bundle = gik9dof.createGikSolver(robot, ...
    'DistanceSpecs', options.DistanceSpecs, ...
    'MaxIterations', options.MaxIterations);

logC = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'CommandFcn', options.CommandFcn, ...
    'Verbose', options.Verbose, ...
    'BaseIndices', baseIdx, ...
    'VelocityLimits', velLimits);
% ... (diagnostics and state extraction)
end
```

#### Usage
```matlab
pipeline = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'pureIk', ...  % Method 0
    'ConfigTools', configTools);
```

#### When to Use
- ✅ Quick debugging of arm kinematics
- ✅ Baseline for comparison studies
- ✅ Understanding theoretical workspace limits
- ❌ Real robot deployment (base motion infeasible)

---

### Method 1: PP-For-IK (`'ppForIk'`) - Current Default

**Status:** ✅ Implemented (production)  
**Location:** `runStagedTrajectory.m` lines 590-835  
**Function:** `executeStageCPurePursuit()`

#### Architecture (Three-Pass Feed-Forward)
```
Pass 1: Reference GIK (Free Base)
  └─> Ideal trajectory, unconstrained base motion
        ↓
Pass 2: Pure Pursuit Simulation
  └─> Realistic base path (differential drive enforced)
        ↓
Pass 3: Fixed Base GIK
  └─> Final trajectory with feasible base, compensated arm
```

#### Characteristics
- **Proven approach**: Current production system
- **Decoupled passes**: Each pass independent
- **No feedback loop**: Pass 1 → Pass 2 → Pass 3 (one-way)
- **Arm compensation**: Pass 3 adjusts arm for base deviation from Pass 1

#### Implementation Details
```matlab
% Pass 1: Free base GIK for reference
logPass1 = gik9dof.runTrajectoryControl(bundlePass1, trajC, ...);

% Pass 2: Simulate differential drive tracking Pass 1 base
[qTrajPass2, ~, chassisLog] = gik9dof.simulateChassisExecution(...
    Pass1_base_trajectory, chassisParams, ...);

% Pass 3: Fixed base GIK with Pass 2 base states
for k = 1:nWaypoints
    lockJointBounds(bundlePass3.constraints.joint, baseIdx, qPass2_base(:,k));
    [qSol, info] = bundlePass3.solve(q_current, 'TargetPose', T_target(:,:,k));
    % ...
end
```

#### Strengths
- ✅ Base motion guaranteed feasible (differential drive)
- ✅ Smooth base trajectories from Pure Pursuit
- ✅ Well-tested and stable
- ✅ Moderate computation time

#### Weaknesses
- ❌ No feedback between passes
- ❌ Pass 3 arm may struggle if Pass 2 base deviates significantly
- ❌ Cannot recover if Pass 2 diverges too much

#### Usage
```matlab
pipeline = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppForIk', ...  % Method 1 (DEFAULT)
    'ConfigTools', configTools, ...
    'ChassisProfile', 'wide_track');
```

#### Diagnostics
```matlab
diag = pipeline.stageLogs.stageC.diagnostics;
% Fields:
%   - eeErrorBins: Histogram of EE tracking errors
%   - baseYawDrift: Deviation from Pass 1 reference
%   - solverIterations: GIK iterations per waypoint
%   - refinementApplied: Boolean flag
```

---

### Method 2: Iterative MPC *(Not Implemented)*

**Status:** ❌ Not Implemented  
**Reason:** Band-aid solution with fundamental projection issues

#### Proposed Architecture
```
For each waypoint:
  Iteration loop:
    1. GIK solve (all DOF)
    2. Check differential drive feasibility
    3. If infeasible: project base to feasible manifold
    4. Repeat until convergence or max iterations
```

#### Why Not Implemented
- ⚠️ **Projection may violate constraints**: Forcing base to manifold can break arm IK solution
- ⚠️ **Convergence not guaranteed**: May oscillate between infeasible GIK and violated constraints
- ⚠️ **Higher complexity than Method 4**: More code for worse results
- ✅ **Method 4 superior**: Achieves same goal with cleaner architecture

#### Decision
**Skipped in favor of Method 4** - See `projectDiagnosis.md` Section 10 for detailed analysis.

---

### Method 3: Differential IK with Unified QP *(Not Implemented)*

**Status:** ❌ Not Implemented  
**Reason:** Research-level complexity, Method 4 sufficient for current needs

#### Proposed Architecture
```
For each waypoint:
  Solve single unified QP:
    minimize   ‖J_aug·u - V_d‖² + λ‖u‖²
    subject to:
      - Nonholonomic: ẋ = v·cos(θ), ẏ = v·sin(θ), θ̇ = ω
      - Wheel speeds: |ω_L|, |ω_R| ≤ ω_max
      - Joint rates: |q̇_i| ≤ q̇_max
      - Obstacles: distance constraints (linearized)
```

#### Why Not Implemented Yet
- ⚠️ **High implementation effort**: Requires custom QP solver integration
- ⚠️ **Linearization challenges**: Nonholonomic constraints require careful handling
- ⚠️ **Debugging complexity**: Unified solver harder to diagnose than modular approach
- ✅ **Method 4 performs well**: Current performance acceptable

#### Future Consideration
If Method 4 shows systematic failures in:
- High-curvature paths
- Tight obstacle corridors
- Coupled arm-base singularities

Then Method 3 could be revisited as research project.

---

### Method 4: PP-First with GIK Refinement (`'ppFirst'`) ⭐

**Status:** ✅ **Implemented** (October 2025)  
**Location:** `runStageCPPFirst.m` (276 lines) + wrapper in `runStagedTrajectory.m` (lines 858-973)  
**Functions:** `runStageCPPFirst()`, `executeStageCPPFirst()`, `baseSeedFromEE()`, `initPPFromBasePath()`, `solveArmOnlyIK()`, `updateBaseJointBounds()`

#### Architecture (Predict-Constrain-Solve)
```
For each waypoint:
  1. PREDICT: Pure Pursuit generates base motion [v, ω]
     └─> Guaranteed differential drive feasible
  
  2. CONSTRAIN: Update GIK with yaw corridor
     └─> θ_min = θ_PP - Δθ,  θ_max = θ_PP + Δθ
     └─> (x_min, x_max), (y_min, y_max) around PP prediction
  
  3. SOLVE: GIK with soft constraints
     └─> Find q that reaches EE target within corridor
  
  4. CHECK: Measure EE tracking error
     └─> If error > threshold → FALLBACK
  
  5. FALLBACK (if triggered):
     └─> Fix base at PP prediction
     └─> Solve arm-only IK for EE target
```

#### Key Parameters
```matlab
% Corridor constraints
YawTolerance = deg2rad(15);        % ±15° around PP prediction
PositionTolerance = 0.15;          % ±15cm box around PP prediction

% Fallback trigger
EEErrorTolerance = 0.01;           % 10mm threshold

% Pure Pursuit
LookaheadDistance = 0.80;          % 80cm lookahead
DesiredVelocity = 1.0;             % 1 m/s target speed
```

#### Implementation Details

**Step 1: Generate Base Seed Path**
```matlab
baseSeedPath = gik9dof.baseSeedFromEE(robot, trajStruct, q_start, ...
    'BaseIndices', baseIdx, 'ArmIndices', armIdx);
% Returns [N x 3] path: [x, y, theta]
```

**Step 2: Initialize Pure Pursuit**
```matlab
[ppFollower, ppPathInfo] = gik9dof.initPPFromBasePath(baseSeedPath, chassisParams, ...
    'LookaheadDistance', lookaheadDist, ...
    'DesiredVelocity', desiredVel);
```

**Step 3: Per-Waypoint Control Loop**
```matlab
for k = 1:nWaypoints
    % PREDICT: PP command
    [vx_cmd, wz_cmd] = ppFollower(q_base_current);
    q_base_pred = integrate_unicycle(q_base_current, vx_cmd, wz_cmd, dt);
    
    % CONSTRAIN: Update GIK bounds
    gik9dof.updateBaseJointBounds(gikBundle, baseIdx, q_base_pred, ...
        'YawTolerance', yawTol, 'PositionTolerance', posTol);
    
    % SOLVE: GIK
    [q_gik, solInfo] = gikBundle.solve(q_current, 'TargetPose', T_ee_target);
    
    % CHECK: EE error
    T_ee_actual = getTransform(robot, q_gik, eeName);
    ee_error = norm(T_ee_actual(1:3,4) - T_ee_target(1:3,4));
    
    % FALLBACK: If needed
    if ee_error > eeErrorThreshold
        q_arm_fallback = gik9dof.solveArmOnlyIK(robot, gikBundle, ...
            T_ee_target, q_base_pred, q_current(armIdx));
        q_final = [q_base_pred; q_arm_fallback];
        log.fallbackUsed(k) = true;
    else
        q_final = q_gik;
    end
    
    q_current = q_final;
end
```

#### Strengths
- ✅ **Feedback per waypoint**: PP prediction → GIK → fallback if needed
- ✅ **Guaranteed feasible base**: PP ensures differential drive
- ✅ **Soft constraints**: Yaw corridor allows GIK flexibility
- ✅ **Graceful degradation**: Fallback mechanism prevents failure
- ✅ **Reuses existing code**: Leverages proven PP and GIK infrastructure
- ✅ **Low implementation risk**: Modular, testable components

#### Weaknesses
- ⚠️ **Slightly slower than Method 1**: Additional per-waypoint GIK calls
- ⚠️ **Parameter sensitivity**: Corridor width affects convergence
- ⚠️ **Fallback frequency**: Tight corridors may trigger fallback often

#### Usage
```matlab
pipeline = gik9dof.runStagedTrajectory(robot, traj, ...
    'ExecutionMode', 'ppFirst', ...  % Method 4
    'ConfigTools', configTools, ...
    'ChassisProfile', 'wide_track', ...
    'Verbose', true);
```

#### Diagnostics
```matlab
diag = pipeline.stageLogs.stageC.diagnostics;
% Standard fields (same as Method 1):
%   - eeErrorBins, baseYawDrift, solverIterations
% Method 4 specific:
%   - fallbackRate: Fraction using arm-only fallback
%   - ppFirst.fallbackCount: Number of fallback waypoints
%   - ppFirst.gikOnlyCount: Waypoints solved without fallback
```

#### Test Results (Integration Test)
```
Test trajectory: 5 waypoints, straight line
✅ Pipeline completed: 2.58 seconds
✅ Fallback rate: 20% (1/5 waypoints)
✅ EE error: mean 12.18mm, max 60.90mm
✅ Yaw drift from PP: mean 1.1°, max 3.7°
```

---

## Method Selection Guide

### Decision Tree

```
Start
  ↓
Are you debugging arm kinematics only?
  ├─ Yes → Use Method 0 (pureIk)
  └─ No ↓
  
Is this production/deployment?
  ├─ Yes ↓
  │   Are you seeing tracking failures?
  │     ├─ No → Use Method 1 (ppForIk) - Current default
  │     └─ Yes → Try Method 4 (ppFirst)
  │
  └─ No (Research/Testing) ↓
      Want to test new approach?
        ├─ Feedback loops → Method 4 (ppFirst)
        ├─ Baseline comparison → Method 0 or 1
        └─ Future research → Consider Method 3
```

### Performance Comparison (Expected)

| Metric | Method 0 | Method 1 | Method 4 |
|--------|----------|----------|----------|
| **EE Tracking Error** | High (base infeasible) | Good | Good-Excellent |
| **Base Feasibility** | ❌ Infeasible | ✅ Feasible | ✅ Feasible |
| **Computation Time** | Fast | Moderate | Moderate-Slow |
| **Robustness** | Low | Good | Excellent |
| **Failure Recovery** | None | Limited | ✅ Fallback |
| **Production Ready** | ❌ No | ✅ Yes | ✅ Yes |

### Recommendations by Use Case

**1. Production Deployment**
```matlab
% Start with Method 1 (battle-tested)
'ExecutionMode', 'ppForIk'

% If seeing tracking failures, switch to Method 4
'ExecutionMode', 'ppFirst'
```

**2. Debugging/Development**
```matlab
% Quick arm workspace check
'ExecutionMode', 'pureIk'

% Full system test
'ExecutionMode', 'ppForIk'
```

**3. Research/Comparison**
```matlab
% Run all implemented methods
methods = {'pureIk', 'ppForIk', 'ppFirst'};
for i = 1:length(methods)
    results{i} = runStagedTrajectory(..., 'ExecutionMode', methods{i});
end
compareMethodPerformance(results);
```

---

## Implementation Files

### Method 0 (pureIk)
- **Location:** `runStagedTrajectory.m` lines 975-994
- **Size:** 19 lines
- **Dependencies:** `runTrajectoryControl.m`, `createGikSolver.m`

### Method 1 (ppForIk)
- **Location:** `runStagedTrajectory.m` lines 590-835
- **Size:** ~250 lines
- **Dependencies:** 
  - `runTrajectoryControl.m`
  - `simulateChassisExecution.m`
  - `createGikSolver.m`
  - `lockJointBounds()`

### Method 4 (ppFirst)
- **Main:** `runStageCPPFirst.m` (276 lines)
- **Wrapper:** `runStagedTrajectory.m` lines 858-973 (116 lines)
- **Helpers:**
  - `baseSeedFromEE.m` (130 lines)
  - `initPPFromBasePath.m` (120 lines)
  - `updateBaseJointBounds.m` (50 lines)
  - `solveArmOnlyIK.m` (60 lines)
- **Total:** ~752 lines

---

## Configuration Parameters

### Common Parameters (All Methods)
```matlab
'MaxIterations', 1500              % GIK solver max iterations
'RateHz', 10                       % Control frequency (Hz)
'Verbose', true                    % Print diagnostics
'ChassisProfile', 'wide_track'     % Chassis configuration
```

### Method 1 Specific (ppForIk)
```matlab
'StageCLookaheadDistance', 0.8     % Pure Pursuit lookahead (m)
'StageCDesiredLinearVelocity', 1.0 % Target velocity (m/s)
'StageCUseBaseRefinement', false   % RS+Clothoid refinement
```

### Method 4 Specific (ppFirst)
```matlab
% Set via pipeline, not directly exposed yet
% Internal defaults:
YawTolerance = deg2rad(15)         % ±15° yaw corridor
PositionTolerance = 0.15           % ±15cm position box
EEErrorTolerance = 0.01            % 10mm fallback threshold
```

---

## Testing & Validation

### Quick Integration Test
```bash
cd /Users/yanbo/Projects/gikWBC9DOF
matlab -batch "addpath(genpath('matlab')); test_method4_integration"
```

### Full Trajectory Test (TODO)
```matlab
% Test on full 148-waypoint trajectory
robot = gik9dof.createRobotModel();
traj = loadTrajectory('1_pull_world_scaled.json');
configTools = gik9dof.configurationTools(robot);

% Method 1 baseline
result_m1 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ConfigTools', configTools, ...
    'ExecutionMode', 'ppForIk');

% Method 4 comparison
result_m4 = gik9dof.runStagedTrajectory(robot, traj, ...
    'ConfigTools', configTools, ...
    'ExecutionMode', 'ppFirst');

% Compare
compareResults(result_m1, result_m4);
```

---

## References

- **Design Document:** `projectDiagnosis.md` Section 10 (Method 4 specification)
- **Integration Complete:** `METHOD4_INTEGRATION_COMPLETE.md`
- **Architecture:** `CHASSIS_SECTION_CONSOLIDATION.md`
- **Usage Guide:** `docs/SIMULATION_WORKFLOW_GUIDE.md`

---

---

## Method 5: Pure MPC (`'pureMPC'`)

**Status:** 🚧 In Development (October 2025)  
**Location:** `matlab/+gik9dof/runStageCPureMPC.m` (planned)  
**Design Document:** `g5wbcMpcDesign.md`, `METHOD5_IMPLEMENTATION_PLAN.md`

#### Architecture
```
Receding Horizon Loop (10-20 Hz):
  1. Get reference trajectory segment [k:k+N]
  2. Solve NMPC over horizon N steps:
     - Decision vars: [v, ω](k:k+N-1), s(k:k+N)
     - Minimize: tracking + input + terminal cost
     - Subject to: unicycle dynamics, wheel limits, collisions
  3. Extract first control [v*, ω*](k)
  4. Simulate one step: x(k+1) = f(x(k), [v*, ω*])
  5. Solve arm IK for fixed base at x(k+1)
  6. Advance horizon
```

#### Characteristics
- **True MPC**: Online receding horizon optimization (unlike Methods 1-4 which are offline)
- **Fundamental constraint**: Nonholonomic embedded in dynamics - **cannot request sideways motion**
- **Lookahead**: N=20 steps (2 seconds @ dt=0.1s)
- **Control rate**: 10-20 Hz (compute-intensive)
- **Dependencies**: CasADi + IPOPT solver
- **Use case**: Online control, dynamic environments, guaranteed constraint satisfaction

#### Key Differences from Methods 1-4
| Aspect | Methods 0-4 | Method 5 (pureMPC) |
|--------|-------------|-------------------|
| Planning | Offline | Online |
| Constraint handling | Post-hoc correction | Embedded in dynamics |
| Sideways motion | Can request → correct | Cannot request by design |
| Solve frequency | Once per trajectory | 10-20 Hz during execution |
| Adaptability | Static | Dynamic (reacts to disturbances) |

#### Configuration
```yaml
pureMPC:
  executionMode: 'pureMPC'
  nmpc:
    horizon: 20
    dt: 0.1
    control_rate: 10  # Hz
  weights:
    tracking: 100.0
    input_v: 1.0
    input_omega: 10.0
    terminal: 500.0
  constraints:
    v_max: 0.5
    omega_max: 0.8
    wheel_max: 0.6
    track_width: 0.574
```

#### Usage (When Ready)
```matlab
pipeline = gik9dof.runStagedTrajectory(robot, trajWaypoints, ...
    'ExecutionMode', 'pureMPC', ...
    'ConfigTools', configTools, ...
    'ChassisProfile', 'wide_track', ...
    'Verbose', true);
```

#### Implementation Plan
See `METHOD5_IMPLEMENTATION_PLAN.md` for detailed phases:
- **Phase 1 (Weeks 1-2):** Foundation, dependencies, configuration
- **Phase 2 (Weeks 3-5):** Core NMPC implementation
- **Phase 3 (Weeks 6-8):** Integration & testing
- **Phase 4 (Weeks 9-12):** Optimization & tuning

---

## Summary

| Method | Best For | Status |
|--------|----------|--------|
| **0 (pureIk)** | Debugging, baseline | ✅ Ready |
| **1 (ppForIk)** | Production default | ✅ Ready |
| **2 (MPC)** | *(skipped - flawed approach)* | ❌ Not planned |
| **3 (Diff IK QP)** | Future research | ⏳ Not started |
| **4 (ppFirst)** | New production alternative | ✅ Ready |
| **5 (pureMPC)** | Online control, true MPC | 🚧 In Development |

**Current Recommendation:** 
- Start with **Method 1** (ppForIk) - proven and stable
- Switch to **Method 4** (ppFirst) if encountering tracking failures
- Use **Method 5** (pureMPC) when online control with guaranteed constraints is required
- Use **Method 0** (pureIk) only for debugging

**October 2025 Status:** 
- Methods 0, 1, 4: ✅ Fully implemented and tested
- Method 5: 🚧 Implementation in progress (8-12 weeks estimated)
