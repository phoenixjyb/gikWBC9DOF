# Method 4 Integrated Improvement Plan
## Combining Guide Best Practices + Our Innovations

**Created:** 2025-10-13  
**Status:** Ready for Implementation  
**Goal:** Reduce Method 4 EE error from 319mm → <150mm using field-tested techniques

---

## Executive Summary

After analyzing the `method4_Guide.md`, we've identified **synergies** between the guide's proven approaches and our analysis-driven recommendations. This integrated plan combines:

- ✅ **Guide's validated techniques**: Adaptive lookahead, micro-segment PP, v_lat diagnostics, velocity-limited corridor
- ✅ **Our innovations**: Warm-starting, relaxed solver tolerances, arm-aware predictions
- ✅ **Shared validation**: Both approaches use velocity-based corridor formula

**Expected trajectory:** 319mm → ~270mm (Phase 1) → ~145mm (Phase 2) → <120mm (Phase 3)

---

## Phase 1: Quick Wins (8-10 hours)
### Implement Guide's Proven Techniques + Critical Fixes

### 1.1 Adaptive Lookahead for Pure Pursuit (2 hours)
**Priority:** CRITICAL | **Impact:** +15-20% convergence, -20mm error

**Current Issue:** Fixed lookahead causes PP instability near waypoints

**Guide's Solution:**
```matlab
Ld_eff = min(Ld_nom, max(d, Ld_min))
```

**Implementation:**
```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: Before Pure Pursuit call (~line 200)

% Add after computing distance to next waypoint
d = norm([baseCandidateWaypoints(iWp, 1:2) - currentBasePose(1:2)]);

% Adaptive lookahead
Ld_nom = 0.8;  % nominal lookahead (default)
Ld_min = 0.15; % minimum near goals
Ld_eff = min(Ld_nom, max(d, Ld_min));

% Apply to Pure Pursuit controller
purePursuitObj.LookaheadDistance = Ld_eff;
```

**Testing:**
- Run on comparison trajectory, measure convergence rate
- Target: 47% → 62%+ convergence

---

### 1.2 Micro-Segment for Single-Waypoint PP (2 hours)
**Priority:** HIGH | **Impact:** Smoother commands, -10mm error

**Current Issue:** PP designed for paths, unstable for single targets

**Guide's Solution:** Create artificial path extension
```matlab
% current -> goal -> goal+extension
xgp = xg + max(Ld_min, 0.5*Ld_eff)*cos(theta_goal);
ygp = yg + max(Ld_min, 0.5*Ld_eff)*sin(theta_goal);
pp.Waypoints = [x y; xg yg; xgp ygp];
```

**Implementation:**
```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: Before PP call in waypoint loop

% Get next waypoint goal
xg = baseCandidateWaypoints(iWp, 1);
yg = baseCandidateWaypoints(iWp, 2);
theta_goal = baseCandidateWaypoints(iWp, 3);

% Create micro-segment extension
L_ext = max(Ld_min, 0.5*Ld_eff);
xgp = xg + L_ext * cos(theta_goal);
ygp = yg + L_ext * sin(theta_goal);

% Build 3-point waypoint list
purePursuitWaypoints = [
    currentBasePose(1:2);  % current position
    xg, yg;                 % goal
    xgp, ygp                % extension
];

purePursuitObj.Waypoints = purePursuitWaypoints;
```

---

### 1.3 Warm-Starting (3 hours)
**Priority:** CRITICAL | **Impact:** +20% convergence, -30mm error

**Our Innovation (not in guide, but essential):**

```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: Top of function (add persistent variable)

persistent prevSolution;

% Location: Before GIK solve in waypoint loop (~line 200)
if ~isempty(prevSolution) && iWp > 1
    % Use previous solution as initial guess
    try
        gik.setInitialGuess(prevSolution);
    catch
        % Fallback if setInitialGuess not available
        q0 = prevSolution;  % Use as seed instead
    end
end

[qSolution, solutionInfo] = gik(q0, eePoseConstraint, ...);

% Store solution for next iteration
prevSolution = qSolution;
```

**Reset strategy:**
- Clear `prevSolution` at segment boundaries
- Keep within segments for temporal consistency

---

### 1.4 Velocity-Limited Corridor (1.5 hours)
**Priority:** CRITICAL | **Impact:** 100% kinematically feasible

**Guide's Formula (validates our approach):**
```matlab
eps_long = max(eps_long_min, abs(v_cmd)*dt + 0.01)
```

**Implementation:**
```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: Before setting GIK bounds

% Velocity-based corridor sizing
dt = 0.1;  % control period
v_cmd = norm([v_x, v_y]);  % from PP output
eps_long_min = 0.01;  % 1cm minimum
eps_lat = 0.010;       % 10mm lateral (tight!)

% Dynamic longitudinal corridor
eps_long = max(eps_long_min, abs(v_cmd)*dt + 0.01);

% Apply to base Cartesian bounds
baseBoxConstraint.Bounds = [
    -eps_long, +eps_long;   % x (body-aligned forward/back)
    -eps_lat,  +eps_lat;    % y (body-aligned lateral)
    -Inf,      +Inf         % z (free)
];
```

---

### 1.5 Lateral Velocity Diagnostic (1 hour)
**Priority:** HIGH | **Impact:** Direct nonholonomy measurement

**Guide's Critical Metric:**
```matlab
function [v,w,v_lat] = vw_from_gik_step(x0,y0,th0, x1,y1,th1, dt)
    dth=wrapToPi(th1-th0); dx=x1-x0; dy=y1-y0; 
    thm=wrapToPi(th0+0.5*dth);
    v=(cos(thm)*dx + sin(thm)*dy)/dt;
    v_lat=(-sin(thm)*dx + cos(thm)*dy)/dt;  % KEY METRIC
    w=dth/dt;
end
```

**Target:** `|v_lat| < 0.02 m/s` for nonholonomic compliance

**Implementation:**
```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: After GIK solve, in logging section

% Compute lateral velocity residual
dx = qSolution(baseIdx.x) - q0(baseIdx.x);
dy = qSolution(baseIdx.y) - q0(baseIdx.y);
dth = wrapToPi(qSolution(baseIdx.yaw) - q0(baseIdx.yaw));
thm = wrapToPi(q0(baseIdx.yaw) + 0.5*dth);

v_lat = (-sin(thm)*dx + cos(thm)*dy) / dt;

% Log to diagnostics
solutionInfo.lateralVelocity = v_lat;

% Warning if excessive
if abs(v_lat) > 0.02
    warning('High lateral velocity: %.3f m/s (target < 0.02)', abs(v_lat));
end
```

---

### 1.6 Relax Solver Tolerances (0.5 hours)
**Priority:** HIGH | **Impact:** +10% convergence, -20mm error

**Our Innovation:**
```matlab
% File: matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Location: GIK solver configuration (~line 150)

gik.SolverParameters.ConstraintTolerance = 1e-4;  % was 1e-6
gik.SolverParameters.StepTolerance = 1e-10;       % was 1e-12
gik.SolverParameters.OptimalityTolerance = 1e-4;  % was 1e-6
gik.SolverParameters.MaxIterations = 2000;        % was 1500
```

---

## Phase 1 Testing & Validation (2 hours)

**Test Script:**
```matlab
% File: test_method4_phase1_improvements.m

% Test on comparison trajectory
trajectory_file = '1_pull_world_scaled.json';
config.yaw_corridor = 15;     % degrees (keep tight!)
config.position_tolerance = 0.15;  % meters
config.ee_error_threshold = 0.015; % meters

% Enable new features
config.use_adaptive_lookahead = true;
config.use_micro_segment = true;
config.use_warm_starting = true;
config.use_velocity_corridor = true;
config.log_v_lat = true;

% Run Method 4 ppFirst
results = gik9dof.trackReferenceTrajectory(trajectory_file, 'ppFirst', config);

% Expected outcomes
fprintf('=== Phase 1 Results ===\n');
fprintf('Mean EE Error: %.1f mm (target: ~270mm, baseline: 319mm)\n', ...
    mean(results.eeError)*1000);
fprintf('Convergence Rate: %.1f%% (target: >77%%, baseline: 47.1%%)\n', ...
    results.convergenceRate*100);
fprintf('Fallback Rate: %.1f%% (target: <30%%, baseline: 44.3%%)\n', ...
    results.fallbackRate*100);
fprintf('Mean |v_lat|: %.4f m/s (target: <0.02)\n', ...
    mean(abs(results.diagnostics.v_lat)));
```

**Success Criteria:**
- Mean error: 250-280mm (vs 319mm baseline)
- Convergence: >77% (vs 47% baseline)
- Fallback: <30% (vs 44% baseline)
- |v_lat|: <0.02 m/s (nonholonomy respected)
- All solutions kinematically feasible ✅

---

## Phase 2: Arm-Aware Predictions (12-14 hours)
### Guide's "Orientation+Z" vs Our "Full Arm-Aware PP"

### Option 2A: Guide's Nominal Posture Approach (Simpler, 6 hours)

**Guide's Strategy:** Compute nominal posture matching EE orientation+height, relax x-y

**Advantages:**
- ✅ Simpler implementation
- ✅ Proven in field
- ✅ Branch-consistent IK

**Implementation:**
```matlab
% File: matlab/+gik9dof/+helpers/computeNominalPoseOrientationZ.m (NEW)

function [q_nom, T_b2ee_nom] = computeNominalPoseOrientationZ(robot, eeName, q_seed, R_nom, z_nom, baseIdx)
    % Match orientation and height, relax x-y position
    
    gik_nom = generalizedInverseKinematics( ...
        "RigidBodyTree", robot, ...
        "ConstraintInputs", {'orientation','cartesian','joint'});
    
    % Orientation constraint (tight)
    ori = constraintOrientationTarget(eeName);
    ori.ReferenceBody = '';
    ori.TargetOrientation = rotm2quat(R_nom);
    ori.OrientationTolerance = deg2rad(0.5);
    
    % Cartesian constraint (tight z, free x-y)
    cart = constraintCartesianBounds(eeName);
    cart.ReferenceBody = '';
    cart.TargetTransform = eye(4);
    BIG = 10;
    cart.Bounds = [ -BIG, +BIG;  -BIG, +BIG;  z_nom, z_nom ];
    
    % Base locked at origin for nominal
    jb = constraintJointBounds(robot);
    B = jb.Bounds;
    B(baseIdx.ix,:) = [0 0]; 
    B(baseIdx.iy,:) = [0 0]; 
    B(baseIdx.ith,:) = [0 0];
    jb.Bounds = B;
    
    [q_nom,~] = gik_nom(q_seed, ori, cart, jb);
    T_b2ee_nom = getTransform(robot, q_nom, eeName, robot.BaseName);
end
```

**Expected Impact:**
- Better nominal → improved base path → reduced PP errors
- -50mm error reduction (319 → ~220mm)

---

### Option 2B: Our Arm-Aware PP (More sophisticated, 14 hours)

**Our Innovation:** PP samples multiple base poses, evaluates arm reachability

```matlab
% File: matlab/+gik9dof/+helpers/armAwarePurePursuit.m (NEW)

function [v_cmd, w_cmd, diagnostics] = armAwarePurePursuit(robot, eeName, ...
    currentPose, eeTargetPose, purePursuitObj, params)
    
    % Standard PP prediction
    [v_pp, w_pp] = purePursuitObj(currentPose);
    
    % Sample candidate base poses around PP prediction
    dt = params.dt;
    x0 = currentPose(1); y0 = currentPose(2); th0 = currentPose(3);
    
    candidates = [];
    for dv = [-0.1, 0, 0.1] * v_pp  % velocity variations
        for dw = [-0.2, 0, 0.2] * w_pp  % angular velocity variations
            v_cand = v_pp + dv;
            w_cand = w_pp + dw;
            
            % Integrate unicycle
            x1 = x0 + v_cand*cos(th0)*dt;
            y1 = y0 + v_cand*sin(th0)*dt;
            th1 = wrapToPi(th0 + w_cand*dt);
            
            % Evaluate arm reachability at this base pose
            q_base_cand = [x1; y1; th1];
            [reachable, manipulability] = evaluateArmReachability(...
                robot, eeName, q_base_cand, eeTargetPose);
            
            if reachable
                candidates = [candidates; v_cand, w_cand, manipulability];
            end
        end
    end
    
    % Select candidate with best manipulability
    if ~isempty(candidates)
        [~, best_idx] = max(candidates(:,3));
        v_cmd = candidates(best_idx, 1);
        w_cmd = candidates(best_idx, 2);
        diagnostics.manipulability = candidates(best_idx, 3);
        diagnostics.numCandidates = size(candidates, 1);
    else
        % Fallback to standard PP
        v_cmd = v_pp;
        w_cmd = w_pp;
        diagnostics.manipulability = 0;
        diagnostics.numCandidates = 0;
    end
end
```

**Expected Impact:**
- PP predictions optimized for arm reachability
- -100 to -150mm error reduction (270 → ~145mm)
- <15% fallback rate (vs 44% baseline)

---

### Recommendation: Try Guide's Approach First

**Rationale:**
1. Simpler implementation (6 vs 14 hours)
2. Proven in production
3. Can always upgrade to 2B if insufficient
4. Lower risk, faster results

**Decision point:** If Option 2A achieves <180mm error, skip 2B. Otherwise, implement 2B.

---

## Phase 3: Advanced Features (Optional, 8-12 hours)

### 3.1 Progressive Collision Handling (Guide's Approach)
```matlab
% Reactive policy chain:
% 1) Halve lateral corridor width
% 2) Reduce speed / increase lookahead
% 3) Shift corridor +ε along body-x
% 4) Relax EE orientation before position
% 5) Local RS/TEB micro-replan
```

### 3.2 Reverse-Aware PP (Guide's Approach)
```matlab
% "Carrot-behind" trick for backward motion
if cos(th)*dx + sin(th)*dy < 0  % goal behind
    theta_eff = wrapToPi(th + pi);
    sgn = -1;
end
[v_tmp, w_cmd] = pp([x y theta_eff]);
v_cmd = sgn * abs(v_tmp);
```

### 3.3 Windowed Horizon Conversion
```matlab
% Convert only next H waypoints instead of entire trajectory
H = 10;  % horizon length
idxWin = (iWp:min(iWp+H-1, numWaypoints));
basePath_window = eeToBaseWindow(eeTrajectory, q_nom, robot, eeName, idxWin);
```

---

## Implementation Timeline

| Phase | Task | Hours | Cumulative | Expected Error |
|---|---|---:|---:|---:|
| **1.1** | Adaptive lookahead | 2 | 2 | 300mm |
| **1.2** | Micro-segment PP | 2 | 4 | 290mm |
| **1.3** | Warm-starting | 3 | 7 | 260mm |
| **1.4** | Velocity corridor | 1.5 | 8.5 | 260mm |
| **1.5** | v_lat diagnostic | 1 | 9.5 | 260mm |
| **1.6** | Relax tolerances | 0.5 | 10 | 255mm |
| **Phase 1 Test** | Validation | 2 | 12 | **~270mm** ✅ |
| **2A** | Orientation+Z nominal | 6 | 18 | **~220mm** ✅ |
| **2A Test** | Validation | 2 | 20 | Decision point |
| **2B** | Arm-aware PP (if needed) | 14 | 34 | **~145mm** ✅✅ |
| **3.x** | Advanced (optional) | 8-12 | 42-46 | **<120mm** 🎯 |

---

## Success Metrics

### Phase 1 Targets (End of Week 1)
- ✅ Mean EE Error: ~270mm (vs 319mm baseline, -15%)
- ✅ Convergence: ~77% (vs 47% baseline, +64%)
- ✅ Fallback: <30% (vs 44% baseline, -32%)
- ✅ |v_lat|: <0.02 m/s (nonholonomy respected)
- ✅ All solutions kinematically feasible

### Phase 2A Targets (End of Week 2)
- ✅ Mean EE Error: ~220mm (vs 270mm Phase 1, -18%)
- ✅ Convergence: ~85% (vs 77% Phase 1, +10%)
- ✅ Fallback: <20% (vs 30% Phase 1, -33%)

### Phase 2B Targets (End of Week 3, if needed)
- ✅✅ Mean EE Error: ~145mm (vs 220mm Phase 2A, -34%)
- ✅✅ Convergence: >90% (vs 85% Phase 2A, +6%)
- ✅✅ Fallback: <15% (vs 20% Phase 2A, -25%)
- 🎯 **Competitive with Method 1's 129mm!**

---

## Key Insights from Guide Integration

1. **Validated Approach**: Guide confirms our velocity-limited corridor formula is correct
2. **Simpler Alternatives**: Orientation+Z nominal may suffice before full arm-aware PP
3. **Critical Diagnostics**: v_lat measurement directly monitors nonholonomy
4. **Proven Stability**: Adaptive lookahead + micro-segment solve single-waypoint PP issues
5. **Progressive Refinement**: Start simple (Phase 1), add complexity only if needed

---

## Next Steps

1. **Immediate**: Implement Phase 1 improvements (1.1 → 1.6)
2. **Test**: Run `test_method4_phase1_improvements.m` on comparison trajectory
3. **Validate**: Confirm ~270mm error, >77% convergence, |v_lat| < 0.02
4. **Decision**: If successful, proceed to Phase 2A (simpler Orientation+Z approach)
5. **Iterate**: Only implement Phase 2B if Phase 2A insufficient

**Ready to start?** Begin with Phase 1.1 (Adaptive Lookahead) - highest ROI, lowest risk!

---

**End of Integrated Plan**
