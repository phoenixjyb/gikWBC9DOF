# Phase 2A Implementation Plan: Orientation+Z Nominal Posture

**Date:** October 14, 2025  
**Goal:** Reduce error from 507mm to ~400mm (-20%)  
**Estimated Time:** 6 hours  
**Current Status:** Starting implementation

---

## Overview

### Current Problem:
The `baseSeedFromEE()` function generates base path using only **EE position (x, y, z)**:
- Ignores EE orientation (roll, pitch, yaw)
- Ignores EE height constraints
- Results in poor initial path → PP predictions have ~500mm error

### Phase 2A Solution:
Generate nominal base poses that **match EE orientation AND height** while relaxing x-y position:

```matlab
% CURRENT (Phase 1): Position-only nominal
nominal = inverseKinematics(robot, EE_target);
% Problem: IK minimizes ||q - q_home||, ignores that we care about orientation

% PHASE 2A: Orientation+Z priority nominal
nominal = inverseKinematics(robot, EE_target, ...
    'OrientationWeight', 1.0,      % Match orientation exactly
    'PositionWeight', [0.1, 0.1, 1.0]);  % Relax x-y, match z
% Benefit: Base positioned to achieve orientation+height, x-y can adjust
```

**Key Insight:** For manipulation tasks:
- **Orientation matters more than position** (tool alignment)
- **Height matters more than x-y** (avoid collisions, reachability)
- **Base x-y can adjust** (mobile platform freedom)

---

## Implementation Steps

### Step 1: Create Enhanced Nominal Pose Generator (2 hours)

**File:** `matlab/+gik9dof/computeNominalPoseOrientationZ.m` (NEW)

**Function Signature:**
```matlab
function [nominalConfig, basePose] = computeNominalPoseOrientationZ(robot, T_ee_target, q_current, options)
%COMPUTENOMINALPOSEORIENTATIONZ Generate nominal pose matching EE orientation+height.
%   [nominalConfig, basePose] = computeNominalPoseOrientationZ(robot, T_ee_target, q_current)
%   computes a nominal configuration that prioritizes matching the target end-effector
%   orientation and Z-height, while allowing the base X-Y position to vary.
%
%   This is a key Phase 2A improvement: instead of treating all EE position dimensions
%   equally, we recognize that orientation and height are more critical for manipulation
%   tasks, and the mobile base can adjust X-Y position to achieve better arm configurations.
%
%   Inputs:
%       robot        - rigidBodyTree (9-DOF mobile manipulator)
%       T_ee_target  - 4x4 SE(3) target pose for end-effector
%       q_current    - 9x1 current configuration (optional, default: home)
%       options      - struct with optional fields:
%           .EndEffector      - string, EE name (default: "left_gripper_link")
%           .BaseIndices      - [1,2,3] base joint indices
%           .ArmIndices       - [4:9] arm joint indices
%           .OrientationWeight - scalar, weight for orientation (default: 1.0)
%           .PositionWeightXY  - scalar, weight for x-y position (default: 0.1)
%           .PositionWeightZ   - scalar, weight for z position (default: 1.0)
%           .MaxIterations    - scalar, IK iterations (default: 500)
%
%   Outputs:
%       nominalConfig - 9x1 nominal configuration (base + arm)
%       basePose      - [x, y, theta] extracted base pose
%
%   Example:
%       T_target = trajStruct.Poses(:,:,k);
%       [q_nom, base_nom] = computeNominalPoseOrientationZ(robot, T_target, q_current);
%
%   See also: baseSeedFromEE, inverseKinematics, generalizedInverseKinematics

arguments
    robot (1,1) rigidBodyTree
    T_ee_target (4,4) double
    q_current (:,1) double = homeConfiguration(robot)
    options.EndEffector (1,1) string = "left_gripper_link"
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
    options.OrientationWeight (1,1) double = 1.0
    options.PositionWeightXY (1,1) double = 0.1
    options.PositionWeightZ (1,1) double = 1.0
    options.MaxIterations (1,1) double = 500
end

% Extract target EE position and orientation
p_target = T_ee_target(1:3, 4);
R_target = T_ee_target(1:3, 1:3);

% Create IK solver with weighted constraints
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [options.PositionWeightXY, options.PositionWeightXY, ...
             options.PositionWeightZ, options.OrientationWeight, ...
             options.OrientationWeight, options.OrientationWeight];

% Solve IK with orientation+Z priority
[nominalConfig, solInfo] = ik(char(options.EndEffector), T_ee_target, ...
    ikWeights, q_current, 'MaxIterations', options.MaxIterations);

% Extract base pose
basePose = nominalConfig(options.BaseIndices);

% Validate solution quality
T_ee_achieved = getTransform(robot, nominalConfig, char(options.EndEffector));
p_achieved = T_ee_achieved(1:3, 4);
R_achieved = T_ee_achieved(1:3, 1:3);

% Position error breakdown
pos_error = p_achieved - p_target;
xy_error = norm(pos_error(1:2));
z_error = abs(pos_error(3));

% Orientation error (Frobenius norm of rotation difference)
R_error = R_target' * R_achieved - eye(3);
orient_error = norm(R_error, 'fro');

% Store diagnostics
nominalConfig = nominalConfig(:);  % Ensure column vector
solInfo.xy_error = xy_error;
solInfo.z_error = z_error;
solInfo.orient_error = orient_error;

end
```

**Key Design Choices:**
1. **Weighted IK** - Not GIK! Simple IK with position/orientation weights
2. **Orientation weight = 1.0** - Match orientation exactly
3. **Z weight = 1.0** - Match height exactly
4. **XY weight = 0.1** - Allow 10x more deviation in x-y plane
5. **Start from q_current** - Use current config as initial guess

---

### Step 2: Integrate into baseSeedFromEE (1 hour)

**File:** `matlab/+gik9dof/baseSeedFromEE.m` (MODIFY)

**Current Implementation:**
```matlab
% For each waypoint
for k = 1:nWaypoints
    T_ee = trajStruct.Poses(:,:,k);
    
    % Compute nominal config (uses standard IK, equal weights)
    [qNominal, solInfo] = bundle.solve(q, 'TargetPose', T_ee);
    
    % Extract base pose
    basePath(k,:) = qNominal(baseIdx)';
end
```

**Enhanced Implementation:**
```matlab
% For each waypoint
for k = 1:nWaypoints
    T_ee = trajStruct.Poses(:,:,k);
    
    % PHASE 2A: Use orientation+Z priority nominal
    if options.UseOrientationZNominal
        [qNominal, basePose] = gik9dof.computeNominalPoseOrientationZ(...
            robot, T_ee, q, ...
            'EndEffector', options.EndEffector, ...
            'BaseIndices', baseIdx, ...
            'ArmIndices', armIdx);
        basePath(k,:) = basePose';
    else
        % Standard approach (Phase 1)
        [qNominal, solInfo] = bundle.solve(q, 'TargetPose', T_ee);
        basePath(k,:) = qNominal(baseIdx)';
    end
    
    q = qNominal;  % Update for next iteration
end
```

**Changes:**
1. Add `options.UseOrientationZNominal` flag (default: false for backward compat)
2. Call new `computeNominalPoseOrientationZ()` when enabled
3. Use weighted IK instead of GIK for nominal generation

---

### Step 3: Update runStageCPPFirst_enhanced (30 min)

**File:** `matlab/+gik9dof/runStageCPPFirst_enhanced.m`

**Add new option:**
```matlab
arguments
    % ... existing arguments ...
    
    % NEW: Phase 2A feature
    options.UseOrientationZNominal (1,1) logical = true  % Enable by default!
end
```

**Pass to baseSeedFromEE:**
```matlab
baseSeedPath = gik9dof.baseSeedFromEE(robot, trajStruct, q_start, ...
    'ChassisParams', options.ChassisParams, ...
    'EndEffector', options.EndEffector, ...
    'BaseIndices', options.BaseIndices, ...
    'ArmIndices', options.ArmIndices, ...
    'UseOrientationZNominal', options.UseOrientationZNominal, ...  % NEW!
    'ApplyRefinement', options.ApplyRefinement, ...
    'Verbose', detailedVerbose);
```

---

### Step 4: Update Test Script (30 min)

**File:** `test_method4_phase1_improvements.m` → rename to `test_method4_phase2a.m`

**Add Phase 2A test:**
```matlab
%% Run Enhanced with Phase 2A
fprintf('\n--- Running ENHANCED (Phase 2A: Orientation+Z Nominal) ---\n');

tic;
log_enhanced = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'UseAdaptiveLookahead', true, ...
    'UseMicroSegment', true, ...
    'UseWarmStarting', true, ...
    'UseVelocityCorridor', true, ...
    'UseOrientationZNominal', true, ...  % NEW: Phase 2A feature
    'RelaxedTolerances', true, ...
    'LogLateralVelocity', true, ...
    'VerboseLevel', 1);
enhanced_time = toc;
```

**Update success criteria:**
```matlab
% Phase 2A targets (revised from Phase 1)
target_error = 420;      % mm (507mm * 0.80 = ~400mm, -20% from Phase 1)
target_fallback = 12;    % % (14.8% * 0.80 = ~12%)
target_convergence = 75; % % (68.6% * 1.10 = ~75%)
target_vlat = 0.02;      % m/s (aspirational, may not achieve)
```

---

### Step 5: Testing and Validation (2 hours)

**Test 1: Smoke Test (5 waypoints)**
```matlab
% Quick test to verify function works
test_orientation_z_nominal_smoke.m
```

**Test 2: Full Comparison (210 waypoints)**
```matlab
% Baseline vs Phase 1 vs Phase 2A
test_method4_phase2a.m
```

**Test 3: Ablation Study**
```matlab
% Test with/without orientation+Z nominal
% Measure actual impact of Phase 2A
```

**Expected Results:**
- Mean Error: 507mm → ~400mm (-20%)
- Fallback: 14.8% → ~12% (-19%)
- Convergence: 68.6% → ~75% (+9%)
- Lateral vel: 0.25 m/s → ~0.20 m/s (-20%)

---

## Expected Benefits

### Quantitative:
- **Error reduction: -20%** (507mm → 400mm)
- **Fallback reduction: -19%** (14.8% → 12%)
- **Convergence improvement: +9%** (68.6% → 75%)
- **Better base path** → PP predictions closer to reachable poses

### Qualitative:
- **Better arm configurations** - Orientation-aware positioning
- **Fewer singularities** - Height awareness avoids unreachable poses
- **More natural motion** - Base positions respect manipulation goals
- **Foundation for Phase 2B** - Good nominal → easier arm-aware refinement

---

## Success Criteria

### Phase 2A Targets:
1. ✅ **Mean EE Error: ≤420mm** (current 507mm, target -20%)
2. ✅ **Fallback Rate: <12%** (current 14.8%, target -19%)
3. ✅ **Convergence: >75%** (current 68.6%, target +9%)
4. ⚠️ **Lateral Vel: <0.20 m/s** (current 0.25 m/s, aspirational)

**Minimum Success:** 2/4 criteria (error + fallback)  
**Good Success:** 3/4 criteria  
**Excellent Success:** 4/4 criteria

---

## Risk Assessment

### Low Risk:
- ✅ Guide validates this approach (method4_Guide.md lines 197-213)
- ✅ Weighted IK is standard MATLAB feature
- ✅ Backward compatible (flag-gated)
- ✅ Simple implementation (~200 lines)

### Potential Issues:
1. **IK convergence** - Weighted IK might fail sometimes
   - Mitigation: Fall back to standard IK if fails
2. **Base positions too far** - XY weight 0.1 might allow too much drift
   - Mitigation: Tune weight (try 0.2, 0.3 if needed)
3. **Height not achievable** - Some Z heights unreachable
   - Mitigation: Relax Z weight if necessary (try 0.7, 0.5)

### Debugging Plan:
1. Log IK success rate for nominal computation
2. Compare base path: standard vs orientation+Z
3. Visualize base positions (should be closer to EE)
4. Check orientation errors (should be <5°)

---

## Timeline

- **12:30 PM** - Start implementation
- **2:00 PM** - Complete computeNominalPoseOrientationZ.m
- **2:30 PM** - Integrate into baseSeedFromEE.m
- **3:00 PM** - Update runStageCPPFirst_enhanced.m
- **3:30 PM** - Create test scripts
- **4:00 PM** - Smoke test (5 waypoints)
- **4:30 PM** - Full test run (210 waypoints, 15 min)
- **5:00 PM** - Analyze results
- **5:30 PM** - Decision: iterate or proceed to Phase 2B

**Total Time:** ~5 hours (1 hour buffer)

---

## Next Steps After Phase 2A

### If Successful (≥3/4 criteria):
1. ✅ Document Phase 2A results
2. 🚀 Proceed to Phase 2B: Arm-aware Pure Pursuit (14 hours)
   - Expected: 400mm → 200mm (-50%)
3. 🎯 Target competitive with Method 1 (129mm)

### If Partial Success (2/4 criteria):
1. 🔧 Tune orientation/position weights
2. 🔄 Retest with adjustments (1-2 hours)
3. ✅ Document lessons learned

### If Failed (<2/4 criteria):
1. 🔍 Debug why nominal poses not helping
2. 🤔 Consider alternative approaches:
   - Manipulability-aware nominal
   - Inverse reachability maps
   - Sampling-based nominal generation

---

**Status:** Ready to implement! Starting with Step 1...
