# Relationship: executeStageCPPFirst vs runStageCPPFirst_enhanced

## Function Hierarchy

```
User Script (e.g., run_fresh_sim_with_animation.m)
    ↓
trackReferenceTrajectory()
    ↓
runStagedTrajectory()
    ├─ Stage A: executeStageAFixedBase()
    ├─ Stage B: executeStageBPureHyb() or executeStageBGikInLoop()
    └─ Stage C: [ExecutionMode switch]
        ├─ "pureIk"   → executeStageCPureIk()
        ├─ "ppForIk"  → executeStageCPurePursuit()
        └─ "ppFirst"  → executeStageCPPFirst()  ← THIS ONE!
                            ↓
                        runStageCPPFirst_enhanced()  ← OUR ENHANCED IMPLEMENTATION
```

## Detailed Relationship

### executeStageCPPFirst()
**Location:** `matlab/+gik9dof/runStagedTrajectory.m` (lines 861-978)

**Role:** **Adapter/Wrapper Function**
- Internal helper function within `runStagedTrajectory.m`
- **Purpose:** Adapts staged pipeline parameters to Method 4 interface
- Extracts parameters from `options` and `PipelineConfig`
- Converts to name-value pairs for `runStageCPPFirst_enhanced`
- Transforms output log to staged pipeline format

**Key Responsibilities:**
1. **Parameter Translation:**
   ```matlab
   % Extract from options structure
   ppFirstOpts.ChassisParams = chassisParams;
   ppFirstOpts.MaxIterations = options.MaxIterations;
   ppFirstOpts.LookaheadDistance = options.StageCLookaheadDistance;
   
   % Convert to name-value pairs
   rawLog = gik9dof.runStageCPPFirst_enhanced(..., ...
       'ChassisParams', ppFirstOpts.ChassisParams, ...
       'LookaheadDistance', ppFirstOpts.LookaheadDistance, ...);
   ```

2. **Enable Phase 1 & 2A Features:**
   ```matlab
   rawLog = gik9dof.runStageCPPFirst_enhanced(..., ...
       'UseAdaptiveLookahead', true, ...      % Phase 1
       'UseMicroSegment', true, ...           % Phase 1
       'UseWarmStarting', true, ...           % Phase 1
       'UseVelocityCorridor', true, ...       % Phase 1
       'UseOrientationZNominal', true, ...    % Phase 2A ← CRITICAL!
       'OrientationWeight', 1.0, ...          % Phase 2A
       'PositionWeightXY', 0.1, ...);         % Phase 2A
   ```

3. **Log Format Conversion:**
   ```matlab
   % Transform raw log into staged pipeline format
   logC.qTraj = rawLog.qTraj;
   logC.timestamps = rawLog.timestamps;
   logC.positionError = rawLog.positionError;
   logC.targetPositions = rawLog.targetPositions;
   ```

**Think of it as:** A "translator" that sits between the staged pipeline and Method 4.

---

### runStageCPPFirst_enhanced()
**Location:** `matlab/+gik9dof/runStageCPPFirst_enhanced.m` (432 lines)

**Role:** **Core Implementation**
- Standalone function (can be called directly)
- **Purpose:** Implements Method 4 (PP-First) with all Phase 1 & 2A improvements
- Contains the actual control algorithm

**Key Responsibilities:**
1. **Base Seed Path Generation:**
   ```matlab
   baseSeedPath = gik9dof.baseSeedFromEE(robot, trajStruct, q_start, ...
       'UseOrientationZNominal', true, ...);  % Phase 2A!
   ```

2. **Pure Pursuit Initialization:**
   ```matlab
   [ppFollower, ppPathInfo] = gik9dof.initPPFromBasePath(baseSeedPath, ...);
   ```

3. **Main Control Loop:**
   ```matlab
   for i = 1:nWaypoints
       % PREDICT: Pure Pursuit generates base prediction
       [vx_pp, wz_pp, status] = ppFollower.step(currentPose, dt);
       
       % CONSTRAIN: Set yaw corridor around PP prediction
       yawBounds = [predictedYaw - yawTol, predictedYaw + yawTol];
       
       % SOLVE: GIK with constraints
       [qSolution, solInfo] = gik.solve(targetEE, qPrev, ...
           'YawBounds', yawBounds);
       
       % FALLBACK: If EE error too large, fix base and solve arm-only
       if eeError > threshold
           qSolution = armOnlyIK(qSolution, targetEE);
       end
   end
   ```

4. **Phase 1 Improvements:**
   - Adaptive lookahead
   - Micro-segment PP extension
   - Warm-starting GIK
   - Velocity-limited corridor
   - Lateral velocity diagnostic

5. **Phase 2A Improvement:**
   - Orientation+Z priority nominal pose via weighted IK
   - Tight orientation (weight 1.0) + Z (weight 1.0)
   - Relaxed XY position (weight 0.1)

**Think of it as:** The "engine" that does the actual work.

---

## Call Chain Example

### User calls staged pipeline with ppFirst mode:
```matlab
% run_fresh_sim_with_animation.m
log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'ExecutionMode', 'ppFirst', ...);  ← Selects Method 4
```

### Internal call flow:
```
1. trackReferenceTrajectory()
   ↓ calls with 'Mode'='staged'
   
2. runStagedTrajectory()
   ↓ Stage C: switch on ExecutionMode
   
3. case "ppFirst":
   logC = executeStageCPPFirst(...)  ← Wrapper
   ↓ prepares parameters
   ↓ enables Phase 1 & 2A flags
   
4. rawLog = runStageCPPFirst_enhanced(...)  ← Core implementation
   ↓ executes Method 4 algorithm
   ↓ returns results
   
5. executeStageCPPFirst() converts log format
   ↓ returns logC to runStagedTrajectory
   
6. runStagedTrajectory() merges Stage A, B, C logs
   ↓ returns pipeline to trackReferenceTrajectory
   
7. User receives final log
```

---

## The Critical Bug (Now Fixed!)

### Before the fix:
```matlab
% executeStageCPPFirst() in runStagedTrajectory.m
rawLog = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, qStart, ...
    'ChassisParams', ppFirstOpts.ChassisParams, ...
    'MaxIterations', ppFirstOpts.MaxIterations, ...
    'ApplyRefinement', ppFirstOpts.ApplyRefinement);
    % ❌ MISSING: All Phase 1 & 2A parameters!
    % Result: 974mm error (800x worse than expected!)
```

### After the fix:
```matlab
% executeStageCPPFirst() in runStagedTrajectory.m
rawLog = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, qStart, ...
    'ChassisParams', ppFirstOpts.ChassisParams, ...
    'MaxIterations', ppFirstOpts.MaxIterations, ...
    'ApplyRefinement', ppFirstOpts.ApplyRefinement, ...
    'UseAdaptiveLookahead', true, ...      % ✅ Phase 1
    'UseMicroSegment', true, ...           % ✅ Phase 1
    'UseWarmStarting', true, ...           % ✅ Phase 1
    'UseVelocityCorridor', true, ...       % ✅ Phase 1
    'LogLateralVelocity', true, ...        % ✅ Phase 1
    'RelaxedTolerances', true, ...         % ✅ Phase 1
    'LookaheadMin', 0.15, ...              % ✅ Phase 1
    'EpsLatMax', 0.015, ...                % ✅ Phase 1
    'EpsLongMin', 0.05, ...                % ✅ Phase 1
    'UseOrientationZNominal', true, ...    % ✅ Phase 2A - CRITICAL!
    'OrientationWeight', 1.0, ...          % ✅ Phase 2A
    'PositionWeightXY', 0.1, ...           % ✅ Phase 2A
    'PositionWeightZ', 1.0);               % ✅ Phase 2A
    % Expected result: ~1.2mm error (99.8% improvement)
```

---

## Why This Architecture?

### Design Rationale

1. **Separation of Concerns:**
   - `runStageCPPFirst_enhanced`: Pure Method 4 implementation (reusable)
   - `executeStageCPPFirst`: Staged pipeline integration (adapter)

2. **Flexibility:**
   - Can call `runStageCPPFirst_enhanced` directly for testing
   - Can call through staged pipeline for full A→B→C workflow

3. **Reusability:**
   - `runStageCPPFirst_enhanced` works standalone
   - No dependencies on staged pipeline structure

4. **Parameter Management:**
   - Staged pipeline has its own parameter naming (e.g., `StageCLookaheadDistance`)
   - Method 4 has its own naming (e.g., `LookaheadDistance`)
   - Wrapper translates between them

### Example Usage Patterns

#### Pattern 1: Direct Call (for testing)
```matlab
% test_method4_phase2a.m
log = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'UseOrientationZNominal', true, ...
    'OrientationWeight', 1.0, ...);
% Direct control over all parameters
```

#### Pattern 2: Through Staged Pipeline (production)
```matlab
% run_fresh_sim_with_animation.m
log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'ExecutionMode', 'ppFirst', ...);
% Parameters handled by pipeline → wrapper → implementation
```

---

## Analogy

Think of it like a car:

**runStageCPPFirst_enhanced** = The **Engine**
- Core functionality
- Can be tested on a test bench (direct calls)
- Self-contained, does one job well

**executeStageCPPFirst** = The **Transmission**
- Adapts engine power to wheels
- Translates rotational speed to linear motion
- Connects engine to the rest of the car

**runStagedTrajectory** = The **Car Body**
- Full system integration
- Provides driver interface
- Coordinates all subsystems (A, B, C)

You wouldn't drive just an engine, but the engine can be tested separately!

---

## Summary

| Aspect | executeStageCPPFirst | runStageCPPFirst_enhanced |
|--------|---------------------|--------------------------|
| **Type** | Adapter/Wrapper | Core Implementation |
| **Location** | Inside runStagedTrajectory.m | Standalone file |
| **Visibility** | Internal (nested function) | Public (package function) |
| **Purpose** | Integrate with pipeline | Implement Method 4 |
| **Parameters** | Extracts from options | Explicit name-value |
| **Can call directly?** | No (internal only) | Yes (for testing) |
| **Dependencies** | Requires runStagedTrajectory context | Self-contained |
| **Log format** | Staged pipeline format | Method 4 format |

**The fix we made:** Added Phase 1 & 2A parameters to the wrapper so they get passed to the implementation!

**Result:** Performance should now match isolated test (1.2mm error instead of 974mm).
