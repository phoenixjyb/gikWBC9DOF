# Project Diagnosis: gikWBC9DOF - Comprehensive System Analysis

**Generated:** October 11, 2025  
**Focus:** Staged Mode Data Flow, File Relationships, and System Architecture  
**Project:** gikWBC9DOF = **G**eneralized **I**nverse **K**inematics **W**hole **B**ody **C**ontrol for **9 DOF** mobile manipulator

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Project Architecture Overview](#project-architecture-overview)
3. [Staged Mode: Complete Data Flow](#staged-mode-complete-data-flow)
4. [File Inventory & Functional Classification](#file-inventory--functional-classification)
5. [Dependency Graph & File Relationships](#dependency-graph--file-relationships)
6. [Preprocessing & Configuration System](#preprocessing--configuration-system)
7. [Real-Time Simulation Pipeline](#real-time-simulation-pipeline)
8. [Animation from Logs Pipeline](#animation-from-logs-pipeline)
9. [Helper Functions & Utilities](#helper-functions--utilities)
10. [Redundancies, Orphans & Deprecated Files](#redundancies-orphans--deprecated-files)
11. [Key Insights & Recommendations](#key-insights--recommendations)

---

## Executive Summary

This document provides a **comprehensive analysis** of the gikWBC9DOF project, with special focus on the **staged mode** execution pipeline. The project implements a whole-body control system for a mobile manipulator using generalized inverse kinematics (GIK).

### Key Findings

**✅ Strengths:**
- Well-structured package system (`+gik9dof/`)
- Clear separation of concerns (control, visualization, evaluation)
- Comprehensive logging and diagnostics infrastructure
- **Unified parameter configuration system** (`config/pipeline_profiles.yaml`)
- Flexible chassis profiles via YAML with inheritance
- Multiple execution modes (holistic/staged, ppForIk/pureIk)
- Track width standardized to 0.574 m across all files

**⚠️ Areas of Concern (Previously Identified, Now Resolved):**
- ~~Multiple overlapping scripts at root level with unclear usage~~
- ~~Legacy animation functions (`animateStagedLegacy.m`) still present~~
- ~~Some configuration scattered between code and YAML~~ ✅ **FIXED: Unified config**
- ~~Parameter inconsistencies across functions~~ ✅ **FIXED: Single source of truth**
- ~~Track width inconsistency (0.674 vs 0.574)~~ ✅ **FIXED: Standardized to 0.574 m**
- Documentation spread across multiple files with some overlap

**📊 Project Statistics:**
- **Total MATLAB files:** ~60 files
- **Core library functions:** 44 files in `+gik9dof/` package
- **Top-level scripts:** 12 orchestration/test scripts
- **Documentation files:** 15+ markdown documents
- **Configuration files:** 2 YAML files (chassis_profiles.yaml, pipeline_profiles.yaml)
- **Reference trajectories:** 1 JSON end-effector path

**Robot Configuration:**
- **9 DOF total**: 3 base (planar) + 6 arm (revolute)
- **Base joints**: joint_x, joint_y, joint_theta
- **Arm joints**: left_arm_joint1 through left_arm_joint6
- **End effector**: left_gripper_link

---

## Project Architecture Overview

```
gikWBC9DOF/
├── 🎯 Entry Points (Top-level Scripts)
│   ├── run_staged_reference.m                 # Main staged entry
│   ├── run_environment_compare.m              # Holistic vs staged comparison
│   ├── run_fresh_sim_with_animation.m         # Quick test with animation
│   ├── run_parametric_study.m                 # Parameter sweep studies
│   └── test_*.m                               # Various test scripts
│
├── 📦 Core Library (+gik9dof/ Package)
│   ├── trackReferenceTrajectory.m             # Mode router (holistic/staged)
│   ├── runStagedTrajectory.m                  # Staged orchestrator
│   ├── runTrajectoryControl.m                 # IK control loop
│   ├── createGikSolver.m                      # Solver bundle factory
│   ├── createRobotModel.m                     # Robot model factory
│   ├── environmentConfig.m                    # Environment setup
│   │
│   ├── +control/                              # Chassis control subsystem
│   │   ├── unifiedChassisCtrl.m               # Unified controller
│   │   ├── purePursuitFollower.m              # Path follower
│   │   ├── simulateChassisController.m        # Chassis simulation
│   │   ├── defaultUnifiedParams.m             # Default parameters
│   │   ├── loadChassisProfile.m               # YAML profile loader
│   │   ├── rsRefinePath.m                     # Reeds-Shepp smoothing
│   │   ├── rsClothoidRefine.m                 # Clothoid smoothing
│   │   └── defaultReedsSheppParams.m          # RS parameters
│   │
│   ├── +viz/                                  # Visualization subsystem
│   │   ├── animate_whole_body.m               # Main animator
│   │   └── animatePurePursuitSimulation.m     # Pure pursuit viz
│   │
│   ├── +internal/                             # Internal utilities
│   │   ├── createResultsFolder.m              # Timestamped folders
│   │   ├── resolvePath.m                      # Path resolution
│   │   ├── projectRoot.m                      # Project root finder
│   │   ├── VelocityEstimator.m                # Base velocity estimator
│   │   └── addChassisFootprint.m              # Footprint builder
│   │
│   ├── +debug/                                # Debug utilities
│   │   └── visualizeStageBOccupancy.m         # Occupancy grid viz
│   │
│   └── [Animation, Evaluation, Environment]   # See detailed sections
│
├── 📊 Configuration & Assets
│   ├── config/chassis_profiles.yaml           # Chassis tuning presets
│   ├── 1_pull_world_scaled.json               # Reference EE trajectory
│   └── meshes/                                # Collision geometry (STL)
│
├── 📁 Results Archive
│   └── results/                               # Timestamped run outputs
│       └── <YYYYMMDD_HHMMSS>_<label>/
│           ├── log_*.mat                      # Simulation logs
│           ├── *.mp4                          # Animations
│           ├── *.png                          # Diagnostic plots
│           └── *.csv                          # Metrics exports
│
└── 📚 Documentation
    ├── projectDiagnosis.md                    # This file
    ├── DATA_FLOW_ANALYSIS.m                   # Data flow explanation
    ├── PROJECT_OVERVIEW.md                    # High-level overview
    ├── PROJECT_STATUS_SUMMARY.md              # Status & achievements
    ├── HANDOVER.md                            # Handover notes
    ├── ALGORITHM_IMPROVEMENT_PLAN.md          # Improvement roadmap
    ├── SIMULATION_WORKFLOW_GUIDE.md           # Operational guide
    └── [Others...]                            # Various guides
```

---

## Staged Mode: Complete Data Flow

### Overview

The **staged mode** execution divides the motion into three sequential phases:

1. **Stage A** - Arm-only ramp-up (base locked, arm aligns to first waypoint)
2. **Stage B** - Base-only navigation (arm locked, base moves to docking pose)
3. **Stage C** - Full-body tracking (both base and arm track reference trajectory)

### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           INITIALIZATION                                 │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Entry Script: run_staged_reference.m                                    │
│  ├─ Loads: config/chassis_profiles.yaml                                  │
│  ├─ Calls: gik9dof.environmentConfig() → base home, obstacles, margins  │
│  └─ Calls: gik9dof.trackReferenceTrajectory('Mode', 'staged', ...)      │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  trackReferenceTrajectory.m (Mode Router)                                │
│  ├─ Loads: 1_pull_world_scaled.json → 148 EE waypoints                  │
│  ├─ Creates: robot model via createRobotModel()                          │
│  ├─ Sets: q0 (initial configuration with base at home pose)              │
│  ├─ Attaches: floor disc obstacles to robot tree                         │
│  ├─ Creates: distance constraints for collision avoidance                │
│  └─ Routes to: gik9dof.runStagedTrajectory(...)                          │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        STAGE A: ARM RAMP-UP                              │
│  runStagedTrajectory.m → Stage A execution                               │
│  ├─ Input: q0, first EE pose T1                                          │
│  ├─ Generates: 50 interpolated poses from current T0 → T1                │
│  ├─ Creates: bundleA (GIK solver with base locked)                       │
│  ├─ Calls: runTrajectoryControl(bundleA, trajA, ...)                     │
│  └─ Output: logA (qTraj, timestamps, diagnostics)                        │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                   STAGE B: BASE NAVIGATION                               │
│  runStagedTrajectory.m → Stage B execution                               │
│  ├─ Input: qA_end (end config from Stage A)                              │
│  ├─ Mode: "gikInLoop" OR "pureHyb"                                       │
│  │                                                                        │
│  ├─ IF pureHyb:                                                          │
│  │   ├─ Computes goal base pose from first EE waypoint                   │
│  │   ├─ Plans: Hybrid A* path (or linear interpolation)                  │
│  │   ├─ Smooths: Reeds-Shepp shortcuts (optional)                        │
│  │   ├─ Smooths: Clothoid refinement (optional)                          │
│  │   ├─ Simulates: Pure pursuit controller execution                     │
│  │   ├─ Generates: base trajectory (x, y, theta)                         │
│  │   └─ Output: logB (synthetic log, pathStates, cmdLog)                 │
│  │                                                                        │
│  └─ IF gikInLoop:                                                        │
│      ├─ Interpolates: straight-line base path                            │
│      ├─ Creates: bundleB (GIK solver with arm locked)                    │
│      ├─ Calls: runTrajectoryControl(bundleB, trajB, ...)                 │
│      └─ Output: logB (qTraj, timestamps, diagnostics)                    │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                  STAGE C: FULL-BODY TRACKING                             │
│  runStagedTrajectory.m → Stage C execution                               │
│  ├─ Input: qB_end (end config from Stage B), remaining waypoints         │
│  ├─ Mode: "ppForIk" OR "pureIk"                                          │
│  │                                                                        │
│  ├─ IF ppForIk:                                                          │
│  │   ├─ Creates: bundleRef (GIK solver for reference pass)               │
│  │   ├─ Runs: runTrajectoryControl() to get reference base path          │
│  │   ├─ (Optional) Smooths: RS + Clothoid on base path                   │
│  │   ├─ Simulates: Pure pursuit on reference path                        │
│  │   ├─ Creates: bundleFinal (GIK solver with fixed base trajectory)     │
│  │   ├─ Runs: runTrajectoryControl() with FixedJointTrajectory           │
│  │   └─ Output: logC (qTraj, purePursuit data, diagnostics)              │
│  │                                                                        │
│  └─ IF pureIk:                                                           │
│      ├─ Creates: bundleC (standard GIK solver)                           │
│      ├─ Runs: runTrajectoryControl(bundleC, trajC, ...)                  │
│      └─ Output: logC (qTraj, timestamps, diagnostics)                    │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      LOG MERGING & SAVING                                │
│  runStagedTrajectory.m → mergeStageLogs()                                │
│  ├─ Concatenates: qTraj from all stages                                  │
│  ├─ Concatenates: timestamps                                             │
│  ├─ Stores: stageLogs.stageA, stageLogs.stageB, stageLogs.stageC         │
│  ├─ Returns: pipeline struct                                             │
│  └─ Saved to: results/<timestamp>_<label>/log_staged_<mode>.mat          │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      ANIMATION GENERATION                                │
│  (Optional) Called by entry script or regeneration script                │
│  ├─ Loads: log_staged_*.mat                                              │
│  ├─ Calls: animateStagedWithHelper() OR viz.animate_whole_body()         │
│  ├─ Renders: Robot poses via forward kinematics from qTraj               │
│  ├─ Overlays: Reference paths, obstacles, stage labels                   │
│  └─ Saves: *.mp4 animation file                                          │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## Data Objects & Their Evolution

### 1. Configuration Object (q)

**Type:** Column vector `[9×1 double]`  
**Contents:** Joint angles for all DOFs

```matlab
% Joint ordering (9 DOF total):
q(1:3)   = [joint_x, joint_y, joint_theta]  % Mobile base (planar)
q(4:9)   = [left_arm_joint1 ... left_arm_joint6]  % 6-DOF arm
```

**Evolution through stages:**
- `q0` - Initial configuration (base at home pose)
- `qA_end` - After Stage A (arm aligned, base unchanged)
- `qB_end` - After Stage B (base moved, arm unchanged)
- `qTraj` - Full trajectory (9×N) through all stages

### 2. Trajectory Structure (trajStruct)

**Type:** Struct loaded from JSON

```matlab
trajStruct = struct(
    'Poses',                  % [4×4×N] SE(3) transforms
    'EndEffectorPositions',   % [3×N] xyz positions
    'EndEffectorName',        % 'left_gripper_link'
    'AimTargets'              % [3×N] optional aiming points
);
```

**Source:** `1_pull_world_scaled.json` (148 waypoints)

### 3. Log Structure (log)

**Type:** Comprehensive struct with nested data

```matlab
log = struct(
    % Core trajectory data
    'qTraj',                  % [9×N] joint angles over time
    'timestamps',             % [1×N] time stamps
    'successMask',            % [1×N] solver success per step
    
    % End-effector tracking
    'eePositions',            % [3×N] actual EE positions (FK)
    'targetPositions',        % [3×N] desired EE positions (JSON)
    'eePoses',                % [4×4×N] actual EE poses
    'positionError',          % [3×N] tracking error vectors
    'positionErrorNorm',      % [1×N] error magnitudes
    
    % Solver diagnostics
    'solutionInfo',           % {1×N} cell array of solver info
    'iterations',             % [1×N] iterations per step
    'solveTime',              % [1×N] solve time per step
    'solverSummary',          % Aggregated statistics
    
    % Velocity estimation (if enabled)
    'baseVelocityEstimate',   % Struct with velocity data
    
    % Mode-specific data
    'mode',                   % 'holistic' or 'staged'
    'simulationMode',         % 'ppForIk' or 'pureIk'
    
    % Pure pursuit data (if ppForIk)
    'purePursuit',            % Struct with controller data
    '  .referencePath',       % [N×3] reference base states
    '  .simulation',          % Controller simulation results
    '  .executedPath',        % [N×3] executed base states
    '  .commands',            % [N×2] (Vx, Wz) commands
    '  .wheelSpeeds',         % [N×2] left/right wheel speeds
    '  .status',              % Controller status per step
    
    % Stage-specific logs (staged mode only)
    'stageLogs',              % Struct with stageA, stageB, stageC
    '  .stageA',              % Stage A log
    '  .stageB',              % Stage B log (includes diagnostics)
    '  .stageC',              % Stage C log
    
    % Environment and configuration
    'floorDiscs',             % Obstacle information
    'distanceSpecs',          % Collision constraints
    'environment',            % Environment config
    'chassisParams',          % Chassis parameters
    'chassisProfile',         % Profile name (e.g., 'wide_track')
);
```

### 4. Stage B Diagnostics (Enhanced Logging)

```matlab
stageBDiagnostics = struct(
    % Path curvature metrics
    'baseCurvature',          % [N×1] curvature at each point
    'curvatureHistogram',     % Struct with bins (low/med/high/veryHigh)
    'maxCurvature',           % Maximum curvature
    'meanCurvature',          % Average curvature
    'pathSmoothness',         % Std dev of curvature
    
    % Cusp detection
    'cuspLocations',          % [M×1] indices of cusps (gear reversals)
    'cuspCount',              % Total number of cusps
    
    % Smoothing metrics
    'rsAcceptanceRate',       % Reeds-Shepp acceptance ratio
    'rsImprovements',         % Number of accepted improvements
    'rsIterations',           % Total RS iterations
    'rsPathLengthImprovement',% Path length reduction (m)
    'clothoidApplied',        % Boolean flag
    'clothoidSegments',       % Number of fitted segments
    
    % Performance
    'plannerComputeTime'      % Planning time (seconds)
);
```

### 5. Stage C Diagnostics (Enhanced Logging)

```matlab
stageCDiagnostics = struct(
    % Solver performance
    'solverIterationsPerWaypoint',  % [N×1] iterations per step
    'maxIterationsHit',             % Number of steps hitting cap
    
    % Tracking quality bins
    'eeErrorBins',            % Struct with counts
    '  .excellent',           % <0.05m (count)
    '  .good',                % 0.05-0.10m
    '  .acceptable',          % 0.10-0.20m
    '  .poor',                % >0.20m
    
    % Error statistics
    'eeErrorMean',            % Mean tracking error (m)
    'eeErrorMax',             % Max tracking error (m)
    'eeErrorRMS',             % RMS error
    
    % Base tracking deviation
    'baseYawDriftMean',       % Mean yaw deviation (rad)
    'baseYawDriftMax',        % Max yaw deviation (rad)
    'basePosDeviationMean',   % Mean position deviation (m)
    'basePosDeviationMax',    % Max position deviation (m)
    
    % Refinement info (if applied)
    'refinementApplied',      % Boolean flag
    'refinementDelta',        % Changes from refinement
    '  .pathLength',          % Path length delta (m)
    '  .eeError'              % EE error delta (m)
);
```

**Solver Telemetry** (captured in `runTrajectoryControl`):
- Per-step iteration counts, solve times, constraint violations
- Aggregate statistics: mean/max/min solve time, success rate
- Status tallies for convergence analysis
- Exit flags and solver status strings

**Performance Observations** (from iteration studies):
- Iteration cap 150: ~0.2s mean solve time, hits cap frequently
- Iteration cap 1500: ~0.69s mean solve time, better convergence
- Stage C EE errors typically <1mm RMS for both caps
- Trade-off: Speed vs. convergence quality

---

## File Inventory & Functional Classification

### Core Control & Planning (14 files)

| File | Purpose | Inputs | Outputs | Dependencies |
|------|---------|--------|---------|--------------|
| **trackReferenceTrajectory.m** | Mode router | JSON path, options | log | runStagedTrajectory, runTrajectoryControl |
| **runStagedReference.m** | Convenience wrapper | options | result struct | trackReferenceTrajectory + I/O |
| **runStagedTrajectory.m** | Staged orchestrator | robot, trajStruct, q0 | pipeline | Stage A/B/C functions |
| **runTrajectoryControl.m** | IK control loop | bundle, trajectory, q0 | log | GIK solver |
| **createGikSolver.m** | Solver factory | robot, constraints | bundle | generalizedInverseKinematics |
| **createRobotModel.m** | Robot model | URDF path | robot, footprint | rigidBodyTree |
| **configurationTools.m** | Config utilities | robot | tools | - |
| **environmentConfig.m** | Environment setup | - | config struct | - |
| **generateHolisticRamp.m** | Ramp generator | q0, T1 | rampInfo | IK solver |
| **runEnvironmentCompare.m** | Compare modes | - | - | trackReferenceTrajectory |
| **addFloorDiscs.m** | Add obstacles | robot, discs | discInfo | rigidBody |
| **collisionTools.m** | Collision setup | robot | tools | STL meshes |
| **evaluateLog.m** | Log analysis | log | metrics | - |
| **comprehensiveEvaluation.m** | Full evaluation | results | summary | Multiple |

### Chassis Control Subsystem (11 files)

The chassis control subsystem orchestrates mobile base motion through three execution modes: **holistic** (GIK-driven whole-body control), **staged-C** (Stage C tracking), and **staged-B** (pure path following). The architecture consists of four functional layers:

#### **Architecture Overview**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         EXECUTION LAYER                                  │
│  ┌──────────────────┐  ┌─────────────────┐  ┌──────────────────────┐  │
│  │ Holistic Mode    │  │ Staged-C Mode   │  │ Staged-B Mode        │  │
│  │ (GIK pipeline)   │  │ (Stage C track) │  │ (path following)     │  │
│  │ poses + arm qdot │  │ poses + arm qdot│  │ path → follower      │  │
│  └─────────┬────────┘  └────────┬────────┘  └──────────┬───────────┘  │
│            │                     │                       │               │
│            └─────────────────────┴───────────────────────┘               │
│                                  │                                       │
│                         ┌────────▼───────────┐                          │
│                         │ unifiedChassisCtrl │  ◄── Central Hub        │
│                         │  (mode routing)    │                          │
│                         └────────┬───────────┘                          │
│                                  │                                       │
│                         UnifiedCmd {base.Vx, Vy, Wz; arm.qdot}         │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
┌─────────────────────────────────────────────────────────────────────────┐
│                      PATH FOLLOWING LAYER                                │
│         (Used by staged-B mode, bypassed in holistic/staged-C)          │
│                                                                           │
│  Input Path (Nx3 [x,y,yaw])                                             │
│        │                                                                  │
│        ├──[optional]──► rsRefinePath ──► Reeds-Shepp shortcutting       │
│        │                                                                  │
│        ├──[optional]──► rsClothoidRefine ──► Clothoid smoothing         │
│        │                                                                  │
│        ▼                                                                  │
│  preparePathForFollower                                                  │
│    │ • Normalize & validate                                             │
│    │ • Detect discontinuities                                           │
│    │ • Resample with uniform spacing                                    │
│    │ • Compute curvature & arc length                                   │
│    │                                                                      │
│    └──► PathInfo struct {States, Curvature, ArcLength, SegmentIndex}   │
│              │                                                            │
│              ▼                                                            │
│      purePursuitFollower (class)                                        │
│        │ • 3 controller modes: blended / purePursuit / stanley          │
│        │ • Adaptive lookahead: base + vel*gain + accel*time_gain        │
│        │ • PID heading control with feedforward                         │
│        │ • Kinematic constraints: wheel speeds, accel, jerk             │
│        │                                                                  │
│        └──► step(pose, dt) → [vx, wz, status]                          │
│                  │                                                        │
│                  └────► to unifiedChassisCtrl (staged-B mode)           │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
┌─────────────────────────────────────────────────────────────────────────┐
│                    CONFIGURATION LAYER                                   │
│                                                                           │
│  config/chassis_profiles.yaml                                           │
│        │                                                                  │
│        ├──► loadChassisProfile(name) ◄──┬── defaultUnifiedParams()     │
│        │       │                          │                              │
│        │       │ Profiles:               └── defaultReedsSheppParams()  │
│        │       │  • wide_track                                           │
│        │       │  • compact_track                                        │
│        │       │  • ...                                                  │
│        │       │                                                          │
│        │       └──► params struct:                                       │
│        │              • track (wheel base width)                         │
│        │              • vx_max, wz_max (platform limits)                │
│        │              • wheel_speed_max (per-wheel limit)               │
│        │              • lookahead params, PID gains                      │
│        │              • RS refinement params                             │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
┌─────────────────────────────────────────────────────────────────────────┐
│                      CONSTRAINT LAYER                                    │
│                                                                           │
│  clampYawByWheelLimit(Vx, Wz, track, Vwheel_max, Wmax)                 │
│    │ Enforces differential-drive kinematics:                            │
│    │   v_left  = Vx - 0.5 * track * Wz  ≤  Vwheel_max                  │
│    │   v_right = Vx + 0.5 * track * Wz  ≤  Vwheel_max                  │
│    │                                                                      │
│    └──► WzOut (clamped yaw rate)                                        │
│                                                                           │
│  Used by: unifiedChassisCtrl (all modes)                                │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                     SIMULATION/TESTING LAYER                             │
│                                                                           │
│  simulateChassisController(pathStates, options)                         │
│    │ • Mode 0: Legacy five-point differentiation                        │
│    │ • Mode 1: Simple heading controller (P + FF yaw)                   │
│    │ • Mode 2: Pure pursuit (delegates to simulatePurePursuitExecution) │
│    │                                                                      │
│    └──► result {poses, commands, wheelSpeeds, status}                  │
│                                                                           │
│  simulatePurePursuitExecution(pathStates, options)                      │
│    │ • Integrates (vx, wz) using diff-drive kinematics                  │
│    │ • x_{k+1} = x_k + vx*cos(θ)*dt                                     │
│    │ • y_{k+1} = y_k + vx*sin(θ)*dt                                     │
│    │ • θ_{k+1} = θ_k + wz*dt                                            │
│    │                                                                      │
│    └──► result {poses, commands, wheelSpeeds, status, follower}        │
└─────────────────────────────────────────────────────────────────────────┘
```

#### **Component Relationships**

**1. Configuration Chain**
```
chassis_profiles.yaml → loadChassisProfile() → params struct → {unifiedChassisCtrl, purePursuitFollower, rsRefinePath}
                                                    ▲
                                                    │
                            defaultUnifiedParams() ─┴─ defaultReedsSheppParams()
```

**2. Mode-Based Data Flow**

| Mode | Input | Path Through | Output |
|------|-------|--------------|--------|
| **Holistic** | GIK poses (x,y,θ,t) + arm.qdot | → unifiedChassisCtrl<br>→ numerical differentiation<br>→ heading correction | UnifiedCmd<br>{base.Vx, Vy, Wz;<br>arm.qdot} |
| **Staged-C** | Stage C poses (x,y,θ,t) + arm.qdot | → unifiedChassisCtrl<br>→ numerical differentiation<br>→ heading correction | UnifiedCmd<br>{base.Vx, Vy, Wz;<br>arm.qdot} |
| **Staged-B** | Path waypoints (Nx3) | → [optional smoothing]<br>→ preparePathForFollower<br>→ purePursuitFollower<br>→ unifiedChassisCtrl | UnifiedCmd<br>{base.Vx, 0, Wz;<br>arm.qdot} |

**3. Path Smoothing Pipeline (Optional, Stage B)**
```
Raw Path → rsRefinePath         → rsClothoidRefine        → Smoothed Path
            (Reeds-Shepp shortcuts)  (Clothoid spline fitting)
            • Random shortcut attempts   • Split at gear changes
            • Collision checking         • Fit referencePathFrenet
            • Cusp penalty               • Curvature continuity
```

#### **File Descriptions**

| File | Lines | Layer | Purpose | Key Inputs | Key Outputs |
|------|-------|-------|---------|------------|-------------|
| **unifiedChassisCtrl.m** | 129 | Execution | Central controller routing 3 modes | mode, ref struct, params | UnifiedCmd struct |
| **purePursuitFollower.m** | 338 | Following | Chassis-aware path follower (class) | PathInfo, pose, dt | vx, wz, status |
| **preparePathForFollower.m** | 255 | Following | Path normalization & preprocessing | raw path (Nx3) | PathInfo struct |
| **clampYawByWheelLimit.m** | 45 | Constraint | Enforce diff-drive wheel limits | Vx, Wz, track, limits | WzOut (clamped) |
| **loadChassisProfile.m** | 148 | Config | YAML profile loader with overrides | profile name, yaml path | params struct |
| **defaultUnifiedParams.m** | 12 | Config | Default unified controller params | profile name (opt) | params struct |
| **defaultReedsSheppParams.m** | 32 | Config | Default RS refinement params | - | params struct |
| **rsRefinePath.m** | 292 | Following | Reeds-Shepp shortcutting smoother | path, map, RS params | smoothed path |
| **rsClothoidRefine.m** | 204 | Following | Clothoid spline smoother | path, params | smoothed path |
| **simulateChassisController.m** | 328 | Testing | Multi-mode controller simulator | path, mode, options | simulation result |
| **simulatePurePursuitExecution.m** | 83 | Testing | Pure pursuit integration | path, sample time | poses, commands |

#### **Key Data Structures**

**UnifiedCmd** (output of `unifiedChassisCtrl`)
```matlab
struct with fields:
    base: struct
        Vx: double      % Forward velocity (m/s)
        Vy: double      % Lateral velocity (m/s, 0 for diff-drive)
        Wz: double      % Yaw rate (rad/s)
    arm: struct
        qdot: [7x1]     % Arm joint velocities (rad/s)
```

**PathInfo** (output of `preparePathForFollower`)
```matlab
struct with fields:
    States: [Nx3]        % [x, y, yaw] waypoints
    Curvature: [Nx1]     % Curvature at each waypoint
    ArcLength: [Nx1]     % Cumulative arc length
    SegmentIndex: [Nx1]  % Segment ID (for discontinuous paths)
```

**Chassis Params** (from `loadChassisProfile`)
```matlab
struct with fields:
    track: double              % Wheel base width (m) - STANDARDIZED at 0.574 m
    vx_max: double             % Max forward velocity (m/s)
    wz_max: double             % Max yaw rate (rad/s)
    wheel_speed_max: double    % Max per-wheel speed (m/s)
    LookaheadBase: double      % Base lookahead distance (m)
    LookaheadVelGain: double   % Velocity-dependent lookahead gain
    HeadingKp: double          % Heading PID proportional gain
    FeedforwardGain: double    % Feedforward gain for heading
    yawKp: double              % Yaw correction gain (unified ctrl)
    yawKff: double             % Yaw feedforward gain (unified ctrl)
    % ... plus RS refinement params
```

**Note on Configuration:**
Chassis parameters are now part of the **unified configuration system** (see Section 6: Unified Parameter Configuration System). While `loadChassisProfile.m` still exists for chassis-only parameters, the recommended approach is to use `loadPipelineProfile()` which loads chassis parameters alongside Stage B/C/GIK settings from a single YAML file.

```matlab
% New unified approach (recommended):
cfg = gik9dof.loadPipelineProfile('default');
chassisParams = cfg.chassis;  % Extract chassis parameters

% Legacy chassis-only approach (still supported):
chassisParams = gik9dof.control.loadChassisProfile("wide_track");
```

#### **Integration with Main Pipeline**

The chassis control subsystem is invoked by:
- **Stage B**: `runPipelineStageB.m` → path planning → `rsRefinePath` → `preparePathForFollower` → `purePursuitFollower` → `unifiedChassisCtrl("staged-B")`
- **Stage C**: `runPipelineStageC.m` → GIK tracking → `unifiedChassisCtrl("staged-C")`
- **Holistic**: `runPipelineHolistic.m` → GIK solver → `unifiedChassisCtrl("holistic")`

All modes use **clampYawByWheelLimit** internally to ensure differential-drive kinematic feasibility. Parameters for all stages (B, C, holistic, GIK) are now consolidated in `config/pipeline_profiles.yaml` for consistency.

### Visualization Subsystem (6 files)

| File | Purpose | Inputs | Output | Primary Use |
|------|---------|--------|--------|-------------|
| **+viz/animate_whole_body.m** | Main animator | robot, trajectories, options | MP4 video | Primary animation |
| **animateStagedWithHelper.m** | Staged wrapper | log, options | Calls animate_whole_body | Staged mode |
| **animateHolisticWithHelper.m** | Holistic wrapper | log, options | Calls animate_whole_body | Holistic mode |
| **animateStagedLegacy.m** | Legacy animator | log, options | Figure | **DEPRECATED** |
| **animateTrajectory.m** | Simple animator | trajectory | Figure | Quick visualization |
| **+viz/animatePurePursuitSimulation.m** | PP visualizer | simulation | Animated plot | Debug pure pursuit |

### Evaluation & Plotting (7 files)

| File | Purpose | Metrics Computed | Usage Context |
|------|---------|------------------|---------------|
| **evaluateLog.m** | Log evaluation | EE error, success rate | Post-simulation |
| **generateLogPlots.m** | Standard plots | Arm joints, chassis vel | Diagnostic output |
| **plotTrajectoryLog.m** | Trajectory plots | Paths, errors | Quick analysis |
| **evaluatePathSmoothness.m** | Smoothness metrics | Curvature, jerk | Path quality |
| **evaluateCollisionIntrusion.m** | Collision check | Min distances | Safety validation |
| **evaluateChassisConstraints.m** | Constraint check | Velocity, wheel limits | Feasibility check |
| **computeBaseRibbonMetrics.m** | Base path metrics | Curvature histogram | Stage B/C analysis |

### Environment & Assets (4 files)

| File | Purpose | Key Data | Dependencies |
|------|---------|----------|--------------|
| **environmentConfig.m** | Config factory | Base home, obstacles, margins | - |
| **addFloorDiscs.m** | Obstacle setup | Disc bodies added to tree | rigidBody |
| **collisionTools.m** | Mesh attachment | STL → collision geometry | meshes/ directory |
| **demoFloorDiscs.m** | Visualization demo | Shows obstacle setup | Testing |

### Internal Utilities (5 files)

| File | Purpose | Functionality | Critical For |
|------|---------|---------------|--------------|
| **+internal/createResultsFolder.m** | Folder management | Timestamped directories | Result organization |
| **+internal/resolvePath.m** | Path resolution | Relative → absolute paths | Asset loading |
| **+internal/projectRoot.m** | Root finder | Project root detection | Path resolution |
| **+internal/VelocityEstimator.m** | Velocity estimation | Backward differences | Base velocity |
| **+internal/addChassisFootprint.m** | Footprint builder | Collision bodies | Obstacle avoidance |

### Orchestration & Entry Scripts (12 files at root)

| File | Purpose | Typical Usage | Status |
|------|---------|---------------|--------|
| **run_staged_reference.m** | Staged entry | Production runs | ✅ Active |
| **run_environment_compare.m** | Holistic vs staged | Comparison studies | ✅ Active |
| **run_fresh_sim_with_animation.m** | Quick test | Fast iteration | ✅ Active |
| **run_parametric_study.m** | Parameter sweep | Optimization | ✅ Active |
| **run_comprehensive_chassis_study.m** | Chassis study | Controller tuning | ✅ Active |
| **regenerate_animations_from_logs.m** | Reanimate | Post-processing | ✅ Active |
| **matlab/run_gik_iteration_study.m** | Iteration study | Solver analysis | ✅ Active |
| **matlab/run_parameter_sweep.m** | General sweep | Batch testing | ✅ Active |
| **matlab/run_stageb_mode_compare.m** | Stage B compare | Mode validation | ✅ Active |
| **matlab/run_stageb_purehyb_smoke.m** | Smoke test | Quick validation | ✅ Active |
| **matlab/unified_chassis_replay.m** | Chassis replay | Controller debug | ✅ Active |
| **export_all_commands.m** | Command export | Documentation | ⚠️ Utility |

### Test & Debug Scripts (20+ files at root)

**Status:** Many overlap in functionality, candidates for consolidation

| Category | Files | Purpose | Recommendation |
|----------|-------|---------|----------------|
| **Animation Debug** | `debug_animation_sync.m`, `debug_sampling_mismatch.m`, `debug_stage_boundaries.m`, `debug_stagec_ee_path.m` | Fix animation issues | ⚠️ Archive after fixes |
| **Animation Generation** | `generate_animation_from_saved_log.m`, `generate_comprehensive_animations.m`, `generate_final_animation.m`, `generate_parametric_animations.m`, `generate_sweep_animations.m` | Various generators | ⚠️ Consolidate |
| **Test Scripts** | `test_animation_sync_fix.m`, `test_complete_fix.m`, `test_comprehensive_evaluation.m`, `test_enhanced_logging.m`, `test_issue_fixes.m`, `test_parameter_sweep.m`, `test_single_animation_sync.m`, `test_stage_sync_fix.m`, `test_stagec_path_fix.m`, `test_tuned_parameters.m` | Validation tests | ⚠️ Move to tests/ |
| **Analysis** | `analyze_all_tests.m`, `analyze_stage_collisions.m`, `check_reference_quality.m`, `compare_test_configs.m`, `investigate_cusps.m` | Post-run analysis | ✅ Keep |
| **Temporary** | `tmp_compare.mat`, `tmp_json.mat`, `tmp_pipeline.mat`, `tmp_regen_staged_only.m`, `tmp_regen.m`, `temp_ref_rs_refine.m` | Scratch work | ❌ Delete |

---

## Dependency Graph & File Relationships

### ⚠️ CRITICAL: Understanding runStagedReference vs runStagedTrajectory

**These two functions have confusingly similar names but serve very different purposes:**

---

### 🚨 CRITICAL WARNING: Different Default Parameters! **[RESOLVED]**

**✅ UPDATE (2025-10-11): Defaults have been unified!**

Both `runStagedReference` and `trackReferenceTrajectory` now use **identical production-tuned defaults**:
- ✅ **MaxIterations = 1500** (high accuracy)
- ✅ **StageBMode = "pureHyb"** (production planning mode)
- ✅ **RateHz = 10** Hz (production control frequency)
- ✅ **UseStageBHybridAStar = true** (enables path planning)

**The core algorithm is identical, and NOW the default parameters are also identical!**

#### Historical Context (Before Unification)

Previously, these functions had different defaults, causing inconsistent behavior:

| Parameter | OLD runStagedReference | OLD trackReferenceTrajectory | Status |
|-----------|------------------------|-----------------------------|---------|
| MaxIterations | 150 | 1500 | ✅ **NOW: Both use 1500** |
| StageBMode | "pureHyb" | "gikInLoop" | ✅ **NOW: Both use "pureHyb"** |
| RateHz | 10 Hz | 100 Hz | ✅ **NOW: Both use 10 Hz** |
| UseStageBHybridAStar | true | false | ✅ **NOW: Both use true** |
| StageBDesiredLinearVelocity | 0.5 m/s | 0.6 m/s | ⚠️ Still different (minor) |
| StageBMaxAngularVelocity | 2.0 rad/s | 2.5 rad/s | ⚠️ Still different (minor) |

**Minor remaining differences:**
- Velocity parameters have small differences but don't fundamentally change behavior
- Can be explicitly overridden if exact matching is needed

**✅ Result:** Calling either function without parameters now produces consistent, production-ready results!

---

#### **runStagedReference.m** - High-Level Convenience Wrapper (132 lines)

**Location:** `matlab/+gik9dof/runStagedReference.m`

**Purpose:** User-facing convenience function for quick staged pipeline execution

**What it does:**
1. Loads default environment configuration (`environmentConfig()`)
2. Applies user overrides to environment settings
3. Calls `trackReferenceTrajectory` with `'Mode', 'staged'`
4. Creates timestamped results folder
5. Saves log to disk automatically
6. Returns result struct with paths and metadata

**Who calls it:**
- User scripts: `test_tuned_parameters.m`, `run_parametric_study.m`, etc.
- Parametric studies and batch comparisons
- Any script needing quick staged execution with auto-save

**Who it calls:**
- `gik9dof.trackReferenceTrajectory(..., 'Mode', 'staged', ...)`

**Usage pattern:**
```matlab
% Simple one-liner for staged execution
result = gik9dof.runStagedReference( ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 150);

% Automatically:
% - Loads environment config
% - Creates results/<timestamp>/ folder
% - Saves log_staged_ppForIk.mat
% - Returns result with log and file paths
```

**Key features:**
- ✅ Convenience: No manual environment setup or result saving
- ✅ Production-tuned defaults (1500 iter, pureHyb, 10 Hz)
- ✅ Auto-save to timestamped folder
- ✅ Returns complete metadata struct
- ✅ **Now uses same defaults as trackReferenceTrajectory!**

---

#### **runStagedTrajectory.m** - Core Staged Execution Engine (1986 lines)

**Location:** `matlab/+gik9dof/runStagedTrajectory.m`

**Purpose:** Low-level engine that orchestrates the actual 3-stage execution

**What it does:**
1. Executes **Stage A** (arm ramp-up) - 50 samples
2. Executes **Stage B** (base navigation) - Hybrid A* + controller
3. Executes **Stage C** (full-body tracking) - remaining waypoints
4. Merges stage logs into unified pipeline
5. Returns complete pipeline log (no file I/O)

**Who calls it:**
- `trackReferenceTrajectory` (when `Mode='staged'`)
- **NOT** called directly by user scripts

**Who it calls:**
- `runStageA()` - Local function for arm ramp
- `runStageBGikInLoop()` or `runStageBPureHyb()` - Base navigation
- `runStageCPpForIk()` or `runStageCPureIk()` - Full tracking
- `mergeStageLogs()` - Log concatenation

**Usage pattern:**
```matlab
% Low-level call (typically NOT used directly by users)
pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
    'InitialConfiguration', q0, ...
    'ConfigTools', configTools, ...
    'DistanceSpecs', distanceSpecs, ...
    'RateHz', 10, ...
    'ExecutionMode', 'ppForIk');
```

**Key features:**
- ⚙️ Core logic: Implements actual 3-stage decomposition
- ⚙️ No I/O: Does not create folders or save files
- ⚙️ Complex options: 50+ parameters for fine-grained control
- ⚙️ Pure computation: Returns log struct only

---

#### **Which Defaults Should You Use?**

**✅ Good News: Defaults are now unified!**

Both `runStagedReference` and `trackReferenceTrajectory` now use the same production-tuned defaults:
- **MaxIterations = 1500** (high accuracy)
- **StageBMode = "pureHyb"** (production planning)
- **RateHz = 10 Hz** (efficient control frequency)
- **UseStageBHybridAStar = true** (path planning enabled)

| Use Case | Recommended Function | Additional Settings | Rationale |
|----------|---------------------|---------------------|-----------|
| **Production/Deployment** | Either function with defaults | None needed | Both use production-tuned settings |
| **Quick testing** | `runStagedReference` | Use defaults | Auto-save convenience |
| **Custom environment** | `trackReferenceTrajectory` | Specify EnvironmentConfig | More flexibility |
| **Higher control rate** | Either function | Set RateHz=100 | If needed for specific research |
| **Consistency** | Either function | Defaults now match! | No explicit overrides needed |

**Note:** The small velocity parameter differences (0.5 vs 0.6 m/s) are minor and don't fundamentally affect behavior. Override them if exact matching is critical for your use case.

---

#### **The Three-Level Architecture**

```
┌─────────────────────────────────────────────────────────────┐
│  LEVEL 3: User Convenience                                  │
│  runStagedReference()                                       │
│  - Loads default environment                                │
│  - Auto-saves to timestamped folder                         │
│  - Returns result struct with paths                         │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  LEVEL 2: Mode Router                                       │
│  trackReferenceTrajectory()                                 │
│  - Routes between holistic vs staged modes                  │
│  - Loads JSON trajectory and robot model                    │
│  - Sets up environment (obstacles, constraints)             │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  LEVEL 1: Execution Engine                                  │
│  runStagedTrajectory()                                      │
│  - Stage A: Arm ramp (50 samples)                           │
│  - Stage B: Base navigation (Hybrid A* + controller)        │
│  - Stage C: Full tracking (IK + controller)                 │
│  - Merges A+B+C logs → unified pipeline                     │
└─────────────────────────────────────────────────────────────┘
```

**Design Philosophy:**
- **Level 3** = Ease of use (defaults, auto I/O, metadata)
- **Level 2** = Flexibility (mode switching, custom setup)
- **Level 1** = Core logic (pure computation, no I/O side effects)

---

#### **When to Use Which Function**

| Your Goal | Use This Function | Reason |
|-----------|-------------------|--------|
| **Quick test/study** | `runStagedReference` | Auto-saves, minimal setup required |
| **Parametric sweep** | `runStagedReference` | Handles folder creation automatically |
| **Custom pipeline** | `trackReferenceTrajectory` | Full control over mode and environment |
| **Mode comparison** | `trackReferenceTrajectory` | Can switch between holistic/staged |
| **Direct engine** | ~~`runStagedTrajectory`~~ | ❌ Almost never call this directly |
| **Debugging stages** | `trackReferenceTrajectory` | Access individual stage logs |

---

#### **Common Confusion Points**

**Q: Are the core algorithms the same?**
- ✅ **YES!** `runStagedReference` simply calls `trackReferenceTrajectory` with `Mode='staged'`
- ✅ The underlying Stage A/B/C execution logic in `runStagedTrajectory` is identical
- ✅ **UPDATE:** Default parameters are now also identical!

**Q: Do they produce the same results by default?**
- ✅ **YES (as of 2025-10-11)!** Defaults have been unified
- Both use: MaxIterations=1500, StageBMode="pureHyb", RateHz=10, UseStageBHybridAStar=true
- Minor velocity parameter differences remain but don't fundamentally change behavior
- **No need to explicitly specify parameters for consistency anymore!**

**Q: Why two functions with such similar names?**
- `runStagedReference` = High-level **workflow** function (convenience + I/O)
- `runStagedTrajectory` = Low-level **computation** function (pure engine)
- Separation of concerns: I/O management vs pure computation

**Q: Which one should I call in my script?**
- **Users:** Almost always use `runStagedReference` for convenience
- **Framework:** Let `trackReferenceTrajectory` call `runStagedTrajectory`
- **Direct:** Never call `runStagedTrajectory` unless you're modifying core logic
- **Either works now:** Defaults are consistent!

**Q: Where does the actual 3-stage execution logic live?**
- **ALL** in `runStagedTrajectory` (1986 lines of stage orchestration)
- `runStagedReference` is just a thin wrapper (132 lines) with I/O + defaults

**Q: Can I bypass runStagedReference?**
- Yes: Call `trackReferenceTrajectory` directly for more control
- But: You lose auto-save, default environment loading, and result metadata
- Trade-off: Maximum flexibility vs convenience
- **Now with unified defaults:** Either approach produces consistent results!

---

#### **Code Examples: All Three Levels**

```matlab
% ============================================================
% METHOD 1: High-level convenience (RECOMMENDED FOR USERS)
% ============================================================
result = gik9dof.runStagedReference( ...
    'ExecutionMode', 'ppForIk', ...
    'MaxIterations', 150, ...
    'StageBMode', 'pureHyb', ...
    'RunLabel', 'my_test');

% Result contains:
%   result.log           - Full pipeline log
%   result.logPath       - 'results/20251011_123456_my_test/log_staged_ppForIk.mat'
%   result.resultsDir    - 'results/20251011_123456_my_test/'
%   result.environment   - Environment config used
%   result.executionMode - 'ppForIk'
%   result.options       - All options used

% ============================================================
% METHOD 2: Mid-level with custom environment setup
% ============================================================
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.25;  % Custom safety margin
env.FloorDiscs(3).Radius = 0.30;  % Modify obstacle

log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'ExecutionMode', 'ppForIk', ...
    'EnvironmentConfig', env, ...
    'MaxIterations', 150, ...
    'RateHz', 10);

% You manually handle saving:
resultsDir = sprintf('results/custom_%s', datestr(now, 'yyyymmdd_HHMMSS'));
mkdir(resultsDir);
save(fullfile(resultsDir, 'log_staged_custom.mat'), 'log');

% ============================================================
% METHOD 3: Low-level direct call (RARE - ONLY FOR EXPERTS)
% ============================================================
% Only if you need to completely bypass the standard pipeline
robot = gik9dof.createRobotModel();
trajStruct = gik9dof.loadTrajectory('1_pull_world_scaled.json');
configTools = configurationTools(robot);
q0 = homeConfiguration(robot);

pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
    'InitialConfiguration', q0, ...
    'ConfigTools', configTools, ...
    'DistanceSpecs', [], ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 150);

% pipeline.qTraj contains full [9×N] trajectory
% pipeline.stageLogs.{stageA,stageB,stageC} contain per-stage logs
% You handle ALL setup and saving yourself
```

---

### High-Level Call Hierarchy

```
USER
 │
 ├─ run_staged_reference.m ─────────────┐
 ├─ run_environment_compare.m ──────────┤
 ├─ run_fresh_sim_with_animation.m ─────┤
 └─ run_parametric_study.m ─────────────┤
                                        │
                                        ▼
                    trackReferenceTrajectory.m
                    (Mode Router: holistic vs staged)
                         │
                         ├─ holistic ───────────────┐
                         │                          │
                         └─ staged ──────────────┐  │
                                                 │  │
                ┌────────────────────────────────┘  │
                ▼                                   ▼
        runStagedTrajectory.m              runTrajectoryControl.m
                │                                   │
    ┌───────────┼───────────┐                      │
    ▼           ▼           ▼                      │
  Stage A    Stage B     Stage C                   │
    │           │           │                      │
    └───────────┴───────────┴──────────────────────┘
                            │
                            ▼
                    createGikSolver.m
                            │
                            ▼
              generalizedInverseKinematics
                    (MATLAB Toolbox)
```

### Detailed Staged Mode Call Graph

```
run_staged_reference.m
│
├─> gik9dof.environmentConfig()
│   └─> Returns: base home, obstacles, margins
│
├─> gik9dof.control.loadChassisProfile()
│   ├─> Reads: config/chassis_profiles.yaml
│   └─> Returns: chassis parameters struct
│
└─> gik9dof.trackReferenceTrajectory('Mode', 'staged', ...)
    │
    ├─> gik9dof.internal.resolvePath()
    │   └─> Resolves: 1_pull_world_scaled.json
    │
    ├─> jsondecode(fileread(jsonPath))
    │   └─> Loads: 148 EE waypoints
    │
    ├─> gik9dof.createRobotModel()
    │   ├─> importrobot() → URDF parsing
    │   └─> Returns: rigidBodyTree + footprint info
    │
    ├─> gik9dof.configurationTools(robot)
    │   └─> Returns: utility functions for configs
    │
    ├─> gik9dof.addFloorDiscs(robot, discs)
    │   └─> Adds obstacle bodies to tree
    │
    ├─> gik9dof.collisionTools(robot)
    │   └─> Attaches STL meshes from meshes/
    │
    └─> gik9dof.runStagedTrajectory(robot, trajStruct, ...)
        │
        ├─────> STAGE A: Arm Ramp-Up
        │       │
        │       ├─> generateStagePoses(T0, T1, 50, 'arm')
        │       ├─> gik9dof.createGikSolver(robot, ...)
        │       │   └─> generalizedInverseKinematics()
        │       │
        │       └─> gik9dof.runTrajectoryControl(bundleA, trajA, ...)
        │           ├─> bundle.solve() loop (50 iterations)
        │           └─> Returns: logA
        │
        ├─────> STAGE B: Base Navigation
        │       │
        │       ├─ IF pureHyb:
        │       │  │
        │       │  ├─> plannerPlanStateSpace() [Hybrid A*]
        │       │  ├─> gik9dof.control.rsRefinePath() [optional]
        │       │  │   └─> Reeds-Shepp shortcuts
        │       │  ├─> gik9dof.control.rsClothoidRefine() [optional]
        │       │  │   └─> Clothoid smoothing
        │       │  ├─> gik9dof.control.preparePathForFollower()
        │       │  │   └─> Interpolate + resample
        │       │  ├─> gik9dof.control.simulateChassisController()
        │       │  │   ├─> gik9dof.control.unifiedChassisCtrl() [loop]
        │       │  │   └─> Returns: simRes with poses, commands
        │       │  └─> buildSyntheticStageBLog()
        │       │      └─> Returns: logB
        │       │
        │       └─ IF gikInLoop:
        │          │
        │          ├─> interpolateBaseStates()
        │          ├─> gik9dof.createGikSolver()
        │          └─> gik9dof.runTrajectoryControl(bundleB, trajB, ...)
        │              └─> Returns: logB
        │
        └─────> STAGE C: Full-Body Tracking
                │
                ├─ IF ppForIk:
                │  │
                │  ├─> gik9dof.createGikSolver() → bundleRef
                │  ├─> gik9dof.runTrajectoryControl() → logRef
                │  │   └─> Generate reference base path
                │  │
                │  ├─> stageCApplyBaseRefinement() [optional]
                │  │   ├─> gik9dof.control.rsRefinePath()
                │  │   └─> gik9dof.control.rsClothoidRefine()
                │  │
                │  ├─> gik9dof.control.simulateChassisController()
                │  │   └─> Returns: executed base trajectory
                │  │
                │  ├─> gik9dof.createGikSolver() → bundleFinal
                │  └─> gik9dof.runTrajectoryControl(bundleFinal, ...)
                │      └─> FixedJointTrajectory: base from simulation
                │
                └─ IF pureIk:
                   │
                   ├─> gik9dof.createGikSolver() → bundleC
                   └─> gik9dof.runTrajectoryControl(bundleC, trajC, ...)
```

### Animation Pipeline Call Graph

```
USER (any entry script with animation flag)
│
├─> gik9dof.saveRunArtifacts()
│   └─> Generates plots + animations
│
OR
│
└─> regenerate_animations_from_logs.m
    │
    └─> For each log file:
        │
        ├─> load('log_staged_*.mat')
        │
        └─> gik9dof.animateStagedWithHelper(log, ...)
            │
            ├─> Extract qTraj, timestamps
            ├─> Sample trajectory (SampleStep)
            ├─> Extract reference paths
            │
            └─> gik9dof.viz.animate_whole_body(robot, ...)
                │
                ├─> ensureArmVisuals() → load/attach STL meshes
                ├─> interpolateChassisToArm() → sync base to arm
                ├─> setupDualViewFigure() → create perspective + top views
                │
                └─> Animation loop:
                    ├─> For each frame k:
                    │   ├─> Set robot joint values to qTraj(:,k)
                    │   ├─> show(robot) → forward kinematics + render
                    │   ├─> Plot obstacles (inflated discs)
                    │   ├─> Plot reference paths (lines/ribbons)
                    │   ├─> Plot actual EE path (trail)
                    │   ├─> Plot target EE marker (red dot)
                    │   ├─> Update stage label
                    │   └─> drawnow / writeVideo
                    │
                    └─> Save to MP4 file
```

---

## Preprocessing & Configuration System

### 1. Reference Trajectory (JSON)

**File:** `1_pull_world_scaled.json`

**Structure:**
```json
{
  "poses": [
    {
      "position": [x, y, z],
      "orientation": [qx, qy, qz, qw]
    },
    ...
  ]
}
```

**Usage:**
- Loaded by `trackReferenceTrajectory.m` via `jsondecode()`
- Converted to SE(3) matrices (4×4×N)
- 148 waypoints total
- Represents desired end-effector path in world frame

**Loading Process:**
```matlab
% In trackReferenceTrajectory.m
jsonPath = gik9dof.internal.resolvePath("1_pull_world_scaled.json");
raw = jsondecode(fileread(jsonPath));
% Convert to MATLAB format (quaternion [qx,qy,qz,qw] → [qw,qx,qy,qz])
trajStruct = loadJsonTrajectory(jsonPath);
```

### 2. Chassis Configuration (YAML)

**File:** `config/chassis_profiles.yaml`

**Structure:**
```yaml
profiles:
  wide_track:
    # Kinematic parameters
    track_width: 0.574      # m (wheel separation)
    wheel_base: 0.36        # m (wheelbase length)
    wheel_radius: 0.076     # m
    
    # Velocity limits
    vx_max: 1.5             # m/s (forward)
    vx_min: -0.4            # m/s (reverse)
    wz_max: 2.0             # rad/s (yaw rate)
    wheel_speed_max: 3.3    # rad/s
    
    # Pure pursuit parameters
    lookahead_base: 0.8     # m
    lookahead_vel_gain: 0.2 # Velocity-adaptive gain
    lookahead_time_gain: 0.05 # Time-adaptive gain
    
    # Controller gains
    heading_kp: 1.2         # Heading error gain
    feedforward_gain: 0.9   # Feedforward scaling
    accel_limit: 0.8        # m/s² acceleration limit
    
    # Path processing
    waypoint_spacing: 0.15  # m (path interpolation)
    interp_spacing_min: 0.05 # m
    interp_spacing_max: 0.15 # m
    goal_tolerance: 0.05    # m
    
    # Mode selection
    reverse_enabled: true
    stageB_controller_mode: 2  # 0=legacy, 1=heading, 2=pure pursuit
    stageC_controller_mode: 2
```

**Loading Process:**
```matlab
% In trackReferenceTrajectory.m or runStagedReference.m
chassisParams = gik9dof.control.loadChassisProfile("wide_track", ...
    "Overrides", struct('accel_limit', 0.6, 'lookahead_base', 1.0));
```

**Override Mechanism:**
- Base profile loaded from YAML
- Field-by-field overrides applied
- Final struct passed to controllers

### 3. Environment Configuration

**File:** `+gik9dof/environmentConfig.m` (function)

**Returns:**
```matlab
config = struct(
    'BaseHome', [-2.0, -2.0, 0.0],  % Initial base pose [x, y, yaw]
    
    'FloorDiscs', [                  % Obstacle definitions
        struct('x', -1.0, 'y', 0.5, 'r', 0.4, 'margin', 0.05)
        struct('x', 0.5, 'y', -0.8, 'r', 0.3, 'margin', 0.05)
        ...
    ],
    
    'DistanceMargin', 0.10,          % Additional safety margin (m)
    'DistanceWeight', 5.0,           % Constraint weight
    
    'StageBMode', 'pureHyb',         % Stage B execution mode
    'StageBDockingPositionTolerance', 0.02,  % m
    'StageBDockingYawTolerance', 2*pi/180    % rad
);
```

**Usage:**
```matlab
% In entry scripts
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.15;  % Override if needed

% Pass to trackReferenceTrajectory
log = gik9dof.trackReferenceTrajectory(...
    'EnvironmentConfig', env, ...
);
```

### 4. Robot Model (URDF)

**Files:** 
- `mobile_manipulator_PPR_base_corrected.urdf` (main)
- `mobile_manipulator_PPR_base_corrected_sltRdcd.urdf` (reduced meshes)

**Loading:**
```matlab
% In createRobotModel.m
urdfPath = gik9dof.internal.resolvePath('mobile_manipulator_PPR_base_corrected.urdf');
robot = importrobot(urdfPath, 'DataFormat', 'column');
```

**Structure:**
- Base: 3 DOF (planar: x, y, theta)
- Left arm: 6 DOF (revolute joints)
- Total: 9 DOF
- End effector: left_gripper_link

**Collision Geometry:**
- Visual meshes loaded from `meshes/` directory
- Collision bodies attached via `collisionTools.apply()`

### 5. Solver Configuration

**Created by:** `createGikSolver.m`

**Key Parameters:**
```matlab
bundle = gik9dof.createGikSolver(robot, ...
    'MaxIterations', 150,           % Iteration cap
    'EnableAiming', false,          % Aiming constraint
    'DistanceSpecs', distanceSpecs, % Collision avoidance
    'DistanceWeight', 5.0           % Constraint weight
);
```

**Constraints Applied:**
1. **Pose Constraint** - End-effector SE(3) target
2. **Joint Bounds** - Position limits from URDF
3. **Distance Constraints** - Obstacle avoidance (if enabled)
4. **Aiming Constraint** - Optional camera pointing

**Weights (default):**
- Pose: 100 (position + orientation)
- Joint bounds: 10
- Distance: 5-20 (dynamic based on body)

**Solver Bundle Features** (from `createGikSolver.m`):
- Exposes helper closures: `updatePrimaryTarget`, `setDistanceBounds`
- `solve` wrapper refreshes constraint targets before invoking solver
- Works with `configurationTools` for column/struct conversions
- Supports per-waypoint aiming targets and distance bounds

**Collision Avoidance Strategy:**
- **Floor discs**: Enforced via `constraintDistanceBounds` in holistic mode and Stage C
- **Hybrid A* planning**: Occupancy map inflated by safety margin for Stage B
- **Stage A**: No distance constraints (base is locked)
- **Stage B gikInLoop**: No distance constraints (arm is locked)
- **Self-collision**: Not currently configured (meshes available but not used for constraints)

**Important Note:** Distance constraints are NOT applied in Stage A and Stage B (gikInLoop mode) because joints are locked. Only holistic mode and Stage C enforce continuous obstacle clearance.

### 6. Unified Parameter Configuration System

**Motivation:** Previously, pipeline parameters were scattered across multiple files:
- `config/chassis_profiles.yaml` - chassis-only parameters
- `runStagedTrajectory.m` - default values in function arguments
- `runStagedReference.m` - different defaults for same parameters
- `trackReferenceTrajectory.m` - another set of defaults

This led to inconsistencies and maintenance burden. The unified configuration system consolidates all parameters into a single source of truth.

#### File Structure

**File:** `config/pipeline_profiles.yaml`

**Profiles Available:**
1. **default** - Balanced, general-purpose parameters
2. **aggressive** - Faster motion (inherits from default)
3. **conservative** - Slower, safer motion (inherits from default)
4. **compact_track** - Narrower track width with appropriate adjustments

**Structure:**
```yaml
profiles:
  default:
    # Chassis parameters (kinematic + control)
    chassis:
      track: 0.574              # m (CONSISTENT with chassis_profiles.yaml)
      wheelbase: 0.36
      wheel_radius: 0.076
      vx_max: 1.5
      wz_max: 2.0
      accel_limit: 0.8
      # ... (all chassis params)
    
    # Stage B: Base navigation parameters
    stage_b:
      mode: "pureHyb"           # 'pureHyb' or 'gikInLoop'
      lookahead_distance: 0.6   # m
      desired_linear_velocity: 0.6  # m/s (now CONSISTENT across all functions)
      max_angular_velocity: 2.0 # rad/s
      hybrid_safety_margin: 0.1 # m (now CONSISTENT)
      # Hybrid A* settings
      hybrid_resolution: 0.05
      motion_primitive_length: 0.20
      # Reeds-Shepp parameters
      reeds_shepp:
        shortcut_enabled: true
        max_step: 0.05
        # ...
      # ...
    
    # Stage C: Whole-body tracking parameters
    stage_c:
      track_width: 0.574        # MUST match chassis.track (validated)
      use_base_refinement: true
      controller_mode: 2        # 2 = pure pursuit
    
    # GIK solver parameters
    gik:
      max_iterations: 150
      enable_aiming: false
      distance_weight: 5.0
    
    # Pure pursuit parameters (shared)
    pure_pursuit:
      lookahead_base: 0.8
      lookahead_vel_gain: 0.2
      # ...
    
    # Holistic mode parameters
    holistic:
      lookahead_distance: 0.6
      desired_linear_velocity: 0.5
      # ...

  aggressive:
    inherits: "default"
    overrides:
      chassis: {vx_max: 1.8, accel_limit: 1.2}
      stage_b: {desired_linear_velocity: 0.8}
      holistic: {desired_linear_velocity: 0.7}

  conservative:
    inherits: "default"
    overrides:
      chassis: {vx_max: 1.0, accel_limit: 0.5}
      stage_b: {desired_linear_velocity: 0.4}
```

#### Loading Function

**File:** `matlab/+gik9dof/loadPipelineProfile.m`

**Usage:**
```matlab
% Load a profile
cfg = gik9dof.loadPipelineProfile('default');

% Load with section overrides
cfg = gik9dof.loadPipelineProfile('aggressive', ...
    'stage_b', struct('desired_linear_velocity', 0.9), ...
    'chassis', struct('accel_limit', 1.5));
```

**Features:**
- **Inheritance resolution**: Profiles can inherit from others
- **Deep merge**: Overrides are recursively merged
- **Validation**: Checks consistency (e.g., `stage_c.track_width == chassis.track`)
- **YAML fallback**: Uses custom parser if `yamlread()` unavailable
- **Metadata**: Returns `config.meta.profile`, `config.meta.validationWarnings`

**Return Structure:**
```matlab
cfg = struct(
    'chassis', struct(...),      % All chassis parameters
    'stage_b', struct(...),      % Stage B parameters
    'stage_c', struct(...),      % Stage C parameters
    'gik', struct(...),          % GIK solver parameters
    'pure_pursuit', struct(...), % Pure pursuit controller
    'holistic', struct(...),     % Holistic mode parameters
    'meta', struct(              % Metadata
        'profile', 'aggressive',
        'validationWarnings', {})
);
```

#### Integration with Main Functions

All three main pipeline functions now accept `PipelineConfig`:

**1. trackReferenceTrajectory.m**
```matlab
% Recommended: Unified config
cfg = gik9dof.loadPipelineProfile('aggressive');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);

% Legacy: Individual parameters (still supported)
log = gik9dof.trackReferenceTrajectory('MaxIterations', 200);
```

**2. runStagedTrajectory.m**
```matlab
% Recommended: Unified config
cfg = gik9dof.loadPipelineProfile('default');
log = gik9dof.runStagedTrajectory(robot, 'PipelineConfig', cfg);

% Legacy: Individual parameters
log = gik9dof.runStagedTrajectory(robot, 'StageBDesiredLinearVelocity', 0.7);
```

**3. runStagedReference.m**
```matlab
% Recommended: Unified config
cfg = gik9dof.loadPipelineProfile('conservative');
result = gik9dof.runStagedReference('PipelineConfig', cfg);

% Legacy: Individual parameters
result = gik9dof.runStagedReference('StageBMode', 'gikInLoop');
```

#### Benefits

1. **Single Source of Truth**: All parameters in `pipeline_profiles.yaml`
2. **Consistency**: Validation ensures parameters match across stages
3. **Profile Management**: Easy to switch between aggressive/conservative/default
4. **Maintainability**: Change parameter once, affects all functions
5. **Backward Compatible**: Existing code with individual parameters still works
6. **Inheritance**: Profiles can build on others (DRY principle)
7. **Track Width Consistency**: Corrected to 0.574 m throughout (was 0.674, 0.576, 0.573)

#### Parameter Inconsistencies Resolved

| Parameter | Before (runStagedTrajectory) | Before (runStagedReference) | After (Unified) |
|-----------|------------------------------|----------------------------|-----------------|
| StageBDesiredLinearVelocity | 0.6 m/s | 0.5 m/s | 0.6 m/s (default) |
| StageBHybridSafetyMargin | 0.1 m | 0.15 m | 0.1 m (default) |
| StageBMode | "gikInLoop" | "pureHyb" | "pureHyb" (default) |
| Track Width | 0.674, 0.576 | 0.573 | 0.574 m (all files) |

#### Migration Guide

**Step 1:** Create or modify a profile in `pipeline_profiles.yaml`
```yaml
profiles:
  my_custom:
    inherits: "default"
    overrides:
      stage_b: {desired_linear_velocity: 0.75}
```

**Step 2:** Load profile in your script
```matlab
cfg = gik9dof.loadPipelineProfile('my_custom');
```

**Step 3:** Pass to pipeline function
```matlab
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
% OR
result = gik9dof.runStagedReference('PipelineConfig', cfg);
```

**Step 4 (Optional):** Override specific parameters
```matlab
cfg = gik9dof.loadPipelineProfile('default', ...
    'stage_b', struct('desired_linear_velocity', 0.9));
```

---

## Real-Time Simulation Pipeline

### Stage A: Arm Ramp-Up

**Purpose:** Align arm to first waypoint while keeping base stationary

**Data Flow:**
```
Input:
  q0           [9×1]  Initial configuration
  T1           [4×4]   First EE pose target
  
Process:
  1. T0 = getTransform(robot, q0, 'left_gripper_link')
  2. Generate 50 interpolated poses: T0 → T1 (orientation + z-height)
  3. Lock base joints in GIK solver (tight bounds = q0)
  4. Run IK loop for 50 waypoints
  
Output:
  logA.qTraj   [9×50]  Joint trajectory (base unchanged, arm moves)
  logA.eePositions [3×50]  Actual EE positions
  logA.timestamps  [1×50]  Time stamps
  qA_end       [9×1]  Final configuration
```

**Key Function:** `lockJointBounds(bundleA.constraints.joint, baseIdx, q0)`

### Stage B: Base Navigation

#### Mode 1: pureHyb (Pure Hybrid A*)

**Purpose:** Plan and execute base-only path to docking pose

**Data Flow:**
```
Input:
  qA_end             [9×1]  Start configuration
  T1                 [4×4]   First EE target (defines goal base pose)
  options            Struct with planning parameters
  
Process:
  1. Compute goal base pose via inverse FK:
     goalBase = computeGoalBasePose(robot, qA_end, T1, armJoints)
     
  2. Plan path:
     a. Create occupancy map from floor discs
     b. Run Hybrid A* planner
     c. (Optional) Apply Reeds-Shepp shortcuts
     d. (Optional) Apply clothoid smoothing
     
  3. Simulate controller execution:
     a. Prepare path (interpolate, resample)
     b. Run unifiedChassisCtrl in loop
     c. Generate (Vx, Wz) commands
     d. Integrate to get base trajectory
     
  4. Build synthetic log:
     a. Create qTraj with fixed arm, moving base
     b. Compute EE poses via FK
     c. Generate timestamps
     
  5. Compute diagnostics:
     a. Base ribbon metrics (curvature, cusps)
     b. RS smoothing statistics
     c. Clothoid fitting results
  
Output:
  logB.qTraj         [9×N]  Base moves, arm frozen
  logB.pathStates    [N×3]   (x, y, theta) executed path
  logB.cmdLog        Table   (time, Vx, Wz) commands
  logB.purePursuit   Struct  Controller simulation data
  logB.planner       Struct  Planning metadata
  logB.diagnostics   Struct  Enhanced metrics (Phase 2)
  qB_end             [9×1]  Final docked configuration
```

**Planning Chain:**
```
plannerPlanStateSpace() → Hybrid A* path
    ↓
rsRefinePath() → Reeds-Shepp shortcuts (optional)
    ↓
rsClothoidRefine() → Clothoid smoothing (optional)
    ↓
preparePathForFollower() → Interpolation & resampling
    ↓
simulateChassisController() → Pure pursuit execution
```

#### Mode 2: gikInLoop (GIK-based)

**Purpose:** Use IK solver to move base while keeping arm frozen

**Data Flow:**
```
Input:
  qA_end       [9×1]  Start configuration
  goalBase     [1×3]   Goal (x, y, theta)
  
Process:
  1. Interpolate straight-line base path (50 samples)
  2. Lock arm joints in GIK solver
  3. Run IK loop with base pose constraints
  
Output:
  logB.qTraj   [9×50]  IK-driven base trajectory
  qB_end       [9×1]   Final configuration
```

### Stage C: Full-Body Tracking

#### Mode 1: ppForIk (Pure Pursuit for IK)

**Purpose:** Track remaining waypoints with chassis controller driving base

**Data Flow:**
```
Input:
  qB_end          [9×1]  Start configuration (docked)
  trajStruct      Struct  Remaining waypoints (148 total)
  
Process:
  PASS 1 - Generate Reference Path:
    1. Create bundleRef (standard GIK solver)
    2. Run runTrajectoryControl() → logRef
    3. Extract base trajectory: baseReference = logRef.qTraj(baseIdx, :)
    
  PASS 1.5 - Optional Base Refinement:
    IF StageCUseBaseRefinement:
      a. Apply RS shortcuts to baseReference
      b. Apply clothoid smoothing
      c. Update baseReference with smoothed path
    
  PASS 2 - Simulate Controller:
    1. Prepare base path for follower
    2. Run simulateChassisController()
       → generates executed base trajectory
    
  PASS 3 - Final IK with Fixed Base:
    1. Create bundleFinal (GIK solver)
    2. Run runTrajectoryControl() with:
       FixedJointTrajectory.Indices = baseIdx
       FixedJointTrajectory.Values = executed base trajectory
    3. IK solves for arm only, base follows executed path
    
Output:
  logC.qTraj                [9×N]  Full trajectory
  logC.referenceInitialIk   logRef Reference pass log
  logC.purePursuit          Struct Controller simulation
  logC.execBaseStates       [N×3]  Executed base path
  logC.referenceBaseStates  [N×3]  Reference base path
  logC.cmdLog               Table  (time, Vx, Wz) commands
  logC.diagnostics          Struct Enhanced metrics
```

**Three-Pass Architecture:**
```
Pass 1: GIK Reference
  runTrajectoryControl(bundleRef, trajC, ...)
  → Full IK solution (reference base path)

Pass 1.5: Base Smoothing (optional)
  rsRefinePath() + rsClothoidRefine()
  → Smoother base path

Pass 2: Chassis Simulation
  simulateChassisController(baseReference, ...)
  → Realistic executed base path

Pass 3: GIK with Fixed Base
  runTrajectoryControl(bundleFinal, trajC, ...
      'FixedJointTrajectory', executedBase)
  → Arm IK with actual base motion
```

#### Mode 2: pureIk (Pure IK)

**Purpose:** Standard full-body IK without controller simulation

**Data Flow:**
```
Input:
  qB_end      [9×1]  Start configuration
  trajC        Struct  Remaining waypoints
  
Process:
  1. Create bundleC (standard GIK solver)
  2. Run runTrajectoryControl()
     → Full-body IK for all waypoints
  
Output:
  logC.qTraj  [9×N]  Full trajectory
```

### Log Merging

**Function:** `mergeStageLogs(logA, logB, logC)`

**Process:**
```matlab
% Concatenate trajectories
pipeline.qTraj = [logA.qTraj, logB.qTraj(:,2:end), logC.qTraj(:,2:end)];

% Adjust timestamps (offset by previous stage duration)
offsetB = logA.timestamps(end);
offsetC = offsetB + logB.timestamps(end);
pipeline.timestamps = [logA.timestamps, ...
                       logB.timestamps + offsetB, ...
                       logC.timestamps + offsetC];

% Store per-stage logs
pipeline.stageLogs.stageA = logA;
pipeline.stageLogs.stageB = logB;
pipeline.stageLogs.stageC = logC;

% Stage boundaries (for visualization)
pipeline.stageBoundaries = [length(logA.timestamps), ...
                            length(logA.timestamps) + length(logB.timestamps)];
```

---

## Holistic Mode: Full-Body IK Pipeline

### Overview

**Holistic mode** directly controls all 9 DOF simultaneously to track the reference trajectory without staged decomposition. The entire robot (base + arm) moves together from start to finish.

**Key Differences from Staged Mode:**
- No artificial stage boundaries
- Base and arm move together throughout
- Optional ramp-up phase for smooth start
- Simpler execution flow
- Continuous obstacle avoidance enforcement

### Holistic Mode Architecture

```
trackReferenceTrajectory('Mode', 'holistic', ...)
    │
    ├─ Load JSON trajectory (148 waypoints)
    ├─ Create robot model
    ├─ Setup environment (obstacles, constraints)
    │
    ├─ (Optional) Generate ramp-up trajectory
    │   └─ generateHolisticRamp() → smooth acceleration
    │
    └─ Execute based on mode:
        │
        ├─ ppForIk Mode:
        │   ├─ Pass 1: Reference IK
        │   ├─ Pass 2: Simulate chassis controller
        │   └─ Pass 3: Final IK with fixed base
        │
        └─ pureIk Mode:
            └─ Single-pass full-body IK
```

### Holistic Mode: pureIk

**Purpose:** Direct full-body IK tracking with no controller simulation

**Data Flow:**
```
Input:
  q0                [9×1]   Initial configuration
  trajStruct        Struct  Full trajectory (148 waypoints)
  velLimits         Struct  Velocity constraints
  
Process:
  1. Create bundleSingle (standard GIK solver)
     - Pose constraint for end effector
     - Joint bounds
     - Distance constraints (obstacle avoidance)
     
  2. Run runTrajectoryControl()
     - Streams all 148 waypoints through IK
     - Applies velocity limits via clamping
     - Logs iterations, solve times, errors
     
  3. Base motion emerges from IK solution
     - No explicit controller
     - Solver determines optimal base trajectory
     - May violate kinematic feasibility
  
Output:
  log.qTraj         [9×N]   Full joint trajectory
  log.eePositions   [3×N]   Actual EE positions (FK)
  log.targetPositions [3×N] Desired EE positions
  log.positionError [3×N]   Tracking errors
  log.mode          'holistic'
  log.simulationMode 'pureIk'
  log.execBaseStates [N×3]  Base trajectory (same as qTraj base)
  log.cmdLog        Empty   (no controller commands)
```

**Characteristics:**
- **Fastest execution** - Single IK pass
- **Simplest pipeline** - No controller simulation
- **May be kinematically infeasible** - Base motion not validated against chassis constraints
- **Good for:** Quick testing, algorithm development, ideal trajectory generation

### Holistic Mode: ppForIk (Pure Pursuit for IK)

**Purpose:** Kinematically feasible base motion using chassis controller

**Three-Pass Architecture:**

#### Pass 1: Generate Reference Base Path

```
Input:
  q0          [9×1]   Initial configuration
  trajStruct  Struct  Full trajectory
  
Process:
  1. Create bundleRef (standard GIK solver)
  2. Run runTrajectoryControl() → logRef
  3. Extract base trajectory:
     baseReference = logRef.qTraj(baseIdx, :)'
     
Output:
  logRef.qTraj       [9×N]   Reference solution
  baseReference      [N×3]   (x, y, theta) reference path
```

#### Pass 2: Simulate Chassis Controller

```
Input:
  baseReference   [N×3]   Reference base path from Pass 1
  chassisParams   Struct  Chassis constraints & controller gains
  
Process:
  1. Prepare path for follower
     - Interpolate to desired spacing
     - Resample for controller frequency
     
  2. Run simulateChassisController()
     - Unified chassis controller (heading + pure pursuit)
     - Generates (Vx, Wz) commands
     - Integrates to get executed trajectory
     - Respects velocity, acceleration limits
     - Respects wheel speed limits
     
  3. Validate execution
     - Check against kinematic constraints
     - Ensure goal reached
  
Output:
  simRes.poses      [N×3]   Executed base trajectory
  simRes.commands   [N×2]   (Vx, Wz) velocity commands
  simRes.wheelSpeeds [N×2]  Left/right wheel speeds
  simRes.status     [N×1]   Controller status per step
```

#### Pass 3: Final IK with Fixed Base

```
Input:
  baseExecutedFull  [N×3]   Executed base from Pass 2
  trajStruct        Struct  Full trajectory
  
Process:
  1. Resample base path to match trajectory length
  2. Create bundleFinal (GIK solver)
  3. Run runTrajectoryControl() with:
     FixedJointTrajectory.Indices = baseIdx (1:3)
     FixedJointTrajectory.Values = executedBase
     
  4. IK solves for arm only
     - Base follows prescribed trajectory
     - Arm tracks end-effector targets
     - Ensures achievable poses
  
Output:
  log.qTraj                [9×N]   Full trajectory
  log.referenceInitialIk   logRef  Pass 1 log
  log.purePursuit          Struct  Pass 2 simulation
  log.execBaseStates       [N×3]   Executed base path
  log.referenceBaseStates  [N×3]   Reference base path
  log.cmdLog               Table   (time, Vx, Wz) commands
```

**Why Three Passes?**
1. **Pass 1**: Get ideal base trajectory from IK (may not be feasible)
2. **Pass 2**: Validate against chassis dynamics, get realistic trajectory
3. **Pass 3**: Ensure arm can track EE with actual base motion

**Characteristics:**
- **Kinematically feasible** - Respects chassis constraints
- **Realistic simulation** - Matches actual robot capabilities
- **Slower execution** - Three IK passes + simulation
- **Good for:** Validation, deployment preparation, realistic testing

### Optional Holistic Ramp-Up

**Purpose:** Smooth acceleration from rest to trajectory start

**Generated by:** `generateHolisticRamp(robot, bundle, q0, T1, options)`

**Process:**
```
Input:
  q0              [9×1]   Initial configuration
  T1              [4×4]   First trajectory pose target
  MaxLinearSpeed  double  Max base linear velocity (m/s)
  MaxYawRate      double  Max base angular velocity (rad/s)
  MaxJointSpeed   double  Max arm joint velocity (rad/s)
  SampleTime      double  Control timestep (s)
  
Algorithm:
  1. Compute required motion:
     - Base translation distance
     - Base rotation angle
     - Arm joint displacements
     
  2. Compute ramp durations:
     tBase = max(translation/MaxLinearSpeed, rotation/MaxYawRate)
     tArm = max(joint_displacement/MaxJointSpeed)
     tRamp = max(tBase, tArm)
     
  3. Generate trajectory:
     - Interpolate base pose (linear + SLERP)
     - Interpolate arm joints (cubic)
     - NumSteps = ceil(tRamp / SampleTime)
     
  4. Solve IK for each ramp step
     - Refine poses via IK
     - Ensure consistency
  
Output:
  rampInfo.Poses              [4×4×N] SE(3) ramp poses
  rampInfo.EndEffectorPositions [3×N] EE positions
  rampInfo.NumSteps           int     Ramp length
```

**Usage:**
```matlab
log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'UseHolisticRamp', true, ...
    'RampMaxLinearSpeed', 1.5, ...
    'RampMaxYawRate', 3.0, ...
    'RampMaxJointSpeed', 1.0);
```

**Stored in log:**
```matlab
log.ramp        % Ramp info struct
log.rampSamples % Number of ramp steps
```

### Holistic Mode: Velocity Limiting

**Applied in:** `runTrajectoryControl`

**Velocity Limits Structure:**
```matlab
velLimits = struct(
    'BaseIndices',     [1 2 3],        % Base joint indices
    'ArmIndices',      [4 5 6 7 8 9],  % Arm joint indices
    'MaxLinearSpeed',  1.5,            % m/s
    'MaxYawRate',      3.0,            % rad/s
    'MaxJointSpeed',   1.0             % rad/s
);
```

**Clamping Process:**
```matlab
% For each IK solution qCandidate:
for step k:
    1. Compute velocities: v = (qCandidate - qPrev) / dt
    
    2. Clamp base velocities:
       vx_world, vy_world = decompose planar velocity
       v_linear = sqrt(vx² + vy²)
       if v_linear > MaxLinearSpeed:
           scale = MaxLinearSpeed / v_linear
           qCandidate(1:2) = qPrev(1:2) + scale * (qCandidate(1:2) - qPrev(1:2))
       
       w_z = (qCandidate(3) - qPrev(3)) / dt
       if |w_z| > MaxYawRate:
           qCandidate(3) = qPrev(3) + sign(w_z) * MaxYawRate * dt
    
    3. Clamp arm velocities:
       for each arm joint:
           v_joint = (qCandidate(i) - qPrev(i)) / dt
           if |v_joint| > MaxJointSpeed:
               qCandidate(i) = qPrev(i) + sign(v_joint) * MaxJointSpeed * dt
    
    4. Accept clamped solution: q = qCandidate
```

**Important:** Velocity limits are applied **after** IK solve, not during. This ensures feasibility but may increase tracking error.

### Holistic vs Staged: Comparison

| Aspect | Holistic pureIk | Holistic ppForIk | Staged ppForIk |
|--------|----------------|------------------|----------------|
| **IK Passes** | 1 | 3 | 3 (per stage) |
| **Base Motion** | Emergent from IK | Controller-driven | Hybrid A* + Controller |
| **Feasibility** | Not guaranteed | Guaranteed | Guaranteed |
| **Obstacle Avoidance** | Continuous | Continuous | Stages A/B: No, Stage C: Yes |
| **Execution Time** | Fastest (~5-10s) | Medium (~30-45s) | Slowest (~45-60s) |
| **Tracking Quality** | Good | Excellent | Excellent |
| **Base Path Quality** | May have kinks | Smooth, feasible | Very smooth (planned) |
| **Complexity** | Lowest | Medium | Highest |
| **Use Case** | Quick testing | Validation | Deployment preparation |

### Holistic Mode: Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│  Entry: run_environment_compare.m or custom script          │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  trackReferenceTrajectory('Mode', 'holistic', ...)          │
│  ├─ Load 1_pull_world_scaled.json (148 waypoints)           │
│  ├─ Create robot model (9 DOF)                              │
│  ├─ Setup environment (base home, obstacles)                │
│  ├─ Add floor discs → distance constraints                  │
│  └─ Create GIK solver bundle                                │
└────────────────────┬────────────────────────────────────────┘
                     │
          ┌──────────┴──────────┐
          │                     │
          ▼                     ▼
    pureIk Mode          ppForIk Mode
          │                     │
          │              ┌──────┴──────┐
          │              │             │
          │              ▼             ▼
          │      (Optional)     Pass 1: Ref IK
          │   generateHolisticRamp  │
          │              │         runTrajectoryControl
          │              │         (bundleRef)
          │              │              │
          │              │              ▼
          │              │      Pass 2: Simulate
          │              │      simulateChassisController
          │              │       - Pure pursuit
          │              │       - Generate (Vx, Wz)
          │              │       - Integrate base path
          │              │              │
          │              │              ▼
          │              │      Pass 3: Final IK
          │              │      runTrajectoryControl
          │              │      (bundleFinal + FixedBase)
          │              │              │
          ▼              ▼              ▼
    runTrajectoryControl     Prepend ramp + Merge passes
    (bundleSingle)                     │
          │                            │
          └────────────┬───────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  Unified Log Structure                                       │
│  ├─ mode: 'holistic'                                        │
│  ├─ simulationMode: 'pureIk' or 'ppForIk'                  │
│  ├─ qTraj: [9×N] full trajectory                           │
│  ├─ eePositions, targetPositions, positionError            │
│  ├─ velocityLimits (ramp settings)                         │
│  ├─ (if ramp) ramp: info, rampSamples                      │
│  └─ (if ppForIk) purePursuit, referenceInitialIk, cmdLog   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
         Save to results/<timestamp>/
         - log_holistic_pureIk.mat or
         - log_holistic_ppForIk.mat
```

### Holistic Mode: Log Structure

```matlab
log_holistic = struct(
    % Mode identification
    'mode',                'holistic',
    'simulationMode',      'pureIk' or 'ppForIk',
    
    % Core trajectory
    'qTraj',               [9×N] joint angles
    'timestamps',          [1×N] time sequence
    'successMask',         [1×N] solver success flags
    
    % End-effector tracking
    'eePositions',         [3×N] actual EE (FK)
    'targetPositions',     [3×N] desired EE (JSON)
    'eePoses',             [4×4×N] actual SE(3) poses
    'positionError',       [3×N] tracking errors
    'positionErrorNorm',   [1×N] error magnitudes
    
    % Solver diagnostics
    'solutionInfo',        {1×N} solver details
    'iterations',          [1×N] iterations per step
    'solveTime',           [1×N] solve time per step
    'solverSummary',       Aggregated statistics
    
    % Base velocity estimate (if enabled)
    'baseVelocityEstimate', Struct with velocities
    
    % Velocity limits (ramp settings)
    'velocityLimits',      Struct with max speeds
    '  .MaxLinearSpeed',   double
    '  .MaxYawRate',       double
    '  .MaxJointSpeed',    double
    '  .SampleTime',       double
    
    % Ramp info (if UseHolisticRamp=true)
    'ramp',                Ramp info struct
    'rampSamples',         Number of ramp steps
    
    % ppForIk specific data
    'referenceInitialIk',  logRef (Pass 1 log)
    'purePursuit',         Struct with simulation
    '  .referencePath',    [N×3] reference base
    '  .simulation',       simRes struct
    '  .executedPath',     [N×3] executed base
    '  .commands',         [N×2] (Vx, Wz)
    '  .wheelSpeeds',      [N×2] left/right
    '  .status',           [N×1] controller status
    'execBaseStates',      [N×3] executed base path
    'referenceBaseStates', [N×3] reference base path
    'cmdLog',              Table (time, Vx, Wz)
    
    % Environment
    'floorDiscs',          Obstacle definitions
    'distanceSpecs',       Distance constraints
    'environment',         Environment config
    'chassisParams',       Chassis parameters
    'chassisProfile',      Profile name
);
```

### Holistic Mode: Entry Points

**Main Entry:**
```matlab
% Via trackReferenceTrajectory
log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', ...  % or 'pureIk'
    'RateHz', 10, ...
    'UseHolisticRamp', true, ...
    'RampMaxLinearSpeed', 1.5, ...
    'RampMaxYawRate', 3.0, ...
    'RampMaxJointSpeed', 1.0);
```

**Via Batch Comparison:**
```matlab
% run_environment_compare.m
% Runs both holistic and staged modes
run_environment_compare
```

**Custom Execution:**
```matlab
% Custom environment setup
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.20;

log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', ...
    'EnvironmentConfig', env, ...
    'MaxIterations', 150, ...
    'ChassisProfile', 'wide_track');
```

### Holistic Mode: Performance Characteristics

**pureIk Mode:**
- Single IK pass: ~5-10 seconds total
- Mean solve time per step: ~0.03-0.05s (at 150 iter cap)
- EE tracking error: Typically <5mm mean, <20mm max
- No controller overhead
- Base motion may violate kinematic constraints

**ppForIk Mode:**
- Pass 1 (Reference): ~5-10 seconds
- Pass 2 (Simulation): ~2-5 seconds
- Pass 3 (Final IK): ~5-10 seconds
- Total: ~12-25 seconds
- EE tracking error: Typically <8mm mean, <30mm max
- Base motion kinematically feasible
- Matches real robot capabilities

**With Ramp:**
- Adds ~50-100 steps to beginning
- Additional ~1-2 seconds execution time
- Smoother start from rest
- Better real-world performance

---

## Mode Selection Guide

### When to Use Staged Mode

**Advantages:**
- Explicit base path planning (Hybrid A*)
- Obstacle avoidance in base planning
- Very smooth base trajectories
- Good for complex environments
- Mimics real deployment strategy

**Disadvantages:**
- Slowest execution
- Most complex pipeline
- Stage boundaries visible in motion
- Requires careful tuning

**Use Cases:**
- Deployment preparation
- Complex obstacle environments
- When base path quality is critical
- Research on staged control

### When to Use Holistic pureIk

**Advantages:**
- Fastest execution
- Simplest pipeline
- Good for algorithm development
- Ideal trajectory generation

**Disadvantages:**
- Base motion may be infeasible
- No controller validation
- May violate velocity limits

**Use Cases:**
- Quick testing and iteration
- Algorithm development
- Generating reference trajectories
- Debugging IK solver

### When to Use Holistic ppForIk

**Advantages:**
- Kinematically feasible
- Realistic simulation
- Validates controller
- Good tracking quality

**Disadvantages:**
- Slower than pureIk
- More complex than pureIk
- Three-pass overhead

**Use Cases:**
- Validation testing
- Pre-deployment verification
- Realistic performance evaluation
- Controller tuning

### Summary Comparison Table

| Mode | Stages | IK Passes | Base Motion | Feasibility | Speed | Use Case |
|------|--------|-----------|-------------|-------------|-------|----------|
| **Staged ppForIk** | 3 (A/B/C) | 9 total | Planned | Guaranteed | Slowest | Deployment |
| **Holistic ppForIk** | 1 | 3 | Controller | Guaranteed | Medium | Validation |
| **Holistic pureIk** | 1 | 1 | Emergent | Not guaranteed | Fastest | Development |

---

## Animation from Logs Pipeline

### Overview

Animation can be generated:
1. **During simulation** - via `saveRunArtifacts()`
2. **Post-simulation** - via `regenerate_animations_from_logs.m`

### Data Flow: Log → Animation

```
Input:
  log_staged_ppForIk.mat  (or pureIk)
  
Load:
  log = load('results/.../log_staged_ppForIk.mat').log;
  
Extract:
  qTraj          [9×N]  Joint angles over time
  stageLogs      Struct  Per-stage breakdown
  timestamps     [1×N]   Time sequence
  
Process:
  1. Sample trajectory (SampleStep, e.g., every 5th frame)
  2. Extract base trajectory: basePose = qTraj(baseIdx,:)'
  3. Extract arm trajectory: armTrajectory = qTraj(armIdx,:)'
  
  4. Gather reference paths:
     a. Stage C desired path: stageLogs.stageC.targetPositions
     b. Stage C actual path: stageLogs.stageC.eePositions
     c. Pure pursuit reference: log.purePursuit.referencePath (if exists)
  
  5. Prepare visualization options:
     options.StageBoundaries = [NA, NA+NB]
     options.StageLabels = ["Stage A", "Stage B", "Stage C"]
     options.Obstacles = log.floorDiscs
     options.StageCReferenceEEPath = sampled reference
  
Call:
  gik9dof.animateStagedWithHelper(log, ...
      'SampleStep', 5, ...
      'FrameRate', 20, ...
      'ExportVideo', 'output.mp4', ...
      'HelperOptions', options);
  
Animation:
  gik9dof.viz.animate_whole_body(robot, armJointNames, ...
      armTrajectory, armTimes, basePose, baseTimes, eePoses, options)
  
  Loop (for each frame k):
    1. Interpolate base pose at armTimes(k)
    2. Set robot configuration: q = [base(k); arm(k)]
    3. Forward kinematics: T_ee = getTransform(robot, q, 'left_gripper_link')
    4. Render robot: show(robot)
    5. Overlay obstacles (inflated discs with halos)
    6. Draw reference paths (ribbons/lines)
    7. Draw actual EE path (trail)
    8. Plot target EE marker (red dot at stageCRef(k))
    9. Update stage label
    10. Capture frame → writeVideo()
  
Output:
  output.mp4  Video file
```

### Key Visualization Elements

**1. Robot Rendering:**
- Forward kinematics from qTraj
- STL meshes attached (if available)
- Transparency: arm = 0.6, chassis = 0.35

**2. Obstacles:**
- Floor discs rendered as cylinders
- Inflated by safety margin
- Semi-transparent halos

**3. Paths:**
- **Desired EE Path** (full trajectory): Blue ribbon
- **Stage C Reference EE Path** (target for tracking): Red markers/dots
- **Actual EE Path** (from FK): Green trail
- **Base Path** (reference): Dashed line
- **Base Path** (executed): Solid line

**4. Stage Labels:**
- Text annotations with current stage
- Auto-positioned based on stage boundaries

**5. Dual Views:**
- **Perspective view**: 3D visualization
- **Top view**: 2D bird's-eye (XY plane)

### Animation Scripts Comparison

| Script | Purpose | Primary Use | Status |
|--------|---------|-------------|--------|
| **animateStagedWithHelper.m** | Wrapper for staged logs | Production | ✅ Active |
| **animateHolisticWithHelper.m** | Wrapper for holistic logs | Production | ✅ Active |
| **+viz/animate_whole_body.m** | Core animator | Called by wrappers | ✅ Active |
| **animateStagedLegacy.m** | Old staged animator | Legacy | ⚠️ Deprecated |
| **animateTrajectory.m** | Simple visualization | Quick checks | ✅ Utility |
| **regenerate_animations_from_logs.m** | Batch regeneration | Post-processing | ✅ Active |
| **generate_*.m** (5 files) | Various generators | Ad-hoc testing | ⚠️ Overlapping |

---

## Helper Functions & Utilities

### Configuration & Setup Helpers

```matlab
% Configuration tools
tools = gik9dof.configurationTools(robot);
q_home = tools.home();              % Home configuration
q_col = tools.column(q_struct);     % Struct → column vector
q_struct = tools.struct(q_col);     % Column → struct array

% Path resolution
absPath = gik9dof.internal.resolvePath('relative/path.json');
projectRoot = gik9dof.internal.projectRoot();

% Results management
resultsDir = gik9dof.internal.createResultsFolder('my_experiment');
% Creates: results/YYYYMMDD_HHMMSS_my_experiment/
```

### Velocity Estimation

```matlab
% Base velocity estimator (backward differences)
estimator = gik9dof.internal.VelocityEstimator();
estimator.update([x, y, theta], t);  % Add state at time t
vel = estimator.estimate();          % Returns struct with velocities

% Fields:
vel.world      % [vx_world, vy_world] (m/s)
vel.worldRaw   % Raw before projection
vel.robot      % [vx_robot, 0] (diff-drive constraint)
vel.omega      % yaw rate (rad/s)
vel.method     % '5-point', '3-point', or '2-point'
```

**Adaptive Backward Differences:**
- **5-point stencil** (O(h⁴)): When 5+ samples available, highest accuracy
- **3-point stencil** (O(h²)): When 3-4 samples available  
- **2-point stencil** (O(h)): When only 2 samples available

**Features:**
- Maintains up to 5 recent base states in circular buffer
- Yaw angle unwrapped before differentiation
- Rejects degenerate time steps
- Projects world-frame velocity to chassis frame (enforces v_y = 0 for differential drive)
- Logged in `log.baseVelocityEstimate` when base joints present

**Usage Context:**
- `runTrajectoryControl` instantiates when `VelocityLimits.BaseIndices` provided
- Stage B command CSVs fall back to estimator when pure pursuit inactive
- Critical for ROS/firmware integration (provides filtered velocity signals)

### Path Processing

```matlab
% Prepare path for follower
path = gik9dof.control.preparePathForFollower(rawPath, ...
    'InterpSpacing', 0.05, ...        % Interpolation density
    'WaypointSpacing', 0.15, ...      % Waypoint extraction
    'PathBufferSize', 30.0);          % Lookahead buffer

% Clothoid refinement
[smoothed, info] = gik9dof.control.rsClothoidRefine(path, params);
% Returns: clothoid-smoothed path + fitting statistics
```

### Evaluation Utilities

```matlab
% Log evaluation
metrics = gik9dof.evaluateLog(log);
% Returns: EE error stats, success rate, solver metrics

% Path smoothness
smoothness = gik9dof.evaluatePathSmoothness(path);
% Returns: curvature, jerk, smoothness metrics

% Base ribbon metrics (Stage B/C paths)
metrics = gik9dof.computeBaseRibbonMetrics(baseStates);
% Returns: curvature histogram, cusps, smoothness

% Collision intrusion
intrusion = gik9dof.evaluateCollisionIntrusion(robot, qTraj, obstacles);
% Returns: minimum distances, violations

% Chassis constraints
feasible = gik9dof.evaluateChassisConstraints(commands, chassisParams);
% Returns: velocity/wheel speed limit violations
```

### Plotting & Visualization

```matlab
% Generate standard plots
gik9dof.generateLogPlots(log, 'OutputDir', resultsDir);
% Saves: arm joints, chassis velocity, EE error plots

% Plot trajectory
gik9dof.plotTrajectoryLog(log);
% Interactive figure with paths and errors

% External plots (for reports)
gik9dof.generateExternalPlots(log, 'Format', 'png');
```

---

## Redundancies, Orphans & Deprecated Files

### ✅ Active & Essential Files

**Core Library (`+gik9dof/`):** All 44 files actively used

**Entry Points:** 
- `run_staged_reference.m` ✅
- `run_environment_compare.m` ✅
- `run_fresh_sim_with_animation.m` ✅
- `run_parametric_study.m` ✅
- `run_comprehensive_chassis_study.m` ✅

**Configuration:**
- `config/chassis_profiles.yaml` ✅
- `1_pull_world_scaled.json` ✅

### ⚠️ Overlapping Functionality

#### Animation Generators (5 files - Consolidate)

| File | Purpose | Unique Feature | Recommendation |
|------|---------|----------------|----------------|
| `generate_animation_from_saved_log.m` | Single log → MP4 | Basic wrapper | **Merge into regenerate_animations_from_logs.m** |
| `generate_comprehensive_animations.m` | Batch processing | Multiple logs | **Keep as is** OR merge |
| `generate_final_animation.m` | "Final" output | Specific settings | **Remove** (use regenerate with options) |
| `generate_parametric_animations.m` | Parameter study | Handles sweep results | **Keep** (specialized) |
| `generate_sweep_animations.m` | Sweep animations | Similar to parametric | **Merge** with parametric |

**Recommendation:** Consolidate to 2 scripts:
1. `regenerate_animations_from_logs.m` - General purpose, flexible options
2. `generate_parametric_animations.m` - Specialized for parameter sweeps

#### Test Scripts (10+ files - Organize)

**Animation Tests:**
- `test_animation_sync_fix.m`
- `test_single_animation_sync.m`
- `test_stage_sync_fix.m`
- `test_stagec_path_fix.m`

**Recommendation:** Archive these after confirming fixes are stable. Move to `tests/archive/`.

**Algorithm Tests:**
- `test_complete_fix.m`
- `test_comprehensive_evaluation.m`
- `test_enhanced_logging.m`
- `test_issue_fixes.m`
- `test_parameter_sweep.m`
- `test_tuned_parameters.m`

**Recommendation:** Move to `tests/` directory. Create `tests/README.md` explaining each test's purpose.

#### Debug Scripts (4 files - Archive)

- `debug_animation_sync.m`
- `debug_sampling_mismatch.m`
- `debug_stage_boundaries.m`
- `debug_stagec_ee_path.m`

**Status:** These solved specific bugs documented in:
- `ANIMATION_SYNC_FIX.md`
- `ANIMATION_SYNC_FIX_FINAL.md`
- `COMPLETE_TIMING_FIX.md`
- `STAGEC_PATH_FIX.md`

**Recommendation:** Move to `debug/archive/` with reference to fix documentation.

### ❌ Temporary Files (Delete)

**MAT Files:**
- `tmp_compare.mat`
- `tmp_json.mat`
- `tmp_pipeline.mat`

**Scripts:**
- `tmp_regen_staged_only.m`
- `tmp_regen.m`
- `temp_ref_rs_refine.m`

**Recommendation:** Delete immediately. Add to `.gitignore`:
```
tmp_*.m
tmp_*.mat
temp_*.m
```

### ⚠️ Legacy/Deprecated Files

**Animation:**
- `animateStagedLegacy.m` - Replaced by `animateStagedWithHelper.m` + `animate_whole_body.m`

**Comparison:**
- `compareAnimationsLegacy.m` - Old comparison function

**Recommendation:** 
1. Add deprecation warning to these functions
2. Update all call sites to use new functions
3. Move to `deprecated/` directory after 1 release cycle

### 📊 Documentation - Consolidate

**Current State:** 15+ markdown files, some overlapping

**Overlap Analysis:**
- `DATA_FLOW_ANALYSIS.m` + `projectDiagnosis.md` (this file) - **Merge insights**
- `PROJECT_OVERVIEW.md` + `GIK_SETUP_OVERVIEW.md` - **Consolidate**
- `ANIMATION_SYNC_FIX.md` + `ANIMATION_SYNC_FIX_FINAL.md` + `COMPLETE_TIMING_FIX.md` - **Archive** (bugs fixed)
- `ALGORITHM_IMPROVEMENT_PLAN.md` - **Keep** (roadmap)
- `SIMULATION_WORKFLOW_GUIDE.md` - **Keep** (operational guide)
- `PROJECT_STATUS_SUMMARY.md` - **Keep** (status tracking)
- `HANDOVER.md` - **Keep** (handover notes)

**Recommended Structure:**
```
docs/
├── README.md                          # Entry point
├── SETUP_GUIDE.md                     # Installation & setup
├── USER_GUIDE.md                      # How to run simulations
├── ARCHITECTURE.md                    # System design (merge GIK_SETUP + PROJECT_OVERVIEW)
├── DATA_FLOW.md                       # Data flow details (this document)
├── API_REFERENCE.md                   # Function documentation
├── WORKFLOW_GUIDE.md                  # Operational workflows
├── STATUS.md                          # Current status & achievements
└── archive/
    ├── bug_fixes/                     # Old bug fix docs
    └── legacy/                        # Deprecated guides
```

---

## Key Insights & Recommendations

### System Architecture Insights

#### ✅ Strengths

1. **Well-Organized Package Structure**
   - Clear namespace (`+gik9dof/`)
   - Logical subsystems (`+control/`, `+viz/`, `+internal/`)
   - Minimal global namespace pollution

2. **Flexible Configuration System**
   - YAML-based chassis profiles
   - Runtime overrides supported
   - Easy to add new profiles

3. **Comprehensive Logging**
   - Per-stage logs preserved
   - Enhanced diagnostics (Phase 2)
   - Full simulation replay capability

4. **Clean Data Flow**
   - Staged pipeline clearly separated
   - Mode selection explicit
   - Data objects well-structured

5. **Animation Infrastructure**
   - Log → Animation decoupling
   - Regeneration without re-simulation
   - Multiple view options

#### ⚠️ Areas for Improvement

1. **Root-Level Script Proliferation**
   - 38 scripts at root level
   - 20+ test/debug scripts
   - Unclear which are active vs. deprecated

2. **Documentation Fragmentation**
   - 15+ markdown files
   - Overlapping content (data flow explained 3 times)
   - No single entry point

3. **Animation Script Redundancy**
   - 5 `generate_*.m` scripts with overlapping purposes
   - Legacy functions still present
   - Unclear which to use for what

4. **Configuration Split**
   - Some params in YAML (`chassis_profiles.yaml`)
   - Some hardcoded in functions (`environmentConfig.m`)
   - Some passed as options
   - Difficult to track all tuneable parameters

5. **Temporary Files Not .gitignored**
   - `tmp_*.m`, `tmp_*.mat` committed
   - Clutters repository

### Recommendations

#### Priority 1: Immediate Cleanup

1. **Delete Temporary Files**
   ```bash
   rm tmp_*.m tmp_*.mat temp_*.m
   ```

2. **Update .gitignore**
   ```
   # Temporary files
   tmp_*
   temp_*
   scratch_*
   
   # Generated data
   *.mat
   *.mp4
   *.avi
   results/
   
   # MATLAB artifacts
   *.asv
   *.mex*
   slprj/
   ```

3. **Archive Completed Debug Scripts**
   ```bash
   mkdir -p debug/archive
   mv debug_*.m debug/archive/
   # Add README.md explaining each was for specific bugs
   ```

#### Priority 2: Organization

1. **Create tests/ Directory**
   ```bash
   mkdir -p tests/integration tests/unit
   mv test_*.m tests/integration/
   # Add tests/README.md
   ```

2. **Consolidate Animation Scripts**
   - Keep: `regenerate_animations_from_logs.m`
   - Keep: `generate_parametric_animations.m`
   - Remove: `generate_animation_from_saved_log.m`, `generate_final_animation.m`
   - Merge: `generate_comprehensive_animations.m` + `generate_sweep_animations.m`

3. **Organize Documentation**
   ```bash
   mkdir -p docs/archive docs/archive/bug_fixes
   # Follow structure outlined above
   ```

#### Priority 3: API Improvements

1. **Unify Configuration**
   - Move all environment config to YAML (obstacles, margins, etc.)
   - Single configuration file: `config/simulation_config.yaml`
   - Profiles for different scenarios

2. **Consolidate Entry Points**
   - Single main script: `run_simulation.m`
   - Sub-commands: `--mode staged`, `--execution ppForIk`, etc.
   - Example:
     ```matlab
     run_simulation('Mode', 'staged', 'Execution', 'ppForIk', ...
         'Profile', 'wide_track', 'Animate', true);
     ```

3. **Deprecation Warnings**
   - Add warnings to legacy functions
   - Point to replacements
   - Plan removal timeline

#### Priority 4: Documentation

1. **Create docs/README.md** (Entry Point)
   ```markdown
   # gikWBC9DOF Documentation
   
   ## Quick Start
   - [Setup Guide](SETUP_GUIDE.md)
   - [User Guide](USER_GUIDE.md)
   
   ## Reference
   - [Architecture](ARCHITECTURE.md)
   - [Data Flow](DATA_FLOW.md)
   - [API Reference](API_REFERENCE.md)
   
   ## Operations
   - [Workflow Guide](WORKFLOW_GUIDE.md)
   - [Status & Achievements](STATUS.md)
   ```

2. **Consolidate Overlapping Docs**
   - Merge data flow explanations
   - Single architecture document
   - Archive old bug fix docs

3. **Generate API Reference**
   - Use MATLAB's `help` system
   - Generate HTML documentation
   - Link from main README

### Data Flow - Critical Observations

#### Staged Mode: Three-Pass Architecture (ppForIk)

The **ppForIk** execution mode uses an innovative three-pass approach in Stage C:

**Pass 1: Generate Reference**
- Pure IK solution (no controller simulation)
- Provides ideal base trajectory
- May not be kinematically feasible

**Pass 2: Simulate Controller**
- Realistic chassis dynamics
- Respects velocity/acceleration limits
- Generates executable base trajectory

**Pass 3: IK with Fixed Base**
- Arm IK with actual base motion
- Ensures achievable poses
- **Critical:** Base from Pass 2, not Pass 1

**Why This Matters:**
- Animation uses **qTraj from Pass 3** (actual robot motion)
- Red dot shows **targetPositions** (desired EE from JSON)
- Gap between robot EE and red dot = **real tracking error**
- System is **truthful** - not an artifact

#### Animation Synchronization

**Fixed in Recent Updates:**
- Stage C reference path properly sampled
- Frame indices match animation steps
- No more drift between robot and reference markers

**Key Insight:**
- `eePathStageCRef` must be sampled at same rate as `qTraj`
- Pass `eePoses(:, 1:SampleStep:end)` to animator
- Not full-resolution path

### Performance Characteristics

**From Recent Runs:**
- Stage A: ~50 steps, <5 seconds
- Stage B (pureHyb): ~50 steps, 1-3 seconds (planning) + <5 seconds (simulation)
- Stage C (ppForIk): ~148 waypoints, 10-15 seconds per pass (3 passes total)

**Bottlenecks:**
1. IK solver iterations (hitting 150 cap frequently)
2. Reeds-Shepp shortcut iterations (if enabled)
3. Animation rendering (real-time playback)

**Optimizations Applied:**
- Iteration cap reduced to 150 (from 1500)
- Solver tolerances tuned
- Chassis parameters optimized (Phase 1)
- Enhanced logging adds <5% overhead (Phase 2)

---

## Conclusion

### Project Maturity

**Current State:** Well-structured, feature-complete system with some organizational debt

**Technical Quality:** High
- Robust error handling
- Comprehensive logging
- Flexible architecture
- Good separation of concerns

**Documentation Quality:** Moderate
- Extensive but fragmented
- Some duplication
- Needs consolidation and entry points

**Code Organization:** Moderate
- Core library excellent
- Root-level scripts need organization
- Some redundant test scripts

### Next Steps for Maintainability

1. **Immediate** (1-2 hours)
   - Delete temporary files
   - Update .gitignore
   - Archive completed debug scripts

2. **Short-term** (1 day)
   - Create tests/ directory
   - Consolidate animation scripts
   - Organize documentation structure

3. **Medium-term** (2-3 days)
   - Unify configuration system
   - Add deprecation warnings
   - Generate API reference
   - Create consolidated docs

4. **Long-term** (1 week)
   - Consider single entry point script
   - Full YAML-based configuration
   - Automated testing framework
   - CI/CD for regression testing

### For New Team Members

**Start Here:**
1. Read `projectDiagnosis.md` (this file) - System overview
2. Read `PROJECT_STATUS_SUMMARY.md` - Current achievements
3. Read `SIMULATION_WORKFLOW_GUIDE.md` - How to run sims
4. Explore `+gik9dof/` package - Core library

**To Run A Simulation:**
```matlab
% Simple staged run with defaults
result = gik9dof.runStagedReference();

% Custom parameters
result = gik9dof.runStagedReference(...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'UseStageBHybridAStar', true, ...
    'ChassisProfile', 'wide_track');

% Results saved to: results/<timestamp>_staged_reference/
```

**To Regenerate Animation:**
```matlab
regenerate_animations_from_logs('results/<your_folder>');
```

**To Tune Parameters:**
1. Edit `config/chassis_profiles.yaml` for chassis params
2. Pass overrides via `ChassisOverrides` option
3. Edit `environmentConfig.m` for environment setup

---

## Appendix: File Reference

### Complete File Inventory

**Root-Level Scripts (38 total)**

*Active Entry Points (5):*
- run_staged_reference.m
- run_environment_compare.m
- run_fresh_sim_with_animation.m
- run_parametric_study.m
- run_comprehensive_chassis_study.m

*Animation Generation (5):*
- regenerate_animations_from_logs.m ✅
- generate_animation_from_saved_log.m ⚠️
- generate_comprehensive_animations.m ⚠️
- generate_final_animation.m ⚠️
- generate_parametric_animations.m ✅
- generate_sweep_animations.m ⚠️

*Test Scripts (10):*
- test_animation_sync_fix.m
- test_complete_fix.m
- test_comprehensive_evaluation.m
- test_enhanced_logging.m
- test_issue_fixes.m
- test_parameter_sweep.m
- test_single_animation_sync.m
- test_stage_sync_fix.m
- test_stagec_path_fix.m
- test_tuned_parameters.m

*Debug Scripts (4):*
- debug_animation_sync.m
- debug_sampling_mismatch.m
- debug_stage_boundaries.m
- debug_stagec_ee_path.m

*Analysis Scripts (5):*
- analyze_all_tests.m
- analyze_stage_collisions.m
- check_reference_quality.m
- compare_test_configs.m
- investigate_cusps.m

*Temporary (6):*
- tmp_regen_staged_only.m ❌
- tmp_regen.m ❌
- temp_ref_rs_refine.m ❌
- tmp_compare.mat ❌
- tmp_json.mat ❌
- tmp_pipeline.mat ❌

*Utilities (3):*
- export_all_commands.m
- view_json_waypoints.m
- plotJsonPath.m → matlab/plotJsonPath.m (duplicate)

**Core Library (+gik9dof/) - 44 files**

*See detailed tables above for complete breakdown*

**Documentation - 15+ files**

*Active:*
- projectDiagnosis.md (this file)
- PROJECT_STATUS_SUMMARY.md
- SIMULATION_WORKFLOW_GUIDE.md
- ALGORITHM_IMPROVEMENT_PLAN.md
- HANDOVER.md
- diary.md
- guideline.md

*Archive Candidates:*
- ANIMATION_SYNC_FIX.md
- ANIMATION_SYNC_FIX_FINAL.md
- COMPLETE_TIMING_FIX.md
- STAGEC_PATH_FIX.md
- ANIMATION_FIX_SUMMARY.md
- ANIMATION_GENERATION_GUIDE.md
- STAGE_SYNC_FIX_COMPLETE.md

---

**Document Version:** 1.0  
**Last Updated:** October 11, 2025  
**Author:** AI Analysis of gikWBC9DOF Project  
**Status:** Complete ✅

---

