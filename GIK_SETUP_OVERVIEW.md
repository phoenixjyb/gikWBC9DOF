# GIK Setup Overview

This document captures how generalized inverse kinematics (GIK) is configured and used throughout the project, including the constraint set applied in holistic and staged control modes and the current collision-avoidance strategy.

## Core Solver Bundle

- `gik9dof.createGikSolver` builds a `generalizedInverseKinematics` object around a shared set of constraint handles.
  - Always instantiates `constraintPoseTarget` for the end effector and `constraintJointBounds` for soft joint-limit enforcement.
  - Optionally adds `constraintAiming` when aiming is requested and one `constraintDistanceBounds` object per supplied distance spec.
  - Exposes helper closures (e.g., `updatePrimaryTarget`, `setDistanceBounds`) and a `solve` wrapper that refreshes constraint targets before invoking the solver.
- The solver works with `gik9dof.configurationTools` to translate between column/struct representations and to access the default home configuration.
- `gik9dof.runTrajectoryControl` is the common velocity-level loop that streams waypoints through the solver bundle. It supports per-waypoint aiming targets, distance bounds/references, velocity limiting, and optional command callbacks while logging solver diagnostics, per-step solver status, iteration counts, constraint-violation metrics, and wall-clock solve times (with aggregate statistics and success/failure tallies captured in the returned log).
- `gik9dof.internal.VelocityEstimator` tracks the planar base history (up to five samples) and supplies adaptive O(h), O(h²), or O(h⁴) backward-difference velocities with angle unwrapping. `runTrajectoryControl` instantiates the estimator whenever base joints are present, publishing the filtered outputs (including method tags) under `log.baseVelocityEstimate` for downstream consumers.

## Environment and Collision Inputs

- `gik9dof.environmentConfig` centralises default environment parameters:
  - Base home pose `[-2, -2, 0]`.
  - Two floor-disc obstacles (radius 0.10 m, safety margin 0.05 m, height 0.15 m) plus an additional solver distance margin of 0.15 m.
  - Default distance weight (5.0) and Stage B docking tolerances.
- `gik9dof.addFloorDiscs` attaches each floor disc as a fixed frame under the robot base and returns disc metadata for constraint construction.
- `gik9dof.collisionTools` binds STL meshes to the robot bodies for downstream collision checks and visualisation. The project currently leverages these meshes for reporting/visuals but does not add self-collision constraints to GIK.

## Holistic Mode Pipeline

`gik9dof.trackReferenceTrajectory` controls the entire 9-DOF robot directly:

1. Loads the JSON reference trajectory and builds the robot model (with collision meshes applied).
2. Applies the environment base-home override if provided.
3. Adds floor discs to the robot and converts each disc into a `constraintDistanceBounds` spec that keeps `abstract_chassis_link` outside `radius + safety + DistanceMargin`.
4. Instantiates a single solver bundle with optional aiming and the collected distance specs.
5. Streams the (optionally ramp-augmented) trajectory through `runTrajectoryControl` with velocity caps derived from the holistic ramp settings.

Result: Holistic tracking enforces base-to-disc clearance continuously via the distance constraints and honours joint limits and any requested aiming targets.

## Staged Mode Pipeline

`gik9dof.runStagedTrajectory` breaks execution into three phases, each with its own solver context and constraints.

### Stage A – Arm-Only Ramp-Up
- Creates a fresh solver bundle and locks the base joints by collapsing the joint-bounds constraint onto the current base configuration.
- Interpolates from the starting pose to the first trajectory pose, moving only the manipulator joints.

### Stage B – Base Alignment
- Two submodes are available:
  - **`gikInLoop`**: generates interpolated end-effector poses (optionally seeded with planner states) and runs a solver bundle with the arm joints frozen via joint bounds so only the planar base moves. Velocity limits match the Stage B settings.
  - **`pureHyb`**: plans a base path in SE(2) using Hybrid A*, then integrates it with a pure-pursuit follower. GIK is only used to compute the Stage B goal configuration with the arm locked; the executed path is dictated by the planner/simulator outputs.
- Hybrid A* planning inflates floor discs by the environment safety margin before constructing the occupancy map, ensuring the planner respects the same obstacles.

### Stage C – Full-Body Tracking
- Instantiates another solver bundle that reuses the environment-derived distance specs.
- Streams the remainder of the reference trajectory with the same velocity limits used in prior stages.

The final pipeline log includes per-stage logs, distance specs, environment settings, and the selected Stage B mode, enabling comparisons such as `run_stageb_mode_compare`.

## Constraint Summary

| Constraint Type            | Source                        | Usage                                                  |
|----------------------------|-------------------------------|--------------------------------------------------------|
| Pose Target                | `constraintPoseTarget`        | Enforced in every solver bundle.                       |
| Joint Bounds               | `constraintJointBounds`       | Always active; tightened to lock joints when needed.   |
| Aiming (optional)          | `constraintAiming`            | Enabled via solver options and per-waypoint aim data.  |
| Distance Bounds (optional) | `constraintDistanceBounds`    | Added per distance spec; used for floor-disc clearance |

## Collision Avoidance Status

- Floor-disc avoidance is implemented through distance constraints in holistic mode and Stage C, plus occupancy inflation inside Hybrid A* for Stage B planning.
- Stage A and Stage B (`gikInLoop`) currently omit the distance constraints when the base is locked/unlocked respectively. If base clearance during those transients is required, distance specs could be carried into those bundles.
- No self-collision constraints are presently configured despite collision meshes being available; extending `DistanceSpecs` to cover body-to-body distances would be a straightforward next step if needed.

## Follow-Up Considerations

1. Evaluate whether Stage A and Stage B (gik-in-loop) should include the distance constraints to maintain uniform obstacle clearance across the entire staged pipeline.
2. Investigate leveraging the attached collision meshes to formulate additional distance constraints for self-collision avoidance if future tasks demand it.

## Recent MATLAB Enhancements

### Adaptive Base-Velocity Estimation
- Introduced `gik9dof.internal.VelocityEstimator`, a five-sample circular buffer that auto-selects between 5-point (O(h⁴)), 3-point (O(h²)), and 2-point (O(h)) backward stencils depending on history length.
- Handles yaw unwrapping before differentiation, rejects degenerate time steps, and projects the world-frame derivative through the chassis frame to enforce `v_y = 0`.
- `runTrajectoryControl` now instantiates the estimator whenever `VelocityLimits.BaseIndices` are provided, logging the filtered world/robot velocities, yaw rate, and method tags under `log.baseVelocityEstimate`. Stage B command CSVs in `run_stageb_mode_compare` fall back to this estimate when pure pursuit is inactive.

### Solver Iteration Controls & Telemetry
- Added a `MaxIterations` option to `gik9dof.createGikSolver`; the value is propagated through holistic (`trackReferenceTrajectory`) and staged (`runStagedTrajectory`) pipelines, including Stage B sub-solvers and Hybrid A* goal solves.
- Captured per-step iteration counts, solve times, constraint-violation summaries, and status tallies (`solverSummary`) in `runTrajectoryControl`, enabling quick health checks of GIK convergence.
- `run_gik_iteration_study.m` automates comparative runs for any iteration cap: it executes holistic/staged modes, plots EE error vs. time, solver iterations, solve-time trace, saves MP4 animations, and writes a consolidated `iteration_metrics.csv` table.
  - Example usage:
    ```matlab
    summary = run_gik_iteration_study( ...
        'IterationCaps', [150, 1500], ...
        'RateHz', 30, ...
        'UseStageBHybridAStar', false, ...
        'StageBMode', 'gikInLoop');
    ```
  - Observations: capping at 150 iterations cuts mean solve time (holistic: 1.7 s → 0.2 s, staged Stage C: 0.69 s → 0.20 s) while keeping Stage C EE errors within 1 mm RMS; both settings saturate Stage B constraints, motivating further tuning of tolerances/step sizes.

### Dual-View Animation Pipeline
- `gik9dof.viz.animate_whole_body` renders synchronized perspective/top views, overlays chassis/EE reference paths, inflates disc obstacles, and displays stage captions. It now accepts optional `Obstacles`, `StageBoundaries`, and `TargetPath` inputs for richer context.
- `regenerate_iter_study_animations.m` converts any `log_*.mat` (e.g., from the iteration study) into dual-view MP4s, reusing cached logs without rerunning the controller. It subsamples frames (`SampleStep`), inserts stage boundary annotations derived from `log.stageLogs`, and reuses the disc definitions stored under `log.floorDiscs`.
- Typical call:
  ```matlab
  regenerate_iter_study_animations( ...
      'results/20251007_122040_gik_iter_study_iterCompare', ...
      'SampleStep', 5, 'VideoFrameRate', 20, 'FigureScale', 0.8);
  ```
- Outputs include `anim_wholebody_<mode>_iterXXXX.mp4`, preserving the legacy two-pane staging while reflecting the latest velocity/constraint telemetry.

### Updated Batch Compare Utilities
- `run_stageb_mode_compare` now honors on-disk velocity estimates when pure pursuit is bypassed, saves enriched logs (including base velocity, solve telemetry), and exports command CSVs with the differentiation method markup.
- The stage-comparison results (`results/<timestamp>_stageB_compare_*.mat`) remain compatible with the plotting helpers while offering deeper solver diagnostics for post-processing.

## Suggested Next Steps
- Sweep reduced iteration caps with relaxed tolerances or adjusted IK weights to avoid hitting the cap each step.
- Integrate the adaptive velocity estimator into ROS/firmware code paths so real-time deployments gain the same filtered `(Vx, Wz, ω)` signals.
- Extend obstacle visualization to pull from environment JSONs dynamically when multiple test scenes are introduced.
