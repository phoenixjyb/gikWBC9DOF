# Project Overview

## Repository Layout

```
gikWBC9DOF/
├─ config/
│  └─ chassis_profiles.yaml          # YAML presets for chassis/follower tuning
├─ meshes/                          # Runtime geometry (STL)
├─ 1_pull_world_scaled.json         # Reference EE trajectory
├─ results/<timestamp>_*            # Holistic vs. staged artifacts (logs, MP4, PNG)
├─ matlab/+gik9dof/
│  ├─ assets & environment
│  │   ├─ environmentConfig.m       # Disc locations, base home, margins
│  │   └─ collisionTools.m          # Attaches meshes to the rigidBodyTree
│  ├─ core control library
│  │   ├─ createRobotModel.m
│  │   ├─ configurationTools.m
│  │   ├─ createGikSolver.m         # IK constraints/weights
│  │   ├─ runTrajectoryControl.m    # Velocity-level IK loop
│  │   ├─ runStagedTrajectory.m     # Stage A (arm), B (Hybrid A*), C (full body)
│  │   ├─ trackReferenceTrajectory.m# Entry point (holistic | staged)
│  │   ├─ +control/
│  │   │    ├─ purePursuitFollower.m
│  │   │    ├─ unifiedChassisCtrl.m
│  │   │    ├─ clampYawByWheelLimit.m
│  │   │    └─ defaultUnifiedParams.m
│  │   └─ generateHolisticRamp.m, addFloorDiscs.m, …
│  ├─ visualization
│  │   ├─ +viz/animate_whole_body.m
│  │   ├─ animateStagedWithHelper.m
│  │   └─ animateHolisticWithHelper.m
│  ├─ evaluation / plotting
│  │   ├─ generateLogPlots.m
│  │   ├─ evaluateLog.m, plotTrajectoryLog.m
│  │   └─ saveRunArtifacts.m
│  └─ internal/
│      ├─ createResultsFolder.m
│      └─ resolvePath.m, projectRoot.m
├─ run_environment_compare.m            # Batch orchestrator (holistic + staged)
├─ run_staged_reference.m               # Staged-only regression entry point
├─ regenerate_animations_from_logs.m    # Rebuild MP4/PNG from saved logs
├─ unified_chassis_controller_summary.md
└─ diary.md
```

## Functional Categories

| Area                      | Key Files / Directories                                             | Purpose |
|---------------------------|---------------------------------------------------------------------|---------|
| Assets & environment      | `meshes/`, `1_pull_world_scaled.json`, `environmentConfig.m`, `collisionTools.m`, `config/chassis_profiles.yaml` | Physical geometry, base home pose, obstacle definitions, chassis presets. |
| Core control & planning   | `trackReferenceTrajectory.m`, `runStagedTrajectory.m`, `runTrajectoryControl.m`, `createGikSolver.m`, `generateHolisticRamp.m`, `+control/*.m` | Holistic controller, staged pipeline, Stage B Hybrid A* integration, unified chassis interface. |
| Animation & visualization | `+viz/animate_whole_body.m`, `animateStagedWithHelper.m`, `animateHolisticWithHelper.m` | Staged-style animations, obstacle halos, target/actual path overlays. |
| Orchestration             | `run_environment_compare.m`, `run_staged_reference.m`, `regenerate_animations_from_logs.m`, `saveRunArtifacts.m`, `createResultsFolder.m` | Batch runs, timestamped result folders, artifact regeneration. |
| Diagnostics / plots       | `generateLogPlots.m`, `evaluateLog.m`, `plotTrajectoryLog.m`        | Arm joint traces, chassis velocity plots, log summaries. |

## Control & Planning Architecture

```
         ┌─────────────────────────────┐
         │ environmentConfig           │
         └──────────┬──────────────────┘
                    │
         ┌──────────▼───────────┐        ┌───────────────────────────┐
         │ trackReferenceTrajectory ─────►│ runStagedTrajectory       │
         │  Mode = holistic               │   Stage A: arm-only align │
         │  Mode = staged                 │   Stage B: Hybrid A* path │
         └──────────┬───────────┘        │   Stage C: full IK track  │
                    │                    └─────────────┬─────────────┘
                    │                                  │
    ┌───────────────▼──────────────┐   ┌───────────────▼────────────┐
    │ runTrajectoryControl          │◄──┤ unifiedChassisCtrl         │
    │ - 9-DOF IK loop               │   │ - Holistic / Stage C feeds │
    │ - velocity clamps             │   │ - Stage B: pure pursuit -> │
    │ - logs cmd / diagnostics      │   │   (Vx, Wz)                 │
    └──────────────────────────────┘   └──────────────┬────────────┘
                                                   ▲             │
                                                   │             │
                                  ┌─────────────────┴────────────┐
                                  │ gik9dof.control.purePursuitFollower │
                                  │ - Follows Hybrid A* path             │
                                  └──────────────────────────────────────┘
```

- **Holistic mode**: `trackReferenceTrajectory` drives the full IK in `runTrajectoryControl`. Unified chassis outputs (Vx, Wz) are available for logging/firmware integration.
- **Staged mode**: Stage A locks the base, Stage B plans a base-only path (Hybrid A* → pure pursuit → unifiedChassisCtrl), Stage C resumes full-body tracking.

## Running Simulations

1. **Holistic vs. staged regression**
   ```matlab
   run_environment_compare
   ```
   Edit the configuration block at the top of `run_environment_compare.m` to pick execution modes (`"ppForIk"` vs `"pureIk"`), Stage B planner mode, solver iteration cap, and animation settings.

2. **Staged-only regression**
   ```matlab
   run_staged_reference
   ```
   The script surfaces the same options for execution mode, Stage B Hybrid A*, lookahead distance, and solver limits. Logs land in a timestamped results folder.

3. **Rebuild animations from existing logs**
   ```matlab
   regenerate_animations_from_logs
   ```
   Point `runFolder` at the desired `results/<timestamp>_*` directory to regenerate staged-style MP4s and diagnostic PNGs without rerunning the controllers.

> Chassis/follower tuning lives in `config/chassis_profiles.yaml`. Use
> `gik9dof.control.loadChassisProfile("wide_track")` (or supply overrides)
> before running the scripts above to keep MATLAB and future C++ builds in sync.

## Animation Workflow

1. `run_environment_compare.m` runs holistic then staged (configurable execution modes), writing logs/MP4/PNG into `results/<timestamp>_compare`.
2. `animateHolisticWithHelper`/`animateStagedWithHelper` share the staged-style scene (dark floor, inflated discs, desired path ribbons).
3. `regenerate_animations_from_logs` can be executed later to rebuild MP4/PNG from existing logs without rerunning the controller.

## ROS 2 / Code Generation Notes

- Replace runtime JSON loading and dynamic field access with coder-friendly data structures.
- Ensure collision meshes are attached procedurally for codegen (no `addCollision` calls at runtime).
- Provide MATLAB Coder entry points for holistic Stage C control; for Stage B, plan paths outside the generated code (e.g., Hybrid A* in C++/ROS) and feed `(Vx, Wz)` into `unifiedChassisCtrl`.
- ROS node responsibilities:
  - Perception publishes obstacles / occupancy grids.
  - Planner (Stage B) produces base path if needed.
  - Unified controller subscribes to odometry, joint states, and the desired EE trajectory; publishes chassis velocities `(Vx, Wz)` (and optional arm rates) for the firmware.
- Logging: replicate MATLAB telemetry (min obstacle clearances, command logs) in ROS for consistent validation.

## Handy Scripts

- `run_environment_compare.m` — holistic vs staged batch run. Edit the configuration block (execution modes, Stage B planner toggle, solver iterations) before launching.
- `run_staged_reference.m` — staged-only regression helper; configurable execution mode and Stage B settings.
- `regenerate_animations_from_logs.m` — rebuild animations/plots for a chosen results directory.
*** End Patch
PATCH
