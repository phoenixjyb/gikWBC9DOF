# Project Diary

## Current State
- **Robot Modeling & Control**: Implemented the 9‑DOF robot import, configuration helpers, collision utilities, and a generalized IK solver bundle. `trackReferenceTrajectory` supports both holistic control and a staged (arm → base → combined) pipeline via `runStagedTrajectory`.
- **Runtime Pipeline**: `runTrajectoryControl` logs solver diagnostics, EE errors, and timings; artifact capture (`saveRunArtifacts`) archives logs, JSON reports, plots, and optional animations into timestamped `results/` folders.
- **Visualization**: `animateTrajectory` and `evaluateLog` cover MATLAB-side playback and summarization, while `generateExternalPlots` reproduces legacy top/3‑D/error charts.
- **Code Generation Prep**: Added MATLAB Coder entry points (`solveGIKStep`, `followTrajectory`) plus a wrapper script. Coder currently halts because `loadRobotForCodegen` still imports a `rigidBodyTree` from MAT—needs a procedural builder.
- **ROS 2 Interface Skeletons**: Authored C++ nodes for holistic and staged controllers, a trajectory manager, and an obstacle provider. `gik_solver_wrapper.hpp` stubs out solver calls pending C++ code from MATLAB Coder.
- **Repo & Docs**: Repository initialized with `.gitignore`, `CODEGEN.md` (with ROS 2 deployment roadmap), and ROS node README. All assets committed.

## Next Steps
1. **Coder-Compatible Robot Builder**: Replace MAT-file loading so MATLAB Coder runs fully on Linux.
2. **Wrap Generated Solver**: Swap `gik_solver_wrapper.hpp` for actual C++ bindings once codegen works; expose ROS 2 services/actions and diagnostics.
3. **ROS 2 Integration**: Add launch files, parameter parsing, obstacle updates, and test the controllers against live joint/odometry topics.
4. **Testing**: Co-simulate with MATLAB, ensure staged/holistic toggles behave, validate collateral (logs/plots) inside the ROS environment.

## Comparative Test Plan (Holistic vs Staged)
- **Objective**: quantify tracking accuracy, solver effort, and runtime for both control modes on the reference trajectory.
- **Metrics**:
  - Mean/max position error (m)
  - Mean/max orientation error (deg)
  - Mean/max solver iterations and max constraint violation
  - Total runtime (s) and number of successful waypoints
- **Procedure**:
  1. Run `trackReferenceTrajectory` in holistic mode (`Mode='holistic'`, verbose false) and store the resulting log (`trajectory_log_holistic.mat`).
  2. Archive artifacts via `saveRunArtifacts` (with external plots) into a timestamped `results/` folder.
  3. Evaluate log using `evaluateLog` and note metrics.
  4. Repeat steps 1–3 for staged mode (`Mode='staged'`).
  5. Summarize comparisons (tables/notes) back in this diary.
- **Automation**: Script test runs directly in MATLAB batch commands to ensure reproducibility and to keep console outputs captured.

## Comparative Results (2025-10-01)
- **Holistic** (`results/20251001_160247_holistic_test`)
  - Position error mean/max: 3.27e-4 m / 6.2e-3 m
  - Orientation error mean/max: 0.0025° / 0.0461°
  - Solver iterations mean/max: 1.03e3 / 1500 (exit flags histogram [1;59;88])
  - Max constraint violation: 6.2e-3
  - Completed 148/148 waypoints (baseline trajectory)
- **Staged** (`results/20251001_161221_staged_test`)
  - Position error mean/max: 2.16e-4 m / 1.2e-3 m
  - Orientation error mean/max: 0.0030° / 0.0777°
  - Solver iterations mean/max: 1.38e3 / 1500 (exit flags histogram [1;190;56])
  - Max constraint violation: 2.60e-2
  - Completed 247/247 waypoints (due to inserted Stage A/B/C samples)
- **Notes**:
  - Staged control tightens positional accuracy (lower mean/max) but increases solver effort and constraint violations due to additional sample points.
  - Holistic run maintains small violation margins and shorter runtime with fewer waypoints; staged adds more setpoints which raised average iterations.
- **Visualization**: Generated side-by-side animation (`results/comparison_holistic_vs_staged.mp4`) showing both runs with desired/actual EE traces and floor-disc obstacles, mirroring the legacy demo styling.
  Updated animation now mirrors the legacy demo (fixed camera, stage overlays, obstacles, robot meshes) and is saved at `results/comparison_holistic_vs_staged.mp4` (~265 KB).

## 2025-10-04 – Holistic Ramp & Visualization refresh

- aligned `mobile_manipulator_PPR_base_corrected.urdf` with the new STL assets under `meshes/`; confirmed visuals via direct `importrobot` preview
- updated `gik9dof.collisionTools` catalog so collision checks use `base_link.STL`, `left_arm_link*.STL`, `end_effector.STL`; `trackReferenceTrajectory` now resolves meshes from `meshes/`
- added velocity-limited ramp generation (`gik9dof.generateHolisticRamp`) and velocity clamping in `runTrajectoryControl`; holistic logs now carry ramp/limit metadata
- pulled the animation helper into `gik9dof/viz`, pointing at repo meshes (with optional `mesh/stl_output` reduced files) and added `FigureScale`, inflated-disc visualization, and live EE error readout
- reran holistic scenario (`results/log_holistic_latest_fullres.mat`) and produced `results/holistic_latest_fullres.mp4`; chassis/arm visuals match URDF and no mesh warnings remain

## 2025-10-05 – Unified Chassis Control, Stage-B Follower & Batch Harness

- Implemented `gik9dof.control.unifiedChassisCtrl`, `clampYawByWheelLimit`, `defaultUnifiedParams`, plus offline replay (`unified_chassis_replay`) so holistic and staged pipelines emit a single `(Vx,0,Wz)` interface while logging wheel-feasible yaw limits.
- Dropped in an in-repo pure‑pursuit follower (`gik9dof.control.purePursuitFollower`) and wired Stage B to expose the Hybrid A* path as `stageLogs.stageB.pathStates`. The replay tool now recovers Stage B chassis commands via the same follower logic.
- Refreshed `run_environment_compare` to create timestamped result dirs, export holistic/staged animations, and call the new `gik9dof.generateLogPlots` helper for arm/chassis plots. Added a top-level `run_environment_compare_latest.m` convenience script.
- Latest batch run (`results/20251005_171238_holistic_staged/`) contains matched holistic & staged logs, MP4 animations, arm joint plots, and chassis velocity plots. Staged run still shows higher final EE error than holistic (0.7059 vs 1.6971) but now benefits from unified logging and the stored Stage B path for future tuning.

### Outstanding Items
- Tighten pure-pursuit parameters (lookahead, velocity) per chassis configuration; add automated handover trigger once `status.isFinished` is asserted.
- Record unified commands inside `runTrajectoryControl` during live runs (currently only accessible via staged replay); consider adding a logging hook for holistic Stage C as well.
- Integrate `unified_chassis_replay` into ROS notebook/tests so Stage B velocity commands can be validated against real odometry before firmware execution.

## 2025-10-05 (evening) – Disk Obstacle Regression + Compare Harness v2

- **EnvironmentConfig** now programmatically anchors Disc 2 at waypoint 112 of the JSON reference path, sets uniform radius/height (0.10 m / 0.15 m), and applies safety + distance margins (0.05 m + 0.15 m) with a stronger weight (5.0). This ensures both holistic and staged runs share the exact obstacle definition when the config is fetched.
- **Visualization helpers** (`animateHolisticWithHelper`, `animateStagedWithHelper`) now inject target paths and obstacle metadata so regenerated animations carry the full desired EE ribbon, chassis plan, and inflated disc halos. Legends no longer default to “Data1/Data2” when videos are rebuilt from logs.
- **Batch harness** (`run_environment_compare` / `run_environment_compare_latest`) drives both controllers at 50 Hz, exports staged-style MP4s, and drops arm/chassis diagnostic PNGs through `generateLogPlots`. Results land in timestamped folders; `regenerate_animations_from_logs.m` rebuilds artifacts later without re-simulating.
- **Latest compare (results/20251005_225256_compare)** shows the staged pipeline respecting both discs (min clearance ~0.70 m on disc 1), whereas the holistic controller still collides with disc 1 (−0.14 m clearance) and consequently halts mid-route. This confirms that tighter obstacles require either a planned base motion in holistic mode or relaxed distance constraints.

**Still in progress**
- Add a holistic pre-planning step (reuse the Stage B Hybrid A*) so the base can clear Disc 1 before switching to full-body tracking.
- Evaluate margin/weight combinations that guarantee clearance yet keep the solver feasible; log the resulting clearances to verify.
- Update the regen helper to preserve existing MP4 timestamps when overwriting, or add an option to skip regeneration for long runs.
