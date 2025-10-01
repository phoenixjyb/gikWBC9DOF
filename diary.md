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
