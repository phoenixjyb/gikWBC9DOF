# GIK 9-DOF Workstream – Handover Notes

This document captures the current state of the MATLAB-side development, the key
utilities/scripts in play, and the datasets available for downstream validation
(e.g., C++ solver testing on WSL). It is intended to make it easy to resume
work in a fresh session or hand the branch to another engineer.

---

## Repository Highlights (Post-Refactor)

- **Removed code-generation & ROS stubs**: The `codegen/` directory, the legacy
  `ros2/` nodes, and `matlab/+gik9dof/+codegen` were deleted so this branch
  focuses on MATLAB runtime tooling only.
- **New documentation**:
  - `GIK_SETUP_OVERVIEW.md` – Deep dive into the solver bundle, collision/mode
    setup, adaptive base velocity estimation, iteration caps, animation tools.
  - `VALIDATION_ASSETS.md` – Step-by-step guide to export test bundles from
    existing logs for C++ validation.
  - `HANDOVER.md` (this file) – Summary of all recent changes and current
    testing assets.
- **Gitignore updates**: Added global ignores for generated `.mat` and `.mp4`
  files to keep the repo clean after export/animation runs.

---

## MATLAB Enhancements

### Adaptive Base-Velocity Estimation
- `matlab/+gik9dof/+internal/VelocityEstimator.m`
  - Maintains up to 5 recent base states and time stamps.
  - Chooses between 5-point, 3-point, or 2-point backward differences
    (O(h⁴)/O(h²)/O(h)) depending on available history.
  - Unwraps yaw, validates time steps, projects linear velocity to chassis
    frame (enforcing differential-drive `v_y = 0`).
- `runTrajectoryControl` now logs the estimator output in `log.baseVelocityEstimate`
  (world/raw/robot velocities, yaw rate, method tags).
- Stage comparison CSVs (`run_stageb_mode_compare`) fall back to the estimator
  when pure pursuit isn’t providing `(Vx, Wz)` commands.

### Solver Iteration Cap & Telemetry
- `createGikSolver` accepts `MaxIterations` and applies it through
  `SolverParameters` (with a fallback warning if unsupported).
- Holistic/staged entrypoints propagate `MaxIterations` down to Stage A/B/C.
- `runTrajectoryControl` captures per-step iteration counts, solve times,
  constraint max violation, and aggregated summaries (`solverSummary`).

### Iteration Study Harness
- `matlab/run_gik_iteration_study.m` executes holistic and staged runs for any
  set of iteration caps, producing:
  - Logs (`log_<mode>_iterXXXX.mat`)
  - Plots of EE error, iterations per step, and solve time
  - MP4 animations via the updated pipeline
  - `iteration_metrics.csv`

Example usage:
```matlab
summary = run_gik_iteration_study( ...
    'IterationCaps', [150, 1500], ...
    'RateHz', 30, ...
    'UseStageBHybridAStar', false, ...
    'StageBMode', 'gikInLoop');
```
Results can be found in `results/<timestamp>_gik_iter_study_*`.

### Dual-View Animations (Perspective + Top View)
- `gik9dof.viz.animate_whole_body` renders two synchronized views, overlays
  stage labels, disc obstacles, chassis/EE paths, and displays GIK metrics.
- `matlab/regenerate_iter_study_animations.m` crawls any `results/<timestamp>`
  directory, extracts log data, and rebuilds MP4s using the above helper.
  Options: `SampleStep`, `VideoFrameRate`, `FigureScale`, `StageLabels`.
- Example:
```matlab
regenerate_iter_study_animations( ...
    'results/20251007_122040_gik_iter_study_iterCompare', ...
    'SampleStep', 5, 'VideoFrameRate', 20, 'FigureScale', 0.8);
```
Outputs: `anim_wholebody_<mode>_iterXXXX.mp4`.

### Stage B Comparison Script
- `matlab/run_stageb_mode_compare.m` now patches Stage B logs with velocity
  estimates if pure pursuit is skipped, and resaves the log after generating
  CSV/MP4 assets.
- A companion script `matlab/regenerate_stageb_comparison_animations.m` (added
  but not yet exercised) follows the same pipeline with dual-view animations.

### Validation Asset Export
- A helper function `export_gik_validation_assets` (documented in
  `VALIDATION_ASSETS.md`) converts any log into a bundle of CSV/JSON files:
  `initial_state.csv`, `target_poses.csv`, `reference_solution.csv`, and
  `metadata.json` – ideal for quick C++ solver validation runs.

---

## Existing Artifacts (Ready for Validation)

- `results/20251007_122040_gik_iter_study_iterCompare/`
  - `log_holistic_iter0150.mat`, `log_holistic_iter1500.mat`,
    `log_staged_iter0150.mat`, `log_staged_iter1500.mat`
  - Corresponding metrics plots and dual-view MP4 animations
  - `iteration_metrics.csv` summarizing EE errors, iterations, solve times

To export a validation dataset from one of these logs:
```matlab
assets = export_gik_validation_assets( ...
    'results/20251007_122040_gik_iter_study_iterCompare/log_holistic_iter1500.mat', ...
    'validation_assets/holistic_iter1500', ...
    'SampleStride', 5);
```
You’ll get CSV+JSON files under `validation_assets/holistic_iter1500/` ready to feed
into a WSL C++ harness.

---

## Testing Considerations

- With `MaxIterations` capped at 150, the solver now meets a tighter runtime
  budget but hits the cap almost every step. Consider adjusting weights or
  tolerances if this cap becomes the default.
- The new telemetry makes it straightforward to detect iteration saturation or
  solve-time spikes. Check `log.iterations`, `log.solveTime`, and
  `log.solverSummary` in exported logs when debugging.
- Stage B docking warnings persist in current runs; investigate tolerance or
  planner path adjustments if this impacts validation.

---

## Next Steps / TODOs

1. Expand the validation export helper into a callable CLI/script if needed for
   automated pipelines (e.g., `matlab -batch export_gik_validation_assets(...)`).
2. Integrate the adaptive velocity estimator logic into the upcoming C++/ROS
   implementation so runtime estimates match MATLAB expectations.
3. Explore adding self-collision constraints now that collision meshes are
   consistently attached (optional future work).
4. Consider tuning `SolverParameters` (step size, tolerance) when using the
   150-iteration cap to avoid systematic cap saturation.

---

## Useful Commands Recap

```matlab
% Run iteration study for specified caps
summary = run_gik_iteration_study('IterationCaps', [150, 1500]);

% Regenerate animations with dual-view layoutegenerate_iter_study_animations('results/<timestamp>_gik_iter_study_*');

% Export validation assets for C++ solver testing
export_gik_validation_assets('results/.../log_holistic_iter1500.mat', ...
    'validation_assets/holistic_iter1500', 'SampleStride', 5);
```

The latest code is committed with the message “Refine GIK tooling and drop codegen
artifacts.” From here, you can either push upstream, branch off for C++ codegen,
or continue iterating on solver parameters and validation assets.

