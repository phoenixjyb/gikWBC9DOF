# Codex Analysis: Stage C Pipeline (Staged Mode)

## Placement in the Staged Workflow
- `trackReferenceTrajectory` switches to staged control by invoking `gik9dof.runStagedTrajectory` with the decoded JSON trajectory, solver bundle, and Stage C tuning knobs (`matlab/+gik9dof/trackReferenceTrajectory.m:421`).
- Inside `runStagedTrajectory`, Stage C executes after the Stage B docking result is aligned (if necessary) and produces the final `logC` structure that feeds the aggregated pipeline output (`matlab/+gik9dof/runStagedTrajectory.m:173` and `matlab/+gik9dof/runStagedTrajectory.m:185`).

## Entry Conditions & Alignment
- `generateStageCAlignmentInfo` measures residual error between the Stage B start pose and its goal; if tolerances are exceeded it synthesises an alignment ribbon and command log (`matlab/+gik9dof/runStagedTrajectory.m:913`).
- `applyStageCAlignment` splices the alignment tail into Stage B’s executed path and updates `stageBResult.qFinal`, ensuring Stage C starts from the corrected pose (`matlab/+gik9dof/runStagedTrajectory.m:979`).

## Pass 1 – GIK Reference Ribbon
- `executeStageCPurePursuit` begins by solving the full Stage C trajectory with the base free, using a fresh solver bundle created via `gik9dof.createGikSolver` (`matlab/+gik9dof/runStagedTrajectory.m:616`).
- `gik9dof.runTrajectoryControl` produces the initial `logRef`, capturing joint trajectories, achieved end-effector poses, and solver diagnostics that will serve as the ideal reference ribbon (`matlab/+gik9dof/runStagedTrajectory.m:620`).

## Optional Base Refinement
- `stageCApplyBaseRefinement` rebuilds an occupancy map around the initial Stage C path, densifies samples, and runs Reeds–Shepp shortcutting and optional clothoid smoothing to reduce curvature or cusps (`matlab/+gik9dof/runStagedTrajectory.m:1840`).
- Successful refinements trigger a re-solve of the reference pass with the smoothed base trajectory locked via `FixedJointTrajectory`, ensuring the arm reference matches the refined path (`matlab/+gik9dof/runStagedTrajectory.m:1928`).
- The helper relies on `gik9dof.control.rsRefinePath` for stochastic shortcutting (`matlab/+gik9dof/+control/rsRefinePath.m:1`) and `gik9dof.control.rsClothoidRefine` when clothoid smoothing is enabled.

## Pass 2 – Chassis Simulation
- Stage C clones the loaded chassis profile, applies Stage C velocity/yaw limits, and constructs pure-pursuit follower options (`matlab/+gik9dof/runStagedTrajectory.m:634`).
- `gik9dof.control.simulateChassisExecution` integrates the chassis along the reference ribbon, selecting the requested controller mode (pure pursuit by default) (`matlab/+gik9dof/runStagedTrajectory.m:655`, `matlab/+gik9dof/+control/simulateChassisExecution.m:1`).
- Pure pursuit mode delegates to `simulatePurePursuitExecution`, which iterates `gik9dof.control.purePursuitFollower` to generate velocity commands, executed poses, wheel speeds, and per-step status records (`matlab/+gik9dof/+control/simulatePurePursuitExecution.m:35`, `matlab/+gik9dof/+control/purePursuitFollower.m:56`).
- Resampling logic keeps the simulated base states aligned with the reference waypoint count, and falls back to the unmodified ribbon if the controller returns no progress (`matlab/+gik9dof/runStagedTrajectory.m:659`).

## Pass 3 – GIK with Locked Base
- The executed base sequence is passed back into `gik9dof.runTrajectoryControl` through the `FixedJointTrajectory` hook so the arm solves against the realistic chassis motion (`matlab/+gik9dof/runStagedTrajectory.m:685` and `matlab/+gik9dof/runTrajectoryControl.m:205`).
- This pass populates final joint trajectories (`logC.qTraj`), achieved EE poses, tracking errors, and solver statistics with the chassis motion baked in.

## Logging & Diagnostics
- Stage C attaches the reference ribbon, executed base path, command logs, wheel speeds, follower status, and refinement metadata under `logC.purePursuit`, ensuring downstream tools can reconstruct the chassis story (`matlab/+gik9dof/runStagedTrajectory.m:695`).
- Enhanced diagnostics quantify solver iteration distributions, end-effector tracking bins, base yaw drift, and refinement deltas so regression dashboards can flag degradations (`matlab/+gik9dof/runStagedTrajectory.m:722`).
- All Stage C artifacts flow into the pipeline aggregate (`pipeline.stageLogs.stageC`) and are available to visualisation helpers such as `animateStagedWithHelper`, which overlays reference and executed base paths during review (`matlab/+gik9dof/runStagedTrajectory.m:208` and `matlab/+gik9dof/animateStagedWithHelper.m:1`).

## Alternative Execution Mode
- If `ExecutionMode="pureIk"`, Stage C skips chassis simulation and runs a single IK pass; the base states are taken directly from `logC.qTraj`, and velocity command logs remain empty (`matlab/+gik9dof/runStagedTrajectory.m:833`).

## Key Data Dependencies
- Controls and limits originate from the chassis profile loaded earlier in the pipeline, and the same `velLimits` struct constrains both the GIK passes and base refinement densification (`matlab/+gik9dof/runStagedTrajectory.m:635`).
- Distance constraints introduced at ingestion are preserved across all Stage C solves because each solver bundle is constructed with the same `options.DistanceSpecs` set (`matlab/+gik9dof/runStagedTrajectory.m:616`).
