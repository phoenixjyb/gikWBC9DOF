# Project Data & Module Flow

This guide expands the concise notes in our other design documents and details how trajectory data moves through ingestion, staged control, logging, and animation until a review-ready MP4 is produced.

## Ingestion and Solver Setup
- **Environment provisioning** – Every staged run begins in `gik9dof.runStagedReference`, which clones the shared configuration returned by `gik9dof.environmentConfig` before applying CLI overrides (`matlab/+gik9dof/runStagedReference.m:61`, `matlab/+gik9dof/environmentConfig.m:20`). The environment file seeds the base home pose, floor-disc radii, solver distance margins, and docking tolerances. A second obstacle is positioned automatically by sampling waypoint 112 from the bundled JSON so that planning tests the tightest region of the reference path (`matlab/+gik9dof/environmentConfig.m:30`).
- **Robot model and configuration utilities** – `gik9dof.trackReferenceTrajectory` imports the URDF via `gik9dof.createRobotModel`, normalises the planar base joint axes, and, when requested, attaches chassis footprint bodies that become distance references later in the pipeline (`matlab/+gik9dof/createGikSolver.m:1`, `matlab/+gik9dof/createRobotModel.m:60`). The helper `gik9dof.configurationTools` exposes conversion closures so staged routines can switch between struct and column representations without repeating boilerplate (`matlab/+gik9dof/configurationTools.m:1`).
- **Trajectory ingestion and solver bundling** – Once the robot is ready, `trackReferenceTrajectory` resolves the JSON path relative to the repository, decodes all 148 waypoints, and converts each entry into SE(3) matrices plus xyz arrays through `loadJsonTrajectory` (`matlab/+gik9dof/trackReferenceTrajectory.m:95`, `matlab/+gik9dof/trackReferenceTrajectory.m:481`). Floor discs from the environment are promoted to `constraintDistanceBounds` specs that guard both the chassis link and footprint bodies (`matlab/+gik9dof/trackReferenceTrajectory.m:200`). With constraints assembled, `gik9dof.createGikSolver` instantiates a `generalizedInverseKinematics` object and returns the bundle of constraint handles and update utilities (`matlab/+gik9dof/createGikSolver.m:1`).
- **Hand-off to the staged controller** – Depending on the `Mode` option, `trackReferenceTrajectory` either executes holistic control immediately or defers to `gik9dof.runStagedTrajectory` to orchestrate the three-stage pipeline (`matlab/+gik9dof/trackReferenceTrajectory.m:387`).

## Staged Control Pipeline
### Stage A – Arm ramp with base locked
- Stage A builds an interpolated pose sequence between the home configuration and the first JSON waypoint. Base joint bounds are collapsed so only the manipulator moves, producing `log.stageLogs.stageA.qTraj` alongside solver telemetry (`matlab/+gik9dof/runStagedTrajectory.m:114`). The terminal configuration becomes the starting point for Stage B.

### Stage B – Base alignment alternatives
- When `StageBMode="gikInLoop"`, the code optionally seeds a Hybrid-A*/pure-pursuit ribbon (if `UseStageBHybridAStar` is true) before running GIK with the arm joints locked. The helper `executeStageBGikInLoop` records the planner states, pure-pursuit simulation, docking metrics, and command log table (`matlab/+gik9dof/runStagedTrajectory.m:216`). Successful runs enrich `log.stageLogs.stageB` with `pathStates`, `execBaseStates`, `goalBase`, `achievedBase`, and any RS/clothoid diagnostics.
- When `StageBMode="pureHyb"`, the planner output is promoted directly into `log.stageLogs.stageB` via a synthetic log builder so post-processing can analyse curvature, cusp counts, and smoothing acceptance rates (`matlab/+gik9dof/runStagedTrajectory.m:420`). These diagnostics feed RS tuning studies documented elsewhere.

### Stage C – Pure pursuit integration and locked-base GIK
- Stage C starts by generating a fresh reference using GIK with the base free, then optionally refines the base ribbon with RS and clothoids through `stageCApplyBaseRefinement` (`matlab/+gik9dof/runStagedTrajectory.m:567`). The refined ribbon drives `gik9dof.control.simulateChassisExecution`, providing `purePursuit` command logs, wheel speeds, and status traces (`matlab/+gik9dof/runStagedTrajectory.m:618`).
- The executed base states are injected back into `runTrajectoryControl` through `FixedJointTrajectory`, forcing a second GIK solve with the chassis locked while the arm chases the JSON poses (`matlab/+gik9dof/runStagedTrajectory.m:644`). `log.stageLogs.stageC` therefore contains both the reference ribbon and the executed base path, solver iteration statistics, end-effector errors, base deviation metrics, and stage-specific diagnostics (`matlab/+gik9dof/runStagedTrajectory.m:685`).

### Shared control-loop instrumentation
- Each stage relies on `gik9dof.runTrajectoryControl` for waypoint execution. The routine logs joint trajectories, solver exit flags, iteration counts, constraint violations, end-effector poses, tracking error vectors, orientation from quaternions, and base velocity estimates whenever base indices are provided (`matlab/+gik9dof/runTrajectoryControl.m:292`). `DATA_FLOW_ANALYSIS.m:56` clarifies that these EE poses are computed from the *executed* joint angles, confirming that downstream plots and animations reflect real solver output, not the JSON targets.

## Logging and Artifact Generation
- Once the staged controller finishes, the combined pipeline structure is saved to the timestamped results folder via `runStagedReference`. The helper emits the log path, stores the environment snapshot, and persists solver options for traceability (`matlab/+gik9dof/runStagedReference.m:109`).
- The merged log carries stage-specific substructures (`stageLogs.stageA/B/C`), the original reference trajectory, distance specs, chassis profile details, pre-Stage C alignment info, and the execution mode (`matlab/+gik9dof/runStagedTrajectory.m:188`). Stage B and C further expose command tables (`cmdLog`), planner metrics (`planner`, `diagnostics`), and pure-pursuit artefacts to support regression dashboards and tuning sweeps.
- Supporting scripts such as `generate_animation_from_saved_log.m` use these saved artifacts to print frame counts, Stage C error statistics, and reference path endpoints before rendering animations, providing a lightweight QA checklist (`generate_animation_from_saved_log.m:30`).

## Animation and MP4 Export
- `gik9dof.animateStagedWithHelper` is the bridge between logs and the visualiser. It resamples arm and base trajectories according to `SampleStep`, resolves the Stage C desired path with a priority order (reference GIK → JSON targets → executed EE), assembles stage boundaries, and packages obstacles from `log.floorDiscs` and `distanceSpecs` for consistent overlays (`matlab/+gik9dof/animateStagedWithHelper.m:23`, `matlab/+gik9dof/animateStagedWithHelper.m:42`).
- The helper forwards everything to `gik9dof.viz.animate_whole_body`, which carries out synchronized interpolation, stage labelling, target-path rendering, obstacle drawing, and optional video export (`matlab/+gik9dof/+viz/animate_whole_body.m:105`). Because the visualiser drives the rigidBodyTree with `log.qTraj`, the animation depicts the actual joint motion while overlaying the desired Stage C path highlighted in `DATA_FLOW_ANALYSIS.m:97`.
- User-level scripts (`run_fresh_simulation.m`, `generate_animation_from_saved_log.m`) wrap these helpers, set video frame rates, and perform sanity checks so the same pipeline can be replayed automatically after parameter sweeps.

## Flow Diagram (Mermaid)
The chart below captures the same module/data progression in a glanceable form. Modules appear as rounded rectangles; data artifacts and logs are shown as documents.

```mermaid
flowchart TD
    subgraph Setup
        A[runStagedReference] -->|clone & override| B(environmentConfig)
        A -->|launch| C(trackReferenceTrajectory)
        C -->|import URDF| D(createRobotModel)
        C -->|JSON decode| E[[1_pull_world_scaled.json]]
        C -->|distance specs| F(createGikSolver)
    end

    subgraph StagedPipeline
        C -->|Mode=staged| G(runStagedTrajectory)
        G --> SA[Stage A: arm ramp]
        G --> SB[Stage B: base align]
        G --> SC[Stage C: pure pursuit + IK]
        SA --> K(runTrajectoryControl)
        SB --> RS[RS refine (rsRefinePath / temp\_ref\_rs\_refine)]
        RS --> SimB(simulateChassisExecution)
        SimB --> PP[purePursuitFollower]
        SimB --> K
        SimB --> Metrics(computeBaseRibbonMetrics / evaluate*)
        SC --> Ref(stageCApplyBaseRefinement)
        Ref --> SimC(simulateChassisExecution)
        SimC --> PP
        SimC --> K
    end

    K --> Logs[[log.stageLogs.stageA/B/C]]
    Metrics --> Logs
    Logs --> M[[results/<timestamp>/log_staged_ppForIk.mat]]

    subgraph Visualisation
        M --> N(animateStagedWithHelper)
        N --> O(animate_whole_body)
        O --> P[[animation MP4]]
    end
```

## Helper Module Reference
- `matlab/+gik9dof/runStagedReference.m:1` orchestrates staged regressions: clones `environmentConfig`, forwards options to `trackReferenceTrajectory`, and persists the returned log under `results/<timestamp>/log_staged_ppForIk.mat`.
- `matlab/+gik9dof/environmentConfig.m:1` defines base home, obstacle discs, safety margins, and docking tolerances that Stage B planners, Stage C refinement, and animation overlays re-use.
- `matlab/+gik9dof/trackReferenceTrajectory.m:95` imports the robot (`createRobotModel`), decodes `1_pull_world_scaled.json`, builds distance specs, and either runs holistic control or calls `runStagedTrajectory`.
- `matlab/+gik9dof/createGikSolver.m:1` wraps `generalizedInverseKinematics` with pose, joint, aiming, and distance constraints, returning the solver bundle consumed by `runTrajectoryControl`.
- `matlab/+gik9dof/runStagedTrajectory.m:1` splits execution into Stage A/B/C, calling sub-helpers such as `executeStageBGikInLoop`, `executeStageBPureHyb`, `stageCApplyBaseRefinement`, and aggregating `stageLogs`, planner diagnostics, and pure-pursuit telemetry.
- `matlab/+gik9dof/runTrajectoryControl.m:1` is the waypoint loop that logs `qTraj`, solver iterations/exit flags, EE poses, tracking error, and base velocity estimates for every stage.
- `matlab/+gik9dof/+control/purePursuitFollower.m:1` computes adaptive lookahead and steering commands; it is instantiated by Stage B/Stage C when `simulateChassisExecution` runs in pure-pursuit mode.
- `matlab/+gik9dof/+control/simulateChassisExecution.m:1` integrates SE(2) ribbons (planner outputs or refined paths) to produce executed base states, velocity commands, wheel speeds, and controller status.
- `matlab/+gik9dof/+control/rsRefinePath.m:1` and `temp_ref_rs_refine.m:1` apply Reeds–Shepp shortcutting; their acceptance metrics are logged in Stage B/Stage C diagnostics.
- `matlab/+gik9dof/computeBaseRibbonMetrics.m:1`, `matlab/+gik9dof/evaluatePathSmoothness.m:1`, `matlab/+gik9dof/evaluateCollisionIntrusion.m:1`, and `matlab/+gik9dof/evaluateChassisConstraints.m:1` post-process base paths and logs to quantify curvature, cusps, smoothness, and clearance.
- `matlab/+gik9dof/animateStagedWithHelper.m:1` samples the staged log, resolves Stage C reference paths, obstacles, and stage boundaries, and relays them to the visualiser.
- `matlab/+gik9dof/+viz/animate_whole_body.m:1` renders the robot via forward kinematics on `log.qTraj`, overlays desired Stage C EE paths, obstacles, and base ribbons, and optionally exports MP4 files.
- Wrapper scripts (`matlab/renderWholeBodyAnimation.m:1`, `generate_animation_from_saved_log.m:1`, `run_fresh_simulation.m:1`) automate log analysis and animation regeneration for analysts and CI pipelines.

## End-to-End Flow Summary
1. **JSON ingestion** – `trackReferenceTrajectory` resolves repository-relative assets, decodes pose data, attaches collision/distance constraints, and prepares solver bundles (`matlab/+gik9dof/trackReferenceTrajectory.m:95`).
2. **Staged execution** – `runStagedTrajectory` steers Stage A/B/C, recording solver logs, planner diagnostics, and pure-pursuit telemetry (`matlab/+gik9dof/runStagedTrajectory.m:114`).
3. **Artifact capture** – `runStagedReference` stores the pipeline log in `results/<timestamp>/log_staged_ppForIk.mat` alongside the environment snapshot and CLI options (`matlab/+gik9dof/runStagedReference.m:109`).
4. **Visualisation** – `animateStagedWithHelper` and `viz.animate_whole_body` replay the logged data with stage-aware overlays, providing the visual evidence used in reviews (`matlab/+gik9dof/animateStagedWithHelper.m:90`, `matlab/+gik9dof/+viz/animate_whole_body.m:181`).
5. **MP4 export** – Automation helpers invoke the visualiser with video settings, producing MP4 files and verification prompts for analysts (`generate_animation_from_saved_log.m:62`).

Collectively, these modules fabricate a traceable pipeline from reference JSON to animation, with rich solver telemetry ensuring that any discrepancy observed in video can be cross-referenced directly against the logged data.
