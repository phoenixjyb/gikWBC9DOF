# Method 4 (PP-First) Implementation Plan

## Purpose & Scope
- Deliver Stage C **Method 4: Pure Pursuit First with GIK refinement** as a selectable execution mode alongside existing `ppForIk` and `pureIk` flows.
- Reuse current chassis, GIK, and diagnostics infrastructure without breaking legacy scripts or configuration profiles.
- Provide a roadmap for engineering, validation, and rollout so Method 4 can be toggled on/off per run and compared against other modes.

## Current Architecture Touchpoints
- `runStagedTrajectory.m` orchestrates Stage A/B/C and already switches on `options.ExecutionMode` (`"pureIk"`, `"ppForIk"`). Method 4 will extend this switch with `"ppFirst"`.
- Stage B delivers chassis-aligned base paths (`stageBResult`) that can seed Pure Pursuit; existing helper `applyStageCAlignment` aligns start poses.
- GIK solver bundle from `createGikSolver.m` supplies joint bounds (`constraints.joint`) and distance constraints used in Stage C.
- Helper utilities already in tree:  
  • `baseSeedFromEE.m` – derive base path from EE poses  
  • `initPPFromBasePath.m` – configure `purePursuitFollower` and preprocessing  
  • `updateBaseJointBounds.m` – mutate GIK joint bounds for Stage C  
  • `solveArmOnlyIK.m` – fallback arm-only solve with locked base  
  • `runStageCPPFirst.m` – PP-first loop scaffolding (needs full integration and validation)

## Target Workflow (Predict → Constrain → Solve → Fallback)
1. **Predict** base motion with `purePursuitFollower.step`, seeded by Stage B or EE-derived path.
2. **Constrain** GIK base joints via `updateBaseJointBounds` (±yaw corridor, ±position box).
3. **Solve** for full-body posture using `gikBundle.solve`.
4. **Fallback** to arm-only IK when EE error exceeds tolerance; lock base at PP prediction.
5. **Log & Compare** results versus Method 1 for diagnostics and regression testing.

## Work Breakdown Structure

### 1. Baseline & Configuration (Prep)
- [ ] Audit `pipeline_profiles.yaml` and add `stage_c.mode = ppFirst` option plus tolerances (`yawCorridorDeg`, `positionTolerance`, `eeErrorTolerance`, `ppDesiredVelocity`, `ppLookahead`) with sane defaults.
- [ ] Update `gik9dof.configurationTools` to surface Stage C Method 4 parameters and map legacy configs to new defaults.
- [ ] Ensure `runStagedTrajectory` parses `options.StageC*` settings and exposes them when `"ppFirst"` is selected (line refs: `matlab/+gik9dof/runStagedTrajectory.m:70`, `:195`).

### 2. Pure Pursuit Prediction Path
- [ ] Confirm Stage B output (`stageBResult.basePath`) feeds `initPPFromBasePath`; fall back to `baseSeedFromEE` when Stage B skipped.
- [ ] Extend `initPPFromBasePath` to respect reverse segments, Stage C lookahead tuning, and optional RS/Clothoid refinement toggled by `ApplyRefinement`.
- [ ] Add alignment guard so PP start pose equals Stage C initial base (reuse `generateStageCAlignmentInfo`).

### 3. GIK Constraint Integration
- [ ] Harden `updateBaseJointBounds` with bounds clamping (respect robot joint limits) and optional adaptive corridor scaling (based on curvature).
- [ ] Expose corridor + box sizes in log for post-run tuning (`log.diagnostics.ppFirst` bundle).
- [ ] Verify `createGikSolver` returns mutable `constraints.joint` handles across calls (avoid unintended copies).

### 4. Stage C Execution Loop
- [ ] Finalize `runStageCPPFirst` main loop to:
  - sync PP state with actual GIK base pose every iteration,
  - capture PP controller status codes, and
  - record solve timings (`tic/toc` around GIK).
- [ ] Implement fallback branch via `solveArmOnlyIK`, populate `fallbackUsed`, and store dual `solInfo` structs.
- [ ] Surface summary diagnostics back to Stage C (`logC.diagnostics`) so pipelines downstream continue to work (refs: `matlab/+gik9dof/runStagedTrajectory.m:901-945`).

### 5. Method Switching & Script Integration
- [ ] Update orchestration scripts (`run_fresh_simulation.m`, `run_fresh_sim_with_animation.m`, `run_stagec_path_fix.m`, etc.) to accept `ExecutionMode="ppFirst"` or map to CLI flags.
- [ ] Provide utilities (`runStageCPPFirst.m` wrapper) for standalone debugging; ensure they respect shared config loading and logging conventions.
- [ ] Document selection workflow in `SIMULATION_WORKFLOW_GUIDE.md` and `PROJECT_ARCHITECTURE_UPDATE.md`.

### 6. Telemetry, Logging, & Tooling
- [ ] Extend `saveRunArtifacts.m` to persist Method 4-specific telemetry: `ppCommands`, `basePredicted`, fallback count, corridor settings.
- [ ] Update plotting/evaluation helpers (`generateLogPlots`, `computeBaseRibbonMetrics`, `evaluateLog`) to recognize `"ppFirst"` mode and render comparison overlays (Method 1 vs Method 4).
- [ ] Add dedicated diagnostic script (`compare_method4_vs_method1.m`) to automate side-by-side metrics and animations.

### 7. Testing & Validation
- **Unit Tests**
  - [ ] Add tests for `updateBaseJointBounds` (corridor math, bounds saturation).
  - [ ] Add tests for `solveArmOnlyIK` fallback achieving tight base lock.
- **Simulation Campaign**
  - [ ] Smoke-test on short straight-line and tight-turn trajectories.
  - [ ] Run canonical `1_pull_world_scaled.json` with Method 1 & Method 4, log EE error, fallback rate, solver iterations.
  - [ ] Validate reverse-segment handling using Stage B scenarios that include backing maneuvers.
- **Performance**
  - [ ] Profile per-waypoint runtime; target <100 ms average (MATLAB) and <10% fallback usage.
- **Regression**
  - [ ] Ensure legacy modes produce identical logs (hash or numeric diff) after Method 4 integration.

### 8. Rollout & Backward Compatibility
- [ ] Default pipeline remains `ppForIk`; expose Method 4 via config toggle to minimize risk.
- [ ] Provide migration checklist for downstream consumers (ROS bridge, log analyzers).
- [ ] Schedule final review comparing KPIs (EE RMS, base feasibility violations, solver runtime) before promoting Method 4 to default.

## Milestones & Timeline (Indicative)
1. **Week 1** – Prep & configuration wiring (Sections 1-3) complete; internal tests on toy trajectories.
2. **Week 2** – Full Stage C loop finalized, fallback stable, telemetry integrated.
3. **Week 3** – Validation campaign + comparative reporting; address tuning gaps.
4. **Week 4 (optional)** – Performance optimizations, adaptive corridor, real-time considerations.

## Risks & Mitigations
- **PP drift vs GIK corrections**: corridor too tight → sudden fallbacks. Mitigate with adaptive widening and synchronization on actual base pose each step.
- **Constraint conflicts**: distance bounds or joint limits may clash with corridor. Detect infeasible solves and relax bounds gradually; log incidents for tuning.
- **Config fragmentation**: new parameters scattered. Centralize defaults in `pipeline_profiles.yaml` and keep `configurationTools` as single source.
- **Legacy script breakage**: update entry points and documentation; keep default mode unchanged until regression passes.

## Deliverables
- Updated MATLAB source (`runStagedTrajectory`, `runStageCPPFirst`, helpers).
- Enhanced configuration profiles and documentation.
- Automated comparison scripts + plots demonstrating Method 4 benefits.
- Test evidence (unit + simulation) stored under `results/` with Method 4 metadata.

