# Method 5 (pureMPC) – Current Configuration & Status

## 1. Configuration Snapshot

- **Profile:** `config/pipeline_profiles.yaml` → `pureMPC`
- **Controller setup:** `nlmpc(9, 12, 8)` created in `matlab/+gik9dof/runStageCPureMPC.m`
  - States: base pose `[x, y, θ]` + 6 arm joints
  - Inputs: base velocities `[v, ω]` + 6 joint velocities
  - Outputs: EE position (3) + rotation matrix flattened (9)
- **Timing:**
  - Sample time `Ts = 0.1 s` (10 Hz target)
  - Prediction horizon `p = 10` (1.0 s look-ahead)
  - Control horizon `m = 5`
- **Constraints (`pureMPC.stage_c.nmpc.constraints`):**
  - Linear velocity `[-1.0, 1.0]` m/s, angular `[-2.0, 2.0]` rad/s
  - Wheel-speed budget (3.0 m/s) – currently enforced via MV limits
  - Arm joint velocity `[-2.0, 2.0]` rad/s
  - Arm joint position limits pulled from URDF entries
- **Cost structure (see `matlab/+gik9dof/+mpc/eeTrackingCostFcn.m`):**
  - EE position & orientation tracking via FK
  - Input effort weights (`input_v`, `input_omega`, `input_arm`)
  - Rate/smoothness weights (`rate_v`, `rate_omega`, `rate_arm`)
  - Optional posture regularisation (base/arm) now supported
  - Terminal pose penalty scaled by `weights.terminal`
- **Gradients:** Analytic Jacobian registered via `eeTrackingCostJacobian.m` (fallbacks if field absent).

## 2. Pipeline Flow

1. `trackReferenceTrajectory` (Mode `staged`, ExecutionMode `pureMPC`)
2. `runStagedTrajectory` routes Stage C to `executeStageCPureMPC`
3. `executeStageCPureMPC` loads NMPC params, calls `runStageCPureMPC`
4. `runStageCPureMPC` loop:
   - Extracts EE reference block from JSON poses
   - Calls `nlmpcmove` with custom cost & Jacobian
   - Applies first input, propagates unicycle + joint integrator
   - Logs state, EE pose, errors, solver diagnostics (`solutionInfo`)
5. Results merged back into pipeline logs; staged summary prints mean EE errors, solve time, convergence, iterations.

## 3. Recent Test Result (Stage C Only)

Comparison script: `matlab/compare_stageC_only.m` using `logToCompare/log_method1_ppForIk.mat`

| Metric | Method 1 (ppForIk) | Method 5 (pureMPC) |
| --- | --- | --- |
| Mean EE position error | 129 mm | 2.31 m |
| Max EE position error | 343 mm | 4.07 m |
| RMS EE position error | 165 mm | 2.65 m |
| Mean EE orientation error | n/a | 1.48 rad |
| Mean solve time | ~? (from log) | 3.59 s (≈0.28 Hz) |
| MPC convergence | n/a | 100 % (but slow) |

Observations:
- MPC converged every step but produced very large tracking errors; base motion minimal while arm saturated.
- Solve times (~3.6 s) far exceed 10 Hz objective, even with analytic gradients.

## 4. Known Issues / Next Actions

- Tune posture weights & timing (progress variable or resampling) to prevent arm-only solutions.
- Revisit constraints (wheel-speed, collision) once performance stabilises.
- Investigate cost scaling and reference timing to reduce 3+ s solve times.
- Re-run Stage C comparison after tuning to quantify improvements.
