# Stage B & Stage C Path Findings (10&nbsp;Hz)

## Overview
- Default control rate has been moved to **10&nbsp;Hz**. All staged runs, animations, and plots referenced here were generated with that setting.
- Reference run: `results/20251010_155837_staged_10hz_legends3/log_staged_ppForIk.mat`
- Animation: `results/20251010_155837_staged_10hz_legends3/anim/staged_run.mp4`
- EE comparison plot: `results/20251010_155837_staged_10hz_legends3/analysis/ee_path_compare.png`

## Terminology
- **JSON desired EE path** (`1_pull_world_scaled.json`): Original straight + arc path (148 poses).
- **Base ribbon**: SE(2) path the chassis should follow; refined in Stage B/C (Hybrid A*, RS, clothoids).
- **Stage C reference EE path** (`log.stageLogs.stageC.referenceInitialIk.eePositions`): Arm solution after re-running GIK with refined base locked.
- **Stage C executed EE path** (`log.stageLogs.stageC.eePositions`): Final arm motion after pure pursuit executes the base ribbon.

## Pipeline (textual flowchart)
```
JSON desired EE path
    ↓
Stage A – arm ramp, base frozen (GIK)
    ↓
Stage B – base alignment
    • Hybrid A* → base states
    • RS shortcut + clothoid smoothing → base ribbon
    • GIK re-run for Stage B log
    ↓
Stage C planning reference
    • First GIK pass (base free) → logRef
    • Stage C base refinement (RS/clothoid)
    • Re-run GIK with refined base locked → referenceInitialIk
    ↓
Stage C execution
    • Pure pursuit integrates base ribbon
    • GIK with executed base locked → final EE path
    ↓
Logging + animation overlays
```

## Observations (staged_10hz_legends3)
- Stage C reference vs JSON: mean **0.161&nbsp;m**, max **1.078&nbsp;m**.
- Stage C executed vs JSON: mean **0.096&nbsp;m**, max **0.165&nbsp;m**.
- Stage C executed closely follows Stage C reference but both diverge from the JSON during obstacle clearance.
- Stage B executed base retains zig-zags when RS shortcut cannot replace them; those produce jagged Stage C reference paths.
- Animation legend now shows desired EE (white dashed), Stage C reference EE (magenta dash-dot), Stage C executed EE (green), Stage C executed base (blue), Stage B executed base (magenta dotted), obstacle safety zones (both discs labeled).

## Recommendations
### Stage B ribbon
1. **Relax clearance margins** (lower `StageBHybridSafetyMargin`, `DistanceMargin`) to reduce detours.
2. **RS tuning**: consider `allowReverse=true`, lower `lambdaCusp`, or reduce iterations to prevent overfit.
3. **Clothoid smoothing**: increase discretization distance (e.g., 0.05–0.08) for gentler smoothing.

### Stage C refinement & execution
1. Disable `StageCUseBaseRefinement` when Stage B ribbon is already smooth.
2. Use softer RS defaults in Stage C (`allowReverse=true`, lower `lambdaCusp`) to preserve the JSON shape.
3. Tune pure pursuit lookahead values for 10&nbsp;Hz to soften curvature; adjust `accel_limit` and `jerk_limit` accordingly.

### Diagnostics
- Plot Stage B `pathStates`, `execBaseStates`, and Stage C `referenceBaseStates` to localize jagged segments.
- Compare `referenceInitialIk.eePositions` with `referenceTrajectory.EndEffectorPositions` to quantify drift caused by base refinement.
- Use `plotJsonPath` to inspect any new trajectory prior to planning.
- Monitor per-frame EE error (displayed in animation HUD) to detect problematic segments.

### Future Avenues
- Joint-space smoothing/optimization after Stage C GIK to reduce jerk while respecting the refined ribbon.
- Adaptive base refinement that only accepts RS/clothoid edits when they significantly improve path length/curvature.
- Collision-aware Stage C GIK to encourage a closer match to the JSON path without relying on large margins.
- Increase controller rate (>10&nbsp;Hz) if hardware allows; reduces discretization errors in both base and arm motions.

