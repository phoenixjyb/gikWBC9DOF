# Phase 2B Implementation Plan: Arm-Aware Pure Pursuit

## Executive Summary

**Objective:** Make Pure Pursuit controller arm-aware to account for arm configuration constraints, further reducing tracking error from 1.2mm to sub-millimeter levels.

**Expected Impact:**
- Mean EE Error: 1.2mm → <0.5mm (-60%)
- Max EE Error: 104mm → <50mm (-50%)
- Convergence: 74.3% → >80% (+8%)
- Fallback: 0.5% → <0.2% (-60%)

**Implementation Time:** 14 hours (2 work days)

---

## Background

### Phase 2A Success
Phase 2A achieved spectacular results (99.8% error reduction from 507mm to 1.2mm) by using Orientation+Z priority nominal pose generation. This produces much better base paths that respect manipulation requirements.

### Why Phase 2B?
Current Pure Pursuit predicts base pose using only:
- **XY position** from lookahead point on base path
- **Yaw** from path tangent direction

This ignores:
- Arm configuration constraints (joint limits, singularities)
- EE orientation requirements
- Workspace boundaries

**Phase 2B** makes PP arm-aware by incorporating arm reachability into the prediction.

---

## Core Concept: Arm-Aware Lookahead

### Standard Pure Pursuit (Current)
```
Lookahead Point → Base (x, y, θ) → GIK Solve
                   ↑
                   Pure geometry
```

### Arm-Aware Pure Pursuit (Phase 2B)
```
Lookahead Point → Candidate Base Poses → Score by Arm Config → Best Pose → GIK Solve
                   ↑                       ↑
                   Multiple samples        Arm reachability check
```

### Scoring Function
For each candidate base pose `(x, y, θ)`:
1. **Quick IK check**: Can arm reach target EE pose?
2. **Joint deviation**: How far from nominal arm configuration?
3. **Manipulability**: Is pose near singularities?
4. **Score**: `score = α * reachability + β * (1 - joint_deviation) + γ * manipulability`

---

## Implementation Steps

### Step 1: Create Arm-Aware Lookahead Function (3 hours)
**File:** `matlab/+gik9dof/computeArmAwareLookahead.m`

**Inputs:**
- `ppFollower`: Pure Pursuit controller
- `robot`: Robot model
- `q_current`: Current configuration
- `T_ee_target`: Target EE pose
- `options`: Sampling parameters

**Outputs:**
- `basePose_predicted`: Arm-aware predicted base pose [x, y, θ]
- `score`: Quality score (higher = better)
- `diagInfo`: Diagnostic information

**Algorithm:**
```matlab
function [basePose_predicted, score, diagInfo] = computeArmAwareLookahead(...)
    % 1. Get standard PP prediction
    [v, w] = ppFollower(q_current(1:3)');
    basePose_pp = predictBasePose(q_current, v, w, dt);
    
    % 2. Sample candidate poses around PP prediction
    candidates = sampleCandidatePoses(basePose_pp, options);
    
    % 3. Score each candidate
    scores = zeros(size(candidates, 1), 1);
    for i = 1:size(candidates, 1)
        % Quick IK reachability check
        q_test = [candidates(i, :)'; q_current(4:9)];
        [~, solInfo] = ik(EE_name, T_ee_target, q_test);
        
        % Score = reachability + arm_quality
        scores(i) = computeScore(solInfo, q_test, options);
    end
    
    % 4. Select best candidate
    [score, best_idx] = max(scores);
    basePose_predicted = candidates(best_idx, :);
end
```

**Parameters:**
- `NumCandidates`: Number of poses to sample (default: 5)
- `PositionRadius`: Radial sampling around PP pose (default: 0.1m)
- `YawRange`: Angular sampling around PP yaw (default: ±15°)
- `ReachabilityWeight`: α = 1.0
- `JointDeviationWeight`: β = 0.5
- `ManipulabilityWeight`: γ = 0.3

---

### Step 2: Integrate into Enhanced Executor (2 hours)
**File:** `matlab/+gik9dof/runStageCPPFirst_enhanced.m`

**Modifications:**
1. Add Phase 2B option flags:
```matlab
options.UseArmAwarePP (1,1) logical = false
options.PPNumCandidates (1,1) double = 5
options.PPPositionRadius (1,1) double = 0.10
options.PPYawRange (1,1) double = deg2rad(15)
```

2. Update base prediction in control loop (around line 200):
```matlab
if options.UseArmAwarePP
    % Phase 2B: Arm-aware lookahead
    [basePose_k, score_k, diagInfo] = gik9dof.computeArmAwareLookahead(...
        ppFollower, robot, q_current, T_ee_k, ...
        'NumCandidates', options.PPNumCandidates, ...
        'PositionRadius', options.PPPositionRadius, ...
        'YawRange', options.PPYawRange);
    log.armAwareScores(k) = score_k;
else
    % Standard PP prediction
    [v_cmd, w_cmd] = ppFollower(q_current(1:3)');
    basePose_k = predictBasePose(q_current, v_cmd, w_cmd, dt);
end
```

3. Add new diagnostic arrays:
```matlab
log.armAwareScores = zeros(1, nWaypoints);
log.candidateEvaluations = zeros(1, nWaypoints);
```

---

### Step 3: Create Phase 2B Test Script (1 hour)
**File:** `test_method4_phase2b.m`

Compare three configurations:
1. **Baseline:** Phase 1 (standard nominal + all Phase 1 improvements)
2. **Phase 2A:** Orientation+Z nominal pose
3. **Phase 2B:** Phase 2A + Arm-aware Pure Pursuit

**Success Criteria (Phase 2B vs Phase 2A):**
- ✅ Mean Error: <0.5mm (target: 1.2mm → 0.5mm, -58%)
- ✅ Max Error: <50mm (target: 104mm → 50mm, -52%)
- ✅ Convergence: >80% (target: 74.3% → 80%, +8%)
- ✅ Fallback: <0.2% (target: 0.5% → 0.2%, -60%)

Minimum for success: **2/4 criteria** (must include mean error)

---

### Step 4: Run Smoke Test (15 minutes)
**File:** `test_phase2b_smoke.m`

Test on first 10 waypoints to verify:
- Arm-aware lookahead works without errors
- Candidate sampling is reasonable
- Scoring function produces valid results
- No significant performance degradation

---

### Step 5: Run Full Test (30 minutes)
Execute `test_method4_phase2b` on all 210 waypoints.

**Expected Timeline:**
- Test execution: ~15 minutes (similar to Phase 2A)
- Results analysis: ~15 minutes

---

### Step 6: Analysis and Visualization (3 hours)
If Phase 2B passes (≥2/4 criteria):

1. **Generate plots:**
   - EE tracking error comparison (Baseline → Phase 2A → Phase 2B)
   - Arm-aware score distribution
   - Candidate evaluation statistics
   - Convergence timeline

2. **Generate animations:**
   - Phase 2B robot motion
   - Side-by-side Phase 2A vs Phase 2B

3. **Document results:**
   - Performance metrics table
   - Key findings
   - Next steps recommendation

---

### Step 7: Tuning (if needed) (5 hours)
If Phase 2B fails some criteria:

**Tuning Parameters:**
1. **Candidate sampling:**
   - Increase `NumCandidates` (5 → 10)
   - Adjust `PositionRadius` (0.10 → 0.15m)
   - Adjust `YawRange` (±15° → ±20°)

2. **Scoring weights:**
   - Increase `ReachabilityWeight` (emphasize feasibility)
   - Decrease `JointDeviationWeight` (allow more arm motion)
   - Adjust `ManipulabilityWeight` (avoid singularities)

3. **Integration settings:**
   - Combine with tighter corridor constraints
   - Adjust warm-starting parameters

---

## Risk Assessment

### Low Risk
- ✅ Arm-aware concept proven in literature
- ✅ Building on Phase 2A success
- ✅ Can fallback to Phase 2A if fails
- ✅ Modular implementation (easy to debug)

### Medium Risk
- ⚠️ IK calls per waypoint: 5-10 candidates × 210 waypoints = 1000-2000 IKs
  - **Mitigation:** Cache results, use fast IK (ikWeights)
- ⚠️ Sampling strategy may need tuning
  - **Mitigation:** Conservative defaults, empirical testing

### High Risk
- ⚠️ May not improve much beyond Phase 2A (1.2mm already excellent)
  - **Mitigation:** Accept Phase 2A as success, Phase 2B as bonus

---

## Success Metrics

### Excellent (4/4 criteria)
- Mean error <0.5mm
- Max error <50mm
- Convergence >80%
- Fallback <0.2%
- **Action:** Document success, consider Phase 3 (real robot validation)

### Good (3/4 criteria, including mean error)
- Significant improvement in at least 3 metrics
- **Action:** Document success, minor tuning optional

### Acceptable (2/4 criteria, including mean error)
- Mean error improved
- **Action:** Consider tuning or accept current state

### Needs Work (<2/4 or mean error not improved)
- Phase 2B not effective
- **Action:** Debug, tune, or accept Phase 2A as final

---

## Estimated Timeline

| Step | Task | Time | Cumulative |
|------|------|------|------------|
| 1 | Create arm-aware lookahead function | 3h | 3h |
| 2 | Integrate into executor | 2h | 5h |
| 3 | Create test script | 1h | 6h |
| 4 | Smoke test | 0.25h | 6.25h |
| 5 | Full test | 0.5h | 6.75h |
| 6 | Analysis & visualization | 3h | 9.75h |
| 7 | Tuning (if needed) | 5h | 14.75h |

**Total: ~15 hours (2 work days)**

---

## Phase 2B vs Phase 2A: Key Differences

| Aspect | Phase 2A | Phase 2B |
|--------|----------|----------|
| **Nominal Generation** | Orientation+Z weighted IK | Same (inherited) |
| **Pure Pursuit** | Standard (geometric) | **Arm-aware (scored candidates)** |
| **Base Prediction** | Path tangent + lookahead | **Sample + score + select** |
| **Arm Consideration** | Only in nominal | **Also in PP prediction** |
| **IK Calls per WP** | 1 (nominal) | **6-11 (nominal + candidates)** |
| **Expected Error** | 1.2mm (achieved) | **<0.5mm (target)** |

---

## Next Steps After Phase 2B

### If Successful (Error <0.5mm)
1. **Phase 3:** Real robot validation
2. Document complete method as "Method 4 Enhanced"
3. Compare with Methods 1-3 on benchmark

### If Marginal (Error 0.5-1.0mm)
1. Accept current performance
2. Focus on computational efficiency
3. Consider hybrid approaches

### If Not Improved (Error >1.0mm)
1. Accept Phase 2A as final (1.2mm is excellent!)
2. Investigate Phase 2B failure modes
3. Consider alternative improvements

---

## References

- Phase 2A results: `results/phase2a_orientation_z/`
- Implementation guide: `PHASE2A_IMPLEMENTATION_PLAN.md`
- Method 4 guide: `method4_Guide.md`

---

**Status:** Ready to implement
**Next Action:** Create `computeArmAwareLookahead.m` (Step 1)
**Estimated Completion:** 2 work days from start
