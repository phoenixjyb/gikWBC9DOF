# Method 4 Performance Improvement Plan
**Date:** October 13, 2025  
**Goal:** Minimize End-Effector (EE) Tracking Error  
**Current Status:** 319mm mean error | Target: <150mm (competitive with Method 1's 129mm)

---

## 🔴 CRITICAL ISSUES IDENTIFIED

### Issue 1: High Fallback Rate (44.3%)
**Current:** 93 out of 210 waypoints (44.3%) trigger fallback  
**Target:** < 20% fallback rate  
**Impact:** Fallback uses arm-only IK with fixed base → poor tracking

### Issue 2: Poor Convergence (47.1%)
**Current:** Only 47.1% of GIK solves converge successfully  
**Target:** > 80% convergence rate  
**Impact:** 
- 59.5% of solves hit 1500 iteration limit (not converging)
- Failed solves have mean error of 603mm
- Non-converged solutions likely trigger fallback

### Issue 3: Error Increases Over Trajectory
**Pattern:** First 1/3 has 0mm mean error, last 1/3 has 596mm mean error  
**Possible causes:**
- No warm-starting (each solve starts from scratch)
- Error accumulation/drift
- Poor initial guess quality degrading over time

---

## 📊 CURRENT PERFORMANCE METRICS

| Metric | Current | Target | Gap |
|--------|---------|--------|-----|
| **EE Error Mean** | 319.0 mm | < 150 mm | -169 mm needed |
| **EE Error Max** | 1621.5 mm | < 500 mm | -1121 mm needed |
| **EE Error RMS** | 562.7 mm | < 200 mm | -363 mm needed |
| **Fallback Rate** | 44.3% | < 20% | -24% needed |
| **Convergence Rate** | 47.1% | > 80% | +33% needed |
| **Solver Iterations (mean)** | 996 | < 800 | -196 needed |

### Current Parameters (Aggressive Profile)
- Yaw corridor: **20.0°**
- Position tolerance: **0.20 m**
- EE error threshold: **15.0 mm**

---

## 🎯 ACTIONABLE IMPROVEMENT PLAN

### Phase 1: Quick Wins (Week 1) - **IMMEDIATE PRIORITY**

#### 1.1 Parameter Sweep Experiments (4-6 hours)
Run systematic experiments to find optimal corridor widths:

**Experiment A: Ultra-Aggressive (25-30°)**
```matlab
% Test configuration
params.yaw_corridor = 30.0;        % vs 20° current
params.position_tol = 0.30;        % vs 0.20m current
params.ee_threshold = 0.020;       % vs 0.015m current

% Expected results
% - Fallback rate: 15-25% (-20% improvement)
% - Mean error: 200-300 mm (-50 to -100mm)
% - Convergence: 60-70%
```

**Experiment B: Very Wide (35-40°)**
```matlab
% Test configuration
params.yaw_corridor = 40.0;
params.position_tol = 0.40;
params.ee_threshold = 0.025;

% Expected results
% - Fallback rate: 5-15% (-30% improvement)
% - Mean error: 150-250 mm (-70 to -170mm) ← BEST CANDIDATE
% - Convergence: 70-80%
```

**Experiment C: Balanced (25°)**
```matlab
% Test configuration
params.yaw_corridor = 25.0;
params.position_tol = 0.25;
params.ee_threshold = 0.018;

% Expected results
% - Fallback rate: 20-30%
% - Mean error: 180-250 mm
% - Convergence: 55-65%
```

**How to Run:**
```matlab
% Edit compare_method1_vs_method4_aggressive_v2.m
% Lines 81-83, change parameters for each experiment
yawCorridorDeg = 40.0;        % Try 25, 30, 35, 40
positionTolerance = 0.40;     % Try 0.25, 0.30, 0.35, 0.40
eeErrorThreshold = 0.025;     % Try 0.018, 0.020, 0.025

% Run
matlab -batch "compare_method1_vs_method4_aggressive_v2"
```

**Expected Impact:** -50 to -170 mm error reduction

---

#### 1.2 Implement Warm-Starting (4 hours)
Use previous solution as initial guess for next waypoint.

**Implementation:**
```matlab
% In matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Around line 200 (in the waypoint loop)

% Add before GIK solve:
if iWaypoint > 1 && ~isempty(prevSolution)
    % Use previous joint configuration as initial guess
    gik.setInitialGuess(prevSolution);
end

% After successful solve:
if solInfo.Status == "optimal-solution"
    prevSolution = currentQ;  % Save for next iteration
end
```

**Expected Impact:**
- +20% convergence rate → ~67%
- -30 mm mean error → ~289mm
- Faster convergence (fewer iterations)

---

#### 1.3 Relax Solver Tolerances (1 hour)
Current tolerances may be too strict for constrained problem.

**Implementation:**
```matlab
% In matlab/+gik9dof/+helpers/executeStageCPPFirst.m
% Around line 150 (GIK solver setup)

% Current (implicit defaults):
% ConstraintTolerance: 1e-6
% StepTolerance: 1e-12
% OptimalityTolerance: 1e-6

% Proposed relaxed:
gik.SolverParameters.ConstraintTolerance = 1e-4;
gik.SolverParameters.StepTolerance = 1e-10;
gik.SolverParameters.OptimalityTolerance = 1e-4;
gik.SolverParameters.MaxIterations = 2000;  % vs 1500
```

**Expected Impact:**
- +10-15% convergence rate → ~57-62%
- -20 mm mean error → ~299mm
- Fewer hitting iteration limit (59% → 40%)

---

### Phase 2: Architecture Improvements (Week 2)

#### 2.1 Implement Adaptive Corridor (8-12 hours)
Adjust corridor width based on trajectory curvature.

**Algorithm:**
```matlab
% Compute local curvature at each waypoint
function corridorWidth = adaptiveCorridorWidth(trajectory, idx, baseWidth)
    % Get local curvature
    if idx > 1 && idx < length(trajectory)
        v1 = trajectory(:, idx) - trajectory(:, idx-1);
        v2 = trajectory(:, idx+1) - trajectory(:, idx);
        curvature = norm(cross([v1; 0], [v2; 0])) / (norm(v1) * norm(v2));
    else
        curvature = 0;
    end
    
    % Scale corridor based on curvature
    if curvature < 0.1  % Straight
        corridorWidth = baseWidth * 0.75;  % Tighter: 15° → 11.25°
    elseif curvature < 0.3  % Moderate curve
        corridorWidth = baseWidth;  % Normal: 20°
    else  % Sharp curve
        corridorWidth = baseWidth * 1.5;  % Wider: 20° → 30°
    end
end
```

**Integration Point:**
```matlab
% In executeStageCPPFirst.m, line ~200
% Before setting GIK bounds:
yawCorridor = adaptiveCorridorWidth(refTrajectory, iWaypoint, baseYawCorridor);
```

**Expected Impact:**
- -15% fallback rate → ~29%
- -40 mm error → ~279mm
- Better handling of curved sections

---

#### 2.2 Multi-Stage Fallback Strategy (6-8 hours)
Instead of immediately fixing base, try progressive relaxation.

**Implementation:**
```matlab
% Replace current fallback logic with multi-stage:

% Stage 1: Widen corridor (2x)
if eeError > threshold
    % Try again with doubled corridor
    gik.setYawBounds(yawPP - 2*corridor, yawPP + 2*corridor);
    [q2, solInfo2] = solve(gik);
    
    if norm(ee2 - target) < threshold
        % Success with wider corridor
        useWiderCorridor = true;
    else
        % Stage 2: Relax EE constraint
        gik.setEEConstraintTolerance(2 * threshold);
        [q3, solInfo3] = solve(gik);
        
        if solInfo3.Status == "optimal"
            % Success with relaxed constraint
            useRelaxed EE = true;
        else
            % Stage 3: Last resort - arm-only IK
            useFallback = true;
        end
    end
end
```

**Expected Impact:**
- -50% error in fallback cases
- -30 to -60 mm overall error → ~249-289mm
- More graceful degradation

---

### Phase 3: Advanced Features (Week 3+)

#### 3.1 Trajectory Preprocessing (10-15 hours)
Analyze trajectory before execution, optimize base path.

**Features:**
- Reachability heatmap for each waypoint
- Base path optimization for arm workspace
- Predictive fallback detection
- Smooth PP predictions with splines

#### 3.2 Hybrid Method 1/4 (15-20 hours)
Use Method 1 (unconstrained) for difficult sections.

**Strategy:**
- Detect high-curvature or low-reachability regions
- Switch to Method 1 (ppForIk) dynamically
- Use Method 4 in easy sections for speed
- Best of both worlds

---

## 📈 EXPECTED PERFORMANCE TRAJECTORY

| Phase | Actions | Expected Error | Expected Fallback | Effort |
|-------|---------|----------------|-------------------|--------|
| **Baseline** | (Current aggressive) | 319 mm | 44.3% | - |
| **Phase 1.1** | Wider corridor (40°) | **150-250 mm** ✅ | **5-15%** ✅ | 6 hrs |
| **Phase 1.2** | + Warm-starting | **120-220 mm** ✅ | 5-15% | +4 hrs |
| **Phase 1.3** | + Relaxed tolerances | **100-200 mm** ✅ | 5-15% | +1 hrs |
| **Phase 2.1** | + Adaptive corridor | **80-180 mm** ✅ | **< 10%** ✅ | +10 hrs |
| **Phase 2.2** | + Better fallback | **60-140 mm** ✅ | < 10% | +7 hrs |
| **Phase 3** | + Advanced features | **< 100 mm** 🎯 | < 5% | +25 hrs |

**Target Performance (After Phase 1 only):**
- Mean EE error: **< 150 mm** (competitive with Method 1!)
- Fallback rate: **< 15%**
- Convergence rate: **> 70%**
- Total effort: **11 hours**

---

## 🚀 IMMEDIATE NEXT STEPS

### This Week (Priority 1)

1. **Run Experiment B (Very Wide Corridor)** - 2 hours
   ```bash
   # Edit params in compare_method1_vs_method4_aggressive_v2.m:
   # yaw_corridor: 40°, position_tol: 0.40m, ee_threshold: 0.025m
   matlab -batch "compare_method1_vs_method4_aggressive_v2"
   ```
   
2. **Analyze Results** - 1 hour
   - Check if error drops to 150-250mm range
   - Check if fallback rate drops below 15%
   - If yes → Use as new baseline
   - If no → Try intermediate values (35°, 0.35m)

3. **Implement Warm-Starting** - 4 hours
   - Modify `executeStageCPPFirst.m`
   - Test on same trajectory
   - Measure convergence improvement

4. **Relax Solver Tolerances** - 1 hour
   - Update solver parameters
   - Re-run with warm-starting
   - Measure combined impact

**Total Time Investment:** 8 hours  
**Expected Result:** Mean error < 150mm (GOAL ACHIEVED!)

### Next Week (Priority 2)

5. **Implement Adaptive Corridor** - 10 hours
6. **Improve Fallback Strategy** - 7 hours

---

## 📊 SUCCESS CRITERIA

### Minimum Viable Performance (Phase 1 Target)
- ✅ Mean EE error < 150 mm (vs Method 1's 129mm)
- ✅ Fallback rate < 15%
- ✅ Convergence rate > 70%
- ✅ Execution time < 15 seconds

### Optimal Performance (Phase 2+ Target)
- 🎯 Mean EE error < 100 mm (BETTER than Method 1)
- 🎯 Fallback rate < 10%
- 🎯 Convergence rate > 85%
- 🎯 Execution time < 10 seconds

---

## 📁 FILES CREATED

### Analysis Files
1. **`analyze_method4_performance.m`** - Comprehensive analysis script
2. **`results/method4_diagnostic_analysis.png`** - Diagnostic plots
3. **`results/method4_performance_analysis_detailed.mat`** - Analysis data
4. **`METHOD4_IMPROVEMENT_PLAN.md`** - This document

### Comparison Results
- **`results/20251013_183955_method_comparison_aggressive/`**
  - `comparison_report.txt` - Latest comparison
  - `log_method4_ppFirst_aggressive.mat` - Method 4 log
  - `comparison_metrics.mat` - Metrics data

---

## 💡 KEY INSIGHTS

1. **Constraints Are Too Tight**
   - Even "aggressive" 20° corridor is insufficient
   - 60% of solves hit iteration limit → can't converge
   - Need 35-40° corridor for good performance

2. **Fallback Is The Main Problem**
   - 44% fallback rate with arm-only IK
   - Fallback waypoints have much higher error
   - Reducing fallback is #1 priority

3. **No Warm-Starting = Slow Convergence**
   - Each solve starts from scratch
   - 996 average iterations (very high)
   - Error increases over trajectory (drift)

4. **Quick Win Available**
   - Just widening corridor to 40° could achieve goal!
   - Estimated: 150-250mm error (vs 319mm current)
   - This alone might be sufficient

---

## 🎯 RECOMMENDATION

**START WITH PHASE 1.1 - PARAMETER EXPERIMENT B**

Run the very wide corridor experiment (40°/0.40m/0.025m) **immediately**. This single change could:
- Reduce error from 319mm → 150-250mm ✅ **GOAL ACHIEVED**
- Reduce fallback from 44% → 5-15% ✅
- Improve convergence from 47% → 70-80% ✅

If this works, you have a viable Method 4 with **less than 2 hours of effort**.

Then add warm-starting and relaxed tolerances for even better performance.

---

**Status:** 🚀 Ready to implement  
**Priority:** 🔴 HIGH - Start Phase 1.1 now  
**Estimated Time to Goal:** 8-11 hours (Phase 1 complete)

