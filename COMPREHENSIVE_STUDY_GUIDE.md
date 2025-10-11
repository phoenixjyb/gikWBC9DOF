# Comprehensive Parametric Study Guide

## Overview

This guide describes the comprehensive parametric study framework for optimizing the 9-DOF whole-body chassis controller. The framework evaluates parameter combinations across **6 rigorous criteria** to find optimal configurations.

## Evaluation Criteria

### 1. EE Reference Path Smoothness
- **What**: Evaluates smoothness of Stage C reference end-effector trajectory (from initial IK pass)
- **Metrics**: RMS jerk (m/s³), kink count (sharp angle changes > 30°)
- **Target**: RMS jerk < 5 m/s³, kinks < 3
- **Weight**: 15%

### 2. Base Reference Path Smoothness
- **What**: Evaluates smoothness of Stage C reference base trajectory (from pure pursuit)
- **Metrics**: RMS jerk (m/s³), kink count
- **Target**: RMS jerk < 5 m/s³, kinks < 3
- **Weight**: 15%

### 3. EE Tracking Accuracy
- **What**: Measures how well the executed end-effector position matches JSON waypoints
- **Metrics**: Mean error (m), max error (m), RMS error (m)
- **Target**: Mean < 0.05m, max < 0.15m
- **Weight**: 25% (highest)

### 4. Collision Avoidance
- **What**: Distinguishes actual disc penetration from safety margin violations
- **Metrics**: Actual intrusions (count), margin intrusions (count), max penetration depth
- **Penalty**: Actual intrusion = 0.5 per occurrence, margin = 0.05 per occurrence
- **Weight**: 20%

### 5. Cusp Count and Severity
- **What**: Counts direction reversals in Stage B path and measures angle severity
- **Metrics**: Cusp count, max cusp angle (degrees), mean cusp angle
- **Penalty**: 0.2 per cusp + 0.3 × (maxSeverity/180°)
- **Weight**: 15%

### 6. Sideways Movement Violations
- **What**: Validates differential drive kinematic constraints (no sideways motion)
- **Metrics**: Violation percentage, max alignment error (degrees), mean lateral velocity
- **Detection**: Flags when body-frame velocity angle > 10° and speed > 0.01 m/s
- **Weight**: 10%

## Parameter Space

### Stage B (Path Planning)
| Parameter | Values | Description |
|-----------|--------|-------------|
| `SafetyMargin` | [0.05, 0.08, 0.10, 0.12, 0.15] m | Obstacle clearance buffer |
| `LambdaCusp` | [0.5, 1.0, 1.5, 2.0, 3.0] | Cusp penalty weight |
| `MaxIters` | [100, 200, 400, 600] | Reeds-Shepp iterations |
| `AllowReverse` | [true, false] | Enable/disable reversing |
| `ClothoidDiscretization` | [0.03, 0.05, 0.08, 0.10] m | Smoothing resolution |

**Total Stage B configs**: 5 × 5 × 4 × 2 × 4 = **800**

### Stage C (Tracking)
| Parameter | Values | Description |
|-----------|--------|-------------|
| `Lookahead` | [0.4, 0.6, 0.8, 1.0, 1.2] m | Pure pursuit lookahead |
| `AccelLimit` | [0.6, 0.8, 1.0, 1.2] m/s² | Base acceleration limit |
| `HeadingKp` | [0.8, 1.0, 1.2, 1.5] | Heading controller gain |

**Total Stage C configs**: 5 × 4 × 4 = **80**

### Combined Grid
**Total configurations**: 800 × 80 = **64,000** configs

⚠️ **Estimated Runtime**: 27-53 hours @ 30-60s per config

> **Recommendation**: Start with reduced grid (e.g., 3 values per param → ~1,000 configs, 8-16 hours)

## File Structure

### Core Evaluation Functions
```
matlab/+gik9dof/
├── comprehensiveEvaluation.m      # Main 6-criteria evaluation
├── evaluatePathSmoothness.m       # Jerk and kink analysis
├── evaluateChassisConstraints.m   # Sideways motion detection
└── evaluateCollisionIntrusion.m   # Disc vs margin separation
```

### Scripts
```
run_comprehensive_chassis_study.m    # Main parametric sweep (full grid)
test_comprehensive_evaluation.m      # Quick test (4 configs)
generate_comprehensive_animations.m  # Post-hoc animation generation
```

## Usage

### Step 1: Quick Test (5-10 minutes)
```matlab
test_comprehensive_evaluation
```
This validates the evaluation framework with 4 diverse configurations:
- Tuned Baseline (current parameters)
- Conservative (high safety, low cusp penalty)
- Aggressive (low safety, high cusp penalty)
- High Resolution (fine discretization, many iterations)

**Expected Output**:
- 4 logs with comprehensive evaluation reports
- Score comparison table
- Detailed metrics per criterion
- Best config per criterion

### Step 2: Full Study (or Reduced Grid)
```matlab
% OPTION A: Edit parameter grid in run_comprehensive_chassis_study.m
% Reduce to 3 values per param for ~1,000 configs (8-16 hours)

% OPTION B: Run full grid (64,000 configs, 27-53 hours)
run_comprehensive_chassis_study
```

**Interactive Prompts**:
1. Confirms parameter space size and estimated time
2. Asks for user confirmation before starting

**Progress Logging**:
- Displays config ID, parameters, and scores for each run
- Saves checkpoints every 10 configs (crash recovery)
- Final results saved to `results/comprehensive_study_TIMESTAMP/`

**Output Files**:
```
results/comprehensive_study_TIMESTAMP/
├── comprehensive_results.mat        # Full results struct
├── results_table.csv                # Sortable table (Excel-friendly)
├── checkpoint_XXXX.mat              # Periodic backups
└── logs/
    ├── config_0001.mat
    ├── config_0002.mat
    └── ...
```

### Step 3: Analyze Results

#### Top 20 Configurations
Automatically displayed at the end of the study:
```
[Rank  1] Config 4321 | Score: 0.873 | Violations: 0
           SM=0.10, LC=1.0, MI=200, AR=1, CD=0.08, LH=0.8, AL=0.8, HK=1.0
           Scores: [0.92, 0.89, 0.95, 1.00, 0.85, 0.92]
```

#### Results Table (CSV)
Open `results_table.csv` in Excel/MATLAB:
- Sort by `OverallScore` (descending)
- Filter by `ViolationCount == 0`
- Compare individual criterion scores

#### Best Per Criterion
Displayed at end of study:
```
[C1] EE Ref Smoothness:   Config 1234 (score 0.950)
[C2] Base Ref Smoothness: Config 2345 (score 0.938)
...
```

### Step 4: Generate Animations
```matlab
generate_comprehensive_animations
```

**Options**:
1. Top N by overall score (default: top 10)
2. Specific config IDs (comma-separated)
3. Best per criterion (6 configs)
4. All configs with 0 violations

**Output**:
- MP4 files in `STUDY_DIR/animations/`
- Filenames include config ID, score, and all parameters

## Scoring System

### Individual Criterion Score
Each criterion produces a score in [0, 1]:
- **1.0** = Perfect (meets all targets)
- **0.7-1.0** = Good (passing threshold)
- **0.0-0.7** = Violation (fails criterion)

### Overall Score
Weighted average:
```
OverallScore = 0.15×C1 + 0.15×C2 + 0.25×C3 + 0.20×C4 + 0.15×C5 + 0.10×C6
```

### Violation Tracking
Any criterion with score < 0.7 counts as a violation:
- **0 violations**: Excellent configuration
- **1-2 violations**: Acceptable with trade-offs
- **3+ violations**: Poor configuration

## Implementation Details

### comprehensiveEvaluation.m
- **Input**: Log struct, trajectory struct, obstacle discs, options
- **Output**: Report struct with 6 criterion results + overall score
- **Key Features**:
  - Extracts Stage B and Stage C logs
  - Calls specialized evaluation functions
  - Applies weighted scoring
  - Identifies violations (score < 0.7)

### evaluatePathSmoothness.m
- **Algorithm**:
  1. Compute 1st derivative (velocity): `diff(path) / dt`
  2. Compute 2nd derivative (acceleration): `diff(velocity) / dt`
  3. Compute 3rd derivative (jerk): `diff(acceleration) / dt`
  4. Detect kinks: Normalized velocity vectors with angle > 30°
- **Smoothness Score**: `0.6 × exp(-rmsJerk/5) + 0.4 × exp(-kinkCount/3)`

### evaluateChassisConstraints.m
- **Algorithm**:
  1. Transform world-frame velocity to body frame via rotation matrix
  2. Compute alignment error: `atan2(|vy_body|, |vx_body|)`
  3. Flag violation if alignment > 10° and speed > 0.01 m/s
- **Metrics**: Violation count, percentage, max/mean lateral velocity, RMS alignment error

### evaluateCollisionIntrusion.m
- **Algorithm**:
  1. For each pose, compute distance to all obstacle surfaces
  2. Categorize: actual intrusion (dist < 0), margin intrusion (0 ≤ dist < margin), safe
- **Penalty**: Actual intrusion heavily penalized (0.5 per), margin lightly (0.05 per)

## Interpreting Results

### High Overall Score (> 0.80)
✅ Configuration achieves good balance across all criteria
- Check individual criterion scores for any weak spots
- Generate animation to visually confirm smoothness

### Medium Overall Score (0.60-0.80)
⚠️ Configuration has 1-2 violations
- Identify which criteria failed (score < 0.7)
- Examine trade-offs (e.g., collision safety vs smoothness)
- May be acceptable depending on application priorities

### Low Overall Score (< 0.60)
❌ Configuration has 3+ violations
- Multiple failure modes present
- Review parameter choices for extremes
- Unlikely to be usable

### Pareto Front Analysis
When no single config dominates all criteria:
1. Extract top 20 configs
2. Plot each criterion score in parallel coordinates
3. Identify trade-off patterns (e.g., safety vs smoothness)
4. Select based on application priorities

## Troubleshooting

### Study Crashes Mid-Run
- Resume from checkpoint: Load `checkpoint_XXXX.mat`
- Restart study with `configID > last_checkpoint_id`

### All Configs Have Similar Scores
- Parameter ranges may be too narrow
- Expand grid to explore more diverse parameter space
- Check if trajectory is too simple (not stressing parameters)

### Many Configs Fail with Errors
- Check MATLAB path includes `matlab/` directory
- Verify URDF and JSON files exist
- Run `test_comprehensive_evaluation.m` first to validate

### Animations Take Too Long
- Reduce `TOP_N` (generate fewer animations)
- Increase `SAMPLE_STEP` (skip more frames)
- Lower `FRAME_RATE` (e.g., 10 fps instead of 20)

## Next Steps

After completing the study:

1. **Document Best Configurations**
   - Update `staged_path_findings.md` with top 5 configs
   - Include parameter values and achieved metrics

2. **Analyze Parameter Sensitivity**
   - Plot each parameter vs each criterion score
   - Identify which parameters most impact which criteria

3. **Cross-Validation**
   - Test top 3 configs on different trajectories
   - Verify generalization beyond `1_pull_world_scaled.json`

4. **Integration**
   - Update default parameters in `defaultReedsSheppParams.m`, `chassis_profiles.yaml`
   - Freeze parameters for production use

5. **Further Investigation**
   - If cusps persist despite low `LambdaCusp`, investigate Hybrid A* planner
   - If EE tracking errors remain, examine GIK solver configuration
   - If kinky reference paths continue, analyze pure pursuit lookahead logic

---

**Author**: GitHub Copilot  
**Date**: 2025-10-10  
**Version**: 1.0
