# Animation Generation Summary

**Date:** October 10, 2025  
**Script:** `generate_sweep_animations.m`

## Overview

Simplified animation generation script that leverages existing infrastructure (`animateStagedWithHelper`) to create videos from parameter sweep results.

## Usage

```matlab
% Generates animations for all configs in the most recent sweep
generate_sweep_animations
```

## What It Does

1. Loads `sweep_results.mat` from most recent SWEEP directory
2. Extracts embedded log structures for each configuration
3. Generates MP4 animations using existing `animateStagedWithHelper`
4. Saves to `animations/` subdirectory with descriptive filenames

## Output Files

**Location:** `results/20251010_211447_SWEEP_STAGEB_quick_test/animations/`

**Naming Convention:**
```
config{N}_margin{X}_cusp{Y}_iter{Z}.mp4

Where:
  N = config number (1-4)
  X = safety margin (0.10, 0.15)
  Y = lambda cusp (1.0, 2.0)
  Z = max iterations (200)
```

**Expected Files:**
- `config1_margin0.10_cusp1.0_iter200.mp4`
- `config2_margin0.10_cusp2.0_iter200.mp4`
- `config3_margin0.15_cusp1.0_iter200.mp4`
- `config4_margin0.15_cusp2.0_iter200.mp4`

## Animation Settings

**Performance Optimized:**
- Frame rate: 20 fps (vs 30 fps for validation tests)
- Sample step: 2 (every other frame)
- Figure scale: 0.5 (smaller window size)
- Estimated time: ~30-60 seconds per animation

**Visual Features:**
- Dual view (3D + top-down)
- Multiple path overlays:
  - Desired EE path (white dashed)
  - Stage C reference EE (magenta dash-dot)
  - Executed EE (green)
  - Stage B base (magenta dotted)
  - Executed base (blue)
- Stage boundaries marked
- Per-frame EE error HUD
- Obstacle visualization

## Comparison Guide

### What to Look For

**Path Smoothness:**
- Observe Stage B base ribbon (magenta dotted)
- Look for zig-zags or sharp turns
- Check for smooth curvature transitions

**Tracking Accuracy:**
- Compare executed EE (green) to desired (white dashed)
- Check HUD for per-frame error values
- Look for consistent tracking vs divergence

**Parameter Effects:**
- **Safety Margin (0.10 vs 0.15):**
  - 0.10m should have tighter paths near obstacles
  - 0.15m should show wider berths
  
- **Lambda Cusp (1.0 vs 2.0):**
  - 1.0 may show smoother shortcuts with gentle cusps
  - 2.0 may show more conservative paths avoiding cusps

## Quick View Commands

```bash
# Open animations folder
open results/20251010_211447_SWEEP_STAGEB_quick_test/animations/

# Play specific animation
open results/20251010_211447_SWEEP_STAGEB_quick_test/animations/config1_margin0.10_cusp1.0_iter200.mp4

# Compare all in QuickTime
open results/20251010_211447_SWEEP_STAGEB_quick_test/animations/*.mp4
```

## Metrics Correlation

Each animation corresponds to metrics in `SWEEP_summary_report.txt`:

| Config | Margin | Cusp | EE Mean | EE Max | Smoothness | Animation |
|--------|--------|------|---------|--------|------------|-----------|
| 1 | 0.10 | 1.0 | 0.0488m | 0.1332m | 1.281 | config1_*.mp4 |
| 2 | 0.10 | 2.0 | 0.0488m | 0.1332m | 1.281 | config2_*.mp4 |
| 3 | 0.15 | 1.0 | 0.0488m | 0.1332m | 1.281 | config3_*.mp4 |
| 4 | 0.15 | 2.0 | 0.0488m | 0.1332m | 1.281 | config4_*.mp4 |

**Note:** All configs achieved identical metrics in this test, suggesting parameters had minimal impact on this particular scenario. Visual comparison via animation may reveal subtle differences.

## Reusability

The script can be easily adapted for other sweep results:

```matlab
% Edit sweepDir variable to point to different sweep
sweepDir = 'results/YYYYMMDD_HHMMSS_SWEEP_*';

% Or it will auto-find the most recent SWEEP directory
```

## Integration with Workflow

**Step 1:** Run parameter sweep
```matlab
test_parameter_sweep  % Quick test
% OR
run_stageb_parameter_sweep('stageb', ...)  % Full sweep
```

**Step 2:** Generate animations
```matlab
generate_sweep_animations
```

**Step 3:** Visual comparison + metrics review
```bash
open results/.../animations/
cat results/.../SWEEP_summary_report.txt
```

**Step 4:** Select optimal config based on:
- Visual smoothness
- Quantitative metrics
- Scenario-specific requirements

## Future Enhancements

Potential improvements (not implemented):

1. **Side-by-side comparison video** - Merge animations into single video with tiled layout
2. **Pareto front overlay** - Annotate animations with trade-off positions
3. **Differential highlighting** - Color-code differences between configs
4. **Metric overlays** - Display live metric comparison in HUD
5. **Auto-selection** - Automatically identify and highlight "best" config

---

**Status:** ✅ Script running in background  
**Expected completion:** 2-4 minutes  
**Output location:** `results/20251010_211447_SWEEP_STAGEB_quick_test/animations/`
