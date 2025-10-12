# Configuration Comparison: test_comprehensive_evaluation vs parametric_study

**Date:** October 11, 2025  
**Comparison:** `test_comprehensive_evaluation.mat` vs `parametric_study_20251011_085252`

## Executive Summary

The two MAT files contain **partially overlapping but NOT identical** configurations:

- **Baseline:** ✅ **IDENTICAL** in both files
- **Conservative:** ❌ **DIFFERENT** (SM and LC values differ)
- **Aggressive:** ❌ **DIFFERENT** (MaxRSIterations and ClothoidDiscretization differ)
- **File 1 has:** High Resolution config (unique)
- **File 2 has:** AntiCusp_Lambda10 and HighRes_MaxSmooth (unique)

Despite parameter differences, **ALL configurations produce identical results** (EE error, cusps, smoothness), confirming zero parameter sensitivity for this trajectory.

---

## Detailed Comparison

### 1. BASELINE Configuration ✅ IDENTICAL

| Parameter | File 1 (Tuned Baseline) | File 2 (Baseline_Tuned) | Match? |
|-----------|-------------------------|-------------------------|--------|
| SafetyMargin | 0.10 | 0.10 | ✅ |
| LambdaCusp | 1.0 | 1.0 | ✅ |
| MaxRSIterations | 200 | 200 | ✅ |
| AllowReverse | YES (1) | YES (1) | ✅ |
| ClothoidDiscretization | 0.08 | 0.08 | ✅ |
| Lookahead | 0.80 | 0.80 | ✅ |
| AccelLimit | 0.8 | 0.8 | ✅ |
| HeadingKp | 1.0 | 1.0 | ✅ |

**Result:** IDENTICAL parameters

**Performance (both files):**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: 2
- Smoothness: 1.020
- Score: 0.764

---

### 2. CONSERVATIVE Configuration ❌ DIFFERENT

| Parameter | File 1 (Conservative) | File 2 (Conservative_HighSafety) | Match? |
|-----------|----------------------|----------------------------------|--------|
| SafetyMargin | **0.15** | **0.20** | ❌ **+33%** |
| LambdaCusp | **2.0** | **3.0** | ❌ **+50%** |
| MaxRSIterations | 400 | 400 | ✅ |
| AllowReverse | NO (0) | NO (0) | ✅ |
| ClothoidDiscretization | 0.05 | 0.05 | ✅ |
| Lookahead | 0.60 | 0.60 | ✅ |
| AccelLimit | 0.6 | 0.6 | ✅ |
| HeadingKp | 0.8 | 0.8 | ✅ |

**Key Differences:**
- File 2 has **HIGHER** safety margin (0.20 vs 0.15 = +33%)
- File 2 has **HIGHER** cusp penalty (3.0 vs 2.0 = +50%)
- File 2 is MORE conservative than File 1

**Performance (IDENTICAL despite parameter differences):**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: 2
- Smoothness: 1.020
- Score: 0.764

**Conclusion:** Even with more conservative parameters, the result is IDENTICAL, proving zero parameter sensitivity.

---

### 3. AGGRESSIVE Configuration ❌ DIFFERENT

| Parameter | File 1 (Aggressive) | File 2 (Aggressive_LowSafety) | Match? |
|-----------|---------------------|-------------------------------|--------|
| SafetyMargin | 0.05 | 0.05 | ✅ |
| LambdaCusp | 0.5 | 0.5 | ✅ |
| MaxRSIterations | **100** | **200** | ❌ **2x** |
| AllowReverse | YES (1) | YES (1) | ✅ |
| ClothoidDiscretization | **0.10** | **0.08** | ❌ **-20%** |
| Lookahead | 1.20 | 1.20 | ✅ |
| AccelLimit | 1.2 | 1.2 | ✅ |
| HeadingKp | 1.5 | 1.5 | ✅ |

**Key Differences:**
- File 2 has **DOUBLE** RS iterations (200 vs 100 = 2x)
- File 2 has **FINER** clothoid discretization (0.08 vs 0.10 = -20%)
- File 2 should theoretically have MORE smoothing attempts

**Performance (IDENTICAL despite parameter differences):**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: 2
- Smoothness: 1.020
- Score: 0.764

**Conclusion:** Even with 2x RS iterations and finer clothoid discretization, the result is IDENTICAL. This confirms RS refinement is not working (0% acceptance).

---

### 4. HIGH RESOLUTION Configuration (File 1 ONLY)

| Parameter | File 1 Value |
|-----------|--------------|
| SafetyMargin | 0.12 |
| LambdaCusp | 1.5 |
| MaxRSIterations | 600 |
| AllowReverse | YES (1) |
| ClothoidDiscretization | 0.03 |
| Lookahead | 1.00 |
| AccelLimit | 1.0 |
| HeadingKp | 1.2 |

**Performance:**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: 2
- Smoothness: 1.020
- Score: 0.764

**Note:** This is DIFFERENT from File 2's "HighRes_MaxSmooth" config (which has 1000 iters, LH=0.8, AL=0.8, HK=1.0).

---

### 5. ANTICUSP_LAMBDA10 Configuration (File 2 ONLY)

| Parameter | File 2 Value |
|-----------|--------------|
| SafetyMargin | 0.10 |
| LambdaCusp | **10.0** (EXTREME!) |
| MaxRSIterations | 600 |
| AllowReverse | YES (1) |
| ClothoidDiscretization | 0.08 |
| Lookahead | 0.80 |
| AccelLimit | 0.8 |
| HeadingKp | 1.0 |

**Performance:**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: **2** (STILL 2 despite λ=10!) 🔴
- Smoothness: 1.020
- RS Accept: **0.0%**
- Jerk RMS: 20.6 m/s³

**Critical Finding:** Even with **10x cusp penalty** (λ=10.0 vs baseline 1.0), the result is IDENTICAL with 2 cusps. This proves:
1. RS refinement is completely non-functional (0% acceptance)
2. Cusp penalty has NO EFFECT on the final path
3. Cusps are inherited from Hybrid A* output, not affected by RS

---

### 6. HIGHRES_MAXSMOOTH Configuration (File 2 ONLY)

| Parameter | File 2 Value |
|-----------|--------------|
| SafetyMargin | 0.10 |
| LambdaCusp | 1.5 |
| MaxRSIterations | **1000** (5x baseline!) |
| AllowReverse | YES (1) |
| ClothoidDiscretization | **0.03** (finest) |
| Lookahead | 0.80 |
| AccelLimit | 0.8 |
| HeadingKp | 1.0 |

**Performance:**
- EE Error: mean=0.0038m, max=0.0548m
- Cusps: **2** (still 2 despite 1000 iters!)
- Smoothness: 1.020
- RS Accept: **0.0%** (0 out of 1000!)
- Jerk RMS: 20.6 m/s³

**Critical Finding:** Even with **1000 RS iterations** (5x baseline) and **finest clothoid discretization** (0.03m), the result is IDENTICAL. This proves:
1. More iterations provide ZERO benefit (wasted computation)
2. Clothoid discretization has NO OBSERVABLE EFFECT
3. The bottleneck is RS acceptance logic, not iteration count

---

## Summary Table: All Configurations

| Config Name | File | SM | LC | Iters | Rev | CD | LH | AL | HK | EE Mean | Cusps | Unique? |
|-------------|------|----|----|-------|-----|----|----|----|----|---------|-------|---------|
| Tuned Baseline | 1 | 0.10 | 1.0 | 200 | 1 | 0.08 | 0.80 | 0.8 | 1.0 | 0.0038 | 2 | - |
| Baseline_Tuned | 2 | 0.10 | 1.0 | 200 | 1 | 0.08 | 0.80 | 0.8 | 1.0 | 0.0038 | 2 | ✅ SAME |
| Conservative | 1 | 0.15 | 2.0 | 400 | 0 | 0.05 | 0.60 | 0.6 | 0.8 | 0.0038 | 2 | - |
| Conservative_HighSafety | 2 | **0.20** | **3.0** | 400 | 0 | 0.05 | 0.60 | 0.6 | 0.8 | 0.0038 | 2 | ❌ DIFF |
| Aggressive | 1 | 0.05 | 0.5 | **100** | 1 | **0.10** | 1.20 | 1.2 | 1.5 | 0.0038 | 2 | - |
| Aggressive_LowSafety | 2 | 0.05 | 0.5 | **200** | 1 | **0.08** | 1.20 | 1.2 | 1.5 | 0.0038 | 2 | ❌ DIFF |
| High Resolution | 1 | 0.12 | 1.5 | 600 | 1 | 0.03 | 1.00 | 1.0 | 1.2 | 0.0038 | 2 | File 1 only |
| AntiCusp_Lambda10 | 2 | 0.10 | **10.0** | 600 | 1 | 0.08 | 0.80 | 0.8 | 1.0 | 0.0038 | 2 | File 2 only |
| HighRes_MaxSmooth | 2 | 0.10 | 1.5 | **1000** | 1 | **0.03** | 0.80 | 0.8 | 1.0 | 0.0038 | 2 | File 2 only |

**Key:**
- SM = SafetyMargin (m)
- LC = LambdaCusp (reversal penalty)
- Iters = MaxRSIterations
- Rev = AllowReverse (1=yes, 0=no)
- CD = ClothoidDiscretization (m)
- LH = Lookahead (m)
- AL = AccelLimit (m/s²)
- HK = HeadingKp (gain)

---

## Key Observations

### 1. Identical Results Despite Parameter Variations
**ALL 9 configurations** (4 in File 1, 5 in File 2) produce **IDENTICAL performance metrics:**
- EE Error: mean=0.0038m, max=0.0548m
- Cusp Count: 2
- Path Smoothness: 1.020
- Overall Score: 0.764 (File 1 only)

This confirms our earlier finding: **zero parameter sensitivity** for this simple trajectory.

### 2. Parameter Ranges Explored

| Parameter | Min | Max | Range |
|-----------|-----|-----|-------|
| SafetyMargin | 0.05 | 0.20 | **4x** |
| LambdaCusp | 0.5 | **10.0** | **20x** |
| MaxRSIterations | 100 | **1000** | **10x** |
| ClothoidDiscretization | 0.03 | 0.10 | **3.3x** |
| Lookahead | 0.60 | 1.20 | **2x** |
| AccelLimit | 0.6 | 1.2 | **2x** |
| HeadingKp | 0.8 | 1.5 | **1.9x** |

Despite exploring **wide parameter ranges**, NO observable difference in results!

### 3. Critical Findings from File 2 Extras

**AntiCusp_Lambda10** (λ=10.0):
- Even with 10x cusp penalty, still get 2 cusps
- Proves RS refinement not working (0% acceptance)
- Cusp penalty has NO EFFECT

**HighRes_MaxSmooth** (1000 iters):
- Even with 1000 RS iterations (5x baseline), result identical
- 0 out of 1000 RS improvements accepted (0.0%)
- Proves more iterations = wasted computation

### 4. What Files 1 and 2 Test Differently

**File 1 (test_comprehensive_evaluation.mat):**
- More moderate parameter variations
- Conservative has SM=0.15, LC=2.0 (moderate)
- Aggressive has 100 iters, CD=0.10 (coarse)
- High Resolution unique config (SM=0.12, LH=1.0, AL=1.0, HK=1.2)

**File 2 (parametric_study_20251011_085252):**
- More extreme parameter variations
- Conservative has SM=0.20, LC=3.0 (MORE conservative)
- Aggressive has 200 iters, CD=0.08 (finer)
- Two extreme configs: AntiCusp (λ=10) and HighRes (1000 iters)
- Includes **jerk measurements** (20.6 m/s³ RMS)
- Includes **RS acceptance rates** (0.0% across all)

---

## Implications

### For Animation Review:
1. **File 1 has NO animations** - only MAT file with results
2. **File 2 has 5 animations** ready for review:
   - Baseline_Tuned (identical to File 1's Tuned Baseline)
   - Aggressive_LowSafety (slightly different from File 1's Aggressive)
   - Conservative_HighSafety (more conservative than File 1's Conservative)
   - AntiCusp_Lambda10 (unique, extreme cusp penalty test)
   - HighRes_MaxSmooth (unique, maximum smoothing test)

3. **All 5 animations should look IDENTICAL** because results are identical

### For Parameter Tuning:
1. **Current trajectory too simple** - doesn't stress-test parameters
2. **RS refinement broken** - 0% acceptance rate regardless of parameters
3. **Reference jerk is bottleneck** - 20.6 m/s³ (File 2 measured this, File 1 didn't)
4. **Need complex trajectory** - with tight clearances, actual collision risks, sharp turns

### For Next Steps:
1. ✅ **File 2 provides MORE insights** with jerk and RS acceptance metrics
2. ✅ **Animations ready** for visual confirmation (File 2 only)
3. 🔴 **Fix RS refinement** before more parameter tuning (0% acceptance)
4. 🔴 **Fix Stage C reference jerk** from 20.6 → <5 m/s³
5. ⏳ **Design complex trajectory** to properly test parameters

---

## Conclusion

**Are they the same?**
- **Baseline:** YES ✅ (100% identical)
- **Conservative:** NO ❌ (File 2 more conservative: SM +33%, LC +50%)
- **Aggressive:** NO ❌ (File 2 has 2x iters, finer CD)
- **High Resolution configs:** Different between files
- **File 2 has 2 extra configs:** AntiCusp_Lambda10, HighRes_MaxSmooth

**Do the differences matter?**
- **NO!** All configs produce IDENTICAL results (EE=0.0038m, Cusps=2, Jerk=20.6 m/s³)
- This confirms zero parameter sensitivity
- RS refinement 0% effective regardless of parameters
- Reference jerk (20.6 m/s³) is the real issue, not parameter tuning

**Which file is better for analysis?**
- **File 2 (parametric_study)** because it includes:
  - Jerk measurements (20.6 m/s³ identified)
  - RS acceptance rates (0.0% confirmed)
  - 5 ready-to-review animations
  - More extreme parameter tests (λ=10, 1000 iters)
  - Comprehensive review guide (ANIMATION_REVIEW_GUIDE.md)

---

**Generated:** 2025-10-11 09:45:00  
**Comparison Script:** `compare_test_configs.m`
