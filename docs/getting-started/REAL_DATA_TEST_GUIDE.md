# Trajectory Smoothing - Real Data Validation

**Created**: October 9, 2025  
**Status**: Ready for testing  
**Test Type**: Real robot waypoints (not synthetic)

---

## 🎯 What Was Created

### 1. Test Script: `test_smoothing_real_data.m`
**Location**: `matlab/test_smoothing_real_data.m`

**Purpose**: Validate trajectory smoothing using **actual 300 waypoints** from `1_pull_world_scaled.json`

**Key Features**:
- ✅ Loads real JSON waypoint data (300 poses, ~30 seconds)
- ✅ Converts quaternions to yaw angles
- ✅ Simulates 50Hz control loop (realistic timing)
- ✅ Compares raw vs smoothed velocities
- ✅ Validates acceleration/jerk limits
- ✅ Generates comprehensive analysis plots

### 2. Documentation: `REAL_TRAJECTORY_TEST.md`
**Location**: `REAL_TRAJECTORY_TEST.md`

**Contents**:
- Test overview and data source
- Expected results and validation criteria
- Troubleshooting guide
- Real-world implications comparison
- Next steps after validation

### 3. Batch Runner: `run_real_trajectory_test.bat`
**Location**: `run_real_trajectory_test.bat`

**Usage**: Double-click to run test (Windows)

---

## 📊 Test Data Source

**File**: `data/1_pull_world_scaled.json`
- **Waypoints**: 300 poses
- **Format**: JSON with position [x,y,z] and quaternion [x,y,z,w]
- **Spacing**: ~100ms (10 Hz) - **REAL robot timing**
- **Duration**: ~30 seconds of motion
- **Origin**: Actual GIK solver output from pull world scenario

**Why this matters**: This is NOT synthetic test data - these are real waypoints your robot will execute!

---

## 🚀 How to Run

### Option 1: Batch Script (Easiest)
```cmd
# In Windows Explorer
Double-click: run_real_trajectory_test.bat
```

### Option 2: MATLAB Directly
```matlab
% In MATLAB
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab
test_smoothing_real_data
```

### Option 3: Command Line
```cmd
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF
matlab -batch "cd matlab; test_smoothing_real_data"
```

---

## 📈 What to Expect

### Console Output
```
Loading waypoint data from 1_pull_world_scaled.json...
Loaded 300 waypoints
Trajectory duration: 29.9 seconds
Position range: X=[...], Y=[...], Z=[...]

Smoothing Parameters:
  vx_max  = 1.50 m/s
  ax_max  = 1.00 m/s²
  jx_max  = 5.00 m/s³
  ...

Running trajectory smoothing...
Smoothing completed in 0.XXX seconds

========== ANALYSIS ==========
Forward Velocity:
  Raw:    max=X.XXX m/s
  Smooth: max=X.XXX m/s (limit: 1.50 m/s)

Forward Acceleration:
  Raw:    max=X.XXX m/s² ← May be > 10 m/s² (TIPPING!)
  Smooth: max=X.XXX m/s² (limit: 1.00 m/s²)

Forward Jerk:
  Smooth: max=X.XXX m/s³ (limit: 5.00 m/s³)

========== LIMIT CHECKS ==========
✅ All limits respected!
```

### Figure Window (9 Subplots)
1. **3D Trajectory**: Full path visualization
2. **XY Top View**: Motion in horizontal plane
3. **Forward Velocity**: Raw (stairs) vs Smooth (curve)
4. **Angular Velocity**: Raw vs Smooth
5. **Forward Acceleration**: With limit lines at ±1.0 m/s²
6. **Angular Acceleration**: With limit lines
7. **Forward Jerk**: Should stay within ±5.0 m/s³
8. **Angular Jerk**: Should stay within ±10.0 rad/s³
9. **Summary Stats**: Pass/fail indicators

---

## ✅ Success Criteria

**PASS if you see**:
- ✅ All limit check lines show "OK"
- ✅ Smooth velocity curves (no sudden jumps)
- ✅ Acceleration stays within ±1.0 m/s² (green zone)
- ✅ Jerk stays within ±5.0 m/s³
- ✅ No oscillations or overshoot
- ✅ Console shows "✅ All limits respected!"

**INVESTIGATE if you see**:
- ⚠️ Warnings about limit violations
- ⚠️ Acceleration exceeding limits
- ⚠️ Oscillations in velocity plots
- ⚠️ Large differences between raw and smooth paths

---

## 🔍 Key Comparisons

### Raw (Current System - No Smoothing)
**What happens without this module**:
```
At waypoint transition (100ms gap):
vx jumps from 0.3 → 0.9 m/s
Implied acceleration = (0.9-0.3)/0.1 = 6.0 m/s²
Implied jerk = ∞ (instantaneous jump)
→ ROBOT TIPS OVER!
```

### Smoothed (With S-Curve Module)
**What happens with smoothing**:
```
Gradual transition over ~600ms:
t=0.0s: vx=0.3, ax=0.0
t=0.1s: vx=0.35, ax=0.5 (jerk-up)
t=0.3s: vx=0.5, ax=1.0 (max accel)
t=0.5s: vx=0.75, ax=0.5 (jerk-down)
t=0.6s: vx=0.9, ax=0.0
→ SMOOTH, STABLE!
```

**Trade-off**: Takes ~6× longer to accelerate, but robot stays upright!

---

## 🎨 How to Read the Plots

### Plot 3: Forward Velocity (Most Important!)
- **Red stairs**: Raw waypoint velocities (current system)
  - **Look for**: Sudden vertical jumps = instant velocity changes
  - **Problem**: These cause high acceleration
- **Blue curve**: Smoothed velocities (proposed system)
  - **Look for**: Gradual slopes = controlled acceleration
  - **Benefit**: No sudden changes
- **Black dashed**: Velocity limit (1.5 m/s)
  - **Check**: Blue stays below black

### Plot 5: Forward Acceleration (Safety Check!)
- **Blue line**: Smoothed acceleration
  - **Must stay**: Between ±1.0 m/s² (black dashed lines)
  - **If exceeds**: Robot may tip over
- **Red dotted**: Raw acceleration
  - **Expect**: Much higher peaks (why we need smoothing!)

### Plot 7: Forward Jerk (Smoothness Check!)
- **Blue line**: Rate of acceleration change
  - **Must stay**: Between ±5.0 m/s³
  - **Shows**: How "smooth" the motion feels
  - **Low jerk**: Passenger comfort, reduced wear

---

## 🔧 Parameter Tuning (If Needed)

### If acceleration still exceeds limits:
Edit `test_smoothing_real_data.m`:
```matlab
params.ax_max = 0.8;  % Reduce from 1.0
params.jx_max = 3.0;  % Reduce from 5.0
```

### If motion is too conservative/slow:
```matlab
params.ax_max = 1.2;  % Increase from 1.0
params.jx_max = 7.0;  % Increase from 5.0
```

### If using different robot (heavier/lighter):
```matlab
% For heavier robot (more inertia):
params.ax_max = 0.6;
params.alpha_max = 2.0;

% For lighter robot (less inertia):
params.ax_max = 1.5;
params.alpha_max = 4.0;
```

---

## 📝 Test Results Template

**After running, record**:

```
Date: ___________
Test: test_smoothing_real_data.m
Data: 1_pull_world_scaled.json (300 waypoints)

Results:
- Max forward velocity: _____ m/s (limit: 1.50)
- Max forward accel:    _____ m/s² (limit: 1.00)
- Max forward jerk:     _____ m/s³ (limit: 5.00)
- Max angular velocity: _____ rad/s (limit: 2.00)
- Max angular accel:    _____ rad/s² (limit: 3.00)
- Max angular jerk:     _____ rad/s³ (limit: 10.00)

Limit Check: ☐ PASS  ☐ FAIL
Visual Check: ☐ Smooth curves  ☐ Oscillations
Ready for C++ codegen: ☐ YES  ☐ NO (tune params)

Notes:
_________________________________________________
```

---

## 🚦 Next Steps After Validation

### If Test PASSES ✅
1. Document results (copy console output)
2. Save figure (File → Save As → `smoothing_test_results.png`)
3. Proceed to **Phase 2**: C++ code generation
4. Follow `TRAJECTORY_SMOOTHING_PLAN.md` Phase 2

### If Test FAILS ⚠️
1. Check which limits are violated
2. Tune parameters (see tuning guide above)
3. Re-run test
4. If still failing, consult `REAL_TRAJECTORY_TEST.md` troubleshooting

### After C++ Codegen (Phase 2)
1. Integrate into `gik9dof_solver_node.cpp`
2. Build on WSL
3. Deploy to Orin
4. **Real robot testing** with monitoring

---

## 📊 Performance Expectations

**Computational**:
- Processing time: < 5 seconds for 1500 control steps
- Per-call time: < 10 microseconds
- CPU overhead at 50Hz: < 0.1%
- Memory usage: < 100 bytes

**Motion Quality**:
- Velocity smoothness: Continuous (no jumps)
- Acceleration smoothness: Continuous (no jumps)
- Jerk: Bounded and controlled
- Path following: Within 0.1-0.3m tolerance

---

## 🎯 Why This Test is Critical

1. **Real Data**: Uses actual waypoints from your robot
2. **Real Timing**: 10Hz input, 50Hz output (actual system)
3. **Real Limits**: Based on your robot's physical constraints
4. **Real Problem**: Solves the tipping issue you identified

**This is the validation that proves the concept works before deploying to hardware.**

---

## 📞 Support

**If test doesn't run**:
- Check MATLAB path includes `+gik9dof/+control/`
- Verify `smoothTrajectoryVelocity.m` exists
- Ensure `data/1_pull_world_scaled.json` exists

**If results look wrong**:
- Check console for error messages
- Verify waypoint data loaded correctly
- Check if quaternion → yaw conversion is correct
- Try different smoothing method (`'exponential'`)

---

**Status**: ✅ Ready to run  
**Estimated time**: < 1 minute  
**Required**: MATLAB R2024a, Statistics Toolbox

**Run command**: `matlab -batch "cd matlab; test_smoothing_real_data"`

