# Trajectory Smoothing Strategy - Input vs Output

**Date**: October 9, 2025  
**Question**: What happens to waypoint count before/after smoothing?

---

## Current Strategy

### Input: Sparse Waypoints (10 Hz)
```
Source: GIK solver output
Frequency: 10 Hz (100ms spacing)
Count: 300 waypoints
Duration: 29.9 seconds

Example:
t=0.0s: [x=1.65, y=-0.018, θ=0.0]
t=0.1s: [x=1.64, y=-0.018, θ=0.0]
t=0.2s: [x=1.64, y=-0.019, θ=0.0]
...
```

### Output: Dense Commands (50 Hz)
```
Destination: Motor controller
Frequency: 50 Hz (20ms spacing)
Count: 1546 commands (for 30.9s simulation)
Duration: 30.9 seconds

Example:
t=0.00s: [vx=0.00, wz=0.0, ax=0.00, α=0.0]
t=0.02s: [vx=0.02, wz=0.0, ax=0.05, α=0.0]
t=0.04s: [vx=0.04, wz=0.0, ax=0.10, α=0.0]
...
```

---

## Strategy: **INTERPOLATION + SMOOTHING**

### Not a 1-to-1 Mapping!

**Input**:  300 waypoints → **Position** waypoints (x, y, θ)
**Output**: 1546 commands → **Velocity** commands (vx, wz)

**Ratio**: 5× more output points than input points (50Hz / 10Hz)

---

## How It Works

### Step 1: Find Current Segment (Line 90-98)
```matlab
% At each 50Hz control timestep, find which waypoint segment we're in
idx = find(t_waypoints >= t_current, 1, 'first');

% We're between waypoints idx-1 and idx
p0 = [waypoints_x(idx-1), waypoints_y(idx-1)];
p1 = [waypoints_x(idx), waypoints_y(idx)];
t0 = t_waypoints(idx-1);
t1 = t_waypoints(idx);
```

### Step 2: Compute Target Velocity (Line 123-133)
```matlab
% Direction from current segment
segment_length = norm(p1 - p0);
segment_duration = t1 - t0;

% Target velocity to reach next waypoint
vx_target = segment_length / segment_duration;
wz_target = (theta1 - theta0) / segment_duration;
```

### Step 3: Apply S-Curve Smoothing (Line 145-152)
```matlab
% Smooth transition from current velocity to target
% Enforces acceleration and jerk limits
[vx_cmd, ax_cmd] = applySCurve(vx_prev, ax_prev, vx_target, dt, ...);
```

### Step 4: Output Velocity Command (50 Hz)
```matlab
% Return smoothed velocity for THIS timestep
% Next timestep will compute again based on new segment
```

---

## Example Timeline

### Waypoints (10 Hz input):
```
t=0.0s: [x=1.65, y=-0.018]  ← Waypoint 1
        ↓
        ↓ (smoothing creates 5 commands)
        ↓
t=0.1s: [x=1.64, y=-0.018]  ← Waypoint 2
        ↓
        ↓ (smoothing creates 5 commands)
        ↓
t=0.2s: [x=1.64, y=-0.019]  ← Waypoint 3
```

### Commands (50 Hz output):
```
t=0.00s: vx=0.00 m/s, ax=0.00 m/s²  ← Command 1
t=0.02s: vx=0.02 m/s, ax=0.10 m/s²  ← Command 2
t=0.04s: vx=0.04 m/s, ax=0.10 m/s²  ← Command 3
t=0.06s: vx=0.06 m/s, ax=0.10 m/s²  ← Command 4
t=0.08s: vx=0.08 m/s, ax=0.10 m/s²  ← Command 5

t=0.10s: vx=0.10 m/s, ax=0.08 m/s²  ← Command 6 (new segment!)
t=0.12s: vx=0.11 m/s, ax=0.06 m/s²  ← Command 7
t=0.14s: vx=0.12 m/s, ax=0.04 m/s²  ← Command 8
t=0.16s: vx=0.13 m/s, ax=0.02 m/s²  ← Command 9
t=0.18s: vx=0.14 m/s, ax=0.00 m/s²  ← Command 10

t=0.20s: vx=0.14 m/s, ax=0.00 m/s²  ← Command 11 (next segment)
...
```

---

## Key Differences

| Aspect | Input Waypoints | Output Commands |
|--------|----------------|-----------------|
| **Type** | Position (x, y, θ) | Velocity (vx, wz) |
| **Frequency** | 10 Hz (sparse) | 50 Hz (dense) |
| **Count** | 300 points | 1546 points |
| **Source** | GIK solver | Smoothing module |
| **Continuity** | Discrete jumps | Smooth transitions |
| **Units** | meters, radians | m/s, rad/s |

---

## Why This Matters

### Without Smoothing (Direct Waypoint Tracking):
```
At t=0.1s transition:
Old: vx = 0.5 m/s
New: vx = 1.5 m/s
Acceleration = (1.5-0.5)/0.1 = 10 m/s²  ← TIPPING!

Number of commands = Number of waypoints = 300
```

### With Smoothing (50 Hz Interpolation):
```
At t=0.1s transition:
t=0.08s: vx = 0.50 m/s
t=0.10s: vx = 0.52 m/s  ← Smooth transition
t=0.12s: vx = 0.54 m/s
t=0.14s: vx = 0.56 m/s
...
t=0.60s: vx = 1.50 m/s  ← Gradual acceleration

Number of commands = 5× waypoints = 1500
Acceleration = 1.0 m/s²  ← SAFE!
```

---

## Algorithm Type: **Temporal Interpolation + Jerk-Limited Tracking**

This is **NOT**:
- ❌ Spatial resampling (same # points, different positions)
- ❌ Filtering (modifying existing points)
- ❌ 1-to-1 waypoint transformation

This **IS**:
- ✅ **Temporal upsampling** (10 Hz → 50 Hz)
- ✅ **Position → Velocity conversion**
- ✅ **Real-time trajectory tracking** with limits
- ✅ **Online algorithm** (processes one timestep at a time)

---

## Real-World Analogy

**Waypoints** = GPS checkpoints on a road trip
```
"Be at checkpoint A at 10:00"
"Be at checkpoint B at 10:06"
"Be at checkpoint C at 10:12"
```

**Smoothing** = Your driving between checkpoints
```
10:00:00 - Start at A, speed=0
10:00:12 - Accelerate to 30 mph
10:00:24 - Cruise at 40 mph
10:00:36 - Cruise at 40 mph
...
10:05:48 - Start slowing down
10:06:00 - Arrive at B, speed=0
```

**Result**:
- Input: 3 checkpoints
- Output: 360 driving commands (1 per second for 6 minutes)
- Smooth acceleration/deceleration between checkpoints

---

## Test Data Analysis

From `test_smoothing_real_data.m`:

```matlab
% INPUT
N_waypoints = 300;          % Sparse waypoints
dt_waypoint = 0.1;          % 10 Hz
t_total = 29.9 seconds;

% OUTPUT  
dt_control = 0.02;          % 50 Hz
N_sim = 1546;               % Dense commands
t_sim_end = 30.9 seconds;

% RATIO
N_sim / N_waypoints = 1546 / 300 = 5.15×
```

**Interpretation**:
- For each waypoint interval (100ms), we generate **5 velocity commands** (at 20ms spacing)
- This provides **5× temporal resolution** for smoother motion
- Total duration slightly longer (30.9 vs 29.9s) due to smooth deceleration at end

---

## Memory & Computational Impact

### Memory Usage:
```
Input buffer: 5 waypoints × 3 doubles = 15 doubles = 120 bytes
Output: Generated on-the-fly, no storage needed
State: 4 persistent variables = 32 bytes

Total: ~152 bytes (negligible!)
```

### Computation:
```
Per waypoint (10 Hz): No computation (just stored)
Per command (50 Hz): ~75 µs per call
Total CPU: 75µs × 50 = 3.75 ms/s = 0.375% CPU

Completely real-time capable!
```

---

## Comparison to Alternative Approaches

### Approach 1: Spline Interpolation (NOT used)
```
Input: 300 waypoints
Process: Fit cubic spline through all points
Output: 300 smoothed waypoints (same count)

Problem: Not real-time, requires future knowledge
```

### Approach 2: Moving Average Filter (NOT used)
```
Input: 300 waypoints
Process: Apply window filter
Output: 300 filtered waypoints (same count)

Problem: Still has discrete jumps, adds lag
```

### Approach 3: Our Method - Temporal Upsampling (USED) ✅
```
Input: 300 position waypoints @ 10 Hz
Process: Real-time velocity tracking with jerk limits
Output: 1546 velocity commands @ 50 Hz

Advantages:
✅ Real-time (online algorithm)
✅ Enforces physical limits
✅ Smooth acceleration
✅ No future knowledge needed
```

---

## Integration in ROS2

### Current System (No Smoothing):
```
GIK Solver (10 Hz)
    ↓ Waypoint [x, y, θ]
    ↓
Pure Pursuit (10 Hz)
    ↓ cmd_vel [vx, wz]
    ↓
Motor Controller (??Hz)
```

**Problem**: 10 Hz velocity updates → jerky motion

### With Smoothing Module:
```
GIK Solver (10 Hz)
    ↓ Waypoint [x, y, θ]
    ↓
Smoothing Buffer (stores last 5 waypoints)
    ↓
Smoothing Module (50 Hz timer)
    ↓ Smooth velocity [vx, wz]
    ↓
Motor Controller (50 Hz)
```

**Benefit**: 50 Hz smooth velocity updates → stable motion

---

## Summary

**Question**: Same number of points before/after?
**Answer**: **NO! We upsample 5×**

- **Input**: 300 position waypoints @ 10 Hz (from GIK)
- **Output**: 1546 velocity commands @ 50 Hz (to motors)
- **Strategy**: Temporal interpolation with jerk-limited tracking
- **Type**: Online algorithm (processes one timestep at a time)
- **Result**: Smooth, physically-realizable motion

**Analogy**: 
- Waypoints = "Be here at this time" (sparse GPS checkpoints)
- Smoothing = "How to drive between checkpoints" (dense steering commands)

The module **doesn't modify waypoints** - it **generates smooth velocities** to follow them!

