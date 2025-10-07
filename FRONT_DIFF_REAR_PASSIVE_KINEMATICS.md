# Front Differential + Passive Rear Omniwheels - Kinematics

**CRITICAL CORRECTION**: Platform is NOT pure differential drive!

## Platform Description

**Your WHEELTEC Platform**:
- **Front axle**: Two powered wheels (differential drive control)
- **Rear axle**: Two passive omniwheels with lateral rolling rollers
- **Wheelbase**: 0.36 m (front-to-rear distance)
- **Track**: 0.573 m (front wheel spacing)
- **Drive wheels**: 107.5mm radius (215mm diameter)

## Kinematic Model

### Hybrid Characteristics

This is a **hybrid kinematic model** combining:
1. **Differential drive** (front axle control)
2. **Ackermann-like constraints** (passive rear creates ICR)

**NOT**:
- ❌ Pure differential drive (can't spin in place due to passive rear)
- ❌ Classic Ackermann (no mechanical steering linkage)
- ❌ Holonomic/omnidirectional (constrained motion)

### Forward Kinematics

**Inputs**: Left/right front wheel speeds (v_L, v_R)

**Front Axle Differential**:
```
Vx_front = (v_R + v_L) / 2           [m/s]
Wz = (v_R - v_L) / track             [rad/s]
```

**Instantaneous Center of Rotation (ICR)**:
- The passive rear wheels force the robot to pivot around a point
- ICR location depends on the speed differential at front wheels
- Unlike pure diff-drive, ICR is NOT at the center of the robot

**Equivalent Steering Angle** (virtual):
```
tan(delta_virtual) = wheelbase * Wz / Vx_front
```

**Minimum Turning Radius**:
```
R_min ≈ track / 2 = 0.573 / 2 = 0.286 m (geometric)
R_min ≈ 0.34 m (with 20% safety margin)
```

### Inverse Kinematics

**Given desired** (Vx, Wz):

**Front wheel speeds**:
```matlab
v_L = Vx - (track/2) * Wz
v_R = Vx + (track/2) * Wz
```

**Constraints**:
```matlab
|v_L| <= Vwheel_max  % 1.5 m/s
|v_R| <= Vwheel_max  % 1.5 m/s
```

**Yaw rate limits**:
```
Wz_max = 2 * Vwheel_max / track = 5.24 rad/s (theoretical)
Wz_max ≈ 2.5 rad/s (firmware limit, conservative)
```

### State Evolution (SE(2))

**Continuous dynamics**:
```
dx/dt = Vx * cos(theta)
dy/dt = Vx * sin(theta)
dtheta/dt = Wz
```

**Discrete update** (Euler integration, dt = 0.1s):
```matlab
x_new = x + Vx * cos(theta) * dt
y_new = y + Vx * sin(theta) * dt
theta_new = theta + Wz * dt
```

**Arc motion** (constant Vx, Wz):
```matlab
if abs(Wz) < 1e-4
    % Straight line
    x_new = x + Vx * cos(theta) * T
    y_new = y + Vx * sin(theta) * T
    theta_new = theta
else
    % Circular arc
    R = Vx / Wz                          % Turning radius
    arc_length = Vx * T                  % Distance traveled
    dtheta = Wz * T                      % Heading change
    
    % Center of circular arc
    cx = x - R * sin(theta)
    cy = y + R * cos(theta)
    
    % New position
    theta_new = theta + dtheta
    x_new = cx + R * sin(theta_new)
    y_new = cy - R * cos(theta_new)
end
```

## Comparison: Pure Diff vs Front Diff + Passive Rear

| Property | Pure Differential | **Your Platform** |
|----------|------------------|-------------------|
| Front axle | Driven (diff) | **Driven (diff)** ✓ |
| Rear axle | Driven (diff) | **Passive omni** ← KEY |
| Control inputs | (v_L, v_R) | **(v_L, v_R)** ✓ |
| Spin in place? | ✅ Yes (Vx=0, Wz≠0) | ❌ **No** (passive rear resists) |
| Min turn radius | 0 m | **~0.34 m** ← CONSTRAINT |
| ICR location | Robot center | **Variable** (depends on Wz) |
| Model for planning | Diff-drive | **Simplified Ackermann** |
| Motion primitives | (Vx, Wz) arcs | **(Vx, Wz) arcs** ✓ |
| Heuristic | Euclidean | **Dubins** (min radius) |

## Implications for Hybrid A*

### 1. Motion Primitives

**Generate arcs** using bicycle model:
```matlab
% Define primitive set
Vx_options = [0.4, 0.6, 0.8];        % [m/s] Forward speeds
Wz_options = [-2.5, -1.5, 0, 1.5, 2.5];  % [rad/s] Yaw rates
T_options = [0.5, 1.0, 1.5];         % [s] Durations

% For each combination (Vx, Wz, T):
% 1. Check R = Vx/Wz >= R_min (0.34 m)
% 2. Compute arc endpoint (x,y,theta)
% 3. Check wheel speed limits: |Vx ± (track/2)*Wz| <= 1.5 m/s
```

**Key difference from pure diff**:
- ❌ **No spin-in-place primitives** (Vx=0, Wz≠0 won't work)
- ✅ Use **forward/backward arcs** with curvature limits
- ✅ Enforce **R >= R_min = 0.34 m**

### 2. Heuristic Function

**Use Dubins distance** (not Euclidean):
- Minimum path length considering turn radius constraint
- Accounts for R_min = 0.34 m
- More accurate than Euclidean for non-holonomic systems

**Alternative**: Reeds-Shepp (allows backward motion)

### 3. Collision Checking

**Robot footprint**:
```matlab
robot_length = 0.60 m  % Including bumpers
robot_width = 0.70 m   % Track + clearance
robot_radius = sqrt((0.60/2)^2 + (0.70/2)^2) = 0.46 m  % Bounding circle
```

**Check arc segments**:
- Sample every 0.1 m along arc
- Check footprint at each sample point
- Inflate obstacles by robot_radius + margin

### 4. State Space

**3D lattice** (x, y, theta):
```matlab
grid_resolution = 0.1 m       % 10 cm spatial
theta_resolution = pi/8       % 16 heading bins (22.5°)
grid_size = 200 x 200 x 16    % 20m x 20m x 360°
```

**Memory**: ~640 KB (much smaller than differential assumption!)

## Velocity Limits

**From kinematics**:
```matlab
% Front wheel speeds
Vwheel_max = 1.5 m/s

% Forward speed (conservative)
Vx_max = 0.8 m/s

% Yaw rate limits
Wz_max_theoretical = 2 * Vwheel_max / track = 5.24 rad/s
Wz_max_firmware = 2.5 rad/s  % Use this (conservative)

% At max forward speed
Wz_max_at_Vx_max = 2 * (Vwheel_max - Vx_max) / track = 2.44 rad/s
```

**Constraint check** for primitive (Vx, Wz):
```matlab
v_L = Vx - (track/2) * Wz
v_R = Vx + (track/2) * Wz

% Both must satisfy:
assert(abs(v_L) <= Vwheel_max)
assert(abs(v_R) <= Vwheel_max)
```

## Control Commands

**Output from Hybrid A***:
```matlab
% Sequence of (Vx, Wz) commands
commands = [
    % Vx [m/s], Wz [rad/s], Duration [s]
    0.6,  1.5,  1.0;   % Forward left arc
    0.8,  0.0,  2.0;   % Straight
    0.6, -1.5,  1.0;   % Forward right arc
];
```

**Existing controller compatibility**:
```matlab
% unifiedChassisCtrl already uses (Vx, Wz)!
[Wz, yawCaps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, params.track, ...)
```

✅ **Your existing controller is compatible!**

## Summary

**Your platform is**:
- Control: Differential front axle (v_L, v_R)
- Constraint: Passive rear creates min turn radius
- Model: **Simplified Ackermann** (virtual steering via diff)
- Planning: Use (Vx, Wz) arcs with R >= 0.34 m
- Advantage: Tighter turns than true Ackermann cars (R_min ~ 1-2m)
- Limitation: No spin-in-place (unlike pure diff-drive)

**Next steps**:
1. ✅ Updated `getChassisParams()` with wheelbase=0.36m, min_turning_radius
2. 🔄 Revise motion primitive generation (enforce R_min)
3. 🔄 Implement Dubins heuristic (not Euclidean)
4. 🔄 Remove spin-in-place primitives from design

---
**Date**: 2025-01-07  
**Status**: CRITICAL DESIGN CORRECTION  
**Impact**: Changes motion primitive set, heuristic function, and state space size
