# Chassis Integration for Hybrid A* Path Planner

**Date**: October 7, 2025  
**Platform**: WHEELTEC Four-Wheel Differential Drive  
**Source**: `docs/chassis-summary.txt` + firmware analysis

---

## Your Robot Specifications

### Physical Parameters (Four-Wheel Variant, Types 2-5)
```
Track Width:      0.573 m  (wider differential platform)
Wheel Radius:     0.1075 m (107.5mm, 215mm diameter)
Robot Length:     ~0.60 m  (estimated from platform)
Robot Width:      ~0.70 m  (track + bumper clearance)
Bounding Radius:  ~0.46 m  (conservative circle for collision)
```

### Kinematic Constraints
```
Chassis Type:     Differential Drive (NOT Ackermann steering)
                  - Two independently-driven wheels
                  - Skid-steer capable
                  - Can rotate in place (zero turning radius)
```

### Velocity Limits (from firmware + controller)
```
Max Wheel Speed:    1.5 m/s per wheel (verified in defaultUnifiedParams)
Max Forward Speed:  0.8 m/s (conservative, firmware allows 3.5 m/s)
Max Yaw Rate:
  - Pure spin (Vx=0):     5.2 rad/s (~300 deg/s)
  - At max speed (0.8):   2.4 rad/s (~140 deg/s)
  - Firmware clamp:       2.5 rad/s (default W_max)
```

### Acceleration Limits (from firmware ramp)
```
Max Accel:        2.0 m/s² (0.02 m/s per 10ms step)
Ramp Control:     Vel_SmoothControl smooths commands
```

---

## Differential Drive Kinematics

### Forward Kinematics (Wheel → Chassis)
```matlab
% From measured wheel speeds to chassis velocity
omega_L = left_wheel_speed;   % [rad/s]
omega_R = right_wheel_speed;  % [rad/s]
r = wheel_radius;             % 0.1075 m
L = track_width;              % 0.573 m

% Linear velocities
v_L = omega_L * r;
v_R = omega_R * r;

% Chassis velocity
Vx = (v_R + v_L) / 2;          % Forward speed [m/s]
Vy = 0;                         % No lateral motion (diff-drive)
Wz = (v_R - v_L) / L;          % Yaw rate [rad/s]
```

### Inverse Kinematics (Chassis → Wheel)
```matlab
% From desired chassis velocity to wheel commands
Vx_desired = ...;  % [m/s]
Wz_desired = ...;  % [rad/s]

% Required wheel speeds
v_L = Vx_desired - (L/2) * Wz_desired;
v_R = Vx_desired + (L/2) * Wz_desired;

% Check wheel limits
v_L_clamped = clamp(v_L, -1.5, 1.5);  % m/s
v_R_clamped = clamp(v_R, -1.5, 1.5);  % m/s
```

### Constraint Enforcement
```matlab
% Differential drive constraint (from chassis-summary.txt):
% |Vx ± (L/2) * Wz| <= Vwheel_max
%
% Rearranging for max yaw rate:
% |Wz| <= 2 * (Vwheel_max - |Vx|) / L
%
% Examples (L = 0.573 m, Vwheel_max = 1.5 m/s):
%   Pure spin (Vx = 0):      Wz_max = 5.2 rad/s
%   At Vx = 0.8 m/s:         Wz_max = 2.4 rad/s
%   At Vx = 1.5 m/s:         Wz_max = 0 rad/s (can't turn)
```

---

## Implications for Hybrid A* Planner

### 1. Motion Primitives (NOT Ackermann arcs!)

**Differential drive is fundamentally different from Ackermann steering**:

| Aspect | Ackermann (car-like) | Differential (your robot) |
|--------|---------------------|---------------------------|
| **Turning** | Fixed steering angle → circular arc | Independent wheel speeds → any curvature |
| **Min radius** | ~1-2m (limited by steering) | **0m** (can spin in place!) |
| **Reverse** | Can drive backward with turn | Can drive backward + spin |
| **Curvature** | Limited by wheelbase/steer | **Unlimited** (skid-steer) |

**For Hybrid A*, this means**:
- ✅ **Don't use bicycle model** (that's for Ackermann steering)
- ✅ **Use differential drive model** (simpler!)
- ✅ **Allow in-place rotation** (huge advantage for tight spaces)
- ✅ **Generate arcs via (Vx, Wz) commands**, not steering angle

### 2. Motion Primitive Set

**Recommended primitives** (differential drive specific):
```matlab
% Forward motion with various curvatures
Vx = [0.5, 0.5, 0.5, 0.5, 0.5];  % Constant forward speed
Wz = [-1.0, -0.5, 0.0, +0.5, +1.0];  % Vary yaw rate

% In-place rotation (unique to diff-drive!)
Vx = [0.0, 0.0];
Wz = [-1.0, +1.0];  % Left spin, right spin

% Backward motion
Vx = [-0.3, -0.3, -0.3];
Wz = [-0.5, 0.0, +0.5];

% Total: 5 forward + 2 spin + 3 backward = 10 primitives
```

**Arc generation** (simpler than Ackermann):
```matlab
function [x, y, theta] = diffDriveArc(x0, y0, theta0, Vx, Wz, dt, steps)
    % Integrate constant-velocity arc
    for i = 1:steps
        theta = theta + Wz * dt;
        x = x + Vx * cos(theta) * dt;
        y = y + Vx * sin(theta) * dt;
    end
end
```

### 3. Collision Checking

**Robot footprint** (use your actual dimensions):
```matlab
robot_radius = 0.46;  % [m] Conservative bounding circle
safety_margin = 0.05; % [m] Extra clearance
inflation_radius = 0.51; % [m] Total buffer
```

**Check arc, not just endpoints**:
```matlab
% Sample arc at 10cm intervals
arc_length = sqrt(Vx^2 * duration^2 + (Wz * duration)^2);
num_checks = ceil(arc_length / 0.1);  % Check every 10cm

for i = 1:num_checks
    % Check collision at each sample point
    if isCollision(x(i), y(i), grid)
        return false;  % Invalid primitive
    end
end
```

### 4. Heuristic Function

**For differential drive**, Euclidean distance is actually **more appropriate** than Dubins:
- Dubins assumes min turning radius (Ackermann constraint)
- Your robot has **no min radius** (can spin in place)
- Euclidean is admissible and computationally cheap

```matlab
function h = heuristic(state, goal)
    % Simple Euclidean (admissible for diff-drive)
    dx = goal.x - state.x;
    dy = goal.y - state.y;
    h_pos = sqrt(dx^2 + dy^2);
    
    % Optional: Add heading penalty for smoother paths
    dtheta = abs(wrapToPi(goal.theta - state.theta));
    h_heading = 0.1 * dtheta;  % Small weight (non-admissible but practical)
    
    h = h_pos + h_heading;
end
```

---

## Key Differences vs Initial Design

| Original Assumption | Your Actual Robot | Impact |
|---------------------|-------------------|--------|
| Bicycle model (Ackermann) | Differential drive | Use (Vx, Wz) instead of steering angle |
| Min turning radius ~1m | Zero turning radius | Can spin in place! Easier planning |
| Dubins curves | Arbitrary arcs | Simpler motion primitives |
| Track 0.329m (compact) | Track 0.573m (fourwheel) | Lower max yaw rate (5.2 vs 9.1 rad/s) |
| Robot radius 0.3m | Robot radius 0.46m | Larger inflation, more conservative |

---

## Updated Parameter Set

From `gik9dof.getChassisParams('fourwheel')`:
```matlab
params.chassis_type = 'differential';
params.track = 0.573;              % [m] Your platform
params.wheel_radius = 0.1075;      % [m] 107.5mm wheels
params.robot_radius = 0.46;        % [m] Bounding circle
params.inflation_radius = 0.51;    % [m] With safety margin
params.Vx_max = 0.8;               % [m/s] Conservative
params.Wz_max = 2.5;               % [rad/s] Firmware limit
params.Wz_max_pure_spin = 5.2;     % [rad/s] Theoretical max
params.Wz_max_at_speed = 2.4;      % [rad/s] At 0.8 m/s
params.min_turning_radius = 0.32;  % [m] Effective (0.8/2.5)
params.accel_max = 2.0;            % [m/s²]
```

---

## Implementation Checklist

**Updated for differential drive**:
- [ ] Replace bicycle model with diff-drive kinematics
- [ ] Add in-place rotation primitives (Vx=0, Wz≠0)
- [ ] Use (Vx, Wz) motion primitives, not steering angles
- [ ] Simplify arc generation (no Dubins/Reed-Shepp needed)
- [ ] Use Euclidean heuristic (appropriate for zero-radius turns)
- [ ] Update collision checking with robot_radius = 0.46m
- [ ] Test with tight spaces (utilize spin-in-place advantage)

---

## Testing Scenarios (Differential-Specific)

1. **Tight corridor navigation**: Width < 1m (leverages spin capability)
2. **90° corner turn**: Should spin in place vs wide arc
3. **Parallel parking**: Back in, spin, exit (classic diff-drive maneuver)
4. **Obstacle avoidance**: Quick spin + forward vs smooth curve

---

**Next Steps**:
1. Implement differential drive motion primitives (10 total)
2. Update HybridState to use (Vx, Wz) instead of steer angle
3. Test with narrow passages to validate spin-in-place advantage

---

**Document Status**: DESIGN - Chassis-specific adaptation  
**Author**: GitHub Copilot + User  
**Last Updated**: October 7, 2025
