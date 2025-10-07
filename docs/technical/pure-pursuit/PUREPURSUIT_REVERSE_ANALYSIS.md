# Pure Pursuit Bidirectional Motion Analysis

**Date**: October 7, 2025  
**Issue**: Pure Pursuit controller is **forward-only**, not utilizing chassis reverse capability

## Current Implementation Analysis

### Velocity Clamping (Line 296)
```cpp
*vx = std::fmax(0.0, std::fmin(params->vxMax, idx));
```
**Result**: `vx ∈ [0, vxMax]` - **NO NEGATIVE VALUES**

### Configuration (`gik9dof_solver.yaml`)
```yaml
vx_nominal: 1.0           # Always positive
vx_max: 1.5               # Always positive  
wz_max: 2.0               # Can be ± (rotation both directions)
```

## Problem Statement

**Standard Pure Pursuit Assumption**: Robot always moves forward toward target
- Works well for: Car-like robots, Ackermann steering
- **Limitation**: Cannot reverse, must rotate-in-place first

**Your Chassis Capability**: Differential drive with **bidirectional motion**
- Can move: Forward (+vx), Backward (-vx), Rotate (±wz)
- **Unused capability**: Reverse motion

## Impact

### Inefficient Maneuvering
```
Scenario: Target is 1m behind robot

Current behavior (forward-only):
1. Rotate 180° in place (takes ~1.5s at wz=2.0 rad/s)
2. Drive forward 1m
3. Rotate 180° back to original heading
Total time: ~4-5 seconds

Efficient behavior (with reverse):
1. Drive backward 1m
Total time: ~1 second
```

### Cannot Handle:
- ❌ Tight spaces requiring backup
- ❌ Obstacle avoidance by reversing
- ❌ Efficient path execution when waypoints alternate direction

## Solutions

### Option 1: Add Reverse Support to Pure Pursuit (RECOMMENDED)

**Modify velocity clamping to allow negative velocities:**

```cpp
// BEFORE (forward-only):
*vx = std::fmax(0.0, std::fmin(params->vxMax, idx));

// AFTER (bidirectional):
*vx = std::fmax(-params->vxMax, std::fmin(params->vxMax, idx));
```

**Update configuration to specify min/max:**
```yaml
purepursuit:
  vx_nominal: 1.0           # Nominal forward velocity (m/s)
  vx_max: 1.5               # Max forward velocity (m/s)
  vx_min: -1.0              # Max reverse velocity (m/s) - NEW!
  # OR use symmetric:
  # vx_max: 1.5             # Max speed in either direction
```

**Add direction logic:**
```cpp
// Check if lookahead point is behind robot
double dx_robot_frame = /* transformed x */;
if (dx_robot_frame < 0) {
    // Target behind robot, allow reverse
    vx_sign = -1.0;
} else {
    vx_sign = 1.0;
}
*vx = vx_sign * std::abs(calculated_vx);
```

**Pros**:
- ✅ Uses existing Pure Pursuit framework
- ✅ Simple modification
- ✅ Efficient maneuvering
- ✅ Utilizes chassis reverse capability

**Cons**:
- ⚠️ Need to handle direction switching smoothly
- ⚠️ May oscillate near goal if not tuned properly

### Option 2: Keep Forward-Only (SIMPLE)

**Keep current implementation, accept limitations**

**Pros**:
- ✅ Proven stable behavior
- ✅ No code changes needed
- ✅ Predictable motion

**Cons**:
- ❌ Inefficient for targets behind robot
- ❌ Cannot use reverse capability
- ❌ May get stuck in tight spaces

### Option 3: Hybrid Approach (COMPLEX)

**Use different strategies based on scenario:**
- Forward motion for normal tracking
- Reverse motion only in specific cases (e.g., parking, tight spaces)
- Controlled by higher-level planner decision

**Pros**:
- ✅ Best of both worlds
- ✅ Context-aware behavior

**Cons**:
- ⚠️ More complex state machine
- ⚠️ Harder to tune

## Recommended Configuration Changes

### If Enabling Reverse (Option 1):

```yaml
# gik9dof_solver.yaml
purepursuit:
  lookahead_base: 0.8       
  lookahead_vel_gain: 0.3   
  lookahead_time_gain: 0.1  
  vx_nominal: 1.0           # Nominal forward speed
  vx_max: 1.5               # Max forward speed
  vx_min: -1.0              # Max reverse speed (NEW!)
  vx_reverse_threshold: -0.5 # Distance behind robot to trigger reverse (NEW!)
  wz_max: 2.0               
  track: 0.674              
  vwheel_max: 2.0           
  waypoint_spacing: 0.15    
  path_buffer_size: 30.0    
  goal_tolerance: 0.2       
  interp_spacing: 0.05      
```

### Code Changes Required:

**File**: `ros2/gik9dof_solver/src/purepursuit/purePursuitVelocityController.cpp`

1. **Add vx_min parameter** (line ~35)
2. **Check lookahead direction** (line ~285)
3. **Update velocity clamping** (line ~296)
4. **Smooth direction transitions** (add hysteresis)

## Chassis Hardware Considerations

### Verify Chassis Capabilities:

Before enabling reverse, confirm:

1. **Motor controllers support bidirectional PWM**
   ```
   Forward:  PWM > 0
   Reverse:  PWM < 0
   ```

2. **Wheel encoders measure direction**
   ```
   Forward:  encoder_delta > 0
   Reverse:  encoder_delta < 0
   ```

3. **Safety systems allow reverse**
   - Rear obstacle detection?
   - E-stop behavior?
   - Speed limits in reverse?

4. **cmd_vel topic interpretation**
   ```
   geometry_msgs/Twist:
     linear.x > 0  →  Forward
     linear.x < 0  →  Reverse  ✓ Should work!
     angular.z     →  Yaw rate (+ = CCW, - = CW)
   ```

### Typical Differential Drive Kinematics:
```
v_left = vx - (track/2) * wz
v_right = vx + (track/2) * wz

If vx < 0:  BOTH wheels reverse
If vx > 0:  BOTH wheels forward
```

**Assumption**: Your chassis low-level controller correctly interprets negative `linear.x` as reverse motion.

## Testing Strategy

### Phase 1: Configuration Check (SAFE)
```bash
# Publish manual cmd_vel to test reverse
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}" --once

# Expected: Robot moves backward at 0.5 m/s
# If robot doesn't move OR moves forward: HARDWARE ISSUE
```

### Phase 2: Modified Pure Pursuit (CONTROLLED)
1. Enable reverse with conservative limits:
   ```yaml
   vx_min: -0.5  # Start slow
   ```

2. Test simple scenarios:
   - Target 0.5m behind robot
   - Monitor smooth transitions
   - Check for oscillations

3. Gradually increase limits:
   ```yaml
   vx_min: -1.0  # Match forward speed
   ```

### Phase 3: Full Validation
- Complex paths with direction changes
- Verify wheel speed limits respected
- Test emergency stops during reverse

## Recommendation

**For your application**, I recommend **Option 1** (Enable Reverse) because:

1. ✅ Your chassis is differential drive (supports reverse)
2. ✅ More efficient maneuvering (avoid unnecessary rotations)
3. ✅ Better obstacle avoidance capability
4. ✅ Future-proof for complex scenarios (parking, docking, tight spaces)

**Implementation Priority**:
- **Low effort**: Change velocity clamping to allow negative
- **Medium effort**: Add direction detection logic
- **High effort**: Tune hysteresis to avoid oscillations

**Next Step**: Would you like me to create a patched version of the Pure Pursuit controller with bidirectional support?

---

## Current Status

**Pure Pursuit**: ✅ Works (forward-only)  
**Reverse Support**: ❌ Not implemented  
**Chassis Capability**: ✓ Assumed bidirectional (needs verification)

**Action Required**: Decide on Option 1, 2, or 3 and test chassis reverse capability.
