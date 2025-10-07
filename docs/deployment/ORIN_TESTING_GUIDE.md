# Pure Pursuit - Orin Testing Quick Guide

**Date:** October 7, 2025  
**Deployed to:** `/home/nvidia/camo_9dof/gikWBC9DOF`  
**Orin IP:** 192.168.100.150

---

## 🚀 Quick Start on Orin

### **1. SSH to Orin**
```bash
ssh cr@192.168.100.150
# Or: ssh nvidia@192.168.100.150 (if username is nvidia)
```

### **2. Navigate to Workspace**
```bash
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
```

### **3. Build ROS2 Packages**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Build solver with Pure Pursuit
colcon build --packages-select gik9dof_solver
source install/setup.bash
```

**Expected:** Build should complete successfully (warnings only, no errors)

### **4. Run with Pure Pursuit Mode**
```bash
# Default config (Pure Pursuit enabled, mode 2)
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

**Expected startup log:**
```
[INFO] GIK9DOF Solver Node initialized
[INFO] MATLAB solver initialized
[INFO] Control rate: 10.0 Hz
[INFO] Velocity controller mode: 2 - Pure Pursuit (lookahead path following)
[INFO]   Pure Pursuit params:
[INFO]     Lookahead: base=0.80, vel_gain=0.30, time_gain=0.10
[INFO]     Velocity: nominal=1.00 m/s, max=1.50 m/s, wz_max=2.00 rad/s
[INFO]     Path: buffer=30 waypoints, spacing=0.15 m, tolerance=0.20 m
```

---

## 🎯 Controller Mode Selection

### **Run with Pure Pursuit (Mode 2) - DEFAULT**
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args \
    --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

### **Run with Simple Heading Controller (Mode 1)**
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args \
    --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml \
    -p velocity_control_mode:=1
```

### **Run with Legacy 5-Point Differentiation (Mode 0)**
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args \
    --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml \
    -p velocity_control_mode:=0
```

---

## 🔧 Runtime Parameter Tuning

### **Adjust Lookahead Distance (Most Important)**
```bash
# Increase for smoother, wider turns
ros2 param set /gik9dof_solver_node purepursuit.lookahead_base 1.2

# Decrease for tighter tracking
ros2 param set /gik9dof_solver_node purepursuit.lookahead_base 0.6
```

### **Adjust Nominal Speed**
```bash
# Slower cruising speed
ros2 param set /gik9dof_solver_node purepursuit.vx_nominal 0.7

# Faster cruising speed
ros2 param set /gik9dof_solver_node purepursuit.vx_nominal 1.2
```

### **Switch Controller Mode Live**
```bash
# Switch to Pure Pursuit
ros2 param set /gik9dof_solver_node velocity_control_mode 2

# Switch to Simple Heading
ros2 param set /gik9dof_solver_node velocity_control_mode 1

# Switch to Legacy
ros2 param set /gik9dof_solver_node velocity_control_mode 0
```

---

## 📊 Monitoring

### **Check Published Velocities**
```bash
# Monitor cmd_vel output
ros2 topic echo /cmd_vel

# Expected: vx ≤ 1.5 m/s, wz within limits
```

### **Check Path Buffer Status**
Enable debug logging to see waypoint buffer:
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args \
    --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml \
    --log-level debug
```

Look for:
```
[DEBUG] Pure Pursuit: Vx=1.000 m/s, Wz=0.250 rad/s | Buffer: 15 waypoints | ref: x=2.50, y=1.20, θ=0.45
```

### **Monitor Node Status**
```bash
# Check if node is running
ros2 node list

# Check topics
ros2 topic list

# Check parameters
ros2 param list /gik9dof_solver_node | grep purepursuit
```

---

## 🧪 Testing Procedure

### **Test 1: Straight Line**
1. Send straight-line position references
2. Expected: Low angular velocity (|wz| < 0.1), stable forward motion
3. Check: Buffer accumulates waypoints, removes passed ones

### **Test 2: Circular Path**
1. Send circular position references
2. Expected: Smooth constant angular velocity, no oscillations
3. Check: Speed reduction in turns (vx < vx_nominal)

### **Test 3: Sharp Corner**
1. Send 90-degree corner waypoints
2. Expected: Anticipatory turning (starts before corner), smooth path
3. Check: No overshoot, stable tracking

### **Test 4: Sparse Waypoints**
1. Send waypoints sparsely (>0.5s apart)
2. Expected: Time-adaptive lookahead compensates, smooth motion
3. Check: Buffer doesn't overflow, waypoints managed properly

### **Test 5: Compare Modes**
1. Run same path with mode 0, 1, 2
2. Compare: smoothness, overshoot, tracking error
3. Expected: Mode 2 (Pure Pursuit) smoothest, least oscillation

---

## 🐛 Troubleshooting

### **Build Fails on Orin**
```bash
# Check architecture
uname -m  # Should show: aarch64

# Verify ROS2 sourced
echo $ROS_DISTRO  # Should show: humble

# Clean and rebuild
rm -rf build install log
colcon build --packages-select gik9dof_msgs
source install/setup.bash
colcon build --packages-select gik9dof_solver
```

### **Robot Oscillates**
- **Issue:** Lookahead too small
- **Fix:** Increase `purepursuit.lookahead_base` to 1.0-1.5m

### **Robot Cuts Corners**
- **Issue:** Lookahead too large
- **Fix:** Decrease `purepursuit.lookahead_base` to 0.5-0.7m

### **Robot Too Slow in Turns**
- **Issue:** Conservative speed reduction
- **Fix:** Adjust turn speed logic in purePursuitVelocityController.cpp lines 267-273

### **Velocities Exceed Limits**
- **Check:** Wheel speed limit enforcement working
- **Verify:** `vwheel_max`, `vx_max`, `wz_max` parameters
- **Monitor:** Actual wheel commands

### **Path Buffer Not Growing**
- **Issue:** Waypoints too close together
- **Check:** `purepursuit.waypoint_spacing` (default 0.15m)
- **Monitor:** Debug logs show new waypoints added

---

## 📈 Performance Metrics

**Expected Performance:**
- **Control rate:** 10 Hz (100ms period)
- **Max forward speed:** 1.5 m/s
- **Max angular rate:** 2.0 rad/s
- **Path buffer:** 5-30 waypoints (depending on path)
- **Lookahead:** 0.8-2.5m (adaptive)
- **CPU usage:** <5% on Orin (ARM Cortex-A78)

**Benchmark:**
```bash
# Monitor CPU usage
top -p $(pgrep gik9dof_solver)

# Monitor message rates
ros2 topic hz /cmd_vel
```

---

## 🔄 Iteration Workflow

1. **Test** - Run with default parameters
2. **Observe** - Monitor behavior, check logs
3. **Tune** - Adjust one parameter at a time
4. **Verify** - Confirm improvement
5. **Document** - Note optimal parameters
6. **Compare** - Test against other modes (0, 1)

---

## 📝 Configuration File Location

**On Orin:**
```
/home/nvidia/camo_9dof/gikWBC9DOF/ros2/src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

**Edit parameters:**
```bash
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
nano src/gik9dof_solver/config/gik9dof_solver_params.yaml

# After editing, restart node to load new params
```

---

## ✅ Success Criteria

**Pure Pursuit is working correctly if:**
1. ✅ Node starts without errors
2. ✅ Startup log shows "Mode 2 - Pure Pursuit"
3. ✅ Path buffer accumulates waypoints (visible in debug logs)
4. ✅ Smooth velocity commands (no sudden jumps)
5. ✅ Velocities within limits (vx ≤ 1.5, wz ≤ 2.0)
6. ✅ No oscillations on straight paths
7. ✅ Smooth cornering without overshoot
8. ✅ Better performance than mode 0 and mode 1

---

## 📞 Next Steps

1. **Build on Orin** - Verify ARM64 compilation succeeds
2. **Run node** - Check startup logs confirm Pure Pursuit mode
3. **Send test commands** - Validate path following
4. **Tune parameters** - Optimize for your robot
5. **Benchmark** - Compare all 3 modes
6. **Document** - Record optimal parameters

---

**Good luck with testing!** 🎯

If you encounter issues, check:
- Build logs for compilation errors
- Runtime logs for parameter loading
- `/cmd_vel` topic for velocity commands
- Debug logs for waypoint buffer status
