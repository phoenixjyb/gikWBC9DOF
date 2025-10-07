# Quick Start Guide - Velocity Controller Testing

**Last Updated:** October 7, 2025  
**For:** Testing the new holistic velocity controller vs legacy 5-point differentiation

---

## 🚀 Quick Deploy & Test (3 Steps)

### Step 1: Deploy to Orin
```powershell
# From Windows PowerShell
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

# Deploy to /home/nvidia/camo_9dof/gikWBC9DOF on Orin
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.x.x"
```

### Step 2: Run on Orin
```bash
# SSH to Orin
ssh nvidia@<orin-ip>

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
source install/setup.bash

# Run with configuration file
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

**Expected startup log:**
```
[gik9dof_solver_node]: Velocity controller: Holistic (heading control)
[gik9dof_solver_node]:   Controller params: track=0.500, Vx_max=1.00, W_max=2.00, Kp=2.00, Kff=0.90
```

### Step 3: Test Switching
```bash
# In another terminal on Orin

# Check current mode
ros2 param get /gik9dof_solver_node use_velocity_controller
# Should return: true

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Switch to legacy mode
ros2 param set /gik9dof_solver_node use_velocity_controller false
# Observe change in cmd_vel output

# Switch back to new controller
ros2 param set /gik9dof_solver_node use_velocity_controller true
```

---

## 🧪 Performance Comparison Test

### Setup: Record Both Modes

**Terminal 1 (Orin): Run node**
```bash
ros2 run gik9dof_solver gik9dof_solver_node \
    --ros-args --params-file src/gik9dof_solver/config/gik9dof_solver_params.yaml
```

**Terminal 2 (Orin): Send test trajectory**
```bash
# Send your test trajectory command here
# Example: circular path, straight line, etc.
ros2 topic pub /gik9dof/target_trajectory ...
```

**Terminal 3 (Development machine): Record data**
```bash
# Test 1: New controller
ros2 param set /gik9dof_solver_node use_velocity_controller true
ros2 bag record /cmd_vel /odom_wheel /gik9dof/target_trajectory -o test_new_controller

# Wait for trajectory to complete, then Ctrl+C

# Test 2: Legacy controller
ros2 param set /gik9dof_solver_node use_velocity_controller false
ros2 bag record /cmd_vel /odom_wheel /gik9dof/target_trajectory -o test_legacy_controller
```

### Analyze Results

**Metrics to compare:**
1. Position tracking error (RMSE)
2. Heading tracking error (RMSE)
3. Velocity smoothness (acceleration, jerk)
4. Wheel speed violations (should be 0 with new controller)

**Expected improvements:**
- Position RMSE: ~50% better
- Heading RMSE: ~60% better
- Smoother velocity commands
- No wheel limit violations

---

## 🎛️ Parameter Tuning

### Current Default Settings
```yaml
vel_ctrl:
  track: 0.5      # Wheel track width (m)
  vx_max: 1.0     # Max forward velocity
  w_max: 2.0      # Max yaw rate
  yaw_kp: 2.0     # Heading P gain
  yaw_kff: 0.9    # Yaw feedforward
```

### If Robot Oscillates (Too Aggressive)
```bash
# Reduce proportional gain
ros2 param set /gik9dof_solver_node vel_ctrl.yaw_kp 1.0

# Reduce feedforward
ros2 param set /gik9dof_solver_node vel_ctrl.yaw_kff 0.7
```

### If Robot Tracks Slowly (Too Conservative)
```bash
# Increase proportional gain
ros2 param set /gik9dof_solver_node vel_ctrl.yaw_kp 3.0

# Increase feedforward
ros2 param set /gik9dof_solver_node vel_ctrl.yaw_kff 0.95
```

### Safety Limits (Adjust for Your Robot)
```bash
# Max forward speed (m/s)
ros2 param set /gik9dof_solver_node vel_ctrl.vx_max 0.8

# Max rotation rate (rad/s)
ros2 param set /gik9dof_solver_node vel_ctrl.w_max 1.5

# Wheel track width (measure your robot!)
ros2 param set /gik9dof_solver_node vel_ctrl.track 0.48
```

---

## 🐛 Troubleshooting

### Problem: Node crashes on startup
**Check:** Generated files copied correctly
```bash
ls -la src/gik9dof_solver/include/velocity_controller/
ls -la src/gik9dof_solver/src/velocity_controller/
```

**Fix:** Rebuild clean
```bash
colcon build --packages-select gik9dof_solver --cmake-clean-cache
```

### Problem: All velocities are zero
**Check:** Topics publishing
```bash
ros2 topic hz /odom_wheel  # Should be ~10Hz
ros2 topic echo /gik9dof/target_trajectory  # Should show data
```

**Debug:** Enable verbose logging
```bash
ros2 run gik9dof_solver gik9dof_solver_node --ros-args --log-level debug
```

### Problem: Can't switch modes
**Symptom:** Parameter set succeeds but behavior doesn't change

**Note:** Current implementation requires node restart for the switch to take effect. The parameter controls which branch of code runs in `publishBaseCommand()`, but the initialization happens in the constructor.

**Workaround:** Restart node after parameter change
```bash
# Kill node
Ctrl+C

# Set parameter via config file edit, or:
ros2 param set /gik9dof_solver_node use_velocity_controller false

# Restart node
ros2 run gik9dof_solver gik9dof_solver_node --ros-args ...
```

---

## 📊 Quick Test Checklist

- [ ] Node starts without errors
- [ ] Startup log shows correct controller mode
- [ ] `/cmd_vel` publishes at ~10Hz
- [ ] Velocity commands are non-zero during motion
- [ ] Switch parameter changes controller mode (after restart)
- [ ] Legacy mode matches previous behavior
- [ ] New mode produces smooth trajectories
- [ ] Wheel limits are respected (new mode)
- [ ] Parameters can be tuned at runtime

---

## 📈 Success Criteria

### Functional Requirements
✅ Node compiles and runs  
✅ Both controller modes work  
✅ Runtime parameter switch functional  
✅ No crashes or exceptions  

### Performance Requirements
🎯 Position tracking: < 5cm RMSE  
🎯 Heading tracking: < 2° RMSE  
🎯 Velocity smoothness: < 1 m/s² acceleration  
🎯 Wheel limits: 0 violations  

### Comparison Requirements
📊 New controller ≥ legacy in tracking accuracy  
📊 New controller smoother velocity commands  
📊 No regressions in system stability  

---

## 🔗 Related Documentation

- **Full integration guide:** `ROS2_INTEGRATION_COMPLETE.md`
- **Code generation:** `VELOCITY_CONTROLLER_CODEGEN_SUCCESS.md`
- **Build verification:** `BUILD_VERIFICATION.md`
- **Session summary:** `SESSION_SUMMARY_PHASE2A.md`
- **Configuration file:** `ros2/gik9dof_solver/config/gik9dof_solver_params.yaml`

---

**Ready to test!** 🚀  
Start with Step 1 above and work through the checklist.
