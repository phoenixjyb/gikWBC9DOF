# Pure Pursuit Quick Start Guide

**Single Node Architecture:** Everything runs in `gik9dof_solver_node` - no separate controllers or planners.

---

## Quick Mode Switch

Edit `ros2/gik9dof_solver/config/gik9dof_solver_params.yaml`:

```yaml
velocity_control_mode: 2  # 0=legacy, 1=heading, 2=purepursuit
```

Restart node for changes to take effect.

---

## Deploy to Orin

```powershell
.\deploy_to_orin_complete.ps1 -OrinIP "192.168.100.150"
```

---

## Build on Orin

```bash
ssh nvidia@192.168.100.150
cd /home/nvidia/camo_9dof/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_solver
```

---

## Run the Node

```bash
source install/setup.bash
ros2 run gik9dof_solver gik9dof_solver_node
```

Check startup messages - should say:
```
Velocity controller mode: 2 - Pure Pursuit (lookahead path following)
```

---

## Monitor Velocity Commands

```bash
ros2 topic echo /cmd_vel
```

Verify:
- `linear.x` < 1.5 m/s
- `angular.z` within reasonable limits
- Smooth, non-jerky values

---

## Quick Tuning

If robot **oscillates** → Increase lookahead:
```yaml
purepursuit:
  lookahead_base: 1.2  # was 0.8
```

If robot **cuts corners** → Decrease lookahead:
```yaml
purepursuit:
  lookahead_base: 0.5  # was 0.8
```

Restart node after param changes.

---

## Troubleshooting

**Problem:** Robot not moving  
**Check:** Is GIK publishing targets? `ros2 topic echo /gik9dof/target_trajectory`

**Problem:** Jerky motion  
**Try:** Increase `interp_spacing` from 0.05 to 0.10

**Problem:** Too slow in turns  
**Try:** Increase `vx_nominal` or decrease turn speed reduction logic

---

## Compare Controllers

Test Mode 0 (baseline):
```yaml
velocity_control_mode: 0
```

Test Mode 1 (heading control):
```yaml
velocity_control_mode: 1
```

Test Mode 2 (Pure Pursuit):
```yaml
velocity_control_mode: 2
```

Record bag file for each mode, compare performance offline.

---

**Full docs:** See `PUREPURSUIT_INTEGRATION_COMPLETE.md`
