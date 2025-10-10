# Chassis Path Follower - ROS2 Integration Guide

## Overview

This guide explains how to build and use the MATLAB Coder generated chassis path follower in your ROS2 workspace. The controller supports 3 modes:

- **Mode 0**: 5-point differentiation (open-loop feedforward)
- **Mode 1**: Heading-aware P-controller with feedforward
- **Mode 2**: Pure pursuit with full limiting (default)

## Quick Start

### Build

```bash
# Navigate to ROS2 workspace
cd ros2

# Build the controllers package
colcon build --packages-select gik9dof_controllers

# Source the workspace
source install/setup.bash
```

### Launch

```bash
# Launch with default parameters (Mode 2: Pure Pursuit)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py

# Launch with Mode 1 (Heading-Aware)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1

# Launch with Mode 0 (Differentiation)
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0

# Launch with custom parameters
ros2 launch gik9dof_controllers chassis_path_follower_launch.py \
    controller_mode:=2 \
    vx_max:=1.5 \
    lookahead_base:=0.5
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/path` | `nav_msgs/Path` | Target path to follow (up to 500 points) |
| `/odom` | `nav_msgs/Odometry` | Current robot odometry |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |
| `/path_following_active` | `std_msgs/Bool` | True when actively following path |

## Parameters

### Controller Selection

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `controller_mode` | int | 2 | 0=differentiation, 1=heading, 2=pure pursuit |

### Velocity Limits

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vx_max` | double | 1.0 | Maximum forward velocity (m/s) |
| `vx_min` | double | -0.5 | Maximum reverse velocity (m/s) |
| `wz_max` | double | 1.0 | Maximum angular velocity (rad/s) |

### Acceleration Limits

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `accel_max` | double | 0.5 | Maximum acceleration (m/s²) |
| `decel_max` | double | 0.8 | Maximum deceleration (m/s²) |
| `jerk_limit` | double | 2.0 | Maximum jerk (m/s³) |

### Lookahead Parameters (Modes 1 & 2)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lookahead_base` | double | 0.3 | Base lookahead distance (m) |
| `lookahead_gain` | double | 0.5 | Velocity-dependent gain |
| `lookahead_min` | double | 0.2 | Minimum lookahead (m) |
| `lookahead_max` | double | 2.0 | Maximum lookahead (m) |

### Pure Pursuit Parameters (Mode 2 Only)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kappa_threshold` | double | 0.5 | Curvature threshold for slowdown (1/m) |
| `vx_reduction` | double | 0.3 | Minimum speed scaling factor |

### Heading Controller Gains (Mode 1 Only)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `heading_kp` | double | 2.0 | Proportional gain for heading error |
| `feedforward_gain` | double | 0.8 | Feedforward gain for trajectory |

### Chassis Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `track_width` | double | 0.5 | Distance between wheels (m) |
| `wheel_radius` | double | 0.1 | Wheel radius (m) |
| `wheel_speed_max` | double | 2.0 | Max wheel angular velocity (rad/s) |

### Other Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal_tolerance` | double | 0.1 | Distance to goal for completion (m) |
| `reverse_enabled` | bool | false | Allow reverse motion |

## Controller Modes Explained

### Mode 0: 5-Point Differentiation (Open-Loop)

**Best for**: Smooth, pre-planned trajectories with no disturbances

**How it works**:
- Uses 5-point finite difference to compute velocities from path
- Transforms world velocities to robot body frame
- Direct feedforward with no feedback correction

**Advantages**:
- Fastest computation
- No oscillation or instability
- Works well for smooth paths

**Limitations**:
- No error correction
- Sensitive to initial conditions
- No disturbance rejection

### Mode 1: Heading-Aware (Simple Feedback)

**Best for**: General-purpose tracking with good stability

**How it works**:
- P-control on heading error: `wz = Kp * error`
- Feedforward from trajectory derivatives
- Adaptive lookahead based on velocity/acceleration

**Advantages**:
- Simple and robust
- Good balance of performance and stability
- Fewer parameters to tune

**Limitations**:
- Less sophisticated than pure pursuit
- May have larger cross-track errors

### Mode 2: Pure Pursuit (Full Feedback)

**Best for**: High-precision tracking with complex paths

**How it works**:
- Geometric tracking with lookahead point
- Curvature-based speed control
- Full acceleration/jerk/wheel speed limiting

**Advantages**:
- Best tracking performance
- Handles high curvature paths
- Smooth acceleration profiles

**Limitations**:
- More parameters to tune
- Slightly higher computation
- May oscillate if gains too high

## Usage Examples

### Example 1: Send a Simple Path

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, 'path', 10)
        
    def publish_straight_line(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Create 10m straight line with 50 points
        for i in range(50):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = i * 0.2  # 0 to 10m
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0  # Facing forward
            path.poses.append(pose)
            
        self.publisher.publish(path)
        self.get_logger().info('Published straight line path')

def main():
    rclpy.init()
    node = PathPublisher()
    node.publish_straight_line()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Circular Path

```python
def publish_circle(self, radius=2.0, points=100):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = self.get_clock().now().to_msg()
    
    for i in range(points):
        angle = 2.0 * math.pi * i / points
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = radius * math.cos(angle)
        pose.pose.position.y = radius * math.sin(angle)
        
        # Tangent angle
        yaw = angle + math.pi / 2
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        path.poses.append(pose)
        
    self.publisher.publish(path)
```

### Example 3: Monitor Status

```bash
# Watch velocity commands
ros2 topic echo /cmd_vel

# Watch path following status
ros2 topic echo /path_following_active

# Check node parameters
ros2 param list /chassis_path_follower

# Update parameter at runtime
ros2 param set /chassis_path_follower vx_max 1.5
```

## Tuning Guide

### For Aggressive Tracking (Mode 2)

```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py \
    controller_mode:=2 \
    vx_max:=1.5 \
    lookahead_base:=0.4 \
    lookahead_gain:=0.6 \
    kappa_threshold:=0.8 \
    vx_reduction:=0.5
```

### For Smooth, Conservative Tracking (Mode 1)

```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py \
    controller_mode:=1 \
    vx_max:=0.8 \
    heading_kp:=1.5 \
    feedforward_gain:=0.9 \
    accel_max:=0.3
```

### For High-Speed Open-Loop (Mode 0)

```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py \
    controller_mode:=0 \
    vx_max:=2.0 \
    accel_max:=1.0 \
    jerk_limit:=3.0
```

## Troubleshooting

### Robot doesn't move
- Check that path is published: `ros2 topic echo /path`
- Check that odometry is published: `ros2 topic echo /odom`
- Verify node is running: `ros2 node list`
- Check `/path_following_active` status

### Oscillation or instability
- **Mode 1**: Reduce `heading_kp`, increase `feedforward_gain`
- **Mode 2**: Increase `lookahead_base`, reduce `vx_max`
- All modes: Reduce `accel_max` and `jerk_limit`

### Poor tracking accuracy
- **Mode 1**: Increase `heading_kp`, reduce `lookahead_min`
- **Mode 2**: Reduce `lookahead_base`, increase `kappa_threshold`
- All modes: Reduce `vx_max` for tighter corners

### Wheel speed saturation
- Reduce `vx_max` and `wz_max`
- Increase `track_width` (if physically accurate)
- Reduce `wheel_speed_max` (if it's too high)

## File Structure

```
ros2/gik9dof_controllers/
├── CMakeLists.txt                     # Build configuration
├── package.xml                        # ROS2 package manifest
├── launch/
│   └── chassis_path_follower_launch.py  # Launch file
├── src/
│   ├── chassis_path_follower_node.cpp   # ROS2 wrapper node
│   └── matlab_generated/
│       └── chassis_path_follower/
│           ├── ChassisPathFollower.cpp  # MATLAB generated (33.6 KB)
│           ├── ChassisPathFollower.h    # Class interface
│           ├── chassisPathFollowerCodegen_types.h  # Type definitions
│           ├── rtwtypes.h               # Runtime types
│           └── (15+ other generated files)
└── include/
    └── gik9dof_controllers/
        └── (headers for other controllers)
```

## Performance Notes

- **Control Rate**: 100 Hz (10ms cycle time)
- **Max Path Points**: 500 (limited by codegen)
- **Computation Time**: < 1ms typical on ARM Cortex-A78
- **Memory**: ~100 KB per controller instance

## ARM64 Optimizations

The generated code includes ARM64-specific optimizations:
- **Architecture**: ARMv8-A (Cortex-A78 tuned)
- **SIMD**: NEON vectorization enabled
- **Compiler Flags**: `-O3 -ffast-math -march=armv8-a`

These provide ~2-3x speedup compared to unoptimized code.

## Next Steps

1. **Test in simulation** before hardware deployment
2. **Record rosbags** for offline analysis
3. **Tune parameters** for your specific robot
4. **Compare modes** to find best fit
5. **Profile performance** on target hardware

## Support

For issues or questions:
- Check MATLAB tests: `matlab/test_chassis_path_3modes.m`
- Review codegen summary: `DAY5_CODEGEN_SESSION_SUMMARY.md`
- Check generated code: `codegen/chassis_path_follower_arm64/`
