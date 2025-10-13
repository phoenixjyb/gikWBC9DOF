# Orin Deployment Instructions

**Date**: 2025-10-11  
**Package**: `orin_chassis_follower_20251011_132300.zip`  
**Size**: 4.24 MB  
**Status**: ✅ Transferred to Orin (cr@192.168.100.150)

---

## ✅ Completed Steps

1. **Package Created** ✅
   - Location: `deployments/orin_chassis_follower_20251011_132300/`
   - Size: 4.24 MB
   - Contents: ROS2 workspace + ARM64 codegen + build scripts

2. **Transferred to Orin** ✅
   - Method: `scp` via WSL
   - Target: `cr@192.168.100.150:~/`
   - Transfer time: ~25 seconds

---

## 🚀 Next Steps on Orin

### Step 1: SSH to Orin

```bash
ssh cr@192.168.100.150
```

### Step 2: Extract Package

```bash
cd ~
unzip orin_chassis_follower_20251011_132300.zip
cd orin_chassis_follower_20251011_132300
```

**Expected output**:
```
Archive:  orin_chassis_follower_20251011_132300.zip
  inflating: README.md
  inflating: build_on_orin.sh
  inflating: test_on_orin.sh
   creating: ros2/
   creating: codegen/
   ...
```

### Step 3: Make Scripts Executable

```bash
chmod +x build_on_orin.sh
chmod +x test_on_orin.sh
```

### Step 4: Build ROS2 Workspace

```bash
./build_on_orin.sh
```

**Expected output**:
```
===========================================
  Building ROS2 Workspace on Orin
===========================================

[1/3] Sourcing ROS2 Humble...
[2/3] Cleaning previous build...
[3/3] Building workspace...

Starting >>> gik9dof_msgs
Finished <<< gik9dof_msgs [~10s]
Starting >>> gik9dof_controllers
Starting >>> gik9dof_solver
Finished <<< gik9dof_controllers [~40s]
Finished <<< gik9dof_solver [~5min]

✅ Build successful!

To use the workspace:
  source install/setup.bash

To launch chassis path follower:
  ros2 launch gik9dof_controllers chassis_path_follower_launch.py
```

**Expected build time**: 5-6 minutes total

### Step 5: Test the Node

```bash
./test_on_orin.sh
```

**Expected output**:
```
===========================================
  Testing Chassis Path Follower
===========================================

[Test 1] Checking node availability...
  ✅ gik9dof_controllers package found

[Test 2] Launching node (Mode 2 - Pure Pursuit)...
  Press Ctrl+C to stop

[INFO] [chassis_path_follower]: Chassis Path Follower Node initialized
[INFO] [chassis_path_follower]: Controller Mode: 2 (0=diff, 1=heading, 2=pure pursuit)
[INFO] [chassis_path_follower]: Controller initialized with mode 2
```

---

## 🧪 Testing the Controller

### Test 1: Launch Node (Different Modes)

**Mode 0 - Differentiation Controller:**
```bash
cd ~/orin_chassis_follower_20251011_132300/ros2
source install/setup.bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=0
```

**Mode 1 - Heading Controller:**
```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=1
```

**Mode 2 - Pure Pursuit (Default):**
```bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py controller_mode:=2
```

### Test 2: Publish Test Path

Open a **second terminal** on Orin:

```bash
ssh cr@192.168.100.150
cd ~/orin_chassis_follower_20251011_132300/ros2
source install/setup.bash

# Publish straight line path (4 points)
ros2 topic pub /path nav_msgs/msg/Path '{
  header: {frame_id: "map"},
  poses: [
    {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]
}' -1
```

### Test 3: Monitor Velocity Commands

Open a **third terminal** on Orin:

```bash
ssh cr@192.168.100.150
cd ~/orin_chassis_follower_20251011_132300/ros2
source install/setup.bash

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

**Expected output**:
```yaml
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.12
---
```

### Test 4: Check Status

```bash
# Path following status
ros2 topic echo /path_following_active

# Node info
ros2 node info /chassis_path_follower

# List all topics
ros2 topic list
```

---

## 📊 Controller Parameters

Key parameters that can be modified in the launch file:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `controller_mode` | 2 | 0=differentiation, 1=heading, 2=pure pursuit |
| `vx_max` | 1.0 | Maximum linear velocity (m/s) |
| `vx_min` | -0.5 | Minimum linear velocity (m/s) |
| `wz_max` | 1.0 | Maximum angular velocity (rad/s) |
| `accel_max` | 0.5 | Maximum acceleration (m/s²) |
| `decel_max` | 0.8 | Maximum deceleration (m/s²) |
| `jerk_limit` | 2.0 | Jerk limit (m/s³) |
| `lookahead_base` | 0.3 | Base lookahead distance (m) |
| `lookahead_gain` | 0.5 | Velocity-dependent lookahead gain |
| `heading_kp` | 2.0 | Heading controller proportional gain |
| `feedforward_gain` | 0.8 | Feedforward gain for heading control |
| `goal_tolerance` | 0.1 | Goal reached distance threshold (m) |
| `track_width` | 0.5 | Robot track width (m) |
| `wheel_speed_max` | 2.0 | Maximum wheel speed (rad/s) |

---

## 🔍 Troubleshooting

### Build Fails

**Problem**: Build errors during `./build_on_orin.sh`

**Solution**:
```bash
cd ~/orin_chassis_follower_20251011_132300/ros2
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Node Won't Start

**Problem**: Node crashes or doesn't start

**Check 1**: Verify ROS2 environment
```bash
env | grep ROS
```

**Check 2**: Verify package is installed
```bash
ros2 pkg list | grep gik9dof
```

**Check 3**: Check for error messages
```bash
ros2 run gik9dof_controllers chassis_path_follower_node --ros-args --log-level debug
```

### No Velocity Output

**Problem**: Node runs but doesn't publish velocity commands

**Check 1**: Verify path was received
```bash
ros2 topic echo /path
```

**Check 2**: Verify odometry is publishing
```bash
ros2 topic echo /odom
```

**Check 3**: Check if path following is active
```bash
ros2 topic echo /path_following_active
```

**Possible causes**:
- No path published yet
- No odometry data available
- Goal already reached
- Controller not initialized

### Performance Issues

**Problem**: High CPU usage or slow response

**Solution 1**: Lower control rate
Edit `chassis_path_follower_node.cpp` line ~45:
```cpp
// Change from 100 Hz to 50 Hz
control_timer_ = create_wall_timer(
  std::chrono::milliseconds(20),  // Was 10
  std::bind(&ChassisPathFollowerNode::control_loop, this));
```

**Solution 2**: Check CPU usage
```bash
top -H
htop
```

---

## 📝 Package Contents

```
orin_chassis_follower_20251011_132300/
├── README.md                      # Full documentation
├── build_on_orin.sh              # Automated build script
├── test_on_orin.sh               # Automated test script
├── ros2/                         # ROS2 workspace
│   ├── gik9dof_msgs/            # Message definitions
│   ├── gik9dof_controllers/     # Chassis path follower
│   │   ├── src/
│   │   │   ├── chassis_path_follower_node.cpp
│   │   │   └── matlab_generated/
│   │   │       ├── ChassisPathFollower.cpp
│   │   │       ├── ChassisPathFollower.h
│   │   │       ├── chassisPathFollowerCodegen_types.h
│   │   │       └── ... (18 files total)
│   │   ├── launch/
│   │   │   └── chassis_path_follower_launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── gik9dof_solver/          # GIK solver (dependency)
└── codegen/                      # MATLAB generated code
    └── chassis_path_follower_arm64/
```

---

## ✅ Success Criteria

After completing all steps, you should see:

1. **Build Success**
   - All 3 packages build without errors
   - Build completes in ~5-6 minutes
   - No critical warnings

2. **Node Starts**
   - Node launches without crashes
   - Initialization message appears
   - Controller mode is displayed

3. **Path Processing**
   - Node receives path messages
   - Status changes to "active"
   - Current index increments

4. **Velocity Output**
   - `/cmd_vel` publishes non-zero velocities
   - Linear and angular velocities within limits
   - Commands update at 100 Hz

5. **Goal Completion**
   - Node reaches path end
   - Status changes to "inactive"
   - Velocity commands go to zero

---

## 🎯 Quick Command Reference

```bash
# SSH to Orin
ssh cr@192.168.100.150

# Extract and build
cd ~ && unzip orin_chassis_follower_20251011_132300.zip
cd orin_chassis_follower_20251011_132300
chmod +x *.sh
./build_on_orin.sh

# Launch node
./test_on_orin.sh

# Or manual launch
cd ros2 && source install/setup.bash
ros2 launch gik9dof_controllers chassis_path_follower_launch.py

# Monitor topics (new terminal)
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /path_following_active

# Publish test path (new terminal)
ros2 topic pub /path nav_msgs/msg/Path '...' -1

# Check node status
ros2 node list
ros2 node info /chassis_path_follower
```

---

## 📞 Support

**Documentation**:
- Package README: `~/orin_chassis_follower_20251011_132300/README.md`
- WSL build log: Repository `WSL_BUILD_SUCCESS.md`

**Repository**:
- Name: `phoenixjyb/gikWBC9DOF`
- Branch: `codegencc45-main`

**Package Info**:
- Created: 2025-10-11 13:23:00
- Validated: WSL Ubuntu 22.04 + ROS2 Humble
- Target: NVIDIA AGX Orin ARM64

---

**Ready to deploy!** 🚀

Follow the steps above to build and test on the Orin hardware.
