# CODEGENCC45: 2-Day Fast Track Implementation Guide

## 🚀 Quick Overview
You have **2 days** to get the 9-DOF IK solver running on AGX Orin. This guide provides the streamlined path.

---

## 📋 DAY 1: MATLAB Code Generation + Initial ROS2 Setup

### Morning (4 hours): MATLAB Code Preparation

#### Step 1: Validate Robot Builder (30 min)
```matlab
cd matlab/+gik9dof/+codegen_realtime
validate_robot_builder
```

**Expected output:** All tests pass ✓

#### Step 2: Test IK Solver in MATLAB (30 min)
```matlab
% Quick smoke test
robot = gik9dof.codegen_realtime.buildRobotForCodegen();
q0 = zeros(9, 1);
targetPose = eye(4);
targetPose(1:3, 4) = [0.5; 0.2; 0.8];

[qNext, info] = gik9dof.codegen_realtime.solveGIKStepWrapper(...
    q0, targetPose, 0.1, 1.0);

% Should converge
assert(info.Status == 1, 'Solver failed');
fprintf('✓ IK solver works: %d iterations\n', info.Iterations);
```

#### Step 3: Generate C++ Code (1 hour)
```matlab
cd matlab/+gik9dof/+codegen_realtime
generateCodeARM64
```

**Critical files generated:**
- `codegen/arm64_realtime/solveGIKStepWrapper.h`
- `codegen/arm64_realtime/solveGIKStepWrapper.cpp`
- `codegen/arm64_realtime/GIKSolver.h` (C++ class wrapper)
- `codegen/arm64_realtime/*.a` (static library)

**Troubleshooting:**
- If code generation fails with "persistent variable" error:
  - Check that `solveGIKStepWrapper.m` uses `persistent robot solver`
  - Ensure `#codegen` directive is present
  - Verify no file I/O in `buildRobotForCodegen.m`

#### Step 4: Copy Generated Code to ROS2 (15 min)
```bash
# On Windows (PowerShell)
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

# Create target directories
New-Item -ItemType Directory -Force ros2/gik9dof_solver/matlab_codegen/include
New-Item -ItemType Directory -Force ros2/gik9dof_solver/matlab_codegen/lib

# Copy headers
Copy-Item codegen/arm64_realtime/*.h ros2/gik9dof_solver/matlab_codegen/include/
Copy-Item codegen/arm64_realtime/*.hpp ros2/gik9dof_solver/matlab_codegen/include/

# Copy libraries
Copy-Item codegen/arm64_realtime/*.a ros2/gik9dof_solver/matlab_codegen/lib/
```

### Afternoon (4 hours): ROS2 Integration

#### Step 5: Build Message Package (30 min)
```bash
# On AGX Orin (SSH)
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Verify messages
ros2 interface show gik9dof_msgs/msg/EndEffectorTrajectory
ros2 interface show gik9dof_msgs/msg/SolverDiagnostics
```

#### Step 6: Update Solver Node CMakeLists (15 min)
Edit `ros2/gik9dof_solver/CMakeLists.txt`:

Uncomment these lines:
```cmake
add_library(gik_solver_lib STATIC IMPORTED)
set_target_properties(gik_solver_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/matlab_codegen/lib/libsolveGIKStepWrapper.a
)

# In target_link_libraries:
target_link_libraries(gik9dof_solver_node
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  gik_solver_lib  # <-- Uncomment this
)
```

#### Step 7: Wire Up Generated Code in Node (2 hours)
Edit `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp`:

Replace the `solveIK()` placeholder:
```cpp
#include "gik9dof/GIKSolver.h"  // Add at top

bool solveIK(const geometry_msgs::msg::Pose& target_pose)
{
    // Convert to 4x4 matrix (column-major for MATLAB)
    double target_matrix[16];
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    T(0, 3) = target_pose.position.x;
    T(1, 3) = target_pose.position.y;
    T(2, 3) = target_pose.position.z;
    
    Eigen::Quaterniond q(target_pose.orientation.w,
                         target_pose.orientation.x,
                         target_pose.orientation.y,
                         target_pose.orientation.z);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    // Column-major flatten
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            target_matrix[col * 4 + row] = T(row, col);
        }
    }
    
    // Prepare inputs
    double q_current[9];
    double q_next[9];
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        std::copy(current_config_.begin(), current_config_.end(), q_current);
    }
    
    // Call MATLAB generated solver
    gik9dof::GIKSolver solver;
    double solve_info[4];  // [status, iterations, pose_error, time]
    
    solver.solveGIKStepWrapper(q_current, target_matrix, 
                              distance_lower_, distance_weight_,
                              q_next, solve_info);
    
    // Copy result
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        std::copy(q_next, q_next + 9, target_config_.begin());
    }
    
    return (solve_info[0] == 1.0);  // Status: 1 = success
}
```

#### Step 8: Build Solver Package (1 hour)
```bash
# On AGX Orin
cd ~/gikWBC9DOF/ros2
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check for errors
# Common issues:
#   - Missing Eigen3: sudo apt install libeigen3-dev
#   - Missing OpenMP: sudo apt install libomp-dev
#   - Linker errors: Check library path in CMakeLists.txt

source install/setup.bash
```

---

## 📋 DAY 2: Testing + Integration

### Morning (4 hours): Standalone Testing

#### Step 9: Create Test Launch File (15 min)
Create `ros2/gik9dof_solver/launch/test_solver.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gik9dof_solver',
            executable='gik9dof_solver_node',
            name='gik9dof_solver',
            output='screen',
            parameters=[{
                'control_rate': 10.0,
                'max_solve_time': 0.05,
                'distance_lower_bound': 0.1,
                'distance_weight': 1.0,
                'publish_diagnostics': True
            }]
        )
    ])
```

#### Step 10: Create Mock State Publishers (30 min)
Test script: `ros2/gik9dof_solver/scripts/test_mock_inputs.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gik9dof_msgs.msg import EndEffectorTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math

class MockInputPublisher(Node):
    def __init__(self):
        super().__init__('mock_input_publisher')
        
        self.joint_pub = self.create_publisher(JointState, '/hdas/feedback_arm_left', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)
        self.traj_pub = self.create_publisher(EndEffectorTrajectory, '/gik9dof/target_trajectory', 10)
        
        self.timer = self.create_timer(0.1, self.publish_mock_data)
        self.get_logger().info('Mock input publisher started')
    
    def publish_mock_data(self):
        # Mock joint state (6 arm joints)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
                   'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6']
        js.position = [0.0, 0.5, -0.3, 0.0, 0.2, 0.0]
        self.joint_pub.publish(js)
        
        # Mock odometry (base: x, y, theta)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 0.5
        odom.pose.pose.orientation.w = math.cos(0.1)
        odom.pose.pose.orientation.z = math.sin(0.1)
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = MockInputPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 11: Run Standalone Test (1 hour)
```bash
# Terminal 1: Launch solver
ros2 launch gik9dof_solver test_solver.launch.py

# Terminal 2: Mock inputs
python3 ros2/gik9dof_solver/scripts/test_mock_inputs.py

# Terminal 3: Monitor
ros2 topic echo /gik9dof/solver_diagnostics
ros2 topic echo /motion_target/target_joint_state_arm_left

# Terminal 4: Send test trajectory
ros2 topic pub /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory "{
  waypoints: [{pose: {position: {x: 0.5, y: 0.2, z: 0.8}, orientation: {w: 1.0}}}],
  timestamps: [0.0],
  sequence_id: 1
}" --once
```

**Expected Results:**
- ✓ Solver publishes diagnostics at 10 Hz
- ✓ `status: 1` (success) in most iterations
- ✓ `solve_time_ms < 50`
- ✓ Joint commands published on `/motion_target/target_joint_state_arm_left`

### Afternoon (4 hours): Real Robot Integration

#### Step 12: Integration with Real Robot (2 hours)
```bash
# Launch your existing perception + trajectory stack
ros2 launch <your_existing_launch_file>

# Launch GIK solver
ros2 launch gik9dof_solver test_solver.launch.py

# Verify topics are connected
ros2 topic list | grep -E "(feedback_arm_left|odom_wheel|target_trajectory)"

# Monitor performance
ros2 topic hz /gik9dof/solver_diagnostics  # Should be ~10 Hz
```

#### Step 13: Performance Tuning (1 hour)
Monitor and tune these parameters:
```bash
ros2 param set /gik9dof_solver control_rate 10.0  # Start conservative
ros2 param set /gik9dof_solver max_solve_time 0.05

# Watch solve times
ros2 topic echo /gik9dof/solver_diagnostics --field solve_time_ms

# If consistently < 20ms, can increase rate:
ros2 param set /gik9dof_solver control_rate 20.0
```

#### Step 14: Basic Motion Test (1 hour)
```bash
# Send a simple trajectory via your trajectory manager
# OR manually publish:
ros2 topic pub /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory "{
  header: {frame_id: 'base'},
  waypoints: [
    {pose: {position: {x: 0.6, y: 0.0, z: 0.9}, orientation: {w: 1.0}}},
    {pose: {position: {x: 0.6, y: 0.2, z: 0.9}, orientation: {w: 1.0}}},
    {pose: {position: {x: 0.6, y: 0.0, z: 0.9}, orientation: {w: 1.0}}}
  ],
  timestamps: [0.0, 2.0, 4.0],
  sequence_id: 1,
  max_velocity: 0.2,
  max_acceleration: 0.5
}" --once

# Watch robot execute
```

---

## 🔧 Troubleshooting Quick Reference

### Issue: Code generation fails
```matlab
% Check code generation report
open(fullfile('codegen/arm64_realtime/html/report.mldatx'))

% Common fixes:
% 1. Variable size arrays: Use coder.typeof with fixed sizes
% 2. Unsupported functions: Check MATLAB Coder compatibility
% 3. File I/O: Remove all file operations from buildRobotForCodegen
```

### Issue: ROS2 build fails
```bash
# Missing dependencies
sudo apt update
sudo apt install libeigen3-dev libomp-dev

# Clean build
cd ~/gikWBC9DOF/ros2
rm -rf build install log
colcon build --packages-select gik9dof_msgs gik9dof_solver
```

### Issue: Solver crashes at runtime
```bash
# Enable debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Run with GDB
gdb --args ros2 run gik9dof_solver gik9dof_solver_node

# Check array sizes in generated code
# Ensure 9 DOF config matches: [x, y, theta, arm1-6]
```

### Issue: Poor solve performance
```matlab
% In MATLAB, tune solver params in solveGIKStepWrapper.m:
solver.MaxIterations = 50;  % Reduce from 100
solver.MaxTime = 0.03;      % Tighter deadline
solver.GradientTolerance = 1e-6;  % Looser tolerance

% Regenerate code
generateCodeARM64
```

---

## ✅ Success Criteria (End of Day 2)

- [ ] MATLAB code generates without errors
- [ ] ROS2 packages build successfully on AGX Orin
- [ ] Solver node runs at 10 Hz
- [ ] Solve times < 50 ms (target: < 20 ms)
- [ ] Robot executes basic pick-and-place motion
- [ ] Diagnostics show convergence (status = 1) > 95%

---

## 📦 Deliverables

1. **Code artifacts:**
   - Generated C++ code in `codegen/arm64_realtime/`
   - ROS2 packages: `gik9dof_msgs`, `gik9dof_solver`
   
2. **Test results:**
   - Solve time statistics (mean, max, 95th percentile)
   - Trajectory following accuracy
   - Screenshot/video of robot motion

3. **Documentation:**
   - Issues encountered and resolutions
   - Performance measurements
   - Recommended next steps

---

## 🚀 Next Steps (Post-2-Day Sprint)

1. **Week 2-3:** Add unified chassis controller
2. **Week 3-4:** Obstacle avoidance integration
3. **Week 4-5:** Performance optimization (target 50 Hz)
4. **Week 5-6:** Production deployment

---

## 📞 Emergency Contacts / Resources

- **MATLAB Coder Issues:** Check `codegen/arm64_realtime/html/report.mldatx`
- **ROS2 Build Issues:** `colcon build --event-handlers console_direct+`
- **Performance Issues:** `ros2 topic hz`, `ros2 topic bw`, `top -H`

**Good luck! 🎯**
