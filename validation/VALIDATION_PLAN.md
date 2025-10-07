# MATLAB vs C++ ARM64 IK Validation Plan

**Purpose**: Validate that the ARM64-compiled C++ solver produces identical (or acceptably close) IK solutions compared to native MATLAB implementation.

**Date Created**: October 7, 2025  
**Status**: 📋 Planning Phase

---

## Overview

### Test Scenario
- **Trajectory**: `1_pull_world_scaled.json` (1,928 waypoints)
- **Test Subset**: First 20 waypoints (sufficient for validation)
- **Platforms**: 
  - MATLAB R2024b on Windows (x86_64) - **Reference**
  - C++ Solver on AGX Orin (ARM64) - **Under Test**

### Success Criteria
1. ✅ IK solutions converge for all 20 waypoints
2. ✅ Joint angle differences < 0.01 radians (0.57°)
3. ✅ End-effector pose error < 1mm position, < 0.1° orientation
4. ✅ Solve times comparable (within 2x)

---

## Test Workflow

```
┌─────────────────────────────────────────────────────┐
│ Phase 1: MATLAB Reference (Windows)                │
│   - Load 1_pull_world_scaled.json                  │
│   - Extract first 20 waypoints                      │
│   - Solve IK with MATLAB GIK                       │
│   - Export: matlab_reference_results.json           │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ Phase 2: Transfer Test Data (Windows → Orin)       │
│   - Copy 20-waypoint trajectory subset             │
│   - Transfer to Orin via scp                        │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ Phase 3: C++ Solver Test (Orin ARM64)              │
│   - Run gik9dof_solver_node                        │
│   - Publish 20 trajectory commands via ROS2        │
│   - Record joint solutions                          │
│   - Export: cpp_arm64_results.json                  │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ Phase 4: Comparison & Analysis (Windows/Orin)      │
│   - Compare joint angles per waypoint              │
│   - Compute forward kinematics for pose errors     │
│   - Analyze solve time performance                 │
│   - Generate validation report                      │
└─────────────────────────────────────────────────────┘
```

---

## Phase 1: MATLAB Reference Generation (Windows)

### Script: `generate_matlab_reference.m`

**Location**: `matlab/validation/`

**Functionality**:
```matlab
% Load trajectory
traj = jsondecode(fileread('1_pull_world_scaled.json'));

% Extract first 20 waypoints
testWaypoints = traj.poses(1:20);

% Build robot and solver
robot = importrobot('mobile_manipulator_PPR_base_corrected_sltRdcd.urdf');
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint'});

% Solve for each waypoint
results = struct();
results.waypoints = [];
for i = 1:20
    % Extract target pose
    pose = testWaypoints(i);
    target = trvec2tform(pose.position') * quat2tform(pose.orientation');
    
    % Solve IK
    tic;
    [qSol, solInfo] = gik(initialGuess, poseConstraint, jointConstraint);
    solveTime = toc;
    
    % Store results
    results.waypoints(i).index = i;
    results.waypoints(i).target_pose = target;
    results.waypoints(i).joint_config = qSol';
    results.waypoints(i).solve_time_ms = solveTime * 1000;
    results.waypoints(i).status = solInfo.Status;
    results.waypoints(i).iterations = solInfo.Iterations;
    
    initialGuess = qSol; % Use previous solution as next guess
end

% Export to JSON
jsonText = jsonencode(results);
fid = fopen('matlab_reference_results.json', 'w');
fprintf(fid, '%s', jsonText);
fclose(fid);
```

**Output**: `matlab_reference_results.json`

**Structure**:
```json
{
  "platform": "MATLAB R2024b",
  "architecture": "x86_64",
  "date": "2025-10-07",
  "waypoints": [
    {
      "index": 1,
      "target_pose": {
        "position": [1.65, 0.08, 0.86],
        "orientation": [0.5, -0.5, 0.5, -0.5]
      },
      "joint_config": [1.65, 0.08, 0.0, 0.1, 0.2, ...],
      "solve_time_ms": 12.5,
      "status": "success",
      "iterations": 8
    },
    ...
  ]
}
```

---

## Phase 2: Transfer Test Data

### Files to Transfer to Orin

```bash
# From Windows PowerShell
$ORIN_IP = "192.168.100.150"
$ORIN_USER = "cr"

# Create validation directory on Orin
ssh ${ORIN_USER}@${ORIN_IP} "mkdir -p ~/gikWBC9DOF/validation"

# Transfer test files
scp matlab_reference_results.json ${ORIN_USER}@${ORIN_IP}:~/gikWBC9DOF/validation/
scp mobile_manipulator_PPR_base_corrected_sltRdcd.urdf ${ORIN_USER}@${ORIN_IP}:~/gikWBC9DOF/validation/
scp validation/run_cpp_test.py ${ORIN_USER}@${ORIN_IP}:~/gikWBC9DOF/validation/
```

---

## Phase 3: C++ ARM64 Test Execution

### Test Script: `run_cpp_test.py`

**Location**: `validation/` (runs on Orin)

**Functionality**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gik9dof_msgs.msg import EndEffectorTrajectory, EndEffectorPoint
from sensor_msgs.msg import JointState
import json
import time

class IKValidationNode(Node):
    def __init__(self):
        super().__init__('ik_validation_node')
        
        # Publishers
        self.traj_pub = self.create_publisher(
            EndEffectorTrajectory, 
            '/gik9dof/target_trajectory', 
            10
        )
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/motion_target/target_joint_state_arm_left',
            self.joint_callback,
            10
        )
        
        # Load reference data
        with open('matlab_reference_results.json', 'r') as f:
            self.reference = json.load(f)
        
        self.results = []
        self.current_waypoint = 0
        
    def joint_callback(self, msg):
        """Record joint solution from solver"""
        if self.current_waypoint < len(self.reference['waypoints']):
            waypoint = {
                'index': self.current_waypoint + 1,
                'joint_config': list(msg.position),
                'timestamp': time.time()
            }
            self.results.append(waypoint)
            self.current_waypoint += 1
            
    def run_test(self):
        """Send trajectory waypoints and record results"""
        for i, ref_wp in enumerate(self.reference['waypoints']):
            # Create trajectory message
            traj_msg = EndEffectorTrajectory()
            point = EndEffectorPoint()
            
            pose = ref_wp['target_pose']
            point.pose.position.x = pose['position'][0]
            point.pose.position.y = pose['position'][1]
            point.pose.position.z = pose['position'][2]
            point.pose.orientation.x = pose['orientation'][0]
            point.pose.orientation.y = pose['orientation'][1]
            point.pose.orientation.z = pose['orientation'][2]
            point.pose.orientation.w = pose['orientation'][3]
            
            traj_msg.points = [point]
            
            # Publish and wait for solution
            self.traj_pub.publish(traj_msg)
            time.sleep(0.1)  # Wait for solver
        
        # Export results
        output = {
            'platform': 'C++ ARM64',
            'architecture': 'aarch64',
            'date': time.strftime('%Y-%m-%d'),
            'waypoints': self.results
        }
        
        with open('cpp_arm64_results.json', 'w') as f:
            json.dump(output, f, indent=2)

def main():
    rclpy.init()
    node = IKValidationNode()
    node.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run on Orin**:
```bash
cd ~/gikWBC9DOF/validation
source ../ros2/install/setup.bash

# Start solver node in background
ros2 run gik9dof_solver gik9dof_solver_node &

# Run validation test
python3 run_cpp_test.py

# Results saved to: cpp_arm64_results.json
```

---

## Phase 4: Comparison & Analysis

### Comparison Script: `compare_results.m`

**Location**: `matlab/validation/`

**Functionality**:
```matlab
% Load both results
matlabResults = jsondecode(fileread('matlab_reference_results.json'));
cppResults = jsondecode(fileread('cpp_arm64_results.json'));

% Compare each waypoint
comparison = struct();
comparison.waypoints = [];

for i = 1:20
    mRef = matlabResults.waypoints(i);
    cCpp = cppResults.waypoints(i);
    
    % Joint angle differences
    qDiff = mRef.joint_config - cCpp.joint_config;
    
    % Compute FK to get pose error
    robot = importrobot('mobile_manipulator_PPR_base_corrected_sltRdcd.urdf');
    T_matlab = getTransform(robot, mRef.joint_config', 'end_effector');
    T_cpp = getTransform(robot, cCpp.joint_config', 'end_effector');
    
    posError = norm(T_matlab(1:3,4) - T_cpp(1:3,4)) * 1000; % mm
    rotError = rad2deg(norm(rotm2axang(T_matlab(1:3,1:3) \ T_cpp(1:3,1:3)) * [0 0 0 1]'));
    
    % Store comparison
    wp.index = i;
    wp.joint_diff_rad = qDiff;
    wp.joint_diff_deg = rad2deg(qDiff);
    wp.max_joint_diff_rad = max(abs(qDiff));
    wp.rms_joint_diff_rad = rms(qDiff);
    wp.position_error_mm = posError;
    wp.orientation_error_deg = rotError;
    wp.matlab_time_ms = mRef.solve_time_ms;
    wp.cpp_time_ms = cCpp.solve_time_ms;
    wp.time_ratio = cCpp.solve_time_ms / mRef.solve_time_ms;
    
    comparison.waypoints(i) = wp;
end

% Summary statistics
comparison.summary.max_joint_error_rad = max([comparison.waypoints.max_joint_diff_rad]);
comparison.summary.mean_joint_error_rad = mean([comparison.waypoints.max_joint_diff_rad]);
comparison.summary.max_position_error_mm = max([comparison.waypoints.position_error_mm]);
comparison.summary.mean_position_error_mm = mean([comparison.waypoints.position_error_mm]);
comparison.summary.max_orientation_error_deg = max([comparison.waypoints.orientation_error_deg]);
comparison.summary.matlab_avg_time_ms = mean([matlabResults.waypoints.solve_time_ms]);
comparison.summary.cpp_avg_time_ms = mean([cppResults.waypoints.solve_time_ms]);
comparison.summary.time_ratio = comparison.summary.cpp_avg_time_ms / comparison.summary.matlab_avg_time_ms;

% Generate report
fprintf('=== MATLAB vs C++ ARM64 Validation Report ===\n\n');
fprintf('Joint Angle Errors:\n');
fprintf('  Max error: %.4f rad (%.2f deg)\n', ...
    comparison.summary.max_joint_error_rad, ...
    rad2deg(comparison.summary.max_joint_error_rad));
fprintf('  Mean error: %.4f rad (%.2f deg)\n', ...
    comparison.summary.mean_joint_error_rad, ...
    rad2deg(comparison.summary.mean_joint_error_rad));

fprintf('\nEnd-Effector Pose Errors:\n');
fprintf('  Max position error: %.3f mm\n', comparison.summary.max_position_error_mm);
fprintf('  Mean position error: %.3f mm\n', comparison.summary.mean_position_error_mm);
fprintf('  Max orientation error: %.3f deg\n', comparison.summary.max_orientation_error_deg);

fprintf('\nPerformance:\n');
fprintf('  MATLAB avg: %.2f ms\n', comparison.summary.matlab_avg_time_ms);
fprintf('  C++ ARM64 avg: %.2f ms\n', comparison.summary.cpp_avg_time_ms);
fprintf('  Ratio (C++/MATLAB): %.2fx\n', comparison.summary.time_ratio);

% Pass/Fail
if comparison.summary.max_joint_error_rad < 0.01 && ...
   comparison.summary.max_position_error_mm < 1.0
    fprintf('\n✅ VALIDATION PASSED\n');
else
    fprintf('\n❌ VALIDATION FAILED\n');
end

% Export report
jsonText = jsonencode(comparison);
fid = fopen('validation_comparison.json', 'w');
fprintf(fid, '%s', jsonText);
fclose(fid);
```

---

## File Structure

```
gikWBC9DOF/
├── matlab/
│   └── validation/
│       ├── generate_matlab_reference.m      # NEW: Phase 1
│       ├── compare_results.m                # NEW: Phase 4
│       └── matlab_reference_results.json    # Generated
│
├── validation/                              # NEW: Validation directory
│   ├── run_cpp_test.py                      # NEW: Phase 3 (runs on Orin)
│   ├── cpp_arm64_results.json               # Generated on Orin
│   ├── validation_comparison.json           # Generated by compare_results.m
│   └── VALIDATION_PLAN.md                   # This document
│
└── 1_pull_world_scaled.json                 # Existing trajectory
```

---

## Expected Timeline

| Phase | Duration | Platform | Blocker? |
|-------|----------|----------|----------|
| 1. MATLAB Reference | 10 min | Windows | No - can run now |
| 2. Transfer Files | 2 min | Windows → Orin | No - scp ready |
| 3. C++ Test | 15 min | Orin | ⚠️ Solver must be working |
| 4. Comparison | 5 min | Windows | No - MATLAB analysis |
| **Total** | **~30 min** | | |

---

## Next Steps

### Immediate Actions

1. **Create MATLAB reference script** (`generate_matlab_reference.m`)
2. **Create Python test script** (`run_cpp_test.py`)
3. **Create comparison script** (`compare_results.m`)
4. **Run Phase 1** on Windows to generate reference data

### Before C++ Test (Phase 3)

⚠️ **Prerequisites**:
- Solver node must be running on Orin
- Need to verify solver accepts trajectory messages
- May need to create ROS2 message publisher script

### Fallback Plan

If ROS2 integration is complex, alternative approach:
1. Create standalone C++ test program that calls solver directly
2. Load waypoints from JSON
3. Call IK solver function in loop
4. Export results to JSON
5. Compare offline

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Solver not computing valid IK | Medium | High | First run `test_solver_arm64.sh` |
| ROS2 message structure mismatch | Low | Medium | Check message definitions first |
| File transfer issues | Low | Low | Test scp connection |
| Numerical differences > threshold | Medium | Medium | Adjust tolerance if reasonable |
| ARM64 performance too slow | Low | Medium | Document, plan optimization |

---

## Success Metrics

### Validation Passes If:
- ✅ Max joint error < 0.01 rad (0.57°)
- ✅ Max position error < 1 mm
- ✅ Max orientation error < 0.1°
- ✅ All 20 waypoints converge
- ✅ ARM64 solve time < 100 ms (2x MATLAB)

### Investigation Needed If:
- ⚠️ Joint errors 0.01-0.1 rad
- ⚠️ Position errors 1-5 mm
- ⚠️ Some waypoints fail to converge
- ⚠️ ARM64 solve time > 100 ms

### Validation Fails If:
- ❌ Joint errors > 0.1 rad (5.7°)
- ❌ Position errors > 5 mm
- ❌ Many waypoints fail
- ❌ ARM64 solve time > 200 ms

---

## Deliverables

1. ✅ `matlab/validation/generate_matlab_reference.m`
2. ✅ `validation/run_cpp_test.py`
3. ✅ `matlab/validation/compare_results.m`
4. ✅ `validation/VALIDATION_PLAN.md` (this document)
5. 📊 `matlab_reference_results.json` (after Phase 1)
6. 📊 `cpp_arm64_results.json` (after Phase 3)
7. 📊 `validation_comparison.json` (after Phase 4)
8. 📄 Validation report (summary in markdown)

---

**Status**: 📋 Plan complete, ready for implementation  
**Next Action**: Create validation scripts and run Phase 1

---

**Created**: October 7, 2025  
**Author**: GitHub Copilot + User  
**Branch**: codegencc45
