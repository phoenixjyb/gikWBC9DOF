# Quick Start Guide - CodegenCC45

## TL;DR - What Is This Project?

Transform MATLAB mobile manipulator control into production C++ ROS2 nodes for NVIDIA AGX Orin.

**Input**: MATLAB algorithms + URDF robot model  
**Output**: Real-time ROS2 nodes controlling arm + chassis  
**Target**: Ubuntu 22.04 + ROS2 Humble on AGX Orin  

---

## Key Documents

| Document | Purpose | Read If... |
|----------|---------|------------|
| **CODEGENCC45_PROJECT_PLAN.md** | Architecture, system design, Q&A | You need the big picture |
| **MATLAB_CODEGEN_ANALYSIS.md** | What MATLAB code to generate | You're refactoring MATLAB |
| **ROS2_INTEGRATION_GUIDE.md** | ROS2 nodes, messages, launch files | You're writing ROS2 code |
| **IMPLEMENTATION_ROADMAP.md** | Week-by-week execution plan | You're managing the project |
| **README_CODEGENCC45.md** | This file - quick reference | You're getting started |

---

## 30-Second Overview

### What We Have (MATLAB)
- 9-DOF inverse kinematics solver
- Unified chassis controller  
- Trajectory following
- Collision avoidance
- Simulation & visualization

### What We Want (C++ ROS2)
- IK solver node → publishes arm commands
- Chassis controller node → publishes base velocities
- Trajectory manager → coordinates motion
- **NO** visualization (MATLAB only)
- **NO** static obstacles (use live perception)
- **NO** Hybrid A* planning (use ROS2 Nav2)

### How We Get There
1. Refactor MATLAB code (remove file I/O, persistent vars)
2. Generate C++ with MATLAB Coder
3. Wrap generated code in ROS2 nodes
4. Deploy to AGX Orin
5. Test & optimize

---

## Project Structure (After Completion)

```
gikWBC9DOF/
├── matlab/                           # MATLAB development (design/simulation)
│   └── +gik9dof/
│       └── +codegen/                # Code generation entry points
│           ├── buildRobotForCodegenARM64.m  # NEW
│           ├── solveGIKStepRealtime.m       # NEW
│           ├── unifiedChassisControlRealtime.m  # NEW
│           └── generateCodeARM64.m          # NEW
│
├── codegen/
│   └── linux_arm64/                 # Generated C++ code
│       ├── solveGIKStepRealtime/
│       ├── followTrajectory/
│       └── unifiedChassisControlRealtime/
│
└── ros2/                            # ROS2 deployment workspace
    ├── gik9dof_msgs/                # Custom ROS2 messages
    ├── gik9dof_solver/              # C++ solver library
    │   ├── include/                 # Headers
    │   │   └── gik9dof_solver/
    │   │       ├── generated/       # From MATLAB Coder
    │   │       └── solver_wrapper.hpp
    │   └── src/
    │       ├── generated/           # From MATLAB Coder
    │       └── solver_wrapper.cpp
    │
    └── gik9dof_controllers/         # ROS2 nodes
        ├── config/                  # YAML parameters
        ├── launch/                  # Launch files
        └── src/
            ├── gik9dof_solver_node.cpp
            ├── chassis_controller_node.cpp
            └── trajectory_manager_node.cpp
```

---

## ROS2 System Architecture

```
┌─────────────────────────────────────────────────┐
│  External Systems                                │
│  • SLAM → /odom_wheel                           │
│  • Perception → /obstacles                      │
│  • Planner → /task/trajectory_goals             │
└────────────┬────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────┐
│  trajectory_manager_node                        │
│  • Orchestrates arm + base                      │
│  • Manages control modes                        │
└─────┬──────────────────────────┬────────────────┘
      │                          │
      │ EE Trajectory            │ Chassis Cmd
      ▼                          ▼
┌──────────────────┐     ┌──────────────────────┐
│ gik9dof_solver   │     │ chassis_controller   │
│ _node            │     │ _node                │
│                  │     │                      │
│ • Solves IK      │     │ • Computes (Vx, Wz) │
└─────┬────────────┘     └──────┬───────────────┘
      │                         │
      │ JointState              │ Twist
      ▼                         ▼
┌──────────────────────────────────────────────────┐
│  Hardware Controllers                            │
│  • Arm controller (6 DOF)                       │
│  • Chassis controller (differential drive)       │
└──────────────────────────────────────────────────┘
```

---

## Message Flow

### Input Messages (Subscribe)

| Topic | Type | Source | Frequency |
|-------|------|--------|-----------|
| `/hdas/feedback_arm_left` | `JointState` | Arm hardware | ~100 Hz |
| `/odom_wheel` | `Odometry` | SLAM/Encoders | ~50 Hz |
| `/target/end_effector_trajectory` | Custom | Task planner | As needed |

### Output Messages (Publish)

| Topic | Type | Destination | Frequency |
|-------|------|-------------|-----------|
| `/motion_target/target_joint_state_arm_left` | `JointState` | Arm controller | 10 Hz |
| `/mobile_base/commands/velocity` | `Twist` | Chassis controller | 10 Hz |
| `/gik9dof/solver_diagnostics` | Custom | Monitoring | 10 Hz |

---

## Key Design Decisions

### ✅ What We're Doing

1. **Holistic control only** (initially) - Full 9-DOF IK
2. **Procedural robot model** - No file I/O at runtime
3. **Fixed-size arrays** - Max 50 waypoints, deterministic memory
4. **ROS2 Humble** - Long-term support, mature ecosystem
5. **C++17** - Modern, no external libs (except Eigen, OpenMP)

### ❌ What We're NOT Doing

1. **Staged control** - Too complex for first release
2. **Hybrid A* planning** - Use ROS2 Nav2 instead
3. **Static obstacles** - Use live perception
4. **Visualization** - Keep in MATLAB
5. **Python nodes** - Performance critical, C++ only

---

## Getting Started

### Prerequisites

**Windows Side** (MATLAB Development):
- MATLAB R2024b + Robotics System Toolbox + MATLAB Coder
- This repository cloned

**Ubuntu Side** (Deployment):
- Ubuntu 22.04 (native or WSL)
- ROS2 Humble installed
- Build tools: `build-essential`, `cmake`, `libeigen3-dev`, `libomp-dev`

### Step 1: Review Documentation (Day 1)

1. Read **CODEGENCC45_PROJECT_PLAN.md** - Understand architecture
2. Read **MATLAB_CODEGEN_ANALYSIS.md** - See what code changes needed
3. Read **ROS2_INTEGRATION_GUIDE.md** - Understand ROS2 structure
4. Answer the questions in PROJECT_PLAN (see "Additional Information Needed")

### Step 2: MATLAB Refactoring (Week 1)

1. Create `buildRobotForCodegenARM64.m`:
   ```matlab
   function robot = buildRobotForCodegenARM64()
   %#codegen
   % Build rigidBodyTree procedurally (no importrobot!)
   ```

2. Refactor `solveGIKStep.m` → `solveGIKStepRealtime.m`:
   ```matlab
   function qNext = solveGIKStepRealtime(robot, qCurrent, targetPose, ...)
   %#codegen
   % No persistent variables, robot passed as input
   ```

3. Test in MATLAB:
   ```matlab
   robot = gik9dof.codegen.buildRobotForCodegenARM64();
   q0 = zeros(9,1);
   target = trvec2tform([0.6 0.2 1.0]);
   qNext = gik9dof.codegen.solveGIKStepRealtime(robot, q0, target, 0.2, 0.5);
   ```

### Step 3: Code Generation (Week 1)

1. Configure MATLAB Coder:
   ```matlab
   cfg = coder.config('lib', 'ecoder', false);
   cfg.TargetLang = 'C++';
   cfg.EnableOpenMP = true;
   cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
   cfg.DynamicMemoryAllocation = 'Off';
   ```

2. Generate code:
   ```matlab
   gik9dof.codegen.generateCodeARM64('codegen/linux_arm64');
   ```

3. Check report: `codegen/linux_arm64/html/report.mldatx`

### Step 4: ROS2 Setup (Week 2-3)

1. Create message package:
   ```bash
   cd ros2
   ros2 pkg create --build-type ament_cmake gik9dof_msgs
   # Add message definitions
   ```

2. Create solver library:
   ```bash
   ros2 pkg create --build-type ament_cmake gik9dof_solver
   # Copy generated code, write wrappers
   ```

3. Build:
   ```bash
   colcon build --packages-select gik9dof_msgs gik9dof_solver
   ```

### Step 5: Node Implementation (Week 3-4)

See **ROS2_INTEGRATION_GUIDE.md** for full node code examples.

### Step 6: Testing (Week 4-5)

1. **Unit tests**: Test solver wrapper with GTest
2. **Integration tests**: Launch nodes, publish mock data
3. **Performance tests**: Measure latency, CPU usage

### Step 7: Deployment (Week 5-6)

1. Copy workspace to AGX Orin
2. Build on target
3. Test with real hardware
4. Optimize and validate

---

## Common Issues & Solutions

### Issue: MATLAB Coder errors

**Symptoms**: "Extrinsic function call", "Variable-size arrays"  
**Solution**: 
- Check for file I/O → remove
- Check for strings → use char arrays
- Check for unbounded arrays → add `coder.typeof` bounds

### Issue: Generated code doesn't compile

**Symptoms**: Missing headers, undefined references  
**Solution**:
- Verify OpenMP installed: `sudo apt install libomp-dev`
- Check Eigen3: `sudo apt install libeigen3-dev`
- Review MATLAB Coder report for dependencies

### Issue: ROS2 nodes crash

**Symptoms**: Segfault on startup  
**Solution**:
- Check robot model initialization
- Verify message types match
- Use `gdb` to get stack trace

### Issue: Performance too slow

**Symptoms**: Cannot maintain 10 Hz  
**Solution**:
- Profile with `perf`: `perf record -g ./node`
- Check OpenMP threads: Set `OMP_NUM_THREADS=4`
- Review solver iterations (reduce max iterations)

---

## Important Notes

### MATLAB Coder Constraints

❌ **Cannot use**:
- `importrobot()` - Use procedural builder
- Persistent variables - Pass state explicitly
- File I/O - Load at init only
- Variable-size arrays - Bound with `coder.typeof`
- Handle classes in MAT files - Build objects in code

✅ **Can use**:
- `rigidBodyTree` - If built procedurally
- Fixed-size arrays
- Structs (with care)
- Most math functions
- OpenMP parallelization

### Real-Time Considerations

- **Deterministic memory**: No dynamic allocation in control loop
- **Bounded execution time**: Max iterations, timeouts
- **Thread safety**: Mutexes if needed, avoid shared state
- **Priority**: Set ROS2 executor priority (real-time kernel recommended)

### Safety

⚠️ **Critical**:
- Always enforce joint limits
- Always enforce velocity limits
- Implement emergency stop
- Add watchdog timers
- Validate sensor data

---

## Performance Targets

| Metric | Target | Why |
|--------|--------|-----|
| Control rate | ≥10 Hz | Standard robot control rate |
| IK solve time | <10 ms | Leaves 90ms for ROS2 overhead |
| End-to-end latency | <100 ms | Acceptable for manipulation |
| Position error | <1 cm | Task requirements |
| Orientation error | <5 deg | Task requirements |

---

## Testing Checklist

Before deployment, verify:

- [ ] MATLAB functions pass unit tests
- [ ] Code generation succeeds without warnings
- [ ] Generated C++ compiles on Ubuntu 22.04
- [ ] Solver library passes unit tests
- [ ] ROS2 messages build correctly
- [ ] Nodes launch without errors
- [ ] Topic connections verified
- [ ] IK converges reliably
- [ ] Velocity limits enforced
- [ ] Performance meets targets
- [ ] 24-hour stress test passes

---

## Support & Contact

**Documentation**: See `docs/` folder  
**Issues**: Check IMPLEMENTATION_ROADMAP.md for known risks  
**Questions**: Review CODEGENCC45_PROJECT_PLAN.md Q&A section  

---

## Timeline Summary

| Phase | Duration | Milestone |
|-------|----------|-----------|
| 1. Preparation | Week 1-2 | MATLAB refactored, code generated |
| 2. Core Library | Week 3-4 | Solver library complete |
| 3. ROS2 Nodes | Week 4-5 | All nodes implemented |
| 4. Integration | Week 5-6 | System integrated and tested |
| 5. Deployment | Week 6-8 | Deployed to AGX Orin |

**Total**: 6-8 weeks

---

## Success Criteria

✅ **Technical**:
- Real-time performance (10 Hz)
- Accurate control (<1cm error)
- Stable operation (24-hour test)

✅ **Functional**:
- Follows trajectories
- Safety features work
- Integrates with SLAM/perception

✅ **Process**:
- On-time delivery
- Complete documentation
- Team trained

---

## Quick Commands Reference

### MATLAB

```matlab
% Generate code
addpath('matlab');
gik9dof.codegen.generateCodeARM64('codegen/linux_arm64');

% Test solver
robot = gik9dof.codegen.buildRobotForCodegenARM64();
q = zeros(9,1);
qNext = gik9dof.codegen.solveGIKStepRealtime(robot, q, eye(4), 0.2, 0.5);
```

### ROS2

```bash
# Build
cd ros2
colcon build --symlink-install

# Run
source install/setup.bash
ros2 launch gik9dof_controllers full_system.launch.py

# Monitor
ros2 topic echo /gik9dof/solver_diagnostics
ros2 topic hz /motion_target/target_joint_state_arm_left
```

---

**Last Updated**: 2025-10-06  
**Branch**: `codegencc45`  
**Status**: Documentation complete, ready for implementation
