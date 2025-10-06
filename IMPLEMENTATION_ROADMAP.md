# Implementation Roadmap - CodegenCC45

## Overview

This roadmap outlines the step-by-step implementation plan to transform the MATLAB-based mobile manipulator control system into production-ready C++ ROS2 nodes for deployment on NVIDIA AGX Orin.

**Estimated Timeline**: 6-8 weeks  
**Team Size**: 2-3 engineers  
**Risk Level**: Medium

---

## Phase 1: Foundation & Preparation (Week 1-2)

### Milestone 1.1: MATLAB Code Refactoring

**Goal**: Make core MATLAB functions compatible with MATLAB Coder constraints

#### Tasks:

1. **Create Procedural Robot Builder** 
   - [ ] Extract URDF parameters into MATLAB struct
   - [ ] Write `buildRobotForCodegenARM64.m` that builds `rigidBodyTree` procedurally
   - [ ] Remove dependency on `importrobot()` file I/O
   - [ ] Hard-code collision mesh attachment
   - [ ] Test in MATLAB: verify `showdetails(robot)` matches original

   **Deliverable**: `matlab/+gik9dof/+codegen/buildRobotForCodegenARM64.m`  
   **Validation**: Robot model matches URDF, all joints present, meshes attached

2. **Refactor IK Solver for Codegen**
   - [ ] Remove persistent variables from `solveGIKStep.m`
   - [ ] Create `solveGIKStepRealtime.m` with robot as input parameter
   - [ ] Add explicit initialization function
   - [ ] Convert to fixed-size arrays (9x1 for joints, 4x4 for poses)
   - [ ] Test: compare output with original `solveGIKStep.m`

   **Deliverable**: `matlab/+gik9dof/+codegen/solveGIKStepRealtime.m`  
   **Validation**: Identical output to original within 1e-6 tolerance

3. **Refactor Chassis Controller**
   - [ ] Remove persistent state from `unifiedChassisCtrl.m`
   - [ ] Create `unifiedChassisControlRealtime.m` with explicit state struct
   - [ ] Separate parameters into data-only struct
   - [ ] Test with holistic mode only (ignore staged modes)

   **Deliverable**: `matlab/+gik9dof/+codegen/unifiedChassisControlRealtime.m`  
   **Validation**: Outputs match original for holistic trajectory

4. **Create Configuration Structure**
   - [ ] Define MATLAB struct for all robot parameters
   - [ ] Create YAML exporter to generate ROS2 config files
   - [ ] Document all parameters with units and ranges

   **Deliverable**: `matlab/+gik9dof/+codegen/exportConfigToYAML.m`  
   **Validation**: YAML loads correctly in Python

**Success Criteria**:
- ✅ All refactored functions pass unit tests
- ✅ No file I/O in critical path
- ✅ No persistent variables in entry points
- ✅ Fixed-size arrays throughout

**Risk**: Medium - MATLAB Coder constraints may require significant refactoring  
**Mitigation**: Start with smallest function (`clampYawByWheelLimit`), iterate incrementally

---

### Milestone 1.2: Code Generation Setup

**Goal**: Successfully generate C++ code from MATLAB on Windows

#### Tasks:

1. **Configure MATLAB Coder**
   - [ ] Create code generation script: `generateCodeARM64.m`
   - [ ] Configure for Linux ARM64 target
   - [ ] Enable C++17, OpenMP
   - [ ] Set up bounded array types
   - [ ] Disable dynamic memory allocation

   **Deliverable**: `matlab/+gik9dof/+codegen/generateCodeARM64.m`  
   **Config**: See MATLAB_CODEGEN_ANALYSIS.md for details

2. **Generate Initial Code**
   - [ ] Run codegen on `solveGIKStepRealtime.m`
   - [ ] Review MATLAB Coder report
   - [ ] Fix any errors or warnings
   - [ ] Verify generated code structure

   **Deliverable**: `codegen/linux_arm64/solveGIKStepRealtime/`  
   **Validation**: No errors in MATLAB Coder report

3. **Cross-Compilation Test**
   - [ ] Copy generated code to Ubuntu 22.04 (WSL or native)
   - [ ] Write minimal CMake build file
   - [ ] Compile with GCC/Clang
   - [ ] Link against OpenMP
   - [ ] Run basic smoke test

   **Deliverable**: Compiled `.so` library on Ubuntu  
   **Validation**: Library loads without errors

**Success Criteria**:
- ✅ MATLAB Coder runs without errors
- ✅ Generated C++ code compiles on Ubuntu 22.04
- ✅ No unresolved symbols
- ✅ Basic functionality test passes

**Risk**: High - Cross-platform compatibility issues  
**Mitigation**: Test on actual Ubuntu 22.04 early, not just WSL

---

## Phase 2: Core Library Development (Week 3-4)

### Milestone 2.1: Solver Library

**Goal**: Complete and tested C++ solver library with ROS-agnostic interface

#### Tasks:

1. **Create C++ Wrapper Class**
   - [ ] Implement `SolverWrapper` class (see ROS2_INTEGRATION_GUIDE.md)
   - [ ] Manage robot model lifecycle
   - [ ] Wrap MATLAB-generated IK functions
   - [ ] Add error handling and validation
   - [ ] Implement thread safety (mutexes if needed)

   **Deliverable**: `ros2/gik9dof_solver/src/solver_wrapper.cpp`  
   **Interface**: See `solver_wrapper.hpp` in ROS2 guide

2. **Implement Chassis Controller Wrapper**
   - [ ] Wrap `unifiedChassisControlRealtime` generated code
   - [ ] Manage controller state
   - [ ] Add velocity clamping
   - [ ] Implement safety checks

   **Deliverable**: `ros2/gik9dof_solver/src/chassis_controller.cpp`

3. **Setup CMake Build System**
   - [ ] Write `CMakeLists.txt` for `gik9dof_solver`
   - [ ] Link OpenMP, Eigen3
   - [ ] Export ament target
   - [ ] Setup include directories
   - [ ] Handle generated code dependencies

   **Deliverable**: `ros2/gik9dof_solver/CMakeLists.txt`  
   **Validation**: `colcon build` succeeds

4. **Unit Testing**
   - [ ] Write GTest suite for `SolverWrapper`
   - [ ] Test robot initialization
   - [ ] Test IK convergence
   - [ ] Test trajectory following
   - [ ] Test chassis controller
   - [ ] Benchmark performance

   **Deliverable**: `ros2/gik9dof_solver/test/test_solver_wrapper.cpp`  
   **Validation**: All tests pass, <10ms per IK step

**Success Criteria**:
- ✅ Library compiles without warnings
- ✅ All unit tests pass
- ✅ IK solver converges reliably
- ✅ Performance meets real-time requirements (<10ms/step)
- ✅ Memory leaks: none (valgrind clean)

**Risk**: Medium - Generated code may have unexpected dependencies  
**Mitigation**: Extensive testing with mock data before ROS integration

---

### Milestone 2.2: ROS2 Message Definitions

**Goal**: Custom ROS2 messages ready for use

#### Tasks:

1. **Create Message Package**
   - [ ] Setup `gik9dof_msgs` package
   - [ ] Define all 4 custom messages (see ROS2 guide)
   - [ ] Write CMakeLists.txt with rosidl generation
   - [ ] Build and verify message generation

   **Deliverable**: `ros2/gik9dof_msgs/`  
   **Validation**: `ros2 interface show gik9dof_msgs/msg/...` works

2. **Message Validation**
   - [ ] Verify message sizes are reasonable
   - [ ] Test serialization/deserialization
   - [ ] Confirm compatibility with C++ and Python

   **Validation**: Can publish/subscribe from command line

**Success Criteria**:
- ✅ All messages compile
- ✅ Python and C++ can use messages
- ✅ Message definitions match specification

**Risk**: Low  
**Mitigation**: N/A

---

## Phase 3: ROS2 Node Implementation (Week 4-5)

### Milestone 3.1: Solver Node

**Goal**: Working `gik9dof_solver_node` that computes IK solutions

#### Tasks:

1. **Implement Node Class**
   - [ ] Create `gik9dof_solver_node.cpp`
   - [ ] Setup subscriptions (trajectory, joint states)
   - [ ] Setup publications (arm commands, diagnostics)
   - [ ] Integrate `SolverWrapper`
   - [ ] Implement control loop callback

   **Deliverable**: `ros2/gik9dof_controllers/src/gik9dof_solver_node.cpp`

2. **Parameter Management**
   - [ ] Load solver config from YAML
   - [ ] Implement parameter validation
   - [ ] Support dynamic reconfigure (optional)

   **Deliverable**: `ros2/gik9dof_controllers/config/solver_config.yaml`

3. **Diagnostics**
   - [ ] Publish solver status
   - [ ] Monitor convergence rate
   - [ ] Track timing statistics
   - [ ] Publish to `/diagnostics` topic

4. **Testing**
   - [ ] Unit test node callbacks
   - [ ] Integration test with mock publishers
   - [ ] Verify message flow
   - [ ] Test error handling

   **Validation**: Node runs at 10 Hz, publishes valid commands

**Success Criteria**:
- ✅ Node launches without errors
- ✅ Subscribes to correct topics
- ✅ Publishes valid JointState messages
- ✅ Diagnostics show healthy status
- ✅ Real-time performance maintained (10 Hz)

**Risk**: Medium - ROS2 integration complexity  
**Mitigation**: Start with minimal node, add features incrementally

---

### Milestone 3.2: Chassis Controller Node

**Goal**: Working `chassis_controller_node`

#### Tasks:

1. **Implement Node**
   - [ ] Create `chassis_controller_node.cpp`
   - [ ] Subscribe to odometry and chassis commands
   - [ ] Publish velocity commands
   - [ ] Integrate chassis controller wrapper

   **Deliverable**: `ros2/gik9dof_controllers/src/chassis_controller_node.cpp`

2. **Safety Features**
   - [ ] Implement velocity limits
   - [ ] Add watchdog timer
   - [ ] Emergency stop handling
   - [ ] Odometry timeout detection

3. **Testing**
   - [ ] Test with mock odometry
   - [ ] Verify velocity clamping
   - [ ] Test safety features

**Success Criteria**:
- ✅ Node publishes valid Twist messages
- ✅ Velocity limits enforced
- ✅ Safety features work correctly

**Risk**: Low  
**Mitigation**: N/A

---

### Milestone 3.3: Trajectory Manager Node

**Goal**: Orchestration node for coordinated control

#### Tasks:

1. **Implement Node**
   - [ ] Create `trajectory_manager_node.cpp`
   - [ ] Coordinate arm and base
   - [ ] Manage control modes
   - [ ] Handle goal poses

2. **Control Logic**
   - [ ] Implement holistic mode
   - [ ] Add mode switching
   - [ ] Trajectory generation (simple)
   - [ ] Error recovery

3. **Testing**
   - [ ] Test mode transitions
   - [ ] Verify coordination
   - [ ] Test with full system

**Success Criteria**:
- ✅ Successful holistic control
- ✅ Smooth mode transitions
- ✅ Coordinated arm+base motion

**Risk**: High - Complex coordination logic  
**Mitigation**: Start with holistic only, add staged modes later

---

## Phase 4: Integration & Testing (Week 5-6)

### Milestone 4.1: System Integration

**Goal**: All nodes running together

#### Tasks:

1. **Launch File Creation**
   - [ ] Write `full_system.launch.py`
   - [ ] Configure all nodes
   - [ ] Setup parameter passing
   - [ ] Add logging configuration

   **Deliverable**: `ros2/gik9dof_controllers/launch/full_system.launch.py`

2. **Configuration Management**
   - [ ] Finalize all YAML configs
   - [ ] Document all parameters
   - [ ] Create parameter validation

3. **Integration Testing**
   - [ ] Launch full system
   - [ ] Verify topic connections
   - [ ] Test message flow end-to-end
   - [ ] Monitor resource usage

   **Validation**: `ros2 node list`, `ros2 topic list` show all expected nodes/topics

**Success Criteria**:
- ✅ All nodes launch successfully
- ✅ No error messages in logs
- ✅ Topics connected correctly
- ✅ Parameters load from YAML
- ✅ CPU usage reasonable (<50% on AGX Orin)

**Risk**: Medium - Integration issues  
**Mitigation**: Incremental integration, test subsystems first

---

### Milestone 4.2: Simulation Testing

**Goal**: Validate with simulated hardware

#### Tasks:

1. **Create Mock Hardware**
   - [ ] Write mock joint state publisher
   - [ ] Write mock odometry publisher
   - [ ] Simulate sensor delays

2. **Test Scenarios**
   - [ ] Straight-line trajectory
   - [ ] Circular trajectory
   - [ ] Obstacle avoidance (if implemented)
   - [ ] Rapid direction changes
   - [ ] Recovery from errors

3. **Data Collection**
   - [ ] Log all messages
   - [ ] Record rosbags
   - [ ] Analyze performance metrics
   - [ ] Compare with MATLAB simulation

   **Validation**: MATLAB vs C++ error <1cm position, <5deg orientation

**Success Criteria**:
- ✅ Robot follows trajectories accurately
- ✅ Performance matches MATLAB baseline
- ✅ No crashes or errors in stress tests
- ✅ Graceful degradation on sensor loss

**Risk**: Low  
**Mitigation**: N/A

---

## Phase 5: Deployment & Optimization (Week 6-8)

### Milestone 5.1: Hardware Deployment

**Goal**: Deploy to NVIDIA AGX Orin

#### Tasks:

1. **Environment Setup**
   - [ ] Install Ubuntu 22.04 on AGX Orin
   - [ ] Install ROS2 Humble
   - [ ] Setup cross-compilation (if needed)
   - [ ] Install all dependencies

2. **Deploy Code**
   - [ ] Copy/build ROS2 workspace on AGX
   - [ ] Configure launch files for hardware
   - [ ] Setup systemd service (auto-start)
   - [ ] Configure logging

3. **Hardware Interface**
   - [ ] Connect to real arm controller
   - [ ] Connect to chassis controller
   - [ ] Verify sensor connections
   - [ ] Test emergency stop

   **Validation**: All hardware interfaces working

**Success Criteria**:
- ✅ ROS2 nodes run on AGX Orin
- ✅ Real-time performance maintained
- ✅ Hardware responds to commands
- ✅ Sensors publish data

**Risk**: High - Hardware compatibility issues  
**Mitigation**: Test with minimal functionality first, incrementally add features

---

### Milestone 5.2: Performance Optimization

**Goal**: Optimize for real-time performance

#### Tasks:

1. **Profiling**
   - [ ] Profile solver performance
   - [ ] Identify bottlenecks
   - [ ] Analyze cache misses
   - [ ] Check OpenMP scaling

2. **Optimization**
   - [ ] Tune solver parameters
   - [ ] Optimize memory access patterns
   - [ ] Enable NEON intrinsics (if applicable)
   - [ ] Adjust thread priorities

3. **Benchmarking**
   - [ ] Measure control loop jitter
   - [ ] Measure end-to-end latency
   - [ ] Test under load
   - [ ] Compare before/after optimization

   **Target**: <10ms IK solve, <100ms end-to-end latency

**Success Criteria**:
- ✅ Meets real-time deadlines 99.9% of time
- ✅ Latency <100ms
- ✅ CPU usage <60% average
- ✅ No priority inversions

**Risk**: Medium  
**Mitigation**: Profile early, optimize incrementally

---

### Milestone 5.3: Validation & Testing

**Goal**: Comprehensive hardware testing

#### Tasks:

1. **Functional Tests**
   - [ ] Pick-and-place task
   - [ ] Navigation with obstacles
   - [ ] Coordinated arm+base motion
   - [ ] Recovery scenarios

2. **Stress Tests**
   - [ ] 24-hour continuous operation
   - [ ] Rapid trajectory changes
   - [ ] Worst-case scenarios
   - [ ] Sensor fault injection

3. **Safety Tests**
   - [ ] Emergency stop response
   - [ ] Joint limit enforcement
   - [ ] Velocity limit enforcement
   - [ ] Watchdog timeout handling

4. **Documentation**
   - [ ] Create user manual
   - [ ] Document APIs
   - [ ] Write troubleshooting guide
   - [ ] Record demo videos

**Success Criteria**:
- ✅ All functional tests pass
- ✅ No crashes in 24-hour test
- ✅ Safety features verified
- ✅ Documentation complete

**Risk**: Medium - Real-world edge cases  
**Mitigation**: Extensive testing with diverse scenarios

---

## Phase 6: Handoff & Maintenance (Week 8+)

### Milestone 6.1: Knowledge Transfer

**Goal**: Team is trained and documentation complete

#### Tasks:

1. **Training**
   - [ ] Train operators on system
   - [ ] Train maintainers on debugging
   - [ ] Conduct code walkthrough
   - [ ] Provide troubleshooting workshop

2. **Documentation**
   - [ ] Finalize all technical docs
   - [ ] Create maintenance procedures
   - [ ] Document known issues
   - [ ] Provide contact information

**Success Criteria**:
- ✅ Team can operate system independently
- ✅ Team can debug common issues
- ✅ Documentation complete and clear

**Risk**: Low  
**Mitigation**: N/A

---

### Milestone 6.2: Production Readiness

**Goal**: System ready for production use

#### Tasks:

1. **Final Review**
   - [ ] Code review
   - [ ] Security audit
   - [ ] Performance validation
   - [ ] Safety certification (if required)

2. **Release**
   - [ ] Tag release version
   - [ ] Create deployment package
   - [ ] Setup CI/CD (optional)
   - [ ] Publish documentation

3. **Monitoring**
   - [ ] Setup logging infrastructure
   - [ ] Configure alerts
   - [ ] Create dashboard
   - [ ] Establish support process

**Success Criteria**:
- ✅ Passes all acceptance criteria
- ✅ Approved for production
- ✅ Monitoring in place
- ✅ Support process established

**Risk**: Low  
**Mitigation**: N/A

---

## Resource Requirements

### Personnel

| Role | Responsibilities | Time Commitment |
|------|------------------|-----------------|
| MATLAB Engineer | Code refactoring, codegen | Full-time (Week 1-3) |
| C++ Engineer | Wrapper development, optimization | Full-time (Week 2-8) |
| ROS2 Engineer | Node implementation, integration | Full-time (Week 3-8) |
| Test Engineer | Testing, validation | Part-time (Week 4-8) |

### Hardware

- [ ] Windows workstation with MATLAB R2024b
- [ ] Ubuntu 22.04 development machine
- [ ] NVIDIA AGX Orin development kit
- [ ] Mobile manipulator hardware (for testing)
- [ ] Network infrastructure

### Software

- [ ] MATLAB R2024b + Robotics System Toolbox
- [ ] MATLAB Coder
- [ ] ROS2 Humble
- [ ] Development tools (VSCode, GCC, gdb, etc.)

---

## Risk Management

### High-Priority Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| MATLAB Coder incompatibility | High | Medium | Incremental testing, fallback to simulation |
| Real-time performance not met | High | Medium | Early profiling, optimization buffer |
| Hardware integration issues | High | Low | Test with mock hardware first |
| Robot model mismatch | Medium | Low | Validate against MATLAB early |

### Contingency Plans

1. **If codegen fails**: Use ROS2 with MATLAB Engine (slower, but functional)
2. **If performance insufficient**: Reduce control rate (5 Hz), optimize later
3. **If hardware unavailable**: Continue development with simulation
4. **If AGX Orin issues**: Test on x86_64 Ubuntu first

---

## Success Metrics

### Technical Metrics

| Metric | Target | Measured |
|--------|--------|----------|
| Control loop frequency | ≥10 Hz | TBD |
| IK solve time | <10 ms | TBD |
| End-to-end latency | <100 ms | TBD |
| Position accuracy | <1 cm | TBD |
| Orientation accuracy | <5 deg | TBD |
| CPU usage | <60% avg | TBD |
| Memory usage | <2 GB | TBD |
| Uptime (24-hour test) | >99% | TBD |

### Functional Metrics

- [ ] Successfully follows trajectories
- [ ] Holistic control working
- [ ] Safety features verified
- [ ] Integration with SLAM/perception
- [ ] Recovery from errors
- [ ] All ROS2 messages correct

### Process Metrics

- [ ] On-time delivery (±1 week)
- [ ] Within budget
- [ ] Documentation complete
- [ ] Team trained
- [ ] No critical bugs

---

## Next Steps (Immediate Actions)

1. **This Week**:
   - [ ] Review and approve this roadmap
   - [ ] Answer questions in CODEGENCC45_PROJECT_PLAN.md
   - [ ] Setup development environment
   - [ ] Clone repository to Ubuntu machine

2. **Week 1, Day 1**:
   - [ ] Kickoff meeting
   - [ ] Assign roles
   - [ ] Setup communication channels
   - [ ] Start Milestone 1.1, Task 1

3. **Week 1, End**:
   - [ ] Complete procedural robot builder
   - [ ] Test first code generation
   - [ ] Review progress

---

## Communication Plan

- **Daily standups**: 15 min, progress + blockers
- **Weekly reviews**: 1 hour, demos + planning
- **Milestone reviews**: 2 hours, validation + signoff
- **Documentation updates**: Continuous (commit with code)

---

## Appendices

### A. Glossary

- **GIK**: Generalized Inverse Kinematics
- **AGX Orin**: NVIDIA Jetson AGX Orin embedded platform
- **ROS2**: Robot Operating System 2
- **Codegen**: MATLAB Coder code generation
- **Holistic Control**: Full 9-DOF (arm + base) coordinated control
- **Staged Control**: Phased control (Stage A: arm only, B: base path, C: full)

### B. References

- CODEGENCC45_PROJECT_PLAN.md
- MATLAB_CODEGEN_ANALYSIS.md
- ROS2_INTEGRATION_GUIDE.md
- [MATLAB Coder Documentation](https://www.mathworks.com/help/coder/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**Roadmap Version**: 1.0  
**Created**: 2025-10-06  
**Status**: Draft for Approval  
**Next Review**: TBD
