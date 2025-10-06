# 🚀 QUICK START CARD - 2-Day Implementation

## ⏱️ YOUR NEXT 15 MINUTES

### Open MATLAB R2024b
```matlab
% Navigate to project
cd C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF

% Add to path
addpath(genpath('matlab'))

% Run validation
cd matlab/+gik9dof/+codegen_realtime
validate_robot_builder
```

**✅ Expected Result:** All 7 tests pass in green

**❌ If tests fail:**
- Check MATLAB version: `ver` (must be R2024b)
- Check Robotics Toolbox: `ver('robotics')`
- Read error message carefully

---

## ⏱️ NEXT 30 MINUTES

### Generate C++ Code
```matlab
% Still in matlab/+gik9dof/+codegen_realtime
generateCodeARM64
```

**✅ Expected Result:**
```
===================================================
✓ Code generation successful!
===================================================
Generated files location: C:\...\codegen\arm64_realtime
```

**❌ If code generation fails:**
1. Open: `codegen/arm64_realtime/html/report.mldatx`
2. Look for red errors
3. Most common: Unsupported function (check if rigidBodyTree is supported)
4. Fix: Simplify code or use supported alternatives

---

## ⏱️ NEXT 1 HOUR

### Transfer to AGX Orin
```powershell
# On Windows, compress generated code
Compress-Archive -Path codegen\arm64_realtime\* -DestinationPath gik_codegen.zip

# SCP to AGX Orin (replace <orin-ip>)
scp gik_codegen.zip nvidia@<orin-ip>:~/
```

### On AGX Orin
```bash
# Extract
cd ~
unzip gik_codegen.zip -d gik_codegen

# Copy to ROS2 workspace
cd ~/gikWBC9DOF/ros2/gik9dof_solver
mkdir -p matlab_codegen/include matlab_codegen/lib

cp ~/gik_codegen/*.h matlab_codegen/include/
cp ~/gik_codegen/*.hpp matlab_codegen/include/
cp ~/gik_codegen/*.a matlab_codegen/lib/

# Update CMakeLists.txt (see FAST_TRACK_2DAY.md Step 6)
nano CMakeLists.txt  # Uncomment MATLAB library lines
```

---

## ⏱️ NEXT 2 HOURS

### Build ROS2 Packages
```bash
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash

# Build messages first
colcon build --packages-select gik9dof_msgs
source install/setup.bash

# Verify messages
ros2 interface show gik9dof_msgs/msg/EndEffectorTrajectory

# Build solver (this will take time on first build)
colcon build --packages-select gik9dof_solver --cmake-args -DCMAKE_BUILD_TYPE=Release

# If build fails, check dependencies:
sudo apt install libeigen3-dev libomp-dev
```

---

## ⏱️ REMAINING TIME

### Test and Integrate
```bash
# Terminal 1: Launch solver
source install/setup.bash
ros2 launch gik9dof_solver test_solver.launch.py

# Terminal 2: Monitor
ros2 topic echo /gik9dof/solver_diagnostics

# Terminal 3: Publish test trajectory
ros2 topic pub /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory "..." --once
```

**See FAST_TRACK_2DAY.md for detailed commands**

---

## 📋 CRITICAL CHECKPOINTS

- [ ] MATLAB validation passes (7/7 tests)
- [ ] Code generation completes without errors
- [ ] Generated files copied to AGX Orin
- [ ] ROS2 messages build successfully
- [ ] ROS2 solver node builds successfully
- [ ] Solver node runs without crashes
- [ ] Solver publishes diagnostics at ~10 Hz
- [ ] Solve time < 50ms (target: < 20ms)
- [ ] Robot responds to trajectory commands

---

## 🆘 QUICK TROUBLESHOOTING

| Problem | Solution |
|---------|----------|
| MATLAB codegen fails | Check `codegen/arm64_realtime/html/report.mldatx` |
| ROS2 build fails | `sudo apt install libeigen3-dev libomp-dev` |
| Node crashes at runtime | Check array sizes: must be 9 DOF [x,y,θ,arm1-6] |
| Slow solve times | Reduce `MaxIterations` in `solveGIKStepWrapper.m` |
| No topics published | Check topic names: `ros2 topic list` |

---

## 📞 CRITICAL FILES

| File | Purpose |
|------|---------|
| `FAST_TRACK_2DAY.md` | Complete hour-by-hour guide |
| `IMPLEMENTATION_SUMMARY.md` | What you have and where it is |
| `matlab/+gik9dof/+codegen_realtime/` | All MATLAB code |
| `ros2/gik9dof_solver/src/gik9dof_solver_node.cpp` | ROS2 wrapper (edit this!) |

---

## ✅ SUCCESS = 🎉

When you see this, you've won:
```bash
ros2 topic echo /gik9dof/solver_diagnostics
---
header:
  stamp: ...
status: 1          # ← 1 = SUCCESS
iterations: 15     # ← Reasonable (< 100)
solve_time_ms: 18.5  # ← Fast (< 50ms)
pose_error_norm: 0.000234  # ← Small error
```

---

**NOW GO! OPEN MATLAB AND RUN `validate_robot_builder`** 🚀
