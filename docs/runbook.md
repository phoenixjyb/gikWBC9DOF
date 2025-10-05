# GIK 9-DOF Solver Runbook / GIK 9-DOF 求解器运行手册

This bilingual guide explains how to regenerate MATLAB Coder artifacts and deploy the ROS 2 controllers in this repository. It assumes MATLAB R2024b on Windows, ROS 2 Humble in WSL, and the workspace located at `c:\Users\yanbo\wSpace\codegenGIKsample`.

本双语指南说明如何重新生成 MATLAB Coder 工件以及在本仓库中部署 ROS 2 控制器。假设在 Windows 上使用 MATLAB R2024b，在 WSL 中使用 ROS 2 Humble，工作目录位于 `c:\Users\yanbo\wSpace\codegenGIKsample`。

## 0. Native Ubuntu 22.04 Setup / 原生 Ubuntu 22.04 配置

- **Clone the repository / 克隆仓库**: 
  ```bash
  git clone <repo-url> ~/workspace/codegenGIKsample
  cd ~/workspace/codegenGIKsample
  ```
  Adjust the path to your preferred workspace root. 在 Linux 上可克隆到任意工作目录，上例使用 `~/workspace`。

- **MATLAB availability / MATLAB 使用**: If MATLAB is installed on Ubuntu, run the existing scripts directly (see §1). A headless batch command is: 
  ```bash
  matlab -batch "addpath('Trial/gikWBC9DOF/matlab');gik9dof.codegen.generateRobotModelData;gik9dof.codegen.generateCode('codegen');"
  ```
  若 Linux 端没有 MATLAB，可在 Windows 上生成代码后将 `Trial/gikWBC9DOF/codegen`、`ros2/gik9dof_solver/src/generated` 等目录同步到 Ubuntu。

- **Path adjustments / 路径调整**: Replace Windows-style paths in this runbook with your Linux locations (e.g., `/home/<user>/workspace/codegenGIKsample`). 在后续步骤中将 `c:\Users\yanbo\wSpace` 替换为 Linux 的绝对路径。

- **Dependencies / 依赖项**: All WSL build commands apply to native Ubuntu; ensure ROS 2 Humble, `build-essential`, `python3-colcon-common-extensions`, and `libomp-dev` are installed as described later. 原生系统执行的 `apt` 和 `colcon` 命令与 WSL 相同。

- **ROS workspace entry point / ROS 工作空间入口**: After cloning, `Trial/gikWBC9DOF/ros2` remains the colcon workspace root. Use `source /opt/ros/humble/setup.bash` followed by `colcon build` as in §3–§4. 克隆后，`Trial/gikWBC9DOF/ros2` 仍然是 colcon 工作空间根目录，直接在此运行后续命令。

## 1. MATLAB Code Generation / MATLAB 代码生成

1. **Prepare MATLAB / 准备 MATLAB**
   - Launch MATLAB at the repo root and add the project path:
     ```matlab
     addpath("Trial/gikWBC9DOF/matlab")
     ```
   - Confirm the Robotics System Toolbox is licensed (`ver` should list it).
   - 在仓库根目录启动 MATLAB 并添加项目路径：
     ```matlab
     addpath("Trial/gikWBC9DOF/matlab")
     ```
   - 确认已授权 Robotics System Toolbox（使用 `ver` 检查）。

2. **Refresh the Robot Cache / 刷新机器人缓存**
   - Run `gik9dof.codegen.generateRobotModelData` so `robotModel.mat` reflects the latest URDF and meshes (source: `Trial/gikWBC9DOF/matlab/+gik9dof/+codegen/generateRobotModelData.m`).
   - Execute again whenever robot geometry changes.
   - 运行 `gik9dof.codegen.generateRobotModelData`，确保 `robotModel.mat` 包含最新 URDF 与网格（脚本位置如上）。
   - 每当机器人几何体更新时重新运行。

3. **Generate C++ Code / 生成 C++ 代码**
   - From Windows run `Trial/gikWBC9DOF/run_codegen.bat`, or inside MATLAB call:
     ```matlab
     gik9dof.codegen.generateCode("codegen")
     ```
   - This targets a Linux C++ static library and writes outputs to `Trial/gikWBC9DOF/codegen/linux_arm64`.
   - 在 Windows 中运行 `Trial/gikWBC9DOF/run_codegen.bat`，或在 MATLAB 中执行上述命令。
   - 生成面向 Linux 的 C++ 静态库，输出位于 `Trial/gikWBC9DOF/codegen/linux_arm64`。

4. **Review Artifacts / 检查生成结果**
   - Confirm sources such as `solveGIKStep.cpp` and headers in `codegen/linux_arm64`.
   - Inspect the MATLAB Coder report at `codegen/linux_arm64/html/report.mldatx` if issues arise.
   - 核对 `codegen/linux_arm64` 下的源文件与头文件。
   - 如有问题，查看 `codegen/linux_arm64/html/report.mldatx` 中的 MATLAB Coder 报告。

## 2. Syncing Generated Code into ROS 2 / 将生成代码同步到 ROS 2

1. Copy or merge updated sources into `ros2/gik9dof_solver/src/generated` and headers into `ros2/gik9dof_solver/include/gik9dof_solver/generated`.
2. MATLAB collision helpers reside in `ros2/gik9dof_solver/src/collisioncodegen`; libccd sources live in `ros2/gik9dof_solver/src/libccd`.
3. `ros2/gik9dof_solver/CMakeLists.txt` compiles these directories and exports target `gik9dof_solver::gik9dof_solver` with OpenMP.
4. 将更新的源文件复制或合并到上述 `generated` 目录，并将头文件同步到 `include/gik9dof_solver/generated`。
5. MATLAB 的碰撞辅助代码位于 `ros2/gik9dof_solver/src/collisioncodegen`，libccd 源码位于 `ros2/gik9dof_solver/src/libccd`。
6. `ros2/gik9dof_solver/CMakeLists.txt` 会编译这些目录并导出带 OpenMP 的 `gik9dof_solver::gik9dof_solver` 目标。

## 3. WSL Environment Setup / 配置 WSL 环境

1. In WSL source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Install build dependencies if not already present:
   ```bash
   sudo apt update
   sudo apt install build-essential python3-colcon-common-extensions libomp-dev
   ```
3. Navigate to the workspace:
   ```bash
   cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
   ```
4. 在 WSL 中加载 ROS 2 环境：`source /opt/ros/humble/setup.bash`。
5. 如尚未安装依赖，执行上述 `apt` 命令获得编译工具链。
6. 切换到工作空间目录：`cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2`。

## 4. Building with Colcon / 使用 colcon 构建

1. Build the workspace:
   ```bash
   colcon build
   ```
   - `gik9dof_solver` compiles generated sources, collision helpers, and libccd.
   - `gik9dof_controllers` links to `gik9dof_solver::gik9dof_solver` and ROS 2 dependencies.
2. After the build, source the overlay:
   ```bash
   source install/setup.bash
   ```
3. 使用 `colcon build` 构建整个工作空间，`gik9dof_solver` 将编译生成代码与碰撞辅助文件，`gik9dof_controllers` 会链接该库及 ROS 2 依赖。
4. 构建完成后运行 `source install/setup.bash`，加载覆盖环境。

## 5. Running the Controllers / 运行控制器

1. Example launch for the holistic controller:
   ```bash
   ros2 run gik9dof_controllers holistic_controller --ros-args \
     -p distance_lower_bound:=0.2 -p distance_weight:=0.5
   ```
2. Required input topics: `/robot_state/joint_states` (sensor_msgs/JointState) and `/target/end_effector_trajectory` (trajectory_msgs/MultiDOFJointTrajectory).
3. The staged controller clamps `stage_a_samples` and `stage_b_samples` to non-negative ints (`ros2/gik9dof_controllers/src/staged_controller.cpp`).
4. Enable solver timing logs via `--ros-args -p log_solver_timing:=true`.
5. 示例启动命令如上，可根据需要调整参数。
6. 必需输入话题为 `/robot_state/joint_states` 与 `/target/end_effector_trajectory`。
7. 分阶段控制器会将 `stage_a_samples`、`stage_b_samples` 限制为非负整数（实现见源码）。
8. 如需记录求解器耗时，可追加 `--ros-args -p log_solver_timing:=true`。

## 6. Validation Checklist / 验证清单

1. **Smoke Test / 冒烟测试**
   - Publish a short trajectory from MATLAB or ROS 2 and watch `/control/joint_commands` and `/control/base_commands`.
   - 通过 MATLAB 或 ROS 2 发布短轨迹，观察 `/control/joint_commands` 与 `/control/base_commands`。

2. **Compare Against MATLAB / 对比 MATLAB 结果**
   - Use logs described in Phase 5 of `genCodeGIK.md` to confirm joint outputs and constraint violations.
   - 参考 `genCodeGIK.md` 第 5 阶段记录比对关节输出与约束违规情况。

3. **Performance Monitoring / 性能监测**
   - Check solver timing and consider `OMP_NUM_THREADS` to avoid oversubscription:
     ```bash
     export OMP_NUM_THREADS=4
     ```
   - 关注求解耗时。必要时设置 `OMP_NUM_THREADS`，避免线程过度占用。

## 7. Troubleshooting / 故障排查

- **Codegen Errors / 代码生成错误**: Re-run `generateRobotModelData`, ensure MATLAB paths are correct, review the HTML report.
- **Link Failures / 链接失败**: Confirm `libomp-dev` is installed; try `colcon build --cmake-clean-first` for a clean configure.
- **WSL Proxy Warning / WSL 代理警告**: WSL NAT mode warns about localhost proxies; informational unless you rely on loopback networking.
- **Parameter Type Mismatch / 参数类型不匹配**: `stage_a_samples` and `stage_b_samples` are cast to `int` before `std::max`; keep launch defaults integral.
- 重新运行 `generateRobotModelData`，确保 MATLAB 路径正确，并检查 HTML 报告。
- 确认已安装 `libomp-dev`，必要时使用 `colcon build --cmake-clean-first` 清理后重建。
- WSL NAT 模式可能提示本地代理警告，通常无需处理，除非依赖回环网络。
- `stage_a_samples` 与 `stage_b_samples` 在调用 `std::max` 前已转换为 `int`，发布参数时保持整数即可。

## 8. Maintenance Notes / 维护提示

- Log every code generation session (MATLAB commit, URDF revision, report path) in `Trial/gikWBC9DOF/diary.md`.
- If MathWorks updates collisioncodegen or libccd, copy new sources into `ros2/gik9dof_solver/src/collisioncodegen` and `ros2/gik9dof_solver/src/libccd`, then rebuild.
- Plan to add automated tests (gtest or ROS 2 launch tests) once deterministic trajectories and expected outputs are captured.
- 在 `Trial/gikWBC9DOF/diary.md` 中记录每次代码生成，注明 MATLAB 提交、URDF 版本和报告路径。
- 如 MathWorks 发布新的 collisioncodegen 或 libccd 源码，更新到对应目录后重新构建。
- 当获得稳定轨迹与期望结果后，考虑添加自动化测试（gtest 或 ROS 2 launch 测试）。
