# 项目诊断：gikWBC9DOF - 综合系统分析

**生成日期：**2025年10月12日  
**最后更新：**2025年10月12日  
**重点：**分阶段模式数据流、文件关系与系统架构  
**项目：**gikWBC9DOF = **G**eneralized（广义）**I**nverse（逆）**K**inematics（运动学）**W**hole（整体）**B**ody（身体）**C**ontrol（控制），适用于**9 自由度**移动操作臂

---

## 目录

1. [执行摘要](#执行摘要)
2. [项目架构概览](#项目架构概览)
3. [分阶段模式：完整数据流](#分阶段模式完整数据流)
4. [数据对象及其演化](#数据对象及其演化)
5. [文件清单与功能分类](#文件清单与功能分类)
6. [依赖关系与文件关联](#依赖关系与文件关联)
7. [预处理与配置系统](#预处理与配置系统)
8. [实时仿真流水线](#实时仿真流水线)
9. [整体模式：全身 IK 流水线](#整体模式全身-ik-流水线)
10. [模式选择指南](#模式选择指南)
11. [基于日志的动画流水线](#基于日志的动画流水线)
12. [动画系统：数据来源与图例对应](#动画系统数据来源与图例对应)
13. [辅助函数与实用工具](#辅助函数与实用工具)
14. [函数关系分析](#函数关系分析)
15. [Stage C 深入解析：三遍架构与数据流](#stage-c-深入解析三遍架构与数据流)
16. [近期缺陷修复与改进](#近期缺陷修复与改进)
17. [冗余、孤立与弃用文件](#冗余孤立与弃用文件)
18. [关键洞察与建议](#关键洞察与建议)
19. [结论](#结论)
20. [附录：文件索引](#附录文件索引)

---

## 执行摘要

本文对 gikWBC9DOF 项目进行了**全面分析**，重点关注**分阶段模式**的执行流水线。项目通过广义逆运动学（GIK）为移动操作臂实现全身控制系统。

### 关键发现

**✅ 优势：**
- 结构清晰的包系统（`+gik9dof/`）
- 职责划分明确（控制、可视化、评估）
- 日志与诊断基础设施完善
- **统一的参数配置系统**（`config/pipeline_profiles.yaml`）
- 通过带继承的 YAML 提供灵活的底盘配置
- 多种执行模式（整体/分阶段，ppForIk/pureIk）
- 履带宽度在所有文件中统一为 0.574 m

**⚠️ 关注点（此前识别，现已解决）：**
- ~~根目录存在用途不明的重叠脚本~~
- ~~遗留动画函数（`animateStagedLegacy.m`）仍被保留~~
- ~~配置分散在代码与 YAML 之间~~ ✅ **已修复：配置统一**
- ~~函数间参数不一致~~ ✅ **已修复：单一事实来源**
- ~~履带宽度不一致（0.674 对 0.574）~~ ✅ **已修复：统一为 0.574 m**
- 文档分散在多份文件中且部分内容重叠

**📊 项目统计：**
- **MATLAB 文件总数：**约 60 个
- **核心库函数：**`+gik9dof/` 包内 44 个文件
- **顶层脚本：**12 个编排/测试脚本
- **文档文件：**15+ 份 Markdown 文档
- **配置文件：**2 个 YAML 文件（chassis_profiles.yaml、pipeline_profiles.yaml）
- **参考轨迹：**1 个 JSON 末端执行器轨迹

**机器人配置：**
- **总计 9 自由度：**3 个底盘（平面）+ 6 个机械臂（转动）
- **底盘关节：**joint_x、joint_y、joint_theta
- **机械臂关节：**left_arm_joint1 至 left_arm_joint6
- **末端执行器：**left_gripper_link

---

## 项目架构概览

### 系统概述

gikWBC9DOF 项目实现了一个**复杂的全身控制系统**，面向 9 自由度移动操作臂。架构特性包括：
- **双执行模式：**整体模式（单阶段）与分阶段模式（三阶段）
- **统一底盘控制：**所有模式共享的控制器接口
- **灵活配置：**支持继承的 YAML 参数配置
- **全面日志：**具备完整仿真回放能力
- **高级路径规划：**支持 Hybrid A*、Reeds-Shepp 与 Clothoid 平滑

### 目录结构

```
gikWBC9DOF/
├── 🎯 入口脚本（顶层脚本）
│   ├── run_staged_reference.m                 # 分阶段主入口
│   ├── run_environment_compare.m              # 整体 vs 分阶段对比
│   ├── run_fresh_sim_with_animation.m         # 快速测试并生成动画
│   ├── run_parametric_study.m                 # 参数扫描研究
│   ├── run_comprehensive_chassis_study.m      # 底盘控制器调优
│   └── regenerate_animations_from_logs.m      # 日志后处理动画
│
├── 📦 核心库（+gik9dof/ 包）[44 个文件]
│   │
│   ├── 🔧 核心流水线函数
│   │   ├── trackReferenceTrajectory.m         # 模式路由（整体/分阶段）
│   │   ├── runStagedReference.m               # 分阶段便捷封装
│   │   ├── runStagedTrajectory.m              # 三阶段调度器（1986 行）
│   │   ├── runTrajectoryControl.m             # IK 控制循环（518 行）
│   │   ├── createGikSolver.m                  # 求解器组合工厂
│   │   ├── createRobotModel.m                 # 机器人模型工厂
│   │   ├── configurationTools.m               # 配置工具
│   │   ├── environmentConfig.m                # 环境设定
│   │   ├── generateHolisticRamp.m             # 平滑斜坡生成器
│   │   └── loadPipelineProfile.m              # 统一配置加载 ✨新增
│   │
│   ├── 🚗 底盘控制子系统 [11 个文件]
│   │   ├── +control/
│   │   │   ├── unifiedChassisCtrl.m           # 中央指令枢纽（129 行）
│   │   │   ├── purePursuitFollower.m          # 自适应路径跟随（338 行）
│   │   │   ├── simulateChassisExecution.m     # 多模式仿真器（328 行）
│   │   │   ├── simulatePurePursuitExecution.m # 纯跟踪仿真封装（83 行）
│   │   │   ├── preparePathForFollower.m       # 路径预处理（255 行）
│   │   │   ├── rsRefinePath.m                 # Reeds-Shepp 平滑（292 行）
│   │   │   ├── rsClothoidRefine.m             # Clothoid 平滑（204 行）
│   │   │   ├── clampYawByWheelLimit.m         # 运动学可行性守门（45 行）
│   │   │   ├── loadChassisProfile.m           # YAML 配置加载器（148 行）
│   │   │   ├── defaultUnifiedParams.m         # 备用默认值（12 行）
│   │   │   └── defaultReedsSheppParams.m      # RS 默认参数（32 行）
│   │   └── 详见第 10 节深入分析
│   │
│   ├── 🎨 可视化子系统 [6 个文件]
│   │   ├── +viz/
│   │   │   ├── animate_whole_body.m           # 主动画器（600+ 行）
│   │   │   └── animatePurePursuitSimulation.m # 纯跟踪可视化
│   │   ├── animateStagedWithHelper.m          # 分阶段封装（219 行）
│   │   ├── animateHolisticWithHelper.m        # 整体模式封装
│   │   ├── animateStagedLegacy.m              # 遗留动画器 ⚠️ 已弃用
│   │   └── animateTrajectory.m                # 简易快速可视化
│   │
│   ├── 🔍 评估与诊断 [7 个文件]
│   │   ├── evaluateLog.m                      # 日志指标分析
│   │   ├── generateLogPlots.m                 # 标准诊断绘图
│   │   ├── plotTrajectoryLog.m                # 轨迹可视化
│   │   ├── evaluatePathSmoothness.m           # 曲率/冲击指标
│   │   ├── evaluateCollisionIntrusion.m       # 障碍距离检查
│   │   ├── evaluateChassisConstraints.m       # 速度/车轮限制检查
│   │   ├── computeBaseRibbonMetrics.m         # 底盘路径曲率分析
│   │   └── comprehensiveEvaluation.m          # 全面评估套件
│   │
│   ├── 🌍 环境与碰撞 [4 个文件]
│   │   ├── environmentConfig.m                # 环境工厂
│   │   ├── addFloorDiscs.m                    # 障碍物布置
│   │   ├── collisionTools.m                   # 网格附着
│   │   └── demoFloorDiscs.m                   # 可视化示例
│   │
│   ├── 🔧 内部工具 [5 个文件]
│   │   ├── +internal/
│   │   │   ├── createResultsFolder.m          # 时间戳目录
│   │   │   ├── resolvePath.m                  # 路径解析
│   │   │   ├── projectRoot.m                  # 项目根目录定位
│   │   │   ├── VelocityEstimator.m            # 底盘速度估计
│   │   │   └── addChassisFootprint.m          # 碰撞包络生成
│   │
│   ├── 🐛 调试工具 [1 个文件]
│   │   └── +debug/
│   │       └── visualizeStageBOccupancy.m     # Stage B 占据网格可视化
│   │
│   └── 📊 其他辅助工具 [约 10 个文件]
│       ├── loadJsonTrajectory.m               # JSON 轨迹加载器
│       ├── saveRunArtifacts.m                 # 保存结果/动画
│       └── [完整清单见第 4 节]
│
├── 📐 配置文件（统一系统）✨新增
│   ├── config/
│   │   ├── pipeline_profiles.yaml             # 统一配置（推荐）
│   │   │   ├── chassis（履带、限制、增益）
│   │   │   ├── stage_b（模式、规划、控制器）
│   │   │   ├── stage_c（跟踪、精修）
│   │   │   ├── gik（求解器、迭代、权重）
│   │   │   ├── pure_pursuit（前视、PID）
│   │   │   └── holistic（坡道、速度限制）
│   │   └── chassis_profiles.yaml              # 仅底盘配置（遗留支持）
│   │
│   ├── 📏 参考轨迹
│   │   └── 1_pull_world_scaled.json           # 148 个末端执行器航点
│   │
│   └── 🎭 机器人模型与网格
│       ├── mobile_manipulator_PPR_base_corrected.urdf        # 主 URDF
│       ├── mobile_manipulator_PPR_base_corrected_sltRdcd.urdf # 精简网格
│       └── meshes/                            # STL 碰撞几何
│           ├── base_link.STL
│           ├── left_arm_link[1-6].STL
│           ├── left_gripper_link.STL
│           ├── wheel_[lf|lr|rf|rr]_link.STL
│           └── stl_output/                    # 网格精简版本
│
├── 📁 结果归档（自动生成）
│   └── results/                               # 时间戳仿真输出
│       └── <YYYYMMDD_HHMMSS>_<label>/
│           ├── log_*.mat                      # 仿真日志（全状态）
│           ├── *.mp4                          # 动画（双视角）
│           ├── *.png                          # 诊断图
│           ├── *.csv                          # 指标导出（指令、误差）
│           └── *.json                         # 元数据（配置、环境）
│
├── 📚 文档中心
│   ├── docs/
│   │   ├── projectDiagnosis.md                # 本文件——完整分析
│   │   ├── CHASSIS_CONTROL_ANALYSIS.md        # 底盘系统深度解析 ✨新增
│   │   ├── unified_chassis_controller_summary.md # 设计说明
│   │   ├── PROJECT_STATUS_SUMMARY.md          # 状态与成果
│   │   ├── SIMULATION_WORKFLOW_GUIDE.md       # 用户指南
│   │   ├── COMPREHENSIVE_STUDY_GUIDE.md       # 参数调优指南
│   │   ├── GIK_SETUP_OVERVIEW.md              # 求解器配置
│   │   ├── UNIFIED_CONFIG_IMPLEMENTATION.md   # 配置系统设计 ✨新增
│   │   └── [其他 10+ 指南]
│   │
│   └── 📝 根目录文档
│       ├── README.md                          # 项目入口
│       ├── HANDOVER.md                        # 交接说明
│       ├── guideline.md                       # 开发指南
│       └── diary.md                           # 开发日志
│
└── 📂 MATLAB 脚本与测试
    ├── matlab/                                # MATLAB 专用工具
    │   ├── plotJsonPath.m                     # JSON 路径绘图
    │   ├── renderWholeBodyAnimation.m         # 动画渲染器
    │   ├── unified_chassis_replay.m           # 底盘指令回放
    │   ├── run_gik_iteration_study.m          # 求解器迭代分析
    │   ├── run_parameter_sweep.m              # 通用参数扫描
    │   ├── run_stageb_mode_compare.m          # Stage B 模式对比
    │   └── +gik9dof/                          # [核心包——见上]
    │
    ├── tests/                                 # 测试脚本（待整理）
    │   └── [各类 test_*.m 文件]               # 集成测试
    │
    ├── scripts/                               # 分析脚本
    │   ├── analyze_stage_collisions.m         # 碰撞诊断
    │   └── [其他分析工具]
    │
    └── archive/                               # 归档/弃用文件
        ├── docs/                              # 旧文档
        ├── scripts/                           # 旧脚本
        └── temp/                              # 临时文件
```

### 关键架构特性

#### 1. **统一的底盘控制系统** ✨
- **单一命令接口**（`unifiedChassisCtrl`）覆盖所有执行模式
- **四层架构**：执行 → 控制 → 预处理 → 配置
- **11 个专用函数** 无冗余（详见第 10 节）
- **运动学可行性约束**由差动驱动限制确保

#### 2. **灵活的配置系统** ✨
- **统一配置文件** `pipeline_profiles.yaml`（包含底盘、各阶段、GIK 与纯跟踪）
- **配置继承**（例如 `aggressive` 继承自 `default`）
- **运行时覆写** 支持所有参数
- **向后兼容** 保留 `chassis_profiles.yaml`

#### 3. **双执行模式**
- **Holistic（整体模式）**：单阶段全身 IK（pureIk 或 ppForIk）
- **Staged（分阶段模式）**：三阶段拆解（A：手臂，B：底盘，C：全身）

#### 4. **三遍求解架构（ppForIk）** ✨
- **Pass 1**：参考 IK（理想轨迹，无底盘约束）
- **Pass 2**：底盘仿真（真实底盘运动，包含动力学限制）
- **Pass 3**：固定底盘的最终 IK（可实现的手臂动作）

#### 5. **全面日志体系**
- **按阶段保存日志**（stageA、stageB、stageC）
- **增强诊断**：曲率、回转、平滑指标
- **完整回放**：无需重仿真即可再生成动画
- **速度估计**：底盘采用自适应反向差分

#### 6. **高级路径规划**
- **Hybrid A***：考虑曲率的网格规划
- **Reeds-Shepp 精修**：随机捷径 + 碰撞检查
- **Clothoid 平滑**：满足 C¹ 连续性的样条
- **纯跟踪跟随**：自适应前视，支持混合/Stanley 模式

### 架构演进

| 项目 | 初始状态 | 当前状态 |
|------|----------|----------|
| **配置** | 分散 | ✅ 统一 YAML 系统 |
| **履带宽度** | 0.674/0.576/0.573 不一致 | ✅ 标准化 0.574 m |
| **底盘控制** | 多个实现 | ✅ 单一统一控制器 |
| **文档** | 零散 | ✅ 整合并提供交叉引用 |
| **动画** | 遗留函数 | ✅ 统一至 `animate_whole_body` |
| **默认参数** | 函数各自维护 | ✅ 流水线配置（1500 次迭代、pureHyb、10 Hz） |

### 快速上手指南

**运行基础分阶段仿真：**
```matlab
% 使用默认设置
result = gik9dof.runStagedReference();

% 使用自定义配置档
cfg = gik9dof.loadPipelineProfile('aggressive');
result = gik9dof.runStagedReference('PipelineConfig', cfg);

% 运行时覆写参数
result = gik9dof.runStagedReference(...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 1500, ...
    'ChassisProfile', 'wide_track');
```

**结果保存位置：**`results/<timestamp>_staged_reference/`

**重新生成动画：**
```matlab
regenerate_animations_from_logs('results/<your_folder>');
```

### 新成员指南

**了解系统：**
1. 阅读本节（架构概览）
2. 阅读第 3 节（分阶段数据流）
3. 查阅第 10 节（函数关系，重点关注底盘控制）
4. 参考 `docs/SIMULATION_WORKFLOW_GUIDE.md`（操作指南）

**修改参数：**
- 推荐编辑 `config/pipeline_profiles.yaml`
- 或在函数调用中直接覆写参数

**新增配置档：**
```yaml
# 位于 pipeline_profiles.yaml
profiles:
  my_custom:
    inherits: "default"
    overrides:
      chassis: {vx_max: 2.0, accel_limit: 1.0}
      stage_b: {desired_linear_velocity: 0.8}
```

**调试建议：**
- 检查 `log.stageLogs.{stageA,stageB,stageC}` 获取分阶段详情
- 使用 `evaluateLog()` 快速生成关键指标
- 使用 `generateLogPlots()` 绘制诊断图
- 阅读 `docs/CHASSIS_CONTROL_ANALYSIS.md` 了解控制器行为

---

## 分阶段模式：完整数据流

### 概览

分阶段模式将运动分解为三个连续阶段：

1. **Stage A** —— 仅手臂渐进（底盘锁定，手臂对齐首个航点）
2. **Stage B** —— 仅底盘导航（手臂锁定，底盘移动至停泊位）
3. **Stage C** —— 全身跟踪（底盘与手臂共同跟随参考轨迹）

### 数据流图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           初始化                                         │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  入口脚本：run_staged_reference.m                                       │
│  ├─ 加载：config/chassis_profiles.yaml                                  │
│  ├─ 调用：gik9dof.environmentConfig() → 底盘初始位姿、障碍、距离裕度      │
│  └─ 调用：gik9dof.trackReferenceTrajectory('Mode', 'staged', ...)       │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  trackReferenceTrajectory.m（模式路由器）                                │
│  ├─ 加载：1_pull_world_scaled.json → 148 个末端执行器航点               │
│  ├─ 创建：调用 createRobotModel() 构建机器人模型                         │
│  ├─ 设置：q0（底盘处于基准位姿的初始配置）                              │
│  ├─ 附加：地面圆盘障碍至机器人模型                                      │
│  ├─ 创建：避障距离约束                                                  │
│  └─ 路由：gik9dof.runStagedTrajectory(...)                              │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        STAGE A：手臂渐进                                 │
│  runStagedTrajectory.m → Stage A 执行                                    │
│  ├─ 输入：q0、首个末端执行器位姿 T1                                      │
│  ├─ 生成：从当前 T0→T1 的 50 个插值位姿                                 │
│  ├─ 创建：bundleA（底盘锁定的 GIK 求解器）                               │
│  ├─ 调用：runTrajectoryControl(bundleA, trajA, ...)                      │
│  └─ 输出：logA（qTraj、时间戳、诊断信息）                                │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                   STAGE B：底盘导航                                      │
│  runStagedTrajectory.m → Stage B 执行                                    │
│  ├─ 输入：qA_end（Stage A 结束配置）                                     │
│  ├─ 模式："gikInLoop" 或 "pureHyb"                                       │
│  │                                                                        │
│  ├─ 若为 pureHyb：                                                       │
│  │   ├─ 计算：首个航点对应的目标底盘位姿                                 │
│  │   ├─ 规划：Hybrid A* 路径（或线性插值）                               │
│  │   ├─ 平滑：Reeds-Shepp 捷径（可选）                                   │
│  │   ├─ 平滑：Clothoid 精修（可选）                                      │
│  │   ├─ 仿真：纯跟踪控制器                                               │
│  │   ├─ 生成：底盘轨迹（x, y, theta）                                    │
│  │   └─ 输出：logB（合成日志、pathStates、cmdLog）                        │
│  │                                                                        │
│  └─ 若为 gikInLoop：                                                     │
│      ├─ 插值：直线底盘轨迹                                               │
│      ├─ 创建：bundleB（手臂锁定的 GIK 求解器）                           │
│      ├─ 调用：runTrajectoryControl(bundleB, trajB, ...)                  │
│      └─ 输出：logB（qTraj、时间戳、诊断信息）                            │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                  STAGE C：全身跟踪                                      │
│  runStagedTrajectory.m → Stage C 执行                                    │
│  ├─ 输入：qB_end（Stage B 结束配置）、剩余航点                            │
│  ├─ 模式："ppForIk" 或 "pureIk"                                          │
│  │                                                                        │
│  ├─ 若为 ppForIk：                                                       │
│  │   ├─ 创建：bundleRef（参考遍 GIK 求解器）                              │
│  │   ├─ 执行：runTrajectoryControl() 获取参考底盘轨迹                     │
│  │   ├─ （可选）平滑：对底盘轨迹应用 RS + Clothoid                       │
│  │   ├─ 仿真：纯跟踪控制器                                               │
│  │   ├─ 创建：bundleFinal（固定底盘轨迹的 GIK 求解器）                   │
│  │   ├─ 执行：runTrajectoryControl()（含 FixedJointTrajectory）           │
│  │   └─ 输出：logC（qTraj、purePursuit 数据、诊断信息）                   │
│  │                                                                        │
│  └─ 若为 pureIk：                                                        │
│      ├─ 创建：bundleC（标准 GIK 求解器）                                 │
│      ├─ 执行：runTrajectoryControl(bundleC, trajC, ...)                  │
│      └─ 输出：logC（qTraj、时间戳、诊断信息）                            │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      日志合并与保存                                     │
│  runStagedTrajectory.m → mergeStageLogs()                                │
│  ├─ 拼接：各阶段 qTraj                                                    │
│  ├─ 拼接：各阶段时间戳                                                   │
│  ├─ 存储：stageLogs.stageA/B/C                                           │
│  ├─ 返回：pipeline 结构体                                                │
│  └─ 保存：results/<timestamp>_<label>/log_staged_<mode>.mat               │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      动画生成                                           │
│  （可选）由入口脚本或重生成脚本调用                                     │
│  ├─ 加载：log_staged_*.mat                                               │
│  ├─ 调用：animateStagedWithHelper() 或 viz.animate_whole_body()          │
│  ├─ 渲染：基于 qTraj 的正运动学姿态                                      │
│  ├─ 叠加：参考路径、障碍物、阶段标签                                    │
│  └─ 保存：*.mp4 动画文件                                                 │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 数据对象及其演化

### 1. 配置向量（q）

**类型：** `[9×1 double]` 列向量  
**内容：** 所有自由度的关节角

```matlab
% 关节顺序（共 9 自由度）：
q(1:3) = [joint_x, joint_y, joint_theta];           % 移动底盘（平面）
q(4:9) = [left_arm_joint1 ... left_arm_joint6];     % 6 自由度机械臂
```

**阶段演化：**
- `q0` —— 初始配置（底盘处于基准位姿）
- `qA_end` —— Stage A 结束（手臂对齐首个航点，底盘未动）
- `qB_end` —— Stage B 结束（底盘移动至停泊位，手臂保持）
- `qTraj` —— 全流程轨迹（9×N），贯穿三阶段

### 2. 轨迹结构体（trajStruct）

**类型：** 从 JSON 载入的结构体

```matlab
trajStruct = struct(
    'EndEffectorName',        % 末端执行器名称（left_gripper_link）
    'Poses',                  % [4×4×N] 齐次变换矩阵
    'EndEffectorPositions',   % [3×N] 目标位置（米）
    'EndEffectorOrientations' % [4×4×N] 目标姿态
);
```

**来源：** `1_pull_world_scaled.json`（148 个航点）

### 3. 日志结构体（log）

**类型：** 含嵌套数据的综合结构体

```matlab
log = struct(
    % 核心轨迹数据
    'qTraj',                  % [9×N] 全局关节轨迹
    'timestamps',             % [1×N] 时间戳
    'successMask',            % [1×N] 每步求解是否成功

    % 末端执行器跟踪
    'eePositions',            % [3×N] 实际 EE 位置（正运动学）
    'targetPositions',        % [3×N] 目标 EE 位置（JSON）
    'eePoses',                % [4×4×N] 实际 EE 姿态
    'positionError',          % [3×N] 跟踪误差向量
    'positionErrorNorm',      % [1×N] 误差模长

    % 求解器诊断
    'solutionInfo',           % {1×N} 每步求解信息
    'iterations',             % [1×N] 每步迭代次数
    'solveTime',              % [1×N] 每步求解时间
    'solverSummary',          % 汇总统计

    % 速度估计（若启用）
    'baseVelocityEstimate',   % 底盘速度估计结构体

    % 模式信息
    'mode',                   % 'holistic' 或 'staged'
    'simulationMode',         % 'ppForIk' 或 'pureIk'

    % 纯跟踪数据（ppForIk 时）
    'purePursuit', struct(
        'referencePath',      % [N×3] 参考底盘状态
        'simulation',         % 控制器仿真结果
        'executedPath',       % [N×3] 实际执行底盘状态
        'commands',           % [N×2] (Vx, Wz) 指令
        'wheelSpeeds',        % [N×2] 左/右轮速度
        'status'              % 每步控制状态
    ),

    % 分阶段日志（仅分阶段模式）
    'stageLogs', struct(
        'stageA',             % Stage A 日志
        'stageB',             % Stage B 日志（含诊断）
        'stageC'              % Stage C 日志
    ),

    % 环境与配置
    'floorDiscs',             % 障碍物信息
    'distanceSpecs',          % 碰撞距离约束
    'environment',            % 环境配置
    'chassisParams',          % 底盘参数
    'chassisProfile'          % 配置档名称（如 'wide_track'）
);
```

### 4. Stage B 诊断信息（增强日志）

```matlab
stageBDiagnostics = struct(
    % 路径曲率指标
    'baseCurvature',          % [N×1] 每点曲率
    'curvatureHistogram',     % 曲率直方图（低/中/高/极高）
    'maxCurvature',           % 最大曲率
    'meanCurvature',          % 平均曲率
    'pathSmoothness',         % 曲率标准差

    % 回转检测
    'cuspLocations',          % [M×1] 回转位置索引
    'cuspCount',              % 回转次数

    % 平滑指标
    'rsAcceptanceRate',       % Reeds-Shepp 接受率
    'rsImprovements',         % 接受的改进次数
    'rsIterations',           % RS 总迭代次数
    'rsPathLengthImprovement',% 路径长度改进（米）
    'clothoidApplied',        % 是否应用 Clothoid
    'clothoidSegments',       % 拟合段数

    % 性能
    'plannerComputeTime'      % 规划耗时（秒）
);
```

### 5. Stage C 诊断信息（增强日志）

```matlab
stageCDiagnostics = struct(
    % 求解器性能
    'solverIterationsPerWaypoint',  % [N×1] 每步迭代次数
    'maxIterationsHit',             % 命中迭代上限的步数

    % 跟踪质量分档
    'eeErrorBins', struct(
        'excellent',  % <0.05 m
        'good',       % 0.05–0.10 m
        'acceptable', % 0.10–0.20 m
        'poor'        % >0.20 m
    ),

    % 误差统计
    'eeErrorMean',            % 平均跟踪误差（米）
    'eeErrorMax',             % 最大跟踪误差（米）
    'eeErrorRMS',             % 均方根误差

    % 底盘跟踪偏差
    'baseYawDriftMean',       % 平均航向偏差（弧度）
    'baseYawDriftMax',        % 最大航向偏差（弧度）
    'basePosDeviationMean',   % 平均位置偏差（米）
    'basePosDeviationMax',    % 最大位置偏差（米）

    % 精修信息（若启用）
    'refinementApplied',      % 是否应用精修
    'refinementDelta', struct(
        'pathLength',         % 路径长度变化（米）
        'eeError'             % EE 误差变化（米）
    )
);
```

**求解器遥测**（由 `runTrajectoryControl` 记录）：
- 每步迭代次数、求解时间、约束违规
- 汇总统计：平均/最大/最小求解时间与成功率
- 收敛状态汇总（退出标志、状态字符串）

**性能观察**（迭代实验结论）：
- 迭代上限 150：平均求解约 0.2 s，频繁命中上限
- 迭代上限 1500：平均求解约 0.69 s，收敛性更好
- Stage C 末端执行器误差在两种设置下 RMS 均 <1 mm
- 取舍：速度 vs. 收敛质量

---

## 文件清单与功能分类

### 核心控制与规划（14 个文件）

| 文件 | 功能 | 输入 | 输出 | 依赖 |
|------|------|------|------|------|
| **trackReferenceTrajectory.m** | 模式路由器 | JSON 路径、选项 | log | runStagedTrajectory、runTrajectoryControl |
| **runStagedReference.m** | 分阶段封装 | 选项 | result 结构体 | trackReferenceTrajectory + I/O |
| **runStagedTrajectory.m** | 分阶段调度器 | robot、trajStruct、q0 | pipeline | Stage A/B/C 函数 |
| **runTrajectoryControl.m** | IK 控制循环 | bundle、轨迹、q0 | log | GIK 求解器 |
| **createGikSolver.m** | 求解器工厂 | robot、约束 | bundle | generalizedInverseKinematics |
| **createRobotModel.m** | 机器人模型 | URDF 路径 | robot、足迹 | rigidBodyTree |
| **configurationTools.m** | 配置工具 | robot | tools | - |
| **environmentConfig.m** | 环境设定 | - | config 结构体 | - |
| **generateHolisticRamp.m** | 坡道生成 | q0、T1 | rampInfo | IK 求解 |
| **runEnvironmentCompare.m** | 模式对比 | - | - | trackReferenceTrajectory |
| **addFloorDiscs.m** | 障碍添加 | robot、圆盘 | discInfo | rigidBody |
| **collisionTools.m** | 碰撞配置 | robot | tools | STL 网格 |
| **evaluateLog.m** | 日志分析 | log | 指标 | - |
| **comprehensiveEvaluation.m** | 全面评估 | results | summary | 多个子模块 |

### 底盘控制子系统（11 个文件）

底盘控制子系统通过三种执行模式（整体、分阶段 Stage C、分阶段 Stage B）调度移动底盘动作，采用四层功能架构：

| 文件 | 行数 | 层级 | 功能 |
|------|------|------|------|
| **unifiedChassisCtrl.m** | 129 | 执行层 | 三模式中央控制器 |
| **purePursuitFollower.m** | 338 | 跟随层 | 底盘感知的路径跟随（类） |
| **simulateChassisExecution.m** | 328 | 测试层 | 多模式控制器仿真 |
| **simulatePurePursuitExecution.m** | 83 | 测试层 | 纯跟踪积分封装 |
| **preparePathForFollower.m** | 255 | 跟随层 | 路径归一化与预处理 |
| **rsRefinePath.m** | 292 | 跟随层 | Reeds-Shepp 捷径平滑 |
| **rsClothoidRefine.m** | 204 | 跟随层 | Clothoid 样条平滑 |
| **loadChassisProfile.m** | 148 | 配置层 | 支持继承的 YAML 配置加载 |
| **defaultUnifiedParams.m** | 12 | 配置层 | 统一控制默认参数 |
| **defaultReedsSheppParams.m** | 32 | 配置层 | RS 平滑默认参数 |
| **clampYawByWheelLimit.m** | 45 | 约束层 | 差动驱动车轮限制检查 |

**→ 详见“函数关系分析”章节以获取架构、数据流与冗余分析。**

### 可视化子系统（6 个文件）

| 文件 | 功能 | 输入 | 输出 | 主要用途 |
|------|------|------|------|----------|
| **+viz/animate_whole_body.m** | 主动画器 | robot、轨迹、选项 | MP4 视频 | 主动画流水线 |
| **animateStagedWithHelper.m** | 分阶段封装 | log、选项 | 调用 animate_whole_body | 分阶段模式 |
| **animateHolisticWithHelper.m** | 整体封装 | log、选项 | 调用 animate_whole_body | 整体模式 |
| **animateStagedLegacy.m** | 遗留动画器 | log、选项 | Figure | **已弃用** |
| **animateTrajectory.m** | 简易动画器 | 轨迹 | Figure | 快速可视化 |
| **+viz/animatePurePursuitSimulation.m** | 纯跟踪可视化 | 仿真结果 | 动态图 | 调试纯跟踪 |

### 评估与绘图（7 个文件）

| 文件 | 功能 | 指标 | 使用场景 |
|------|------|------|----------|
| **evaluateLog.m** | 日志评估 | EE 误差、成功率 | 仿真后分析 |
| **generateLogPlots.m** | 标准绘图 | 手臂关节、底盘速度 | 诊断输出 |
| **plotTrajectoryLog.m** | 轨迹绘制 | 路径、误差 | 快速分析 |
| **evaluatePathSmoothness.m** | 平滑度指标 | 曲率、冲击 | 路径质量 |
| **evaluateCollisionIntrusion.m** | 碰撞检查 | 最小距离 | 安全验证 |
| **evaluateChassisConstraints.m** | 约束检查 | 速度、车轮限值 | 可行性验证 |
| **computeBaseRibbonMetrics.m** | 底盘路径指标 | 曲率直方图 | Stage B/C 分析 |

### 环境与资产（4 个文件）

| 文件 | 功能 | 关键数据 | 依赖 |
|------|------|----------|------|
| **environmentConfig.m** | 环境工厂 | 底盘初始位、障碍、裕度 | - |
| **addFloorDiscs.m** | 障碍布置 | 将圆盘体添加到树 | rigidBody |
| **collisionTools.m** | 碰撞配置 | STL → 碰撞几何 | meshes/ 目录 |
| **demoFloorDiscs.m** | 可视化示例 | 展示障碍布置 | 测试 |

### 内部工具（5 个文件）

| 文件 | 功能 | 作用 | 关键用途 |
|------|------|------|----------|
| **+internal/createResultsFolder.m** | 目录管理 | 生成时间戳结果目录 | 结果组织 |
| **+internal/resolvePath.m** | 路径解析 | 相对路径 → 绝对路径 | 资源加载 |
| **+internal/projectRoot.m** | 根目录定位 | 查找项目根目录 | 路径解析 |
| **+internal/VelocityEstimator.m** | 速度估计 | 反向差分 | 底盘速度估算 |
| **+internal/addChassisFootprint.m** | 足迹生成 | 碰撞体构建 | 避障 |

### 编排与入口脚本（根目录 12 个）

| 文件 | 功能 | 典型用途 | 状态 |
|------|------|----------|------|
| **run_staged_reference.m** | 分阶段入口 | 正式仿真 | ✅ 使用中 |
| **run_environment_compare.m** | 整体 vs 分阶段 | 模式对比 | ✅ 使用中 |
| **run_fresh_sim_with_animation.m** | 快速测试 | 快速迭代 | ✅ 使用中 |
| **run_parametric_study.m** | 参数扫描 | 优化实验 | ✅ 使用中 |
| **run_comprehensive_chassis_study.m** | 底盘研究 | 控制调优 | ✅ 使用中 |
| **regenerate_animations_from_logs.m** | 动画重生 | 后处理 | ✅ 使用中 |
| **matlab/run_gik_iteration_study.m** | 迭代分析 | 求解器评估 | ✅ 使用中 |
| **matlab/run_parameter_sweep.m** | 通用扫描 | 批量测试 | ✅ 使用中 |
| **matlab/run_stageb_mode_compare.m** | Stage B 对比 | 模式验证 | ✅ 使用中 |
| **matlab/run_stageb_purehyb_smoke.m** | 冒烟测试 | 快速验证 | ✅ 使用中 |
| **matlab/unified_chassis_replay.m** | 底盘回放 | 控制器调试 | ✅ 使用中 |
| **export_all_commands.m** | 指令导出 | 文档用途 | ⚠️ 辅助工具 |

### 测试与调试脚本（根目录 20+ 个）

**状态：** 多数功能重叠，建议整合或迁移

| 类别 | 文件 | 功能 | 建议 |
|------|------|------|------|
| **动画调试** | `debug_animation_sync.m`、`debug_sampling_mismatch.m`、`debug_stage_boundaries.m`、`debug_stagec_ee_path.m` | 修复动画问题 | ⚠️ 修复后归档 |
| **动画生成** | `generate_animation_from_saved_log.m`、`generate_comprehensive_animations.m`、`generate_final_animation.m`、`generate_parametric_animations.m`、`generate_sweep_animations.m` | 各类动画生成 | ⚠️ 建议整合 |
| **测试脚本** | `test_animation_sync_fix.m`、`test_complete_fix.m`、`test_comprehensive_evaluation.m`、`test_enhanced_logging.m`、`test_issue_fixes.m`、`test_parameter_sweep.m`、`test_single_animation_sync.m`、`test_stage_sync_fix.m`、`test_stagec_path_fix.m`、`test_tuned_parameters.m` | 验证测试 | ⚠️ 移至 tests/ |
| **分析脚本** | `analyze_all_tests.m`、`analyze_stage_collisions.m`、`check_reference_quality.m`、`compare_test_configs.m`、`investigate_cusps.m` | 仿真后分析 | ✅ 保留 |
| **临时文件** | `tmp_compare.mat`、`tmp_json.mat`、`tmp_pipeline.mat`、`tmp_regen_staged_only.m`、`tmp_regen.m`、`temp_ref_rs_refine.m` | 临时工作 | ❌ 清理 |

---

## 依赖关系与文件关联

### ⚠️ 关键提醒：区分 runStagedReference 与 runStagedTrajectory

这两个函数名称十分相似，但职责完全不同。

---

### 🚨 重要警示：默认参数曾不一致 **（已解决）**

**✅ 更新（2025-10-11）：默认参数已统一！**

`runStagedReference` 与 `trackReferenceTrajectory` 现均采用**相同的生产级默认值**：
- ✅ **MaxIterations = 1500**（高精度）
- ✅ **StageBMode = "pureHyb"**（生产规划模式）
- ✅ **RateHz = 10 Hz**（生产控制频率）
- ✅ **UseStageBHybridAStar = true**（启用路径规划）

**核心算法相同，如今默认参数也保持一致！**

#### 历史背景（统一前）

过去两个函数默认值不同，导致行为不一致：

| 参数 | 旧 runStagedReference | 旧 trackReferenceTrajectory | 状态 |
|------|-----------------------|-----------------------------|------|
| MaxIterations | 150 | 1500 | ✅ **现均为 1500** |
| StageBMode | "pureHyb" | "gikInLoop" | ✅ **现均为 "pureHyb"** |
| RateHz | 10 Hz | 100 Hz | ✅ **现均为 10 Hz** |
| UseStageBHybridAStar | true | false | ✅ **现均为 true** |
| StageBDesiredLinearVelocity | 0.5 m/s | 0.6 m/s | ⚠️ 仍有小差异 |
| StageBMaxAngularVelocity | 2.0 rad/s | 2.5 rad/s | ⚠️ 仍有小差异 |

**剩余小差异：**
- 速度相关参数存在轻微差别，但对行为影响有限
- 如需完全一致，可显式覆写

**✅ 结果：**无需参数即可调用任一函数并获得一致的生产级结果！

---

#### **runStagedReference.m** —— 高层便捷封装（132 行）

- **位置：** `matlab/+gik9dof/runStagedReference.m`
- **目的：** 面向用户的便捷函数，用于快速运行分阶段流水线

**主要步骤：**
1. 加载默认环境配置（`environmentConfig()`）
2. 应用用户覆写（若提供）
3. 调用 `trackReferenceTrajectory`（`'Mode','staged'`）
4. 创建带时间戳的结果目录
5. 自动保存日志至磁盘
6. 返回包含路径与元信息的 result 结构体

**调用者：**
- 用户脚本：`test_tuned_parameters.m`、`run_parametric_study.m` 等
- 参数研究与批量比对
- 任何需快速运行并自动保存的脚本

**被调用者：**
- `gik9dof.trackReferenceTrajectory(..., 'Mode', 'staged', ...)`

**典型用法：**
```matlab
% 一行代码完成分阶段执行
result = gik9dof.runStagedReference( ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 150);

% 自动执行：
% - 加载环境配置
% - 创建 results/<timestamp>/ 文件夹
% - 保存 log_staged_ppForIk.mat
% - 返回携带路径信息的结果结构体
```

**特点：**
- ✅ 便捷：无需手动配置环境或保存结果
- ✅ 生产级默认值（1500 次迭代、pureHyb、10 Hz）
- ✅ 自动保存日志
- ✅ 返回完整元数据
- ✅ **现已与 trackReferenceTrajectory 使用相同默认值**

---

#### **runStagedTrajectory.m** —— 分阶段执行引擎（1986 行）

- **位置：** `matlab/+gik9dof/runStagedTrajectory.m`
- **目的：** 低层引擎，负责实际的三阶段执行

**主要步骤：**
1. **Stage A** —— 手臂渐进（50 个样本）
2. **Stage B** —— 底盘导航（Hybrid A* + 控制器）
3. **Stage C** —— 全身跟踪（剩余航点）
4. 合并阶段日志为统一流水线
5. 返回完整流水线日志（不含文件 I/O）

**调用者：**
- `trackReferenceTrajectory`（当 `Mode = 'staged'`）
- **不会**被用户脚本直接调用

**被调用者：**
- `runStageA()` —— 手臂渐进
- `runStageBGikInLoop()` 或 `runStageBPureHyb()` —— 底盘导航
- `runStageCPpForIk()` 或 `runStageCPureIk()` —— 全身跟踪
- `mergeStageLogs()` —— 日志合并

**典型用法：**
```matlab
% 低层调用（用户通常不会直接使用）
pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
    'InitialConfiguration', q0, ...
    'ConfigTools', configTools, ...
    'DistanceSpecs', distanceSpecs, ...
    'RateHz', 10, ...
    'ExecutionMode', 'ppForIk');
```

**特点：**
- ⚙️ 核心逻辑：实现三阶段分解
- ⚙️ 无 I/O：不创建目录、不保存文件
- ⚙️ 参数复杂：提供 50+ 个配置项
- ⚙️ 纯计算：仅返回日志结构体

---

#### 应该使用哪个默认值？

**✅ 好消息：默认值已统一！**

两者均采用生产级参数：
- **MaxIterations = 1500**
- **StageBMode = "pureHyb"**
- **RateHz = 10 Hz**
- **UseStageBHybridAStar = true**

| 使用场景 | 推荐函数 | 额外设置 | 理由 |
|----------|----------|----------|------|
| **生产/部署** | 任意函数（默认） | 无需额外配置 | 默认即为生产设置 |
| **快速测试** | `runStagedReference` | 使用默认值 | 自动保存更便捷 |
| **自定义环境** | `trackReferenceTrajectory` | 指定 EnvironmentConfig | 灵活度更高 |
| **提高控制频率** | 任意函数 | 设置 RateHz=100 | 满足特定研究需求 |
| **保持一致性** | 任意函数 | 默认已匹配 | 无需额外覆写 |

> 注：速度类参数虽有小差异，但对总体行为影响有限，必要时可覆写。

---

#### 三层架构概览

```
┌─────────────────────────────────────────────────────────────┐
│  第三层：用户便捷层                                         │
│  runStagedReference()                                       │
│  - 加载默认环境                                             │
│  - 自动保存至时间戳目录                                    │
│  - 返回包含路径的结果结构体                                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  第二层：轨迹处理层                                         │
│  trackReferenceTrajectory('Mode','staged')                  │
│  - 加载 JSON 轨迹                                           │
│  - 构建机器人模型 & 配置                                    │
│  - 创建距离约束                                             │
│  - 调用 runStagedTrajectory()                               │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  第一层：执行引擎层                                         │
│  runStagedTrajectory()                                      │
│  - Stage A/B/C 执行                                          │
│  - 合并日志                                                  │
│  - 返回 pipeline 结构体                                     │
└─────────────────────────────────────────────────────────────┘
```

**总结：**
- 对用户脚本而言，应调用 **`runStagedReference`**
- 内部工具函数（如 `trackReferenceTrajectory`）仍可直接调用核心引擎

---

#### 何时选择哪个函数

| 目标 | 推荐函数 | 原因 |
|------|----------|------|
| **快速测试/研究** | `runStagedReference` | 自动保存，几乎无需配置 |
| **参数扫描** | `runStagedReference` | 自动创建结果目录 |
| **自定义流水线** | `trackReferenceTrajectory` | 可全面控制模式与环境 |
| **模式对比** | `trackReferenceTrajectory` | 可切换整体/分阶段 |
| **直接引擎调用** | ~~`runStagedTrajectory`~~ | ❌ 几乎不直接调用 |
| **调试各阶段** | `trackReferenceTrajectory` | 可访问各阶段日志 |

---

#### 常见疑惑

- **核心算法是否相同？**  
  ✅ 是。`runStagedReference` 仅是调用 `trackReferenceTrajectory('Mode','staged')`；Stage A/B/C 逻辑均在 `runStagedTrajectory` 中实现；**现在默认参数也统一了**。

- **默认情况下输出是否相同？**  
  ✅ 自 2025-10-11 起完全一致。两者默认均为：MaxIterations=1500、StageBMode="pureHyb"、RateHz=10、UseStageBHybridAStar=true。剩余轻微的速度参数差异不会显著影响行为，可按需覆写。

- **为何存在两个名称相似的函数？**  
  - `runStagedReference`：高层工作流，负责便捷性与 I/O  
  - `runStagedTrajectory`：低层核心计算引擎  
  二者各司其职——一个关注流程与数据保存，另一个专注纯计算。

- **脚本应调用哪个？**  
  - **普通用户：**几乎总是调用 `runStagedReference`  
  - **框架内部：**由 `trackReferenceTrajectory` 调用 `runStagedTrajectory`  
  - **直接调用引擎：**除非修改核心逻辑，否则不要直接调用 `runStagedTrajectory`

- **三阶段执行逻辑在哪里？**  
  - 全部在 `runStagedTrajectory`（约 1986 行）  
  - `runStagedReference` 只是 132 行的轻量封装

- **可否跳过 runStagedReference？**  
  - 可以，直接调用 `trackReferenceTrajectory` 以获得更高灵活性  
  - 但需自行保存日志、管理环境配置与元数据  
  - 现在默认一致，任意方式均能得到一致结果，取舍在于便利 vs 灵活

---

#### 三种调用方式示例

```matlab
% ============================================================
% 方法一：高层便捷方式（推荐给用户）
% ============================================================
result = gik9dof.runStagedReference( ...
    'ExecutionMode', 'ppForIk', ...
    'MaxIterations', 150, ...
    'StageBMode', 'pureHyb', ...
    'RunLabel', 'my_test');

% result 包含：
%   result.log           - 完整流水线日志
%   result.logPath       - 'results/20251011_123456_my_test/log_staged_ppForIk.mat'
%   result.resultsDir    - 'results/20251011_123456_my_test/'
%   result.environment   - 实际使用的环境配置
%   result.executionMode - 'ppForIk'
%   result.options       - 所有使用的参数

% ============================================================
% 方法二：中层调用，支持自定义环境
% ============================================================
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.25;        % 自定义安全裕度
env.FloorDiscs(3).Radius = 0.30;  % 调整第三个障碍物

log = gik9dof.trackReferenceTrajectory( ...
    'Mode', 'staged', ...
    'ExecutionMode', 'ppForIk', ...
    'EnvironmentConfig', env, ...
    'MaxIterations', 150, ...
    'RateHz', 10);

% 需要自行保存：
resultsDir = sprintf('results/custom_%s', datestr(now, 'yyyymmdd_HHMMSS'));
mkdir(resultsDir);
save(fullfile(resultsDir, 'log_staged_custom.mat'), 'log');

% ============================================================
% 方法三：低层直接调用（少数专家场景）
% ============================================================
robot = gik9dof.createRobotModel();
trajStruct = gik9dof.loadTrajectory('1_pull_world_scaled.json');
configTools = gik9dof.configurationTools(robot);
q0 = configTools.column(configTools.homeConfig());

pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
    'InitialConfiguration', q0, ...
    'ConfigTools', configTools, ...
    'DistanceSpecs', [], ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'MaxIterations', 150);

% pipeline.qTraj        - [9×N] 全程轨迹
% pipeline.stageLogs.*  - 各阶段日志
% 保存、目录管理等均需手动处理
```

---

### 高层调用层次结构

```
USER
 │
 ├─ run_staged_reference.m ─────────────┐
 ├─ run_environment_compare.m ──────────┤
 ├─ run_fresh_sim_with_animation.m ─────┤
 └─ run_parametric_study.m ─────────────┤
                                        │
                                        ▼
                    trackReferenceTrajectory.m
                    （模式路由：整体 vs 分阶段）
                         │
                         ├─ holistic ───────────────┐
                         │                          │
                         └─ staged ──────────────┐  │
                                                 │  │
                ┌────────────────────────────────┘  │
                ▼                                   ▼
        runStagedTrajectory.m              runTrajectoryControl.m
                │                                   │
    ┌───────────┼───────────┐                      │
    ▼           ▼           ▼                      │
  Stage A    Stage B     Stage C                   │
    │           │           │                      │
    └───────────┴───────────┴──────────────────────┘
                            │
                            ▼
                    createGikSolver.m
                            │
                            ▼
              generalizedInverseKinematics
                  （MATLAB 工具箱）
```

### 分阶段模式详细调用图

```
run_staged_reference.m
│
├─> gik9dof.environmentConfig()
│   └─ 返回：底盘基准位姿、障碍、距离裕度
│
├─> gik9dof.control.loadChassisProfile()
│   ├─ 读取：config/chassis_profiles.yaml
│   └─ 返回：底盘参数结构体
│
└─> gik9dof.trackReferenceTrajectory('Mode','staged', ...)
    │
    ├─> gik9dof.internal.resolvePath()
    │   └─ 解析：1_pull_world_scaled.json
    │
    ├─> jsondecode(fileread(jsonPath))
    │   └─ 加载：148 个末端执行器航点
    │
    ├─> gik9dof.createRobotModel()
    │   ├─ importrobot() 解析 URDF
    │   └─ 返回：rigidBodyTree + 足迹信息
    │
    ├─> gik9dof.configurationTools(robot)
    │   └─ 返回：配置转换工具
    │
    ├─> gik9dof.addFloorDiscs(robot, discs)
    │   └─ 向树中添加障碍体
    │
    ├─> gik9dof.collisionTools(robot)
    │   └─ 附加 meshes/ 下的 STL
    │
    └─> gik9dof.runStagedTrajectory(robot, trajStruct, ...)
        │
        ├─ Stage A：手臂渐进
        │   ├─ generateStagePoses(T0, T1, 50, 'arm')
        │   ├─ gik9dof.createGikSolver(robot, ...)
        │   │   └─ generalizedInverseKinematics()
        │   └─ gik9dof.runTrajectoryControl(bundleA, trajA, ...)
        │       └─ 50 次迭代 → logA
        │
        ├─ Stage B：底盘导航
        │   ├─ 若 pureHyb：
        │   │   ├─ plannerPlanStateSpace() [Hybrid A*]
        │   │   ├─ gik9dof.control.rsRefinePath() [可选]
        │   │   ├─ gik9dof.control.rsClothoidRefine() [可选]
        │   │   ├─ gik9dof.control.preparePathForFollower()
        │   │   ├─ gik9dof.control.simulateChassisExecution()
        │   │   │   ├─ gik9dof.control.unifiedChassisCtrl() 循环
        │   │   │   └─ 返回 simRes（姿态、指令）
        │   │   └─ buildSyntheticStageBLog() → logB
        │   └─ 若 gikInLoop：
        │       ├─ interpolateBaseStates()
        │       ├─ gik9dof.createGikSolver()
        │       └─ gik9dof.runTrajectoryControl(bundleB, trajB, ...) → logB
        │
        └─ Stage C：全身跟踪
            ├─ 若 ppForIk：
            │   ├─ gik9dof.createGikSolver() → bundleRef
            │   ├─ gik9dof.runTrajectoryControl() → logRef
            │   ├─ stageCApplyBaseRefinement() [可选]
            │   │   ├─ gik9dof.control.rsRefinePath()
            │   │   └─ gik9dof.control.rsClothoidRefine()
            │   ├─ gik9dof.control.simulateChassisExecution()
            │   ├─ gik9dof.createGikSolver() → bundleFinal
            │   └─ gik9dof.runTrajectoryControl(bundleFinal, ...)（固定底盘轨迹）
            └─ 若 pureIk：
                ├─ gik9dof.createGikSolver() → bundleC
                └─ gik9dof.runTrajectoryControl(bundleC, trajC, ...)
```

### 动画生成调用图

```
USER（任意启用动画的入口脚本）
│
├─> gik9dof.saveRunArtifacts()
│   └─ 生成图像与动画
│
或
│
└─> regenerate_animations_from_logs.m
    │
    └─ 对每个日志文件：
        ├─> load('log_staged_*.mat')
        └─> gik9dof.animateStagedWithHelper(log, ...)
            ├─ 提取 qTraj、时间戳
            ├─ 采样轨迹（SampleStep）
            ├─ 提取参考路径
            └─> gik9dof.viz.animate_whole_body(robot, ...)
                ├─ ensureArmVisuals() → 加载/附加 STL
                ├─ interpolateChassisToArm() → 同步底盘/手臂
                ├─ setupDualViewFigure() → 创建正视/俯视图
                └─ 动画循环：
                    ├─ 设置姿态 → show(robot)
                    ├─ 绘制障碍
                    ├─ 绘制参考路径
                    ├─ 绘制 Stage 边界
                    └─ 可选保存为 MP4
```

---
## 预处理与配置系统

### 1. 参考轨迹（JSON）

**文件：** `1_pull_world_scaled.json`

**结构：**
```json
{
  "poses": [
    {
      "position": [x, y, z],
      "orientation": [qx, qy, qz, qw]
    },
    ...
  ]
}
```

**用途：**
- 由 `trackReferenceTrajectory.m` 通过 `jsondecode()` 读取
- 转换为 SE(3) 矩阵（4×4×N）
- 共 148 个航点
- 表示世界坐标系下期望的末端执行器轨迹

**加载流程：**
```matlab
% 位于 trackReferenceTrajectory.m
jsonPath = gik9dof.internal.resolvePath("1_pull_world_scaled.json");
raw = jsondecode(fileread(jsonPath));
% 将四元数 [qx,qy,qz,qw] 转换为 MATLAB 使用的 [qw,qx,qy,qz]
trajStruct = loadJsonTrajectory(jsonPath);
```

### 2. 底盘配置（YAML）

**文件：** `config/chassis_profiles.yaml`

**结构：**
```yaml
profiles:
  wide_track:
    track_width: 0.574
    wheel_base: 0.36
    wheel_radius: 0.076
    vx_max: 1.5
    vx_min: -0.4
    wz_max: 2.0
    wheel_speed_max: 3.3
    lookahead_base: 0.8
    lookahead_vel_gain: 0.2
    lookahead_time_gain: 0.05
    heading_kp: 1.2
    feedforward_gain: 0.9
    accel_limit: 0.8
    waypoint_spacing: 0.15
    interp_spacing_min: 0.05
    interp_spacing_max: 0.15
    goal_tolerance: 0.05
    reverse_enabled: true
    stageB_controller_mode: 2
    stageC_controller_mode: 2
```

**加载流程：**
```matlab
% 在 trackReferenceTrajectory.m 或 runStagedReference.m 内
chassisParams = gik9dof.control.loadChassisProfile("wide_track", ...
    "Overrides", struct('accel_limit', 0.6, 'lookahead_base', 1.0));
```

**覆写机制：**
- 先从 YAML 读取基础配置
- 按字段应用覆写值
- 将最终结构体传入控制器

### 3. 环境配置

**文件：** `+gik9dof/environmentConfig.m`（函数）

**返回：**
```matlab
config = struct(
    'BaseHome', [-2.0, -2.0, 0.0],
    'FloorDiscs', [
        struct('x', -1.0, 'y', 0.5, 'r', 0.4, 'margin', 0.05)
        struct('x', 0.5, 'y', -0.8, 'r', 0.3, 'margin', 0.05)
    ],
    'DistanceMargin', 0.10,
    'DistanceWeight', 5.0,
    'StageBMode', 'pureHyb',
    'StageBDockingPositionTolerance', 0.02,
    'StageBDockingYawTolerance', 2*pi/180
);
```

**用途：**
```matlab
% 入口脚本示例
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.15;  % 如有需要可以覆写

% 传递给 trackReferenceTrajectory
log = gik9dof.trackReferenceTrajectory(...
    'EnvironmentConfig', env, ...
);
```

### 4. 机器人模型（URDF）

**文件：**
- `mobile_manipulator_PPR_base_corrected.urdf`（主模型）
- `mobile_manipulator_PPR_base_corrected_sltRdcd.urdf`（精简网格）

**加载：**
```matlab
% 位于 createRobotModel.m
urdfPath = gik9dof.internal.resolvePath('mobile_manipulator_PPR_base_corrected.urdf');
robot = importrobot(urdfPath, 'DataFormat', 'column');
```

**结构特点：**
- 底盘：3 个平面自由度（x、y、theta）
- 左臂：6 个转动关节
- 总自由度：9
- 末端执行器：left_gripper_link

**碰撞几何：**
- 可视网格来自 `meshes/` 目录
- 通过 `collisionTools.apply()` 附着碰撞体

### 5. 求解器配置

**创建者：** `createGikSolver.m`

**关键参数：**
```matlab
bundle = gik9dof.createGikSolver(robot, ...
    'MaxIterations', 150,
    'EnableAiming', false,
    'DistanceSpecs', distanceSpecs,
    'DistanceWeight', 5.0
);
```

**约束组成：**
1. **姿态约束** —— 末端执行器的 SE(3) 目标
2. **关节界限** —— 来自 URDF 的位置限制
3. **距离约束** —— 避障（可选）
4. **瞄准约束** —— 可选的相机指向

**默认权重：**
- 姿态：100（位置 + 姿态）
- 关节界限：10
- 距离：5–20（视目标而定）

**求解器组合特性：**
- 暴露 `updatePrimaryTarget`、`setDistanceBounds` 等闭包
- `solve` 封装会在求解前刷新约束目标
- 与 `configurationTools` 协作完成列向量/结构体转换
- 支持按航点设置瞄准与距离约束

**避障策略：**
- **地面圆盘：** 在整体模式与 Stage C 中通过 `constraintDistanceBounds` 强制执行
- **Hybrid A\*：** Stage B 利用障碍膨胀后的占据网格规划
- **Stage A：** 底盘锁定，不使用距离约束
- **Stage B（gikInLoop）：** 手臂锁定，不使用距离约束
- **自碰撞：** 目前未启用（已有网格，但未加入约束）

> 重要说明：Stage A 与 Stage B（gikInLoop 模式）因关节被锁定，不会应用距离约束；整体模式与 Stage C 才会持续执行避障约束。

### 6. 统一的参数配置系统

**动机：** 过去流水线参数分布在多个位置：
- `config/chassis_profiles.yaml` —— 底盘参数
- `runStagedTrajectory.m` —— 函数参数默认值
- `runStagedReference.m` —— 另一套默认值
- `trackReferenceTrajectory.m` —— 又一套默认值

这导致不一致与维护负担。统一配置系统将所有参数集中在一个可信位置。

#### 文件结构

**文件：** `config/pipeline_profiles.yaml`

**可用配置档：**
1. **default** —— 平衡的通用参数
2. **aggressive** —— 更快的动作（继承 default）
3. **conservative** —— 更保守的安全参数（继承 default）
4. **compact_track** —— 更窄履带的对应调整

**结构示例：**
```yaml
profiles:
  default:
    chassis:
      track: 0.574
      wheelbase: 0.36
      wheel_radius: 0.076
      vx_max: 1.5
      wz_max: 2.0
      accel_limit: 0.8
      # ...
    stage_b:
      mode: "pureHyb"
      lookahead_distance: 0.6
      desired_linear_velocity: 0.6
      max_angular_velocity: 2.0
      hybrid_safety_margin: 0.1
      hybrid_resolution: 0.05
      motion_primitive_length: 0.20
      reeds_shepp:
        shortcut_enabled: true
        max_step: 0.05
      # ...
    stage_c:
      track_width: 0.574
      use_base_refinement: true
      controller_mode: 2
    gik:
      max_iterations: 150
      enable_aiming: false
      distance_weight: 5.0
    pure_pursuit:
      lookahead_base: 0.8
      lookahead_vel_gain: 0.2
    holistic:
      lookahead_distance: 0.6
      desired_linear_velocity: 0.5

  aggressive:
    inherits: "default"
    overrides:
      chassis: {vx_max: 1.8, accel_limit: 1.2}
      stage_b: {desired_linear_velocity: 0.8}
      holistic: {desired_linear_velocity: 0.7}

  conservative:
    inherits: "default"
    overrides:
      chassis: {vx_max: 1.0, accel_limit: 0.5}
      stage_b: {desired_linear_velocity: 0.4}
```

#### 加载函数

**文件：** `matlab/+gik9dof/loadPipelineProfile.m`

**用法：**
```matlab
cfg = gik9dof.loadPipelineProfile('default');

cfg = gik9dof.loadPipelineProfile('aggressive', ...
    'stage_b', struct('desired_linear_velocity', 0.9), ...
    'chassis', struct('accel_limit', 1.5));
```

**特性：**
- **继承解析：** 配置档可以继承自其他配置档
- **深度合并：** 覆写会递归合并
- **一致性校验：** 例如确保 `stage_c.track_width == chassis.track`
- **YAML 回退：** 若无 `yamlread()`，使用自定义解析器
- **元数据：** 返回 `config.meta.profile`、`config.meta.validationWarnings`

**返回结构：**
```matlab
cfg = struct(
    'chassis', struct(...),
    'stage_b', struct(...),
    'stage_c', struct(...),
    'gik', struct(...),
    'pure_pursuit', struct(...),
    'holistic', struct(...),
    'meta', struct('profile', 'aggressive', 'validationWarnings', {})
);
```

#### 与主函数的整合

三大核心函数都支持 `PipelineConfig`：

1. **trackReferenceTrajectory.m**
```matlab
cfg = gik9dof.loadPipelineProfile('aggressive');
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);

log = gik9dof.trackReferenceTrajectory('MaxIterations', 200);
```

2. **runStagedTrajectory.m**
```matlab
cfg = gik9dof.loadPipelineProfile('default');
log = gik9dof.runStagedTrajectory(robot, 'PipelineConfig', cfg);

log = gik9dof.runStagedTrajectory(robot, 'StageBDesiredLinearVelocity', 0.7);
```

3. **runStagedReference.m**
```matlab
cfg = gik9dof.loadPipelineProfile('conservative');
result = gik9dof.runStagedReference('PipelineConfig', cfg);

result = gik9dof.runStagedReference('StageBMode', 'gikInLoop');
```

#### 优势

1. **单一事实来源：** 所有参数集中于 `pipeline_profiles.yaml`
2. **一致性：** 校验可确保跨阶段参数匹配
3. **配置管理：** 可轻松切换 aggressive/conservative/default
4. **易维护：** 修改一次即可影响全部函数
5. **向后兼容：** 仍支持逐项传参的旧代码
6. **继承复用：** 支持基于已有配置扩展（DRY）
7. **履带宽统一：** 全部修正为 0.574 m（曾出现 0.674/0.576/0.573）

#### 参数不一致问题已修复

| 参数 | 旧 runStagedTrajectory | 旧 runStagedReference | 统一后 |
|------|------------------------|------------------------|--------|
| StageBDesiredLinearVelocity | 0.6 m/s | 0.5 m/s | 0.6 m/s |
| StageBHybridSafetyMargin | 0.1 m | 0.15 m | 0.1 m |
| StageBMode | "gikInLoop" | "pureHyb" | "pureHyb" |
| Track Width | 0.674/0.576 | 0.573 | 0.574 m |

#### 迁移指南

**步骤 1：** 在 `pipeline_profiles.yaml` 中创建或修改配置档
```yaml
profiles:
  my_custom:
    inherits: "default"
    overrides:
      stage_b: {desired_linear_velocity: 0.75}
```

**步骤 2：** 在脚本中加载配置
```matlab
cfg = gik9dof.loadPipelineProfile('my_custom');
```

**步骤 3：** 传入流水线函数
```matlab
log = gik9dof.trackReferenceTrajectory('PipelineConfig', cfg);
result = gik9dof.runStagedReference('PipelineConfig', cfg);
```

**步骤 4（可选）：** 额外覆写某些参数
```matlab
cfg = gik9dof.loadPipelineProfile('default', ...
    'stage_b', struct('desired_linear_velocity', 0.9));
```

---

## 实时仿真流水线

### Stage A：手臂渐进

**目的：** 在保持底盘静止的情况下，将手臂对齐到序列第一个航点。

**数据流：**
```
Input:
  q0           [9×1]  初始配置
  T1           [4×4]  第一个末端执行器目标姿态
  
Process:
  1. T0 = getTransform(robot, q0, 'left_gripper_link')
  2. 生成 50 个插值姿态：T0 → T1（包含姿态与 z 高度）
  3. 在 GIK 求解器中锁定底盘关节（上下界 = q0）
  4. 对 50 个航点运行 IK 循环
  
Output:
  logA.qTraj         [9×50]  底盘不动、手臂运动的关节轨迹
  logA.eePositions   [3×50]  实际末端执行器位置
  logA.timestamps    [1×50]  时间戳
  qA_end             [9×1]   阶段结束配置
```

**关键函数：** `lockJointBounds(bundleA.constraints.joint, baseIdx, q0)`

### Stage B：底盘导航

#### 模式 1：pureHyb（Hybrid A* + 纯跟踪）

**目的：** 仅移动底盘至停泊姿态，同时保持手臂锁定。

**数据流：**
```
Input:
  qA_end             [9×1]  初始配置
  T1                 [4×4]  首个末端执行器目标（决定目标底盘位姿）
  options            规划参数结构体
  
Process:
  1. 通过逆向运动学计算目标底盘位姿
  2. 路径规划：
     a. 基于地面圆盘生成占据栅格
     b. 运行 Hybrid A* 规划器
     c. （可选）Reeds-Shepp 捷径
     d. （可选）Clothoid 平滑
  3. 控制器仿真：
     a. 预处理路径（插值、重采样）
     b. 循环运行 unifiedChassisCtrl
     c. 生成 (Vx, Wz) 指令
     d. 积分得到底盘轨迹
  4. 构造合成日志：
     a. 生成手臂锁定、底盘移动的 qTraj
     b. 通过 FK 获得末端执行器姿态
     c. 生成时间戳
  5. 计算诊断信息：
     a. 底盘带状指标（曲率、回转）
     b. RS 平滑统计
     c. Clothoid 拟合结果
  
Output:
  logB.qTraj         [9×N]  底盘运动、手臂锁定
  logB.pathStates    [N×3]  (x, y, theta) 执行路径
  logB.cmdLog        Table  (time, Vx, Wz) 指令
  logB.purePursuit   Struct 控制器仿真数据
  logB.planner       Struct 规划元数据
  logB.diagnostics   Struct 诊断指标
  qB_end             [9×1]  最终停泊配置
```

**规划链路：**
```
plannerPlanStateSpace() → Hybrid A* path
    ↓
rsRefinePath() → Reeds-Shepp shortcuts (optional)
    ↓
rsClothoidRefine() → Clothoid smoothing (optional)
    ↓
preparePathForFollower() → Interpolation & resampling
    ↓
simulateChassisExecution() → Pure pursuit execution
```

#### 模式 2：gikInLoop（GIK 驱动底盘）

**目的：** 使用 IK 求解器移动底盘，同时保持手臂锁定。

**数据流：**
```
Input:
  qA_end       [9×1]  初始配置
  goalBase     [1×3]  目标底盘姿态
  
Process:
  1. 插值直线底盘路径（50 个样本）
  2. 在 GIK 求解器中锁定手臂关节
  3. 运行 IK 循环施加底盘位姿约束
  
Output:
  logB.qTraj   [9×50]  IK 生成的底盘轨迹
  qB_end       [9×1]   阶段结束配置
```

### Stage C：全身跟踪

#### 模式 1：ppForIk（Pure Pursuit for IK）

**目的：** 在底盘控制器驱动下跟踪剩余航点。

**数据流：**
```
Input:
  qB_end          [9×1]  起始配置（停泊完成）
  trajStruct      Struct  剩余航点
  
Process:
  PASS 1 - 生成参考路径
    1. 创建 bundleRef（标准 GIK 求解器）
    2. 运行 runTrajectoryControl() → logRef
    3. 提取底盘轨迹：baseReference = logRef.qTraj(baseIdx, :)
  
  PASS 1.5 - 可选底盘精修
    若启用 StageCUseBaseRefinement：
      a. 对 baseReference 运行 RS 捷径
      b. 运行 Clothoid 平滑
      c. 用平滑结果更新 baseReference
  
  PASS 2 - 控制器仿真
    1. 准备底盘路径
    2. 运行 simulateChassisExecution()
       → 得到底盘执行轨迹
  
  PASS 3 - 底盘锁定的最终 IK
    1. 创建 bundleFinal（GIK 求解器）
    2. 运行 runTrajectoryControl()，设置：
       FixedJointTrajectory.Indices = baseIdx
       FixedJointTrajectory.Values = executed base trajectory
    3. IK 仅求解手臂，底盘按照执行轨迹运动
  
Output:
  logC.qTraj                [9×N]  全程轨迹
  logC.referenceInitialIk   logRef 参考遍日志
  logC.purePursuit          Struct 控制器仿真结果
  logC.execBaseStates       [N×3]  底盘执行路径
  logC.referenceBaseStates  [N×3]  底盘参考路径
  logC.cmdLog               Table  (time, Vx, Wz) 指令
  logC.diagnostics          Struct 诊断指标
```

**三遍架构概览：**
```
Pass 1: GIK Reference
  runTrajectoryControl(bundleRef, trajC, ...)
  → Full IK solution (reference base path)

Pass 1.5: Base Smoothing (optional)
  rsRefinePath() + rsClothoidRefine()
  → Smoother base path

Pass 2: Chassis Simulation
  simulateChassisExecution(baseReference, ...)
  → Realistic executed base path

Pass 3: GIK with Fixed Base
  runTrajectoryControl(bundleFinal, trajC, ...
      'FixedJointTrajectory', executedBase)
  → Arm IK with actual base motion
```

#### 模式 2：pureIk（纯 IK）

**目的：** 不进行控制器仿真的标准全身 IK。

**数据流：**
```
Input:
  qB_end      [9×1]  起始配置
  trajC       Struct 剩余航点
  
Process:
  1. 创建 bundleC（标准 GIK 求解器）
  2. 运行 runTrajectoryControl()
     → 对所有航点执行全身 IK
  
Output:
  logC.qTraj  [9×N]  完整轨迹
```

### 日志合并

**函数：** `mergeStageLogs(logA, logB, logC)`

**流程：**
```matlab
pipeline.qTraj = [logA.qTraj, logB.qTraj(:,2:end), logC.qTraj(:,2:end)];

offsetB = logA.timestamps(end);
offsetC = offsetB + logB.timestamps(end);
pipeline.timestamps = [logA.timestamps, ...
                       logB.timestamps + offsetB, ...
                       logC.timestamps + offsetC];

pipeline.stageLogs.stageA = logA;
pipeline.stageLogs.stageB = logB;
pipeline.stageLogs.stageC = logC;

pipeline.stageBoundaries = [length(logA.timestamps), ...
                            length(logA.timestamps) + length(logB.timestamps)];
```

---

## 整体模式：全身 IK 流水线

### 概览

**整体模式（holistic）** 在整个轨迹过程中同时控制底盘与手臂，不再拆分阶段，机器人从起点直接运动到终点。

**与分阶段模式的主要差异：**
- 没有人工阶段边界
- 底盘与手臂始终同步运动
- 可选平滑斜坡（Ramp）用于柔性起步
- 流水线更简单
- 持续执行避障约束

### 整体模式架构

```
trackReferenceTrajectory('Mode', 'holistic', ...)
    │
    ├─ 加载 JSON 轨迹（148 个航点）
    ├─ 创建机器人模型
    ├─ 配置环境（障碍与约束）
    │
    ├─ （可选）生成 Ramp 轨迹
    │   └─ generateHolisticRamp() → 平滑加速
    │
    └─ 根据执行模式：
        │
        ├─ ppForIk 模式：
        │   ├─ Pass 1：参考 IK
        │   ├─ Pass 2：底盘控制器仿真
        │   └─ Pass 3：底盘锁定的最终 IK
        │
        └─ pureIk 模式：
            └─ 单遍全身 IK
```

### Holistic 模式：pureIk

**目的：** 直接执行全身 IK，不进行控制器仿真。

**数据流：**
```
Input:
  q0           [9×1]  初始配置
  trajStruct   Struct 全部 148 个航点
  velLimits    Struct 速度限制
  
Process:
  1. 创建 bundleSingle（标准 GIK 求解器）
     - 姿态约束
     - 关节界限
     - 距离约束
     
  2. 调用 runTrajectoryControl()
     - 逐航点运行 IK
     - 根据 velLimits 进行速度钳制
     - 记录迭代次数、求解时间与误差
     
  3. 底盘运动由 IK 结果自然产生
     - 无显式控制器
     - 可能不满足底盘动力学约束
  
Output:
  log.qTraj         [9×N]   完整关节轨迹
  log.eePositions   [3×N]   实际末端执行器位置（FK）
  log.targetPositions [3×N] 目标末端执行器位置
  log.positionError [3×N]   跟踪误差
  log.mode          'holistic'
  log.simulationMode 'pureIk'
  log.execBaseStates [N×3]  底盘轨迹（来自 qTraj）
  log.cmdLog        空表    （无控制器指令）
```

**特性：**
- **速度最快** —— 单趟 IK
- **流程最简单** —— 无底盘控制仿真
- **可能违反底盘约束** —— 未验证可行性
- **适用场景：** 快速测试、算法开发、生成理想轨迹

### Holistic 模式：ppForIk

**目的：** 使用底盘控制器生成符合动力学约束的可行底盘运动。

**三遍架构：**

#### Pass 1：生成参考底盘路径

```
Input:
  q0          [9×1]
  trajStruct  Struct
  
Process:
  1. 创建 bundleRef（标准 GIK 求解器）
  2. 运行 runTrajectoryControl() → logRef
  3. 提取底盘轨迹：baseReference = logRef.qTraj(baseIdx, :)'
  
Output:
  logRef.qTraj       [9×N]
  baseReference      [N×3]
```

#### Pass 2：底盘控制器仿真

```
Input:
  baseReference   [N×3]
  chassisParams   Struct
  
Process:
  1. 预处理路径（插值、重采样）
  2. 调用 simulateChassisExecution()
     - 运行统一底盘控制器
     - 产生 (Vx, Wz) 指令
     - 积分获得执行轨迹
     - 受到速度、加速度与轮速约束
  3. 验证执行结果（收敛、目标误差）
  
Output:
  simRes.poses      [N×3]
  simRes.commands   [N×2]
  simRes.wheelSpeeds [N×2]
  simRes.status     [N×1]
```

#### Pass 3：底盘锁定的最终 IK

```
Input:
  baseExecutedFull  [N×3]
  trajStruct        Struct
  
Process:
  1. 将执行轨迹重采样到与航点相同长度
  2. 创建 bundleFinal（GIK 求解器）
  3. 运行 runTrajectoryControl()，固定：
     FixedJointTrajectory.Indices = baseIdx
     FixedJointTrajectory.Values = executedBase
  4. IK 仅求解手臂，底盘沿执行轨迹行进
  
Output:
  log.qTraj                [9×N]
  log.referenceInitialIk   logRef
  log.purePursuit          simRes
  log.execBaseStates       [N×3]
  log.referenceBaseStates  [N×3]
  log.cmdLog               Table
```

**为何需要三遍？**
1. **Pass 1** —— 得到理想底盘轨迹（不一定可行）
2. **Pass 2** —— 在底盘动力学约束下生成实际轨迹
3. **Pass 3** —— 在实际底盘运动下重新求解手臂 IK

**特性：**
- **可行性有保证**
- **仿真与实机一致**
- **执行时间较长**
- **适用场景：** 验证、部署准备、真实性能评估

---

### 🔄 关键等价性：Holistic ppForIk ≡ 分阶段 Stage C (ppForIk)

**重点：** 整体模式 `ppForIk` 与分阶段模式 Stage C (`ppForIk`) 在全身跟踪阶段的算法完全相同，仅起始状态与轨迹长度不同。

#### 相同的三遍架构

```
Holistic Pass 1 → runTrajectoryControl(bundleRef, trajStruct, ...)
Staged   Pass 1 → runTrajectoryControl(bundleRef, trajC, ...)

Holistic Pass 2 → simulateChassisExecution(baseReference, ...)
Staged   Pass 2 → simulateChassisExecution(baseReference, ...)

Holistic Pass 3 → runTrajectoryControl(bundleFinal, ..., 'FixedJointTrajectory', executedBase)
Staged   Pass 3 → runTrajectoryControl(bundle, ..., 'FixedJointTrajectory', executedBase)
```

#### 差异仅在上下文

| 项目 | Holistic ppForIk | Stage C (ppForIk) |
|------|------------------|-------------------|
| 起始配置 | q0 | qB_end（Stage B 结果） |
| 航点集合 | 全部 148 个 | Stage B 之后的剩余航点 |
| 底盘参数 | `chassisHolistic` | `chassisStageC` |
| 算法 | **完全一致** | **完全一致** |

#### 参数映射

两者使用 `pipeline_profiles.yaml` 中的同名字段（默认已对齐）。建议保持 `stage_c` 与 `holistic` 参数一致，以确保比较公平。

---

### 可选整体 Ramp

**目的：** 在起步阶段平滑加速，避免骤然启动。

**生成函数：** `generateHolisticRamp(robot, bundle, q0, T1, options)`

**流程：**
```
Input:
  q0
  T1
  MaxLinearSpeed
  MaxYawRate
  MaxJointSpeed
  SampleTime
  
Algorithm:
  1. 计算所需位移与旋转
  2. 依据速度上限估算 ramp 时长
  3. 生成底盘与手臂插值轨迹
  4. 对 ramp 轨迹逐步运行 IK
  
Output:
  rampInfo.Poses
  rampInfo.EndEffectorPositions
  rampInfo.NumSteps
```

**调用示例：**
```matlab
log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'UseHolisticRamp', true, ...
    'RampMaxLinearSpeed', 1.5, ...
    'RampMaxYawRate', 3.0, ...
    'RampMaxJointSpeed', 1.0);
```

**日志字段：**
```matlab
log.ramp
log.rampSamples
```

### Holistic 模式：速度限制

**实现位置：** `runTrajectoryControl`

**速度限制结构体：**
```matlab
velLimits = struct(
    'BaseIndices',     [1 2 3],
    'ArmIndices',      [4 5 6 7 8 9],
    'MaxLinearSpeed',  1.5,
    'MaxYawRate',      3.0,
    'MaxJointSpeed',   1.0
);
```

**钳制过程（摘要）：**
```matlab
for step k
    1. 计算速度增量
    2. 若线速度超限 → 缩放平面位移
    3. 若角速度超限 → 限制 θ 增量
    4. 若关节速度超限 → 限制对应关节
    5. 接受钳制后的解
end
```

> 注意：速度限制在 IK 求解完成后应用，可能略微增大跟踪误差。

### Holistic 与分阶段对比

| 项目 | Holistic pureIk | Holistic ppForIk | Staged ppForIk |
|------|-----------------|------------------|----------------|
| IK 次数 | 1 | 3 | 3（每阶段） |
| 底盘运动来源 | IK 自主生成 | 控制器仿真 | 规划 + 控制器 |
| 可行性 | 不保证 | 保证 | 保证 |
| 避障约束 | 全程 | 全程 | Stage C 执行（Stage A/B 视模式而定） |
| 执行时间 | 最快 | 中等 | 最慢 |
| 跟踪质量 | 良好 | 优秀 | 优秀 |
| 底盘平滑度 | 可能有转折 | 光滑可行 | 规划最优 |
| 复杂度 | 最低 | 中等 | 最高 |
| 典型场景 | 快速开发 | 验证测试 | 部署准备 |

### Holistic 模式：数据流图

```
（图略：与英文原文相同）
```

### Holistic 模式：日志结构

```matlab
log_holistic = struct(
    'mode', 'holistic',
    'simulationMode', 'pureIk' 或 'ppForIk',
    'qTraj', [9×N],
    'timestamps', [1×N],
    'successMask', [1×N],
    'eePositions', [3×N],
    'targetPositions', [3×N],
    'eePoses', [4×4×N],
    'positionError', [3×N],
    'positionErrorNorm', [1×N],
    'solutionInfo', {1×N},
    'iterations', [1×N],
    'solveTime', [1×N],
    'solverSummary', struct(...),
    'baseVelocityEstimate', struct(...),
    'velocityLimits', struct(...),
    'ramp', struct(...),
    'rampSamples', double,
    'referenceInitialIk', logRef,
    'purePursuit', struct(...),
    'execBaseStates', [N×3],
    'referenceBaseStates', [N×3],
    'cmdLog', table(...),
    'floorDiscs', struct(...),
    'distanceSpecs', struct(...),
    'environment', struct(...),
    'chassisParams', struct(...),
    'chassisProfile', string
);
```

### Holistic 模式：入口

```matlab
log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'UseHolisticRamp', true, ...
    'RampMaxLinearSpeed', 1.5, ...
    'RampMaxYawRate', 3.0, ...
    'RampMaxJointSpeed', 1.0);
```

```matlab
env = gik9dof.environmentConfig();
env.DistanceMargin = 0.20;

log = gik9dof.trackReferenceTrajectory(...
    'Mode', 'holistic', ...
    'ExecutionMode', 'ppForIk', ...
    'EnvironmentConfig', env, ...
    'MaxIterations', 150, ...
    'ChassisProfile', 'wide_track');
```

### Holistic 模式：性能特性

- **pureIk 模式：**
  - 单遍 IK，耗时约 5–10 s
  - 每步求解时间 ~0.03–0.05 s（150 次迭代上限）
  - EE 平均误差 <5 mm，最大误差 <20 mm
  - 无控制器开销，但底盘可行性不保证
- **ppForIk 模式：**
  - Pass 1：~5–10 s
  - Pass 2：~2–5 s
  - Pass 3：~5–10 s
  - 总计 ~12–25 s
  - EE 平均误差 <8 mm，最大误差 <30 mm
  - 底盘运动可行，与实机一致
- **启用 Ramp：**
  - 额外增加 50–100 个步长
  - 执行时间增加 ~1–2 s
  - 起步更平滑，更贴近实机

---

## 模式选择指南

### 何时使用分阶段模式
- **优势：** 底盘路径可规划、避障能力强、轨迹平滑、贴近实际部署流程
- **劣势：** 执行最慢、流程最复杂、需精细调参
- **适用：** 部署前验证、复杂障碍环境、需要高质量底盘路径的任务

### 何时使用 Holistic pureIk
- **优势：** 执行最快、流程最简单、适合快速迭代
- **劣势：** 底盘可行性无法保证、可能超速
- **适用：** 算法研发、理想轨迹生成、调试 IK

### 何时使用 Holistic ppForIk
- **优势：** 底盘可行性有保证、包含控制器验证、跟踪优良
- **劣势：** 相对较慢、流程较复杂
- **适用：** 验证测试、预部署评估、控制器调参

### 总结对比

| 模式 | 阶段数 | IK 次数 | 底盘运动 | 可行性 | 速度 | 典型用途 |
|------|--------|---------|----------|--------|------|----------|
| **Staged ppForIk** | 3 | 9 | 规划 + 控制器 | 有保证 | 最慢 | 部署 |
| **Holistic ppForIk** | 1 | 3 | 控制器 | 有保证 | 中等 | 验证 |
| **Holistic pureIk** | 1 | 1 | IK 自然产生 | 无保证 | 最快 | 开发 |

---

## 基于日志的动画流水线

### 概览

动画生成方式：
1. **仿真过程中** —— 通过 `saveRunArtifacts()`
2. **仿真结束后** —— 通过 `regenerate_animations_from_logs.m`

### 数据流：日志 → 动画

```
Input:
  log_staged_ppForIk.mat（或 pureIk）

Load:
  log = load('results/.../log_staged_ppForIk.mat').log;

Extract:
  qTraj          [9×N]  全部关节轨迹
  stageLogs      Struct 各阶段日志
  timestamps     [1×N]  时间序列

Process:
  1. 按 SampleStep 采样轨迹
  2. 提取底盘轨迹：basePose = qTraj(baseIdx,:)'
  3. 提取手臂轨迹：armTrajectory = qTraj(armIdx,:)'
  4. 收集参考路径：
     a. Stage C 目标路径：stageC.targetPositions
     b. Stage C 实际路径：stageC.eePositions
     c. 纯跟踪参考：log.purePursuit.referencePath（若存在）
  5. 配置可视化选项：
     options.StageBoundaries = [NA, NA+NB]
     options.StageLabels = ["Stage A", "Stage B", "Stage C"]
     options.Obstacles = log.floorDiscs
     options.StageCReferenceEEPath = 已采样参考

Call:
  gik9dof.animateStagedWithHelper(log, ...
      'SampleStep', 5, ...
      'FrameRate', 20, ...
      'ExportVideo', 'output.mp4', ...
      'HelperOptions', options);

Animation:
  gik9dof.viz.animate_whole_body(...)
    每帧执行：
      1. 插值得到当前底盘姿态
      2. 设置机器人关节角
      3. 正运动学获取末端执行器姿态
      4. show(robot) 渲染
      5. 绘制障碍、参考路径、实际轨迹
      6. 更新阶段标签
      7. 捕获帧写入视频

Output:
  output.mp4
```

---

### 控制流程分析

#### Holistic 模式（ppForIk）
```
Pass 1: runTrajectoryControl (IK)
  → generates reference base path (ideal)
  
Pass 2: simulateChassisExecution
  ├─> preparePathForFollower (preprocessing)
  ├─> purePursuitFollower (controller)
  ├─> unifiedChassisCtrl (command generation) ✅ REUSES
  └─> kinematic integration
  → generates executed base path (realistic)
  
Pass 3: runTrajectoryControl (IK with fixed base)
  → generates final trajectory
```

#### 分阶段模式 —— Stage B（pureHyb）
```
Path Planning: Hybrid A* planner
  → generates waypoint path
  
Optional: rsRefinePath → rsClothoidRefine
  → smooths path

Controller Simulation: simulateChassisExecution
  ├─> preparePathForFollower
  ├─> purePursuitFollower
  ├─> unifiedChassisCtrl (command generation)
  └─> integrate commands → base path
  
Synthetic Log: buildSyntheticStageBLog
  ├─> IK with arm locked
  └─> Store metrics + diagnostics
```

#### 分阶段模式 —— Stage B（gikInLoop）
```
Path Interpolation: interpolateBaseStates (straight line)

IK Execution: runTrajectoryControl(bundleB, trajB, ...)
  ├─> Lock arm joints
  ├─> Solve IK per waypoint
  └─> No controller simulation
```

**关键观察：**
- `simulateChassisExecution` 只在 **pureHyb/p pForIk** 流程中使用
- `runTrajectoryControl` 负责 IK，与控制器仿真完全分离
- `unifiedChassisCtrl` 在两者之间 **复用** —— 既用于实时命令也用于仿真

---

### runTrajectoryControl vs simulateChassisExecution

| 函数 | 所属域 | 输入 | 输出 | 求解器 |
|------|--------|------|------|--------|
| **runTrajectoryControl** | **逆运动学** | 末端执行器姿态（SE(3)） | 关节角 qTraj | GIK（9 自由度 IK） |
| **simulateChassisExecution** | **控制器仿真** | 底盘航点 (x,y,θ) | 速度指令 (Vx,Wz) | 纯跟踪（3 自由度控制） |

**根本差异：**
1. **问题类型**：前者是 IK 求解循环；后者是控制器仿真
2. **自由度**：前者 9 个关节；后者仅 3 个底盘自由度
3. **输出**：前者输出关节配置；后者输出速度指令
4. **积分方式**：前者逐点求解无积分；后者对 (Vx, Wz) 积分得到姿态
5. **约束**：前者处理 IK 约束（姿态、距离），后者处理底盘动力学（速度、加速度）

**使用统计：**
- `runTrajectoryControl`：8 处调用（均为 IK 流程）
- `simulateChassisExecution`：5 处调用（均为控制器仿真）

**结论：** ✅ 二者互补，不存在功能重叠。在 Stage C 的三遍架构中二者协同工作。

---

### 冗余与冲突分析

#### ✅ 未发现冗余

逐一检查 11 个底盘相关函数均为单一职责、互不重叠，例如：
1. `unifiedChassisCtrl` —— 模式路由与统一命令输出  
2. `purePursuitFollower` —— 自适应前视的路径跟随器  
3. `simulateChassisExecution` —— 控制器闭环仿真  
4. `preparePathForFollower` —— 路径归一化与重采样  
5. `rsRefinePath` / `rsClothoidRefine` —— 两种不同的平滑算法  
6. `clampYawByWheelLimit` —— 差动驱动可行性检查  
7. `loadChassisProfile` —— YAML 参数加载  
8. `defaultUnifiedParams` / `defaultReedsSheppParams` —— 默认参数  
9. `simulatePurePursuitExecution` —— Mode 2 的轻量封装  

所有函数均无重复职责。

#### ✅ 未发现冲突

- **参数一致性**：统一使用 `track`、`vx_max`、`wz_max` 等命名；由 `loadChassisProfile()` 统一下发  
- **数据流顺畅**：配置 → 预处理 → 控制 → 指令 → 执行，无循环依赖  
- **模式处理**：`unifiedChassisCtrl` 对三种模式统一处理，无互斥实现  
- **履带宽度**：已全局标准化为 **0.574 m**  

---

### 与统一流水线配置的集成

当前配置系统包括：
- `loadChassisProfile()`：从 `chassis_profiles.yaml` 读取底盘参数（遗留支持）  
- `loadPipelineProfile()`：从 `pipeline_profiles.yaml` 读取底盘 + Stage B/C + GIK 的完整配置（推荐）

```
pipeline_profiles.yaml（推荐）
  ├─ chassis     → 底盘参数
  ├─ stage_b     → Stage B 参数
  ├─ stage_c     → Stage C 参数
  └─ gik         → GIK 求解器参数

chassis_profiles.yaml（遗留）
  └─ 底盘参数
```

**建议：** 保留两种加载方式以兼容旧脚本。新代码优先使用 `loadPipelineProfile()`。

---

### 文档交叉引用

- 本文：`projectDiagnosis.md` —— 系统架构与数据流  
- 设计说明：`docs/unified_chassis_controller_summary.md` —— 底盘控制系统详解

二者在以下方面保持一致：
- UnifiedCmd 数据结构（base.Vx、Vy、Wz；arm.qdot）  
- 三种执行模式（holistic、staged-B、staged-C）  
- 车轮限速公式及参数命名  
- 控制器模式编号（0、1、2）

**小更新：** 将设计文档中 “宽履带 0.573 m” 更新为 **0.574 m**。

---

### 结论与建议

#### ✅ 架构评价：优秀

- 分层清晰：执行 → 跟随 → 预处理 → 配置  
- 无冗余：各函数职责唯一  
- 接口统一：参数与数据结构一致  
- 易扩展：支持多种执行模式  
- 关注点分离：IK 与控制仿真严格分开

**无需重构与合并，保留现有结构即可。**

#### 📝 文档小更新

1. 更新 `docs/unified_chassis_controller_summary.md` 中的履带宽度  
2. 在本文件中引用该设计文档，保持互相链接  
3. 在设计文档中添加 11 个底盘控制文件的概览表（与本节一致）

#### 🎯 给新成员的提醒

- **是否存在冗余函数？** ❌ 没有，每个函数职责唯一  
- **是否存在冲突？** ❌ 没有，参数与接口协调  
- **需要重构吗？** ❌ 不需要，架构优良  
- **runTrajectoryControl 与 simulateChassisExecution 是重复的吗？** ❌ 不，它们分别解决 IK 与控制仿真两类问题，是 Stage C 三遍架构的核心  
- **如何按层使用这些函数？**  
  - 第 1 层：`unifiedChassisCtrl` 产生命令  
  - 第 2 层：`simulateChassisExecution` / `purePursuitFollower` 仿真控制器  
  - 第 3 层：`preparePathForFollower` / `rsRefinePath` / `rsClothoidRefine` 处理路径  
  - 第 4 层：`loadChassisProfile` / `clampYawByWheelLimit` 提供配置与约束

---

## 动画系统：数据来源与图例对应

### 概览

动画系统会叠加多条轨迹，展示机器人运动、参考路径与跟踪误差。了解每个元素对应的数据来源对诊断问题至关重要。

| 图例项 | 颜色/样式 | 数据来源 | 数据字段 | 正确性 |
|--------|-----------|----------|----------|--------|
| Stage C 执行底盘 | 蓝色实线 | log.qTraj(1:3,:) | 底盘关节轨迹 | ✅ |
| Stage C 参考（GIK） | 黄色虚线 | stageC.purePursuit.referencePath 或 referenceInitialIk | Pass 1 底盘参考 | ✅ |
| Stage B 执行底盘 | 青色实线 | stageB.qTraj(1:3,:) | Stage B 底盘轨迹 | ✅ |
| 期望 EE 路径 | 紫色虚线 | JSON 航点 | stageC.targetPositions | ✅ |
| Stage B 参考 EE | 白色长虚线 | stageB.eePositions | Stage B 末端执行器轨迹 | ✅ |
| 规划底盘路径 | 品红点线 | stageB.pathStates | Hybrid A* + 平滑 | ✅ |
| 实际 EE 路径 | 绿色实线 | 对 log.qTraj 做 FK | 实际末端执行器轨迹 | ✅ |
| Stage C 参考 EE 航点 | 红点 | **当前错误**：stageC.referenceInitialIk.eePositions | Pass 1 理想轨迹 | ❌ |
| 实际 EE 航点 | 绿色方块 | 对当前帧做 FK | 实际末端执行器位置 | ✅ |

### 发现的关键问题

**问题：** 红点当前显示的是 Pass 1 理想轨迹（无底盘约束），而非实际 Pass 3 轨迹，造成观感偏差。

#### Pass 1/2/3 回顾

- **Pass 1 理想**：忽略底盘速度与控制误差，轨迹不可实现。
- **Pass 2 仿真**：考虑底盘速度、加速度与纯跟踪误差，得到真实底盘轨迹。
- **Pass 3 实际**：在真实底盘轨迹下重新求解 IK，得到可实现的末端执行器轨迹。

红点应该展示 Pass 3 结果，才能与绿色实际轨迹对齐。

#### 当前代码（错误优先级）

```matlab
% 优先使用 Pass 1 理想轨迹（错误）
if isfield(stageC, 'referenceInitialIk') ...
    eePathStageCRef = stageC.referenceInitialIk.eePositions;  % ❌
elseif isfield(stageC, 'targetPositions')
    ...
elseif isfield(stageC, 'eePositions')
    ...
end
```

#### 正确写法

```matlab
if isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
    eePathStageCRef = stageC.eePositions;  % ✅ Pass 3
elseif isfield(stageC, 'targetPositions')
    eePathStageCRef = stageC.targetPositions;
elseif isfield(stageC, 'referenceInitialIk')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;
    warning('Using Pass 1 ideal EE trajectory');
end
```

### 完整数据映射

```
log.qTraj → 机器人渲染、实际 EE 路径、Stage C 底盘
stageC.targetPositions → 期望 EE 路径
stageC.referenceInitialIk.eePositions → 当前红点（错误）
stageC.eePositions → 应显示的实际参考
stageB.pathStates → 规划底盘
stageB.qTraj → Stage B 执行底盘
```

### 时间同步

动画已正确处理时间同步：底盘轨迹会被插值到手臂时间线上，保证双视图一致。

### 渲染元素

1. **机器人网格：** 来自 `meshes/`，半透明渲染。
2. **障碍物：** 圆柱体，带安全裕度。
3. **路径标记：** 如上表。
4. **阶段标签：** 根据 `stageBoundaries` 自动定位。
5. **双视角：** 透视 + 俯视。

### 推荐修复

1. **修复红点数据源（高优先级）**  
   修改 `animateStagedWithHelper.m` 的优先级以使用 `stageC.eePositions`。

2. **更新图例文字（中优先级）**  
   将图例描述改为 “Stage C actual EE trajectory” 等准确表述。

3. **可选调试模式（低优先级）**  
   增加 `ShowPass1Reference` 参数，允许同时展示 Pass 1 与 Pass 3。

修复后，红点与绿色实际轨迹将重合，避免误导。

### 关键可视化元素汇总

- **机器人渲染**：来源于 `log.qTraj` 的 FK，使用 STL 网格。
- **障碍**：来自 `log.floorDiscs`，绘制带安全圈的圆柱。
- **路径**：详见上方数据映射表。
- **阶段标签**：依据 `stageBoundaries` 自动更新。
- **双视角布局**：提供 3D 与俯视视角，便于审查。

---

### 动画脚本对比

| 脚本 | 功能 | 主要用途 | 状态 |
|------|------|----------|------|
| **animateStagedWithHelper.m** | 分阶段动画封装 | 生产使用 | ✅ |
| **animateHolisticWithHelper.m** | 整体模式封装 | 生产使用 | ✅ |
| **+viz/animate_whole_body.m** | 核心渲染器 | 由封装调用 | ✅ |
| **animateStagedLegacy.m** | 旧版动画器 | 遗留 | ⚠️ |
| **animateTrajectory.m** | 简易动画 | 快速查看 | ✅ |
| **regenerate_animations_from_logs.m** | 批量再生成 | 后处理 | ✅ |
| **generate_*.m** | 多个生成脚本 | 专项调试 | ⚠️ 交叠 |

---

## 辅助函数与实用工具

```matlab
% 配置工具
tools = gik9dof.configurationTools(robot);
q_home = tools.home();
q_col = tools.column(q_struct);
q_struct = tools.struct(q_col);

% 路径解析
absPath = gik9dof.internal.resolvePath('relative/path.json');
projectRoot = gik9dof.internal.projectRoot();

% 结果目录
resultsDir = gik9dof.internal.createResultsFolder('my_experiment');
```

### 速度估计器

```matlab
estimator = gik9dof.internal.VelocityEstimator();
estimator.update([x, y, theta], t);
vel = estimator.estimate();
```

- `vel.world`：世界坐标系线速度  
- `vel.robot`：底盘坐标系线速度  
- `vel.omega`：角速度  
- 自适应使用 5 点 / 3 点 / 2 点后向差分
- 当 `VelocityLimits.BaseIndices` 提供时由 `runTrajectoryControl` 自动启用

### 路径处理

```matlab
path = gik9dof.control.preparePathForFollower(rawPath, ...
    'InterpSpacing', 0.05, ...
    'WaypointSpacing', 0.15, ...
    'PathBufferSize', 30.0);

[smoothed, info] = gik9dof.control.rsClothoidRefine(path, params);
```

### 评估工具

```matlab
metrics = gik9dof.evaluateLog(log);
smoothness = gik9dof.evaluatePathSmoothness(path);
metrics = gik9dof.computeBaseRibbonMetrics(baseStates);
intrusion = gik9dof.evaluateCollisionIntrusion(robot, qTraj, obstacles);
feasible = gik9dof.evaluateChassisConstraints(commands, chassisParams);
```

### 绘图工具

```matlab
gik9dof.generateLogPlots(log, 'OutputDir', resultsDir);
gik9dof.plotTrajectoryLog(log);
gik9dof.generateExternalPlots(log, 'Format', 'png');
```

---

## 函数关系分析

### 底盘控制架构概览

文档对底盘控制子系统进行了分层拆解：执行层、路径跟随层、配置层。核心结论：✅ 未发现冗余或冲突，各模块职责清晰。

> ASCII 图与原文一致，此处略。

---

#### 层 1：执行层

**`unifiedChassisCtrl.m`**
- 负责模式路由，将不同输入格式统一成底盘指令
- 进行数值微分、航向修正与差动驱动可行性检查
- 被所有执行模式调用（Holistic、Stage B、Stage C）
- 职能独特，无冗余

---

#### 层 2：路径跟随与仿真

**`purePursuitFollower.m`**  
自适应前视的底盘跟随器，提供 blended / pure pursuit / stanley 三种模式。由 `simulateChassisExecution` 等调用。

**`simulateChassisExecution.m`**  
多模式底盘仿真器：Mode0 差分、Mode1 航向控制、Mode2 纯跟踪（推荐）。负责闭环仿真、积分与诊断。

**`simulatePurePursuitExecution.m`**  
轻量封装，为 Mode2 提供纯跟踪积分循环。

---

#### 层 3：路径预处理与平滑

**`preparePathForFollower.m`**  
对路径进行验证、重采样、曲率与弧长计算，供控制器使用。

**`rsRefinePath.m`**  
Reeds–Shepp 捷径平滑，改进曲率、减少回转。

**`rsClothoidRefine.m`**  
Clothoid 样条平滑，进一步提升连续性。

**辅助工具：**
- `computeReedsSheppMetrics`、`countReedsSheppCusps`
- `densifyHybridStates`、`markDiscOnBinaryMap`

---

#### 配置层

**`loadChassisProfile.m`**  
读取 `config/chassis_profiles.yaml`，结合默认值与覆写生成底盘参数。

**`defaultUnifiedParams.m` / `defaultReedsSheppParams.m`**  
集中定义默认控制与平滑参数。

---

## Stage C 深入解析：三遍架构与数据流

### 概览：Stage C 的 ppForIk 模式

Stage C 在 `ppForIk` 模式下采用与 Holistic 模式完全相同的三遍架构，不同之处仅在于起始姿态：
- **Holistic**：从 `q0`（初始 home 姿态）开始  
- **Stage C**：从 Stage B 停泊后的 `qStart` 开始

**关键事实：**
- Stage C 会调用 **两次 GIK**（Pass 1 和 Pass 3），以及 **一次 simulateChassisExecution**（Pass 2）
- 三个 Pass **顺序执行**，严格依赖前一阶段的输出

### 三遍流水线（串行执行）

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           STAGE C - PASS 1                               │
│                     GIK with Free Base (Reference)                       │
│                                                                           │
│  Input:  qStart (from Stage B docking)                                  │
│          EE trajectory (SE(3) waypoints)                                 │
│                                                                           │
│  Solver: runTrajectoryControl(bundleRef, trajStruct, ...)               │
│          ├─> GIK solver (9-DOF, base free)                              │
│          ├─> Distance constraints                                        │
│          └─> No fixed joint constraints                                  │
│                                                                           │
│  Output: logRef.qTraj (full 9-DOF trajectory)                           │
│          └─> baseReference = qTraj(baseIdx, :)  [Nx3: x, y, θ]         │
│                                                                           │
│  Purpose: Generate IDEAL base path assuming perfect tracking             │
│           (no velocity limits, no dynamics)                              │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼ (SERIAL: Pass 2 waits for Pass 1)
                    ┌──────────────────────────┐
                    │  Optional Refinement     │
                    │  • Reeds-Shepp shortcuts │
                    │  • Clothoid smoothing    │
                    └──────────────────────────┘
                                   │
                                   ▼ (SERIAL: Uses baseReference from Pass 1)
┌─────────────────────────────────────────────────────────────────────────┐
│                           STAGE C - PASS 2                               │
│                    Chassis Simulation (Dynamics)                         │
│                                                                           │
```

三遍架构详解如上图所示：Pass 1 产出理想底盘轨迹，Pass 2 依据底盘约束进行仿真，Pass 3 在真实底盘运动下重新求解手臂 IK。Pass 2 和 Pass 3 均 **依赖** 前一遍的输出，因此无法并行。

### Pass 1 运行前后的关键值

**输入：**
```matlab
qStart = qB_end;  % Stage B docking 后的姿态
trajStruct = struct(
    'Poses', [4×4×N], 
    'EndEffectorPositions', [3×N]
);
bundleRef = gik9dof.createGikSolver(..., 'MaxIterations', options.MaxIterations);
```

**运行：**
```matlab
logRef = gik9dof.runTrajectoryControl(bundleRef, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'Verbose', options.Verbose, ...
    'VelocityLimits', velLimits);
```

**输出：**
```matlab
baseReference = logRef.qTraj(baseIdx, :)';  % [N×3] 理想底盘轨迹
```

### Pass 2 与 Pass 3 的输入/输出

**Pass 2（底盘仿真）**：
```matlab
simRes = gik9dof.control.simulateChassisExecution(baseReference, ...
    'SampleTime', dt, ...
    'FollowerOptions', followerOptions, ...
    'ControllerMode', options.StageCChassisControllerMode);
executedBase = simRes.poses(2:end, :);  % [N×3]
```

**Pass 3（底盘锁定的 IK）**：
```matlab
fixedTrajectory = struct('Indices', baseIdx, 'Values', executedBase');
logC = gik9dof.runTrajectoryControl(bundle, trajStruct, ...
    'InitialConfiguration', qStart, ...
    'RateHz', options.RateHz, ...
    'VelocityLimits', velLimits, ...
    'FixedJointTrajectory', fixedTrajectory);
```

### Pass 1 vs Pass 3 的差异

| 对比项 | Pass 1（底盘自由） | Pass 3（底盘锁定） |
|--------|--------------------|--------------------|
| 底盘运动 | GIK 计算出的理想曲线 | 取自 Pass 2 的真实轨迹 |
| 优化自由度 | 9 DOF（底盘+手臂） | 仅 7 DOF（手臂） |
| 底盘速度 | 不受限 | 满足 1.5 m/s、2.0 rad/s 等约束 |
| 底盘误差 | 理论上 0 | Pass 2 可能出现跟踪误差 |
| EE 误差 | ≈ 0（理想） | <5 mm RMS（极佳） |
| 用途 | 生成理想参考 | 得到可执行轨迹 |

**为什么不能只用 Pass 1？** 因为实际底盘无法达到理想速度与轨迹。  
**为什么不能跳过 Pass 1？** Pass 3 需要 Pass 2 产生的真实底盘轨迹，而 Pass 2 又依赖 Pass 1 的参考。

### 性能指标

- **Pass 1**：10–50 次迭代/航点、0.05–0.2 s/航点、成功率 > 99%  
- **Pass 2**：控制频率 10 Hz，每个航点约 10 个积分步，纯跟踪误差 < 50 mm RMS  
- **Pass 3**：20–100 次迭代/航点、0.1–0.5 s/航点、成功率 > 95%、EE 误差 < 5 mm RMS

### Stage C 与 Holistic 的对比

| 项目 | Stage C | Holistic |
|------|---------|----------|
| 起始姿态 | Stage B 结束后的 `qStart` | 初始 home 姿态 `q0` |
| 航点范围 | Stage B 之后的剩余航点 | 全部航点 |
| 算法 | **相同的三遍架构** | **相同的三遍架构** |
| 目的 | 从 Stage B 切换到全身跟踪 | 全程单阶段执行 |

### 诊断日志

Stage C 会记录丰富的诊断信息，例如：

```matlab
stageCDiagnostics = struct(
    solverIterationsPerWaypoint = [N×1],
    eeErrorBins = struct(excellent, good, acceptable, poor),
    basePosDeviationMean = double,
    basePosDeviationMax = double,
    refinementApplied = true/false,
    refinementDelta = struct(pathLength, eeError)
);
```

这些指标可用于快速评估 GIK 收敛、末端执行器误差与底盘偏差。

### 可视化与调试建议

1. **底盘路径对比**
   ```matlab
   plot(baseReference(:,1), baseReference(:,2), 'b--');
   hold on;
   plot(executedBase(:,1), executedBase(:,2), 'r-');
   legend('Pass 1 Ideal', 'Pass 2 Realistic');
   ```

2. **末端执行器误差**
   ```matlab
   eeError = vecnorm(logC.eePositions - trajStruct.EndEffectorPositions, 2, 1);
   plot(eeError * 1000);
   ylabel('EE Error (mm)');
   ```

3. **求解器收敛情况**
   ```matlab
   histogram(logC.iterations, 0:10:150);
   xlabel('GIK iterations');
   ```

---

## 近期缺陷修复与改进

### 动画数据源修复（2025-10-12）

**状态：** ✅ 已修复  

#### 问题

动画中的红色圆点“Stage C reference EE waypoint”使用了 **Pass 1 理想轨迹**，而非 **Pass 3 实际轨迹**，导致视觉上偏离实际末端执行器位置。

#### 根因

`animateStagedWithHelper.m` 中的优先级顺序错误，优先取用了 `referenceInitialIk.eePositions`（Pass 1）。

#### 影响

- 平均偏差约 268 mm，最大可达 1436 mm  
- 红色参考点远离绿色真实轨迹，用户容易误判跟踪效果  
- 偏差本身是真实存在的底盘误差，但应展示 Pass 3 结果才形象

#### 修复

调整优先级，优先使用 `stageC.eePositions`，必要时回退到 `targetPositions`，最后才使用 Pass 1，并给出警告：

```matlab
if isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
    eePathStageCRef = stageC.eePositions;         % Pass 3（正确）
elseif isfield(stageC, 'targetPositions') && ~isempty(stageC.targetPositions)
    eePathStageCRef = stageC.targetPositions;     % JSON 目标
elseif isfield(stageC, 'referenceInitialIk') && ...
        isfield(stageC.referenceInitialIk, 'eePositions')
    eePathStageCRef = stageC.referenceInitialIk.eePositions;  % Pass 1（调试）
    warning('Using Pass 1 ideal EE trajectory (debug mode)');
end
```

修复后，红点与实际轨迹基本重合，用户能直接看到真实跟踪误差。

#### 验证

在最近的 `log_staged_ppForIk.mat` 中验证：
- `stageC.eePositions` 与 `referenceInitialIk.eePositions` 均存在  
- Pass 1 与 Pass 3 之间的偏差统计维持不变（影响是真实的），但动画展示正确

---

## 冗余、孤立与弃用文件

### ✅ 活跃且关键的文件

**核心库（`+gik9dof/`）：** 44 个文件均在活跃使用

**入口脚本：** `run_staged_reference.m`、`run_environment_compare.m`、`run_fresh_sim_with_animation.m`、`run_parametric_study.m`、`run_comprehensive_chassis_study.m`

**配置资产：** `config/chassis_profiles.yaml` 与 `1_pull_world_scaled.json`

### ⚠️ 功能重叠

#### 动画生成脚本（5 个——建议整合）

| 文件 | 功能 | 独特点 | 建议 |
|------|------|--------|------|
| `generate_animation_from_saved_log.m` | 单个日志 → MP4 | 基础封装 | **合并** 至 `regenerate_animations_from_logs.m` |
| `generate_comprehensive_animations.m` | 批量处理 | 支持多日志 | 保留或并入 `regenerate` |
| `generate_final_animation.m` | “最终版” 动画 | 少量参数特化 | **移除**（使用 `regenerate` 的参数即可） |
| `generate_parametric_animations.m` | 参数扫描 | 面向研究批量 | **保留**（专项能力） |
| `generate_sweep_animations.m` | 扫描动画 | 与 parametric 相似 | 与 `generate_parametric_animations.m` 合并 |

**建议：** 仅保留 `regenerate_animations_from_logs.m`（通用）与 `generate_parametric_animations.m`（参数研究专用）。

#### 测试脚本（10+ 个——建议整理）

**动画回归测试：** `test_animation_sync_fix.m`、`test_single_animation_sync.m`、`test_stage_sync_fix.m`、`test_stagec_path_fix.m` → 验证完成后可归档至 `tests/archive/`。

**算法 / 流程测试：** `test_complete_fix.m`、`test_comprehensive_evaluation.m`、`test_enhanced_logging.m`、`test_issue_fixes.m`、`test_parameter_sweep.m`、`test_tuned_parameters.m` → 建议迁移到 `tests/` 并补充 README 说明用途。

#### 调试脚本（4 个——建议归档）

`debug_animation_sync.m`、`debug_sampling_mismatch.m`、`debug_stage_boundaries.m`、`debug_stagec_ee_path.m` 已完成历史修复，可迁移到 `debug/archive/` 并在文档中保留参考。

### ❌ 临时文件（需立即删除）

- `tmp_regen_staged_only.m`、`tmp_regen.m`、`temp_ref_rs_refine.m`
- `tmp_compare.mat`、`tmp_json.mat`、`tmp_pipeline.mat`

**同时更新 `.gitignore`：**
```
tmp_*.m
tmp_*.mat
temp_*.m
```

### ⚠️ 遗留 / 待弃用脚本

- `animateStagedLegacy.m`（已由 `animateStagedWithHelper.m` + `animate_whole_body.m` 取代）
- `compareAnimationsLegacy.m`（旧版对比工具）

**建议：** 添加弃用提示、更新调用处，1 个版本后移动至 `deprecated/`。

### 📊 文档整合建议

- 合并 `DATA_FLOW_ANALYSIS.m` 与本文内容，避免重复
- 将 `PROJECT_OVERVIEW.md` 与 `GIK_SETUP_OVERVIEW.md` 归并为统一架构文档
- 历史修复文档（如 `ANIMATION_SYNC_FIX*.md`）移动至 `docs/archive/bug_fixes/`
- 保留 `ALGORITHM_IMPROVEMENT_PLAN.md`、`SIMULATION_WORKFLOW_GUIDE.md`、`PROJECT_STATUS_SUMMARY.md`、`HANDOVER.md` 等现行文档
- 建议文档结构：`docs/README.md` → SETUP、USER_GUIDE、ARCHITECTURE、DATA_FLOW、API_REFERENCE、WORKFLOW_GUIDE、STATUS，并在 `docs/archive/` 下存放过时内容

## 关键洞察与建议

### ✅ 优势

1. **包结构清晰**：`+gik9dof/` 下划分控制、可视化、内部工具等子包，避免全局命名冲突  
2. **配置灵活**：底盘参数通过 YAML 配置，可运行时覆写，易于增加新 profile  
3. **日志完备**：阶段日志、诊断指标齐全，可重放仿真  
4. **数据流顺畅**：分阶段管线清晰，数据对象结构化  
5. **动画体系完善**：日志与动画解耦，可离线生成多视角视频

### ⚠️ 改进空间

1. **根目录脚本过多**（38 个），难以分辨活跃/遗留脚本  
2. **文档碎片化**（15+ Markdown），内容重复，无统一入口  
3. **动画脚本功能重叠**（5 个 generate_*.m）  
4. **配置分散**（YAML + 硬编码 + 函数参数）  
5. **临时文件未忽略**（`tmp_*.m` / `.mat`）

---


### 优先级 1：立即清理

1. 删除临时文件：`rm tmp_*.m tmp_*.mat temp_*.m`  
2. 更新 `.gitignore`（添加 tmp_*/temp_*、*.mat、*.mp4、results/ 等）  
3. 将完成的调试脚本移动到 `debug/archive/`

### 优先级 2：结构化整理

1. 创建 `tests/` 目录，迁移所有 `test_*.m` 脚本，并添加说明  
2. 动画脚本整合：保留 `regenerate_animations_from_logs.m` 与 `generate_parametric_animations.m`，删除或合并其他重复脚本  
3. 文档分层：`docs/` 下建立 README、用户指南、架构文档，旧 bug fix 文档归档至 `docs/archive/`

### 优先级 3：API 提升

1. 统一配置：将环境、底盘等参数全部搬到 YAML（如 `config/simulation_config.yaml`）  
2. 收敛入口脚本：提供单一 `run_simulation.m`，通过参数控制模式、执行方式、动画等  
3. 生成 API Reference：利用 MATLAB `help` 系统或自动文档工具

### 数据流关键观察

- **Stage C 三遍架构** 是系统可靠性的关键：Pass 1 生成理想路径，Pass 2 限制动力学，Pass 3 在真实底盘轨迹下重新求解手臂，确保可执行  
- **动画同步** 已修复，`eePathStageCRef` 需按 SampleStep 采样，保证与 `qTraj` 同步  
- **ppForIk** 的视觉效果真实反映底盘误差：机器人模型使用 Pass 3 实际轨迹，红点展示目标轨迹，二者之间的偏差即跟踪误差

### 性能特征

- Stage A：≈ 50 步，<5 s  
- Stage B（pureHyb）：规划 1–3 s，仿真 <5 s  
- Stage C（ppForIk）：每遍 10–15 s，三遍 ≈ 45 s  
- 主要瓶颈：GIK 迭代（150 上限）、RS 捷径迭代、实时渲染

---

## 结论

### 项目成熟度

- **技术质量**：高。错误处理完善、日志详尽、架构灵活、关注点分离  
- **文档质量**：中。内容多但分散，需要统一入口  
- **代码组织**：中。核心库结构良好，根目录脚本需整理，测试脚本分散

### 后续维护路线图

| 时间 | 建议事项 |
|------|----------|
| 立即（≤2h） | 删除临时文件、更新 `.gitignore`、归档调试脚本 |
| 短期（1 天） | 创建 `tests/` 目录、整合动画脚本、重构文档目录 |
| 中期（2–3 天） | 统一配置系统、添加弃用提示、生成 API 文档、整理说明书 |
| 长期（1 周） | 单一入口脚本、全 YAML 配置、搭建自动化测试与 CI |

### 给新成员的快速指引

1. 阅读 `projectDiagnosis.md`（本文件）了解架构  
2. 阅读 `PROJECT_STATUS_SUMMARY.md` 获取进展  
3. 阅读 `SIMULATION_WORKFLOW_GUIDE.md` 学会运行仿真  
4. 浏览 `+gik9dof/` 了解核心库结构

**运行示例：**
```matlab
% 默认参数的分阶段仿真
result = gik9dof.runStagedReference();

% 自定义参数
result = gik9dof.runStagedReference(...
    'ExecutionMode', 'ppForIk', ...
    'RateHz', 10, ...
    'UseStageBHybridAStar', true, ...
    'ChassisProfile', 'wide_track');
% 结果保存在 results/<timestamp>_staged_reference/
```

**重新生成动画：**
```matlab
regenerate_animations_from_logs('results/<your_folder>');
```

**调参步骤：**
1. 编辑 `config/chassis_profiles.yaml`  
2. 通过 `ChassisOverrides` 传入覆写值  
3. 若需改环境，修改 `environmentConfig.m`

---

## 附录：文件索引

### 根目录脚本（共 38 个）

- **活跃入口脚本（5）**：`run_staged_reference.m`、`run_environment_compare.m`、`run_fresh_sim_with_animation.m`、`run_parametric_study.m`、`run_comprehensive_chassis_study.m`
- **动画生成（5）**：`regenerate_animations_from_logs.m` ✅、`generate_parametric_animations.m` ✅、其余 3 个建议整合或移除
- **测试脚本（10）**：`test_*.m` 系列，建议迁移至 `tests/`
- **调试脚本（4）**：`debug_*.m` 系列，可归档
- **分析脚本（5）**：`analyze_all_tests.m`、`check_reference_quality.m` 等，可保留
- **临时文件（6）**：`tmp_*.m` / `.mat`，需删除并加入 `.gitignore`
- **工具脚本（3）**：`export_all_commands.m`、`view_json_waypoints.m`、`plotJsonPath.m`（与 `matlab/plotJsonPath.m` 重复）

### 核心库（+gik9dof/）—— 共 44 个

详细清单参见本文前文。

### 文档（15+）

- **活跃文档**：本文件、`PROJECT_STATUS_SUMMARY.md`、`SIMULATION_WORKFLOW_GUIDE.md`、`ALGORITHM_IMPROVEMENT_PLAN.md`、`HANDOVER.md`、`diary.md`、`guideline.md`
- **可归档**：`ANIMATION_SYNC_FIX.md`、`COMPLETE_TIMING_FIX.md` 等已完成 bug 修复记录

---

**文档版本：** 1.0  
**最后更新：** 2025-10-11  
**作者：** gikWBC9DOF 项目的 AI 分析  
**状态：** ✅ 完成

---

