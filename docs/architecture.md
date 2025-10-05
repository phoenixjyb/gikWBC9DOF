# GIK 9-DOF Software Architecture

```mermaid
graph LR
  subgraph MATLAB Development
    M1[MATLAB Authoring\n`matlab/+gik9dof`] --> M2[Simulation & Analysis\n`runTrajectoryControl`, visualization]
    M2 --> M3[Code Generation Scripts\n`solveGIKStep`, `followTrajectory`]
  end

  M3 -->|MATLAB Coder| G1[Generated C++ Sources\n`codegen/linux_arm64`\n- solver code\n- build metadata]
  M3 -->|Robot model cache| Cache[(robotModel.mat\n`codegen` cache)]

  G1 -->|Sync| S1[gik9dof_solver ROS 2 package\n`src/generated`, `src/collisioncodegen`, `src/libccd`]
  Cache -->|Procedural build via `buildRobotForCodegen`| S1

  subgraph ROS 2 Stack (WSL)
    S1 -->|library export `gik9dof_solver::gik9dof_solver`| S2[gik9dof_controllers package\n`holistic_controller`, `staged_controller`, `trajectory_manager`, `obstacle_provider`]
    S2 -->|Publish commands| Bus[(ROS 2 Graph)]
    Bus -->|`/control/joint_commands`| Actuators[Arm/Base Controllers or Sim]
    Bus -->|`/control/base_commands`| Actuators
    Traj[Trajectory Sources\nMATLAB playback, bags, planners] -->|`/target/end_effector_trajectory`| Bus
    State[Robot State Providers\njoint_state publishers] -->|`/robot_state/joint_states`| S2
  end

  subgraph Tooling & Docs
    Docs[Documentation & Runbook\n`docs/runbook.md`, `docs/architecture.md`]
    Logs[Validation Artifacts\n`diary.md`, `results/`]
  end

  M1 -. updates .-> Docs
  S2 -. metrics .-> Logs
```

## Layer Summary

- **MATLAB Development** – MATLAB code under `matlab/+gik9dof` provides modelling, simulation, and the MATLAB Coder entry points (`solveGIKStep`, `followTrajectory`). The helper `buildRobotForCodegen` ensures the rigid body tree is built procedurally for portable code generation.
- **Generated Sources** – MATLAB Coder produces platform-neutral C++ in `codegen/linux_arm64`, including solver logic and metadata. Robot model data (`robotModel.mat`) supports verification and can be regenerated whenever URDF/meshes change.
- **ROS 2 Solver Package** – `ros2/gik9dof_solver` consumes the generated code, vendored collision helpers, and libccd sources. Its CMake exports the `gik9dof_solver::gik9dof_solver` target with OpenMP linkage for downstream packages.
- **ROS 2 Controllers** – `ros2/gik9dof_controllers` hosts the executable nodes (holistic and staged controllers, trajectory manager, obstacle provider). These nodes link against the solver library and exchange ROS topics with trajectory publishers and hardware/simulation bridges.
- **Tooling & Documentation** – Operational knowledge is captured in `docs/runbook.md` and this architecture note; experiment logs and regression artifacts reside in `diary.md` and `results/`.

## Data & Control Flow Highlights

1. MATLAB regenerates solver code and optional artefacts, then C++ outputs are copied into the ROS workspace.
2. `colcon build` in WSL compiles `gik9dof_solver`, exporting a shared/static library for other ROS packages.
3. Controller nodes subscribe to trajectory and robot state topics, invoke the solver wrapper each control tick, and publish joint/base commands.
4. Validation logs feed back into documentation and planning for subsequent code generation cycles.
