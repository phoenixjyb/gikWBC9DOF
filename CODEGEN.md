# Code Generation Overview

The `matlab/+gik9dof/+codegen` package contains the MATLAB Coder entry points for exporting the inner generalized inverse kinematics (GIK) routines to C++. This note summarises what the pipeline produces and how to run it.

## Components

- `generateRobotModelData.m` – loads the project URDF via `gik9dof.createRobotModel` and caches the resulting `rigidBodyTree` in `matlab/+gik9dof/+codegen/robotModel.mat`.
- `loadRobotForCodegen.m` – codegen-safe loader that pulls the cached model at compile time.
- `solveGIKStep.m` – single velocity-level GIK solve:
  - inputs: current joint vector (9×1), target end-effector transform (4×4), distance lower bound, distance weight.
  - output: updated joint vector after one solver iteration.
- `followTrajectory.m` – loops `solveGIKStep` over a stack of target poses.
- `generateCode.m` – convenience script that compiles both entry points to C++ (`lib` target) using MATLAB Coder, dropping artefacts under the chosen output folder (default `codegen/`).

Running the pipeline yields (per entry point):

```
codegen/
  solveGIKStep/ (or followTrajectory/)
    solveGIKStep.cpp / .h   % generated implementation
    solveGIKStep_data.cpp   % static data
    solveGIKStep_initialize.cpp / terminate.cpp
    examples/               % basic usage stub
    html/report.mldatx      % Coder report
```

## Usage

1. Open MATLAB in the repository root and add `matlab/` to the path.
2. Generate the cached robot model (rerun whenever the URDF changes):

   ```matlab
   gik9dof.codegen.generateRobotModelData
   ```

3. Produce C++ sources:

   ```matlab
   gik9dof.codegen.generateCode('codegen')
   ```

   Adjust the output directory as needed; the script ensures it exists and writes both entry points in the same folder.

## Deployment Roadmap (ROS 2 on Linux)

The long-term target is a Linux robot stack with ROS 2. To make the generated solver consumable there:

1. **Replace MAT-file loading with a builder.** MATLAB Coder cannot import handle objects (like `rigidBodyTree`) from MAT files. Implement a `buildRobotForCodegen` helper that instantiates the 9‑DOF rigid body tree procedurally, and call it from `loadRobotForCodegen`. Once this change is made, codegen will succeed on Linux.

2. **Run MATLAB Coder on the Linux host.** With the builder in place, execute `gik9dof.codegen.generateRobotModelData` (or its replacement) and then `gik9dof.codegen.generateCode`. Target a C++ static library in a directory you can include in your ROS 2 workspace, using the same compiler toolchain as your ROS 2 build (GCC/Clang).

3. **Wrap the generated solver in a ROS 2 node.** Create a thin C++ layer around `solveGIKStep`/`followTrajectory` that exposes ROS 2 services or actions (e.g., `FollowJointTrajectory`). Publish diagnostics for tracking errors and accept obstacle information (disc footprints, etc.) as messages for distance constraints.

4. **Integrate collision objects and staged vs holistic modes.** The generated solver should accept distance specs derived from the same floor-disc definitions used in MATLAB. Provide configuration parameters for choosing holistic or staged control at node startup (mirroring the MATLAB toggle).

5. **Test and validate.** Co-simulate by publishing trajectories from MATLAB into ROS 2, or replay recorded data directly against the node. Once validated, the ROS 2 node replaces MATLAB in the real-time loop while MATLAB remains a design/analysis environment.

Following this roadmap ensures the MATLAB prototypes map cleanly onto a deployable ROS 2 control component.
