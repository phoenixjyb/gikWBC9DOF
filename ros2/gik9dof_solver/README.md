# gik9dof_solver

This package wraps the MATLAB Coder outputs for the 9-DOF GIK solver so they can be linked into ROS 2 nodes. To refresh the generated sources:

1. From MATLAB (or via `run_codegen.bat`), execute `matlab/run_codegen.m`. By default this drops artefacts under `codegen/linux_arm64` with the hardware configuration set for 64-bit ARM/Linux.
2. Copy the emitted `.cpp`/`.c` files into `src/generated` and the `.h` files into `include/gik9dof_solver/generated`. The helper in this branch already performs this copy once.
3. Rebuild the ROS 2 workspace with `colcon build`. When generated sources are present the library defines `GIK9DOF_CODEGEN_AVAILABLE` and the wrapper calls into the native solver; otherwise it stays in the stubbed pass-through mode so other nodes continue to compile.
