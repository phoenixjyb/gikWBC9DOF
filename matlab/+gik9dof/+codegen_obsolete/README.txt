OBSOLETE CODE - DO NOT USE
===========================

This namespace (+codegen_obsolete) is from early development and is NO LONGER USED.

⚠️ DO NOT USE THIS FOR CODE GENERATION ⚠️

Why This Is Obsolete:
---------------------
1. Loads robot from MAT file (robotModel.mat) - incompatible with embedded deployment
2. No real-time optimizations (no MaxTime, no MaxIterations limits)
3. No SIMD-specific optimizations (no ARM NEON or x86 SSE configuration)
4. Requires pre-generation step (generateRobotModelData.m)

What To Use Instead:
--------------------
Use the +codegen_inuse namespace for ALL code generation:

  matlab/+gik9dof/+codegen_inuse/
  
Code generation scripts at project root:
  - generate_code_arm64.m   (for NVIDIA Orin - REAL-TIME)
  - generate_code_x86_64.m  (for WSL - VALIDATION)

Current Active Files:
---------------------
✅ +codegen_inuse/buildRobotForCodegen.m  - Procedural robot builder
✅ +codegen_inuse/solveGIKStepWrapper.m   - Main entry point
✅ +codegen_inuse/solveGIKStepRealtime.m  - Core IK solver

Obsolete Files (This Directory):
---------------------------------
❌ loadRobotForCodegen.m       - Loads from MAT file (bad for codegen)
❌ robotModel.mat              - Pre-generated robot data
❌ generateRobotModelData.m    - Creates MAT file
❌ solveGIKStep.m              - Basic IK (no optimizations)
❌ solveGIKStepWithLock.m      - IK with locking (not used)
❌ stagedFollowTrajectory.m    - Complex trajectory planner (archived)
❌ generateCode.m              - Old codegen script

Why Not Delete?
---------------
Kept for historical reference and possible future trajectory planning features.
Doesn't interfere with current code.

For More Info:
--------------
See: NAMESPACES_EXPLAINED.md at project root

Last Updated: October 7, 2025
