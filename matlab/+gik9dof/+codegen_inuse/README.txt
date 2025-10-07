ACTIVE CODE - USE THIS FOR CODE GENERATION
===========================================

This namespace (+codegen_inuse) contains the CURRENT, OPTIMIZED code for deployment.

✅ USE THIS FOR ALL CODE GENERATION ✅

Why This Is The Current Namespace:
-----------------------------------
1. ✅ Builds robot PROCEDURALLY (no file I/O - embedded-friendly)
2. ✅ REAL-TIME optimized (MaxTime=50ms, MaxIterations=50)
3. ✅ SIMD-specific (ARM NEON for Orin, x86 SSE/AVX for WSL)
4. ✅ Warm-start enabled (uses previous solution as initial guess)
5. ✅ Full code generation compatible

Active Files In This Directory:
--------------------------------
✅ buildRobotForCodegen.m      - Procedural 9-DOF robot builder
✅ solveGIKStepWrapper.m       - Main entry point (persistent solver)
✅ solveGIKStepRealtime.m      - Core IK solver function
✅ validate_robot_builder.m    - Robot builder validation
✅ generateCodeARM64.m         - Legacy ARM64 codegen (use root script instead)

Code Generation Scripts (Use These):
-------------------------------------
At project root:
  - generate_code_arm64.m      → ARM64 for Orin (REAL-TIME deployment)
  - generate_code_x86_64.m     → x86_64 for WSL (VALIDATION only)

Entry Point:
------------
gik9dof.codegen_inuse.solveGIKStepWrapper(qCurrent, targetPose, distanceLower, distanceWeight)

Key Features:
-------------
✅ MaxTime = 50ms (hard real-time constraint)
✅ MaxIterations = 50
✅ AllowRandomRestart = false (deterministic timing)
✅ SolutionTolerance = 1e-6 (good for robotic control)
✅ GradientTolerance = 1e-7

Outputs To:
-----------
codegen/arm64_realtime/       (ARM NEON - for Orin deployment)
codegen/x86_64_validation/    (x86 SSE/AVX - for WSL testing)

For More Info:
--------------
See documentation at project root:
  - NAMESPACES_EXPLAINED.md    - Namespace comparison
  - CODEGEN_QUICK_GUIDE.md     - Code generation workflow
  - PERFORMANCE_OPTIMIZATION.md - Optimization details
  - NAMING_CONVENTION.md       - Directory naming

Last Updated: October 7, 2025
