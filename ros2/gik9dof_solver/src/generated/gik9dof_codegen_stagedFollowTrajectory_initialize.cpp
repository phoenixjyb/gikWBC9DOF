//
// gik9dof_codegen_stagedFollowTrajectory_initialize.cpp
//
// Code generation for function
// 'gik9dof_codegen_stagedFollowTrajectory_initialize'
//

// Include files
#include "gik9dof_codegen_stagedFollowTrajectory_initialize.h"
#include "CoderTimeAPI.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "gik9dof_codegen_stagedFollowTrajectory.h"
#include "gik9dof_codegen_stagedFollowTrajectory_data.h"
#include "loadRobotForCodegen.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWithLock.h"
#include "omp.h"

// Function Definitions
void gik9dof_codegen_stagedFollowTrajectory_initialize()
{
  omp_init_nest_lock(&gik9dof_codegen_stagedFollowTrajectory_nestLockGlobal);
  stagedFollowTrajectory_new();
  loadRobotForCodegen_new();
  solveGIKStepWithLock_new();
  stagedFollowTrajectory_init();
  loadRobotForCodegen_init();
  eml_rand_mt19937ar_stateful_init();
  solveGIKStepWithLock_init();
  CoderTimeAPI::callCoderClockGettime_init();
  isInitialized_gik9dof_codegen_stagedFollowTrajectory = true;
}

// End of code generation
// (gik9dof_codegen_stagedFollowTrajectory_initialize.cpp)
