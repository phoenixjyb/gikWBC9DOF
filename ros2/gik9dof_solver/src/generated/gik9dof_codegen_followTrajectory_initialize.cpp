//
// gik9dof_codegen_followTrajectory_initialize.cpp
//
// Code generation for function 'gik9dof_codegen_followTrajectory_initialize'
//

// Include files
#include "gik9dof_codegen_followTrajectory_initialize.h"
#include "CoderTimeAPI.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "loadRobotForCodegen.h"
#include "rt_nonfinite.h"
#include "solveGIKStep.h"
#include "omp.h"

// Function Definitions
void gik9dof_codegen_followTrajectory_initialize()
{
  omp_init_nest_lock(&gik9dof_codegen_followTrajectory_nestLockGlobal);
  solveGIKStep_new();
  loadRobotForCodegen_new();
  solveGIKStep_init();
  loadRobotForCodegen_init();
  eml_rand_mt19937ar_stateful_init();
  CoderTimeAPI::callCoderClockGettime_init();
  isInitialized_gik9dof_codegen_followTrajectory = true;
}

// End of code generation (gik9dof_codegen_followTrajectory_initialize.cpp)
