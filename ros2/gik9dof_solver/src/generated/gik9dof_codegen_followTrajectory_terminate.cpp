//
// gik9dof_codegen_followTrajectory_terminate.cpp
//
// Code generation for function 'gik9dof_codegen_followTrajectory_terminate'
//

// Include files
#include "gik9dof_codegen_followTrajectory_terminate.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "loadRobotForCodegen.h"
#include "rt_nonfinite.h"
#include "solveGIKStep.h"
#include "omp.h"

// Function Definitions
void gik9dof_codegen_followTrajectory_terminate()
{
  solveGIKStep_delete();
  loadRobotForCodegen_delete();
  omp_destroy_nest_lock(&gik9dof_codegen_followTrajectory_nestLockGlobal);
  isInitialized_gik9dof_codegen_followTrajectory = false;
}

// End of code generation (gik9dof_codegen_followTrajectory_terminate.cpp)
