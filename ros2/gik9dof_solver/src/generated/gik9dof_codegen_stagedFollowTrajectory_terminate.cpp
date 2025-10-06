//
// gik9dof_codegen_stagedFollowTrajectory_terminate.cpp
//
// Code generation for function
// 'gik9dof_codegen_stagedFollowTrajectory_terminate'
//

// Include files
#include "gik9dof_codegen_stagedFollowTrajectory_terminate.h"
#include "gik9dof_codegen_stagedFollowTrajectory.h"
#include "gik9dof_codegen_stagedFollowTrajectory_data.h"
#include "loadRobotForCodegen.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWithLock.h"
#include "omp.h"

// Function Definitions
void gik9dof_codegen_stagedFollowTrajectory_terminate()
{
  stagedFollowTrajectory_delete();
  loadRobotForCodegen_delete();
  solveGIKStepWithLock_delete();
  omp_destroy_nest_lock(&gik9dof_codegen_stagedFollowTrajectory_nestLockGlobal);
  isInitialized_gik9dof_codegen_stagedFollowTrajectory = false;
}

// End of code generation (gik9dof_codegen_stagedFollowTrajectory_terminate.cpp)
