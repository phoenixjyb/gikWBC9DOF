//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.h"
#include "CoderTimeAPI.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rt_nonfinite.h"
#include "omp.h"
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void gik9dof_codegen_inuse_solveGIKStepWrapper_initialize()
{
  omp_init_nest_lock(&gik9dof_codegen_inuse_solveGIKStepWrapper_nestLockGlobal);
  solveGIKStepWrapper_new();
  solveGIKStepWrapper_init();
  eml_rand_mt19937ar_stateful_init();
  CoderTimeAPI::callCoderClockGettime_init();
  isInitialized_gik9dof_codegen_inuse_solveGIKStepWrapper = true;
}

//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.cpp
//
// [EOF]
//
