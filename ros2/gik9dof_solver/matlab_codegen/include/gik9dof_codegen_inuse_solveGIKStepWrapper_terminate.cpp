//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.h"
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
void gik9dof_codegen_inuse_solveGIKStepWrapper_terminate()
{
  solveGIKStepWrapper_delete();
  omp_destroy_nest_lock(
      &gik9dof_codegen_inuse_solveGIKStepWrapper_nestLockGlobal);
  isInitialized_gik9dof_codegen_inuse_solveGIKStepWrapper = false;
}

//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.cpp
//
// [EOF]
//
