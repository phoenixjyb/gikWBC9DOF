//
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "eml_rand_mt19937ar_stateful.h"
#include "GIKSolver.h"
#include "eml_rand_mt19937ar.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void eml_rand_mt19937ar_stateful_init(GIKSolver *aInstancePtr)
{
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  coder::internal::randfun::eml_rand_mt19937ar(localSD->pd->state);
}

} // namespace gik9dof

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
