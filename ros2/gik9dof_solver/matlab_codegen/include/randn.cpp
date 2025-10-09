//
// File: randn.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "randn.h"
#include "GIKSolver.h"
#include "eml_rand_mt19937ar.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
//                double r[3]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void b_randn(GIKSolver *aInstancePtr, double r[3])
{
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  r[0] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
}

//
// Arguments    : GIKSolver *aInstancePtr
//                double r[4]
// Return Type  : void
//
void randn(GIKSolver *aInstancePtr, double r[4])
{
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  r[0] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  r[3] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
}

//
// Arguments    : GIKSolver *aInstancePtr
//                const double varargin_1[2]
//                double r_data[]
// Return Type  : int
//
int randn(GIKSolver *aInstancePtr, const double varargin_1[2], double r_data[])
{
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *localSD;
  int i;
  int r_size;
  localSD = aInstancePtr->getStackData();
  i = static_cast<int>(varargin_1[0]);
  r_size = static_cast<int>(varargin_1[0]);
  for (int k{0}; k < i; k++) {
    r_data[k] = internal::randfun::b_eml_rand_mt19937ar(localSD->pd->state);
  }
  return r_size;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for randn.cpp
//
// [EOF]
//
