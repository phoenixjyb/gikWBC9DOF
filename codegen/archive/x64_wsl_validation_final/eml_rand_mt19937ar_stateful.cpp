//
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "eml_rand_mt19937ar_stateful.h"
#include "GIKSolver.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_types.h"
#include <cstring>

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void eml_rand_mt19937ar_stateful_init(GIKSolver *aInstancePtr)
{
  solveGIKStepWrapperStackData *localSD;
  unsigned int r;
  localSD = aInstancePtr->getStackData();
  std::memset(&localSD->pd->state[0], 0, 625U * sizeof(unsigned int));
  r = 5489U;
  localSD->pd->state[0] = 5489U;
  for (int mti{0}; mti < 623; mti++) {
    r = ((r ^ r >> 30U) * 1812433253U + static_cast<unsigned int>(mti)) + 1U;
    localSD->pd->state[mti + 1] = r;
  }
  localSD->pd->state[624] = 624U;
}

} // namespace gik9dof

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
