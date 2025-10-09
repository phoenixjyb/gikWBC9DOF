//
// File: CoderTimeAPI.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "CoderTimeAPI.h"
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
void CoderTimeAPI::callCoderClockGettime_init(GIKSolver *aInstancePtr)
{
  solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->freq_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for CoderTimeAPI.cpp
//
// [EOF]
//
