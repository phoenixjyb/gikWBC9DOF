//
// File: CoderTimeAPI.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "CoderTimeAPI.h"
#include "GIKSolver.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void CoderTimeAPI::callCoderClockGettime_init(GIKSolver *aInstancePtr)
{
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->freq_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for CoderTimeAPI.cpp
//
// [EOF]
//
