//
// File: CoderTimeAPI.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "CoderTimeAPI.h"
#include "GIKSolver.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void CoderTimeAPI::callCoderClockGettime_init(GIKSolver *aInstancePtr)
{
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->freq_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for CoderTimeAPI.cpp
//
// [EOF]
//
