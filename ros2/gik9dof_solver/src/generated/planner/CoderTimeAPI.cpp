//
// File: CoderTimeAPI.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

// Include Files
#include "CoderTimeAPI.h"
#include "HybridAStarPlanner.h"
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : HybridAStarPlanner *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void CoderTimeAPI::callCoderClockGettime_init(HybridAStarPlanner *aInstancePtr)
{
  gik9dof_planHybridAStarCodegenStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->freq_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for CoderTimeAPI.cpp
//
// [EOF]
//
