//
// File: CoderTimeAPI.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

// Include Files
#include "CoderTimeAPI.h"
#include "HybridAStarPlanner.h"
#include "planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : HybridAStarPlanner *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
void CoderTimeAPI::callCoderClockGettime_init(HybridAStarPlanner *aInstancePtr)
{
  planHybridAStarCodegenStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->freq_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for CoderTimeAPI.cpp
//
// [EOF]
//
