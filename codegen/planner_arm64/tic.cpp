//
// File: tic.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

// Include Files
#include "tic.h"
#include "HybridAStarPlanner.h"
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : HybridAStarPlanner *aInstancePtr
//                coderTimespec &savedTime
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void tic(HybridAStarPlanner *aInstancePtr, coderTimespec &savedTime)
{
  coderTimespec b_timespec;
  gik9dof_planHybridAStarCodegenStackData *localSD;
  localSD = aInstancePtr->getStackData();
  if (!localSD->pd->freq_not_empty) {
    localSD->pd->freq_not_empty = true;
    coderInitTimeFunctions(&localSD->pd->freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
  internal::b_time::impl::timeKeeper(aInstancePtr, b_timespec.tv_sec,
                                     b_timespec.tv_nsec, savedTime);
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for tic.cpp
//
// [EOF]
//
