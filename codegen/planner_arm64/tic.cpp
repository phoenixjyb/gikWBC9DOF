//
// File: tic.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 14:15:48
//

// Include Files
#include "tic.h"
#include "HybridAStarPlanner.h"
#include "planHybridAStarCodegen_types.h"
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
  planHybridAStarCodegenStackData *localSD;
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
