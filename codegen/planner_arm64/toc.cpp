//
// File: toc.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

// Include Files
#include "toc.h"
#include "HybridAStarPlanner.h"
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : HybridAStarPlanner *aInstancePtr
//                const coderTimespec &savedTime
// Return Type  : double
//
namespace gik9dof {
namespace coder {
double toc(HybridAStarPlanner *aInstancePtr, const coderTimespec &savedTime)
{
  coderTimespec b_timespec;
  gik9dof_planHybridAStarCodegenStackData *localSD;
  double t;
  double tstart_tv_nsec;
  localSD = aInstancePtr->getStackData();
  t = internal::b_time::impl::timeKeeper(savedTime, tstart_tv_nsec);
  if (!localSD->pd->freq_not_empty) {
    localSD->pd->freq_not_empty = true;
    coderInitTimeFunctions(&localSD->pd->freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
  return (b_timespec.tv_sec - t) +
         (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for toc.cpp
//
// [EOF]
//
