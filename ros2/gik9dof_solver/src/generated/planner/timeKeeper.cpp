//
// File: timeKeeper.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

// Include Files
#include "timeKeeper.h"
#include "HybridAStarPlanner.h"
#include "planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : const coderTimespec &savedTime
//                double &outTime_tv_nsec
// Return Type  : double
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace b_time {
namespace impl {
double timeKeeper(const coderTimespec &savedTime, double &outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

//
// Arguments    : HybridAStarPlanner *aInstancePtr
//                double newTime_tv_sec
//                double newTime_tv_nsec
//                coderTimespec &savedTime
// Return Type  : void
//
void timeKeeper(HybridAStarPlanner *aInstancePtr, double newTime_tv_sec,
                double newTime_tv_nsec, coderTimespec &savedTime)
{
  planHybridAStarCodegenStackData *localSD;
  localSD = aInstancePtr->getStackData();
  if (!localSD->pd->savedTime_not_empty) {
    coderTimespec b_timespec;
    if (!localSD->pd->freq_not_empty) {
      localSD->pd->freq_not_empty = true;
      coderInitTimeFunctions(&localSD->pd->freq);
    }
    coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
    localSD->pd->savedTime_not_empty = true;
  }
  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
}

//
// Arguments    : HybridAStarPlanner *aInstancePtr
// Return Type  : void
//
} // namespace impl
} // namespace b_time
} // namespace internal
} // namespace coder
void timeKeeper_init(HybridAStarPlanner *aInstancePtr)
{
  planHybridAStarCodegenStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->savedTime_not_empty = false;
}

} // namespace gik9dof

//
// File trailer for timeKeeper.cpp
//
// [EOF]
//
