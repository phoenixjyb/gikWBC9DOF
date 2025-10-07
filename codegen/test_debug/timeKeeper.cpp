//
// File: timeKeeper.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "timeKeeper.h"
#include "gik9dof_planHybridAStarCodegen_data.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Variable Definitions
static boolean_T savedTime_not_empty;

// Function Definitions
//
// Arguments    : double newTime_tv_sec
//                double newTime_tv_nsec
//                coderTimespec &savedTime
// Return Type  : void
//
namespace coder {
namespace internal {
namespace b_time {
namespace impl {
void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec,
                coderTimespec &savedTime)
{
  if (!savedTime_not_empty) {
    coderTimespec b_timespec;
    if (!freq_not_empty) {
      freq_not_empty = true;
      coderInitTimeFunctions(&freq);
    }
    coderTimeClockGettimeMonotonic(&b_timespec, freq);
    savedTime_not_empty = true;
  }
  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
}

//
// Arguments    : const coderTimespec &savedTime
//                double &outTime_tv_nsec
// Return Type  : double
//
double timeKeeper(const coderTimespec &savedTime, double &outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

//
// Arguments    : void
// Return Type  : void
//
} // namespace impl
} // namespace b_time
} // namespace internal
} // namespace coder
void timeKeeper_init()
{
  savedTime_not_empty = false;
}

//
// File trailer for timeKeeper.cpp
//
// [EOF]
//
