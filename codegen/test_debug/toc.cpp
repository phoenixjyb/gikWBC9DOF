//
// File: toc.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "toc.h"
#include "gik9dof_planHybridAStarCodegen_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : const coderTimespec &savedTime
// Return Type  : double
//
namespace coder {
double toc(const coderTimespec &savedTime)
{
  coderTimespec b_timespec;
  double t;
  double tstart_tv_nsec;
  t = internal::b_time::impl::timeKeeper(savedTime, tstart_tv_nsec);
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - t) +
         (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9;
}

} // namespace coder

//
// File trailer for toc.cpp
//
// [EOF]
//
