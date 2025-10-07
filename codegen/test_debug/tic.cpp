//
// File: tic.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "tic.h"
#include "gik9dof_planHybridAStarCodegen_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : coderTimespec &savedTime
// Return Type  : void
//
namespace coder {
void tic(coderTimespec &savedTime)
{
  coderTimespec b_timespec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  internal::b_time::impl::timeKeeper(b_timespec.tv_sec, b_timespec.tv_nsec,
                                     savedTime);
}

} // namespace coder

//
// File trailer for tic.cpp
//
// [EOF]
//
