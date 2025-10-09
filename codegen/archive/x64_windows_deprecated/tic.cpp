//
// File: tic.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "tic.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double &tstart_tv_nsec
// Return Type  : double
//
namespace coder {
double tic(double &tstart_tv_nsec)
{
  coderTimespec b_timespec;
  double tstart_tv_sec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  tstart_tv_sec = b_timespec.tv_sec;
  tstart_tv_nsec = b_timespec.tv_nsec;
  return tstart_tv_sec;
}

} // namespace coder

//
// File trailer for tic.cpp
//
// [EOF]
//
