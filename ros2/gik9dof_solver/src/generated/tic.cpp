//
// tic.cpp
//
// Code generation for function 'tic'
//

// Include files
#include "tic.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Function Definitions
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

// End of code generation (tic.cpp)
