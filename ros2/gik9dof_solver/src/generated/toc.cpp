//
// toc.cpp
//
// Code generation for function 'toc'
//

// Include files
#include "toc.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Function Definitions
namespace coder {
double toc(double tstart_tv_sec, double tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - tstart_tv_sec) +
         (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9;
}

} // namespace coder

// End of code generation (toc.cpp)
