//
// File: tic.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "tic.h"
#include "GIKSolver.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_types.h"
#include "coder_posix_time.h"
#include <cstring>

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
//                double &tstart_tv_nsec
// Return Type  : double
//
namespace gik9dof {
namespace coder {
double tic(GIKSolver *aInstancePtr, double &tstart_tv_nsec)
{
  coderTimespec b_timespec;
  solveGIKStepWrapperStackData *localSD;
  double tstart_tv_sec;
  localSD = aInstancePtr->getStackData();
  if (!localSD->pd->freq_not_empty) {
    localSD->pd->freq_not_empty = true;
    coderInitTimeFunctions(&localSD->pd->freq);
  }
  coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
  tstart_tv_sec = b_timespec.tv_sec;
  tstart_tv_nsec = b_timespec.tv_nsec;
  return tstart_tv_sec;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for tic.cpp
//
// [EOF]
//
