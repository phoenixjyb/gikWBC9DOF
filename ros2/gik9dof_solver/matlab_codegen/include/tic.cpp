//
// File: tic.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "tic.h"
#include "GIKSolver.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

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
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *localSD;
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
