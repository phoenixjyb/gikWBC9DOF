//
// File: angdiff.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "angdiff.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double x
//                double y
// Return Type  : double
//
namespace coder {
double angdiff(double x, double y)
{
  double delta;
  delta = y - x;
  if (std::abs(delta) > 3.1415926535897931) {
    double thetaWrap;
    thetaWrap = b_mod(delta + 3.1415926535897931);
    if ((thetaWrap == 0.0) && (delta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }
    delta = thetaWrap - 3.1415926535897931;
  }
  return delta;
}

} // namespace coder

//
// File trailer for angdiff.cpp
//
// [EOF]
//
