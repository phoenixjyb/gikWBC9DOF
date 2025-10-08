//
// File: xzlangeM.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "xzlangeM.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double x[9]
// Return Type  : double
//
namespace coder {
namespace internal {
namespace reflapack {
double xzlangeM(const double x[9])
{
  double y;
  int k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 9)) {
    double absxk;
    absxk = std::abs(x[k]);
    if (std::isnan(absxk)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }
      k++;
    }
  }
  return y;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlangeM.cpp
//
// [EOF]
//
