//
// File: xzlangeM.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
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
namespace gik9dof {
namespace coder {
namespace internal {
namespace reflapack {
double xzlangeM(const double x[9])
{
  double y;
  int k;
  bool exitg1;
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
} // namespace gik9dof

//
// File trailer for xzlangeM.cpp
//
// [EOF]
//
