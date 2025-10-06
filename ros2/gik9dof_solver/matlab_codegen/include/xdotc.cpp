//
// File: xdotc.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "xdotc.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : int n
//                const double x[9]
//                int ix0
//                const double y[9]
//                int iy0
// Return Type  : double
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  int i;
  d = 0.0;
  i = static_cast<unsigned char>(n);
  for (int k{0}; k < i; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }
  return d;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xdotc.cpp
//
// [EOF]
//
