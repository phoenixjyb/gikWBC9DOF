//
// File: xswap.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "xswap.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : double x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
void xswap(double x[9], int ix0, int iy0)
{
  double temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xswap.cpp
//
// [EOF]
//
