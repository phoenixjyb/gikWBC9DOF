//
// File: xaxpy.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double a
//                const double x[9]
//                int ix0
//                double y[3]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
void xaxpy(double a, const double x[9], int ix0, double y[3])
{
  if (!(a == 0.0)) {
    __m128d r;
    r = _mm_loadu_pd(&y[1]);
    _mm_storeu_pd(&y[1], _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(a),
                                                  _mm_loadu_pd(&x[ix0 - 1]))));
  }
}

//
// Arguments    : double a
//                const double x[3]
//                double y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(double a, const double x[3], double y[9], int iy0)
{
  if (!(a == 0.0)) {
    __m128d r;
    __m128d r1;
    int i;
    i = iy0 - 1;
    r = _mm_loadu_pd(&x[1]);
    r = _mm_mul_pd(_mm_set1_pd(a), r);
    r1 = _mm_loadu_pd(&y[i]);
    r = _mm_add_pd(r1, r);
    _mm_storeu_pd(&y[i], r);
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  if (!(a == 0.0)) {
    for (int k{0}; k < n; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
