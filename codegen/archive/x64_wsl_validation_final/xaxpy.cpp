//
// File: xaxpy.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <cstring>
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
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
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
