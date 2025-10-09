//
// File: xnrm2.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

// Include Files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int n
//                const ::coder::array<double, 2U> &x
//                int ix0
// Return Type  : double
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
double xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int k{ix0}; k < kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const double x[9]
//                int ix0
// Return Type  : double
//
double xnrm2(int n, const double x[9], int ix0)
{
  double scale;
  double y;
  int kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (int k{ix0}; k < kend; k++) {
    double absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * std::sqrt(y);
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
double xnrm2(const double x[3])
{
  double scale;
  double y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int k{2}; k < 4; k++) {
    double absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * std::sqrt(y);
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
