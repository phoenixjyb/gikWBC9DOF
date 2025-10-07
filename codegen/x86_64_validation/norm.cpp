//
// File: norm.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "norm.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace gik9dof {
namespace coder {
double b_norm(const ::coder::array<double, 1U> &x)
{
  double y;
  if (x.size(0) == 0) {
    y = 0.0;
  } else {
    y = 0.0;
    if (x.size(0) == 1) {
      y = std::abs(x[0]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = x.size(0);
      for (int k{0}; k < kend; k++) {
        double absxk;
        absxk = std::abs(x[k]);
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

} // namespace coder
} // namespace gik9dof

//
// File trailer for norm.cpp
//
// [EOF]
//
