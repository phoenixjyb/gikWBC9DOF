//
// File: sqrt.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

// Include Files
#include "sqrt.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : creal_T &x
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace scalar {
void b_sqrt(creal_T &x)
{
  double absxd2;
  double absxi;
  double xi;
  double xr;
  xr = x.re;
  xi = x.im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      absxi = 0.0;
      absxd2 = std::sqrt(-xr);
    } else {
      absxi = std::sqrt(xr);
      absxd2 = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      absxi = std::sqrt(-xi / 2.0);
      absxd2 = -absxi;
    } else {
      absxi = std::sqrt(xi / 2.0);
      absxd2 = absxi;
    }
  } else if (std::isnan(xr)) {
    absxi = rtNaN;
    absxd2 = rtNaN;
  } else if (std::isnan(xi)) {
    absxi = rtNaN;
    absxd2 = rtNaN;
  } else if (std::isinf(xi)) {
    absxi = std::abs(xi);
    absxd2 = xi;
  } else if (std::isinf(xr)) {
    if (xr < 0.0) {
      absxi = 0.0;
      absxd2 = xi * -xr;
    } else {
      absxi = xr;
      absxd2 = 0.0;
    }
  } else {
    double absxr;
    absxr = std::abs(xr);
    absxi = std::abs(xi);
    if ((absxr > 4.4942328371557893E+307) ||
        (absxi > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      absxd2 = absxi * 0.5;
      if (absxr < absxd2) {
        double a;
        a = absxr / absxd2;
        absxd2 *= std::sqrt(a * a + 1.0);
      } else if (absxr > absxd2) {
        absxd2 /= absxr;
        absxd2 = absxr * std::sqrt(absxd2 * absxd2 + 1.0);
      } else {
        absxd2 = absxr * 1.4142135623730951;
      }
      if (absxd2 > absxr) {
        absxi = std::sqrt(absxd2) * std::sqrt(absxr / absxd2 + 1.0);
      } else {
        absxi = std::sqrt(absxd2) * 1.4142135623730951;
      }
    } else {
      if (absxr < absxi) {
        double a;
        a = absxr / absxi;
        absxd2 = absxi * std::sqrt(a * a + 1.0);
      } else if (absxr > absxi) {
        absxd2 = absxi / absxr;
        absxd2 = absxr * std::sqrt(absxd2 * absxd2 + 1.0);
      } else {
        absxd2 = absxr * 1.4142135623730951;
      }
      absxi = std::sqrt((absxd2 + absxr) * 0.5);
    }
    if (xr > 0.0) {
      absxd2 = 0.5 * (xi / absxi);
    } else {
      if (xi < 0.0) {
        absxd2 = -absxi;
      } else {
        absxd2 = absxi;
      }
      absxi = 0.5 * (xi / absxd2);
    }
  }
  x.re = absxi;
  x.im = absxd2;
}

} // namespace scalar
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for sqrt.cpp
//
// [EOF]
//
