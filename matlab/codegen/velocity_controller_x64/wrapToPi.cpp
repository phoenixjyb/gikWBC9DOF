//
// File: wrapToPi.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:21:02
//

// Include Files
#include "wrapToPi.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double &lambda
// Return Type  : void
//
namespace gik9dof_velocity {
namespace coder {
void wrapToPi(double &lambda)
{
  double tmp_data;
  int trueCount;
  bool b;
  trueCount = 0;
  b = ((lambda < -3.1415926535897931) || (lambda > 3.1415926535897931));
  if (b) {
    double q;
    trueCount = 1;
    q = lambda + 3.1415926535897931;
    if (q == 0.0) {
      tmp_data = 0.0;
    } else {
      bool rEQ0;
      tmp_data = std::fmod(q, 6.2831853071795862);
      rEQ0 = (tmp_data == 0.0);
      if (!rEQ0) {
        q = std::abs(q / 6.2831853071795862);
        rEQ0 =
            (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }
      if (rEQ0) {
        tmp_data = 0.0;
      } else if (tmp_data < 0.0) {
        tmp_data += 6.2831853071795862;
      }
    }
  }
  for (int i{0}; i < trueCount; i++) {
    if ((tmp_data == 0.0) && (lambda + 3.1415926535897931 > 0.0)) {
      tmp_data = 6.2831853071795862;
    }
  }
  if (b) {
    lambda = tmp_data - 3.1415926535897931;
  }
}

} // namespace coder
} // namespace gik9dof_velocity

//
// File trailer for wrapToPi.cpp
//
// [EOF]
//
