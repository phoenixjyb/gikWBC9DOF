//
// File: wrapToPi.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "wrapToPi.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double &lambda
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void wrapToPi(double &lambda)
{
  double tmp_data;
  int trueCount;
  bool b;
  trueCount = 0;
  b = ((lambda < -3.1415926535897931) || (lambda > 3.1415926535897931));
  if (b) {
    double varargin_1;
    trueCount = 1;
    varargin_1 = lambda + 3.1415926535897931;
    if (std::isnan(varargin_1) || std::isinf(varargin_1)) {
      tmp_data = rtNaN;
    } else if (varargin_1 == 0.0) {
      tmp_data = 0.0;
    } else {
      bool rEQ0;
      tmp_data = std::fmod(varargin_1, 6.2831853071795862);
      rEQ0 = (tmp_data == 0.0);
      if (!rEQ0) {
        double q;
        q = std::abs(varargin_1 / 6.2831853071795862);
        rEQ0 =
            !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }
      if (rEQ0) {
        tmp_data = 0.0;
      } else if (varargin_1 < 0.0) {
        tmp_data += 6.2831853071795862;
      }
    }
  }
  trueCount--;
  for (int i{0}; i <= trueCount; i++) {
    if ((tmp_data == 0.0) && (lambda + 3.1415926535897931 > 0.0)) {
      tmp_data = 6.2831853071795862;
    }
  }
  if (b) {
    lambda = tmp_data - 3.1415926535897931;
  }
}

//
// Arguments    : double lambda_data[]
// Return Type  : void
//
void wrapToPi(double lambda_data[])
{
  double tmp_data;
  double varargin_1;
  int trueCount;
  bool b;
  bool positiveInput_data;
  trueCount = 0;
  varargin_1 = lambda_data[0];
  b = ((varargin_1 < -3.1415926535897931) || (varargin_1 > 3.1415926535897931));
  if (b) {
    trueCount = 1;
    tmp_data = lambda_data[0] + 3.1415926535897931;
  }
  for (int i{0}; i < trueCount; i++) {
    positiveInput_data = (tmp_data > 0.0);
    varargin_1 = tmp_data;
    if (std::isnan(tmp_data) || std::isinf(tmp_data)) {
      tmp_data = rtNaN;
    } else if (tmp_data == 0.0) {
      tmp_data = 0.0;
    } else {
      bool rEQ0;
      tmp_data = std::fmod(tmp_data, 6.2831853071795862);
      rEQ0 = (tmp_data == 0.0);
      if (!rEQ0) {
        double q;
        q = std::abs(varargin_1 / 6.2831853071795862);
        rEQ0 =
            !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }
      if (rEQ0) {
        tmp_data = 0.0;
      } else if (varargin_1 < 0.0) {
        tmp_data += 6.2831853071795862;
      }
    }
  }
  trueCount--;
  for (int i{0}; i <= trueCount; i++) {
    if ((tmp_data == 0.0) && positiveInput_data) {
      tmp_data = 6.2831853071795862;
    }
  }
  if (b) {
    lambda_data[0] = tmp_data - 3.1415926535897931;
  }
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for wrapToPi.cpp
//
// [EOF]
//
