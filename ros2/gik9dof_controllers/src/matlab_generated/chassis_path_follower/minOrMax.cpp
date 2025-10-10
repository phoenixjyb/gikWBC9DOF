//
// File: minOrMax.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[3]
// Return Type  : double
//
namespace gik9dof {
namespace coder {
namespace internal {
double maximum(const double x[3])
{
  double ex;
  int idx;
  int k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      double d;
      d = x[k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }
  return ex;
}

//
// Arguments    : const double x_data[]
//                int x_size
//                int &idx
// Return Type  : double
//
double minimum(const double x_data[], int x_size, int &idx)
{
  double ex;
  if (x_size <= 2) {
    if (x_size == 1) {
      ex = x_data[0];
      idx = 1;
    } else {
      ex = x_data[x_size - 1];
      if ((x_data[0] > ex) || (std::isnan(x_data[0]) && (!std::isnan(ex)))) {
        idx = x_size;
      } else {
        ex = x_data[0];
        idx = 1;
      }
    }
  } else {
    int k;
    if (!std::isnan(x_data[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x_size)) {
        if (!std::isnan(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x_data[0];
      idx = 1;
    } else {
      int i;
      ex = x_data[idx - 1];
      i = idx + 1;
      for (k = i; k <= x_size; k++) {
        double d;
        d = x_data[k - 1];
        if (ex > d) {
          ex = d;
          idx = k;
        }
      }
    }
  }
  return ex;
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
