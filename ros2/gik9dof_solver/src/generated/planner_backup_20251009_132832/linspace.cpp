//
// File: linspace.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

// Include Files
#include "linspace.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double d2
//                double n
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void linspace(double d2, double n, double y_data[], int y_size[2])
{
  int i;
  y_size[0] = 1;
  i = static_cast<int>(std::floor(n));
  y_size[1] = i;
  y_data[i - 1] = d2;
  y_data[0] = 0.0;
  if (i >= 3) {
    double delta1;
    delta1 = d2 / (static_cast<double>(i) - 1.0);
    for (int k{0}; k <= i - 3; k++) {
      y_data[k + 1] = (static_cast<double>(k) + 1.0) * delta1;
    }
  }
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for linspace.cpp
//
// [EOF]
//
