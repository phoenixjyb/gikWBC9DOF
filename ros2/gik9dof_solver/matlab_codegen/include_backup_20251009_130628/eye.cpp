//
// File: eye.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "eye.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double varargin_1
//                array<double, 2U> &b_I
// Return Type  : void
//
namespace coder {
void eye(double varargin_1, array<double, 2U> &b_I)
{
  double t;
  int loop_ub_tmp;
  int m_tmp;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }
  m_tmp = static_cast<int>(t);
  b_I.set_size(static_cast<int>(t), static_cast<int>(t));
  loop_ub_tmp = static_cast<int>(t) * static_cast<int>(t);
  for (int i{0}; i < loop_ub_tmp; i++) {
    b_I[i] = 0.0;
  }
  if (static_cast<int>(t) > 0) {
    for (loop_ub_tmp = 0; loop_ub_tmp < m_tmp; loop_ub_tmp++) {
      b_I[loop_ub_tmp + b_I.size(0) * loop_ub_tmp] = 1.0;
    }
  }
}

} // namespace coder

//
// File trailer for eye.cpp
//
// [EOF]
//
