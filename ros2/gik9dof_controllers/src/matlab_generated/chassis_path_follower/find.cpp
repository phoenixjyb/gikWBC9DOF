//
// File: find.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "find.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const bool x_data[]
//                int x_size
//                int i_data[]
// Return Type  : int
//
namespace gik9dof {
namespace coder {
int eml_find(const bool x_data[], int x_size, int i_data[])
{
  int i_size;
  int idx;
  int ii;
  bool exitg1;
  i_size = (x_size >= 1);
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x_size - 1)) {
    if (x_data[ii]) {
      idx = 1;
      i_data[0] = ii + 1;
      exitg1 = true;
    } else {
      ii++;
    }
  }
  if (i_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else {
    i_size = (idx >= 1);
  }
  return i_size;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for find.cpp
//
// [EOF]
//
