//
// File: find.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "find.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const bool x[9]
//                int i_data[]
// Return Type  : int
//
namespace gik9dof {
namespace coder {
int eml_find(const bool x[9], int i_data[])
{
  int i_size;
  int ii;
  bool exitg1;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 9)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 9) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
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
