//
// File: strcmp.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "strcmp.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const char a_data[]
//                const int a_size[2]
//                const ::coder::array<char, 2U> &b
// Return Type  : boolean_T
//
namespace gik9dof {
namespace coder {
namespace internal {
boolean_T b_strcmp(const char a_data[], const int a_size[2],
                   const ::coder::array<char, 2U> &b)
{
  boolean_T b_b;
  boolean_T b_bool;
  b_bool = false;
  b_b = (a_size[1] == 0);
  if (b_b && (b.size(1) == 0)) {
    b_bool = true;
  } else if (a_size[1] == b.size(1)) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr <= b.size(1) - 1) {
        if (a_data[kstr] != b[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

//
// Arguments    : const char a_data[]
//                const int a_size[2]
//                const ::coder::array<char, 2U> &b
// Return Type  : boolean_T
//
boolean_T c_strcmp(const char a_data[], const int a_size[2],
                   const ::coder::array<char, 2U> &b)
{
  int nb;
  boolean_T b_b;
  boolean_T b_bool;
  b_bool = false;
  nb = b.size(0) * b.size(1);
  b_b = (a_size[1] == 0);
  if (b_b && ((b.size(0) == 0) || (b.size(1) == 0))) {
    b_bool = true;
  } else if ((b.size(0) == 1) && (a_size[1] == b.size(1))) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr <= nb - 1) {
        if (a_data[kstr] != b[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for strcmp.cpp
//
// [EOF]
//
