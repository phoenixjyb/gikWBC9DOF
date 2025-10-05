//
// strcmp.cpp
//
// Code generation for function 'strcmp'
//

// Include files
#include "strcmp.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
namespace internal {
bool b_strcmp(const char a_data[], const int a_size[2],
              const array<char, 2U> &b)
{
  int nb;
  bool b_b;
  bool b_bool;
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

bool b_strcmp(const char a_data[], const int a_size[2], const char b_data[],
              const int b_size[2])
{
  bool b;
  bool b_bool;
  b_bool = false;
  b = (a_size[1] == 0);
  if (b && (b_size[1] == 0)) {
    b_bool = true;
  } else if (a_size[1] == b_size[1]) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr <= b_size[1] - 1) {
        if (a_data[kstr] != b_data[kstr]) {
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

// End of code generation (strcmp.cpp)
