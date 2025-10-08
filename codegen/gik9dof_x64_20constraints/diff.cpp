//
// File: diff.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "diff.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const array<double, 2U> &x
//                array<double, 1U> &y
// Return Type  : void
//
namespace coder {
void diff(const array<double, 2U> &x, array<double, 1U> &y)
{
  int i;
  i = x.size(0);
  y.set_size(x.size(0));
  if (x.size(0) != 0) {
    int scalarLB;
    int vectorUB;
    scalarLB = (x.size(0) / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int s{0}; s <= vectorUB; s += 2) {
      _mm_storeu_pd(&y[s],
                    _mm_sub_pd(_mm_loadu_pd(&x[s + i]), _mm_loadu_pd(&x[s])));
    }
    for (int s{scalarLB}; s < i; s++) {
      y[s] = x[s + i] - x[s];
    }
  }
}

} // namespace coder

//
// File trailer for diff.cpp
//
// [EOF]
//
