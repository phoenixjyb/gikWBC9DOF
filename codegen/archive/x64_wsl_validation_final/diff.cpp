//
// File: diff.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "diff.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &x
//                ::coder::array<double, 1U> &y
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void diff(const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y)
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
} // namespace gik9dof

//
// File trailer for diff.cpp
//
// [EOF]
//
