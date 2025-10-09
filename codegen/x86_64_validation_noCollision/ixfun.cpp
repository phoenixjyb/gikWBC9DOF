//
// File: ixfun.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "ixfun.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &a
//                const ::coder::array<double, 1U> &b
//                ::coder::array<double, 1U> &c
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
void expand_max(const ::coder::array<double, 1U> &a,
                const ::coder::array<double, 1U> &b,
                ::coder::array<double, 1U> &c)
{
  int csz_idx_0;
  int u1;
  csz_idx_0 = a.size(0);
  u1 = b.size(0);
  if (csz_idx_0 <= u1) {
    u1 = csz_idx_0;
  }
  if (b.size(0) == 1) {
    csz_idx_0 = a.size(0);
  } else if (a.size(0) == 1) {
    csz_idx_0 = b.size(0);
  } else {
    csz_idx_0 = u1;
  }
  c.set_size(csz_idx_0);
  if (csz_idx_0 != 0) {
    boolean_T b1;
    boolean_T b_b;
    b_b = (a.size(0) != 1);
    b1 = (b.size(0) != 1);
    csz_idx_0--;
    for (u1 = 0; u1 <= csz_idx_0; u1++) {
      c[u1] = std::fmax(a[b_b * u1], b[b1 * u1]);
    }
  }
}

//
// Arguments    : const ::coder::array<double, 1U> &a
//                const ::coder::array<double, 1U> &b
//                ::coder::array<double, 1U> &c
// Return Type  : void
//
void expand_min(const ::coder::array<double, 1U> &a,
                const ::coder::array<double, 1U> &b,
                ::coder::array<double, 1U> &c)
{
  int csz_idx_0;
  int u1;
  csz_idx_0 = a.size(0);
  u1 = b.size(0);
  if (csz_idx_0 <= u1) {
    u1 = csz_idx_0;
  }
  if (b.size(0) == 1) {
    csz_idx_0 = a.size(0);
  } else if (a.size(0) == 1) {
    csz_idx_0 = b.size(0);
  } else {
    csz_idx_0 = u1;
  }
  c.set_size(csz_idx_0);
  if (csz_idx_0 != 0) {
    boolean_T b1;
    boolean_T b_b;
    b_b = (a.size(0) != 1);
    b1 = (b.size(0) != 1);
    csz_idx_0--;
    for (u1 = 0; u1 <= csz_idx_0; u1++) {
      c[u1] = std::fmin(a[b_b * u1], b[b1 * u1]);
    }
  }
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for ixfun.cpp
//
// [EOF]
//
