//
// File: ismember.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "ismember.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "coder_array.h"
#include "omp.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
namespace gik9dof {
namespace coder {
static int bsearchni(int k, const ::coder::array<double, 1U> &x,
                     const double s[22]);

}
} // namespace gik9dof

// Function Definitions
//
// Arguments    : int k
//                const ::coder::array<double, 1U> &x
//                const double s[22]
// Return Type  : int
//
namespace gik9dof {
namespace coder {
static int bsearchni(int k, const ::coder::array<double, 1U> &x,
                     const double s[22])
{
  double b_x;
  int idx;
  int ihi;
  int ilo;
  bool exitg1;
  b_x = x[k - 1];
  idx = 0;
  ilo = 1;
  ihi = 22;
  exitg1 = false;
  while ((!exitg1) && (ihi >= ilo)) {
    int imid;
    imid = ((ilo >> 1) + (ihi >> 1)) - 1;
    if (((static_cast<unsigned int>(ilo) & 1U) == 1U) &&
        ((static_cast<unsigned int>(ihi) & 1U) == 1U)) {
      imid++;
    }
    if (b_x == s[imid]) {
      idx = imid + 1;
      exitg1 = true;
    } else {
      bool p;
      if (std::isnan(s[imid])) {
        p = !std::isnan(b_x);
      } else if (std::isnan(b_x)) {
        p = false;
      } else {
        p = (b_x < s[imid]);
      }
      if (p) {
        ihi = imid;
      } else {
        ilo = imid + 2;
      }
    }
  }
  if (idx > 0) {
    idx--;
    while ((idx > 0) && (b_x == s[idx - 1])) {
      idx--;
    }
    idx++;
  }
  return idx;
}

//
// Arguments    : const ::coder::array<double, 1U> &a
//                const double s[22]
//                ::coder::array<bool, 1U> &tf
//                ::coder::array<int, 1U> &loc
// Return Type  : void
//
void isMember(const ::coder::array<double, 1U> &a, const double s[22],
              ::coder::array<bool, 1U> &tf, ::coder::array<int, 1U> &loc)
{
  double ss[22];
  int k;
  int n;
  int na_tmp;
  na_tmp = a.size(0);
  tf.set_size(a.size(0));
  loc.set_size(a.size(0));
  for (k = 0; k < na_tmp; k++) {
    tf[k] = false;
    loc[k] = 0;
  }
  if (a.size(0) <= 9) {
    for (int j{0}; j < na_tmp; j++) {
      bool exitg1;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k < 22)) {
        if (a[j] == s[k]) {
          tf[j] = true;
          loc[j] = k + 1;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
  } else {
    bool exitg1;
    bool y;
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 21)) {
      double d;
      d = s[k + 1];
      if ((s[k] <= d) || std::isnan(d)) {
        k++;
      } else {
        y = false;
        exitg1 = true;
      }
    }
    if (!y) {
      int ssidx[22];
      ::std::copy(&s[0], &s[22], &ss[0]);
      internal::sort(ss, ssidx);
      k = a.size(0);
#pragma omp parallel for num_threads(omp_get_max_threads()) private(n)

      for (int b_k = 0; b_k < k; b_k++) {
        n = bsearchni(b_k + 1, a, ss);
        if (n > 0) {
          tf[b_k] = true;
          loc[b_k] = ssidx[n - 1];
        }
      }
    } else {
      k = a.size(0);
#pragma omp parallel for num_threads(omp_get_max_threads()) private(n)

      for (int b_k = 0; b_k < k; b_k++) {
        n = bsearchni(b_k + 1, a, s);
        if (n > 0) {
          tf[b_k] = true;
          loc[b_k] = n;
        }
      }
    }
  }
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for ismember.cpp
//
// [EOF]
//
