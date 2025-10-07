//
// File: xzlascl.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "xzlascl.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double cfrom
//                double cto
//                double A[3]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace reflapack {
void b_xzlascl(double cfrom, double cto, double A[3])
{
  double cfromc;
  double ctoc;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    __m128d r;
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    r = _mm_loadu_pd(&A[0]);
    _mm_storeu_pd(&A[0], _mm_mul_pd(r, _mm_set1_pd(mul)));
    A[2] *= mul;
  }
}

//
// Arguments    : double cfrom
//                double cto
//                double A[9]
// Return Type  : void
//
void xzlascl(double cfrom, double cto, double A[9])
{
  double cfromc;
  double ctoc;
  bool notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (int j{0}; j < 3; j++) {
      int offset;
      offset = j * 3 - 1;
      A[offset + 1] *= mul;
      A[offset + 2] *= mul;
      A[offset + 3] *= mul;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xzlascl.cpp
//
// [EOF]
//
