//
// File: xgeqp3.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_rtwutil.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &A
//                ::coder::array<double, 1U> &tau
//                ::coder::array<int, 2U> &jpvt
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(::coder::array<double, 2U> &A, ::coder::array<double, 1U> &tau,
            ::coder::array<int, 2U> &jpvt)
{
  static const int offsets[4]{0, 1, 2, 3};
  ::coder::array<double, 1U> vn1;
  ::coder::array<double, 1U> vn2;
  ::coder::array<double, 1U> work;
  int i;
  int ix;
  int m_tmp;
  int n_tmp;
  int u1;
  m_tmp = A.size(0);
  n_tmp = A.size(1);
  ix = A.size(0);
  u1 = A.size(1);
  if (ix <= u1) {
    u1 = ix;
  }
  tau.set_size(u1);
  for (i = 0; i < u1; i++) {
    tau[i] = 0.0;
  }
  if ((A.size(0) == 0) || (A.size(1) == 0) || (u1 < 1)) {
    int lastc;
    jpvt.set_size(1, n_tmp);
    ix = (n_tmp / 4) << 2;
    lastc = ix - 4;
    for (int ii_tmp{0}; ii_tmp <= lastc; ii_tmp += 4) {
      _mm_storeu_si128(
          (__m128i *)&jpvt[ii_tmp],
          _mm_add_epi32(
              _mm_add_epi32(_mm_set1_epi32(ii_tmp),
                            _mm_loadu_si128((const __m128i *)&offsets[0])),
              _mm_set1_epi32(1)));
    }
    for (int ii_tmp{ix}; ii_tmp < n_tmp; ii_tmp++) {
      jpvt[ii_tmp] = ii_tmp + 1;
    }
  } else {
    double smax;
    int k;
    jpvt.set_size(1, n_tmp);
    work.set_size(n_tmp);
    vn1.set_size(n_tmp);
    vn2.set_size(n_tmp);
    for (k = 0; k < n_tmp; k++) {
      jpvt[k] = k + 1;
      work[k] = 0.0;
      smax = blas::xnrm2(m_tmp, A, k * m_tmp + 1);
      vn1[k] = smax;
      vn2[k] = smax;
    }
    for (int b_i{0}; b_i < u1; b_i++) {
      double s;
      double temp2;
      int ii;
      int ii_tmp;
      int ip1;
      int lastc;
      int mmi;
      int nmi;
      int pvt;
      ip1 = b_i + 2;
      ii_tmp = b_i * m_tmp;
      ii = ii_tmp + b_i;
      nmi = n_tmp - b_i;
      mmi = m_tmp - b_i;
      if (nmi < 1) {
        ix = -1;
      } else {
        ix = 0;
        if (nmi > 1) {
          smax = std::abs(vn1[b_i]);
          for (k = 2; k <= nmi; k++) {
            s = std::abs(vn1[(b_i + k) - 1]);
            if (s > smax) {
              ix = k - 1;
              smax = s;
            }
          }
        }
      }
      pvt = b_i + ix;
      if (pvt + 1 != b_i + 1) {
        ix = pvt * m_tmp;
        for (k = 0; k < m_tmp; k++) {
          lastc = ix + k;
          smax = A[lastc];
          i = ii_tmp + k;
          A[lastc] = A[i];
          A[i] = smax;
        }
        ix = jpvt[pvt];
        jpvt[pvt] = jpvt[b_i];
        jpvt[b_i] = ix;
        vn1[pvt] = vn1[b_i];
        vn2[pvt] = vn2[b_i];
      }
      if (b_i + 1 < m_tmp) {
        temp2 = A[ii];
        pvt = ii + 2;
        tau[b_i] = 0.0;
        if (mmi > 0) {
          smax = blas::xnrm2(mmi - 1, A, ii + 2);
          if (smax != 0.0) {
            s = rt_hypotd_snf(A[ii], smax);
            if (A[ii] >= 0.0) {
              s = -s;
            }
            if (std::abs(s) < 1.0020841800044864E-292) {
              __m128d r;
              ix = 0;
              i = ii + mmi;
              do {
                ix++;
                lastc = (((((i - ii) - 1) / 2) << 1) + ii) + 2;
                ii_tmp = lastc - 2;
                for (k = pvt; k <= ii_tmp; k += 2) {
                  r = _mm_loadu_pd(&A[k - 1]);
                  _mm_storeu_pd(
                      &A[k - 1],
                      _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
                }
                for (k = lastc; k <= i; k++) {
                  A[k - 1] = 9.9792015476736E+291 * A[k - 1];
                }
                s *= 9.9792015476736E+291;
                temp2 *= 9.9792015476736E+291;
              } while ((std::abs(s) < 1.0020841800044864E-292) && (ix < 20));
              s = rt_hypotd_snf(temp2, blas::xnrm2(mmi - 1, A, ii + 2));
              if (temp2 >= 0.0) {
                s = -s;
              }
              tau[b_i] = (s - temp2) / s;
              smax = 1.0 / (temp2 - s);
              for (k = pvt; k <= ii_tmp; k += 2) {
                r = _mm_loadu_pd(&A[k - 1]);
                _mm_storeu_pd(&A[k - 1], _mm_mul_pd(_mm_set1_pd(smax), r));
              }
              for (k = lastc; k <= i; k++) {
                A[k - 1] = smax * A[k - 1];
              }
              for (k = 0; k < ix; k++) {
                s *= 1.0020841800044864E-292;
              }
              temp2 = s;
            } else {
              tau[b_i] = (s - A[ii]) / s;
              smax = 1.0 / (A[ii] - s);
              i = ii + mmi;
              ix = (((((i - ii) - 1) / 2) << 1) + ii) + 2;
              lastc = ix - 2;
              for (k = pvt; k <= lastc; k += 2) {
                __m128d r;
                r = _mm_loadu_pd(&A[k - 1]);
                _mm_storeu_pd(&A[k - 1], _mm_mul_pd(_mm_set1_pd(smax), r));
              }
              for (k = ix; k <= i; k++) {
                A[k - 1] = smax * A[k - 1];
              }
              temp2 = s;
            }
          }
        }
        A[ii] = temp2;
      } else {
        tau[b_i] = 0.0;
      }
      if (b_i + 1 < n_tmp) {
        int jA;
        int lastv;
        temp2 = A[ii];
        A[ii] = 1.0;
        jA = (ii + m_tmp) + 1;
        if (tau[b_i] != 0.0) {
          boolean_T exitg2;
          lastv = mmi - 1;
          ix = (ii + mmi) - 1;
          while ((lastv + 1 > 0) && (A[ix] == 0.0)) {
            lastv--;
            ix--;
          }
          lastc = nmi - 2;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            ix = jA + lastc * m_tmp;
            k = ix;
            do {
              exitg1 = 0;
              if (k <= ix + lastv) {
                if (A[k - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }
        if (lastv + 1 > 0) {
          if (lastc + 1 != 0) {
            for (ii_tmp = 0; ii_tmp <= lastc; ii_tmp++) {
              work[ii_tmp] = 0.0;
            }
            ii_tmp = 0;
            i = jA + m_tmp * lastc;
            for (pvt = jA; m_tmp < 0 ? pvt >= i : pvt <= i; pvt += m_tmp) {
              smax = 0.0;
              ix = pvt + lastv;
              for (k = pvt; k <= ix; k++) {
                smax += A[k - 1] * A[(ii + k) - pvt];
              }
              work[ii_tmp] = work[ii_tmp] + smax;
              ii_tmp++;
            }
          }
          if (!(-tau[b_i] == 0.0)) {
            for (ii_tmp = 0; ii_tmp <= lastc; ii_tmp++) {
              if (work[ii_tmp] != 0.0) {
                smax = work[ii_tmp] * -tau[b_i];
                i = lastv + jA;
                for (ix = jA; ix <= i; ix++) {
                  A[ix - 1] = A[ix - 1] + A[(ii + ix) - jA] * smax;
                }
              }
              jA += m_tmp;
            }
          }
        }
        A[ii] = temp2;
      }
      for (ii_tmp = ip1; ii_tmp <= n_tmp; ii_tmp++) {
        ix = b_i + (ii_tmp - 1) * m_tmp;
        smax = vn1[ii_tmp - 1];
        if (smax != 0.0) {
          s = std::abs(A[ix]) / smax;
          s = 1.0 - s * s;
          if (s < 0.0) {
            s = 0.0;
          }
          temp2 = smax / vn2[ii_tmp - 1];
          temp2 = s * (temp2 * temp2);
          if (temp2 <= 1.4901161193847656E-8) {
            if (b_i + 1 < m_tmp) {
              smax = blas::xnrm2(mmi - 1, A, ix + 2);
              vn1[ii_tmp - 1] = smax;
              vn2[ii_tmp - 1] = smax;
            } else {
              vn1[ii_tmp - 1] = 0.0;
              vn2[ii_tmp - 1] = 0.0;
            }
          } else {
            vn1[ii_tmp - 1] = smax * std::sqrt(s);
          }
        }
      }
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for xgeqp3.cpp
//
// [EOF]
//
