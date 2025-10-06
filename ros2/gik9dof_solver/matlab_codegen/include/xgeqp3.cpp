//
// File: xgeqp3.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
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
  int ii_tmp;
  int lastc;
  int m_tmp;
  int minmana;
  int n_tmp;
  boolean_T guard1;
  m_tmp = A.size(0);
  n_tmp = A.size(1);
  lastc = A.size(0);
  minmana = A.size(1);
  if (lastc <= minmana) {
    minmana = lastc;
  }
  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }
  guard1 = false;
  if ((A.size(0) == 0) || (A.size(1) == 0)) {
    guard1 = true;
  } else {
    lastc = A.size(0);
    minmana = A.size(1);
    if (lastc <= minmana) {
      minmana = lastc;
    }
    if (minmana < 1) {
      guard1 = true;
    } else {
      double smax;
      int k;
      int minmn;
      jpvt.set_size(1, n_tmp);
      lastc = A.size(0);
      minmn = A.size(1);
      if (lastc <= minmn) {
        minmn = lastc;
      }
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
      for (int b_i{0}; b_i < minmn; b_i++) {
        double s;
        double temp2;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        int pvt;
        ip1 = b_i + 2;
        ii_tmp = b_i * m_tmp;
        ii = ii_tmp + b_i;
        nmi = n_tmp - b_i;
        mmi = m_tmp - b_i;
        if (nmi < 1) {
          minmana = -1;
        } else {
          minmana = 0;
          if (nmi > 1) {
            smax = std::abs(vn1[b_i]);
            for (k = 2; k <= nmi; k++) {
              s = std::abs(vn1[(b_i + k) - 1]);
              if (s > smax) {
                minmana = k - 1;
                smax = s;
              }
            }
          }
        }
        pvt = b_i + minmana;
        if (pvt + 1 != b_i + 1) {
          minmana = pvt * m_tmp;
          for (k = 0; k < m_tmp; k++) {
            lastc = minmana + k;
            smax = A[lastc];
            i = ii_tmp + k;
            A[lastc] = A[i];
            A[i] = smax;
          }
          minmana = jpvt[pvt];
          jpvt[pvt] = jpvt[b_i];
          jpvt[b_i] = minmana;
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
              s = std::abs(A[ii]);
              smax = std::abs(smax);
              if (s < smax) {
                s /= smax;
                smax *= std::sqrt(s * s + 1.0);
              } else if (s > smax) {
                smax /= s;
                smax = s * std::sqrt(smax * smax + 1.0);
              } else if (std::isnan(smax)) {
                smax = rtNaN;
              } else {
                smax = s * 1.4142135623730951;
              }
              if (A[ii] >= 0.0) {
                smax = -smax;
              }
              if (std::abs(smax) < 1.0020841800044864E-292) {
                __m128d r;
                minmana = 0;
                i = ii + mmi;
                do {
                  minmana++;
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
                  smax *= 9.9792015476736E+291;
                  temp2 *= 9.9792015476736E+291;
                } while ((std::abs(smax) < 1.0020841800044864E-292) &&
                         (minmana < 20));
                s = std::abs(temp2);
                smax = std::abs(blas::xnrm2(mmi - 1, A, ii + 2));
                if (s < smax) {
                  s /= smax;
                  smax *= std::sqrt(s * s + 1.0);
                } else if (s > smax) {
                  smax /= s;
                  smax = s * std::sqrt(smax * smax + 1.0);
                } else if (std::isnan(smax)) {
                  smax = rtNaN;
                } else {
                  smax = s * 1.4142135623730951;
                }
                if (temp2 >= 0.0) {
                  smax = -smax;
                }
                tau[b_i] = (smax - temp2) / smax;
                s = 1.0 / (temp2 - smax);
                for (k = pvt; k <= ii_tmp; k += 2) {
                  r = _mm_loadu_pd(&A[k - 1]);
                  _mm_storeu_pd(&A[k - 1], _mm_mul_pd(_mm_set1_pd(s), r));
                }
                for (k = lastc; k <= i; k++) {
                  A[k - 1] = s * A[k - 1];
                }
                for (k = 0; k < minmana; k++) {
                  smax *= 1.0020841800044864E-292;
                }
                temp2 = smax;
              } else {
                tau[b_i] = (smax - A[ii]) / smax;
                s = 1.0 / (A[ii] - smax);
                i = ii + mmi;
                minmana = (((((i - ii) - 1) / 2) << 1) + ii) + 2;
                lastc = minmana - 2;
                for (k = pvt; k <= lastc; k += 2) {
                  __m128d r;
                  r = _mm_loadu_pd(&A[k - 1]);
                  _mm_storeu_pd(&A[k - 1], _mm_mul_pd(_mm_set1_pd(s), r));
                }
                for (k = minmana; k <= i; k++) {
                  A[k - 1] = s * A[k - 1];
                }
                temp2 = smax;
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
            minmana = (ii + mmi) - 1;
            while ((lastv + 1 > 0) && (A[minmana] == 0.0)) {
              lastv--;
              minmana--;
            }
            lastc = nmi - 2;
            exitg2 = false;
            while ((!exitg2) && (lastc + 1 > 0)) {
              int exitg1;
              minmana = jA + lastc * m_tmp;
              k = minmana;
              do {
                exitg1 = 0;
                if (k <= minmana + lastv) {
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
                minmana = pvt + lastv;
                for (k = pvt; k <= minmana; k++) {
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
                  for (minmana = jA; minmana <= i; minmana++) {
                    A[minmana - 1] =
                        A[minmana - 1] + A[(ii + minmana) - jA] * smax;
                  }
                }
                jA += m_tmp;
              }
            }
          }
          A[ii] = temp2;
        }
        for (ii_tmp = ip1; ii_tmp <= n_tmp; ii_tmp++) {
          minmana = b_i + (ii_tmp - 1) * m_tmp;
          smax = vn1[ii_tmp - 1];
          if (smax != 0.0) {
            s = std::abs(A[minmana]) / smax;
            s = 1.0 - s * s;
            if (s < 0.0) {
              s = 0.0;
            }
            temp2 = smax / vn2[ii_tmp - 1];
            temp2 = s * (temp2 * temp2);
            if (temp2 <= 1.4901161193847656E-8) {
              if (b_i + 1 < m_tmp) {
                smax = blas::xnrm2(mmi - 1, A, minmana + 2);
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
  if (guard1) {
    jpvt.set_size(1, n_tmp);
    minmana = (n_tmp / 4) << 2;
    lastc = minmana - 4;
    for (ii_tmp = 0; ii_tmp <= lastc; ii_tmp += 4) {
      _mm_storeu_si128(
          (__m128i *)&jpvt[ii_tmp],
          _mm_add_epi32(
              _mm_add_epi32(_mm_set1_epi32(ii_tmp),
                            _mm_loadu_si128((const __m128i *)&offsets[0])),
              _mm_set1_epi32(1)));
    }
    for (ii_tmp = minmana; ii_tmp < n_tmp; ii_tmp++) {
      jpvt[ii_tmp] = ii_tmp + 1;
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
