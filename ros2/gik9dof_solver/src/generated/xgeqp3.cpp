//
// xgeqp3.cpp
//
// Code generation for function 'xgeqp3'
//

// Include files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(array<double, 2U> &A, array<double, 1U> &tau, array<int, 2U> &jpvt)
{
  array<double, 1U> vn1;
  array<double, 1U> vn2;
  array<double, 1U> work;
  int i;
  int knt;
  int m_tmp;
  int minmana;
  int n_tmp;
  bool guard1;
  m_tmp = A.size(0);
  n_tmp = A.size(1);
  knt = A.size(0);
  minmana = A.size(1);
  if (knt <= minmana) {
    minmana = knt;
  }
  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }
  guard1 = false;
  if ((A.size(0) == 0) || (A.size(1) == 0)) {
    guard1 = true;
  } else {
    knt = A.size(0);
    minmana = A.size(1);
    if (knt <= minmana) {
      minmana = knt;
    }
    if (minmana < 1) {
      guard1 = true;
    } else {
      double smax;
      int k;
      int minmn;
      jpvt.set_size(1, n_tmp);
      knt = A.size(0);
      minmn = A.size(1);
      if (knt <= minmn) {
        minmn = knt;
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
        int lastc;
        int mmi;
        int nmi;
        int pvt;
        ip1 = b_i + 2;
        lastc = b_i * m_tmp;
        ii = lastc + b_i;
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
            knt = minmana + k;
            smax = A[knt];
            i = lastc + k;
            A[knt] = A[i];
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
          minmana = ii + 2;
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
                knt = 0;
                i = ii + mmi;
                do {
                  knt++;
                  for (k = minmana; k <= i; k++) {
                    A[k - 1] = 9.9792015476736E+291 * A[k - 1];
                  }
                  smax *= 9.9792015476736E+291;
                  temp2 *= 9.9792015476736E+291;
                } while ((std::abs(smax) < 1.0020841800044864E-292) &&
                         (knt < 20));
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
                for (k = minmana; k <= i; k++) {
                  A[k - 1] = s * A[k - 1];
                }
                for (k = 0; k < knt; k++) {
                  smax *= 1.0020841800044864E-292;
                }
                temp2 = smax;
              } else {
                tau[b_i] = (smax - A[ii]) / smax;
                s = 1.0 / (A[ii] - smax);
                i = ii + mmi;
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
            bool exitg2;
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
              for (knt = 0; knt <= lastc; knt++) {
                work[knt] = 0.0;
              }
              knt = 0;
              i = jA + m_tmp * lastc;
              for (pvt = jA; m_tmp < 0 ? pvt >= i : pvt <= i; pvt += m_tmp) {
                smax = 0.0;
                minmana = pvt + lastv;
                for (k = pvt; k <= minmana; k++) {
                  smax += A[k - 1] * A[(ii + k) - pvt];
                }
                work[knt] = work[knt] + smax;
                knt++;
              }
            }
            if (!(-tau[b_i] == 0.0)) {
              for (knt = 0; knt <= lastc; knt++) {
                if (work[knt] != 0.0) {
                  smax = work[knt] * -tau[b_i];
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
        for (knt = ip1; knt <= n_tmp; knt++) {
          minmana = b_i + (knt - 1) * m_tmp;
          smax = vn1[knt - 1];
          if (smax != 0.0) {
            s = std::abs(A[minmana]) / smax;
            s = 1.0 - s * s;
            if (s < 0.0) {
              s = 0.0;
            }
            temp2 = smax / vn2[knt - 1];
            temp2 = s * (temp2 * temp2);
            if (temp2 <= 1.4901161193847656E-8) {
              if (b_i + 1 < m_tmp) {
                smax = blas::xnrm2(mmi - 1, A, minmana + 2);
                vn1[knt - 1] = smax;
                vn2[knt - 1] = smax;
              } else {
                vn1[knt - 1] = 0.0;
                vn2[knt - 1] = 0.0;
              }
            } else {
              vn1[knt - 1] = smax * std::sqrt(s);
            }
          }
        }
      }
    }
  }
  if (guard1) {
    jpvt.set_size(1, n_tmp);
    for (knt = 0; knt < n_tmp; knt++) {
      jpvt[knt] = knt + 1;
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xgeqp3.cpp)
