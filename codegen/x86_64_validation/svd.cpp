//
// File: svd.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include "xzlangeM.h"
#include "xzlascl.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double A[9]
//                double U[9]
//                double s[3]
//                double V[9]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
void svd(const double A[9], double U[9], double s[3], double V[9])
{
  __m128d r;
  double b_A[9];
  double e[3];
  double work[3];
  double anrm;
  double cscale;
  double nrm;
  double rt;
  double snorm;
  double sqds;
  int i;
  int ii;
  int kase;
  int m;
  int qjj;
  int qp1;
  int qq;
  int qq_tmp;
  int scalarLB;
  bool doscale;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_A[i] = A[i];
    U[i] = 0.0;
    V[i] = 0.0;
  }
  doscale = false;
  anrm = reflapack::xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    reflapack::xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    reflapack::xzlascl(anrm, cscale, b_A);
  }
  for (int q{0}; q < 2; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qq_tmp = q + 3 * q;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = blas::xnrm2(3 - q, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        i = (qq_tmp - q) + 3;
        scalarLB = ((((i - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        qjj = scalarLB - 2;
        for (int k{qq}; k <= qjj; k += 2) {
          r = _mm_loadu_pd(&b_A[k - 1]);
          _mm_storeu_pd(&b_A[k - 1], _mm_mul_pd(_mm_set1_pd(nrm), r));
        }
        for (int k{scalarLB}; k <= i; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        i = (qq_tmp - q) + 3;
        scalarLB = ((((i - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        qjj = scalarLB - 2;
        for (int k{qq}; k <= qjj; k += 2) {
          r = _mm_loadu_pd(&b_A[k - 1]);
          _mm_storeu_pd(&b_A[k - 1], _mm_div_pd(r, _mm_set1_pd(s[q])));
        }
        for (int k{scalarLB}; k <= i; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[qq_tmp]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (kase = qp1; kase < 4; kase++) {
      qjj = q + 3 * (kase - 1);
      if (apply_transform) {
        blas::xaxpy(
            3 - q,
            -(blas::xdotc(3 - q, b_A, qq_tmp + 1, b_A, qjj + 1) / b_A[qq_tmp]),
            qq_tmp + 1, b_A, qjj + 1);
      }
      e[kase - 1] = b_A[qjj];
    }
    for (ii = q + 1; ii < 4; ii++) {
      kase = (ii + 3 * q) - 1;
      U[kase] = b_A[kase];
    }
    if (q + 1 <= 1) {
      nrm = blas::xnrm2(e);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        nrm = e[0];
        if (std::abs(e[0]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[0];
          scalarLB = ((((2 - q) / 2) << 1) + q) + 2;
          qjj = scalarLB - 2;
          for (int k{qp1}; k <= qjj; k += 2) {
            r = _mm_loadu_pd(&e[k - 1]);
            _mm_storeu_pd(&e[k - 1], _mm_mul_pd(_mm_set1_pd(nrm), r));
          }
          for (int k{scalarLB}; k < 4; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          scalarLB = ((((2 - q) / 2) << 1) + q) + 2;
          qjj = scalarLB - 2;
          for (int k{qp1}; k <= qjj; k += 2) {
            r = _mm_loadu_pd(&e[k - 1]);
            _mm_storeu_pd(&e[k - 1], _mm_div_pd(r, _mm_set1_pd(nrm)));
          }
          for (int k{scalarLB}; k < 4; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[1]++;
        e[0] = -e[0];
        for (ii = qp1; ii < 4; ii++) {
          work[ii - 1] = 0.0;
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(e[kase - 1], b_A, 3 * (kase - 1) + 2, work);
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(-e[kase - 1] / e[1], work, b_A, 3 * (kase - 1) + 2);
        }
      }
      for (ii = qp1; ii < 4; ii++) {
        V[ii - 1] = e[ii - 1];
      }
    }
  }
  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (int q{1}; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (s[q] != 0.0) {
      for (kase = qp1; kase < 4; kase++) {
        qjj = (q + 3 * (kase - 1)) + 1;
        blas::xaxpy(3 - q, -(blas::xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]),
                    qq + 1, U, qjj);
      }
      scalarLB = q + 3;
      qjj = q + 1;
      for (ii = q + 1; ii <= qjj; ii += 2) {
        i = (ii + 3 * q) - 1;
        r = _mm_loadu_pd(&U[i]);
        _mm_storeu_pd(&U[i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (ii = scalarLB; ii < 4; ii++) {
        kase = 3 * q + 2;
        U[kase] = -U[kase];
      }
      U[qq]++;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (int q{2}; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0)) {
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }
    V[3 * q] = 0.0;
    V[3 * q + 1] = 0.0;
    V[3 * q + 2] = 0.0;
    V[q + 3 * q] = 1.0;
  }
  qq = 0;
  snorm = 0.0;
  for (int q{0}; q < 3; q++) {
    nrm = s[q];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[q] = rt;
      if (q + 1 < 3) {
        e[q] /= nrm;
      }
      kase = 3 * q;
      scalarLB = kase + 3;
      qjj = kase + 1;
      for (int k{kase + 1}; k <= qjj; k += 2) {
        r = _mm_loadu_pd(&U[k - 1]);
        _mm_storeu_pd(&U[k - 1], _mm_mul_pd(_mm_set1_pd(nrm), r));
      }
      for (int k{scalarLB}; k <= scalarLB; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 3) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = std::abs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        s[q + 1] *= nrm;
        kase = 3 * (q + 1);
        scalarLB = kase + 3;
        qjj = kase + 1;
        for (int k{kase + 1}; k <= qjj; k += 2) {
          r = _mm_loadu_pd(&V[k - 1]);
          _mm_storeu_pd(&V[k - 1], _mm_mul_pd(_mm_set1_pd(nrm), r));
        }
        for (int k{scalarLB}; k <= scalarLB; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(s[q]), std::abs(e[q])));
  }
  while ((m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    qq_tmp = m + 1;
    ii = m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <=
           2.2204460492503131E-16 * (std::abs(s[ii - 1]) + std::abs(s[ii]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[ii - 1] = 0.0;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == m + 1) {
      kase = 4;
    } else {
      qjj = m + 2;
      kase = m + 2;
      exitg1 = false;
      while ((!exitg1) && (kase >= ii)) {
        qjj = kase;
        if (kase == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (kase < m + 2) {
            nrm = std::abs(e[kase - 1]);
          }
          if (kase > ii + 1) {
            nrm += std::abs(e[kase - 2]);
          }
          rt = std::abs(s[kase - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[kase - 1] = 0.0;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }
      if (qjj == ii) {
        kase = 3;
      } else if (qjj == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        ii = qjj;
      }
    }
    switch (kase) {
    case 1: {
      rt = e[m];
      e[m] = 0.0;
      for (int k{qq_tmp}; k >= ii + 1; k--) {
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        if (k > ii + 1) {
          rt = -sqds * e[0];
          e[0] *= sm;
        }
        blas::xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
      }
    } break;
    case 2: {
      rt = e[ii - 1];
      e[ii - 1] = 0.0;
      for (int k{ii + 1}; k <= m + 2; k++) {
        double b;
        double sm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = -sqds * b;
        e[k - 1] = b * sm;
        blas::xrot(U, 3 * (k - 1) + 1, 3 * (ii - 1) + 1, sm, sqds);
      }
    } break;
    case 3: {
      double b;
      double scale;
      double sm;
      nrm = s[m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s[m])),
                              std::abs(e[m])),
                    std::abs(s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = s[m] / scale;
      rt = e[m] / scale;
      sqds = s[ii] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        rt = std::sqrt(b * b + nrm);
        if (b < 0.0) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= qq_tmp; k++) {
        sm = blas::xrotg(rt, nrm, sqds);
        if (k > ii + 1) {
          e[0] = rt;
        }
        nrm = e[k - 1];
        b = s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        rt = sqds * s[k];
        s[k] *= sm;
        i = 3 * (k - 1) + 1;
        kase = 3 * k + 1;
        blas::xrot(V, i, kase, sm, sqds);
        s[k - 1] = sm * b + sqds * nrm;
        sm = blas::xrotg(s[k - 1], rt, sqds);
        b = e[k - 1];
        rt = sm * b + sqds * s[k];
        s[k] = -sqds * b + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        blas::xrot(U, i, kase, sm, sqds);
      }
      e[m] = rt;
      qq++;
    } break;
    default:
      if (s[ii] < 0.0) {
        s[ii] = -s[ii];
        kase = 3 * ii;
        scalarLB = kase + 3;
        qjj = kase + 1;
        for (int k{kase + 1}; k <= qjj; k += 2) {
          r = _mm_loadu_pd(&V[k - 1]);
          _mm_storeu_pd(&V[k - 1], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
        }
        for (int k{scalarLB}; k <= scalarLB; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = ii + 1;
      while ((ii + 1 < 3) && (s[ii] < s[qp1])) {
        rt = s[ii];
        s[ii] = s[qp1];
        s[qp1] = rt;
        i = 3 * ii + 1;
        kase = 3 * (ii + 1) + 1;
        blas::xswap(V, i, kase);
        blas::xswap(U, i, kase);
        ii = qp1;
        qp1++;
      }
      qq = 0;
      m--;
      break;
    }
  }
  if (doscale) {
    reflapack::b_xzlascl(cscale, anrm, s);
  }
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for svd.cpp
//
// [EOF]
//
