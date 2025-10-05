//
// mldivide.cpp
//
// Code generation for function 'mldivide'
//

// Include files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
namespace coder {
void mldivide(const array<double, 2U> &A, array<double, 1U> &B)
{
  array<double, 2U> b_A;
  array<double, 1U> b_B;
  array<double, 1U> tau;
  array<int, 2U> jpvt;
  if ((A.size(0) == 0) || (A.size(1) == 0) || (B.size(0) == 0)) {
    int minmn;
    minmn = A.size(1);
    B.set_size(A.size(1));
    for (int i{0}; i < minmn; i++) {
      B[i] = 0.0;
    }
  } else if (A.size(0) == A.size(1)) {
    double tol;
    int LDA_tmp;
    int i;
    int minmn;
    int n;
    int u0;
    u0 = A.size(0);
    n = A.size(1);
    if (u0 <= n) {
      n = u0;
    }
    u0 = B.size(0);
    if (u0 <= n) {
      n = u0;
    }
    LDA_tmp = A.size(0);
    b_A.set_size(A.size(0), A.size(1));
    minmn = A.size(0) * A.size(1);
    for (i = 0; i < minmn; i++) {
      b_A[i] = A[i];
    }
    jpvt.set_size(1, n);
    if (n > 0) {
      jpvt[0] = 1;
      minmn = 1;
      for (int k{2}; k <= n; k++) {
        minmn++;
        jpvt[k - 1] = minmn;
      }
    }
    if (n >= 1) {
      u0 = n - 1;
      if (u0 > n) {
        u0 = n;
      }
      for (int j{0}; j < u0; j++) {
        int b_tmp;
        int jp1j;
        int maxmn;
        int mmj_tmp;
        int rankA;
        mmj_tmp = n - j;
        b_tmp = j * (LDA_tmp + 1);
        jp1j = b_tmp + 2;
        if (mmj_tmp < 1) {
          minmn = -1;
        } else {
          minmn = 0;
          if (mmj_tmp > 1) {
            tol = std::abs(b_A[b_tmp]);
            for (int k{2}; k <= mmj_tmp; k++) {
              double s;
              s = std::abs(b_A[(b_tmp + k) - 1]);
              if (s > tol) {
                minmn = k - 1;
                tol = s;
              }
            }
          }
        }
        if (b_A[b_tmp + minmn] != 0.0) {
          if (minmn != 0) {
            maxmn = j + minmn;
            jpvt[j] = maxmn + 1;
            for (int k{0}; k < n; k++) {
              minmn = k * LDA_tmp;
              rankA = j + minmn;
              tol = b_A[rankA];
              i = maxmn + minmn;
              b_A[rankA] = b_A[i];
              b_A[i] = tol;
            }
          }
          i = b_tmp + mmj_tmp;
          for (int b_i{jp1j}; b_i <= i; b_i++) {
            b_A[b_i - 1] = b_A[b_i - 1] / b_A[b_tmp];
          }
        }
        minmn = b_tmp + LDA_tmp;
        maxmn = minmn;
        for (jp1j = 0; jp1j <= mmj_tmp - 2; jp1j++) {
          tol = b_A[minmn + jp1j * LDA_tmp];
          if (tol != 0.0) {
            i = maxmn + 2;
            rankA = mmj_tmp + maxmn;
            for (int b_i{i}; b_i <= rankA; b_i++) {
              b_A[b_i - 1] =
                  b_A[b_i - 1] + b_A[((b_tmp + b_i) - maxmn) - 1] * -tol;
            }
          }
          maxmn += LDA_tmp;
        }
      }
    }
    for (int b_i{0}; b_i <= n - 2; b_i++) {
      i = jpvt[b_i];
      if (i != b_i + 1) {
        tol = B[b_i];
        B[b_i] = B[i - 1];
        B[i - 1] = tol;
      }
    }
    for (int k{0}; k < n; k++) {
      minmn = LDA_tmp * k;
      if (B[k] != 0.0) {
        i = k + 2;
        for (int b_i{i}; b_i <= n; b_i++) {
          B[b_i - 1] = B[b_i - 1] - B[k] * b_A[(b_i + minmn) - 1];
        }
      }
    }
    for (int k{n}; k >= 1; k--) {
      minmn = LDA_tmp * (k - 1);
      tol = B[k - 1];
      if (tol != 0.0) {
        tol /= b_A[(k + minmn) - 1];
        B[k - 1] = tol;
        for (int b_i{0}; b_i <= k - 2; b_i++) {
          B[b_i] = B[b_i] - B[k - 1] * b_A[b_i + minmn];
        }
      }
    }
  } else {
    double tol;
    int i;
    int maxmn;
    int minmn;
    int rankA;
    int u0;
    b_A.set_size(A.size(0), A.size(1));
    minmn = A.size(0) * A.size(1);
    for (i = 0; i < minmn; i++) {
      b_A[i] = A[i];
    }
    internal::lapack::xgeqp3(b_A, tau, jpvt);
    rankA = 0;
    if (b_A.size(0) < b_A.size(1)) {
      minmn = b_A.size(0);
      maxmn = b_A.size(1);
    } else {
      minmn = b_A.size(1);
      maxmn = b_A.size(0);
    }
    if (minmn > 0) {
      tol = std::fmin(1.4901161193847656E-8,
                      2.2204460492503131E-15 * static_cast<double>(maxmn)) *
            std::abs(b_A[0]);
      while ((rankA < minmn) &&
             (!(std::abs(b_A[rankA + b_A.size(0) * rankA]) <= tol))) {
        rankA++;
      }
    }
    minmn = B.size(0);
    b_B.set_size(minmn);
    for (i = 0; i < minmn; i++) {
      b_B[i] = B[i];
    }
    minmn = b_A.size(1);
    B.set_size(b_A.size(1));
    for (i = 0; i < minmn; i++) {
      B[i] = 0.0;
    }
    u0 = b_A.size(0);
    maxmn = b_A.size(1);
    if (u0 <= maxmn) {
      maxmn = u0;
    }
    for (int j{0}; j < maxmn; j++) {
      minmn = b_A.size(0);
      if (tau[j] != 0.0) {
        tol = b_B[j];
        i = j + 2;
        for (int b_i{i}; b_i <= minmn; b_i++) {
          tol += b_A[(b_i + b_A.size(0) * j) - 1] * b_B[b_i - 1];
        }
        tol *= tau[j];
        if (tol != 0.0) {
          b_B[j] = b_B[j] - tol;
          for (int b_i{i}; b_i <= minmn; b_i++) {
            b_B[b_i - 1] =
                b_B[b_i - 1] - b_A[(b_i + b_A.size(0) * j) - 1] * tol;
          }
        }
      }
    }
    for (int b_i{0}; b_i < rankA; b_i++) {
      B[jpvt[b_i] - 1] = b_B[b_i];
    }
    for (int j{rankA}; j >= 1; j--) {
      i = jpvt[j - 1];
      B[i - 1] = B[i - 1] / b_A[(j + b_A.size(0) * (j - 1)) - 1];
      for (int b_i{0}; b_i <= j - 2; b_i++) {
        B[jpvt[b_i] - 1] =
            B[jpvt[b_i] - 1] - B[i - 1] * b_A[b_i + b_A.size(0) * (j - 1)];
      }
    }
  }
}

} // namespace coder

// End of code generation (mldivide.cpp)
