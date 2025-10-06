//
// File: sortIdx.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "sortIdx.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
static void merge(::coder::array<int, 1U> &idx, ::coder::array<double, 1U> &x,
                  int offset, int np, int nq, ::coder::array<int, 1U> &iwork,
                  ::coder::array<double, 1U> &xwork);

}
} // namespace coder
} // namespace gik9dof

// Function Definitions
//
// Arguments    : ::coder::array<int, 1U> &idx
//                ::coder::array<double, 1U> &x
//                int offset
//                int np
//                int nq
//                ::coder::array<int, 1U> &iwork
//                ::coder::array<double, 1U> &xwork
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
static void merge(::coder::array<int, 1U> &idx, ::coder::array<double, 1U> &x,
                  int offset, int np, int nq, ::coder::array<int, 1U> &iwork,
                  ::coder::array<double, 1U> &xwork)
{
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (int j{0}; j < n_tmp; j++) {
      iout = offset + j;
      iwork[j] = idx[iout];
      xwork[j] = x[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[q]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (int j{p + 1}; j <= np; j++) {
            iout = q + j;
            idx[iout] = iwork[j - 1];
            x[iout] = xwork[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : int idx_data[]
//                double x_data[]
//                int offset
//                int np
//                int nq
//                int iwork_data[]
//                double xwork_data[]
// Return Type  : void
//
void b_merge(int idx_data[], double x_data[], int offset, int np, int nq,
             int iwork_data[], double xwork_data[])
{
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (int j{0}; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (int j{p + 1}; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : int idx[22]
//                double x[22]
//                int offset
//                int np
//                int nq
//                int iwork[22]
//                double xwork[22]
// Return Type  : void
//
void merge(int idx[22], double x[22], int offset, int np, int nq, int iwork[22],
           double xwork[22])
{
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (int j{0}; j < n_tmp; j++) {
      iout = offset + j;
      iwork[j] = idx[iout];
      xwork[j] = x[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[q]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (int j{p + 1}; j <= np; j++) {
            iout = q + j;
            idx[iout] = iwork[j - 1];
            x[iout] = xwork[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : ::coder::array<int, 1U> &idx
//                ::coder::array<double, 1U> &x
//                int offset
//                int n
//                int preSortLevel
//                ::coder::array<int, 1U> &iwork
//                ::coder::array<double, 1U> &xwork
// Return Type  : void
//
void merge_block(::coder::array<int, 1U> &idx, ::coder::array<double, 1U> &x,
                 int offset, int n, int preSortLevel,
                 ::coder::array<int, 1U> &iwork,
                 ::coder::array<double, 1U> &xwork)
{
  int bLen;
  int nPairs;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    int nTail;
    int tailOffset;
    if ((static_cast<unsigned int>(nPairs) & 1U) != 0U) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for sortIdx.cpp
//
// [EOF]
//
