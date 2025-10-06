//
// File: sort.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "sort.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double x_data[]
//                const int x_size[2]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace internal {
void b_sort(double x_data[], const int x_size[2])
{
  int idx_data[36];
  int loop_ub;
  loop_ub = x_size[1];
  if (loop_ub - 1 >= 0) {
    std::memset(&idx_data[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(int));
  }
  if (x_size[1] != 0) {
    double xwork_data[36];
    double x4[4];
    int iwork_data[36];
    int i;
    int i1;
    int i2;
    int i3;
    int i4;
    int ib;
    int nNaNs;
    int quartetOffset;
    int wOffset_tmp;
    signed char idx4[4];
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    nNaNs = 0;
    ib = 0;
    for (int k{0}; k < loop_ub; k++) {
      iwork_data[k] = 0;
      if (std::isnan(x_data[k])) {
        i4 = (loop_ub - nNaNs) - 1;
        idx_data[i4] = k + 1;
        xwork_data[i4] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = static_cast<signed char>(k + 1);
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          double d;
          double d1;
          quartetOffset = k - nNaNs;
          if (x4[0] <= x4[1]) {
            ib = 1;
            i2 = 2;
          } else {
            ib = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[ib - 1];
          d1 = x4[i3 - 1];
          if (d <= d1) {
            d = x4[i2 - 1];
            if (d <= d1) {
              i = ib;
              i1 = i2;
              ib = i3;
              i2 = i4;
            } else if (d <= x4[i4 - 1]) {
              i = ib;
              i1 = i3;
              ib = i2;
              i2 = i4;
            } else {
              i = ib;
              i1 = i3;
              ib = i4;
            }
          } else {
            d1 = x4[i4 - 1];
            if (d <= d1) {
              if (x4[i2 - 1] <= d1) {
                i = i3;
                i1 = ib;
                ib = i2;
                i2 = i4;
              } else {
                i = i3;
                i1 = ib;
                ib = i4;
              }
            } else {
              i = i3;
              i1 = i4;
            }
          }
          idx_data[quartetOffset - 3] = idx4[i - 1];
          idx_data[quartetOffset - 2] = idx4[i1 - 1];
          idx_data[quartetOffset - 1] = idx4[ib - 1];
          idx_data[quartetOffset] = idx4[i2 - 1];
          x_data[quartetOffset - 3] = x4[i - 1];
          x_data[quartetOffset - 2] = x4[i1 - 1];
          x_data[quartetOffset - 1] = x4[ib - 1];
          x_data[quartetOffset] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    wOffset_tmp = x_size[1] - nNaNs;
    if (ib > 0) {
      signed char perm[4];
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      i = static_cast<unsigned char>(ib);
      for (int k{0}; k < i; k++) {
        i4 = (wOffset_tmp - ib) + k;
        i1 = perm[k];
        idx_data[i4] = idx4[i1 - 1];
        x_data[i4] = x4[i1 - 1];
      }
    }
    i2 = nNaNs >> 1;
    for (int k{0}; k < i2; k++) {
      quartetOffset = wOffset_tmp + k;
      i3 = idx_data[quartetOffset];
      i4 = (loop_ub - k) - 1;
      idx_data[quartetOffset] = idx_data[i4];
      idx_data[i4] = i3;
      x_data[quartetOffset] = xwork_data[i4];
      x_data[i4] = xwork_data[quartetOffset];
    }
    if ((static_cast<unsigned int>(nNaNs) & 1U) != 0U) {
      i = wOffset_tmp + i2;
      x_data[i] = xwork_data[i];
    }
    if (wOffset_tmp > 1) {
      i3 = wOffset_tmp >> 2;
      quartetOffset = 4;
      while (i3 > 1) {
        if ((static_cast<unsigned int>(i3) & 1U) != 0U) {
          i3--;
          ib = quartetOffset * i3;
          i2 = wOffset_tmp - ib;
          if (i2 > quartetOffset) {
            b_merge(idx_data, x_data, ib, quartetOffset, i2 - quartetOffset,
                    iwork_data, xwork_data);
          }
        }
        ib = quartetOffset << 1;
        i3 >>= 1;
        for (int k{0}; k < i3; k++) {
          b_merge(idx_data, x_data, k * ib, quartetOffset, quartetOffset,
                  iwork_data, xwork_data);
        }
        quartetOffset = ib;
      }
      if (wOffset_tmp > quartetOffset) {
        b_merge(idx_data, x_data, 0, quartetOffset, wOffset_tmp - quartetOffset,
                iwork_data, xwork_data);
      }
    }
  }
}

//
// Arguments    : double x[22]
//                int idx[22]
// Return Type  : void
//
void sort(double x[22], int idx[22])
{
  double xwork[22];
  double x4[4];
  int iwork[22];
  int b_i1;
  int i;
  int i1;
  int i3;
  int ib;
  int nNaNs;
  int quartetOffset;
  signed char idx4[4];
  std::memset(&idx[0], 0, 22U * sizeof(int));
  x4[0] = 0.0;
  idx4[0] = 0;
  x4[1] = 0.0;
  idx4[1] = 0;
  x4[2] = 0.0;
  idx4[2] = 0;
  x4[3] = 0.0;
  idx4[3] = 0;
  nNaNs = 0;
  ib = 0;
  for (int k{0}; k < 22; k++) {
    if (std::isnan(x[k])) {
      idx[21 - nNaNs] = k + 1;
      xwork[21 - nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = static_cast<signed char>(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        double d;
        double d1;
        int i4;
        quartetOffset = k - nNaNs;
        if (x4[0] <= x4[1]) {
          i1 = 1;
          ib = 2;
        } else {
          i1 = 2;
          ib = 1;
        }
        if (x4[2] <= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }
        d = x4[i1 - 1];
        d1 = x4[i3 - 1];
        if (d <= d1) {
          d = x4[ib - 1];
          if (d <= d1) {
            i = i1;
            b_i1 = ib;
            i1 = i3;
            ib = i4;
          } else if (d <= x4[i4 - 1]) {
            i = i1;
            b_i1 = i3;
            i1 = ib;
            ib = i4;
          } else {
            i = i1;
            b_i1 = i3;
            i1 = i4;
          }
        } else {
          d1 = x4[i4 - 1];
          if (d <= d1) {
            if (x4[ib - 1] <= d1) {
              i = i3;
              b_i1 = i1;
              i1 = ib;
              ib = i4;
            } else {
              i = i3;
              b_i1 = i1;
              i1 = i4;
            }
          } else {
            i = i3;
            b_i1 = i4;
          }
        }
        idx[quartetOffset - 3] = idx4[i - 1];
        idx[quartetOffset - 2] = idx4[b_i1 - 1];
        idx[quartetOffset - 1] = idx4[i1 - 1];
        idx[quartetOffset] = idx4[ib - 1];
        x[quartetOffset - 3] = x4[i - 1];
        x[quartetOffset - 2] = x4[b_i1 - 1];
        x[quartetOffset - 1] = x4[i1 - 1];
        x[quartetOffset] = x4[ib - 1];
        ib = 0;
      }
    }
  }
  if (ib > 0) {
    signed char perm[4];
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }
    i = static_cast<unsigned char>(ib);
    for (int k{0}; k < i; k++) {
      quartetOffset = ((k - nNaNs) - ib) + 22;
      b_i1 = perm[k];
      idx[quartetOffset] = idx4[b_i1 - 1];
      x[quartetOffset] = x4[b_i1 - 1];
    }
  }
  ib = (nNaNs >> 1) + 22;
  for (int k{0}; k <= ib - 23; k++) {
    i1 = (k - nNaNs) + 22;
    quartetOffset = idx[i1];
    idx[i1] = idx[21 - k];
    idx[21 - k] = quartetOffset;
    x[i1] = xwork[21 - k];
    x[21 - k] = xwork[i1];
  }
  if ((static_cast<unsigned int>(nNaNs) & 1U) != 0U) {
    i = ib - nNaNs;
    x[i] = xwork[i];
  }
  if (22 - nNaNs > 1) {
    std::memset(&iwork[0], 0, 22U * sizeof(int));
    i3 = (22 - nNaNs) >> 2;
    quartetOffset = 4;
    while (i3 > 1) {
      if ((static_cast<unsigned int>(i3) & 1U) != 0U) {
        i3--;
        ib = quartetOffset * i3;
        i1 = 22 - (nNaNs + ib);
        if (i1 > quartetOffset) {
          merge(idx, x, ib, quartetOffset, i1 - quartetOffset, iwork, xwork);
        }
      }
      ib = quartetOffset << 1;
      i3 >>= 1;
      for (int k{0}; k < i3; k++) {
        merge(idx, x, k * ib, quartetOffset, quartetOffset, iwork, xwork);
      }
      quartetOffset = ib;
    }
    if (22 - nNaNs > quartetOffset) {
      merge(idx, x, 0, quartetOffset, 22 - (nNaNs + quartetOffset), iwork,
            xwork);
    }
  }
}

//
// Arguments    : ::coder::array<double, 1U> &x
// Return Type  : void
//
void sort(::coder::array<double, 1U> &x)
{
  ::coder::array<double, 1U> vwork;
  ::coder::array<double, 1U> xwork;
  ::coder::array<int, 1U> iidx;
  ::coder::array<int, 1U> iwork;
  int dim;
  int i;
  int i1;
  int vstride;
  dim = 2;
  if (x.size(0) != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    i = x.size(0);
  } else {
    i = 1;
  }
  vwork.set_size(i);
  vstride = 1;
  i1 = dim - 2;
  for (int k{0}; k <= i1; k++) {
    vstride *= x.size(0);
  }
  for (int b_i{0}; b_i < 1; b_i++) {
    for (int j{0}; j < vstride; j++) {
      double x4[4];
      int idx4[4];
      int bLen;
      int bLen2;
      int b_i1;
      int i2;
      int i3;
      int i4;
      int iidx_tmp;
      int nBlocks;
      int wOffset_tmp;
      for (int k{0}; k < i; k++) {
        vwork[k] = x[j + k * vstride];
      }
      bLen2 = vwork.size(0);
      iidx.set_size(vwork.size(0));
      iwork.set_size(vwork.size(0));
      for (i1 = 0; i1 < bLen2; i1++) {
        iidx[i1] = 0;
        iwork[i1] = 0;
      }
      xwork.set_size(vwork.size(0));
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      nBlocks = 0;
      dim = 0;
      for (int k{0}; k < bLen2; k++) {
        if (std::isnan(vwork[k])) {
          iidx_tmp = (bLen2 - nBlocks) - 1;
          iidx[iidx_tmp] = k + 1;
          xwork[iidx_tmp] = vwork[k];
          nBlocks++;
        } else {
          dim++;
          idx4[dim - 1] = k + 1;
          x4[dim - 1] = vwork[k];
          if (dim == 4) {
            double d;
            double d1;
            dim = k - nBlocks;
            if (x4[0] <= x4[1]) {
              b_i1 = 1;
              i2 = 2;
            } else {
              b_i1 = 2;
              i2 = 1;
            }
            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }
            d = x4[b_i1 - 1];
            d1 = x4[i3 - 1];
            if (d <= d1) {
              d = x4[i2 - 1];
              if (d <= d1) {
                i1 = b_i1;
                bLen = i2;
                b_i1 = i3;
                i2 = i4;
              } else if (d <= x4[i4 - 1]) {
                i1 = b_i1;
                bLen = i3;
                b_i1 = i2;
                i2 = i4;
              } else {
                i1 = b_i1;
                bLen = i3;
                b_i1 = i4;
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[i2 - 1] <= d1) {
                  i1 = i3;
                  bLen = b_i1;
                  b_i1 = i2;
                  i2 = i4;
                } else {
                  i1 = i3;
                  bLen = b_i1;
                  b_i1 = i4;
                }
              } else {
                i1 = i3;
                bLen = i4;
              }
            }
            iidx[dim - 3] = idx4[i1 - 1];
            iidx[dim - 2] = idx4[bLen - 1];
            iidx[dim - 1] = idx4[b_i1 - 1];
            iidx[dim] = idx4[i2 - 1];
            vwork[dim - 3] = x4[i1 - 1];
            vwork[dim - 2] = x4[bLen - 1];
            vwork[dim - 1] = x4[b_i1 - 1];
            vwork[dim] = x4[i2 - 1];
            dim = 0;
          }
        }
      }
      wOffset_tmp = vwork.size(0) - nBlocks;
      if (dim > 0) {
        signed char perm[4];
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (dim == 1) {
          perm[0] = 1;
        } else if (dim == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        i1 = static_cast<unsigned char>(dim);
        for (int k{0}; k < i1; k++) {
          iidx_tmp = (wOffset_tmp - dim) + k;
          bLen = perm[k];
          iidx[iidx_tmp] = idx4[bLen - 1];
          vwork[iidx_tmp] = x4[bLen - 1];
        }
      }
      dim = nBlocks >> 1;
      for (int k{0}; k < dim; k++) {
        b_i1 = wOffset_tmp + k;
        i2 = iidx[b_i1];
        iidx_tmp = (bLen2 - k) - 1;
        iidx[b_i1] = iidx[iidx_tmp];
        iidx[iidx_tmp] = i2;
        vwork[b_i1] = xwork[iidx_tmp];
        vwork[iidx_tmp] = xwork[b_i1];
      }
      if ((static_cast<unsigned int>(nBlocks) & 1U) != 0U) {
        dim += wOffset_tmp;
        vwork[dim] = xwork[dim];
      }
      dim = 2;
      if (wOffset_tmp > 1) {
        if (vwork.size(0) >= 256) {
          nBlocks = wOffset_tmp >> 8;
          if (nBlocks > 0) {
            for (int b{0}; b < nBlocks; b++) {
              double b_xwork[256];
              int b_iwork[256];
              i4 = (b << 8) - 1;
              for (int b_b{0}; b_b < 6; b_b++) {
                bLen = 1 << (b_b + 2);
                bLen2 = bLen << 1;
                i1 = 256 >> (b_b + 3);
                for (int k{0}; k < i1; k++) {
                  i2 = (i4 + k * bLen2) + 1;
                  for (b_i1 = 0; b_i1 < bLen2; b_i1++) {
                    dim = i2 + b_i1;
                    b_iwork[b_i1] = iidx[dim];
                    b_xwork[b_i1] = vwork[dim];
                  }
                  i3 = 0;
                  b_i1 = bLen;
                  dim = i2 - 1;
                  int exitg1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (b_xwork[i3] <= b_xwork[b_i1]) {
                      iidx[dim] = b_iwork[i3];
                      vwork[dim] = b_xwork[i3];
                      if (i3 + 1 < bLen) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx[dim] = b_iwork[b_i1];
                      vwork[dim] = b_xwork[b_i1];
                      if (b_i1 + 1 < bLen2) {
                        b_i1++;
                      } else {
                        dim -= i3;
                        for (b_i1 = i3 + 1; b_i1 <= bLen; b_i1++) {
                          iidx_tmp = dim + b_i1;
                          iidx[iidx_tmp] = b_iwork[b_i1 - 1];
                          vwork[iidx_tmp] = b_xwork[b_i1 - 1];
                        }
                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }
            dim = nBlocks << 8;
            b_i1 = wOffset_tmp - dim;
            if (b_i1 > 0) {
              merge_block(iidx, vwork, dim, b_i1, 2, iwork, xwork);
            }
            dim = 8;
          }
        }
        merge_block(iidx, vwork, 0, wOffset_tmp, dim, iwork, xwork);
      }
      for (int k{0}; k < i; k++) {
        x[j + k * vstride] = vwork[k];
      }
    }
  }
}

} // namespace internal
} // namespace coder
} // namespace gik9dof

//
// File trailer for sort.cpp
//
// [EOF]
//
