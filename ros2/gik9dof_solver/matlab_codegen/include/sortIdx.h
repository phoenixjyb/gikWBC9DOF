//
// File: sortIdx.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef SORTIDX_H
#define SORTIDX_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
void b_merge(int idx_data[], double x_data[], int offset, int np, int nq,
             int iwork_data[], double xwork_data[]);

void merge(int idx[22], double x[22], int offset, int np, int nq, int iwork[22],
           double xwork[22]);

void merge_block(::coder::array<int, 1U> &idx, ::coder::array<double, 1U> &x,
                 int offset, int n, int preSortLevel,
                 ::coder::array<int, 1U> &iwork,
                 ::coder::array<double, 1U> &xwork);

} // namespace internal
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for sortIdx.h
//
// [EOF]
//
