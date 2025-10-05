//
// sortIdx.h
//
// Code generation for function 'sortIdx'
//

#ifndef SORTIDX_H
#define SORTIDX_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_merge(int idx_data[], double x_data[], int offset, int np, int nq,
             int iwork_data[], double xwork_data[]);

void merge(int idx[24], double x[24], int offset, int np, int nq, int iwork[24],
           double xwork[24]);

void merge_block(array<int, 1U> &idx, array<double, 1U> &x, int offset, int n,
                 int preSortLevel, array<int, 1U> &iwork,
                 array<double, 1U> &xwork);

} // namespace internal
} // namespace coder

#endif
// End of code generation (sortIdx.h)
