//
// File: xaxpy.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef XAXPY_H
#define XAXPY_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
void xaxpy(double a, const double x[9], int ix0, double y[3]);

void xaxpy(double a, const double x[3], double y[9], int iy0);

void xaxpy(int n, double a, int ix0, double y[9], int iy0);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for xaxpy.h
//
// [EOF]
//
