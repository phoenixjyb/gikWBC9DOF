//
// File: xnrm2.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef XNRM2_H
#define XNRM2_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
double xnrm2(int n, const array<double, 2U> &x, int ix0);

double xnrm2(int n, const double x[9], int ix0);

double xnrm2(const double x[3]);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xnrm2.h
//
// [EOF]
//
