//
// File: xnrm2.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

#ifndef XNRM2_H
#define XNRM2_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
namespace blas {
double xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0);

double xnrm2(int n, const double x[9], int ix0);

double xnrm2(const double x[3]);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for xnrm2.h
//
// [EOF]
//
