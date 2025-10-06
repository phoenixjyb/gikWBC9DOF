//
// File: sort.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef SORT_H
#define SORT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
void b_sort(double x_data[], const int x_size[2]);

void sort(double x[22], int idx[22]);

void sort(::coder::array<double, 1U> &x);

} // namespace internal
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for sort.h
//
// [EOF]
//
