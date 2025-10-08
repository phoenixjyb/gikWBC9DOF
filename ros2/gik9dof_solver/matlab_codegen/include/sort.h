//
// File: sort.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef SORT_H
#define SORT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_sort(double x_data[], const int x_size[2]);

void sort(double x[22], int idx[22]);

void sort(array<double, 1U> &x);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for sort.h
//
// [EOF]
//
