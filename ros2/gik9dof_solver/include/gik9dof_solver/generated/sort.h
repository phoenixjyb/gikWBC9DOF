//
// sort.h
//
// Code generation for function 'sort'
//

#ifndef SORT_H
#define SORT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_sort(double x_data[], const int x_size[2]);

void sort(double x[24], int idx[24]);

void sort(array<double, 1U> &x);

} // namespace internal
} // namespace coder

#endif
// End of code generation (sort.h)
