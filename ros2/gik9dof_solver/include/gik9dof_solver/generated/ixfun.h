//
// ixfun.h
//
// Code generation for function 'ixfun'
//

#ifndef IXFUN_H
#define IXFUN_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void expand_max(const array<double, 1U> &a, const array<double, 1U> &b,
                array<double, 1U> &c);

void expand_min(const array<double, 1U> &a, const array<double, 1U> &b,
                array<double, 1U> &c);

} // namespace internal
} // namespace coder

#endif
// End of code generation (ixfun.h)
