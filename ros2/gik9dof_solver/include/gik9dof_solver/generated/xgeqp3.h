//
// xgeqp3.h
//
// Code generation for function 'xgeqp3'
//

#ifndef XGEQP3_H
#define XGEQP3_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(array<double, 2U> &A, array<double, 1U> &tau, array<int, 2U> &jpvt);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xgeqp3.h)
