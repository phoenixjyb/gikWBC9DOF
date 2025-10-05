//
// xzlascl.h
//
// Code generation for function 'xzlascl'
//

#ifndef XZLASCL_H
#define XZLASCL_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void b_xzlascl(double cfrom, double cto, double A[3]);

void xzlascl(double cfrom, double cto, double A[9]);

} // namespace reflapack
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzlascl.h)
