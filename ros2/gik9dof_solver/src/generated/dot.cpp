//
// dot.cpp
//
// Code generation for function 'dot'
//

// Include files
#include "dot.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
double dot(const double a[4], const double b[4])
{
  return ((a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]) + a[3] * b[3];
}

} // namespace coder

// End of code generation (dot.cpp)
