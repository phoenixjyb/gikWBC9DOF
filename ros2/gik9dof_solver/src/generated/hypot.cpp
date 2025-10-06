//
// hypot.cpp
//
// Code generation for function 'hypot'
//

// Include files
#include "hypot.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double b_hypot(double x, double y)
{
  double b;
  double r;
  r = std::abs(x);
  b = std::abs(y);
  if (r < b) {
    r /= b;
    r = b * std::sqrt(r * r + 1.0);
  } else if (r > b) {
    b /= r;
    r *= std::sqrt(b * b + 1.0);
  } else if (std::isnan(b)) {
    r = rtNaN;
  } else {
    r *= 1.4142135623730951;
  }
  return r;
}

} // namespace coder

// End of code generation (hypot.cpp)
