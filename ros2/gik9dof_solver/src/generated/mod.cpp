//
// mod.cpp
//
// Code generation for function 'mod'
//

// Include files
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double b_mod(double x)
{
  double r;
  if (std::isnan(x) || std::isinf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    bool rEQ0;
    r = std::fmod(x, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      double q;
      q = std::abs(x / 6.2831853071795862);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      r = 0.0;
    } else if (r < 0.0) {
      r += 6.2831853071795862;
    }
  }
  return r;
}

} // namespace coder

// End of code generation (mod.cpp)
