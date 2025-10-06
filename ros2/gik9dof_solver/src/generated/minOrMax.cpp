//
// minOrMax.cpp
//
// Code generation for function 'minOrMax'
//

// Include files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
int minimum2(double y)
{
  double d;
  int ex;
  d = std::round(y);
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      ex = static_cast<int>(d);
    } else {
      ex = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    ex = MAX_int32_T;
  } else {
    ex = 0;
  }
  if (std::isnan(y) || (!(y < 256.0))) {
    ex = 256;
  }
  return ex;
}

int minimum2(double x, int y)
{
  double d;
  int ex;
  d = std::round(x);
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      ex = static_cast<int>(d);
    } else {
      ex = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    ex = MAX_int32_T;
  } else {
    ex = 0;
  }
  if (std::isnan(x) || (x > y)) {
    ex = y;
  }
  return ex;
}

} // namespace internal
} // namespace coder

// End of code generation (minOrMax.cpp)
