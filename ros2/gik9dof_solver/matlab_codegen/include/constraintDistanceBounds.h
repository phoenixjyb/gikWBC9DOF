//
// File: constraintDistanceBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct cell_15 {
  double f2[2];
  double f3;
};

namespace coder {
class constraintDistanceBounds {
public:
  constraintDistanceBounds *init();
  char EndEffector[17];
  array<char, 2U> ReferenceBody;
  double Bounds[2];
  double Weights;

protected:
  cell_15 ConstructorPropertyDefaultValues;
};

} // namespace coder

#endif
//
// File trailer for constraintDistanceBounds.h
//
// [EOF]
//
