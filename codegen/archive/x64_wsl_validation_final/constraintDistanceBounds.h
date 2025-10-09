//
// File: constraintDistanceBounds.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "solveGIKStepWrapper_types1.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
class constraintDistanceBounds {
public:
  constraintDistanceBounds *init();
  constraintDistanceBounds();
  ~constraintDistanceBounds();
  cell_14 ConstructorPropertyDefaultValues;
  ::coder::array<char, 2U> EndEffector;
  ::coder::array<char, 2U> ReferenceBody;
  double Bounds[2];
  double Weights;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintDistanceBounds.h
//
// [EOF]
//
