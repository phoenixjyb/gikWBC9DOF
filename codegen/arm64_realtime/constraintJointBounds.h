//
// File: constraintJointBounds.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 13:57:39
//

#ifndef CONSTRAINTJOINTBOUNDS_H
#define CONSTRAINTJOINTBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "solveGIKStepWrapper_types1.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
class constraintJointBounds {
public:
  constraintJointBounds();
  ~constraintJointBounds();
  cell_12 ConstructorPropertyDefaultValues;
  double NumElements;
  ::coder::array<double, 2U> BoundsInternal;
  ::coder::array<double, 2U> WeightsInternal;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintJointBounds.h
//
// [EOF]
//
