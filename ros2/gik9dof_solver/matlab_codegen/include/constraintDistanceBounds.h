//
// File: constraintDistanceBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include Files
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
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
  char EndEffector[17];
  ::coder::array<char, 2U> ReferenceBody;
  double Bounds[2];
  double Weights;

protected:
  cell_51 ConstructorPropertyDefaultValues;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintDistanceBounds.h
//
// [EOF]
//
