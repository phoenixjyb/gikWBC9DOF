//
// File: constraintDistanceBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:09:07
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
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
  cell_55 ConstructorPropertyDefaultValues;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintDistanceBounds.h
//
// [EOF]
//
