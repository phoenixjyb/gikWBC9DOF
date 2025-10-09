//
// File: constraintDistanceBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h"
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
  constraintDistanceBounds *b_init();
  constraintDistanceBounds *c_init();
  constraintDistanceBounds();
  ~constraintDistanceBounds();
  ::coder::array<char, 2U> EndEffector;
  ::coder::array<char, 2U> ReferenceBody;
  double Bounds[2];
  double Weights;

protected:
  cell_14 ConstructorPropertyDefaultValues;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintDistanceBounds.h
//
// [EOF]
//
