//
// constraintDistanceBounds.h
//
// Code generation for function 'constraintDistanceBounds'
//

#ifndef CONSTRAINTDISTANCEBOUNDS_H
#define CONSTRAINTDISTANCEBOUNDS_H

// Include files
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class constraintDistanceBounds {
public:
  constraintDistanceBounds *init();
  constraintDistanceBounds();
  ~constraintDistanceBounds();
  char EndEffector[17];
  array<char, 2U> ReferenceBody;
  double Bounds[2];
  double Weights;

protected:
  cell_14 ConstructorPropertyDefaultValues;
};

} // namespace coder

#endif
// End of code generation (constraintDistanceBounds.h)
